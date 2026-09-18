#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OpenCodeReview 本地可视化控制中心后端服务。

提供:
- 项目切换、Finder 弹窗选取与多工程历史管理
- 项目 Git 状态与分支查询
- 一键触发 review / scan 任务并实时捕获日志
- 历史会话读取与行级缺陷展示
- 模型与 API Key 配置
"""

import http.server
import json
import os
import re
import socketserver
import subprocess
import sys
import threading
import time
import urllib.parse

PORT = 5488
DEFAULT_REPO = "/Users/Admin1/Desktop/正点原子rk3588"
GUI_CONFIG_FILE = os.path.expanduser("~/.opencodereview/gui_config.json")

current_task = {
    "running": False,
    "mode": None,
    "repo": None,
    "started_at": None,
    "completed_at": None,
    "logs": [],
    "session_id": None,
    "exit_code": None,
    "error": None,
    "summary": None,
}
task_lock = threading.Lock()


def load_gui_config():
    default = {
        "current_repo": DEFAULT_REPO,
        "recent_repos": [DEFAULT_REPO]
    }
    if not os.path.exists(GUI_CONFIG_FILE):
        return default
    try:
        with open(GUI_CONFIG_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
            if not isinstance(data, dict):
                return default
            if "current_repo" not in data or not os.path.exists(data["current_repo"]):
                data["current_repo"] = DEFAULT_REPO
            if "recent_repos" not in data or not isinstance(data["recent_repos"], list):
                data["recent_repos"] = [data["current_repo"]]
            return data
    except Exception:
        return default


def save_gui_config(current_repo):
    current_repo = os.path.abspath(os.path.expanduser(current_repo))
    config = load_gui_config()
    config["current_repo"] = current_repo
    recents = config.get("recent_repos", [])
    if current_repo in recents:
        recents.remove(current_repo)
    recents.insert(0, current_repo)
    config["recent_repos"] = recents[:10]
    try:
        os.makedirs(os.path.dirname(GUI_CONFIG_FILE), exist_ok=True)
        with open(GUI_CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(config, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print(f"Failed to save gui_config: {e}")
    return config


def get_current_repo():
    return load_gui_config().get("current_repo", DEFAULT_REPO)


def choose_folder_dialog():
    script = '''tell application "System Events"
        activate
        set chosenFolder to choose folder with prompt "请选择要审查的代码工程目录:"
        return POSIX path of chosenFolder
    end tell'''
    try:
        res = subprocess.run(["osascript", "-e", script], capture_output=True, text=True, timeout=120)
        if res.returncode == 0:
            path = res.stdout.strip().rstrip('/')
            if os.path.isdir(path):
                return {"status": "ok", "path": path}
            return {"status": "error", "message": "所选路径不是有效目录"}
        else:
            return {"status": "cancelled", "message": "已取消选取"}
    except subprocess.TimeoutExpired:
        return {"status": "timeout", "message": "选取操作超时"}
    except Exception as e:
        return {"status": "error", "message": str(e)}


def get_ocr_path():
    p = subprocess.run(["which", "ocr"], capture_output=True, text=True)
    if p.returncode == 0:
        return p.stdout.strip()
    cand = "/Users/Admin1/.nvm/versions/node/v24.16.0/bin/ocr"
    if os.path.exists(cand):
        return cand
    return "ocr"


def get_git_info(repo_dir):
    if not os.path.exists(os.path.join(repo_dir, ".git")):
        return {"is_git": False}
    branch_p = subprocess.run(["git", "rev-parse", "--abbrev-ref", "HEAD"],
                              cwd=repo_dir, capture_output=True, text=True)
    branch = branch_p.stdout.strip() if branch_p.returncode == 0 else "unknown"

    status_p = subprocess.run(["git", "status", "-s"],
                              cwd=repo_dir, capture_output=True, text=True)
    changed = [l.strip() for l in status_p.stdout.splitlines() if l.strip()]

    branches_p = subprocess.run(["git", "branch", "--format=%(refname:short)"],
                                cwd=repo_dir, capture_output=True, text=True)
    branches = [b.strip() for b in branches_p.stdout.splitlines() if b.strip()]

    return {
        "is_git": True,
        "branch": branch,
        "changed_files": changed,
        "changed_count": len(changed),
        "branches": branches,
    }


def get_ocr_config():
    config_file = os.path.expanduser("~/.opencodereview/config.json")
    if not os.path.exists(config_file):
        return {"provider": "deepseek", "model": "deepseek-chat"}
    try:
        with open(config_file, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        provider = cfg.get("provider", "deepseek")
        p_cfg = cfg.get("providers", {}).get(provider, {})
        key = p_cfg.get("api_key", "")
        masked_key = (key[:4] + "****" + key[-4:]) if len(key) >= 8 else ("****" if key else "")
        return {
            "provider": provider,
            "model": p_cfg.get("model", cfg.get("llm", {}).get("model", "deepseek-chat")),
            "api_key_masked": masked_key,
            "has_key": bool(key),
        }
    except Exception as e:
        return {"error": str(e)}


def list_sessions(repo_dir):
    ocr = get_ocr_path()
    res = subprocess.run([ocr, "session", "list", "--repo", repo_dir],
                         cwd=repo_dir, capture_output=True, text=True)
    lines = [l.strip() for l in res.stdout.splitlines() if l.strip()]
    if len(lines) <= 1:
        return []
    sessions = []
    for line in lines[1:]:
        parts = re.split(r'\s{2,}', line)
        if len(parts) >= 6:
            sessions.append({
                "session_id": parts[0],
                "mode": parts[1],
                "range": parts[2],
                "files": parts[3],
                "comments": parts[4],
                "status": parts[5],
                "started": parts[6] if len(parts) > 6 else "",
            })
    return sessions


def get_session_detail(session_id, repo_dir):
    ocr = get_ocr_path()
    show_p = subprocess.run([ocr, "session", "show", "--json", "--repo", repo_dir, session_id],
                            cwd=repo_dir, capture_output=True, text=True)
    metadata = {}
    if show_p.returncode == 0:
        try:
            metadata = json.loads(show_p.stdout)
        except Exception:
            pass

    comm_p = subprocess.run([ocr, "session", "comments", "--json", "--repo", repo_dir, session_id],
                            cwd=repo_dir, capture_output=True, text=True)
    comments = []
    if comm_p.returncode == 0:
        try:
            comments = json.loads(comm_p.stdout)
        except Exception:
            pass

    return {
        "metadata": metadata,
        "comments": comments,
    }


def run_review_worker(cmd, repo_dir, mode_name="review"):
    global current_task
    with task_lock:
        current_task["running"] = True
        current_task["mode"] = mode_name
        current_task["repo"] = repo_dir
        current_task["started_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        current_task["completed_at"] = None
        current_task["logs"] = [
            f"[工程目录] {repo_dir}",
            f"[执行指令] {' '.join(cmd)}"
        ]
        current_task["session_id"] = None
        current_task["exit_code"] = None
        current_task["error"] = None
        current_task["summary"] = None

    proc = subprocess.Popen(
        cmd,
        cwd=repo_dir,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        universal_newlines=True
    )

    session_id = None
    summary_text = ""

    for line in iter(proc.stdout.readline, ''):
        line = line.strip()
        if not line:
            continue
        with task_lock:
            current_task["logs"].append(line)
        m = re.search(r'Session:\s+([a-f0-9\-]+)', line)
        if m:
            session_id = m.group(1)
        if "Summary:" in line or "Review complete:" in line:
            summary_text += line + "\n"

    proc.wait()
    proc.stdout.close()

    with task_lock:
        current_task["running"] = False
        current_task["completed_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        current_task["exit_code"] = proc.returncode
        current_task["session_id"] = session_id
        current_task["summary"] = summary_text.strip()
        current_task["logs"].append(f"[完成] 退出状态码: {proc.returncode}")


class OCRHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=os.path.dirname(os.path.abspath(__file__)), **kwargs)

    def do_GET(self):
        url = urllib.parse.urlparse(self.path)
        qs = urllib.parse.parse_qs(url.query)

        if url.path == "/" or url.path == "/index.html":
            return super().do_GET()

        if url.path == "/api/projects":
            cfg = load_gui_config()
            self._send_json({
                "current_repo": cfg.get("current_repo", DEFAULT_REPO),
                "recent_repos": cfg.get("recent_repos", [DEFAULT_REPO]),
            })
            return

        if url.path == "/api/status":
            cfg = load_gui_config()
            curr = cfg.get("current_repo", DEFAULT_REPO)
            repo = qs.get("repo", [curr])[0]
            if not os.path.exists(repo):
                repo = DEFAULT_REPO
            git_info = get_git_info(repo)
            config = get_ocr_config()
            with task_lock:
                task_copy = dict(current_task)
            data = {
                "repo": repo,
                "recent_repos": cfg.get("recent_repos", [repo]),
                "git": git_info,
                "config": config,
                "task": task_copy,
            }
            self._send_json(data)
            return

        if url.path == "/api/sessions":
            curr = get_current_repo()
            repo = qs.get("repo", [curr])[0]
            sessions = list_sessions(repo)
            self._send_json({"sessions": sessions, "repo": repo})
            return

        if url.path == "/api/session_detail":
            curr = get_current_repo()
            session_id = qs.get("id", [""])[0]
            repo = qs.get("repo", [curr])[0]
            if not session_id:
                self._send_json({"error": "missing id"}, 400)
                return
            detail = get_session_detail(session_id, repo)
            self._send_json(detail)
            return

        if url.path == "/api/logs":
            with task_lock:
                data = {
                    "running": current_task["running"],
                    "session_id": current_task["session_id"],
                    "logs": current_task["logs"],
                    "exit_code": current_task["exit_code"],
                    "summary": current_task["summary"],
                }
            self._send_json(data)
            return

        return super().do_GET()

    def do_POST(self):
        url = urllib.parse.urlparse(self.path)
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length).decode('utf-8') if content_length > 0 else "{}"
        try:
            payload = json.loads(body)
        except Exception:
            payload = {}

        if url.path == "/api/choose_folder":
            result = choose_folder_dialog()
            if result.get("status") == "ok":
                chosen = result["path"]
                cfg = save_gui_config(chosen)
                git_info = get_git_info(chosen)
                self._send_json({
                    "status": "ok",
                    "repo": chosen,
                    "git": git_info,
                    "recent_repos": cfg.get("recent_repos", [])
                })
            else:
                self._send_json(result)
            return

        if url.path == "/api/set_repo":
            repo_in = payload.get("repo", "").strip()
            if not repo_in:
                self._send_json({"error": "工程路径不能为空"}, 400)
                return
            repo_path = os.path.abspath(os.path.expanduser(repo_in))
            if not os.path.isdir(repo_path):
                self._send_json({"error": f"指定路径不是有效目录: {repo_in}"}, 400)
                return
            cfg = save_gui_config(repo_path)
            git_info = get_git_info(repo_path)
            self._send_json({
                "status": "ok",
                "repo": repo_path,
                "git": git_info,
                "recent_repos": cfg.get("recent_repos", [])
            })
            return

        if url.path == "/api/run":
            repo = payload.get("repo") or get_current_repo()
            if not os.path.isdir(repo):
                self._send_json({"error": f"工程目录不存在: {repo}"}, 400)
                return

            mode = payload.get("mode", "workspace")
            ocr = get_ocr_path()
            cmd = [ocr]

            if mode == "workspace":
                cmd += ["review"]
            elif mode == "branch":
                from_b = payload.get("from_branch", "main")
                to_b = payload.get("to_branch", "HEAD")
                cmd += ["review", "--from", from_b, "--to", to_b]
            elif mode == "commit":
                commit = payload.get("commit", "HEAD")
                cmd += ["review", "-c", commit]
            elif mode == "scan":
                path = payload.get("path", "")
                cmd += ["scan"]
                if path:
                    cmd += ["--path", path]
            else:
                self._send_json({"error": f"unknown mode: {mode}"}, 400)
                return

            with task_lock:
                if current_task["running"]:
                    self._send_json({"error": "审查任务正在执行中，请等待完成"}, 409)
                    return

            thread = threading.Thread(target=run_review_worker, args=(cmd, repo, mode), daemon=True)
            thread.start()
            self._send_json({"status": "started", "cmd": cmd, "repo": repo})
            return

        if url.path == "/api/config":
            ocr = get_ocr_path()
            provider = payload.get("provider")
            model = payload.get("model")
            api_key = payload.get("api_key")

            results = []
            if provider:
                r = subprocess.run([ocr, "config", "set", "provider", provider], capture_output=True, text=True)
                results.append(r.stdout.strip())
            if api_key:
                p_key = f"providers.{provider or 'deepseek'}.api_key"
                r = subprocess.run([ocr, "config", "set", p_key, api_key], capture_output=True, text=True)
                results.append(r.stdout.strip())
            if model:
                p_mod = f"providers.{provider or 'deepseek'}.model"
                r = subprocess.run([ocr, "config", "set", p_mod, model], capture_output=True, text=True)
                results.append(r.stdout.strip())

            self._send_json({"status": "updated", "output": results})
            return

        self._send_json({"error": "not found"}, 404)

    def _send_json(self, data, status=200):
        body = json.dumps(data, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)


def main():
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("127.0.0.1", PORT), OCRHandler) as httpd:
        print(f"OpenCodeReview GUI Server running at http://127.0.0.1:{PORT}")
        sys.stdout.flush()
        httpd.serve_forever()


if __name__ == '__main__':
    main()
