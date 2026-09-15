from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import urllib.request
ROOT=Path(__file__).parent
class Handler(BaseHTTPRequestHandler):
 def do_GET(self):
  if self.path=='/api/stream':
   self.send_response(200);self.send_header('Content-Type','text/event-stream');self.send_header('Cache-Control','no-cache');self.end_headers()
   try:
    with urllib.request.urlopen('http://192.168.0.170:8088/api/stream',timeout=10) as remote:
     for line in remote:
      self.wfile.write(line)
      if line==b'\n':self.wfile.flush()
   except (OSError,TimeoutError):pass
  else:
   name='BASELINE.html' if self.path=='/baseline' else 'MODIFIED_FILE.html'
   self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write((ROOT/name).read_bytes())
 def log_message(self,*a):pass
ThreadingHTTPServer(('127.0.0.1',8090),Handler).serve_forever()
