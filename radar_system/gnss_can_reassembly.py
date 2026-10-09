"""CAN 串口分片的连续性检查及完整 GNSS 报文交付；纯计算，无硬件 I/O。

8 位序号的前向半窗口可计序号缺口；后向跳变可能是迟到、乱序或未通知的发送端重启，
不能按模 256 加成 255 个丢帧。后向已接收且内容相同的分片按重复丢弃，其余保守丢弃并断包。
保持现有线协议：每片 1 字节序号 + 1..7 字节原始内容。只向下游交付通过校验的完整消息。
"""
from collections import deque
import re

from um982 import _next_sync, binary_frame, crc32


class CanGnssReassembler:
    MAX_PENDING = 4096
    HISTORY = 128  # 与 8 位序号的后向半窗口一致，迟到重复片不会清掉当前有效半包。
    ASCII_START = re.compile(rb'\$(?:[A-Z]{2}[A-Z0-9]{3},|INS\.NAVI,)|#(?:BESTPOSA|BESTNAVHA|HEADINGA|BESTVELA),')

    def __init__(self, quiet_s=2.):
        self.quiet_s=quiet_s
        self.frames=self.gaps=self.gap_events=self.duplicates=self.out_of_order=0
        self.resyncs=self.discarded_partials=self.crc_errors=self.bad_headers=0
        self.bytes_in=self.messages=self.verified_bytes=0
        self._expect=None
        self._recent=deque(maxlen=self.HISTORY)
        self.buf=b''
        self.last_rx=self.last_message_rx=None

    def _break(self):
        self.resyncs+=1
        self.discarded_partials+=bool(self.buf)
        self.buf=b''

    def reset(self):
        """重连不保留半包或旧序号；累计诊断保留。"""
        if self._expect is not None or self.buf:self._break()
        self._expect=None
        self._recent.clear()
        self.last_rx=None

    def feed(self, seq, payload, now):
        if not 0<=seq<=255 or not 1<=len(payload)<=7:
            raise ValueError('CAN 串口分片格式无效')
        payload=bytes(payload)
        if self.last_rx is not None and now-self.last_rx>self.quiet_s:self.reset()
        self.last_rx=now
        self.frames+=1
        if self._expect is not None:
            delta=(seq-self._expect)&255
            if delta>=128:
                if (seq,payload) in self._recent:
                    self.duplicates+=1
                else:
                    self.out_of_order+=1
                    self._break()
                # 不倒退期望序号，也不把迟到的残片拼到正在接收的消息中。
                return b''
            if delta:
                self.gaps+=delta
                self.gap_events+=1
                self._break()
        self._expect=(seq+1)&255
        self._recent.append((seq,payload))
        self.bytes_in+=len(payload)
        self.buf+=payload
        out=self._messages()
        if out:self.last_message_rx=now
        return out

    @staticmethod
    def _start(buf, offset=0):
        candidates=[p for p in (_next_sync(buf,offset),buf.find(b'$',offset),buf.find(b'#',offset)) if p>=0]
        return min(candidates) if candidates else -1

    @staticmethod
    def _ascii_crc(line):
        body,star,check=line.rstrip(b'\r\n').rpartition(b'*')
        if not star or not body or any(c<32 or c>126 for c in body):return False
        expected_len=8 if body.startswith(b'#') else 2
        if len(check)!=expected_len or not re.fullmatch(rb'[0-9a-fA-F]+',check):return False
        expected=int(check,16)
        if body.startswith(b'#'):return crc32(body[1:])==expected
        value=0
        for c in body[1:]:value^=c
        return value==expected

    def _messages(self):
        out=bytearray()
        while self.buf:
            start=self._start(self.buf)
            if start<0:
                self.buf=b''
                break
            if start:self.buf=self.buf[start:]
            if self.buf.startswith(b'\xaa'):
                kind,end=binary_frame(self.buf,0)
                if kind=='need':break
                if kind!='frame':
                    if kind=='crc':self.crc_errors+=1
                    else:self.bad_headers+=1
                    self.buf=self.buf[1:]
                    continue
                message,self.buf=self.buf[:end],self.buf[end:]
            else:
                end=self.buf.find(b'\n')
                if end<0:
                    binary=_next_sync(self.buf,1)
                    if binary>=0:
                        self.discarded_partials+=1
                        self.buf=self.buf[binary:]
                        continue
                    if len(self.buf)>self.MAX_PENDING:self._break()
                    break
                message=self.buf[:end+1]
                known=self.ASCII_START.match(message)
                if not known or not self._ascii_crc(message):
                    if known:self.crc_errors+=1
                    nested=self._start(self.buf,1)
                    if 0<=nested<end:
                        self.discarded_partials+=1
                        self.buf=self.buf[nested:]
                    else:self.buf=self.buf[end+1:]
                    continue
                self.buf=self.buf[end+1:]
            out.extend(message)
            self.messages+=1
            self.verified_bytes+=len(message)
        return bytes(out)

    def status(self):
        return dict(frames=self.frames,gaps=self.gaps,missing_frames=self.gaps,gap_events=self.gap_events,
                    duplicates=self.duplicates,out_of_order=self.out_of_order,resyncs=self.resyncs,
                    discarded_partials=self.discarded_partials,crc_errors=self.crc_errors,bad_headers=self.bad_headers,
                    bytes_in=self.bytes_in,valid_messages=self.messages,verified_bytes=self.verified_bytes,
                    pending_bytes=len(self.buf))
