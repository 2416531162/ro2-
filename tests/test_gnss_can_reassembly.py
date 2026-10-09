"""真实 UM982 报文夹具经过 CAN 重复/缺片/乱序后，只允许完整校验通过的数据进入定位。"""
import ast
from collections import deque
from contextlib import nullcontext
import json
from pathlib import Path
import socket
import struct
import sys
from types import SimpleNamespace as NS

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'radar_system'),str(ROOT/'tests')]
from gnss_can_reassembly import CanGnssReassembler
from gnss_bridge import CanSerialBridge,CAN_FRAME,CAN_DATA_ID,CAN_ACK_ID,CAN_EFF_FLAG
from test_um982_binary import frames,SAMPLE
from um982 import Um982Stream,crc32,nmea_checksum

BINARY=frames(Path(SAMPLE).read_bytes())
PACKET=next(p for p in BINARY if struct.unpack_from('<H',p,4)[0]==42)

def parts(messages,start=0,width=7):
    out=[];seq=start
    for msg in messages:
        for at in range(0,len(msg),width):
            out.append((seq,msg[at:at+width]));seq=(seq+1)&255
    return out

def feed(rows,r=None,t0=0.):
    r=r or CanGnssReassembler()
    return r,b''.join(r.feed(s,p,t0+i*.001) for i,(s,p) in enumerate(rows))

def nmea(body):return ('$%s*%02X\r\n'%(body,nmea_checksum(body))).encode()

def ascii_log(body):return b'#'+body+b'*'+('%08X'%crc32(body)).encode()+b'\r\n'

ASCII=ascii_log(b'BESTPOSA,COM1,0,55.0,FINESTEERING,2437,443243.000,00000000,0000,1114;'
                b'SOL_COMPUTED,NARROW_INT,31.61309208,120.72830388,25.15,8.50,WGS84,0.010,0.011,0.020,"0",'
                b'1.000,0.000,40,32,32,32,0,00,00,00')
NMEA=nmea('GNGGA,004553.00,3122.07183817,N,12044.22006923,E,4,28,0.6,13.7943,M,8.7952,M,2.0,137')
INS=nmea('INS.NAVI,4553.000,31.367863969,120.737001154,22.589430003,0.001646,-0.000153,0.002961,'
         '-0.013674,-0.074294,9.999900,-1.000000,0.000000,0.000000,0.000000,-0.000542,-0.001380,0.007070,'
         '46.739132,169607169')

@pytest.mark.parametrize('width',range(1,8))
def test_real_messages_survive_every_fragment_width_and_sequence_wrap(width):
    messages=BINARY+[ASCII,NMEA,INS]
    r,out=feed(parts(messages,250,width))
    assert out==b''.join(messages)
    assert r.messages==len(messages)
    assert r.gaps==r.duplicates==r.out_of_order==r.crc_errors==0

@pytest.mark.parametrize('index',[0,1,5,14])
def test_duplicate_fragment_never_corrupts_packet_or_adds_255_gaps(index):
    rows=parts([PACKET]);rows.insert(index,rows[index])
    r,out=feed(rows)
    assert out==PACKET and r.duplicates==1
    assert r.gaps==r.out_of_order==r.crc_errors==0
    assert len(Um982Stream().feed(out,1.))==1

def test_delayed_duplicate_is_dropped_without_breaking_current_packet():
    rows=parts([PACKET]);rows.insert(8,rows[1])
    r,out=feed(rows)
    assert out==PACKET and r.duplicates==1 and r.discarded_partials==0

def test_duplicate_from_entire_backward_half_window_preserves_current_packet():
    rows=parts([PACKET]*10);rows.insert(105,rows[5])
    r,out=feed(rows)
    assert out==PACKET*10 and r.duplicates==1 and r.discarded_partials==0

@pytest.mark.parametrize('index',[1,5,14])
def test_missing_fragment_discards_only_incomplete_message(index):
    rows=parts([PACKET,PACKET]);del rows[index]
    r,out=feed(rows)
    assert out==PACKET
    assert r.gaps==1 and r.gap_events==1 and r.discarded_partials==1
    assert r.crc_errors==0

def test_reordered_fragments_are_not_stitched_across_the_break():
    rows=parts([PACKET,PACKET]);rows[5],rows[6]=rows[6],rows[5]
    r,out=feed(rows)
    assert out==PACKET
    assert r.gaps==1 and r.out_of_order==1 and r.duplicates==0

def test_conflicting_repeat_is_not_treated_as_identical_duplicate():
    rows=parts([PACKET,PACKET]);seq,payload=rows[5]
    rows.insert(6,(seq,bytes([payload[0]^1])+payload[1:]))
    r,out=feed(rows)
    assert out==PACKET and r.out_of_order==1 and r.duplicates==0

def test_multiple_breaks_do_not_discard_previously_completed_messages():
    rows=parts([PACKET]*4);per=len(parts([PACKET]))
    for index in sorted([per+4,2*per+7],reverse=True):del rows[index]
    r,out=feed(rows)
    assert out==PACKET*2 and r.gaps==2

def test_real_crc_failure_remains_visible_and_next_packet_recovers():
    damaged=bytearray(PACKET);damaged[40]^=1
    r,out=feed(parts([damaged,PACKET]))
    assert out==PACKET and r.crc_errors==1 and r.gaps==0

def test_ascii_checksums_and_single_antenna_messages_remain_strict():
    bad_ascii=ASCII.replace(b'31.613',b'32.613')
    bad_nmea=NMEA[:-4]+b'00\r\n'
    r,out=feed(parts([bad_ascii,bad_nmea,ASCII,NMEA,INS]))
    assert out==ASCII+NMEA+INS and r.crc_errors==2

def test_valid_ascii_body_may_contain_hash_and_dollar():
    line=ascii_log(ASCII[1:ASCII.index(b'*')].replace(b'"0"',b'"base#$1"'))
    _,out=feed(parts([line]))
    assert out==line

def test_missing_newline_recovers_following_valid_ascii():
    r,out=feed(parts([ASCII.rstrip(b'\r\n')+NMEA]))
    assert out==NMEA and r.crc_errors==1

def test_quiet_reconnect_drops_half_packet_and_accepts_new_sequence():
    rows=parts([PACKET],70);r,_=feed(rows[:5])
    assert r.buf
    _,out=feed(parts([PACKET],0),r,3.)
    assert out==PACKET and r.gaps==0 and r.discarded_partials==1

def test_repeated_wraps_do_not_mistake_identical_payloads_for_duplicates():
    r,out=feed(parts([PACKET]*100))
    assert out==PACKET*100 and r.duplicates==r.gaps==0

def test_large_backward_jump_is_not_claimed_as_255_missing_frames():
    r=CanGnssReassembler();r.feed(40,b'\xaa\x44',0.)
    assert not r.feed(200,b'noise',.01)
    assert r.out_of_order==1 and r.gaps==0 and not r.buf
    _,out=feed(parts([PACKET],41),r,.02)
    assert out==PACKET

def test_bad_input_cannot_grow_pending_buffer_without_bound():
    r,out=feed(parts([b'$'+b'x'*20000]))
    assert not out and len(r.buf)<=r.MAX_PENDING and r.discarded_partials>0

def can_frame(row,can_id=CAN_DATA_ID):
    seq,payload=row;data=bytes([seq])+payload
    return struct.pack(CAN_FRAME,can_id,len(data),data.ljust(8,b'\0'))

class FakeSocket:
    def __init__(self,rows=(),repeat=None):
        self.rows=deque(rows);self.repeat=repeat;self.sent=[];self.reads=0;self.closed=False;self.timeout=None
    def settimeout(self,value):self.timeout=value
    def recv(self,n):
        self.reads+=1
        if self.rows:
            value=self.rows.popleft()
            if isinstance(value,Exception):raise value
            return value
        if self.repeat is not None:return self.repeat
        if self.timeout==0.:raise BlockingIOError()
        raise socket.timeout()
    def send(self,value):self.sent.append(value)
    def close(self):self.closed=True

def bridge(sock,clock=None):
    clock=clock or [1.]
    return CanSerialBridge(sock_factory=lambda:sock,clock=lambda:clock[0])

def test_bridge_read_boundaries_never_expose_a_partial_packet():
    sock=FakeSocket([can_frame(x) for x in parts([PACKET])]);b=bridge(sock);out=[]
    while sock.rows:out.append(b.read(7))
    assert b''.join(out)==PACKET
    assert all(x in (b'',PACKET) for x in out)

def test_bridge_gap_across_reads_and_ack_wire_format():
    rows=parts([PACKET,PACKET]);clock=[1.];sock=FakeSocket([can_frame(x) for x in rows[:5]]);b=bridge(sock,clock)
    assert b.read()==b''
    sock.rows.extend(can_frame(x) for x in rows[6:]);clock[0]=1.6
    assert b.read()==PACKET
    status=b.status();assert status['gaps']==status['missing_frames']==1 and status['valid_messages']==1
    cid,size,data=struct.unpack(CAN_FRAME,sock.sent[-1])
    assert cid==CAN_ACK_ID and size==5
    assert struct.unpack('<HHB',data[:5])[1]==1

def test_duplicate_storm_cannot_starve_read_loop():
    sock=FakeSocket(repeat=can_frame(parts([PACKET])[0]));b=bridge(sock)
    assert b.read(14)==b'' and sock.reads==2
    assert b.status()['duplicates']==1 and b.status()['gaps']==0

def test_socket_reopen_resets_partial_and_sequence():
    sock=FakeSocket([can_frame(x) for x in parts([PACKET],80)[:4]]);b=bridge(sock)
    assert b.read()==b''
    b._close_sock();sock.rows.extend(can_frame(x) for x in parts([PACKET],0));b._open()
    assert b.read()==PACKET and b.status()['gaps']==0

def test_disconnect_preserves_messages_completed_before_socket_error():
    rows=[can_frame(x) for x in parts([PACKET])]+[OSError('test disconnect')]
    b=bridge(FakeSocket(rows))
    assert b.read()==PACKET
    assert b.sock is None and not b.connected() and not b.reassembly.buf

def test_invalid_can_frames_do_not_enter_gnss_stream():
    sock=FakeSocket([b'short',can_frame((0,b'wrong'),CAN_DATA_ID|CAN_EFF_FLAG)])
    b=bridge(sock)
    assert b.read()==b'' and b.status()['malformed_frames']==1 and b.status()['frames']==0

def test_can_prevalidation_crc_is_preserved_in_node_status_total():
    tree=ast.parse((ROOT/'radar_system/rtk_node.py').read_text())
    cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='RtkNode')
    method=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='status')
    ns={};exec(compile(ast.Module([method],type_ignores=[]),'rtk_node.status','exec'),ns)
    node=NS(stream_lock=nullcontext(),stream=NS(status=lambda now:{'crc_errors':2},last={}),
            source='can',tablet=None,bt=None,can=NS(status=lambda:{'connected':True,'crc_errors':3}),
            ser=None,port_error=None,tablet_error=None,
            cors=NS(client=None,configured=False,error=None,enabled=True,applied_revision=None,paused=None))
    assert ns['status'](node,1.)['crc_errors']==5
    assert ns['status'](node,1.)['crc_errors']==5
    node.source='usb'
    assert ns['status'](node,1.)['crc_errors']==2
