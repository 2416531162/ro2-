import serial, time
s=serial.Serial('/dev/ttyACM0',460800,timeout=0.05)
s.dtr=True; s.rts=True
s.reset_input_buffer()
chunks=[]; start=time.monotonic()
while time.monotonic()-start<3:
    chunks.append(s.read(max(1,min(s.in_waiting,8192))))
s.close()
b=b''.join(chunks)
open('/tmp/n10p-raw.bin','wb').write(b)
print('captured_bytes='+str(len(b)))
