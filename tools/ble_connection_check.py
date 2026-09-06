"""USB serial helper for connection-only bench checks; never sends motor commands."""
import argparse
import pathlib
import time
import serial

parser = argparse.ArgumentParser()
parser.add_argument('action', choices=['scan', 'connect', 'status', 'capture'])
parser.add_argument('--port', default='COM6')
parser.add_argument('--address', help='Voltra BLE address; required for connect')
args = parser.parse_args()
if args.action == 'connect' and not args.address:
    parser.error('--address is required for connect')
command = f'connect {args.address}' if args.action == 'connect' else args.action
logdir = pathlib.Path(__file__).resolve().parents[1] / 'logs'
logdir.mkdir(exist_ok=True)
logpath = logdir / (time.strftime('%Y%m%d-%H%M%S') + '-' + args.action + '.log')
with serial.Serial(port=None, baudrate=115200, timeout=0.2) as port:
    port.dtr = False
    port.rts = False
    port.port = args.port
    port.open()
    with logpath.open('w', encoding='utf-8') as log:
        def capture(seconds):
            until = time.monotonic() + seconds
            while time.monotonic() < until:
                data = port.read(max(1, port.in_waiting))
                if data:
                    text = data.decode('utf-8', errors='replace')
                    print(text, end='', flush=True)
                    log.write(text)
                    log.flush()
        capture(2)
        if args.action != 'capture':
            port.write((command + '\n').encode())
            port.flush()
        capture(45 if args.action == 'capture' else (25 if args.action == 'connect' else 8))
        if args.action == 'connect':
            port.write(b'status\n')
            capture(3)
print('\nSaved:', logpath)
