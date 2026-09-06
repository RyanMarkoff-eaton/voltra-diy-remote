"""Compare embedded data with a local checkout of the pinned upstream SDK."""
import base64
import json
import pathlib
import re
import subprocess
import sys

root = pathlib.Path(__file__).resolve().parents[1]
upstream = pathlib.Path(sys.argv[1]).resolve()
header = (root / 'include/ProtocolData.h').read_text()
pin = re.search(r'commit ([0-9a-f]{40})', header)[1]
head = subprocess.check_output(['git', '-C', str(upstream), 'rev-parse', 'HEAD'], text=True).strip()
assert head == pin, (head, pin)
source = (upstream / 'src/voltra/protocol/data/protocol-data.generated.ts').read_text()
data = json.loads(base64.b64decode(re.search(r"const _e = '([^']+)'", source)[1]))
arrays = [bytes(int(v, 16) for v in text.split(',')).hex()
          for text in re.findall(r'\{((?:0x[0-9a-fA-F]{2},?)+)\}', header)]
commands = data['commands']
original_auth = bytes.fromhex(commands['auth']['iphone'])
custom_auth = bytes.fromhex(arrays[0])
assert custom_auth[:11] == original_auth[:11]
assert custom_auth[11:32] == b'voltraRemote' + bytes(9)
assert custom_auth[32:-2] == original_auth[32:-2]
assert len(custom_auth) == len(original_auth)
assert arrays[1:4] == [*commands['init'], commands['workout']['stop']]
assert arrays[5:7] == [commands['workout']['setup'], commands['workout']['go']]
assert arrays[7:] == [commands['weights'][str(n)] for n in range(5, 201)]
for key, value in data['ble'].items():
    assert value in header, (key, value)
guided = (upstream / 'src/voltra/protocol/guided-load.ts').read_text()
assert arrays[4] in guided
print('PASS: pinned commit, BLE identifiers, customized identity name, init, STOP, SETUP, GO, 196 weights, guided-load capture. Run check_packets.py for CRC validation.')
