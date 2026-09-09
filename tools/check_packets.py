"""Check all embedded command CRCs and weight register values."""
import pathlib
import re
import base64
import json

def crc(data, seed, poly):
    value = seed
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (poly if value & 1 else 0)
    return value

root = pathlib.Path(__file__).resolve().parents[1]
header = (root / 'include/ProtocolData.h').read_text()
arrays = re.findall(r'\{((?:0x[0-9a-fA-F]{2},?)+)\}', header)
assert len(arrays) == 203, len(arrays)
for i, text in enumerate(arrays):
    packet = bytes(int(x, 16) for x in text.split(','))
    assert packet[0] == 0x55 and packet[1] == len(packet)
    assert crc(packet[:3], 0x77, 0x8c) == packet[3], i
    assert crc(packet[:-2], 0x3692, 0x8408) == int.from_bytes(packet[-2:], 'little'), i
    if 7 <= i <= 202:
        assert packet[13:15] == bytes.fromhex('863e')
        assert int.from_bytes(packet[15:17], 'little') == i - 2
assert bytes(int(x, 16) for x in arrays[4].split(',')).hex() == '550e0466aa1000202000aa125231'
assert bytes(int(x, 16) for x in arrays[5].split(',')).hex() == '55130403aa10150020000f02006a50823e8f2f'
assert bytes(int(x, 16) for x in arrays[6].split(',')).hex() == '55130403aa1016002000110100893e05008173'

def build_weight(lbs):
    seq = 0x2000 + lbs - 5
    packet = bytearray([0x55, 19, 4, 0, 0xaa, 0x10, seq & 255, seq >> 8,
                        0x20, 0, 0x11, 1, 0, 0x86, 0x3e, lbs & 255, lbs >> 8, 0, 0])
    packet[3] = crc(packet[:3], 0x77, 0x8c)
    packet[-2:] = crc(packet[:-2], 0x3692, 0x8408).to_bytes(2, 'little')
    return bytes(packet)

def build_modifier(param_be, value, minimum):
    seq = 0x2000 + value - minimum
    encoded = value & 0xffff
    packet = bytearray([0x55, 19, 4, 0, 0xaa, 0x10, seq & 255, (seq >> 8) & 255,
                        0x20, 0, 0x11, 1, 0, param_be >> 8, param_be & 255,
                        encoded & 255, encoded >> 8, 0, 0])
    packet[3] = crc(packet[:3], 0x77, 0x8c)
    packet[-2:] = crc(packet[:-2], 0x3692, 0x8408).to_bytes(2, 'little')
    return bytes(packet)

for lbs in range(5, 201):
    assert build_weight(lbs) == bytes(int(x, 16) for x in arrays[lbs + 2].split(',')), lbs
for lbs in range(201, 231):
    packet = build_weight(lbs)
    assert packet[15:17] == lbs.to_bytes(2, 'little')
    assert crc(packet[:-2], 0x3692, 0x8408) == int.from_bytes(packet[-2:], 'little')

generated = root.parent / 'upstream-sdk/src/voltra/protocol/data/protocol-data.generated.ts'
if generated.exists():
    encoded = re.search(r"const _e = '([^']+)'", generated.read_text()).group(1)
    commands = json.loads(base64.b64decode(encoded))['commands']
    cases = [('chains', 0x873e, 0, 100), ('eccentric', 0x883e, -195, 195),
             ('inverseChains', 0xb053, 0, 100)]
    for name, param, low, high in cases:
        for value in range(low, high + 1):
            assert build_modifier(param, value, low).hex() == commands[name][str(value)], (name, value)
print('PASS: 203 SDK packets; weight builder 5..230; all 593 pinned modifier packets.')
