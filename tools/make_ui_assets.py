"""Generate antialiased text masks; requires Pillow and Windows Arial Bold."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

root = Path(__file__).resolve().parents[1]
items = {str(i): (str(i), 174) for i in range(10)}
items.update(requested=('REQUESTED', 22), lbs=('LBS', 26),
             unknown=('UNCONFIRMED', 17), connected=('BLE CONNECTED', 16),
             offline=('BLE OFFLINE', 16), hint=('TURN TO SET', 16))
lines = ['#pragma once', '#include <stdint.h>', 'namespace uiAssets {',
         'struct Mask { uint16_t w,h; const uint8_t* data; };']
for name, (text, size) in items.items():
    font = ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf', size)
    box = font.getbbox(text)
    im = Image.new('L', (box[2]-box[0], box[3]-box[1]))
    ImageDraw.Draw(im).text((-box[0], -box[1]), text, font=font, fill=255)
    ident = 'digit'+name if name.isdigit() else name
    lines.append('const uint8_t '+ident+'Data[] = {'+','.join(map(str, im.tobytes()))+'};')
    lines.append(f'const Mask {ident} = {{{im.width},{im.height},{ident}Data}};')
lines.append('const Mask digits[] = {'+','.join('digit'+str(i) for i in range(10))+'};\n}')
(root / 'include/UiAssets.h').write_text('\n'.join(lines))
