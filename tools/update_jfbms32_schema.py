"""Regenerate JetFleet embedded XML and VESC Tool schema signatures."""
import argparse
from pathlib import Path
import re
import xml.etree.ElementTree as ET
import zlib

HW_ROOT = Path(__file__).resolve().parents[1] / 'main/hwconf/jetfleet'
BOARDS = ('jfbms32', 'jfbms_master', 'jfbms_slave', 'jf_link')


def signature(schema):
    # ConfigParams::getSignature() / Utility::crc32c() in VESC Tool.
    params = schema.find('Params')
    parts = []
    for field in schema.find('SerOrder'):
        name = field.text
        param = params.find(name)
        parts.append(name + param.findtext('type') + param.findtext('vTx', '0') +
                     ''.join(item.text or '' for item in param.findall('enumNames')))
    crc = 0xffffffff
    for byte in ''.join(parts).encode('utf-8'):
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82f63b78 if crc & 1 else 0)
    return crc ^ 0xffffffff


def run(board='jfbms32'):
    hw = HW_ROOT / board
    xml = (hw / f'{board}_settings.xml').read_bytes()
    sig = signature(ET.fromstring(xml))
    blob = len(xml).to_bytes(4, 'big') + zlib.compress(xml, 9)
    rows = [ '\t' + ', '.join(f'0x{value:02x}' for value in blob[i:i + 16]) + ','
             for i in range(0, len(blob), 16)]
    (hw / f'{board}_confxml.c').write_text(
        f'// Generated from {board}_settings.xml by tools/update_jfbms32_schema.py\n\n'
        f'#include "{board}_confxml.h"\n\n'
        '__attribute__((used)) uint8_t data_main_config_t_[DATA_MAIN_CONFIG_T__SIZE] = {\n' +
        '\n'.join(rows) + '\n};\n', encoding='utf-8', newline='\n')
    header = hw / f'{board}_confxml.h'
    header.write_text(re.sub(r'(DATA_MAIN_CONFIG_T__SIZE\s+)\d+',
                            lambda m: m[1] + str(len(blob)), header.read_text()), encoding='utf-8', newline='\n')
    parser = hw / f'{board}_confparser.h'
    parser.write_text(re.sub(r'(MAIN_CONFIG_T_SIGNATURE\s+)\d+',
                            lambda m: m[1] + str(sig), parser.read_text()), encoding='utf-8', newline='\n')
    print(f'{board} schema signature {sig}, compressed XML {len(blob)} bytes')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--board', choices=(*BOARDS, 'all'), default='jfbms32')
    args = parser.parse_args()
    for board in BOARDS if args.board == 'all' else (args.board,):
        run(board)
