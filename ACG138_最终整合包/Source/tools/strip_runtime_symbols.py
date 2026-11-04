"""移除ELF调试段，并核对可加载代码和数据。"""
from pathlib import Path
import argparse
import hashlib
import json
import struct
import subprocess
import tempfile


def sections(data):
    if data[:6] != b'\x7fELF\x02\x01':
        raise ValueError('Expected a little-endian ELF64 file')
    offset = struct.unpack_from('<Q', data, 40)[0]
    size, count, strings_index = struct.unpack_from('<HHH', data, 58)
    headers = [struct.unpack_from('<IIQQQQIIQQ', data, offset + i * size)
               for i in range(count)]
    strings = headers[strings_index]
    names = data[strings[4]:strings[4] + strings[5]]
    result = {}
    for header in headers:
        name = names[header[0]:names.find(b'\0', header[0])].decode('ascii')
        content = b'' if header[1] == 8 else data[header[4]:header[4] + header[5]]
        result[name] = (header, content)
    return result


def check_loaded_sections(before, after):
    old = sections(before)
    new = sections(after)
    count = 0
    for name, (header, content) in old.items():
        if not header[2] & 2:
            continue
        if name not in new:
            raise ValueError('Loaded section removed: ' + name)
        other, result = new[name]
        if (header[1:4], header[5], content) != (other[1:4], other[5], result):
            raise ValueError('Loaded section changed: ' + name)
        count += 1
    if before[16:40] != after[16:40]:
        raise ValueError('Executable identity or entry point changed')
    return count


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runtime', type=Path, required=True)
    parser.add_argument('--strip-tool', default='aarch64-linux-gnu-strip')
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    records = []
    with tempfile.TemporaryDirectory(prefix='elf-release-') as temporary:
        stage = Path(temporary)
        for path in sorted(args.runtime.rglob('*')):
            if not path.is_file():
                continue
            before = path.read_bytes()
            if not before.startswith(b'\x7fELF'):
                continue
            headers = sections(before)
            if not any(name.startswith('.debug') for name in headers):
                continue
            original = stage / 'input.elf'
            output = stage / 'output.elf'
            original.write_bytes(before)
            subprocess.run([args.strip_tool, '--strip-debug', '-o', str(output),
                            str(original)], check=True, capture_output=True)
            after = output.read_bytes()
            count = check_loaded_sections(before, after)
            if any(name.startswith('.debug') for name in sections(after)):
                raise ValueError('Debug sections remain')
            path.chmod(path.stat().st_mode | 0o200)
            path.write_bytes(after)
            records.append(dict(file=path.relative_to(args.runtime).as_posix(),
                                before_bytes=len(before), after_bytes=len(after),
                                loaded_sections_verified=count,
                                before_sha256=hashlib.sha256(before).hexdigest(),
                                after_sha256=hashlib.sha256(after).hexdigest()))
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(dict(status='PASS', files=records), indent=2)
                           + '\n', encoding='utf8')
    print('PASS ELF debug removal:', len(records), 'files')


if __name__ == '__main__':
    main()
