"""更新文件校验清单并打包整合目录。"""
from pathlib import Path
import argparse
import hashlib
import json
import re
import stat
import zipfile


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def included(root, path):
    relative = path.relative_to(root)
    if any(part in {'.git', '.gradle', '__pycache__', 'node_modules'}
           for part in relative.parts):
        return False
    if relative.parts[0] == 'Source' and len(relative.parts) > 1:
        if relative.parts[1] in {'build', 'release', 'validation'}:
            return False
    return path.suffix.lower() not in {'.pyc', '.jks', '.keystore'}


def refresh_manifest(folder):
    lines = []
    for path in sorted(folder.rglob('*')):
        if path.is_file() and path != folder / 'SHA256SUMS.txt' and included(folder, path):
            lines.append(sha(path) + '  ' + path.relative_to(folder).as_posix())
    (folder / 'SHA256SUMS.txt').write_text('\n'.join(lines) + '\n', encoding='utf8')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    output = args.output.resolve()
    if output.is_relative_to(root):
        raise ValueError('Write the archive outside the delivery directory')
    if output.exists():
        raise ValueError('Output archive already exists')
    version = json.loads((root / 'VERSION.json').read_text(encoding='utf8'))
    runtime = root / version['runtime']
    for path in sorted(runtime.rglob('*')):
        if not path.is_file() or path.name not in {'SHA256SUMS', 'SHA256SUMS.txt'}:
            continue
        if path.parent == runtime:
            continue
        lines = []
        for line in path.read_text(encoding='utf8').splitlines():
            match = re.fullmatch(r'[a-fA-F0-9]{64}\s+\*?(.+)', line)
            if not match:
                raise ValueError('Invalid checksum record: ' + path.name)
            target = (path.parent / match[1]).resolve()
            if not target.is_relative_to(runtime) or not target.is_file():
                raise ValueError('Checksum target unavailable: ' + match[1])
            lines.append(sha(target) + '  ' + match[1])
        path.write_text('\n'.join(lines) + '\n', encoding='utf8')
    refresh_manifest(runtime)
    refresh_manifest(root)
    files = [p for p in sorted(root.rglob('*')) if p.is_file() and included(root, p)]
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, 'x', zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for path in files:
            if path.name.lower().endswith(('.zip', '.tar.gz', '.rar', '.7z')):
                raise ValueError('Nested delivery archive: ' + path.name)
            data = path.read_bytes()
            record = zipfile.ZipInfo(path.relative_to(root).as_posix(), (2026, 11, 4, 0, 0, 0))
            record.create_system = 3
            record.compress_type = zipfile.ZIP_DEFLATED
            mode = 0o755 if data.startswith((b'\x7fELF', b'#!')) else 0o644
            record.external_attr = (stat.S_IFREG | mode) << 16
            archive.writestr(record, data)
    if output.stat().st_size >= 200_000_000:
        raise ValueError('Archive exceeds the 200 MB delivery limit')
    with zipfile.ZipFile(output) as archive:
        if archive.testzip() is not None:
            raise ValueError('ZIP CRC verification failed')
        for entry in archive.infolist():
            if hashlib.sha256(archive.read(entry)).hexdigest() != sha(root / entry.filename):
                raise ValueError('ZIP file hash mismatch: ' + entry.filename)
    output.with_suffix('.sha256').write_text(sha(output) + '  ' + output.name + '\n', encoding='utf8')
    print(json.dumps(dict(status='PASS', files=len(files), bytes=output.stat().st_size,
                          sha256=sha(output)), indent=2))


if __name__ == '__main__':
    main()
