"""替换WebView资源，并使用现有应用密钥签名。"""
from pathlib import Path
import argparse
import os
import re
import subprocess
import tempfile
import zipfile


def run(command, env):
    subprocess.run(list(map(str, command)), check=True, env=env,
                   capture_output=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--apk', type=Path, required=True)
    parser.add_argument('--assets', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--build-tools', type=Path, required=True)
    parser.add_argument('--keystore', type=Path, required=True)
    parser.add_argument('--key-alias', default='acg138')
    parser.add_argument('--jdk', type=Path, required=True)
    args = parser.parse_args()
    env = dict(os.environ)
    env['JAVA_HOME'] = str(args.jdk)
    env['PATH'] = str(args.jdk / 'bin') + os.pathsep + env.get('PATH', '')
    if not env.get('ACG138_KEYSTORE_PASS') or not env.get('ACG138_KEY_PASS'):
        raise ValueError('Set ACG138_KEYSTORE_PASS and ACG138_KEY_PASS')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='apk-assets-') as temporary:
        stage = Path(temporary)
        unsigned = stage / 'unsigned.apk'
        aligned = stage / 'aligned.apk'
        signed = stage / 'signed.apk'
        changed = []
        with zipfile.ZipFile(args.apk) as original, zipfile.ZipFile(unsigned, 'w') as output:
            for info in original.infolist():
                if re.match(r'META-INF/[^/]+\.(?:SF|RSA|DSA|EC|MF)$', info.filename, re.I):
                    continue
                data = original.read(info)
                if info.filename.startswith('assets/') and not info.is_dir():
                    relative = info.filename[len('assets/'):]
                    replacement = args.assets / relative
                    if not replacement.resolve().is_relative_to(args.assets.resolve()):
                        raise ValueError('Asset path is outside its root')
                    if replacement.is_file() and replacement.read_bytes() != data:
                        data = replacement.read_bytes()
                        changed.append(info.filename)
                output.writestr(info, data)
        run([args.build_tools / 'zipalign.exe', '-p', '-f', '4', unsigned, aligned], env)
        signing = ['--ks', args.keystore, '--ks-key-alias', args.key_alias,
                   '--ks-pass', 'env:ACG138_KEYSTORE_PASS',
                   '--key-pass', 'env:ACG138_KEY_PASS']
        if args.keystore.read_bytes()[:4] != b'\xfe\xed\xfe\xed':
            from cryptography.hazmat.primitives import serialization
            from cryptography.hazmat.primitives.serialization import pkcs12
            key, certificate, _ = pkcs12.load_key_and_certificates(
                args.keystore.read_bytes(), env['ACG138_KEYSTORE_PASS'].encode())
            if key is None or certificate is None:
                raise ValueError('PKCS12 contains no signing key and certificate')
            key_file = stage / 'signing.pk8'
            certificate_file = stage / 'signing.der'
            key_file.write_bytes(key.private_bytes(serialization.Encoding.DER,
                                 serialization.PrivateFormat.PKCS8,
                                 serialization.NoEncryption()))
            key_file.chmod(0o600)
            certificate_file.write_bytes(certificate.public_bytes(serialization.Encoding.DER))
            signing = ['--key', key_file, '--cert', certificate_file]
        run(['cmd.exe', '/d', '/c', args.build_tools / 'apksigner.bat', 'sign',
             *signing, '--out', signed, aligned], env)
        run(['cmd.exe', '/d', '/c', args.build_tools / 'apksigner.bat', 'verify',
             '--verbose', signed], env)
        args.output.write_bytes(signed.read_bytes())
        print('PASS signed assets:', ', '.join(changed))


if __name__ == '__main__':
    main()
