"""使用Android SDK、ECJ和D8构建内置离线语音APK，不下载Gradle。"""
from pathlib import Path
import argparse, hashlib, json, os, shutil, subprocess, sys, zipfile, tempfile, xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
def run(args, env=None):
    print('RUN', Path(str(args[0])).name, ' '.join(map(str,args[1:4])), flush=True)
    subprocess.run([str(a) for a in args], check=True, env=env)

def main():
    p=argparse.ArgumentParser();p.add_argument('--keystore',type=Path,required=True);p.add_argument('--key-alias',default='acg138');p.add_argument('--tools',type=Path,required=True);p.add_argument('--jdk',type=Path,default=Path(r'C:\Program Files\Java\jdk-17'));p.add_argument('--temp-root',type=Path);a=p.parse_args()
    cache=a.tools.resolve();sdk=cache/'sdk';sdk.mkdir(exist_ok=True)
    for name,sha,directory in [('build-tools_r35_windows.zip','af059bb67cf7786f45ee0db85e2d24985df1b4b6','android-15'),('platform-35_r02.zip','0bb560a90a7a2cbd0dd8348224d518b638fe7949','android-35')]:
        file=cache/name
        if hashlib.sha1(file.read_bytes()).hexdigest()!=sha:raise RuntimeError('Official SDK archive checksum mismatch: '+name)
        if not (sdk/directory).exists():
            with zipfile.ZipFile(file) as z:
                for member in z.infolist():
                    if not (sdk/member.filename).resolve().is_relative_to(sdk):raise RuntimeError('Unsafe SDK archive path')
                z.extractall(sdk)
    bt=sdk/'android-15';platform=sdk/'android-35'/'android.jar'
    # 平台包根目录可能使用android-15名称，按内容定位工具。
    tools=next(sdk.rglob('aapt2.exe')).parent
    jars=[x for x in sdk.rglob('android.jar') if 'android-35' in str(x) or (x.parent/'source.properties').exists()]
    if jars:platform=jars[0]
    if not platform.exists():platform=next(sdk.rglob('android.jar'))
    source_original=ROOT/'android/app/src/main';build_root=ROOT/'build/android';build_root.mkdir(parents=True,exist_ok=True)
    build=Path(tempfile.mkdtemp(prefix='acg_v2_apk_',dir=a.temp_root))
    sdk_ascii=build/'sdk';shutil.copytree(sdk,sdk_ascii);sdk=sdk_ascii
    tools=next(sdk.rglob('aapt2.exe')).parent;platform=next(sdk.rglob('android.jar'))
    source=build/'source';shutil.copytree(source_original,source)
    classes=build/'classes';dex=build/'dex';classes.mkdir(exist_ok=True);dex.mkdir(exist_ok=True)
    dependencies=[ROOT/'third_party/sherpa/classes.jar',ROOT/'third_party/kotlin-stdlib-2.0.21.jar']
    for file in dependencies:
        if not file.is_file():raise RuntimeError('Missing offline voice dependency: '+str(file))
    lib_ascii=build/'libs';lib_ascii.mkdir()
    deps=[]
    for file in dependencies:
        dest=lib_ascii/file.name;shutil.copy2(file,dest);deps.append(dest)
    env=os.environ.copy();env['JAVA_HOME']=str(a.jdk);env['PATH']=str(a.jdk/'bin')+os.pathsep+env.get('PATH','')
    resources=build/'resources.zip';unsigned=build/'unsigned.apk';aligned=build/'aligned.apk'
    run([tools/'aapt2.exe','compile','--dir',source/'res','-o',resources],env)
    ET.register_namespace('android','http://schemas.android.com/apk/res/android')
    manifest=ET.parse(source/'AndroidManifest.xml');manifest.getroot().set('package','cn.acg138.rover.odom')
    native_manifest=build/'AndroidManifest.xml';manifest.write(native_manifest,encoding='UTF-8',xml_declaration=True)
    run([tools/'aapt2.exe','link','-I',platform,'--manifest',native_manifest,'--min-sdk-version','26','--target-sdk-version','35','-A',source/'assets','-o',unsigned,resources],env)
    run([a.jdk/'bin/java.exe','-jar',cache/'ecj.jar','-source','8','-target','8','-encoding','UTF-8','-classpath',os.pathsep.join(map(str,[platform,*deps])),'-d',classes,*source.joinpath('java').rglob('*.java')],env)
    app_jar=build/'app-classes.jar'
    with zipfile.ZipFile(app_jar,'w') as z:
        for file in classes.rglob('*.class'):z.write(file,file.relative_to(classes).as_posix())
    run(['cmd.exe','/d','/c',tools/'d8.bat','--lib',platform,'--min-api','26','--output',dex,app_jar,*deps],env)
    with zipfile.ZipFile(unsigned,'a',compression=zipfile.ZIP_DEFLATED) as z:
        for file in dex.glob('*.dex'):z.write(file,file.name)
        for file in source.joinpath('jniLibs').rglob('*.so'):z.write(file,'lib/'+file.relative_to(source/'jniLibs').as_posix())
    run([tools/'zipalign.exe','-p','-f','4',unsigned,aligned],env)
    key=a.keystore.resolve()
    if not key.is_file():raise RuntimeError('Signing keystore not found')
    if not env.get('ACG138_KEYSTORE_PASS') or not env.get('ACG138_KEY_PASS'):raise RuntimeError('Set ACG138_KEYSTORE_PASS and ACG138_KEY_PASS before signing')
    version=manifest.getroot().get('{http://schemas.android.com/apk/res/android}versionName')
    release=ROOT/'release';release.mkdir(exist_ok=True);apk=release/f'ACG138-Rover-v{version}.apk'
    run(['cmd.exe','/d','/c',tools/'apksigner.bat','sign','--ks',key,'--ks-key-alias',a.key_alias,'--ks-pass','env:ACG138_KEYSTORE_PASS','--key-pass','env:ACG138_KEY_PASS','--out',apk,aligned],env)
    validation=ROOT/'validation';validation.mkdir(exist_ok=True)
    result=subprocess.run(['cmd.exe','/d','/c',str(tools/'apksigner.bat'),'verify','--verbose','--print-certs',str(apk)],check=True,capture_output=True,env=env)
    (validation/'apk-signature.txt').write_bytes(result.stdout+result.stderr)
    verify_apk=build/'verify.apk';shutil.copy2(apk,verify_apk)
    result=subprocess.run([str(tools/'aapt.exe'),'dump','badging',str(verify_apk)],check=True,capture_output=True,env=env)
    (validation/'apk-badging.txt').write_bytes(result.stdout+result.stderr)
    with zipfile.ZipFile(apk) as z:
        assert z.testzip() is None
        assert {'classes.dex','AndroidManifest.xml','assets/index.html','assets/app.js','assets/style.css','assets/odometry.js'}.issubset(z.namelist())
        assert {n for n in z.namelist() if n.startswith('assets/')}=={'assets/'+file.relative_to(source/'assets').as_posix() for file in (source/'assets').rglob('*') if file.is_file()}
        assert 'assets/sherpa-zh/model.int8.onnx' in z.namelist() and 'assets/sherpa-zh/tokens.txt' in z.namelist()
        assert 'lib/arm64-v8a/libsherpa-onnx-jni.so' in z.namelist()
        for file in (source/'assets').rglob('*'):
            if file.is_file():assert z.read('assets/'+file.relative_to(source/'assets').as_posix())==file.read_bytes()
        for file in (source/'jniLibs').rglob('*.so'):assert z.read('lib/'+file.relative_to(source/'jniLibs').as_posix())==file.read_bytes()
        for name in ['index.html','app.js','style.css','odometry.js','nav-map.js','objects.js','object-dual.js','voice.js','scene.js']:assert z.read('assets/'+name)==(source/'assets'/name).read_bytes()
    inputs={}
    for path in source.rglob('*'):
        if not path.is_file():continue
        relative=Path('android/app/src/main')/path.relative_to(source)
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        assert hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()==digest,'Source changed while building: '+str(relative)
        inputs[relative.as_posix()]=digest
    (validation/'apk-built-inputs.json').write_text(json.dumps(inputs,indent=2)+'\n',encoding='utf-8')
    (validation/'apk-release.json').write_text(json.dumps({'version':version,'version_code':int(manifest.getroot().get('{http://schemas.android.com/apk/res/android}versionCode')),'apk_sha256':hashlib.sha256(apk.read_bytes()).hexdigest(),'embedded_assets_verified':True,'source_files':len(inputs)},indent=2)+'\n',encoding='utf-8')
    print('APK:',apk,'SHA256:',hashlib.sha256(apk.read_bytes()).hexdigest(),flush=True)
if __name__=='__main__':main()
