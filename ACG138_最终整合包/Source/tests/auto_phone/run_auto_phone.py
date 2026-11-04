"""编译并测试自动启动事务与取消屏障。"""
from pathlib import Path
import argparse
import re
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--java', type=Path, required=True)
    parser.add_argument('--ecj', type=Path, required=True)
    parser.add_argument('--temp-root', type=Path)
    args = parser.parse_args()
    source = Path(__file__).resolve().parents[2]
    tests = Path(__file__).resolve().parent
    app = source / 'android/app/src/main/java/cn/acg138/rover'
    service = (app / 'SemanticTaskService.java').read_text(encoding='utf8')
    begin = service.index(' public static void awaitRevocationForAuto()')
    end = service.index(' private ScheduledExecutorService heartbeat,reader;', begin)
    method = service[begin:end]
    activity = (app / 'MainActivity.java').read_text(encoding='utf8')
    begin = activity.index('    private void startAutomatic(){')
    end = activity.index('    private final class Bridge', begin)
    auto = activity[begin:end]
    assert auto.index('awaitRevocationForAuto();') < auto.index('postControl("stop"')
    assert auto.index('postControl("stop"') < auto.index('AutoSwitch.execute(')
    server = (source / 'board/phone_auto_switch.h').read_text(encoding='utf8')
    phone = (app / 'AutoSwitch.java').read_text(encoding='utf8')
    ttl = int(re.search(r'PHONE_AUTO_INTENT_US\s+UINT64_C\((\d+)\)', server)[1])
    limit = int(re.search(r'expires-issued>(\d+)', phone)[1])
    assert ttl == limit == 8000000
    with tempfile.TemporaryDirectory(prefix='auto-phone-test-', dir=args.temp_root) as temporary:
        stage = Path(temporary)
        fixture = stage / 'SemanticTaskService.java'
        # 编译实际屏障方法，不加载Android服务。
        fixture.write_text('package cn.acg138.rover;\n'
                           'import java.io.*; import java.util.concurrent.*;\n'
                           'final class SemanticTaskService {\n'
                           'static Future<?> revocation;\n' + method + '}\n', encoding='utf8')
        command = [str(args.java), '-jar', str(args.ecj), '-source', '8', '-target', '8',
                   '-encoding', 'UTF-8', '-d', str(stage), str(app / 'AutoSwitch.java'),
                   str(fixture), str(tests / 'AutoSwitchTest.java'), str(tests / 'RevocationTest.java')]
        subprocess.run(command, check=True)
        for name in ['AutoSwitchTest', 'RevocationTest']:
            subprocess.run([str(args.java), '-cp', str(stage), 'cn.acg138.rover.' + name], check=True)
    print('AUTO start ordering and server TTL agreement PASS')


if __name__ == '__main__':
    main()
