"""本地构建目录及编译器查找，不写入私人路径。"""
from pathlib import Path
import os
import shutil
import tempfile


def build_path(relative=""):
    """编译暂存目录位于源码外，可指定纯ASCII路径。"""
    root = Path(os.environ.get("ACG138_BUILD_ROOT", str(Path(tempfile.gettempdir()) / "acg138-build")))
    result = root / relative
    result.mkdir(parents=True, exist_ok=True)
    return result


def zig_path():
    """依次优先使用ZIG_EXE、PATH和构建目录下可选工具链。"""
    configured = os.environ.get("ZIG_EXE")
    if configured:
        return Path(configured)
    found = shutil.which("zig")
    if found:
        return Path(found)
    name = "zig.exe" if os.name == "nt" else "zig"
    return build_path() / "flow-v8-phone-api" / "ziglang" / name
