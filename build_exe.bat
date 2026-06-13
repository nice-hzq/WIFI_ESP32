@echo off
REM ============================================================
REM  步态分析系统 — 打包为独立 .exe
REM  Usage: 双击运行，或在终端执行 build_exe.bat
REM ============================================================
cd /d "%~dp0"

REM 激活 venv
call python\venv\Scripts\activate.bat

REM 清理旧构建
if exist build rmdir /s /q build
if exist dist rmdir /s /q dist

REM PyInstaller 打包（绕过 Python 3.10.0 dis.py 的 bug）
python -c ^"
import dis
_orig = dis._get_const_info
def _patched(a, c):
    try: return _orig(a, c)
    except IndexError: return (repr(a), str(a))
dis._get_const_info = _patched

import PyInstaller.__main__
PyInstaller.__main__.run([
    '--clean',
    'GaitAnalysis.spec',
])
" || goto :error

echo.
echo ============================================================
echo  Build complete!
echo  Output: dist\GaitAnalysis.exe
echo ============================================================
goto :end

:error
echo.
echo BUILD FAILED.
pause

:end
