@echo off
chcp 65001 >nul 2>&1
set PYTHONIOENCODING=utf-8
set PYTHONUTF8=1
set MSYSTEM=
set MSYSTEM_PREFIX=
set MSYSTEM_CHOST=
set MSYSTEM_CARCH=
set MINGW_PREFIX=
set MINGW_CHOST=
set MINGW_PACKAGE_PREFIX=
set SHELL=
set SHLVL=
set TERM=
set IDF_PATH=E:\Espressif\Espressif\frameworks\esp-idf-v5.1.2
set IDF_TOOLS_PATH=E:\Espressif\Espressif
set IDF_PYTHON=E:\Espressif\Espressif\python_env\idf5.1_py3.11_env\Scripts\python.exe
set PATH=E:\Espressif\Espressif\tools\cmake\3.24.0\bin;%PATH%
set PATH=E:\Espressif\Espressif\tools\ninja\1.10.2;%PATH%
set PATH=E:\Espressif\Espressif\tools\xtensa-esp32s3-elf\esp-12.2.0_20230208\xtensa-esp32s3-elf\bin;%PATH%
set PATH=E:\Espressif\Espressif\tools\idf-git\2.43.0\cmd;%PATH%
set PATH=E:\Espressif\Espressif\tools\idf-exe\1.0.3;%PATH%
set PATH=E:\Espressif\Espressif\python_env\idf5.1_py3.11_env\Scripts;%PATH%
cd /d E:\Robot
echo === Building Robot Arm ===
%IDF_PYTHON% "%IDF_PATH%\tools\idf.py" build
