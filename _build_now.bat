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
set MSYS=
set MSYS_HOME=
set MSYS2_PATH_TYPE=

set IDF_PATH=E:\Espressif\Espressif\frameworks\esp-idf-v5.1.2
set IDF_TOOLS_PATH=E:\Espressif\Espressif
set IDF_PYTHON=E:\Espressif\Espressif\python_env\idf5.1_py3.11_env\Scripts\python.exe

set PATH=E:\Espressif\Espressif\tools\cmake\3.24.0\bin;E:\Espressif\Espressif\tools\ninja\1.10.2;E:\Espressif\Espressif\tools\xtensa-esp32s3-elf\esp-12.2.0_20230208\xtensa-esp32s3-elf\bin;E:\Espressif\Espressif\tools\xtensa-esp-elf-gdb\12.1_20221002\xtensa-esp-elf-gdb\bin;E:\Espressif\Espressif\tools\idf-exe\1.0.3;E:\Espressif\Espressif\tools\ccache\4.8\ccache-4.8-windows-x86_64;E:\Espressif\Espressif\tools\dfu-util\0.11\dfu-util-0.11-win64;E:\Espressif\Espressif\tools\idf-git\2.43.0\cmd;E:\Espressif\Espressif\python_env\idf5.1_py3.11_env\Scripts;C:\Windows\system32;C:\Windows

cd /d E:\Robot
%IDF_PYTHON% "%IDF_PATH%\tools\idf.py" build
