@echo off
set IDF_PATH=E:\Espressif\Espressif\frameworks\esp-idf-v5.1.2
set IDF_TOOLS_PATH=E:\Espressif\Espressif
call "%IDF_PATH%\export.bat"
cd /d E:\Robot
"%IDF_TOOLS_PATH%\tools\idf-python\3.11.2\python.exe" "%IDF_PATH%\tools\idf.py" build
