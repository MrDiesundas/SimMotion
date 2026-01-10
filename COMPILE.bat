rem pyinstaller SimMotion.py --onedir --noconfirm
pyinstaller SimMotion.py --onedir --noconfirm --add-data "C:\Arbeitsplatz\SimMotion\.venv\Lib\site-packages\SimConnect\SimConnect.dll;SimConnect"
xcopy config.ini dist\SimMotion /Y /I
xcopy images dist\SimMotion\images /E /H /C /I /Y