@echo off
echo 正在启动 Qt Creator 并打开项目...

REM 尝试常见的 Qt Creator 安装路径
set "QT_CREATOR_PATHS=C:\Qt\Tools\QtCreator\bin\qtcreator.exe;C:\Program Files\Qt\Tools\QtCreator\bin\qtcreator.exe;C:\Program Files (x86)\Qt\Tools\QtCreator\bin\qtcreator.exe"

for %%i in (%QT_CREATOR_PATHS%) do (
    if exist "%%i" (
        echo 找到 Qt Creator: %%i
        "%%i" "F:\Roban\lemon\RobanWeb-main\CMakeLists.txt"
        goto :found
    )
)

echo 未找到 Qt Creator，请手动打开项目
echo 项目路径: F:\Roban\lemon\RobanWeb-main
echo 项目文件: CMakeLists.txt
pause

:found
