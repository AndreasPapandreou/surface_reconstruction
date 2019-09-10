@echo off

set "QT_DIR=D:\Qt\5.5\msvc2013"
set "BOOST_DIR=D:\libs\boost_1_57_0\stage\lib"

%QT_DIR%\bin\windeployqt lib\VVRScene.dll

:: Boost [VS-2013-Release]
copy %BOOST_DIR%\BOOST_LOG-VC120-MT-1_57.DLL             .\lib
copy %BOOST_DIR%\BOOST_SYSTEM-VC120-MT-1_57.DLL          .\lib
copy %BOOST_DIR%\BOOST_THREAD-VC120-MT-1_57.DLL          .\lib
copy %BOOST_DIR%\BOOST_CHRONO-VC120-MT-1_57.DLL          .\lib
copy %BOOST_DIR%\BOOST_DATE_TIME-VC120-MT-1_57.DLL       .\lib
copy %BOOST_DIR%\BOOST_FILESYSTEM-VC120-MT-1_57.DLL      .\lib

:: Boost [VS-2013-Debug]
copy %BOOST_DIR%\BOOST_LOG-VC120-MT-GD-1_57.DLL          .\lib
copy %BOOST_DIR%\BOOST_SYSTEM-VC120-MT-GD-1_57.DLL       .\lib
copy %BOOST_DIR%\BOOST_THREAD-VC120-MT-GD-1_57.DLL       .\lib
copy %BOOST_DIR%\BOOST_CHRONO-VC120-MT-GD-1_57.DLL       .\lib
copy %BOOST_DIR%\BOOST_DATE_TIME-VC120-MT-GD-1_57.DLL    .\lib
copy %BOOST_DIR%\BOOST_FILESYSTEM-VC120-MT-GD-1_57.DLL   .\lib

pause
