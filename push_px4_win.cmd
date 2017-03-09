@echo on
adb wait-for-device
adb remount
adb root



set board=eagle_legacy_driver_default

set adsp_lib_dir=/usr/share/data/adsp
set user_lib_dir=/usr/lib
set usr_app_bin=/home/root

rem RUNNING for 8096!!!

set adsp_lib_dir=/usr/lib/rfsa/adsp
set board=excelsior_legacy



adb shell rm %usr_app_bin%/px4

adb push posix-configs\eagle\210qc\px4.config %adsp_lib_dir%/px4.config

adb push posix-configs\excelsior\px4.config 		%adsp_lib_dir%/px4.config
adb push posix-configs\excelsior\mainapp.config  	%usr_app_bin%/mainapp.config

adb push build_posix_%board%\src\firmware\posix\px4 %usr_app_bin%/px4
adb shell chmod 755 %usr_app_bin%/px4

adb push build_qurt_%board%\src\firmware\qurt\libpx4.so %adsp_lib_dir%/.
adb push build_qurt_%board%\src\firmware\qurt\libpx4muorb_skel.so  %adsp_lib_dir%/.
sleep 8
adb shell %usr_app_bin%/px4 %usr_app_bin%/mainapp.config

