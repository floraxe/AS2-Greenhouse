@echo off
setlocal
set AS2_RUN=
for /f %%I in ('powershell -NoProfile -ExecutionPolicy Bypass -File E:\AS2_Greenhouse\work\check_full_map_simulation_idle.ps1') do set "AS2_RUN=%%I"
if not defined AS2_RUN (
  echo Close the existing Isaac Sim window before opening another simulation.
  pause
  exit /b 1
)
echo Ubuntu: bash ~/as2_slam/start-one-path-live.sh
cd /d E:\AS2_Greenhouse
set ROS_DISTRO=humble
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
set ROS_DOMAIN_ID=0
set ROS_LOCALHOST_ONLY=0
set AMENT_PREFIX_PATH=E:\isaacsim\exts\isaacsim.ros2.core\humble
set PATH=E:\isaacsim\exts\isaacsim.ros2.core\humble\lib;%PATH%
set PYTHONDONTWRITEBYTECODE=1
set OPENBLAS_NUM_THREADS=1
set OMP_NUM_THREADS=2
set WARP_CACHE_PATH=E:\AS2_Greenhouse\runtime\warp_cache
set TEMP=E:\AS2_Greenhouse\runtime\tmp
set TMP=E:\AS2_Greenhouse\runtime\tmp
call E:\isaacsim\python.bat E:\AS2_Greenhouse\work\run_full_map_greenhouse.py --ros --level-odometry --odometry-filter --lidar-detail --online-fusion --route23 --complete-loop-test --output-dir E:\AS2_Greenhouse\outputs\four_aisle_slam_%AS2_RUN% --portable-root E:\AS2_Greenhouse\runtime --/app/extensions/registryEnabled=false
if errorlevel 1 (
  echo.
  echo Simulation failed. The latest error report is shown below:
  powershell -NoProfile -Command "$f=Get-ChildItem 'E:\AS2_Greenhouse\outputs\four_aisle_slam_*\error.txt' | Sort-Object LastWriteTime -Descending | Select-Object -First 1; if($f){Get-Content -LiteralPath $f.FullName -Tail 40}"
)
pause


