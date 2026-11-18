@echo off
setlocal
set "VMHOST="
set /p "VMHOST=Ubuntu IP (Enter for 192.168.220.129): "
if not defined VMHOST set "VMHOST=192.168.220.129"
powershell -NoProfile -Command "$ip=$null; if (-not [Net.IPAddress]::TryParse($env:VMHOST,[ref]$ip)) {exit 1}"
if errorlevel 1 exit /b 1
ssh -o ConnectTimeout=8 -o ServerAliveInterval=15 "flora@%VMHOST%" "while true; do date; uptime; df -h ~/as2_slam | tail -1; ps -eo pid,etime,pcpu,comm | grep -E 'rtabmap|icp_odometry' | head; d=$(ls -td ~/as2_slam/loop-replay-* 2>/dev/null | head -1); if test -n \"$d\"; then ls -lh \"$d\"/*.db 2>/dev/null; tail -c 2000 \"$d\"/*.log 2>/dev/null | tr '\r' '\n' | tail -4; fi; sleep 10; done"
pause
