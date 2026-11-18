$ErrorActionPreference='Stop'
$processes=Get-CimInstance Win32_Process
$busy=$processes | Where-Object { $_.Name -eq 'kit.exe' -or ($_.Name -match '^python(w)?\.exe$' -and $_.CommandLine -match 'run_(lab|full_map)_greenhouse\.py') }
if ($busy) { [Console]::Error.WriteLine('Close the existing Isaac Sim before launching another simulation.'); exit 1 }
Get-Date -Format yyyyMMdd_HHmmss
