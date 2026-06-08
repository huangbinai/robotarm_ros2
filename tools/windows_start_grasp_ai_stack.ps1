param(
    [string]$RepoRoot = "D:\BaiduNetdiskDownload\reBot-DevArm-main\reBot-DevArm-main",
    [bool]$Open3DVisualize = $true,
    [int]$VisualizeTopN = 5,
    [int]$VisualizeMaxPoints = 8000,
    [int]$VisualizeEveryN = 10
)

$ErrorActionPreference = "Stop"
Set-Location $RepoRoot

$yoloScript = Join-Path $RepoRoot "tools\windows_start_yolo_server.ps1"
$graspnetScript = Join-Path $RepoRoot "tools\windows_start_graspnet_bridge.ps1"

Start-Process powershell.exe -ArgumentList @(
    "-NoExit",
    "-ExecutionPolicy", "Bypass",
    "-File", $yoloScript
)

Start-Sleep -Seconds 3

Start-Process powershell.exe -ArgumentList @(
    "-NoExit",
    "-ExecutionPolicy", "Bypass",
    "-File", $graspnetScript,
    "-Open3DVisualize:$Open3DVisualize",
    "-VisualizeTopN", $VisualizeTopN,
    "-VisualizeMaxPoints", $VisualizeMaxPoints,
    "-VisualizeEveryN", $VisualizeEveryN
)
