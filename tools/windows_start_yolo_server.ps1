param(
    [string]$PythonExe = "D:\anaconda3\envs\orbbec_yolo\python.exe",
    [string]$RepoRoot = "D:\BaiduNetdiskDownload\reBot-DevArm-main\reBot-DevArm-main",
    [string]$GraspNetCandidatesPath = "D:\tmp\graspnet_candidates.json"
)

$ErrorActionPreference = "Stop"
Set-Location $RepoRoot

& $PythonExe tools\windows_mjpeg_server.py `
    --capture-source orbbec `
    --host 0.0.0.0 `
    --port 8081 `
    --width 1280 `
    --height 720 `
    --fps 30 `
    --depth-width 1280 `
    --depth-height 720 `
    --depth-fps 30 `
    --jpeg-quality 80 `
    --conf-threshold 0.25 `
    --iou-threshold 0.45 `
    --detection-fps 15.0 `
    --classes bottle `
    --allowed-classes bottle `
    --graspnet-candidates-path $GraspNetCandidatesPath
