param(
    [string]$PythonExe = "D:\anaconda3\envs\graspnet\python.exe",
    [string]$RepoRoot = "D:\BaiduNetdiskDownload\reBot-DevArm-main\reBot-DevArm-main",
    [string]$ServerUrl = "http://127.0.0.1:8081",
    [string]$OutputPath = "D:\tmp\graspnet_candidates.json",
    [string]$ModelRoot = "D:\rebot_ai_models\graspnet-baseline",
    [string]$CheckpointPath = "D:\rebot_ai_models\graspnet-baseline\checkpoints\checkpoint-rs.tar",
    [int]$MaxGrasps = 20,
    [double]$PollHz = 0.5
)

$ErrorActionPreference = "Stop"
Set-Location $RepoRoot

& $PythonExe tools\windows_graspnet_baseline_bridge.py `
    --server-url $ServerUrl `
    --output-path $OutputPath `
    --model-root $ModelRoot `
    --checkpoint-path $CheckpointPath `
    --backend-module graspnet_baseline_inference `
    --device cuda:0 `
    --max-grasps $MaxGrasps `
    --poll-hz $PollHz
