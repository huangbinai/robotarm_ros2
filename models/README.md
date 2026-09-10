# Runtime model assets

Model weights are runtime assets and are intentionally excluded from normal Git
history. On a new Ubuntu checkout, download the versioned GraspNet checkpoint
from the GitHub Release and verify it:

```bash
mkdir -p models/graspnet
curl -L \
  https://github.com/huangbinai/robotarm_ros2/releases/download/graspnet-model-v1/checkpoint-rs.tar \
  -o models/graspnet/checkpoint-rs.tar
sha256sum --check models/MANIFEST.sha256
```

Expected assets and hashes are recorded in `MANIFEST.sha256`. The checked-in
YOLO model remains at `tools/yolo26s-seg.pt` for compatibility with the current
Windows vision launcher.
