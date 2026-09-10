# Runtime model assets

Model weights are runtime assets and are intentionally excluded from normal Git
history. On a new Ubuntu checkout, create `models/graspnet/` and place the
GraspNet checkpoint there:

```bash
mkdir -p models/graspnet
cp /path/to/checkpoint-rs.tar models/graspnet/checkpoint-rs.tar
sha256sum --check models/MANIFEST.sha256
```

Expected assets and hashes are recorded in `MANIFEST.sha256`. The checked-in
YOLO model remains at `tools/yolo26s-seg.pt` for compatibility with the current
Windows vision launcher.
