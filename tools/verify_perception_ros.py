#!/usr/bin/env python3
"""Exercise real GPU perception nodes through ROS with recorded RGB-D input.

Only YOLO, GraspNet and a recorded-image publisher are created. No robot,
camera SDK, candidate executor, TF broadcaster, or motion client is started.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=182)
    parser.add_argument("--timeout", type=float, default=45.)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ["OMP_NUM_THREADS"] = "2"
    os.environ["OPENBLAS_NUM_THREADS"] = "2"
    os.environ["REBOTARM_WORKSPACE"] = str(root)
    import cv2
    import numpy as np
    import rclpy
    import scipy.io
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CameraInfo
    from rebotarm_msgs.msg import GraspCandidateArray
    from rebotarm_vision.converters.image_msgs import color_to_msg, depth_to_msg, camera_info_to_msg

    sample = root / "third_party/graspnet-baseline/doc/example_data"
    color = cv2.imread(str(sample / "color.png"))
    depth = cv2.imread(str(sample / "depth.png"), cv2.IMREAD_UNCHANGED)
    meta = scipy.io.loadmat(str(sample / "meta.mat"))
    k = meta["intrinsic_matrix"]
    depth = (depth.astype(float) * 1000. / float(np.asarray(meta["factor_depth"]).reshape(-1)[0])).astype(np.uint16)
    info = dict(width=depth.shape[1], height=depth.shape[0], fx=float(k[0,0]), fy=float(k[1,1]),
                cx=float(k[0,2]), cy=float(k[1,2]), d=[0.] * 5)
    rclpy.init(args=[])
    node = rclpy.create_node("recorded_rgbd_acceptance")
    children, log_streams = [], []
    try:
        discovery_end = time.monotonic() + 1.
        while time.monotonic() < discovery_end:
            rclpy.spin_once(node, timeout_sec=0.1)
        foreign = [name for name in node.get_node_names() if name != node.get_name()]
        if foreign:
            raise RuntimeError(f"acceptance domain is occupied: {foreign}")
        pubs = [
            node.create_publisher(Image, "/camera/color/image_raw", qos_profile_sensor_data),
            node.create_publisher(Image, "/camera/depth/image_raw", qos_profile_sensor_data),
            node.create_publisher(CameraInfo, "/camera/depth/camera_info", qos_profile_sensor_data),
        ]
        received, published_stamps = [], set()
        node.create_subscription(GraspCandidateArray, "/grasp/graspnet_candidates", received.append, 10)
        log_dir = Path(tempfile.mkdtemp(prefix="perception-ros-", dir=root / ".codex_tmp"))
        for module in ("yolo_node", "graspnet_baseline_node"):
            stream = (log_dir / f"{module}.log").open("w")
            log_streams.append(stream)
            code = f"from rebotarm_vision.{module} import main; main()"
            children.append(subprocess.Popen([sys.executable, "-c", code], stdout=stream, stderr=stream,
                                             start_new_session=True))
        deadline = time.monotonic() + args.timeout
        next_frame = time.monotonic()
        while time.monotonic() < deadline:
            for child in children:
                if child.poll() is not None:
                    raise RuntimeError(f"perception node exited: {child.returncode}; logs: {log_dir}")
            if time.monotonic() >= next_frame:
                stamp = node.get_clock().now().to_msg()
                published_stamps.add((stamp.sec, stamp.nanosec))
                messages = (color_to_msg(color,stamp,"camera_depth_frame"),
                            depth_to_msg(depth,stamp,"camera_depth_frame"),
                            camera_info_to_msg(info,stamp,"camera_depth_frame"))
                for pub, message in zip(pubs, messages): pub.publish(message)
                next_frame = time.monotonic() + 0.2
            rclpy.spin_once(node, timeout_sec=0.02)
            if sum(bool(msg.candidates) for msg in received) >= 2:
                break
        candidates = [msg for msg in received if msg.candidates]
        source_matches = all((msg.header.stamp.sec,msg.header.stamp.nanosec) in published_stamps for msg in candidates)
        received.clear()
        deadline = time.monotonic() + 4.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        cleared = bool(received and not received[-1].candidates and received[-1].best_index == -1)
        report = dict(ok=bool(candidates) and source_matches and cleared, domain=args.domain,
                      source="upstream recorded RGB-D; real YOLO and GraspNet GPU models",
                      nonempty_candidate_messages=len(candidates), candidate_counts=[len(m.candidates) for m in candidates],
                      source_stamps_preserved=source_matches, candidates_cleared_after_input_stop=cleared,
                      camera_connected=False, robot_connected=False, logs=str(log_dir))
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["ok"] else 1
    finally:
        for child in children:
            if child.poll() is None: os.killpg(child.pid, signal.SIGINT)
        for child in children:
            try: child.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=5.)
        for stream in log_streams: stream.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
