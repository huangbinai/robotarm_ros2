from __future__ import annotations

import json
import math
import threading
import time
from contextlib import suppress
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlsplit

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetStateValidity
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rebotarm_msgs.msg import ArmStatus, JointMotorState
from rebotarm_msgs.srv import SetTeachRecordPath
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .parameter_helpers import build_joint_limits
from .parameter_helpers import sensor_qos_kwargs
from .status_panel_state import TeleopStatusStore, encode_sse_event
from .teleop_core import validate_web_keyboard_command
from .teach_recording import (
    ReplayStartBand,
    build_replay_start_soft_points,
    estimate_teach_replay,
    inspect_teach_record,
    list_teach_record_files,
    load_teach_samples,
    normalize_teach_replay_settings,
    prepare_teach_replay_samples,
    prepared_teach_replay_to_dict,
    retime_teach_samples,
    teach_record_info_to_dict,
    teach_trajectory_preview_to_dict,
    validate_teach_dry_run_request,
    validate_teach_replay_execute_request,
    validate_teach_replay_stop_request,
)
from .moveit_planner import MoveItMotionPlanner
from .web_robot_assets import (
    DEFAULT_GRIPPER_LIMITS_M,
    load_gripper_limits,
    load_moveit_velocity_limits,
    load_urdf_joint_limits,
    merge_velocity_limits,
    merge_joint_limits,
    rewrite_package_mesh_uris,
    safe_mesh_path,
)
from .web_execute import (
    WebExecuteDecision,
    WebGripperDecision,
    interpolate_joint_points,
    validate_web_gripper_request,
    validate_web_execute_request,
)


def _set_duration(duration_msg, seconds: float) -> None:
    whole = int(seconds)
    duration_msg.sec = whole
    duration_msg.nanosec = int((float(seconds) - whole) * 1_000_000_000)


def _is_number_like(value) -> bool:
    try:
        float(value)
    except (TypeError, ValueError):
        return False
    return True


def _number_or_default(value, default: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return float(default)
    if not _is_number_like(number) or not math.isfinite(number):
        return float(default)
    return float(number)


def _select_collision_samples(samples, *, max_samples: int) -> list[tuple[int, object]]:
    if not samples:
        return []
    limit = max(int(max_samples), 1)
    if len(samples) <= limit:
        return list(enumerate(samples))
    if limit == 1:
        return [(0, samples[0])]
    indices = sorted(
        {
            round(index * (len(samples) - 1) / (limit - 1))
            for index in range(limit)
        }
    )
    return [(index, samples[index]) for index in indices]


def _decision_response(decision: WebExecuteDecision) -> dict:
    return {
        "accepted": bool(decision.accepted),
        "message": decision.message,
        "max_delta": float(decision.max_delta),
        "max_delta_limit": float(decision.max_delta_limit),
        "duration": float(decision.duration),
    }


def _gripper_decision_response(decision: WebGripperDecision) -> dict:
    return {
        "accepted": bool(decision.accepted),
        "message": decision.message,
        "position": float(decision.position),
        "max_effort": float(decision.max_effort),
    }


def _keyboard_decision_response(decision) -> dict:
    return {
        "accepted": bool(decision.accepted),
        "message": decision.message,
        "key": str(decision.key),
        "joint_name": str(decision.joint_name),
        "step_rad": float(decision.step_rad),
        "duration": float(decision.duration),
        "max_joint_speed_rad_s": float(decision.max_joint_speed_rad_s),
    }


HTML_PAGE = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>reBot Teleop Status</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #f5f7f9;
      --panel: #ffffff;
      --line: #d8dee6;
      --text: #17202a;
      --muted: #667085;
      --good: #188a4d;
      --warn: #b7791f;
      --bad: #c24135;
      --info: #2563a8;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: "Segoe UI", Arial, sans-serif;
      font-size: 14px;
      letter-spacing: 0;
    }
    .shell { max-width: 1440px; margin: 0 auto; padding: 16px; }
    .topbar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      margin-bottom: 14px;
    }
    h1 { margin: 0; font-size: 22px; font-weight: 650; }
    .timestamp { color: var(--muted); font-variant-numeric: tabular-nums; }
    .teleop-workbench {
      display: grid;
      grid-template-columns: minmax(0, 2fr) minmax(380px, 1fr);
      gap: 12px;
      align-items: start;
    }
    .robot-workspace,
    .control-cards {
      display: grid;
      gap: 12px;
      min-width: 0;
    }
    .control-cards {
      align-content: start;
      max-height: calc(100vh - 86px);
      overflow-y: auto;
      padding-right: 2px;
    }
    .card-note {
      color: var(--muted);
      line-height: 1.4;
      padding: 0 12px 10px;
      overflow-wrap: anywhere;
    }
    .label {
      color: var(--muted);
      font-size: 12px;
      line-height: 1.2;
      margin-bottom: 8px;
    }
    .value {
      font-size: 20px;
      font-weight: 650;
      line-height: 1.2;
      overflow-wrap: anywhere;
    }
    .value.small { font-size: 15px; font-weight: 600; }
    .panel {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 6px;
      overflow: hidden;
    }
    .panel-title {
      padding: 10px 12px;
      border-bottom: 1px solid var(--line);
      font-weight: 650;
    }
    .card-header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
      padding: 10px 12px;
      border-bottom: 1px solid var(--line);
      font-weight: 650;
    }
    .card-header .panel-title {
      padding: 0;
      border-bottom: 0;
    }
    .card-toggle {
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #ffffff;
      min-width: 30px;
      min-height: 26px;
      color: var(--muted);
      font: inherit;
      cursor: pointer;
    }
    .card-toggle:hover { background: #f3f6f9; }
    .collapsible-card.collapsed .card-body { display: none; }
    .collapsible-card.collapsed .card-header { border-bottom: 0; }
    .collapsible-card.collapsed .card-toggle::after { content: "+"; }
    .collapsible-card:not(.collapsed) .card-toggle::after { content: "-"; }
    table {
      border-collapse: collapse;
      width: 100%;
      table-layout: fixed;
    }
    th, td {
      border-bottom: 1px solid #edf0f3;
      padding: 9px 10px;
      text-align: right;
      font-variant-numeric: tabular-nums;
      white-space: nowrap;
    }
    th {
      color: var(--muted);
      background: #fafbfc;
      font-size: 12px;
      font-weight: 650;
    }
    th:first-child, td:first-child { text-align: left; width: 110px; }
    tr:last-child td { border-bottom: 0; }
    .robot-viewer { margin-bottom: 0; }
    .viewer-body {
      position: relative;
      min-height: min(72vh, 720px);
      background: #101820;
    }
    #robot-view {
      display: block;
      width: 100%;
      height: min(72vh, 720px);
    }
    .viewer-status {
      position: absolute;
      left: 12px;
      bottom: 12px;
      max-width: calc(100% - 24px);
      padding: 6px 8px;
      border-radius: 6px;
      background: rgba(255,255,255,0.88);
      color: #26313d;
      font-size: 12px;
      overflow-wrap: anywhere;
    }
    .viewer-tools {
      position: absolute;
      right: 12px;
      top: 12px;
      display: flex;
      gap: 8px;
    }
    .slider-panel { margin-bottom: 0; }
    .aux-hidden { display: none; }
    .slider-grid {
      padding: 12px;
      display: grid;
      grid-template-columns: 1fr;
      gap: 10px 14px;
    }
    .joint-slider {
      display: grid;
      grid-template-columns: 76px minmax(0, 1fr) 132px;
      gap: 10px;
      align-items: center;
      min-height: 34px;
    }
    .joint-name { font-weight: 650; font-variant-numeric: tabular-nums; }
    .joint-slider input[type="range"] {
      width: 100%;
      accent-color: var(--info);
    }
    .joint-slider input[disabled] { opacity: 0.72; }
    .joint-values {
      text-align: right;
      font-variant-numeric: tabular-nums;
      color: var(--muted);
      white-space: nowrap;
    }
    .joint-values strong {
      color: var(--text);
      font-weight: 650;
    }
    .joint-limit {
      margin-top: 2px;
      font-size: 11px;
      color: var(--muted);
    }
    .preview-toolbar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 10px;
      padding: 10px 12px;
      border-bottom: 1px solid var(--line);
    }
    .preview-toolbar .panel-title {
      padding: 0;
      border-bottom: 0;
    }
    .slider-pane[hidden] { display: none; }
    .tool-button {
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #ffffff;
      color: var(--text);
      min-height: 30px;
      padding: 5px 10px;
      font: inherit;
      cursor: pointer;
    }
    .tool-button:hover { background: #f3f6f9; }
    .preview-active input[type="range"] { accent-color: var(--warn); }
    .execute-row {
      border-top: 1px solid var(--line);
      padding: 10px 12px;
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 10px;
      align-items: center;
    }
    .execute-status {
      color: var(--muted);
      line-height: 1.35;
      overflow-wrap: anywhere;
    }
    .execute-settings {
      border-top: 1px solid var(--line);
      padding: 10px 12px;
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    .execute-setting {
      display: grid;
      grid-template-columns: 92px minmax(0, 1fr);
      gap: 8px;
      align-items: center;
      min-height: 34px;
    }
    .execute-setting label {
      color: var(--muted);
      font-size: 12px;
      line-height: 1.2;
    }
    .execute-setting input[type="number"] {
      width: 100%;
      min-height: 30px;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 4px 6px;
      font: inherit;
      font-variant-numeric: tabular-nums;
    }
    .execute-setting input[type="text"] {
      width: 100%;
      min-height: 30px;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 4px 6px;
      font: inherit;
    }
    .execute-setting.wide { grid-column: 1 / -1; }
    .robot-command-row {
      border-top: 1px solid var(--line);
      padding: 10px 12px;
      display: flex;
      gap: 8px;
      justify-content: flex-end;
      flex-wrap: wrap;
    }
    .teach-section-title {
      padding: 10px 12px 6px;
      color: var(--text);
      font-weight: 700;
      border-top: 1px solid var(--line);
    }
    .execute-button {
      border-color: #9f3a32;
      background: #c24135;
      color: #ffffff;
      font-weight: 650;
    }
    .execute-button:hover { background: #a9362e; }
    .execute-button:disabled {
      cursor: not-allowed;
      opacity: 0.5;
      background: #ffffff;
      color: var(--muted);
      border-color: var(--line);
    }
    .stop-button {
      border-color: #44515f;
      background: #44515f;
      color: #ffffff;
      font-weight: 650;
    }
    .stop-button:hover { background: #303b46; }
    .stop-button:disabled {
      cursor: not-allowed;
      opacity: 0.5;
      background: #ffffff;
      color: var(--muted);
      border-color: var(--line);
    }
    .gripper-button {
      border-color: #2563a8;
      background: #2563a8;
      color: #ffffff;
      font-weight: 650;
    }
    .gripper-button:hover { background: #1e4f86; }
    .gripper-button:disabled {
      cursor: not-allowed;
      opacity: 0.5;
      background: #ffffff;
      color: var(--muted);
      border-color: var(--line);
    }
    .status-list { padding: 10px 12px; display: grid; gap: 10px; }
    .status-row {
      display: grid;
      grid-template-columns: 96px minmax(0, 1fr);
      gap: 8px;
      align-items: start;
    }
    .status-key { color: var(--muted); font-size: 12px; padding-top: 3px; }
    .status-main { min-width: 0; }
    .chip {
      display: inline-flex;
      align-items: center;
      max-width: 100%;
      min-height: 24px;
      padding: 3px 8px;
      border-radius: 999px;
      background: #eef4ff;
      color: var(--info);
      font-weight: 650;
      overflow-wrap: anywhere;
    }
    .chip.good { background: #e8f6ee; color: var(--good); }
    .chip.warn { background: #fff6df; color: var(--warn); }
    .chip.bad { background: #fdecea; color: var(--bad); }
    .chip.pass { background: #e8f6ee; color: var(--good); }
    .chip.red { background: #fdecea; color: var(--bad); }
    .detail {
      margin-top: 5px;
      color: var(--muted);
      line-height: 1.35;
      overflow-wrap: anywhere;
    }
    .teach-panel { margin-bottom: 0; }
    .precheck-strip {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 8px;
      padding: 10px 12px;
      border-bottom: 1px solid var(--line);
      background: #fbfcfd;
    }
    .teach-grid {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 10px;
      padding: 12px;
    }
    .teach-metric {
      min-height: 70px;
      border: 1px solid #edf0f3;
      border-radius: 6px;
      padding: 9px 10px;
      background: #fafbfc;
      min-width: 0;
    }
    .teach-metric strong {
      display: block;
      font-size: 16px;
      line-height: 1.25;
      overflow-wrap: anywhere;
    }
    .compact-grid {
      grid-template-columns: repeat(2, minmax(0, 1fr));
      padding: 10px 12px;
      gap: 8px;
    }
    .compact-grid .teach-metric {
      min-height: 58px;
      padding: 8px 9px;
    }
    .compact-grid .teach-metric strong { font-size: 14px; }
    .teach-wide { grid-column: span 2; }
    .mini-limit-table {
      width: 100%;
      border-collapse: collapse;
      font-size: 12px;
      font-weight: 500;
      table-layout: fixed;
    }
    .mini-limit-table th,
    .mini-limit-table td {
      border-bottom: 1px solid var(--line);
      padding: 5px 4px;
      text-align: left;
      vertical-align: middle;
      overflow-wrap: anywhere;
    }
    .mini-limit-table th { color: var(--muted); font-weight: 650; }
    .teach-controls {
      border-top: 1px solid var(--line);
      padding: 10px 12px;
      display: grid;
      grid-template-columns: 1fr;
      gap: 10px;
    }
    .teach-control {
      display: grid;
      grid-template-columns: 110px minmax(0, 1fr) 72px;
      gap: 8px;
      align-items: center;
    }
    .teach-control label {
      color: var(--muted);
      font-size: 12px;
    }
    .teach-control input[type="range"] {
      width: 100%;
      accent-color: var(--info);
    }
    .teach-control input[type="number"] {
      width: 100%;
      min-height: 30px;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 4px 6px;
      font: inherit;
      font-variant-numeric: tabular-nums;
    }
    .event-log {
      border-top: 1px solid var(--line);
      padding: 10px 12px;
      display: grid;
      gap: 6px;
      color: var(--muted);
      font-size: 12px;
      font-variant-numeric: tabular-nums;
    }
    .event-item { overflow-wrap: anywhere; }
    details.panel {
      display: block;
      margin-bottom: 12px;
    }
    details.panel > summary {
      cursor: pointer;
      list-style: none;
      padding: 10px 12px;
      border-bottom: 1px solid var(--line);
      font-weight: 650;
      user-select: none;
    }
    details.panel > summary::-webkit-details-marker { display: none; }
    details.panel > summary::after {
      content: "+";
      float: right;
      color: var(--muted);
      font-weight: 650;
    }
    details.panel[open] > summary::after { content: "-"; }
    .details-body {
      border-top: 1px solid var(--line);
    }
    .record-toolbar {
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto;
      gap: 10px;
      padding: 10px 12px;
      border-bottom: 1px solid var(--line);
    }
    .command-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
      padding: 10px 12px;
      border-top: 1px solid var(--line);
    }
    .keymap-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 6px;
      padding: 0 12px 10px;
    }
    .keycap {
      border: 1px solid #edf0f3;
      border-radius: 6px;
      padding: 6px 7px;
      background: #fafbfc;
      font-size: 12px;
      color: var(--muted);
    }
    .keycap strong {
      color: var(--text);
      font-weight: 650;
    }
    .record-toolbar select {
      min-height: 32px;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 4px 8px;
      font: inherit;
      min-width: 0;
    }
    .trajectory-view { padding: 12px; display: grid; gap: 10px; }
    .trajectory-canvas-wrap {
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #ffffff;
      min-height: 240px;
    }
    #teach-trajectory-canvas {
      display: block;
      width: 100%;
      height: 240px;
    }
    .trajectory-controls {
      display: grid;
      grid-template-columns: 96px minmax(0, 1fr) auto;
      gap: 10px;
      align-items: center;
    }
    .trajectory-controls input[type="range"] {
      width: 100%;
      accent-color: var(--info);
    }
    .empty { color: var(--muted); padding: 14px; }
    @media (max-width: 980px) {
      .teleop-workbench { grid-template-columns: 1fr; }
      .control-cards { max-height: none; overflow: visible; padding-right: 0; }
      .precheck-strip { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .teach-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
      .teach-controls { grid-template-columns: 1fr; }
      .slider-grid { grid-template-columns: 1fr; }
      #robot-view { height: 340px; }
      .viewer-body { min-height: 340px; }
    }
    @media (max-width: 560px) {
      .joint-slider { grid-template-columns: 1fr; gap: 4px; }
      .joint-values { text-align: left; }
      .teach-grid { grid-template-columns: 1fr; }
      .precheck-strip { grid-template-columns: 1fr; }
      .compact-grid { grid-template-columns: 1fr; }
      .teach-wide { grid-column: span 1; }
      .keymap-grid { grid-template-columns: 1fr; }
      .command-grid { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>
  <main class="shell">
    <div class="topbar">
      <h1>reBot Teleop Status</h1>
      <div class="timestamp" id="updated">Waiting for data</div>
    </div>
    <section class="teleop-workbench">
      <div class="robot-workspace">
        <section class="panel robot-viewer">
          <div class="panel-title">Web 3D Robot View</div>
          <div class="viewer-body">
            <canvas id="robot-view"></canvas>
            <div class="viewer-status" id="robot-view-status">Loading Three.js robot view</div>
            <div class="viewer-tools">
              <button class="tool-button" id="toggle-grid" type="button">Grid</button>
            </div>
          </div>
          <div class="button-row robot-command-row">
            <button class="tool-button execute-button" id="arm-safe-home" type="button">Safe Home</button>
            <button class="tool-button" id="arm-enable" type="button">Enable</button>
            <button class="tool-button stop-button" id="arm-disable" type="button">Disable</button>
          </div>
        </section>
      </div>
      <aside class="control-cards">
        <section class="panel collapsible-card collapsed" id="arm-status-card">
          <div class="card-header"><div class="panel-title">Arm Status</div><button class="card-toggle" type="button" data-card-toggle="arm-status-card" aria-label="Toggle Arm Status"></button></div>
          <div class="card-body">
          <div class="status-list">
            <div class="status-row">
              <div class="status-key">Mode</div>
              <div class="status-main"><div class="value small" id="mode">-</div></div>
            </div>
            <div class="status-row">
              <div class="status-key">State</div>
              <div class="status-main"><div class="value small" id="state">-</div></div>
            </div>
            <div class="status-row">
              <div class="status-key">Enabled</div>
              <div class="status-main"><div class="value small" id="enabled">-</div></div>
            </div>
            <div class="status-row">
              <div class="status-key">Recording</div>
              <div class="status-main"><div class="value small" id="recording-state">-</div></div>
            </div>
            <div class="status-row">
              <div class="status-key">Replay</div>
              <div class="status-main"><div class="value small" id="replay-state">-</div></div>
            </div>
          </div>
          <div class="panel-title">Arm Errors</div>
          <div class="status-list" id="errors"><div class="empty">No errors</div></div>
          </div>
        </section>

        <section class="panel collapsible-card collapsed" id="motor-state-card">
          <div class="card-header"><div class="panel-title">Motor State</div><button class="card-toggle" type="button" data-card-toggle="motor-state-card" aria-label="Toggle Motor State"></button></div>
          <div class="card-body">
          <table>
            <thead><tr><th>Joint</th><th>Position rad</th><th>Velocity</th><th>Torque</th><th>Status</th></tr></thead>
            <tbody id="joints"><tr><td colspan="5" class="empty">Waiting for joint state</td></tr></tbody>
          </table>
          </div>
        </section>

        <section class="panel slider-panel collapsible-card collapsed" id="web-teleop-card">
          <div class="card-header"><div class="panel-title">Web Teleop</div><button class="card-toggle" type="button" data-card-toggle="web-teleop-card" aria-label="Toggle Web Teleop"></button></div>
          <div class="card-body">
          <div class="preview-toolbar">
            <button class="tool-button" id="sync-preview" type="button">Sync Live</button>
          </div>
          <div class="card-note">Preview sliders move the left 3D model first. Execute sends joint targets and the gripper target with one confirmation.</div>
          <div class="aux-hidden" id="joint-sliders"></div>
          <div class="slider-pane" id="preview-slider-pane">
            <div class="slider-grid" id="preview-sliders"></div>
          </div>
          <div class="execute-settings">
            <div class="execute-setting">
              <label for="execute-max-delta">Max Delta</label>
              <input id="execute-max-delta" type="number" min="0.05" max="1.50" step="0.05" value="0.80">
            </div>
            <div class="execute-setting">
              <label for="execute-duration">Duration</label>
              <input id="execute-duration" type="number" min="1.0" max="8.0" step="0.5" value="3.0">
            </div>
            <div class="execute-setting">
              <label for="execute-speed">Max Speed</label>
              <input id="execute-speed" type="number" min="0.1" max="1.5" step="0.1" value="1.5">
            </div>
          </div>
          <div class="execute-row">
            <div class="execute-status" id="execute-status">Preview only. Move a target slider, then execute with confirmation.</div>
            <div>
              <button class="tool-button execute-button" id="execute-preview" type="button" disabled>Execute Joints + Gripper</button>
              <button class="tool-button stop-button" id="stop-execute" type="button" disabled>Stop</button>
            </div>
          </div>
          </div>
        </section>

        <section class="panel teach-panel collapsible-card collapsed" id="teach-trajectory-card">
          <div class="card-header"><div class="panel-title">Teach Trajectory</div><button class="card-toggle" type="button" data-card-toggle="teach-trajectory-card" aria-label="Toggle Teach Trajectory"></button></div>
          <div class="card-body">
          <div class="record-toolbar">
            <select id="teach-record-select">
              <option value="">Default file</option>
            </select>
            <button class="tool-button" id="refresh-teach-records" type="button">Refresh Files</button>
          </div>
          <div class="teach-section-title">1. Record</div>
          <div class="execute-settings">
            <div class="execute-setting wide">
              <label for="teach-record-name">File Name</label>
              <input id="teach-record-name" type="text" value="teach_record.jsonl" autocomplete="off">
            </div>
          </div>
          <div class="command-grid">
            <button class="tool-button execute-button" id="start-teach-record" type="button">Start Teach</button>
            <button class="tool-button stop-button" id="stop-teach-record" type="button" disabled>Stop Teach</button>
          </div>
          <div class="teach-grid compact-grid" id="teach-record-summary">
            <div class="empty">Choose a file name, then start recording.</div>
          </div>
          <div class="teach-section-title">2. Check</div>
          <div class="precheck-strip" id="replay-precheck-summary">
            <div class="empty">No valid teach trajectory</div>
          </div>
          <button class="tool-button" id="run-teach-dry-run" type="button" disabled>Check Trajectory</button>
          <div class="teach-section-title">3. Replay</div>
          <div class="execute-row">
            <div class="execute-status" id="teach-dry-run-status">Check first. Replay stays blocked until safety checks pass.</div>
            <div>
              <button class="tool-button execute-button" id="run-teach-replay" type="button" disabled>Replay</button>
              <button class="tool-button stop-button" id="stop-teach-replay" type="button" disabled>Stop Replay</button>
            </div>
          </div>
          <div class="teach-controls">
            <div class="teach-control">
              <label for="teach-replay-speed">Replay Speed</label>
              <input id="teach-replay-speed" type="range" min="0.1" max="3.0" step="0.1" value="1.0">
              <input id="teach-replay-speed-number" type="number" min="0.1" max="3.0" step="0.1" value="1.0">
            </div>
            <div class="teach-control">
              <label for="teach-align-duration">Align Duration</label>
              <input id="teach-align-duration" type="range" min="1.0" max="10.0" step="0.5" value="3.0">
              <input id="teach-align-duration-number" type="number" min="1.0" max="10.0" step="0.5" value="3.0">
            </div>
            <div class="teach-control">
              <label for="teach-final-hold">Final Hold</label>
              <input id="teach-final-hold" type="range" min="0.0" max="5.0" step="0.1" value="1.0">
              <input id="teach-final-hold-number" type="number" min="0.0" max="5.0" step="0.1" value="1.0">
            </div>
          </div>
          <div class="teach-grid compact-grid" id="teach-params-details">
            <div class="empty">Waiting for replay estimate</div>
          </div>
          <details class="panel teach-panel" id="teach-file-details-panel">
            <summary>Trajectory File And Curve</summary>
            <div class="teach-grid" id="teach-file-details">
              <div class="empty">Waiting for teach record file</div>
            </div>
            <div class="trajectory-view">
              <div class="teach-grid" id="teach-trajectory-details">
                <div class="empty">Load a teach record to inspect trajectory quality</div>
              </div>
              <div class="trajectory-canvas-wrap">
                <canvas id="teach-trajectory-canvas"></canvas>
              </div>
              <div class="trajectory-controls">
                <label class="label" for="teach-trajectory-frame">Frame</label>
                <input id="teach-trajectory-frame" type="range" min="0" max="0" step="1" value="0" disabled>
                <button class="tool-button" id="load-teach-trajectory" type="button">Load Trajectory</button>
              </div>
            </div>
          </details>
          <details class="panel teach-panel" id="replay-event-log-panel">
            <summary>Advanced Log</summary>
            <div class="event-log" id="replay-event-log">
              <div class="empty">No replay events yet</div>
            </div>
          </details>
          </div>
        </section>

        <section class="panel collapsible-card collapsed" id="keyboard-teleop-card">
          <div class="card-header"><div class="panel-title">Keyboard Teleop</div><button class="card-toggle" type="button" data-card-toggle="keyboard-teleop-card" aria-label="Toggle Keyboard Teleop"></button></div>
          <div class="card-body">
          <div class="status-list">
            <div class="status-row">
              <div class="status-key">Input</div>
              <div class="status-main"><div class="value small" id="teleop-state">-</div></div>
            </div>
          </div>
          <div class="card-note">Enable this card, then press the mapped keys while the browser page is focused. Each key sends one small joint trajectory through the same controller action.</div>
          <div class="keymap-grid">
            <div class="keycap"><strong>1/q</strong> joint1 +/-</div>
            <div class="keycap"><strong>2/w</strong> joint2 +/-</div>
            <div class="keycap"><strong>3/e</strong> joint3 +/-</div>
            <div class="keycap"><strong>4/r</strong> joint4 +/-</div>
            <div class="keycap"><strong>5/t</strong> joint5 +/-</div>
            <div class="keycap"><strong>6/y</strong> joint6 +/-</div>
            <div class="keycap"><strong>Esc</strong> disable</div>
            <div class="keycap"><strong>focus</strong> page</div>
            <div class="keycap"><strong>stop</strong> button</div>
          </div>
          <div class="execute-settings">
            <div class="execute-setting">
              <label for="keyboard-step">Step</label>
              <input id="keyboard-step-number" type="number" min="0.005" max="0.100" step="0.005" value="0.020">
            </div>
            <div class="execute-setting">
              <label for="keyboard-speed">Speed</label>
              <input id="keyboard-speed-number" type="number" min="0.1" max="1.5" step="0.1" value="0.5">
            </div>
          </div>
          <div class="command-grid">
            <button class="tool-button execute-button" id="keyboard-enable" type="button">Enable Keyboard</button>
            <button class="tool-button stop-button" id="keyboard-disable" type="button" disabled>Disable Keyboard</button>
          </div>
          </div>
        </section>

      </aside>
    </section>
  </main>
  <script type="importmap">
    {
      "imports": {
        "three": "https://unpkg.com/three@0.165.0/build/three.module.js",
        "three/examples/jsm/": "https://unpkg.com/three@0.165.0/examples/jsm/"
      }
    }
  </script>
  <script type="module">
    import * as THREE from 'https://unpkg.com/three@0.165.0/build/three.module.js';
    import { OrbitControls } from 'https://unpkg.com/three@0.165.0/examples/jsm/controls/OrbitControls.js';
    import URDFLoader from 'https://unpkg.com/urdf-loader@0.12.6/src/URDFLoader.js';

    const panelConfig = await fetch('/api/config')
      .then((response) => response.json())
      .catch(() => ({
        joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
        joint_limits: {
          joint1: [-3.1416, 3.1416],
          joint2: [-3.1416, 3.1416],
          joint3: [-3.1416, 3.1416],
          joint4: [-3.1416, 3.1416],
          joint5: [-3.1416, 3.1416],
          joint6: [-3.1416, 3.1416],
        },
        gripper_limits: [0.0, 0.09],
        joint_velocity_limits: {
          joint1: 1.5,
          joint2: 1.5,
          joint3: 1.5,
          joint4: 1.5,
          joint5: 1.5,
          joint6: 1.5,
        },
      }));
    const source = new EventSource('/events');
    const panelMode = String(panelConfig.panel_mode || 'control').toLowerCase();
    const isCheckMode = panelMode === 'check';
    const armJointNames = panelConfig.joint_names || ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];
    const sliderNames = [...armJointNames, 'gripper'];
    const jointLimits = {
      ...(panelConfig.joint_limits || {}),
      gripper: panelConfig.gripper_limits || [0.0, 0.09],
    };
    const webExecuteEnabled = panelConfig.web_execute?.enabled === true;
    let latestTeachFileInfo = null;
    let selectedTeachRecordPath = '';
    let lastTeachDryRun = null;
    let lastReplayStatusKey = '';
    let lastFastRenderMs = 0;
    let lastSlowRenderMs = 0;
    const FAST_RENDER_INTERVAL_MS = 250;
    const replayEvents = [];
    const motorRowsByName = new Map();
    const fmt = (value, digits = 4) => Number.isFinite(value) ? Number(value).toFixed(digits) : '-';
    const deg = (value) => Number.isFinite(value) ? (Number(value) * 180 / Math.PI).toFixed(1) : '-';
    const text = (value) => value === undefined || value === null || value === '' ? '-' : String(value);
    const escapeHtml = (value) => text(value)
      .replaceAll('&', '&amp;')
      .replaceAll('<', '&lt;')
      .replaceAll('>', '&gt;')
      .replaceAll('"', '&quot;')
      .replaceAll("'", '&#39;');
    const statusObj = (value) => {
      if (value && typeof value === 'object' && !Array.isArray(value)) return value;
      return { state: text(value) };
    };
    const isDetailsOpen = (id) => document.getElementById(id)?.open === true;
    const isControlCardOpen = (id) => !document.getElementById(id)?.classList.contains('collapsed');
    const clearOptionalDetails = (id, html = '<div class="empty">Open to render details</div>') => {
      const el = document.getElementById(id);
      if (el) el.innerHTML = html;
    };
    const setHtml = (id, html) => {
      const el = document.getElementById(id);
      if (el) el.innerHTML = html;
    };
    const toggleControlCard = (cardId) => {
      const card = document.getElementById(cardId);
      if (!card) return;
      card.classList.toggle('collapsed');
      if (cardId === 'teach-trajectory-card' && !card.classList.contains('collapsed')) {
        refreshTeachFileInfo({ force: true });
      }
    };
    const attachControlCardToggles = () => {
      document.querySelectorAll('[data-card-toggle]').forEach((button) => {
        button.addEventListener('click', () => toggleControlCard(button.dataset.cardToggle));
      });
    };
    const classFor = (state) => {
      const normalized = String(state || '').toLowerCase();
      if (['active', 'recording', 'replaying', 'ready', 'idle', 'deadman', 'done', 'direct', 'green', 'pass', 'planned', 'skipped'].includes(normalized)) return 'good';
      if (['timeout', 'waiting', 'dry_run', 'starting', 'stopped', 'cancel_requested', 'canceled', 'align', 'unknown', 'missing', 'empty', 'invalid', 'yellow', 'disabled'].includes(normalized)) return 'warn';
      if (['failed', 'rejected', 'blocked', 'unavailable', 'reject', 'red'].includes(normalized)) return 'bad';
      return '';
    };
    const displayState = (state, context = '') => {
      const normalized = String(state || '').toLowerCase();
      if (context === 'keyboard' && normalized === 'timeout') return 'No key input';
      if (context === 'teach_file' && ['missing', 'empty', 'invalid'].includes(normalized)) return 'No valid teach trajectory';
      return text(state);
    };
    const setChip = (id, state) => {
      const el = document.getElementById(id);
      const cls = classFor(state);
      const context = id === 'teleop-state' ? 'keyboard' : '';
      el.innerHTML = `<span class="chip ${cls}">${displayState(state, context)}</span>`;
    };
    const syncRealActionButtons = () => {
      if (isCheckMode) {
        document.getElementById('execute-preview')?.setAttribute('disabled', '');
        document.getElementById('set-gripper')?.setAttribute('disabled', '');
        document.getElementById('run-teach-dry-run')?.setAttribute('disabled', '');
        document.getElementById('run-teach-replay')?.setAttribute('disabled', '');
        return;
      }
      if (webExecuteEnabled) return;
      document.getElementById('execute-preview')?.setAttribute('disabled', '');
      document.getElementById('set-gripper')?.setAttribute('disabled', '');
      document.getElementById('run-teach-replay')?.setAttribute('disabled', '');
    };
    const statusBlock = (label, value) => {
      const obj = statusObj(value);
      const state = obj.state || obj.status || '-';
      const details = [];
      if (obj.message) details.push(obj.message);
      if (obj.last_key) details.push(`last key: ${obj.last_key}`);
      if (Number.isFinite(obj.max_error)) details.push(`max error: ${fmt(obj.max_error)}`);
      if (Number.isFinite(obj.max_delta)) details.push(`max delta: ${fmt(obj.max_delta)}`);
      if (Number.isFinite(obj.max_delta_limit)) details.push(`limit: ${fmt(obj.max_delta_limit)}`);
      if (Number.isFinite(obj.duration)) details.push(`duration: ${fmt(obj.duration, 2)}s`);
      if (Number.isFinite(obj.max_joint_speed_rad_s)) details.push(`speed limit: ${fmt(obj.max_joint_speed_rad_s, 2)}rad/s`);
      if (Number.isFinite(obj.position)) details.push(`position: ${fmt(obj.position)}m`);
      if (Number.isFinite(obj.max_effort)) details.push(`effort: ${fmt(obj.max_effort, 2)}`);
      if (Number.isFinite(obj.samples)) details.push(`samples: ${obj.samples}`);
      if (Number.isFinite(obj.elapsed_sec)) details.push(`elapsed: ${fmt(obj.elapsed_sec, 1)}s`);
      if (Number.isFinite(obj.actual_sample_rate_hz)) details.push(`actual rate: ${fmt(obj.actual_sample_rate_hz, 1)}Hz`);
      if (Number.isFinite(obj.file_size_bytes)) details.push(`size: ${obj.file_size_bytes}B`);
      if (Number.isFinite(obj.trajectory_points)) details.push(`points: ${obj.trajectory_points}`);
      if (obj.writing_state) details.push(`writing: ${obj.writing_state}`);
      if (obj.arm_state) details.push(`arm: ${obj.arm_state}`);
      if (obj.record_path) details.push(`file: ${obj.record_path}`);
      if (obj.start_band) details.push(`start: ${obj.start_band}`);
      return `
        <div class="status-row">
          <div class="status-key">${label}</div>
          <div class="status-main">
            <span class="chip ${classFor(state)}">${displayState(state)}</span>
            ${details.length ? `<div class="detail">${details.join(' | ')}</div>` : ''}
          </div>
        </div>`;
    };
    const teachMetric = (label, value, options = {}) => `
      <div class="teach-metric ${options.wide ? 'teach-wide' : ''}">
        <div class="label">${escapeHtml(label)}</div>
        <strong>${options.raw ? value : escapeHtml(value)}</strong>
      </div>`;
    const formatJointMap = (positions) => {
      if (!positions || typeof positions !== 'object') return '-';
      const entries = armJointNames
        .filter((name) => Object.prototype.hasOwnProperty.call(positions, name))
        .map((name) => `${name}: ${fmt(Number(positions[name]))}`);
      return entries.length ? entries.join(' | ') : '-';
    };
    const countLabel = (shown, total, truncated) => {
      if (!Number.isFinite(Number(total))) return String(shown || 0);
      return truncated ? `${shown}/${total}` : String(total);
    };
    const renderTeachFileInfo = (info) => {
      if (!info || typeof info !== 'object') {
        return '<div class="empty">Teach record file check unavailable</div>';
      }
      const stateClass = classFor(info.start_band);
      return [
        teachMetric('File Check', `<span class="chip ${stateClass}">${escapeHtml(info.start_band)}</span>`, { raw: true }),
        teachMetric('Samples', Number.isFinite(info.samples) ? info.samples : '-'),
        teachMetric('Duration', Number.isFinite(info.duration_sec) ? `${fmt(info.duration_sec, 1)} s` : '-'),
        teachMetric('Worst Joint', text(info.worst_joint)),
        teachMetric('Max Start Error', Number.isFinite(info.max_error) ? `${fmt(info.max_error)} rad` : '-'),
        teachMetric('Direct Threshold', Number.isFinite(info.direct_threshold) ? `${fmt(info.direct_threshold)} rad` : '-'),
        teachMetric('Align Threshold', Number.isFinite(info.align_threshold) ? `${fmt(info.align_threshold)} rad` : '-'),
        teachMetric('Exists', info.exists ? 'true' : 'false'),
        teachMetric('Record File', text(info.path), { wide: true }),
        teachMetric('Message', text(info.message), { wide: true }),
        teachMetric('Start Positions', formatJointMap(info.start_positions), { wide: true }),
        teachMetric('End Positions', formatJointMap(info.end_positions), { wide: true }),
        teachMetric('Current To Start Error', formatJointMap(info.per_joint_error), { wide: true }),
        teachMetric('Anomalies', Array.isArray(info.anomalies) && info.anomalies.length ? `${info.anomalies.join(' | ')}${info.anomalies_truncated ? ' ...' : ''}` : 'none', { wide: true }),
        teachMetric('Anomaly Count', countLabel(Array.isArray(info.anomalies) ? info.anomalies.length : 0, info.anomalies_total, info.anomalies_truncated)),
        teachMetric('Trajectory Risk', info.quality?.risk_level ? `<span class="chip ${classFor(info.quality.risk_level)}">${escapeHtml(info.quality.risk_level)}</span>` : '-', { raw: true }),
        teachMetric('Max Jump', Number.isFinite(info.quality?.max_jump_rad) ? `${fmt(info.quality.max_jump_rad)} rad` : '-'),
        teachMetric('Max Velocity', Number.isFinite(info.quality?.max_velocity_rad_s) ? `${fmt(info.quality.max_velocity_rad_s)} rad/s` : '-'),
        teachMetric('Replay Policy', text(info.quality?.replay_policy), { wide: true }),
      ].join('');
    };
    const renderTeachRecordSummary = (info, recordingValue) => {
      const recording = statusObj(recordingValue);
      return [
        teachMetric('Recording', `<span class="chip ${classFor(recording.state)}">${escapeHtml(recording.state || 'idle')}</span>`, { raw: true }),
        teachMetric('Samples', Number.isFinite(recording.samples) ? recording.samples : (Number.isFinite(info?.samples) ? info.samples : '-')),
        teachMetric('File', text(recording.record_path || info?.path), { wide: true }),
      ].join('');
    };
    let latestTeachTrajectory = null;
    const renderTeachTrajectoryDetails = (payload) => {
      if (!payload || payload.accepted === false) {
        return `<div class="empty">${escapeHtml(payload?.message || 'Teach trajectory unavailable')}</div>`;
      }
      const q = payload.quality || {};
      const prepared = payload.prepared_replay || {};
      const collision = payload.collision_precheck || {};
      const after = prepared.after_quality || {};
      const raw = prepared.raw_quality || q;
      const filtered = prepared.filtered_quality || {};
      const retimed = prepared.retimed_quality || after;
      const large = prepared.large_motion || {};
      const limitState = (value, limit) => {
        if (!Number.isFinite(Number(value)) || !Number.isFinite(Number(limit))) return '-';
        return Number(value) <= Number(limit) ? 'PASS' : 'BLOCK';
      };
      const limitChip = (state) => state === '-'
        ? '-'
        : `<span class="chip ${state === 'PASS' ? 'pass' : 'red'}">${state}</span>`;
      const limitRows = [
        ['Jump', raw.max_jump_rad, filtered.max_jump_rad, retimed.max_jump_rad, panelConfig.teach?.max_prepared_jump_rad, 'rad'],
        ['Velocity', raw.max_velocity_rad_s, filtered.max_velocity_rad_s, retimed.max_velocity_rad_s, panelConfig.teach?.max_replay_velocity_rad_s, 'rad/s'],
        ['Acceleration', raw.max_acceleration_rad_s2, filtered.max_acceleration_rad_s2, retimed.max_acceleration_rad_s2, panelConfig.teach?.max_replay_acceleration_rad_s2, 'rad/s2'],
        ['Jerk', raw.max_jerk_rad_s3, filtered.max_jerk_rad_s3, retimed.max_jerk_rad_s3, panelConfig.teach?.max_replay_jerk_rad_s3, 'rad/s3'],
      ].map(([name, rawValue, filteredValue, retimedValue, limit, unit]) => {
        const state = limitState(retimedValue, limit);
        return `<tr><td>${name}</td><td>${fmt(Number(rawValue))}</td><td>${fmt(Number(filteredValue))}</td><td>${fmt(Number(retimedValue))}</td><td>${fmt(Number(limit))} ${unit}</td><td>${limitChip(state)}</td></tr>`;
      }).join('');
      const limitsTable = `
        <div class="teach-metric teach-wide">
          <span>Trajectory Limits</span>
          <strong>
            <table class="mini-limit-table">
              <thead><tr><th>Metric</th><th>Raw</th><th>Filtered</th><th>Retimed</th><th>Limit</th><th>State</th></tr></thead>
              <tbody>${limitRows}</tbody>
            </table>
          </strong>
        </div>`;
      return [
        teachMetric('Raw Risk', `<span class="chip ${classFor(raw.risk_level)}">${escapeHtml(raw.risk_level || '-')}</span>`, { raw: true }),
        teachMetric('Filtered Risk', filtered.risk_level ? `<span class="chip ${classFor(filtered.risk_level)}">${escapeHtml(filtered.risk_level)}</span>` : '-', { raw: true }),
        teachMetric('Retimed Risk', retimed.risk_level ? `<span class="chip ${classFor(retimed.risk_level)}">${escapeHtml(retimed.risk_level)}</span>` : '-', { raw: true }),
        teachMetric('Raw Samples', Number.isFinite(payload.raw_samples) ? payload.raw_samples : '-'),
        teachMetric('Shown Samples', Number.isFinite(payload.returned_samples) ? payload.returned_samples : '-'),
        teachMetric('Prepared Samples', Number.isFinite(prepared.prepared_samples) ? prepared.prepared_samples : '-'),
        teachMetric('Retimed Points', Number.isFinite(prepared.retimed_points) ? prepared.retimed_points : '-'),
        teachMetric('Duration', Number.isFinite(payload.duration_sec) ? `${fmt(payload.duration_sec, 2)} s` : '-'),
        teachMetric('Raw Max Jump', Number.isFinite(raw.max_jump_rad) ? `${fmt(raw.max_jump_rad)} rad` : '-'),
        teachMetric('Retimed Max Jump', Number.isFinite(retimed.max_jump_rad) ? `${fmt(retimed.max_jump_rad)} rad` : '-'),
        teachMetric('Retimed Max Acc', Number.isFinite(retimed.max_acceleration_rad_s2) ? `${fmt(retimed.max_acceleration_rad_s2)} rad/s2` : '-'),
        teachMetric('Retimed Max Jerk', Number.isFinite(retimed.max_jerk_rad_s3) ? `${fmt(retimed.max_jerk_rad_s3)} rad/s3` : '-'),
        teachMetric('Large Motion', large.enabled ? 'true' : 'false'),
        teachMetric('Effective Speed', Number.isFinite(large.effective_speed) ? `${fmt(large.effective_speed, 2)} x` : '-'),
        teachMetric('Collision', collision.state ? `<span class="chip ${classFor(collision.state)}">${escapeHtml(collision.state)}</span>` : '-', { raw: true }),
        teachMetric('Collision Samples', Number.isFinite(collision.checked_samples) ? `${collision.checked_samples}/${collision.requested_samples || '-'}` : '-'),
        teachMetric('Jump Gate', Number.isFinite(panelConfig.teach?.max_prepared_jump_rad) ? `${fmt(panelConfig.teach.max_prepared_jump_rad)} rad` : '-'),
        teachMetric('Accel Gate', Number.isFinite(panelConfig.teach?.max_replay_acceleration_rad_s2) ? `${fmt(panelConfig.teach.max_replay_acceleration_rad_s2)} rad/s2` : '-'),
        teachMetric('Jerk Gate', Number.isFinite(panelConfig.teach?.max_replay_jerk_rad_s3) ? `${fmt(panelConfig.teach.max_replay_jerk_rad_s3)} rad/s3` : '-'),
        teachMetric('Worst Sample', Number.isFinite(q.worst_sample) ? q.worst_sample : '-'),
        teachMetric('Worst Joint', text(q.worst_joint)),
        teachMetric('Smoothing', prepared.smoothing_applied ? `window ${prepared.smoothing_window}` : 'off'),
        teachMetric('Filter', prepared.filter_applied ? `${fmt(prepared.filter_cutoff_hz, 1)} Hz` : 'off'),
        teachMetric('Resampling', prepared.resample_applied ? `${fmt(prepared.resample_rate_hz, 1)} Hz` : 'off'),
        teachMetric('Policy', text(q.replay_policy), { wide: true }),
        limitsTable,
      ].join('');
    };
    const drawTeachTrajectoryChart = (payload) => {
      const canvas = document.getElementById('teach-trajectory-canvas');
      if (!canvas) return;
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.max(320, Math.floor(rect.width * dpr));
      canvas.height = Math.max(220, Math.floor(rect.height * dpr));
      const ctx = canvas.getContext('2d');
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      const width = canvas.width / dpr;
      const height = canvas.height / dpr;
      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = '#ffffff';
      ctx.fillRect(0, 0, width, height);
      ctx.strokeStyle = '#d8dee6';
      ctx.lineWidth = 1;
      const left = 44, right = 12, top = 16, bottom = 28;
      ctx.strokeRect(left, top, width - left - right, height - top - bottom);
      const points = Array.isArray(payload?.points) ? payload.points : [];
      const joints = Array.isArray(payload?.joint_names) ? payload.joint_names : [];
      if (!points.length || !joints.length) {
        ctx.fillStyle = '#667085';
        ctx.fillText('No trajectory samples', left + 12, top + 24);
        return;
      }
      const tMax = Math.max(...points.map((p) => Number(p.t) || 0), 0.001);
      const values = [];
      points.forEach((point) => joints.forEach((joint) => {
        const value = Number(point.positions?.[joint]);
        if (Number.isFinite(value)) values.push(value);
      }));
      const yMin = Math.min(...values, -0.1);
      const yMax = Math.max(...values, 0.1);
      const ySpan = Math.max(yMax - yMin, 0.001);
      const xFor = (t) => left + ((Number(t) || 0) / tMax) * (width - left - right);
      const yFor = (v) => top + (1 - ((Number(v) - yMin) / ySpan)) * (height - top - bottom);
      const colors = ['#2563a8', '#188a4d', '#b7791f', '#c24135', '#6b46c1', '#0f766e'];
      const drawSeries = (seriesPoints, alpha = 1.0, widthScale = 1.5) => {
        joints.forEach((joint, jointIndex) => {
          ctx.beginPath();
          ctx.strokeStyle = colors[jointIndex % colors.length];
          ctx.globalAlpha = alpha;
          ctx.lineWidth = widthScale;
          seriesPoints.forEach((point, index) => {
            const x = xFor(point.t);
            const y = yFor(point.positions?.[joint]);
            if (index === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
          });
          ctx.stroke();
        });
        ctx.globalAlpha = 1.0;
      };
      drawSeries(points, 0.35, 1.0);
      const preparedPoints = Array.isArray(payload.prepared_points) ? payload.prepared_points : [];
      if (preparedPoints.length) {
        drawSeries(preparedPoints, 1.0, 1.8);
      }
      (payload.events || []).forEach((event) => {
        const point = points.find((item) => item.sample === event.sample);
        if (!point) return;
        const x = xFor(point.t);
        ctx.strokeStyle = event.level === 'red' ? '#c24135' : '#b7791f';
        ctx.beginPath();
        ctx.moveTo(x, top);
        ctx.lineTo(x, height - bottom);
        ctx.stroke();
      });
      ctx.fillStyle = '#667085';
      ctx.fillText(`${fmt(yMax)} rad`, 6, top + 8);
      ctx.fillText(`${fmt(yMin)} rad`, 6, height - bottom);
      ctx.fillText(`${fmt(tMax, 2)} s`, width - right - 42, height - 8);
    };
    const previewTeachTrajectoryFrame = (index) => {
      const points = Array.isArray(latestTeachTrajectory?.points) ? latestTeachTrajectory.points : [];
      const point = points[Math.min(Math.max(Number(index) || 0, 0), Math.max(points.length - 1, 0))];
      if (!point || !point.positions) return;
      const joints = {};
      Object.entries(point.positions).forEach(([name, position]) => {
        joints[name] = { position: Number(position) };
      });
      updateRobotViewer(joints);
    };
    const loadTeachTrajectory = async () => {
      try {
        const suffix = selectedTeachRecordPath ? `?path=${encodeURIComponent(selectedTeachRecordPath)}&max_points=500` : '?max_points=500';
        const response = await fetch(`/api/teach_trajectory${suffix}`);
        const payload = await response.json();
        latestTeachTrajectory = payload;
        document.getElementById('teach-trajectory-details').innerHTML = renderTeachTrajectoryDetails(payload);
        drawTeachTrajectoryChart(payload);
        const frame = document.getElementById('teach-trajectory-frame');
        const count = Array.isArray(payload.points) ? payload.points.length : 0;
        frame.max = Math.max(count - 1, 0);
        frame.value = 0;
        frame.disabled = count === 0;
      } catch (error) {
        document.getElementById('teach-trajectory-details').innerHTML =
          `<div class="empty">Teach trajectory load failed: ${escapeHtml(error.message)}</div>`;
      }
    };
    const renderReplayPrecheckSummary = (info) => {
      const band = String(info?.start_band || 'unknown').toLowerCase();
      const quality = info?.quality || {};
      const risk = String(quality.risk_level || '-').toLowerCase();
      const estimate = replayEstimate(info);
      const replay = statusObj(previewState.latestTeleop?.replay);
      const moveit = replay.moveit_start_align || {};
      const collision = replay.collision_precheck || {};
      return [
        teachMetric('Start', `<span class="chip ${classFor(band)}">${escapeHtml(band || 'unknown')}</span>`, { raw: true }),
        teachMetric('Risk', `<span class="chip ${classFor(risk)}">${escapeHtml(risk || '-')}</span>`, { raw: true }),
        teachMetric('MoveIt', moveit.state ? `<span class="chip ${classFor(moveit.state)}">${escapeHtml(moveit.state)}</span>` : '-', { raw: true }),
        teachMetric('Collision', collision.state ? `<span class="chip ${classFor(collision.state)}">${escapeHtml(collision.state)}</span>` : '-', { raw: true }),
        teachMetric('Start Error', Number.isFinite(info?.max_error) ? `${fmt(info.max_error)} rad` : '-'),
        teachMetric('Worst Joint', text(info?.worst_joint)),
        teachMetric('Max Jump', Number.isFinite(quality.max_jump_rad) ? `${fmt(quality.max_jump_rad)} rad` : '-'),
        teachMetric('Max Velocity', Number.isFinite(quality.max_velocity_rad_s) ? `${fmt(quality.max_velocity_rad_s)} rad/s` : '-'),
        teachMetric('Estimated Time', `${fmt(estimate.estimatedDuration, 1)} s`),
        teachMetric('Points', estimate.trajectoryPoints),
      ].join('');
    };
    const replayEstimate = (info) => {
      const speed = Math.min(Math.max(Number(teachReplaySettings.replaySpeed), 0.1), 3.0);
      const alignDuration = Math.min(Math.max(Number(teachReplaySettings.alignDuration), 1.0), 10.0);
      const alignSteps = Math.min(Math.max(Math.round(Number(teachReplaySettings.alignSteps)), 2), 200);
      const finalHold = Math.min(Math.max(Number(teachReplaySettings.finalHold), 0.0), 5.0);
      const useAlign = String(info?.start_band || '').toLowerCase() === 'align';
      const recordDuration = Number(info?.duration_sec);
      const replayDuration = Number.isFinite(recordDuration) ? recordDuration / Math.max(speed, 0.01) : NaN;
      return {
        speed,
        alignDuration,
        alignSteps,
        finalHold,
        useAlign,
        estimatedDuration: (Number.isFinite(replayDuration) ? replayDuration : 0) + (useAlign ? alignDuration : 0) + finalHold,
        trajectoryPoints: (Number(info?.samples) || 0) + (useAlign ? alignSteps : 0) + ((Number(info?.samples) || 0) > 0 && finalHold > 0 ? 1 : 0),
      };
    };
    const replaySettingsPayload = () => ({
      replay_speed: Math.min(Math.max(Number(teachReplaySettings.replaySpeed), 0.1), 3.0),
      align_duration: Math.min(Math.max(Number(teachReplaySettings.alignDuration), 1.0), 10.0),
      align_steps: Math.min(Math.max(Math.round(Number(teachReplaySettings.alignSteps)), 2), 200),
      final_hold_sec: Math.min(Math.max(Number(teachReplaySettings.finalHold), 0.0), 5.0),
    });
    const renderReplayParams = (info) => {
      const estimate = replayEstimate(info);
      return [
        teachMetric('Hardware Mode', panelConfig.teach?.use_hardware ? 'true' : 'false'),
        teachMetric('Panel Mode', panelMode),
        teachMetric('Web Execute', webExecuteEnabled ? 'enabled' : 'disabled'),
        teachMetric('Replay Speed', `${fmt(estimate.speed, 1)} x`),
        teachMetric('Direct Threshold', `${fmt(Number(panelConfig.teach?.direct_threshold), 4)} rad`),
        teachMetric('Align Threshold', `${fmt(Number(panelConfig.teach?.align_threshold), 4)} rad`),
        teachMetric('Align Duration', `${fmt(estimate.alignDuration, 1)} s`),
        teachMetric('Align Steps', estimate.alignSteps),
        teachMetric('Final Hold', `${fmt(estimate.finalHold, 1)} s`),
        teachMetric('Will Align', estimate.useAlign ? 'true' : 'false'),
        teachMetric('Estimated Duration', `${fmt(estimate.estimatedDuration, 1)} s`),
        teachMetric('Estimated Points', estimate.trajectoryPoints),
        teachMetric('Execution Gate', isCheckMode ? 'check mode is read-only' : (webExecuteEnabled ? 'real replay button allowed after dry-run' : 'real replay hidden by web_execute_enabled=false'), { wide: true }),
      ].join('');
    };
    const addReplayEvent = (message) => {
      const now = new Date().toLocaleTimeString();
      replayEvents.unshift(`${now} ${message}`);
      replayEvents.splice(8);
      if (!isDetailsOpen('replay-event-log-panel')) return;
      document.getElementById('replay-event-log').innerHTML = replayEvents.length
        ? replayEvents.map((item) => `<div class="event-item">${escapeHtml(item)}</div>`).join('')
        : '<div class="empty">No replay events yet</div>';
    };
    let latestStatusData = null;
    const renderOptionalDetails = (data) => {
      if (!data) return;
      if (isDetailsOpen('replay-event-log-panel')) {
        document.getElementById('replay-event-log').innerHTML = replayEvents.length
          ? replayEvents.map((item) => `<div class="event-item">${escapeHtml(item)}</div>`).join('')
          : '<div class="empty">No replay events yet</div>';
      }
    };
    const attachDetailsUnloaders = () => {
      [
        ['replay-event-log-panel', ['replay-event-log']],
      ].forEach(([panelId, bodyIds]) => {
        const panel = document.getElementById(panelId);
        if (!panel) return;
        panel.addEventListener('toggle', () => {
          if (panel.open) {
            renderOptionalDetails(latestStatusData);
          } else {
            bodyIds.forEach((id) => clearOptionalDetails(id));
          }
        });
      });
    };
    const updateTeachDryRunButton = (info) => {
      const band = String(info?.start_band || '').toLowerCase();
      const button = document.getElementById('run-teach-dry-run');
      button.disabled = isCheckMode || !['direct', 'align', 'moveit_align'].includes(band);
      const replayButton = document.getElementById('run-teach-replay');
      const stopButton = document.getElementById('stop-teach-replay');
      const dryRunMatches = lastTeachDryRun?.accepted === true &&
        lastTeachDryRun?.record_path === info?.path &&
        lastTeachDryRun?.start_band === info?.start_band &&
        Number(lastTeachDryRun?.max_error ?? NaN) === Number(info?.max_error ?? NaN);
      replayButton.disabled = isCheckMode || !webExecuteEnabled || !dryRunMatches || !['direct', 'align', 'moveit_align'].includes(band);
      const replayState = String(statusObj(previewState.latestTeleop?.replay).state || '').toLowerCase();
      stopButton.disabled = !['replaying', 'cancel_requested'].includes(replayState);
    };
    const refreshTeachFileInfo = async (options = {}) => {
      if (!options.force && !isControlCardOpen('teach-trajectory-card')) return;
      try {
        const suffix = selectedTeachRecordPath ? `?path=${encodeURIComponent(selectedTeachRecordPath)}` : '';
        const response = await fetch(`/api/teach_record_info${suffix}`);
        const info = await response.json();
        latestTeachFileInfo = info;
        setHtml('replay-precheck-summary', renderReplayPrecheckSummary(info));
        setHtml('teach-file-details', renderTeachFileInfo(info));
        setHtml('teach-record-summary', renderTeachRecordSummary(info, previewState.latestTeleop?.recording));
        setHtml('teach-params-details', renderReplayParams(info));
        updateTeachDryRunButton(info);
      } catch (error) {
        setHtml('replay-precheck-summary', `<div class="empty">Precheck unavailable: ${escapeHtml(error.message)}</div>`);
        setHtml('teach-file-details', `<div class="empty">Teach record file check failed: ${escapeHtml(error.message)}</div>`);
        document.getElementById('run-teach-dry-run').disabled = true;
        document.getElementById('run-teach-replay').disabled = true;
        document.getElementById('stop-teach-replay').disabled = true;
        setHtml('teach-params-details', `<div class="empty">Teach replay parameters unavailable: ${escapeHtml(error.message)}</div>`);
      }
    };
    const refreshTeachRecords = async () => {
      try {
        const response = await fetch('/api/teach_records');
        const payload = await response.json();
        const select = document.getElementById('teach-record-select');
        const current = selectedTeachRecordPath || '';
        const records = (payload.records || []).slice(0, 10);
        select.innerHTML = '<option value="">Default file</option>' +
          records.map((record) => `<option value="${escapeHtml(record.path)}">${escapeHtml(record.name || record.path)}</option>`).join('');
        select.value = current;
      } catch (error) {
        document.getElementById('teach-file-details').innerHTML =
          `<div class="empty">Teach record list failed: ${escapeHtml(error.message)}</div>`;
      }
    };
    const selectTeachRecord = async (path) => {
      selectedTeachRecordPath = path || '';
      const selectedName = selectedTeachRecordPath.split('/').pop() || selectedTeachRecordPath.split('\\\\').pop();
      if (selectedName) {
        document.getElementById('teach-record-name').value = selectedName;
      }
      lastTeachDryRun = null;
      document.getElementById('teach-dry-run-status').textContent =
        selectedTeachRecordPath
          ? `Selected teach record: ${selectedTeachRecordPath}`
          : 'Using default record_path. Run dry-run before real replay.';
      addReplayEvent(selectedTeachRecordPath ? `selected record: ${selectedTeachRecordPath}` : 'selected default record_path');
      await refreshTeachFileInfo();
    };
    const previewState = {
      active: false,
      targets: {},
      latestJoints: {},
      executing: false,
      latestTeleop: {},
    };
    const executeSettings = {
      maxDelta: 0.8,
      duration: 3.0,
      maxSpeed: Number(panelConfig.web_execute?.max_joint_speed_rad_s) || 1.5,
    };
    const executeMaxDeltaLimit = Number(panelConfig.web_execute?.max_delta_rad) || 1.5;
    const gripperSettings = {
      maxEffort: Number(panelConfig.web_gripper?.max_effort) || 0.3,
    };
    const keyboardSettings = {
      enabled: false,
      stepRad: Number(panelConfig.web_keyboard?.step_rad) || 0.02,
      minStepRad: Number(panelConfig.web_keyboard?.min_step_rad) || 0.005,
      maxStepRad: Number(panelConfig.web_keyboard?.max_step_rad) || 0.1,
      duration: Number(panelConfig.web_keyboard?.duration) || 0.2,
      minDuration: Number(panelConfig.web_keyboard?.min_duration) || 0.1,
      maxDuration: Number(panelConfig.web_keyboard?.max_duration) || 2.0,
      maxSpeed: Number(panelConfig.web_keyboard?.max_joint_speed_rad_s) || 0.5,
    };
    const teachReplaySettings = {
      replaySpeed: Number(panelConfig.teach?.replay_speed) || 1.0,
      alignDuration: Number(panelConfig.teach?.align_duration) || 3.0,
      alignSteps: Number(panelConfig.teach?.align_steps) || 30,
      finalHold: Number.isFinite(Number(panelConfig.teach?.final_hold_sec)) ? Number(panelConfig.teach.final_hold_sec) : 1.0,
    };
    const clamp = (value, min, max) => Math.min(Math.max(Number(value), min), max);
    const makeSlider = (name, mode = 'live') => {
      const [min, max] = jointLimits[name] || [-3.1416, 3.1416];
      const isGripper = name === 'gripper';
      const isPreview = mode === 'preview';
      const prefix = isPreview ? 'preview' : 'slider';
      return `
        <div class="joint-slider" id="${prefix}-row-${name}">
          <div class="joint-name">${name}</div>
          <input id="${prefix}-${name}" type="range" min="${min}" max="${max}" step="0.0001" value="0" ${isPreview ? '' : 'disabled'}>
          <div class="joint-values">
            <strong id="${prefix}-rad-${name}">-</strong> ${isGripper ? 'm' : 'rad'}
            <span id="${prefix}-deg-wrap-${name}">${isGripper ? '' : ' / <span id="' + prefix + '-deg-' + name + '">-</span> deg'}</span>
            <div class="joint-limit">${Number(min).toFixed(2)} .. ${Number(max).toFixed(2)}</div>
          </div>
        </div>`;
    };
    document.getElementById('joint-sliders').innerHTML = sliderNames.map(makeSlider).join('');
    document.getElementById('preview-sliders').innerHTML = sliderNames.map((name) => makeSlider(name, 'preview')).join('');
    const robotViewer = {
      scene: null,
      camera: null,
      renderer: null,
      controls: null,
      grid: null,
      robot: null,
      gripperVisuals: null,
      ready: false,
      renderPending: false,
    };
    const objectNamePath = (object) => {
      const names = [];
      let cursor = object;
      while (cursor) {
        if (cursor.name) names.push(String(cursor.name).toLowerCase());
        cursor = cursor.parent;
      }
      return names.join('/');
    };
    const markGripperDebug = (gripperVisuals, opening = 0) => {
      window.__rebotGripperDebug = {
        leftCount: gripperVisuals.left.length,
        rightCount: gripperVisuals.right.length,
        leftNames: gripperVisuals.left.map(({ object }) => objectNamePath(object)),
        rightNames: gripperVisuals.right.map(({ object }) => objectNamePath(object)),
        lastOpening: opening,
        lastLeftY: gripperVisuals.left.map(({ object }) => object.position.y),
        lastRightY: gripperVisuals.right.map(({ object }) => object.position.y),
      };
      document.body.dataset.gripperLeftCount = String(gripperVisuals.left.length);
      document.body.dataset.gripperRightCount = String(gripperVisuals.right.length);
      document.body.dataset.gripperOpening = String(opening);
      document.body.dataset.gripperLeftY = gripperVisuals.left.map(({ object }) => object.position.y.toFixed(6)).join(',');
      document.body.dataset.gripperRightY = gripperVisuals.right.map(({ object }) => object.position.y.toFixed(6)).join(',');
    };
    const addGripperVisual = (gripperVisuals, side, object) => {
      if (!object) return;
      const list = gripperVisuals[side];
      if (list.some((entry) => entry.object === object)) return;
      list.push({ object, base: object.position.clone() });
    };
    const namedRobotObject = (robot, name) => {
      return robot?.visual?.[name] || robot?.frames?.[name] || robot?.links?.[name] || null;
    };
    const viewerStatus = (message) => {
      const el = document.getElementById('robot-view-status');
      if (el) el.textContent = message;
    };
    const renderRobotFrame = () => {
      robotViewer.renderPending = false;
      if (!robotViewer.renderer || !robotViewer.scene || !robotViewer.camera) return;
      robotViewer.controls?.update();
      robotViewer.renderer.render(robotViewer.scene, robotViewer.camera);
    };
    const scheduleRobotRender = () => {
      if (robotViewer.renderPending) return;
      robotViewer.renderPending = true;
      requestAnimationFrame(renderRobotFrame);
    };
    const initRobotViewer = async () => {
      const canvas = document.getElementById('robot-view');
      const scene = new THREE.Scene();
      scene.background = new THREE.Color(0x101820);
      const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 10);
      camera.position.set(0.55, -0.75, 0.45);
      const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
      renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
      const controls = new OrbitControls(camera, renderer.domElement);
      controls.target.set(0, 0, 0.18);
      controls.update();
      controls.addEventListener('change', scheduleRobotRender);
      scene.add(new THREE.HemisphereLight(0xffffff, 0x324055, 1.9));
      const keyLight = new THREE.DirectionalLight(0xffffff, 1.2);
      keyLight.position.set(0.5, -0.6, 0.8);
      scene.add(keyLight);
      const grid = new THREE.GridHelper(0.8, 16, 0x5b6775, 0x27313c);
      grid.rotation.x = Math.PI / 2;
      grid.visible = false;
      scene.add(grid);
      robotViewer.grid = grid;
      robotViewer.scene = scene;
      robotViewer.camera = camera;
      robotViewer.renderer = renderer;
      robotViewer.controls = controls;

      const loader = new URDFLoader();
      const robot = await new Promise((resolve, reject) => {
        loader.load('/robot/urdf', resolve, undefined, reject);
      });
      robot.rotation.x = -Math.PI / 2;
      robot.traverse((object) => {
        if (object.isMesh) {
          object.castShadow = false;
          object.receiveShadow = false;
          if (object.material) {
            object.material.color?.set(0xaeb7c2);
            object.material.roughness = 0.62;
            object.material.metalness = 0.12;
            object.material.needsUpdate = true;
          }
        }
      });
      const gripperVisuals = {
        left: [],
        right: [],
      };
      addGripperVisual(gripperVisuals, 'left', namedRobotObject(robot, 'left_finger'));
      addGripperVisual(gripperVisuals, 'right', namedRobotObject(robot, 'right_finger'));
      if (!gripperVisuals.left.length || !gripperVisuals.right.length) {
        robot.traverse((object) => {
          const name = objectNamePath(object);
          if (!gripperVisuals.left.length && name.includes('left_finger')) addGripperVisual(gripperVisuals, 'left', object);
          if (!gripperVisuals.right.length && name.includes('right_finger')) addGripperVisual(gripperVisuals, 'right', object);
        });
      }
      scene.add(robot);
      robotViewer.robot = robot;
      robotViewer.gripperVisuals = gripperVisuals;
      markGripperDebug(gripperVisuals);
      const resize = () => {
        const rect = canvas.getBoundingClientRect();
        renderer.setSize(rect.width, rect.height, false);
        camera.aspect = rect.width / Math.max(rect.height, 1);
        camera.updateProjectionMatrix();
        scheduleRobotRender();
      };
      window.addEventListener('resize', resize);
      resize();
      robotViewer.ready = true;
      scheduleRobotRender();
      viewerStatus('3D view live: following joint_states and gripper state');
    };
    const jointsFromTargets = (targets) => {
      const result = {};
      sliderNames.forEach((name) => {
        if (Number.isFinite(Number(targets[name]))) {
          result[name] = { position: Number(targets[name]) };
        }
      });
      return result;
    };
    const updateRobotViewer = (joints) => {
      if (!robotViewer.ready) return;
      armJointNames.forEach((name) => {
        const state = joints[name];
        const joint = robotViewer.robot?.joints?.[name];
        const position = state && Number.isFinite(Number(state.position)) ? Number(state.position) : NaN;
        if (!joint || !Number.isFinite(position)) return;
        joint.setJointValue(position);
      });
      const gripper = joints.gripper;
      const gripperPosition = gripper && Number.isFinite(Number(gripper.position)) ? Number(gripper.position) : NaN;
      if (Number.isFinite(gripperPosition) && robotViewer.gripperVisuals) {
        const [closed, open] = jointLimits.gripper || [0.0, 0.09];
        const opening = clamp(gripperPosition, closed, open) - Number(closed);
        const halfTravel = opening * 0.5;
        robotViewer.gripperVisuals.left.forEach(({ object, base }) => {
          object.position.copy(base);
          object.position.y += halfTravel;
        });
        robotViewer.gripperVisuals.right.forEach(({ object, base }) => {
          object.position.copy(base);
          object.position.y -= halfTravel;
        });
        markGripperDebug(robotViewer.gripperVisuals, opening);
      }
      scheduleRobotRender();
    };
    initRobotViewer().catch((error) => viewerStatus(`3D view unavailable: ${error.message}`));
    document.getElementById('toggle-grid').addEventListener('click', () => {
      if (!robotViewer.grid) return;
      robotViewer.grid.visible = !robotViewer.grid.visible;
      scheduleRobotRender();
    });
    const updateSlider = (name, joint, mode = 'live') => {
      const prefix = mode === 'preview' ? 'preview' : 'slider';
      const slider = document.getElementById(`${prefix}-${name}`);
      const radLabel = document.getElementById(`${prefix}-rad-${name}`);
      const degLabel = document.getElementById(`${prefix}-deg-${name}`);
      const position = joint && Number.isFinite(Number(joint.position)) ? Number(joint.position) : NaN;
      if (!Number.isFinite(position)) {
        radLabel.textContent = '-';
        if (degLabel) degLabel.textContent = '-';
        return;
      }
      const [min, max] = jointLimits[name] || [-3.1416, 3.1416];
      slider.min = min;
      slider.max = max;
      slider.value = clamp(position, min, max);
      radLabel.textContent = fmt(position);
      if (degLabel) degLabel.textContent = deg(position);
    };
    const syncPreviewFromLive = () => {
      sliderNames.forEach((name) => {
        const joint = previewState.latestJoints[name];
        const position = joint && Number.isFinite(Number(joint.position)) ? Number(joint.position) : 0;
        const [min, max] = jointLimits[name] || [-3.1416, 3.1416];
        previewState.targets[name] = clamp(position, min, max);
        updateSlider(name, { position: previewState.targets[name] }, 'preview');
      });
      previewState.active = false;
      document.getElementById('web-teleop-card').classList.remove('preview-active');
      document.getElementById('execute-preview').disabled = true;
      document.getElementById('stop-execute').disabled = true;
      document.getElementById('execute-status').textContent = 'Preview synced to live state. Move a target slider before executing.';
      updateRobotViewer(previewState.latestJoints);
      viewerStatus('3D view live: following joint_states and gripper state');
    };
    const updateMotorRows = (joints) => {
      const tbody = document.getElementById('joints');
      const entries = Object.entries(joints || {}).sort();
      if (!entries.length) {
        tbody.innerHTML = '<tr><td colspan="5" class="empty">Waiting for joint state</td></tr>';
        motorRowsByName.clear();
        return;
      }
      const liveNames = new Set(entries.map(([name]) => name));
      motorRowsByName.forEach((row, name) => {
        if (!liveNames.has(name)) {
          row.remove();
          motorRowsByName.delete(name);
        }
      });
      if (tbody.querySelector('.empty')) {
        tbody.innerHTML = '';
        motorRowsByName.clear();
      }
      entries.forEach(([name, joint]) => {
        let row = motorRowsByName.get(name);
        if (!row) {
          row = document.createElement('tr');
          row.innerHTML = '<td></td><td></td><td></td><td></td><td></td>';
          tbody.appendChild(row);
          motorRowsByName.set(name, row);
        }
        const cells = row.children;
        cells[0].textContent = name;
        cells[1].textContent = fmt(joint.position);
        cells[2].textContent = fmt(joint.velocity);
        cells[3].textContent = fmt(joint.torque);
        cells[4].textContent = joint.status_code ?? '-';
      });
    };
    const previewArmTargets = () => {
      const result = {};
      const missing = [];
      armJointNames.forEach((name) => {
        const target = Number(previewState.targets[name]);
        const live = Number(previewState.latestJoints[name]?.position);
        if (Number.isFinite(target)) {
          result[name] = target;
        } else if (Number.isFinite(live)) {
          result[name] = live;
        } else {
          missing.push(name);
        }
      });
      return { result, missing };
    };
    const maxPreviewDelta = () => {
      let maxDelta = 0;
      armJointNames.forEach((name) => {
        const live = previewState.latestJoints[name]?.position;
        const target = previewState.targets[name];
        if (Number.isFinite(Number(live)) && Number.isFinite(Number(target))) {
          maxDelta = Math.max(maxDelta, Math.abs(Number(target) - Number(live)));
        }
      });
      return maxDelta;
    };
    const speedLimitForJoint = (name) => {
      const configured = Number(panelConfig.joint_velocity_limits?.[name]);
      return Number.isFinite(configured) ? Math.min(configured, executeSettings.maxSpeed) : executeSettings.maxSpeed;
    };
    const speedCheck = () => {
      let worst = { name: '', speed: 0, limit: executeSettings.maxSpeed, minDuration: 0 };
      armJointNames.forEach((name) => {
        const live = Number(previewState.latestJoints[name]?.position);
        const target = Number(previewState.targets[name]);
        if (!Number.isFinite(live) || !Number.isFinite(target)) return;
        const limit = speedLimitForJoint(name);
        const distance = Math.abs(target - live);
        const speed = distance / Math.max(executeSettings.duration, 1e-6);
        if (speed > worst.speed) {
          worst = { name, speed, limit, minDuration: distance / Math.max(limit, 1e-6) };
        }
      });
      return worst;
    };
    const bindExecuteSetting = (inputId, key, min, max) => {
      const input = document.getElementById(inputId);
      const update = (value) => {
        const clamped = clamp(value, min, max);
        executeSettings[key] = clamped;
        input.value = clamped.toFixed(key === 'duration' ? 1 : 2);
      };
      input.addEventListener('change', (event) => update(event.target.value));
      update(executeSettings[key]);
    };
    const executePreview = async (options = {}) => {
      if (!previewState.active || previewState.executing) return { accepted: false, message: 'preview is not active' };
      const delta = maxPreviewDelta();
      const targets = previewArmTargets();
      if (targets.missing.length) {
        document.getElementById('execute-status').textContent = `Cannot execute: missing live joint state for ${targets.missing.join(', ')}`;
        return { accepted: false, message: 'missing live joint state' };
      }
      const speed = speedCheck();
      if (speed.speed > speed.limit) {
        document.getElementById('execute-status').textContent =
          `${speed.name} speed too high: ${fmt(speed.speed, 3)} rad/s > ${fmt(speed.limit, 3)} rad/s. Use duration >= ${fmt(speed.minDuration, 2)}s.`;
        return { accepted: false, message: 'speed too high' };
      }
      if (options.confirm !== false) {
        const confirmed = window.confirm(
          `Send preview target to real arm?\n` +
          `Target delta: ${fmt(delta)} rad\n` +
          `Allowed delta: ${fmt(executeSettings.maxDelta)} rad\n` +
          `Duration: ${fmt(executeSettings.duration, 2)} s\n` +
          `Speed limit: ${fmt(executeSettings.maxSpeed, 2)} rad/s`
        );
        if (!confirmed) return { accepted: false, message: 'canceled' };
      }
      previewState.executing = true;
      const button = document.getElementById('execute-preview');
      const stopButton = document.getElementById('stop-execute');
      const status = document.getElementById('execute-status');
      button.disabled = true;
      stopButton.disabled = false;
      status.textContent = 'Sending preview target...';
      try {
        const response = await fetch('/api/execute_preview', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            confirm: 'EXECUTE',
            joint_positions: targets.result,
            max_delta_rad: executeSettings.maxDelta,
            max_joint_speed_rad_s: executeSettings.maxSpeed,
            duration: executeSettings.duration,
          }),
        });
        const result = await response.json();
        status.textContent = result.message || (result.accepted ? 'Accepted' : 'Rejected');
        if (!result.accepted) {
          button.disabled = false;
          stopButton.disabled = true;
        }
        return result;
      } catch (error) {
        status.textContent = `Execute request failed: ${error.message}`;
        button.disabled = false;
        stopButton.disabled = true;
        return { accepted: false, message: error.message };
      } finally {
        previewState.executing = false;
      }
    };
    const stopExecute = async () => {
      const stopButton = document.getElementById('stop-execute');
      const status = document.getElementById('execute-status');
      stopButton.disabled = true;
      status.textContent = 'Requesting stop...';
      try {
        const response = await fetch('/api/stop_execute', { method: 'POST' });
        const result = await response.json();
        status.textContent = result.message || (result.accepted ? 'Stop requested' : 'Stop rejected');
        if (!result.accepted && result.state !== 'idle') {
          stopButton.disabled = false;
        }
      } catch (error) {
        status.textContent = `Stop request failed: ${error.message}`;
        stopButton.disabled = false;
      }
    };
    const setGripper = async (options = {}) => {
      const target = Number(previewState.targets.gripper);
      const live = Number(previewState.latestJoints.gripper?.position);
      const position = Number.isFinite(target) ? target : live;
      if (!Number.isFinite(position)) {
        document.getElementById('execute-status').textContent = 'Cannot set gripper: missing gripper state';
        return { accepted: false, message: 'missing gripper state' };
      }
      if (options.confirm !== false) {
        const confirmed = window.confirm(`Set gripper target?\nPosition: ${fmt(position)} m`);
        if (!confirmed) return { accepted: false, message: 'canceled' };
      }
      const button = document.getElementById('execute-preview');
      const status = document.getElementById('execute-status');
      button.disabled = true;
      status.textContent = 'Sending gripper target...';
      try {
        const response = await fetch('/api/set_gripper', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            confirm: 'SET_GRIPPER',
            position,
            max_effort: gripperSettings.maxEffort,
          }),
        });
        const result = await response.json();
        status.textContent = result.message || (result.accepted ? 'Gripper target accepted' : 'Gripper target rejected');
        if (!result.accepted) {
          button.disabled = false;
        }
        return result;
      } catch (error) {
        status.textContent = `Set gripper request failed: ${error.message}`;
        button.disabled = false;
        return { accepted: false, message: error.message };
      }
    };
    const executePreviewAndGripper = async () => {
      const gripperTarget = Number(previewState.targets.gripper);
      const gripperLive = Number(previewState.latestJoints.gripper?.position);
      const gripperPosition = Number.isFinite(gripperTarget) ? gripperTarget : gripperLive;
      const armDelta = maxPreviewDelta();
      const confirmed = window.confirm(
        `Execute preview target?\n` +
        `Arm and gripper commands are sent through existing safe APIs.\n` +
        `Max joint delta: ${fmt(armDelta)} rad\n` +
        `Duration: ${fmt(executeSettings.duration, 2)} s\n` +
        `Gripper: ${Number.isFinite(gripperPosition) ? fmt(gripperPosition) + ' m' : 'unavailable'}`
      );
      if (!confirmed) return;
      if (armDelta > 1e-5) {
        const armResult = await executePreview({ confirm: false });
        if (armResult && armResult.accepted === false) return;
      }
      if (Number.isFinite(gripperPosition) && webExecuteEnabled) {
        await setGripper({ confirm: false });
      }
    };
    const runTeachDryRun = async () => {
      const button = document.getElementById('run-teach-dry-run');
      const status = document.getElementById('teach-dry-run-status');
      const band = String(latestTeachFileInfo?.start_band || '').toLowerCase();
      if (isCheckMode) {
        status.textContent = 'Check mode uses automatic system dry-run; this page is read-only.';
        return;
      }
      if (!['direct', 'align', 'moveit_align'].includes(band)) {
        status.textContent = `Dry-run blocked: file check is ${band || 'unknown'}`;
        return;
      }
      const confirmed = window.confirm(
        `Run teach dry-run check?\n` +
        `No real trajectory will be sent.\n` +
        `File check: ${band}\n` +
        `Record: ${latestTeachFileInfo?.path || '-'}`
      );
      if (!confirmed) return;
      button.disabled = true;
      status.textContent = 'Running dry-run check...';
      addReplayEvent(`dry-run requested: ${latestTeachFileInfo?.path || '-'}`);
      try {
        const response = await fetch('/api/teach_dry_run', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            record_path: latestTeachFileInfo?.path || undefined,
            settings: replaySettingsPayload(),
          }),
        });
        const result = await response.json();
        lastTeachDryRun = result.accepted ? result : null;
        status.textContent = result.message || (result.accepted ? 'Dry-run accepted' : 'Dry-run blocked');
        addReplayEvent(`${result.accepted ? 'dry-run accepted' : 'dry-run blocked'}: ${result.message || '-'}`);
        refreshTeachFileInfo();
      } catch (error) {
        status.textContent = `Dry-run request failed: ${error.message}`;
        addReplayEvent(`dry-run request failed: ${error.message}`);
      } finally {
        updateTeachDryRunButton(latestTeachFileInfo);
      }
    };
    const runTeachReplay = async () => {
      const replayButton = document.getElementById('run-teach-replay');
      const status = document.getElementById('teach-dry-run-status');
      const band = String(latestTeachFileInfo?.start_band || '').toLowerCase();
      if (isCheckMode) {
        status.textContent = 'Replay blocked: check mode is read-only.';
        return;
      }
      if (replayButton.disabled || !lastTeachDryRun?.accepted) {
        status.textContent = 'Real replay blocked: run dry-run first and keep file check valid.';
        return;
      }
      const confirmed = window.confirm(
        `Execute real teach replay?\n` +
        `This will send a FollowJointTrajectory goal to the arm.\n` +
        `File check: ${band}\n` +
        `Max start error: ${fmt(Number(latestTeachFileInfo?.max_error))} rad\n` +
        `Replay speed: ${fmt(Number(teachReplaySettings.replaySpeed), 1)}x\n` +
        `Final hold: ${fmt(Number(teachReplaySettings.finalHold), 1)}s\n` +
        `Estimated duration: ${fmt(replayEstimate(latestTeachFileInfo).estimatedDuration, 1)}s\n` +
        `Estimated points: ${replayEstimate(latestTeachFileInfo).trajectoryPoints}\n` +
        `Record: ${latestTeachFileInfo?.path || '-'}`
      );
      if (!confirmed) return;
      replayButton.disabled = true;
      status.textContent = 'Sending real teach replay trajectory...';
      addReplayEvent(`real replay requested: ${latestTeachFileInfo?.path || '-'}`);
      try {
        const response = await fetch('/api/teach_replay_execute', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            record_path: latestTeachFileInfo?.path || undefined,
            dry_run_token: lastTeachDryRun,
            settings: replaySettingsPayload(),
          }),
        });
        const result = await response.json();
        status.textContent = result.message || (result.accepted ? 'Replay accepted' : 'Replay blocked');
        addReplayEvent(`${result.accepted ? 'real replay accepted' : 'real replay blocked'}: ${result.message || '-'}`);
        if (!result.accepted) {
          updateTeachDryRunButton(latestTeachFileInfo);
        }
      } catch (error) {
        status.textContent = `Replay request failed: ${error.message}`;
        addReplayEvent(`real replay request failed: ${error.message}`);
        updateTeachDryRunButton(latestTeachFileInfo);
      }
    };
    const stopTeachReplay = async () => {
      const button = document.getElementById('stop-teach-replay');
      const status = document.getElementById('teach-dry-run-status');
      button.disabled = true;
      status.textContent = 'Requesting teach replay stop...';
      addReplayEvent('stop replay requested');
      try {
        const response = await fetch('/api/teach_replay_stop', { method: 'POST' });
        const result = await response.json();
        status.textContent = result.message || (result.accepted ? 'Teach replay stop requested' : 'Teach replay stop blocked');
        addReplayEvent(`${result.accepted ? 'stop replay accepted' : 'stop replay blocked'}: ${result.message || '-'}`);
        if (!result.accepted && result.state !== 'idle') {
          button.disabled = false;
        }
      } catch (error) {
        status.textContent = `Teach replay stop failed: ${error.message}`;
        addReplayEvent(`stop replay failed: ${error.message}`);
        updateTeachDryRunButton(latestTeachFileInfo);
      }
    };
    const syncTeachRecordButtons = (recordingValue) => {
      const recording = statusObj(recordingValue || {});
      const state = String(recording.state || '').toLowerCase();
      const active = ['starting', 'recording', 'waiting'].includes(state);
      document.getElementById('start-teach-record').disabled = active || isCheckMode;
      document.getElementById('stop-teach-record').disabled = !active || isCheckMode;
    };
    const startTeachRecord = async () => {
      const button = document.getElementById('start-teach-record');
      button.disabled = true;
      document.getElementById('teach-dry-run-status').textContent = 'Starting gravity compensation and teach recording...';
      addReplayEvent('teach recording start requested');
      try {
        const recordName = String(document.getElementById('teach-record-name')?.value || '').trim();
        const response = await fetch('/api/teach_record_start', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ record_path: recordName }),
        });
        const result = await response.json();
        document.getElementById('teach-dry-run-status').textContent = result.message || (result.accepted ? 'Teach recording started' : 'Teach recording blocked');
        addReplayEvent(`${result.accepted ? 'teach recording started' : 'teach recording blocked'}: ${result.message || '-'}`);
        if (result.record_path) {
          selectedTeachRecordPath = result.record_path;
          document.getElementById('teach-record-name').value = result.record_path.split('/').pop() || result.record_path;
        }
        syncTeachRecordButtons({ state: result.accepted ? 'recording' : 'idle' });
        await refreshTeachRecords();
        await refreshTeachFileInfo();
      } catch (error) {
        document.getElementById('teach-dry-run-status').textContent = `Teach recording start failed: ${error.message}`;
        addReplayEvent(`teach recording start failed: ${error.message}`);
        syncTeachRecordButtons({ state: 'idle' });
      }
    };
    const stopTeachRecord = async () => {
      const button = document.getElementById('stop-teach-record');
      button.disabled = true;
      document.getElementById('teach-dry-run-status').textContent = 'Stopping teach recording and gravity compensation...';
      addReplayEvent('teach recording stop requested');
      try {
        const response = await fetch('/api/teach_record_stop', { method: 'POST' });
        const result = await response.json();
        document.getElementById('teach-dry-run-status').textContent = result.message || (result.accepted ? 'Teach recording stopped' : 'Teach recording stop failed');
        addReplayEvent(`${result.accepted ? 'teach recording stopped' : 'teach recording stop failed'}: ${result.message || '-'}`);
        syncTeachRecordButtons({ state: 'stopped' });
        await refreshTeachRecords();
        await refreshTeachFileInfo();
      } catch (error) {
        document.getElementById('teach-dry-run-status').textContent = `Teach recording stop failed: ${error.message}`;
        addReplayEvent(`teach recording stop failed: ${error.message}`);
        syncTeachRecordButtons({ state: 'recording' });
      }
    };
    const runArmCommand = async (command, label) => {
      const buttons = ['arm-safe-home', 'arm-enable', 'arm-disable']
        .map((id) => document.getElementById(id))
        .filter(Boolean);
      if (!webExecuteEnabled) {
        return;
      }
      if (command === 'safe_home') {
        const confirmed = window.confirm('Run Safe Home now? The arm will move to its configured safe position.');
        if (!confirmed) return;
      }
      buttons.forEach((button) => { button.disabled = true; });
      try {
        const response = await fetch(`/api/arm_${command}`, { method: 'POST' });
        await response.json();
      } catch (_error) {
        // The Arm Status card is driven by /arm_status; keep this row visually quiet.
      } finally {
        buttons.forEach((button) => { button.disabled = isCheckMode || !webExecuteEnabled; });
      }
    };
    const bindTeachReplaySetting = (rangeId, numberId, key, min, max, digits = 1) => {
      const range = document.getElementById(rangeId);
      const number = document.getElementById(numberId);
      const update = (value) => {
        const clamped = key === 'alignSteps'
          ? Math.round(clamp(value, min, max))
          : clamp(value, min, max);
        teachReplaySettings[key] = clamped;
        range.value = clamped;
        number.value = Number(clamped).toFixed(digits);
        document.getElementById('replay-precheck-summary').innerHTML = renderReplayPrecheckSummary(latestTeachFileInfo);
        document.getElementById('teach-params-details').innerHTML = renderReplayParams(latestTeachFileInfo);
        lastTeachDryRun = null;
        updateTeachDryRunButton(latestTeachFileInfo);
      };
      range.addEventListener('input', (event) => update(event.target.value));
      number.addEventListener('change', (event) => update(event.target.value));
      update(teachReplaySettings[key]);
    };
    const syncKeyboardButtons = () => {
      document.getElementById('keyboard-enable').disabled = keyboardSettings.enabled || isCheckMode || !webExecuteEnabled;
      document.getElementById('keyboard-disable').disabled = !keyboardSettings.enabled;
    };
    const bindKeyboardSetting = (numberId, key, min, max, digits = 3) => {
      const number = document.getElementById(numberId);
      number.min = min;
      number.max = max;
      const update = (value) => {
        const clamped = clamp(value, min, max);
        keyboardSettings[key] = clamped;
        number.value = Number(clamped).toFixed(digits);
      };
      number.addEventListener('change', (event) => update(event.target.value));
      update(keyboardSettings[key]);
    };
    const keyboardDurationForStep = () => {
      const speedDuration = keyboardSettings.stepRad / Math.max(keyboardSettings.maxSpeed, 1e-6);
      return clamp(Math.max(keyboardSettings.duration, speedDuration), keyboardSettings.minDuration, keyboardSettings.maxDuration);
    };
    const enableKeyboardTeleop = async () => {
      const status = document.getElementById('teleop-state');
      try {
        const response = await fetch('/api/keyboard_enable', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            step_rad: keyboardSettings.stepRad,
            duration: keyboardDurationForStep(),
            max_joint_speed_rad_s: keyboardSettings.maxSpeed,
          }),
        });
        const result = await response.json();
        keyboardSettings.enabled = response.ok && result.accepted;
        setChip('teleop-state', result.state || (result.accepted ? 'ready' : 'rejected'));
        if (result.message) status.insertAdjacentHTML('beforeend', `<div class="detail">${escapeHtml(result.message)}</div>`);
      } catch (error) {
        keyboardSettings.enabled = false;
        status.innerHTML = `<span class="chip bad">failed</span><div class="detail">${escapeHtml(error.message)}</div>`;
      }
      syncKeyboardButtons();
    };
    const disableKeyboardTeleop = async () => {
      const status = document.getElementById('teleop-state');
      try {
        const response = await fetch('/api/keyboard_disable', { method: 'POST' });
        const result = await response.json();
        keyboardSettings.enabled = false;
        setChip('teleop-state', result.state || 'disabled');
        if (result.message) status.insertAdjacentHTML('beforeend', `<div class="detail">${escapeHtml(result.message)}</div>`);
      } catch (error) {
        status.innerHTML = `<span class="chip bad">failed</span><div class="detail">${escapeHtml(error.message)}</div>`;
      }
      syncKeyboardButtons();
    };
    const sendKeyboardKey = async (key) => {
      if (!keyboardSettings.enabled || isCheckMode || !webExecuteEnabled) return;
      try {
        const response = await fetch('/api/keyboard_key', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            confirm: 'KEYBOARD_TELEOP',
            key,
            step_rad: keyboardSettings.stepRad,
            duration: keyboardDurationForStep(),
            max_joint_speed_rad_s: keyboardSettings.maxSpeed,
          }),
        });
        const result = await response.json();
        setChip('teleop-state', result.accepted ? 'active' : 'rejected');
        const status = document.getElementById('teleop-state');
        status.insertAdjacentHTML('beforeend', `<div class="detail">${escapeHtml(result.message || key)}</div>`);
      } catch (error) {
        const status = document.getElementById('teleop-state');
        status.innerHTML = `<span class="chip bad">failed</span><div class="detail">${escapeHtml(error.message)}</div>`;
      }
    };
    bindExecuteSetting('execute-max-delta', 'maxDelta', 0.05, executeMaxDeltaLimit);
    bindExecuteSetting('execute-duration', 'duration', 1.0, 8.0);
    bindExecuteSetting('execute-speed', 'maxSpeed', 0.1, 1.5);
    bindKeyboardSetting('keyboard-step-number', 'stepRad', keyboardSettings.minStepRad, keyboardSettings.maxStepRad, 3);
    bindKeyboardSetting('keyboard-speed-number', 'maxSpeed', 0.1, executeSettings.maxSpeed, 1);
    bindTeachReplaySetting('teach-replay-speed', 'teach-replay-speed-number', 'replaySpeed', 0.1, 3.0, 1);
    bindTeachReplaySetting('teach-align-duration', 'teach-align-duration-number', 'alignDuration', 1.0, 10.0, 1);
    bindTeachReplaySetting('teach-final-hold', 'teach-final-hold-number', 'finalHold', 0.0, 5.0, 1);
    sliderNames.forEach((name) => {
      document.getElementById(`preview-${name}`).addEventListener('input', (event) => {
        const [min, max] = jointLimits[name] || [-3.1416, 3.1416];
        previewState.targets[name] = clamp(event.target.value, min, max);
        updateSlider(name, { position: previewState.targets[name] }, 'preview');
        previewState.active = true;
        document.getElementById('web-teleop-card').classList.add('preview-active');
        document.getElementById('execute-preview').disabled = false;
        syncRealActionButtons();
        const speed = speedCheck();
        const speedText = speed.speed > speed.limit
          ? ` ${speed.name} speed too high; use duration >= ${fmt(speed.minDuration, 2)}s.`
          : ` Speed ${fmt(speed.speed, 3)} rad/s <= ${fmt(speed.limit, 3)} rad/s.`;
        document.getElementById('execute-status').textContent =
          `Preview target ready. Max delta ${fmt(maxPreviewDelta())} rad. Limit ${fmt(executeSettings.maxDelta)} rad, duration ${fmt(executeSettings.duration, 2)}s.${speedText}`;
        updateRobotViewer(jointsFromTargets(previewState.targets));
        viewerStatus('Preview only: browser model target is not sent to hardware');
      });
    });
    document.getElementById('sync-preview').addEventListener('click', syncPreviewFromLive);
    document.getElementById('execute-preview').addEventListener('click', executePreviewAndGripper);
    document.getElementById('stop-execute').addEventListener('click', stopExecute);
    document.getElementById('arm-safe-home').addEventListener('click', () => runArmCommand('safe_home', 'Safe Home'));
    document.getElementById('arm-enable').addEventListener('click', () => runArmCommand('enable', 'Enable'));
    document.getElementById('arm-disable').addEventListener('click', () => runArmCommand('disable', 'Disable'));
    document.getElementById('keyboard-enable').addEventListener('click', enableKeyboardTeleop);
    document.getElementById('keyboard-disable').addEventListener('click', disableKeyboardTeleop);
    document.getElementById('start-teach-record').addEventListener('click', startTeachRecord);
    document.getElementById('stop-teach-record').addEventListener('click', stopTeachRecord);
    window.addEventListener('keydown', (event) => {
      const targetTag = String(event.target?.tagName || '').toLowerCase();
      if (['input', 'textarea', 'select', 'button'].includes(targetTag)) return;
      const key = event.key.length === 1 ? event.key.toLowerCase() : event.key;
      if (key === 'Escape') {
        if (keyboardSettings.enabled) {
          event.preventDefault();
          disableKeyboardTeleop();
        }
        return;
      }
      const mapped = ['1', 'q', '2', 'w', '3', 'e', '4', 'r', '5', 't', '6', 'y'];
      if (mapped.includes(key)) {
        event.preventDefault();
        sendKeyboardKey(key);
      }
    });
    syncKeyboardButtons();
    document.getElementById('run-teach-dry-run').addEventListener('click', runTeachDryRun);
    document.getElementById('run-teach-replay').addEventListener('click', runTeachReplay);
    document.getElementById('stop-teach-replay').addEventListener('click', stopTeachReplay);
    document.getElementById('load-teach-trajectory').addEventListener('click', loadTeachTrajectory);
    document.getElementById('teach-trajectory-frame').addEventListener('input', (event) => {
      previewTeachTrajectoryFrame(event.target.value);
    });
    document.getElementById('refresh-teach-records').addEventListener('click', async () => {
      await refreshTeachRecords();
      await refreshTeachFileInfo();
    });
    document.getElementById('teach-record-select').addEventListener('change', (event) => {
      selectTeachRecord(event.target.value);
    });
    source.addEventListener('status', (event) => {
      const data = JSON.parse(event.data);
      const nowMs = performance.now();
      const shouldRenderFastPanels = nowMs - lastFastRenderMs > FAST_RENDER_INTERVAL_MS;
      const shouldRenderSlowPanels = nowMs - lastSlowRenderMs > 1000;
      latestStatusData = data;
      previewState.latestJoints = data.joints || {};
      previewState.latestTeleop = data.teleop || {};
      if (!shouldRenderFastPanels) return;
      lastFastRenderMs = nowMs;
      document.getElementById('mode').textContent = data.arm.mode || '-';
      document.getElementById('state').textContent = data.arm.state_machine || '-';
      document.getElementById('enabled').textContent = data.arm.enabled ? 'true' : 'false';
      setChip('teleop-state', statusObj(data.teleop.status).state || '-');
      setChip('recording-state', statusObj(data.teleop.recording).state || '-');
      setChip('replay-state', statusObj(data.teleop.replay).state || '-');
      const replayStatus = statusObj(data.teleop.replay);
      const replayKey = JSON.stringify({
        state: replayStatus.state || '-',
        message: replayStatus.message || '',
        points: replayStatus.trajectory_points || '',
      });
      if (replayKey !== lastReplayStatusKey) {
        lastReplayStatusKey = replayKey;
        if ((replayStatus.state || replayStatus.message) && replayStatus.state !== 'idle') {
          addReplayEvent(`status ${replayStatus.state || '-'}: ${replayStatus.message || '-'}`);
        }
      }
      const webExecuteState = statusObj(data.teleop.web_execute).state || '-';
      document.getElementById('stop-execute').disabled = !['active', 'accepted'].includes(String(webExecuteState).toLowerCase());
      ['arm-safe-home', 'arm-enable', 'arm-disable'].forEach((id) => {
        const button = document.getElementById(id);
        if (button) button.disabled = isCheckMode || !webExecuteEnabled;
      });
      updateTeachDryRunButton(latestTeachFileInfo);
      document.getElementById('updated').textContent = `Updated ${new Date().toLocaleTimeString()}`;
      syncRealActionButtons();
      syncTeachRecordButtons(data.teleop.recording);
      setHtml('teach-record-summary', renderTeachRecordSummary(latestTeachFileInfo, data.teleop.recording));
      updateMotorRows(data.joints || {});
      sliderNames.forEach((name) => updateSlider(name, data.joints[name]));
      if (!previewState.active) {
        sliderNames.forEach((name) => updateSlider(name, data.joints[name], 'preview'));
        updateRobotViewer(data.joints);
      }
      if (shouldRenderSlowPanels) {
        lastSlowRenderMs = nowMs;
        renderOptionalDetails(latestStatusData);
      }
      const errors = data.arm.error_codes || [];
      document.getElementById('errors').innerHTML = errors.length
        ? errors.map((item) => `<div class="chip bad">${item}</div>`).join('')
        : '<div class="empty">No errors</div>';
    });
    attachControlCardToggles();
    attachDetailsUnloaders();
    refreshTeachRecords();
    refreshTeachFileInfo();
    setInterval(refreshTeachFileInfo, 5000);
  </script>
</body>
</html>
"""


class TeleopStatusPanelNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_status_panel_node")
        self.declare_parameter("arm_namespace", "rebotarm")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8088)
        self.declare_parameter("sse_rate_hz", 10.0)
        self.declare_parameter("record_path", "teleop_records/teach_record.jsonl")
        self.declare_parameter("direct_threshold", 0.01)
        self.declare_parameter("align_threshold", 0.25)
        self.declare_parameter("align_duration", 3.0)
        self.declare_parameter("align_steps", 30)
        self.declare_parameter("replay_speed", 1.0)
        self.declare_parameter("green_jump_rad", 0.03)
        self.declare_parameter("yellow_jump_rad", 0.05)
        self.declare_parameter("yellow_max_speed", 0.6)
        self.declare_parameter("max_replay_velocity_rad_s", 1.5)
        self.declare_parameter("max_replay_acceleration_rad_s2", 3.0)
        self.declare_parameter("max_replay_jerk_rad_s3", 8.0)
        self.declare_parameter("large_motion_span_rad", 0.8)
        self.declare_parameter("large_motion_total_rad", 2.5)
        self.declare_parameter("large_motion_max_speed", 0.4)
        self.declare_parameter("start_hold_sec", 0.8)
        self.declare_parameter("soft_start_duration", 1.0)
        self.declare_parameter("soft_start_steps", 30)
        self.declare_parameter("first_hold_sec", 0.3)
        self.declare_parameter("final_hold_sec", 1.0)
        self.declare_parameter("initial_replay_delay_sec", 0.2)
        self.declare_parameter("use_moveit_start_align", True)
        self.declare_parameter("moveit_start_skip_threshold", 0.005)
        self.declare_parameter("moveit_group_name", "arm")
        self.declare_parameter("collision_group_name", "arm_with_gripper")
        self.declare_parameter("moveit_planning_service", "/plan_kinematic_path")
        self.declare_parameter("moveit_planning_pipeline", "ompl")
        self.declare_parameter("moveit_planner_id", "")
        self.declare_parameter("moveit_planning_time", 3.0)
        self.declare_parameter("moveit_num_planning_attempts", 3)
        self.declare_parameter("moveit_joint_goal_tolerance", 0.005)
        self.declare_parameter("moveit_velocity_scaling", 0.1)
        self.declare_parameter("moveit_acceleration_scaling", 0.1)
        self.declare_parameter("collision_check_enabled", True)
        self.declare_parameter("collision_check_service", "/check_state_validity")
        self.declare_parameter("collision_check_max_samples", 80)
        self.declare_parameter("collision_check_timeout_sec", 2.0)
        self.declare_parameter("smoothing_enabled", True)
        self.declare_parameter("smoothing_window", 7)
        self.declare_parameter("filter_enabled", True)
        self.declare_parameter("filter_cutoff_hz", 5.0)
        self.declare_parameter("filter_sample_rate_hz", 50.0)
        self.declare_parameter("resample_enabled", True)
        self.declare_parameter("resample_rate_hz", 100.0)
        self.declare_parameter("max_prepared_jump_rad", 0.02)
        self.declare_parameter("use_hardware", False)
        self.declare_parameter("panel_mode", "control")
        self.declare_parameter(
            "joint_names",
            ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        )
        self.declare_parameter(
            "joint_lower_limits",
            [-3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159],
        )
        self.declare_parameter(
            "joint_upper_limits",
            [3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159],
        )
        self.declare_parameter("web_execute_enabled", False)
        self.declare_parameter("web_execute_max_delta_rad", 1.5)
        self.declare_parameter("web_execute_min_duration", 1.0)
        self.declare_parameter("web_execute_max_duration", 8.0)
        self.declare_parameter("web_execute_max_joint_speed_rad_s", 1.5)
        self.declare_parameter("web_keyboard_default_step_rad", 0.02)
        self.declare_parameter("web_keyboard_min_step_rad", 0.005)
        self.declare_parameter("web_keyboard_max_step_rad", 0.10)
        self.declare_parameter("web_keyboard_default_duration", 0.2)
        self.declare_parameter("web_keyboard_min_duration", 0.1)
        self.declare_parameter("web_keyboard_max_duration", 2.0)
        self.declare_parameter("web_keyboard_default_speed_rad_s", 0.5)
        self.declare_parameter("gripper_lower_limit_m", DEFAULT_GRIPPER_LIMITS_M[0])
        self.declare_parameter("gripper_upper_limit_m", DEFAULT_GRIPPER_LIMITS_M[1])
        self.declare_parameter("web_gripper_max_effort", 0.3)
        self.declare_parameter("web_gripper_max_effort_limit", 1.5)
        self._arm_namespace = str(self.get_parameter("arm_namespace").value).strip("/")
        self._joint_names = tuple(str(v) for v in self.get_parameter("joint_names").value)
        bringup_share = Path(get_package_share_directory("rebotarm_bringup"))
        self._urdf_path = bringup_share / "description" / "urdf" / "reBot-DevArm_fixend.urdf"
        self._mesh_dir = bringup_share / "description" / "meshes"
        lower = tuple(float(v) for v in self.get_parameter("joint_lower_limits").value)
        upper = tuple(float(v) for v in self.get_parameter("joint_upper_limits").value)
        fallback_limits = build_joint_limits(
            joint_names=self._joint_names,
            lower_limits=lower,
            upper_limits=upper,
        )
        try:
            urdf_limits = load_urdf_joint_limits(self._urdf_path, self._joint_names)
        except Exception as exc:
            self.get_logger().warn(f"failed to load URDF joint limits, using parameter limits: {exc}")
            urdf_limits = {}
        self._joint_limits = merge_joint_limits(
            joint_names=self._joint_names,
            fallback_limits=fallback_limits,
            preferred_limits=urdf_limits,
        )
        moveit_share = Path(get_package_share_directory("rebotarm_moveit_config"))
        moveit_velocity_limits = load_moveit_velocity_limits(
            moveit_share / "config" / "joint_limits.yaml",
            self._joint_names,
        )
        self._joint_velocity_limits = merge_velocity_limits(
            joint_names=self._joint_names,
            default_limit=float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
            preferred_limits=moveit_velocity_limits,
        )
        gripper_lower = float(self.get_parameter("gripper_lower_limit_m").value)
        gripper_upper = float(self.get_parameter("gripper_upper_limit_m").value)
        if gripper_upper < gripper_lower:
            gripper_lower, gripper_upper = gripper_upper, gripper_lower
        self._gripper_limits = (gripper_lower, gripper_upper)
        self._use_hardware = bool(self.get_parameter("use_hardware").value)
        self._sim_gripper_position = gripper_lower
        self._store = TeleopStatusStore()
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            f"/{self._arm_namespace}/follow_joint_trajectory",
        )
        self._moveit_planner = MoveItMotionPlanner(
            self,
            group_name=str(self.get_parameter("moveit_group_name").value),
            ee_frame_id="end_link",
            frame_id="base_link",
            planning_service=str(self.get_parameter("moveit_planning_service").value),
            planning_pipeline=str(self.get_parameter("moveit_planning_pipeline").value),
            planner_id=str(self.get_parameter("moveit_planner_id").value),
            planning_time=float(self.get_parameter("moveit_planning_time").value),
            num_attempts=int(self.get_parameter("moveit_num_planning_attempts").value),
            goal_position_tolerance=0.005,
            goal_orientation_tolerance=0.02,
        )
        self._state_validity_client = self.create_client(
            GetStateValidity,
            str(self.get_parameter("collision_check_service").value),
        )
        self._gripper_action_client = ActionClient(
            self,
            GripperCommand,
            f"/{self._arm_namespace}/gripper/command",
        )
        self._gravity_start_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/gravity_compensation/start",
        )
        self._gravity_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/gravity_compensation/stop",
        )
        self._teach_record_start_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/teleop/teach_record/start",
        )
        self._teach_record_set_path_client = self.create_client(
            SetTeachRecordPath,
            f"/{self._arm_namespace}/teleop/teach_record/set_path",
        )
        self._teach_record_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/teleop/teach_record/stop",
        )
        self._trajectory_stop_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/trajectory_stop",
        )
        self._arm_enable_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/enable",
        )
        self._arm_disable_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/disable",
        )
        self._arm_safe_home_client = self.create_client(
            Trigger,
            f"/{self._arm_namespace}/safe_home",
        )
        self._execute_lock = threading.Lock()
        self._execute_goal_handle = None
        self._web_keyboard_lock = threading.Lock()
        self._web_keyboard_enabled = False
        self._web_keyboard_step_rad = float(self.get_parameter("web_keyboard_default_step_rad").value)
        self._web_keyboard_duration = float(self.get_parameter("web_keyboard_default_duration").value)
        self._web_keyboard_speed = float(self.get_parameter("web_keyboard_default_speed_rad_s").value)
        self._teach_replay_lock = threading.Lock()
        self._teach_replay_goal_handle = None
        self._last_teach_dry_run: dict | None = None
        sensor_qos_spec = sensor_qos_kwargs()
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=int(sensor_qos_spec["depth"]),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        arm_status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            JointState,
            f"/{self._arm_namespace}/joint_states",
            self._on_joint_state,
            sensor_qos,
        )
        self.create_subscription(ArmStatus, f"/{self._arm_namespace}/arm_status", self._on_arm_status, arm_status_qos)
        self.create_subscription(String, f"/{self._arm_namespace}/teleop/status", lambda msg: self._on_status("status", msg), 10)
        self.create_subscription(String, f"/{self._arm_namespace}/teleop/recording_status", lambda msg: self._on_status("recording", msg), 10)
        self.create_subscription(String, f"/{self._arm_namespace}/teleop/replay_status", lambda msg: self._on_status("replay", msg), 10)
        for joint_name in self._joint_names:
            self.create_subscription(
                JointMotorState,
                f"/{self._arm_namespace}/joints/{joint_name}/state",
                self._on_motor_state,
                sensor_qos,
            )
        self.create_subscription(
            JointMotorState,
            f"/{self._arm_namespace}/gripper/state",
            self._on_gripper_state,
            sensor_qos,
        )
        self._sim_gripper_state_pub = None
        if not self._use_hardware:
            self._sim_gripper_state_pub = self.create_publisher(
                JointMotorState,
                f"/{self._arm_namespace}/gripper/state",
                sensor_qos,
            )
            self.create_timer(0.1, self._publish_simulated_gripper_state)
        self.create_timer(1.0, self._update_gravity_comp_status)
        self._server = self._make_server()
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self.get_logger().info(f"teleop status panel available at http://{host}:{port}/")

    def _make_server(self) -> ThreadingHTTPServer:
        store = self._store
        node = self
        urdf_path = self._urdf_path
        mesh_dir = self._mesh_dir
        interval = 1.0 / max(float(self.get_parameter("sse_rate_hz").value), 1.0)

        class Handler(BaseHTTPRequestHandler):
            def handle(self):  # noqa: D401
                try:
                    super().handle()
                except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError):
                    return

            def do_GET(self):  # noqa: N802
                url = urlsplit(self.path)
                route = url.path
                if route == "/":
                    body = HTML_PAGE.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if route == "/api/status":
                    body = json.dumps(store.snapshot_dict(), separators=(",", ":")).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if route == "/api/config":
                    body = json.dumps(node._panel_config(), separators=(",", ":")).encode("utf-8")  # noqa: SLF001
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if route == "/api/teach_record_info":
                    query = parse_qs(url.query)
                    record_path = query.get("path", query.get("record_path", [""]))[0]
                    body = json.dumps(
                        node._teach_record_info(record_path or None),  # noqa: SLF001
                        separators=(",", ":"),
                    ).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if route == "/api/teach_records":
                    body = json.dumps(node._teach_records(), separators=(",", ":")).encode("utf-8")  # noqa: SLF001
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if route == "/api/teach_trajectory":
                    query = parse_qs(url.query)
                    record_path = query.get("path", [""])[0]
                    max_points = int(query.get("max_points", ["500"])[0])
                    body = json.dumps(
                        node._teach_trajectory(record_path or None, max_points=max_points),  # noqa: SLF001
                        separators=(",", ":"),
                    ).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if route == "/robot/urdf":
                    body = rewrite_package_mesh_uris(urdf_path.read_text(encoding="utf-8")).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/xml; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if route.startswith("/robot/meshes/"):
                    name = unquote(route.removeprefix("/robot/meshes/"))
                    path = safe_mesh_path(mesh_dir, name)
                    if path is None:
                        self.send_response(404)
                        self.end_headers()
                        return
                    body = path.read_bytes()
                    self.send_response(200)
                    self.send_header("Content-Type", "model/stl")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if route == "/events":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/event-stream")
                    self.send_header("Cache-Control", "no-cache")
                    self.end_headers()
                    try:
                        while True:
                            payload = encode_sse_event(store.snapshot_dict()).encode("utf-8")
                            self.wfile.write(payload)
                            self.wfile.flush()
                            time.sleep(interval)
                    except Exception:
                        return
                self.send_response(404)
                self.end_headers()

            def do_POST(self):  # noqa: N802
                if self.path not in ("/api/execute_preview", "/api/stop_execute", "/api/set_gripper", "/api/keyboard_enable", "/api/keyboard_disable", "/api/keyboard_key", "/api/teach_record_start", "/api/teach_record_stop", "/api/teach_dry_run", "/api/teach_replay_execute", "/api/teach_replay_stop", "/api/arm_safe_home", "/api/arm_enable", "/api/arm_disable"):
                    self.send_response(404)
                    self.end_headers()
                    return
                try:
                    if self.path == "/api/stop_execute":
                        result = node._handle_stop_execute()  # noqa: SLF001
                    elif self.path == "/api/arm_safe_home":
                        result = node._handle_arm_service_command("safe_home")  # noqa: SLF001
                    elif self.path == "/api/arm_enable":
                        result = node._handle_arm_service_command("enable")  # noqa: SLF001
                    elif self.path == "/api/arm_disable":
                        result = node._handle_arm_service_command("disable")  # noqa: SLF001
                    elif self.path == "/api/keyboard_disable":
                        result = node._handle_keyboard_disable()  # noqa: SLF001
                    elif self.path == "/api/teach_record_start":
                        length = int(self.headers.get("Content-Length", "0"))
                        raw = self.rfile.read(length) if length > 0 else b"{}"
                        payload = json.loads(raw.decode("utf-8"))
                        if not isinstance(payload, dict):
                            raise ValueError("payload must be an object")
                        result = node._handle_teach_record_start(payload)  # noqa: SLF001
                    elif self.path == "/api/teach_record_stop":
                        result = node._handle_teach_record_stop()  # noqa: SLF001
                    elif self.path == "/api/teach_replay_stop":
                        result = node._handle_teach_replay_stop()  # noqa: SLF001
                    elif self.path in ("/api/teach_dry_run", "/api/teach_replay_execute"):
                        length = int(self.headers.get("Content-Length", "0"))
                        raw = self.rfile.read(length) if length > 0 else b"{}"
                        payload = json.loads(raw.decode("utf-8"))
                        if not isinstance(payload, dict):
                            raise ValueError("payload must be an object")
                        if self.path == "/api/teach_dry_run":
                            result = node._handle_teach_dry_run(payload)  # noqa: SLF001
                        else:
                            result = node._handle_teach_replay_execute(payload)  # noqa: SLF001
                    else:
                        length = int(self.headers.get("Content-Length", "0"))
                        raw = self.rfile.read(length) if length > 0 else b"{}"
                        payload = json.loads(raw.decode("utf-8"))
                        if not isinstance(payload, dict):
                            raise ValueError("payload must be an object")
                        if self.path == "/api/set_gripper":
                            result = node._handle_set_gripper(payload)  # noqa: SLF001
                        elif self.path == "/api/keyboard_enable":
                            result = node._handle_keyboard_enable(payload)  # noqa: SLF001
                        elif self.path == "/api/keyboard_key":
                            result = node._handle_keyboard_key(payload)  # noqa: SLF001
                        else:
                            result = node._handle_execute_preview(payload)  # noqa: SLF001
                    status = 200 if result.get("accepted") else 400
                except Exception as exc:
                    result = {"accepted": False, "message": f"invalid web execute request: {exc}"}
                    status = 400
                body = json.dumps(result, separators=(",", ":")).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *_args):
                return

        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        return ThreadingHTTPServer((host, port), Handler)

    def _panel_config(self) -> dict:
        return {
            "joint_names": list(self._joint_names),
            "joint_limits": {
                name: [float(lower), float(upper)]
                for name, (lower, upper) in self._joint_limits.items()
            },
            "joint_velocity_limits": {
                name: float(limit)
                for name, limit in self._joint_velocity_limits.items()
            },
            "gripper_limits": [float(self._gripper_limits[0]), float(self._gripper_limits[1])],
            "web_execute": {
                "enabled": bool(self.get_parameter("web_execute_enabled").value),
                "max_delta_rad": float(self.get_parameter("web_execute_max_delta_rad").value),
                "max_joint_speed_rad_s": float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
                "min_duration": float(self.get_parameter("web_execute_min_duration").value),
                "max_duration": float(self.get_parameter("web_execute_max_duration").value),
            },
            "web_keyboard": {
                "step_rad": float(self.get_parameter("web_keyboard_default_step_rad").value),
                "min_step_rad": float(self.get_parameter("web_keyboard_min_step_rad").value),
                "max_step_rad": float(self.get_parameter("web_keyboard_max_step_rad").value),
                "duration": float(self.get_parameter("web_keyboard_default_duration").value),
                "min_duration": float(self.get_parameter("web_keyboard_min_duration").value),
                "max_duration": float(self.get_parameter("web_keyboard_max_duration").value),
                "max_joint_speed_rad_s": float(self.get_parameter("web_keyboard_default_speed_rad_s").value),
            },
            "web_gripper": {
                "max_effort": float(self.get_parameter("web_gripper_max_effort").value),
                "max_effort_limit": float(self.get_parameter("web_gripper_max_effort_limit").value),
            },
            "teach": {
                "record_path": str(self.get_parameter("record_path").value),
                "direct_threshold": float(self.get_parameter("direct_threshold").value),
                "align_threshold": float(self.get_parameter("align_threshold").value),
                "align_duration": float(self.get_parameter("align_duration").value),
                "align_steps": int(self.get_parameter("align_steps").value),
                "replay_speed": float(self.get_parameter("replay_speed").value),
                "green_jump_rad": float(self.get_parameter("green_jump_rad").value),
                "yellow_jump_rad": float(self.get_parameter("yellow_jump_rad").value),
                "yellow_max_speed": float(self.get_parameter("yellow_max_speed").value),
                "max_replay_velocity_rad_s": float(self.get_parameter("max_replay_velocity_rad_s").value),
                "max_replay_acceleration_rad_s2": float(self.get_parameter("max_replay_acceleration_rad_s2").value),
                "max_replay_jerk_rad_s3": float(self.get_parameter("max_replay_jerk_rad_s3").value),
                "large_motion_span_rad": float(self.get_parameter("large_motion_span_rad").value),
                "large_motion_total_rad": float(self.get_parameter("large_motion_total_rad").value),
                "large_motion_max_speed": float(self.get_parameter("large_motion_max_speed").value),
                "start_hold_sec": float(self.get_parameter("start_hold_sec").value),
                "soft_start_duration": float(self.get_parameter("soft_start_duration").value),
                "soft_start_steps": int(self.get_parameter("soft_start_steps").value),
                "first_hold_sec": float(self.get_parameter("first_hold_sec").value),
                "final_hold_sec": float(self.get_parameter("final_hold_sec").value),
                "use_moveit_start_align": bool(self.get_parameter("use_moveit_start_align").value),
                "moveit_start_skip_threshold": float(self.get_parameter("moveit_start_skip_threshold").value),
                "collision_check_enabled": bool(self.get_parameter("collision_check_enabled").value),
                "collision_check_max_samples": int(self.get_parameter("collision_check_max_samples").value),
                "smoothing_enabled": bool(self.get_parameter("smoothing_enabled").value),
                "smoothing_window": int(self.get_parameter("smoothing_window").value),
                "filter_enabled": bool(self.get_parameter("filter_enabled").value),
                "filter_cutoff_hz": float(self.get_parameter("filter_cutoff_hz").value),
                "filter_sample_rate_hz": float(self.get_parameter("filter_sample_rate_hz").value),
                "resample_enabled": bool(self.get_parameter("resample_enabled").value),
                "resample_rate_hz": float(self.get_parameter("resample_rate_hz").value),
                "max_prepared_jump_rad": float(self.get_parameter("max_prepared_jump_rad").value),
                "use_hardware": bool(self.get_parameter("use_hardware").value) if self.has_parameter("use_hardware") else False,
            },
            "panel_mode": str(self.get_parameter("panel_mode").value),
        }

    def _teach_record_info(self, record_path: str | None = None) -> dict:
        snapshot = self._store.snapshot()
        path = record_path or str(self.get_parameter("record_path").value)
        for key in ("recording", "replay"):
            value = snapshot.teleop.get(key)
            if record_path is None and isinstance(value, dict) and value.get("record_path"):
                path = str(value["record_path"])
                break
        current_positions = {
            name: float(data["position"])
            for name, data in snapshot.joints.items()
            if name in self._joint_names and "position" in data
        }
        info = inspect_teach_record(
            path,
            current_positions=current_positions if current_positions else None,
            direct_threshold=float(self.get_parameter("direct_threshold").value),
            align_threshold=float(self.get_parameter("align_threshold").value),
        )
        payload = teach_record_info_to_dict(info)
        if (
            str(payload.get("start_band", "")).lower() == ReplayStartBand.REJECT.value
            and bool(self.get_parameter("use_moveit_start_align").value)
            and _is_number_like(payload.get("max_error"))
        ):
            payload["start_band"] = ReplayStartBand.MOVEIT_ALIGN.value
            payload["message"] = "start error requires MoveIt start alignment"
        payload["direct_threshold"] = float(self.get_parameter("direct_threshold").value)
        payload["align_threshold"] = float(self.get_parameter("align_threshold").value)
        return self._compact_replay_payload(payload)

    @staticmethod
    def _compact_list(items, *, limit: int = 12) -> list:
        values = list(items) if isinstance(items, (list, tuple)) else []
        return values[: max(int(limit), 0)]

    @classmethod
    def _compact_quality_payload(cls, quality: dict, *, limit: int = 12) -> dict:
        compact = dict(quality)
        if isinstance(compact.get("events"), list):
            compact["events_total"] = len(compact["events"])
            compact["events"] = cls._compact_list(compact["events"], limit=limit)
            compact["events_truncated"] = compact["events_total"] > len(compact["events"])
        if isinstance(compact.get("anomalies"), list):
            compact["anomalies_total"] = len(compact["anomalies"])
            compact["anomalies"] = cls._compact_list(compact["anomalies"], limit=limit)
            compact["anomalies_truncated"] = compact["anomalies_total"] > len(compact["anomalies"])
        return compact

    @classmethod
    def _compact_replay_payload(cls, payload: dict, *, limit: int = 12) -> dict:
        compact = dict(payload)
        for key in (
            "quality",
            "before_quality",
            "after_quality",
            "raw_quality",
            "filtered_quality",
            "retimed_quality",
        ):
            if isinstance(compact.get(key), dict):
                compact[key] = cls._compact_quality_payload(compact[key], limit=limit)
        if isinstance(compact.get("anomalies"), list):
            compact["anomalies_total"] = len(compact["anomalies"])
            compact["anomalies"] = cls._compact_list(compact["anomalies"], limit=limit)
            compact["anomalies_truncated"] = compact["anomalies_total"] > len(compact["anomalies"])
        if isinstance(compact.get("prepared_replay"), dict):
            compact["prepared_replay"] = cls._compact_replay_payload(compact["prepared_replay"], limit=limit)
        return compact

    def _prepare_teach_replay_samples(self, samples, settings: dict[str, float | int] | None = None):
        replay_speed = float(settings["replay_speed"]) if settings else float(self.get_parameter("replay_speed").value)
        return prepare_teach_replay_samples(
            samples,
            smoothing_enabled=bool(self.get_parameter("smoothing_enabled").value),
            smoothing_window=int(self.get_parameter("smoothing_window").value),
            filter_enabled=bool(self.get_parameter("filter_enabled").value),
            filter_cutoff_hz=float(self.get_parameter("filter_cutoff_hz").value),
            filter_sample_rate_hz=float(self.get_parameter("filter_sample_rate_hz").value),
            resample_enabled=bool(self.get_parameter("resample_enabled").value),
            resample_rate_hz=float(self.get_parameter("resample_rate_hz").value),
            retime_enabled=True,
            replay_speed=replay_speed,
            max_velocity_rad_s=float(self.get_parameter("max_replay_velocity_rad_s").value),
            max_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            max_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
            large_motion_span_rad=float(self.get_parameter("large_motion_span_rad").value),
            large_motion_total_rad=float(self.get_parameter("large_motion_total_rad").value),
            large_motion_max_speed=float(self.get_parameter("large_motion_max_speed").value),
        )

    def _moveit_align_summary(self, info_payload: dict, samples=None, *, plan: bool = False) -> dict:
        enabled = bool(self.get_parameter("use_moveit_start_align").value)
        max_error = info_payload.get("max_error")
        threshold = float(self.get_parameter("moveit_start_skip_threshold").value)
        if not enabled:
            return {"state": "disabled", "message": "MoveIt start alignment disabled"}
        if not _is_number_like(max_error):
            return {"state": "unknown", "message": "current start error unavailable"}
        if float(max_error) < threshold:
            return {
                "state": "skipped",
                "message": "already near teach start; MoveIt alignment not required",
                "max_error": float(max_error),
                "skip_threshold": threshold,
            }
        available = False
        try:
            available = bool(self._moveit_planner._client.service_is_ready())  # noqa: SLF001
            if not available:
                available = bool(self._moveit_planner._client.wait_for_service(timeout_sec=0.0))  # noqa: SLF001
        except Exception:
            available = False
        if not available:
            return {
                "state": "unavailable",
                "message": "MoveIt planning service unavailable",
                "max_error": float(max_error),
                "skip_threshold": threshold,
                "service": str(self.get_parameter("moveit_planning_service").value),
            }
        if not plan:
            return {
                "state": "ready",
                "message": "MoveIt planning service ready",
                "max_error": float(max_error),
                "skip_threshold": threshold,
                "service": str(self.get_parameter("moveit_planning_service").value),
            }
        if not samples:
            return {
                "state": "unknown",
                "message": "no teach samples for MoveIt start alignment precheck",
                "max_error": float(max_error),
                "skip_threshold": threshold,
                "service": str(self.get_parameter("moveit_planning_service").value),
            }
        first = samples[0]
        result = self._moveit_planner.plan_joint_positions(
            joint_names=tuple(first.joint_names),
            target_positions=tuple(first.positions),
            tolerance=float(self.get_parameter("moveit_joint_goal_tolerance").value),
            velocity_scaling=float(self.get_parameter("moveit_velocity_scaling").value),
            acceleration_scaling=float(self.get_parameter("moveit_acceleration_scaling").value),
        )
        points = len(getattr(result.trajectory, "points", [])) if result.trajectory is not None else 0
        return {
            "state": "planned" if result.success else "failed",
            "message": result.message,
            "max_error": float(max_error),
            "skip_threshold": threshold,
            "service": str(self.get_parameter("moveit_planning_service").value),
            "points": points,
        }

    def _collision_precheck(self, samples) -> dict:
        if not samples:
            return self._collision_precheck_positions((), [])
        first = samples[0]
        positions = [tuple(sample.positions) for sample in samples]
        return self._collision_precheck_positions(tuple(first.joint_names), positions)

    def _collision_precheck_trajectory(self, trajectory: JointTrajectory) -> dict:
        positions = [
            tuple(point.positions)
            for point in getattr(trajectory, "points", [])
            if getattr(point, "positions", None)
        ]
        return self._collision_precheck_positions(tuple(trajectory.joint_names), positions)

    def _collision_precheck_positions(self, joint_names: tuple[str, ...], positions_list: list[tuple[float, ...]]) -> dict:
        if not bool(self.get_parameter("collision_check_enabled").value):
            return {"state": "disabled", "message": "collision precheck disabled"}
        if not joint_names or not positions_list:
            return {"state": "unknown", "message": "no trajectory samples to check"}
        try:
            available = bool(self._state_validity_client.service_is_ready())
            if not available:
                available = bool(self._state_validity_client.wait_for_service(timeout_sec=0.0))
        except Exception:
            available = False
        if not available:
            return {
                "state": "unknown",
                "message": "MoveIt state validity service unavailable",
                "service": str(self.get_parameter("collision_check_service").value),
                "checked_samples": 0,
        }
        max_samples = max(int(self.get_parameter("collision_check_max_samples").value), 1)
        timeout_sec = max(float(self.get_parameter("collision_check_timeout_sec").value), 0.1)
        selected = _select_collision_samples(positions_list, max_samples=max_samples)
        collisions = []
        checked = 0
        deadline = time.monotonic() + timeout_sec
        for sample_index, positions in selected:
            if time.monotonic() >= deadline:
                return {
                    "state": "unknown",
                    "message": "collision precheck timed out",
                    "checked_samples": checked,
                    "requested_samples": len(selected),
                    "collisions": collisions,
                }
            request = GetStateValidity.Request()
            request.group_name = str(self.get_parameter("collision_group_name").value)
            request.robot_state.joint_state.name = list(joint_names)
            request.robot_state.joint_state.position = [float(v) for v in positions]
            future = self._state_validity_client.call_async(request)
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not future.done():
                break
            try:
                response = future.result()
            except Exception as exc:
                return {
                    "state": "unknown",
                    "message": f"collision precheck failed: {exc}",
                    "checked_samples": checked,
                    "requested_samples": len(selected),
                    "collisions": collisions,
                }
            checked += 1
            if not bool(getattr(response, "valid", False)):
                contacts = []
                for contact in list(getattr(response, "contacts", []))[:5]:
                    contacts.append(
                        {
                            "body_1": str(getattr(contact, "contact_body_1", "")),
                            "body_2": str(getattr(contact, "contact_body_2", "")),
                        }
                    )
                collisions.append({"sample": sample_index, "contacts": contacts})
                break
        if collisions:
            return {
                "state": "collision",
                "message": "collision detected in teach trajectory",
                "checked_samples": checked,
                "requested_samples": len(selected),
                "collisions": collisions,
            }
        if checked < len(selected):
            return {
                "state": "unknown",
                "message": "collision precheck incomplete",
                "checked_samples": checked,
                "requested_samples": len(selected),
                "collisions": [],
            }
        return {
            "state": "pass",
            "message": "no collision detected in sampled teach trajectory",
            "checked_samples": checked,
            "requested_samples": len(selected),
            "collisions": [],
        }

    def _teach_trajectory(self, record_path: str | None = None, max_points: int = 500) -> dict:
        path = record_path or str(self._teach_record_info(None).get("path", self.get_parameter("record_path").value))
        try:
            samples = load_teach_samples(path)
        except Exception as exc:
            return {
                "accepted": False,
                "message": f"failed to load teach trajectory: {exc}",
                "path": str(path),
                "points": [],
            }
        payload = teach_trajectory_preview_to_dict(samples, max_points=max_points)
        prepared = self._prepare_teach_replay_samples(samples)
        payload["prepared_replay"] = prepared_teach_replay_to_dict(prepared)
        payload["collision_precheck"] = self._collision_precheck(prepared.samples)
        payload["prepared_points"] = teach_trajectory_preview_to_dict(
            prepared.samples,
            max_points=max_points,
        )["points"]
        payload["accepted"] = True
        payload["path"] = str(path)
        payload["info"] = self._teach_record_info(str(path))
        return payload

    def _teach_records(self) -> dict:
        record_path = Path(str(self.get_parameter("record_path").value))
        directory = record_path.parent if str(record_path.parent) else Path("teleop_records")
        records = list_teach_record_files(directory)
        return {
            "directory": str(directory),
            "default_record_path": str(record_path),
            "records": records,
        }

    def _handle_teach_dry_run(self, payload: dict) -> dict:
        record_path = payload.get("record_path")
        settings = self._teach_replay_settings_from_payload(payload)
        info_payload = self._teach_record_info(str(record_path) if record_path else None)
        quality = info_payload.get("quality") if isinstance(info_payload.get("quality"), dict) else {}
        decision = validate_teach_dry_run_request(str(info_payload.get("start_band", "")))
        prepared_payload = {}
        collision_precheck = {"state": "unknown", "message": "collision precheck not run"}
        moveit_align = self._moveit_align_summary(info_payload)
        samples_for_precheck = []
        trajectory_points = 0
        try:
            samples_for_precheck = load_teach_samples(str(info_payload.get("path", "")))
            prepared = self._prepare_teach_replay_samples(samples_for_precheck, settings)
            prepared_payload = prepared_teach_replay_to_dict(prepared)
            moveit_align = self._moveit_align_summary(info_payload, samples_for_precheck, plan=decision.accepted)
            if decision.accepted and str(moveit_align.get("state", "")).lower() not in ("failed", "unavailable", "unknown"):
                trajectory = self._build_teach_replay_trajectory(
                    samples_for_precheck,
                    str(info_payload.get("start_band", "")),
                    settings,
                )
                trajectory_points = len(trajectory.points)
                collision_precheck = self._collision_precheck_trajectory(trajectory)
        except Exception as exc:
            prepared_payload = {"error": str(exc)}
            collision_precheck = {"state": "unknown", "message": f"collision precheck failed: {exc}"}
        prepared_quality = prepared_payload.get("after_quality") if isinstance(prepared_payload.get("after_quality"), dict) else {}
        gate_blocked = (
            str(moveit_align.get("state", "")).lower() in ("failed", "unavailable")
            or str(collision_precheck.get("state", "")).lower() in ("collision", "unknown")
        )
        estimate = estimate_teach_replay(
            samples=int(info_payload.get("samples") or 0),
            record_duration_sec=float(info_payload.get("duration_sec") or 0.0),
            start_band=str(info_payload.get("start_band", "")),
            replay_speed=float(settings["replay_speed"]),
            align_duration=float(settings["align_duration"]),
            align_steps=int(settings["align_steps"]),
            final_hold_sec=float(settings["final_hold_sec"]),
        )
        result = {
            "accepted": bool(decision.accepted) and not gate_blocked,
            "state": "blocked" if decision.accepted and gate_blocked else decision.state,
            "message": (
                f"{decision.message}; MoveIt/collision precheck blocked real replay"
                if decision.accepted and gate_blocked
                else decision.message
            ),
            "record_path": str(info_payload.get("path", "")),
            "start_band": str(info_payload.get("start_band", "")),
            "max_error": info_payload.get("max_error"),
            "worst_joint": str(info_payload.get("worst_joint", "")),
            "samples": int(info_payload.get("samples") or 0),
            "trajectory_points": int(trajectory_points or prepared_payload.get("prepared_samples") or estimate["trajectory_points"]),
            "estimated_duration_sec": float(estimate["estimated_duration_sec"]),
            "settings": settings,
            "quality": quality,
            "risk_level": str(quality.get("risk_level", "unknown")),
            "prepared_risk_level": str(prepared_quality.get("risk_level", "unknown")),
            "effective_risk_level": str(prepared_quality.get("risk_level", quality.get("risk_level", "unknown"))),
            "prepared_max_jump_rad": prepared_quality.get("max_jump_rad"),
            "retimed_max_acceleration_rad_s2": prepared_quality.get("max_acceleration_rad_s2"),
            "retimed_max_jerk_rad_s3": prepared_quality.get("max_jerk_rad_s3"),
            "max_prepared_jump_rad": float(self.get_parameter("max_prepared_jump_rad").value),
            "max_replay_acceleration_rad_s2": float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            "max_replay_jerk_rad_s3": float(self.get_parameter("max_replay_jerk_rad_s3").value),
            "prepared_replay": prepared_payload,
            "moveit_start_align": moveit_align,
            "collision_precheck": collision_precheck,
            "target_runtime": "hardware" if bool(self.get_parameter("use_hardware").value) else "simulation",
            "dry_run": True,
        }
        result = self._compact_replay_payload(result)
        self._last_teach_dry_run = result if result["accepted"] else None
        self._store.update_teleop_status("replay", result)
        return result

    def _handle_teach_replay_execute(self, payload: dict) -> dict:
        if not bool(self.get_parameter("web_execute_enabled").value):
            message = "web teach replay disabled; launch with web_execute_enabled:=true"
            self._store.update_teleop_status("replay", {"state": "blocked", "message": message})
            return {"accepted": False, "state": "blocked", "message": message}
        record_path = payload.get("record_path")
        settings = self._teach_replay_settings_from_payload(payload)
        info_payload = self._teach_record_info(str(record_path) if record_path else None)
        quality = info_payload.get("quality") if isinstance(info_payload.get("quality"), dict) else {}
        prepared_payload = {}
        prepared_quality = {}
        collision_precheck = {"state": "unknown", "message": "collision precheck not run"}
        moveit_align = self._moveit_align_summary(info_payload)
        trajectory = None
        try:
            source_samples = load_teach_samples(str(info_payload.get("path", "")))
            prepared = self._prepare_teach_replay_samples(source_samples, settings)
            prepared_payload = prepared_teach_replay_to_dict(prepared)
            prepared_quality = prepared_payload.get("after_quality") if isinstance(prepared_payload.get("after_quality"), dict) else {}
            moveit_align = self._moveit_align_summary(info_payload, source_samples, plan=False)
        except Exception as exc:
            prepared_payload = {"error": str(exc)}
            collision_precheck = {"state": "unknown", "message": f"collision precheck failed: {exc}"}
        token = self._last_teach_dry_run or {}
        token_error = token.get("max_error")
        info_error = info_payload.get("max_error")
        error_matches = token_error == info_error
        if token_error is not None and info_error is not None:
            error_matches = abs(float(token_error) - float(info_error)) < 1e-4
        dry_run_passed = (
            bool(token.get("accepted"))
            and str(token.get("record_path", "")) == str(info_payload.get("path", ""))
            and str(token.get("start_band", "")) == str(info_payload.get("start_band", ""))
            and str(token.get("worst_joint", "")) == str(info_payload.get("worst_joint", ""))
            and str(token.get("risk_level", "")) == str(quality.get("risk_level", ""))
            and str(token.get("prepared_risk_level", "")) == str(prepared_quality.get("risk_level", ""))
            and token.get("settings") == settings
            and error_matches
        )
        decision = validate_teach_replay_execute_request(
            str(info_payload.get("start_band", "")),
            dry_run_passed=dry_run_passed,
            risk_level=str(quality.get("risk_level", "unknown")),
            prepared_risk_level=str(prepared_quality.get("risk_level", "")) or None,
            prepared_max_jump_rad=prepared_quality.get("max_jump_rad"),
            max_prepared_jump_rad=float(self.get_parameter("max_prepared_jump_rad").value),
            retimed_max_acceleration_rad_s2=prepared_quality.get("max_acceleration_rad_s2"),
            max_replay_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            retimed_max_jerk_rad_s3=prepared_quality.get("max_jerk_rad_s3"),
            max_replay_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
            replay_speed=float(settings["replay_speed"]),
            yellow_max_speed=float(self.get_parameter("yellow_max_speed").value),
        )
        moveit_state = str(moveit_align.get("state", "")).lower()
        if decision.accepted and moveit_state in ("failed", "unavailable", "unknown"):
            decision = type(decision)(
                accepted=False,
                state="blocked",
                message=f"MoveIt start alignment not ready: {moveit_align.get('message', moveit_state)}",
            )
        if decision.accepted:
            try:
                samples = load_teach_samples(str(info_payload["path"]))
                if not samples:
                    raise ValueError("record contains no samples")
                trajectory = self._build_teach_replay_trajectory(samples, str(info_payload.get("start_band", "")), settings)
                collision_precheck = self._collision_precheck_trajectory(trajectory)
            except Exception as exc:
                collision_precheck = {"state": "unknown", "message": f"collision precheck failed: {exc}"}
        precheck_state = str(collision_precheck.get("state", "")).lower()
        if decision.accepted and precheck_state in ("collision", "unknown"):
            decision = type(decision)(
                accepted=False,
                state="blocked",
                message=f"collision precheck blocked replay: {collision_precheck.get('message', precheck_state)}",
            )
        if not decision.accepted:
            result = {
                "accepted": False,
                "state": decision.state,
                "message": decision.message,
                "record_path": str(info_payload.get("path", "")),
                "start_band": str(info_payload.get("start_band", "")),
                "max_error": info_payload.get("max_error"),
                "quality": quality,
                "risk_level": str(quality.get("risk_level", "unknown")),
                "prepared_risk_level": str(prepared_quality.get("risk_level", "unknown")),
                "effective_risk_level": str(prepared_quality.get("risk_level", quality.get("risk_level", "unknown"))),
                "prepared_max_jump_rad": prepared_quality.get("max_jump_rad"),
                "retimed_max_acceleration_rad_s2": prepared_quality.get("max_acceleration_rad_s2"),
                "retimed_max_jerk_rad_s3": prepared_quality.get("max_jerk_rad_s3"),
                "max_prepared_jump_rad": float(self.get_parameter("max_prepared_jump_rad").value),
                "max_replay_acceleration_rad_s2": float(self.get_parameter("max_replay_acceleration_rad_s2").value),
                "max_replay_jerk_rad_s3": float(self.get_parameter("max_replay_jerk_rad_s3").value),
                "prepared_replay": prepared_payload,
                "moveit_start_align": moveit_align,
                "collision_precheck": collision_precheck,
                "target_runtime": "hardware" if bool(self.get_parameter("use_hardware").value) else "simulation",
                "dry_run": False,
            }
            result = self._compact_replay_payload(result)
            self._store.update_teleop_status("replay", result)
            return result
        if not self._action_client.wait_for_server(timeout_sec=0.1):
            message = "follow_joint_trajectory action unavailable"
            self._store.update_teleop_status("replay", {"state": "unavailable", "message": message})
            return {"accepted": False, "message": message}
        if trajectory is None:
            result = {
                "accepted": False,
                "state": "blocked",
                "message": "failed to build replay trajectory",
                "record_path": str(info_payload.get("path", "")),
                "start_band": str(info_payload.get("start_band", "")),
                "moveit_start_align": moveit_align,
                "collision_precheck": collision_precheck,
                "prepared_replay": prepared_payload,
                "dry_run": False,
            }
            result = self._compact_replay_payload(result)
            self._store.update_teleop_status("replay", result)
            return result
        prepared_payload = getattr(self, "_last_teach_prepared_payload", prepared_payload)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_teach_replay_goal_response(fut, info_payload, len(trajectory.points)))
        result = {
            "accepted": True,
            "state": "replaying",
            "message": decision.message,
            "record_path": str(info_payload.get("path", "")),
            "start_band": str(info_payload.get("start_band", "")),
            "max_error": info_payload.get("max_error"),
            "worst_joint": str(info_payload.get("worst_joint", "")),
            "samples": int(info_payload.get("samples") or 0),
            "trajectory_points": len(trajectory.points),
            "estimated_duration_sec": float(estimate_teach_replay(
                samples=int(info_payload.get("samples") or 0),
                record_duration_sec=float(info_payload.get("duration_sec") or 0.0),
                start_band=str(info_payload.get("start_band", "")),
                replay_speed=float(settings["replay_speed"]),
                align_duration=float(settings["align_duration"]),
                align_steps=int(settings["align_steps"]),
                final_hold_sec=float(settings["final_hold_sec"]),
            )["estimated_duration_sec"]),
            "settings": settings,
            "quality": quality,
            "risk_level": str(quality.get("risk_level", "unknown")),
            "prepared_risk_level": str(prepared_quality.get("risk_level", "unknown")),
            "effective_risk_level": str(prepared_quality.get("risk_level", quality.get("risk_level", "unknown"))),
            "prepared_max_jump_rad": prepared_quality.get("max_jump_rad"),
            "retimed_max_acceleration_rad_s2": prepared_quality.get("max_acceleration_rad_s2"),
            "retimed_max_jerk_rad_s3": prepared_quality.get("max_jerk_rad_s3"),
            "max_prepared_jump_rad": float(self.get_parameter("max_prepared_jump_rad").value),
            "max_replay_acceleration_rad_s2": float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            "max_replay_jerk_rad_s3": float(self.get_parameter("max_replay_jerk_rad_s3").value),
            "prepared_replay": prepared_payload,
            "moveit_start_align": moveit_align,
            "collision_precheck": collision_precheck,
            "target_runtime": "hardware" if bool(self.get_parameter("use_hardware").value) else "simulation",
            "dry_run": False,
        }
        result = self._compact_replay_payload(result)
        self._store.update_teleop_status("replay", result)
        return result

    def _handle_teach_replay_stop(self) -> dict:
        with self._teach_replay_lock:
            goal_handle = self._teach_replay_goal_handle
        decision = validate_teach_replay_stop_request(goal_handle is not None)
        if not decision.accepted:
            stop_requested = self._request_controller_trajectory_stop(timeout_sec=0.8)
            if stop_requested:
                result = {
                    "accepted": True,
                    "state": "stop_requested",
                    "message": "controller trajectory_stop requested",
                }
            else:
                result = {"accepted": False, "state": decision.state, "message": decision.message}
            self._store.update_teleop_status("replay", result)
            return result
        try:
            future = goal_handle.cancel_goal_async()
            future.add_done_callback(self._on_teach_replay_cancel_response)
            self._request_controller_trajectory_stop(timeout_sec=0.2)
        except Exception as exc:
            result = {"accepted": False, "state": "failed", "message": f"failed to request teach replay cancel: {exc}"}
            self._store.update_teleop_status("replay", result)
            return result
        result = {"accepted": True, "state": decision.state, "message": decision.message}
        self._store.update_teleop_status("replay", result)
        return result

    def _request_controller_trajectory_stop(self, *, timeout_sec: float) -> bool:
        try:
            if not self._trajectory_stop_client.wait_for_service(timeout_sec=min(timeout_sec, 0.2)):
                return False
            self._trajectory_stop_client.call_async(Trigger.Request())
            return True
        except Exception:
            return False

    def _call_trigger_service(self, client, *, timeout_sec: float) -> tuple[bool, str]:
        try:
            if not client.wait_for_service(timeout_sec=min(timeout_sec, 0.5)):
                return False, "service unavailable"
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + max(float(timeout_sec), 0.1)
            while time.monotonic() < deadline:
                if future.done():
                    response = future.result()
                    return bool(getattr(response, "success", False)), str(getattr(response, "message", ""))
                time.sleep(0.02)
            return False, "service timeout"
        except Exception as exc:
            return False, str(exc)

    def _handle_arm_service_command(self, command: str) -> dict:
        if not bool(self.get_parameter("web_execute_enabled").value):
            message = "web arm command disabled; launch with web_execute_enabled:=true"
            result = {"accepted": False, "state": "blocked", "command": command, "message": message}
            self._store.update_teleop_status("arm_command", result)
            return result
        clients = {
            "safe_home": self._arm_safe_home_client,
            "enable": self._arm_enable_client,
            "disable": self._arm_disable_client,
        }
        if command not in clients:
            result = {"accepted": False, "state": "rejected", "command": command, "message": "unknown arm command"}
            self._store.update_teleop_status("arm_command", result)
            return result
        timeout_sec = {"safe_home": 30.0, "enable": 8.0, "disable": 10.0}[command]
        ok, message = self._call_trigger_service(clients[command], timeout_sec=timeout_sec)
        result = {
            "accepted": ok,
            "state": "done" if ok else "failed",
            "command": command,
            "message": message or ("done" if ok else "failed"),
        }
        if ok:
            arm = self._store.snapshot().arm
            enabled = bool(arm.get("enabled", False))
            mode = str(arm.get("mode", ""))
            if command == "disable":
                enabled = False
            elif command in ("enable", "safe_home"):
                enabled = True
            if command == "safe_home":
                mode = mode or "pos_vel"
            self._store.update_arm_status(
                mode=mode,
                enabled=enabled,
                state_machine="IDLE",
                error_codes=tuple(str(v) for v in arm.get("error_codes", [])),
            )
        self._store.update_teleop_status("arm_command", result)
        return result

    def _handle_teach_record_start(self, payload: dict | None = None) -> dict:
        payload = payload or {}
        requested_path = str(payload.get("record_path", "")).strip()
        set_path_ok = True
        set_path_message = ""
        normalized_path = ""
        if requested_path:
            try:
                if not self._teach_record_set_path_client.wait_for_service(timeout_sec=0.5):
                    set_path_ok = False
                    set_path_message = "record path service unavailable"
                else:
                    request = SetTeachRecordPath.Request()
                    request.record_path = requested_path
                    future = self._teach_record_set_path_client.call_async(request)
                    deadline = time.monotonic() + 2.0
                    while time.monotonic() < deadline and not future.done():
                        time.sleep(0.02)
                    if future.done():
                        response = future.result()
                        set_path_ok = bool(getattr(response, "success", False))
                        set_path_message = str(getattr(response, "message", ""))
                        normalized_path = str(getattr(response, "normalized_path", ""))
                    else:
                        set_path_ok = False
                        set_path_message = "record path service timeout"
            except Exception as exc:
                set_path_ok = False
                set_path_message = str(exc)
        if not set_path_ok:
            result = {
                "accepted": False,
                "state": "blocked",
                "message": f"record path: {set_path_message}",
                "record_path": normalized_path or requested_path,
            }
            self._store.update_teleop_status("recording", result)
            return result
        gravity_ok, gravity_message = self._call_trigger_service(
            self._gravity_start_client,
            timeout_sec=2.0,
        )
        record_ok, record_message = self._call_trigger_service(
            self._teach_record_start_client,
            timeout_sec=2.0,
        )
        accepted = record_ok and (gravity_ok or "already" in gravity_message.lower())
        result = {
            "accepted": accepted,
            "state": "starting" if accepted else "blocked",
            "message": f"gravity: {gravity_message or gravity_ok}; record: {record_message or record_ok}",
            "gravity_started": gravity_ok,
            "record_started": record_ok,
            "record_path": normalized_path or requested_path,
        }
        self._store.update_teleop_status("recording", result)
        return result

    def _handle_teach_record_stop(self) -> dict:
        record_ok, record_message = self._call_trigger_service(
            self._teach_record_stop_client,
            timeout_sec=2.0,
        )
        gravity_ok, gravity_message = self._call_trigger_service(
            self._gravity_stop_client,
            timeout_sec=2.0,
        )
        accepted = record_ok
        result = {
            "accepted": accepted,
            "state": "stopped" if accepted else "failed",
            "message": f"record: {record_message or record_ok}; gravity: {gravity_message or gravity_ok}",
            "record_stopped": record_ok,
            "gravity_stopped": gravity_ok,
        }
        self._store.update_teleop_status("recording", result)
        return result

    def _teach_replay_settings_from_payload(self, payload: dict) -> dict[str, float | int]:
        values = payload.get("settings") if isinstance(payload.get("settings"), dict) else {}
        return normalize_teach_replay_settings(
            replay_speed=float(values.get("replay_speed", self.get_parameter("replay_speed").value)),
            align_duration=float(values.get("align_duration", self.get_parameter("align_duration").value)),
            align_steps=int(values.get("align_steps", self.get_parameter("align_steps").value)),
            final_hold_sec=float(values.get("final_hold_sec", self.get_parameter("final_hold_sec").value)),
        )

    def _build_teach_replay_trajectory(self, samples, start_band: str, settings: dict[str, float | int]) -> JointTrajectory:
        prepared = self._prepare_teach_replay_samples(samples, settings)
        self._last_teach_prepared_payload = prepared_teach_replay_to_dict(prepared)
        replay_samples = prepared.samples
        first = replay_samples[0]
        trajectory = JointTrajectory()
        trajectory.joint_names = list(first.joint_names)
        snapshot = self._store.snapshot()
        current_map = {
            name: float(data["position"])
            for name, data in snapshot.joints.items()
            if "position" in data
        }
        current_positions = tuple(current_map.get(name, start) for name, start in zip(first.joint_names, first.positions))
        if bool(self.get_parameter("use_moveit_start_align").value):
            elapsed = self._append_moveit_start_alignment(
                trajectory,
                current_positions=current_positions,
                first_positions=first.positions,
            )
        else:
            start_points = build_replay_start_soft_points(
                current_positions=current_positions,
                first_positions=first.positions,
                start_band=start_band,
                start_hold_sec=float(self.get_parameter("start_hold_sec").value),
                soft_start_duration=float(self.get_parameter("soft_start_duration").value),
                soft_start_steps=int(self.get_parameter("soft_start_steps").value),
                align_duration=float(settings["align_duration"]),
                align_steps=int(settings["align_steps"]),
                first_hold_sec=float(self.get_parameter("first_hold_sec").value),
            )
            for start_point in start_points:
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in start_point.positions]
                point.velocities = [0.0 for _ in start_point.positions]
                _set_duration(point.time_from_start, start_point.time_from_start)
                trajectory.points.append(point)
            elapsed = start_points[-1].time_from_start if start_points else 0.0
        speed = max(float(prepared.effective_replay_speed), 0.01)
        prepared_quality = prepared.after_quality
        if str(prepared_quality.risk_level) == "yellow":
            speed = min(speed, float(self.get_parameter("yellow_max_speed").value))
        for retimed in retime_teach_samples(
            replay_samples,
            replay_speed=speed,
            max_velocity_rad_s=float(self.get_parameter("max_replay_velocity_rad_s").value),
            max_acceleration_rad_s2=float(self.get_parameter("max_replay_acceleration_rad_s2").value),
            max_jerk_rad_s3=float(self.get_parameter("max_replay_jerk_rad_s3").value),
            initial_delay_sec=float(self.get_parameter("initial_replay_delay_sec").value),
            boundary_zero_velocity=True,
        ):
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in retimed.positions]
            if retimed.velocities:
                point.velocities = [float(v) for v in retimed.velocities]
            _set_duration(point.time_from_start, elapsed + retimed.time_from_start)
            trajectory.points.append(point)
        self._append_final_hold(trajectory, final_hold_sec=float(settings["final_hold_sec"]))
        return trajectory

    def _append_final_hold(self, trajectory: JointTrajectory, *, final_hold_sec: float) -> None:
        final_hold = max(float(final_hold_sec), 0.0)
        if final_hold <= 0.0 or not trajectory.points:
            return
        last_point = trajectory.points[-1]
        last_time = float(last_point.time_from_start.sec) + float(last_point.time_from_start.nanosec) * 1e-9
        hold_point = JointTrajectoryPoint()
        hold_point.positions = [float(v) for v in last_point.positions]
        hold_point.velocities = [0.0 for _ in hold_point.positions]
        _set_duration(hold_point.time_from_start, last_time + final_hold)
        trajectory.points.append(hold_point)

    def _append_moveit_start_alignment(
        self,
        trajectory: JointTrajectory,
        *,
        current_positions: tuple[float, ...],
        first_positions: tuple[float, ...],
    ) -> float:
        elapsed = max(float(self.get_parameter("start_hold_sec").value), 0.0)
        hold_point = JointTrajectoryPoint()
        hold_point.positions = [float(v) for v in current_positions]
        hold_point.velocities = [0.0 for _ in current_positions]
        _set_duration(hold_point.time_from_start, elapsed)
        trajectory.points.append(hold_point)
        max_error = max(
            (abs(float(a) - float(b)) for a, b in zip(current_positions, first_positions)),
            default=0.0,
        )
        if max_error >= float(self.get_parameter("moveit_start_skip_threshold").value):
            plan = self._moveit_planner.plan_joint_positions(
                joint_names=tuple(trajectory.joint_names),
                target_positions=first_positions,
                tolerance=float(self.get_parameter("moveit_joint_goal_tolerance").value),
                velocity_scaling=float(self.get_parameter("moveit_velocity_scaling").value),
                acceleration_scaling=float(self.get_parameter("moveit_acceleration_scaling").value),
            )
            if not plan.success or plan.trajectory is None:
                raise ValueError(f"moveit start alignment failed: {plan.message}")
            source_names = list(getattr(plan.trajectory, "joint_names", []))
            index_by_name = {name: index for index, name in enumerate(source_names)}
            missing = [name for name in trajectory.joint_names if name not in index_by_name]
            if missing:
                raise ValueError(f"moveit start alignment missing joints: {', '.join(missing)}")
            for source_point in getattr(plan.trajectory, "points", []):
                source_time = float(source_point.time_from_start.sec) + float(source_point.time_from_start.nanosec) * 1e-9
                point = JointTrajectoryPoint()
                point.positions = [
                    float(source_point.positions[index_by_name[name]])
                    for name in trajectory.joint_names
                ]
                if getattr(source_point, "velocities", None):
                    point.velocities = [
                        float(source_point.velocities[index_by_name[name]])
                        for name in trajectory.joint_names
                    ]
                _set_duration(point.time_from_start, elapsed + source_time)
                trajectory.points.append(point)
            if trajectory.points:
                last = trajectory.points[-1].time_from_start
                elapsed = float(last.sec) + float(last.nanosec) * 1e-9
        first_hold = max(float(self.get_parameter("first_hold_sec").value), 0.0)
        if first_hold > 0.0:
            elapsed += first_hold
            first_point = JointTrajectoryPoint()
            first_point.positions = [float(v) for v in first_positions]
            first_point.velocities = [0.0 for _ in first_positions]
            _set_duration(first_point.time_from_start, elapsed)
            trajectory.points.append(first_point)
        return elapsed

    def _on_teach_replay_goal_response(self, future, info_payload: dict, points: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._store.update_teleop_status("replay", {"state": "failed", "message": str(exc)})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._store.update_teleop_status("replay", {"state": "rejected", "message": "teach replay goal rejected"})
            return
        with self._teach_replay_lock:
            self._teach_replay_goal_handle = goal_handle
        self._store.update_teleop_status(
            "replay",
            {
                "state": "replaying",
                "message": "teach replay goal accepted",
                "record_path": str(info_payload.get("path", "")),
                "start_band": str(info_payload.get("start_band", "")),
                "max_error": info_payload.get("max_error"),
                "trajectory_points": points,
                "dry_run": False,
            },
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_teach_replay_result(fut, info_payload, points))

    def _on_teach_replay_cancel_response(self, future) -> None:
        try:
            response = future.result()
            goals_canceling = len(getattr(response, "goals_canceling", []))
        except Exception as exc:
            self._store.update_teleop_status("replay", {"state": "failed", "message": str(exc)})
            return
        state = "cancel_requested" if goals_canceling else "done"
        message = (
            "teach replay cancel accepted"
            if goals_canceling
            else "teach replay already finished before cancel"
        )
        self._store.update_teleop_status("replay", {"state": state, "message": message})

    def _on_teach_replay_result(self, future, info_payload: dict, points: int) -> None:
        try:
            wrapped_result = future.result()
            status = int(getattr(wrapped_result, "status", -1))
            result = getattr(wrapped_result, "result", None)
            error_code = int(getattr(result, "error_code", 0)) if result is not None else 0
            error_string = str(getattr(result, "error_string", "")) if result is not None else ""
        except Exception as exc:
            self._store.update_teleop_status("replay", {"state": "failed", "message": str(exc)})
            with self._teach_replay_lock:
                self._teach_replay_goal_handle = None
            return
        if status == 4 and error_code == 0:
            state = "done"
        elif status == 5:
            state = "canceled"
        else:
            state = "failed"
        self._store.update_teleop_status(
            "replay",
            {
                "state": state,
                "message": f"teach replay result status={status}, error_code={error_code}: {error_string}",
                "record_path": str(info_payload.get("path", "")),
                "start_band": str(info_payload.get("start_band", "")),
                "max_error": info_payload.get("max_error"),
                "trajectory_points": points,
                "dry_run": False,
            },
        )
        with self._teach_replay_lock:
            self._teach_replay_goal_handle = None

    def _handle_keyboard_enable(self, payload: dict) -> dict:
        if not bool(self.get_parameter("web_execute_enabled").value):
            message = "web keyboard disabled; launch with web_execute_enabled:=true"
            self._store.update_teleop_status("status", {"source": "web_keyboard", "state": "blocked", "message": message})
            return {"accepted": False, "message": message}
        if str(self.get_parameter("panel_mode").value).lower() == "check":
            message = "web keyboard blocked: check mode is read-only"
            self._store.update_teleop_status("status", {"source": "web_keyboard", "state": "blocked", "message": message})
            return {"accepted": False, "message": message}
        step = _number_or_default(payload.get("step_rad"), self._web_keyboard_step_rad)
        duration = _number_or_default(payload.get("duration"), self._web_keyboard_duration)
        speed = _number_or_default(payload.get("max_joint_speed_rad_s"), self._web_keyboard_speed)
        with self._web_keyboard_lock:
            self._web_keyboard_step_rad = min(
                max(step, float(self.get_parameter("web_keyboard_min_step_rad").value)),
                float(self.get_parameter("web_keyboard_max_step_rad").value),
            )
            self._web_keyboard_duration = min(
                max(duration, float(self.get_parameter("web_keyboard_min_duration").value)),
                float(self.get_parameter("web_keyboard_max_duration").value),
            )
            self._web_keyboard_speed = min(
                max(speed, 0.05),
                float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
            )
            self._web_keyboard_enabled = True
        result = {
            "accepted": True,
            "source": "web_keyboard",
            "state": "ready",
            "message": "web keyboard teleop enabled",
            "step_rad": self._web_keyboard_step_rad,
            "duration": self._web_keyboard_duration,
            "max_joint_speed_rad_s": self._web_keyboard_speed,
        }
        self._store.update_teleop_status("status", result)
        return result

    def _handle_keyboard_disable(self) -> dict:
        with self._web_keyboard_lock:
            self._web_keyboard_enabled = False
        stop_requested = self._request_controller_trajectory_stop(timeout_sec=0.2)
        result = {
            "accepted": True,
            "source": "web_keyboard",
            "state": "disabled",
            "message": "web keyboard teleop disabled; trajectory_stop requested" if stop_requested else "web keyboard teleop disabled",
            "trajectory_stop_requested": stop_requested,
        }
        self._store.update_teleop_status("status", result)
        return result

    def _handle_keyboard_key(self, payload: dict) -> dict:
        with self._web_keyboard_lock:
            enabled = self._web_keyboard_enabled
            step_rad = self._web_keyboard_step_rad
            duration = self._web_keyboard_duration
            speed = self._web_keyboard_speed
        request_payload = dict(payload)
        request_payload.setdefault("step_rad", step_rad)
        request_payload.setdefault("duration", duration)
        request_payload.setdefault("max_joint_speed_rad_s", speed)
        snapshot = self._store.snapshot()
        current_positions = {
            name: float(data["position"])
            for name, data in snapshot.joints.items()
            if "position" in data
        }
        decision = validate_web_keyboard_command(
            request_payload,
            enabled=enabled,
            joint_names=self._joint_names,
            current_positions=current_positions,
            joint_limits=self._joint_limits,
            default_step_rad=float(self.get_parameter("web_keyboard_default_step_rad").value),
            min_step_rad=float(self.get_parameter("web_keyboard_min_step_rad").value),
            max_step_rad=float(self.get_parameter("web_keyboard_max_step_rad").value),
            default_duration=float(self.get_parameter("web_keyboard_default_duration").value),
            min_duration=float(self.get_parameter("web_keyboard_min_duration").value),
            max_duration=float(self.get_parameter("web_keyboard_max_duration").value),
            joint_velocity_limits=self._joint_velocity_limits,
            max_joint_speed_rad_s=float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
        )
        if not decision.accepted:
            self._store.update_teleop_status(
                "status",
                {"source": "web_keyboard", "state": "rejected", "message": decision.message, "last_key": payload.get("key")},
            )
            return _keyboard_decision_response(decision)
        if not self._action_client.wait_for_server(timeout_sec=0.05):
            message = "follow_joint_trajectory action unavailable"
            self._store.update_teleop_status("status", {"source": "web_keyboard", "state": "unavailable", "message": message, "last_key": decision.key})
            return {"accepted": False, "message": message}
        trajectory = JointTrajectory()
        trajectory.joint_names = list(decision.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in decision.positions]
        _set_duration(point.time_from_start, decision.duration)
        trajectory.points = [point]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_keyboard_goal_response(fut, decision))
        result = _keyboard_decision_response(decision)
        self._store.update_teleop_status(
            "status",
            {
                **result,
                "source": "web_keyboard",
                "state": "active",
                "last_key": decision.key,
            },
        )
        return result

    def _on_keyboard_goal_response(self, future, decision) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._store.update_teleop_status("status", {"source": "web_keyboard", "state": "failed", "message": str(exc), "last_key": decision.key})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._store.update_teleop_status("status", {"source": "web_keyboard", "state": "rejected", "message": "keyboard trajectory goal rejected", "last_key": decision.key})
            return
        self._store.update_teleop_status(
            "status",
            {
                "source": "web_keyboard",
                "state": "accepted",
                "message": decision.message,
                "last_key": decision.key,
                "joint_name": decision.joint_name,
                "step_rad": decision.step_rad,
                "duration": decision.duration,
                "max_joint_speed_rad_s": decision.max_joint_speed_rad_s,
            },
        )

    def _handle_execute_preview(self, payload: dict) -> dict:
        if not bool(self.get_parameter("web_execute_enabled").value):
            return {
                "accepted": False,
                "message": "web execute disabled; launch with web_execute_enabled:=true",
            }
        snapshot = self._store.snapshot()
        current_positions = {
            name: float(data["position"])
            for name, data in snapshot.joints.items()
            if "position" in data
        }
        decision = validate_web_execute_request(
            payload,
            joint_names=self._joint_names,
            current_positions=current_positions,
            joint_limits=self._joint_limits,
            max_delta_rad=float(self.get_parameter("web_execute_max_delta_rad").value),
            min_duration=float(self.get_parameter("web_execute_min_duration").value),
            max_duration=float(self.get_parameter("web_execute_max_duration").value),
            joint_velocity_limits=self._joint_velocity_limits,
            max_joint_speed_rad_s=float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
        )
        if not decision.accepted:
            self._store.update_teleop_status("web_execute", {"state": "rejected", "message": decision.message})
            return _decision_response(decision)
        if not self._action_client.wait_for_server(timeout_sec=0.1):
            message = "follow_joint_trajectory action unavailable"
            self._store.update_teleop_status("web_execute", {"state": "unavailable", "message": message})
            return {"accepted": False, "message": message}

        trajectory = JointTrajectory()
        trajectory.joint_names = list(decision.joint_names)
        current = tuple(current_positions[name] for name in decision.joint_names)
        for elapsed, positions in interpolate_joint_points(
            current=current,
            target=decision.positions,
            duration=decision.duration,
        ):
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in positions]
            _set_duration(point.time_from_start, elapsed)
            trajectory.points.append(point)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_execute_goal_response(fut, decision))
        self._store.update_teleop_status(
            "web_execute",
            {
                "state": "active",
                "message": decision.message,
                "max_delta": decision.max_delta,
                "max_delta_limit": decision.max_delta_limit,
                "duration": decision.duration,
                "max_joint_speed_rad_s": float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
                "points": len(trajectory.points),
            },
        )
        return _decision_response(decision)

    def _handle_stop_execute(self) -> dict:
        with self._execute_lock:
            goal_handle = self._execute_goal_handle
        stop_requested = self._request_controller_trajectory_stop(timeout_sec=0.2)
        if goal_handle is None:
            message = (
                "no active web execute goal; controller trajectory_stop requested"
                if stop_requested
                else "no active web execute goal; controller trajectory_stop unavailable"
            )
            state = "cancel_requested" if stop_requested else "idle"
            self._store.update_teleop_status(
                "web_execute",
                {
                    "state": state,
                    "message": message,
                    "trajectory_stop_requested": stop_requested,
                },
            )
            return {
                "accepted": bool(stop_requested),
                "state": state,
                "message": message,
                "trajectory_stop_requested": stop_requested,
            }
        try:
            future = goal_handle.cancel_goal_async()
            future.add_done_callback(self._on_execute_cancel_response)
        except Exception as exc:
            message = f"failed to request trajectory cancel: {exc}"
            if stop_requested:
                message = f"{message}; controller trajectory_stop requested"
                self._store.update_teleop_status(
                    "web_execute",
                    {
                        "state": "cancel_requested",
                        "message": message,
                        "trajectory_stop_requested": True,
                    },
                )
                return {"accepted": True, "message": message, "trajectory_stop_requested": True}
            self._store.update_teleop_status("web_execute", {"state": "failed", "message": message})
            return {"accepted": False, "message": message, "trajectory_stop_requested": False}
        message = (
            "trajectory cancel requested; controller trajectory_stop requested"
            if stop_requested
            else "trajectory cancel requested; controller trajectory_stop unavailable"
        )
        self._store.update_teleop_status(
            "web_execute",
            {
                "state": "cancel_requested",
                "message": message,
                "trajectory_stop_requested": stop_requested,
            },
        )
        with self._execute_lock:
            self._execute_goal_handle = None
        return {"accepted": True, "message": message, "trajectory_stop_requested": stop_requested}

    def _handle_set_gripper(self, payload: dict) -> dict:
        if not bool(self.get_parameter("web_execute_enabled").value):
            return {
                "accepted": False,
                "message": "web gripper disabled; launch with web_execute_enabled:=true",
            }
        decision = validate_web_gripper_request(
            payload,
            gripper_limits=self._gripper_limits,
            default_max_effort=float(self.get_parameter("web_gripper_max_effort").value),
            max_effort_limit=float(self.get_parameter("web_gripper_max_effort_limit").value),
        )
        if not decision.accepted:
            self._store.update_teleop_status("web_gripper", {"state": "rejected", "message": decision.message})
            return _gripper_decision_response(decision)
        if not self._use_hardware:
            self._sim_gripper_position = float(decision.position)
            self._publish_simulated_gripper_state()
            self._store.update_teleop_status(
                "web_gripper",
                {
                    "state": "done",
                    "message": f"simulated gripper position={decision.position:.4f} m",
                    "position": decision.position,
                    "max_effort": decision.max_effort,
                    "simulated": True,
                },
            )
            return _gripper_decision_response(decision)
        if not self._gripper_action_client.wait_for_server(timeout_sec=0.1):
            message = "gripper command action unavailable"
            self._store.update_teleop_status("web_gripper", {"state": "unavailable", "message": message})
            return {"accepted": False, "message": message}

        goal = GripperCommand.Goal()
        goal.command.position = float(decision.position)
        goal.command.max_effort = float(decision.max_effort)
        future = self._gripper_action_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_gripper_goal_response(fut, decision))
        self._store.update_teleop_status(
            "web_gripper",
            {
                "state": "active",
                "message": decision.message,
                "position": decision.position,
                "max_effort": decision.max_effort,
            },
        )
        return _gripper_decision_response(decision)

    def _on_gripper_goal_response(self, future, decision: WebGripperDecision) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._store.update_teleop_status("web_gripper", {"state": "failed", "message": str(exc)})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._store.update_teleop_status(
                "web_gripper",
                {"state": "rejected", "message": "gripper goal rejected"},
            )
            return
        self._store.update_teleop_status(
            "web_gripper",
            {
                "state": "accepted",
                "message": "gripper goal accepted by controller",
                "position": decision.position,
                "max_effort": decision.max_effort,
            },
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_gripper_result(fut, decision))

    def _on_gripper_result(self, future, decision: WebGripperDecision) -> None:
        try:
            result_response = future.result()
            result = result_response.result
            reached = bool(getattr(result, "reached_goal", False))
            position = float(getattr(result, "position", decision.position))
            effort = float(getattr(result, "effort", 0.0))
        except Exception as exc:
            self._store.update_teleop_status("web_gripper", {"state": "failed", "message": str(exc)})
            return
        self._store.update_teleop_status(
            "web_gripper",
            {
                "state": "done" if reached else "failed",
                "message": f"gripper result reached={reached}",
                "position": position,
                "max_effort": decision.max_effort,
                "effort": effort,
            },
        )

    def _publish_simulated_gripper_state(self) -> None:
        if self._sim_gripper_state_pub is None:
            return
        msg = JointMotorState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_name = "gripper"
        msg.position = float(self._sim_gripper_position)
        msg.velocity = 0.0
        msg.torque = 0.0
        msg.status_code = 1
        self._sim_gripper_state_pub.publish(msg)

    def _on_execute_cancel_response(self, future) -> None:
        try:
            response = future.result()
            goals_canceling = len(getattr(response, "goals_canceling", []))
        except Exception as exc:
            self._store.update_teleop_status("web_execute", {"state": "failed", "message": str(exc)})
            return
        state = "cancel_requested" if goals_canceling else "done"
        message = (
            "trajectory cancel accepted"
            if goals_canceling
            else "trajectory already finished before cancel"
        )
        self._store.update_teleop_status("web_execute", {"state": state, "message": message})

    def _on_execute_goal_response(self, future, decision: WebExecuteDecision) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._store.update_teleop_status("web_execute", {"state": "failed", "message": str(exc)})
            return
        if goal_handle is None or not goal_handle.accepted:
            self._store.update_teleop_status(
                "web_execute",
                {"state": "rejected", "message": "trajectory goal rejected"},
            )
            return
        with self._execute_lock:
            self._execute_goal_handle = goal_handle
        self._store.update_teleop_status(
            "web_execute",
            {
                "state": "accepted",
                "message": "trajectory goal accepted by controller",
                "max_delta": decision.max_delta,
                "max_delta_limit": decision.max_delta_limit,
                "duration": decision.duration,
                "max_joint_speed_rad_s": float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
            },
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_execute_result(fut, decision))

    def _on_execute_result(self, future, decision: WebExecuteDecision) -> None:
        try:
            result_response = future.result()
            result = result_response.result
            status = int(result_response.status)
            error_code = int(getattr(result, "error_code", 0))
            error_string = str(getattr(result, "error_string", ""))
        except Exception as exc:
            self._store.update_teleop_status("web_execute", {"state": "failed", "message": str(exc)})
            with self._execute_lock:
                self._execute_goal_handle = None
            return
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            state = "done"
        elif status == 5:
            state = "canceled"
        else:
            state = "failed"
        with self._execute_lock:
            self._execute_goal_handle = None
        self._store.update_teleop_status(
            "web_execute",
            {
                "state": state,
                "message": f"trajectory result status={status}, error_code={error_code}: {error_string}",
                "max_delta": decision.max_delta,
                "max_delta_limit": decision.max_delta_limit,
                "duration": decision.duration,
                "max_joint_speed_rad_s": float(self.get_parameter("web_execute_max_joint_speed_rad_s").value),
            },
        )

    def _on_joint_state(self, msg: JointState) -> None:
        self._store.update_joint_state(
            names=tuple(str(v) for v in msg.name),
            positions=tuple(float(v) for v in msg.position),
            velocities=tuple(float(v) for v in msg.velocity),
            efforts=tuple(float(v) for v in msg.effort),
        )

    def _on_motor_state(self, msg: JointMotorState) -> None:
        self._store.update_motor_state(
            joint_name=str(msg.joint_name),
            position=float(msg.position),
            velocity=float(msg.velocity),
            torque=float(msg.torque),
            status_code=int(msg.status_code),
        )

    def _on_gripper_state(self, msg: JointMotorState) -> None:
        joint_name = str(msg.joint_name).strip() or "gripper"
        self._store.update_motor_state(
            joint_name=joint_name,
            position=float(msg.position),
            velocity=float(msg.velocity),
            torque=float(msg.torque),
            status_code=int(msg.status_code),
        )
        if joint_name != "gripper":
            self._store.update_motor_state(
                joint_name="gripper",
                position=float(msg.position),
                velocity=float(msg.velocity),
                torque=float(msg.torque),
                status_code=int(msg.status_code),
            )

    def _on_arm_status(self, msg: ArmStatus) -> None:
        self._store.update_arm_status(
            mode=str(msg.mode),
            enabled=bool(msg.enabled),
            state_machine=str(msg.state_machine),
            error_codes=tuple(str(v) for v in msg.error_codes),
        )
        self._update_gravity_comp_status()

    def _on_status(self, key: str, msg: String) -> None:
        try:
            value = json.loads(msg.data)
        except Exception:
            value = msg.data
        self._store.update_teleop_status(key, value)

    def _update_gravity_comp_status(self) -> None:
        snapshot = self._store.snapshot()
        arm_state = str(snapshot.arm.get("state_machine", ""))
        try:
            start_available = bool(self._gravity_start_client.service_is_ready())
            if not start_available:
                start_available = bool(self._gravity_start_client.wait_for_service(timeout_sec=0.0))
        except Exception:
            start_available = False
        try:
            stop_available = bool(self._gravity_stop_client.service_is_ready())
            if not stop_available:
                stop_available = bool(self._gravity_stop_client.wait_for_service(timeout_sec=0.0))
        except Exception:
            stop_available = False
        active = arm_state == "GRAVITY_COMP"
        if active:
            state = "active"
            message = "gravity compensation is active"
        elif start_available:
            state = "ready"
            message = "gravity compensation start service is available"
        else:
            state = "unavailable"
            message = "gravity compensation services unavailable"
        recording = snapshot.teleop.get("recording")
        require_gravity = bool(recording.get("require_gravity_comp")) if isinstance(recording, dict) else True
        self._store.update_teleop_status(
            "gravity_comp",
            {
                "state": state,
                "message": message,
                "arm_state": arm_state,
                "start_service_available": start_available,
                "stop_service_available": stop_available,
                "active": active,
                "ready_for_teach_recording": (not require_gravity) or active,
                "recording_requires_gravity_comp": require_gravity,
            },
        )

    def destroy_node(self) -> bool:
        try:
            self._server.shutdown()
            self._server.server_close()
        finally:
            return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopStatusPanelNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        with suppress(KeyboardInterrupt):
            node.destroy_node()
        if rclpy.ok():
            with suppress(KeyboardInterrupt):
                rclpy.shutdown()


if __name__ == "__main__":
    main()
