#!/usr/bin/env python3
"""Check local package boundaries without importing ROS, SDKs, or project code.

This is a static guard, not a proof of runtime isolation. Constant imports,
known resource lookup calls, package:// URIs, and package.xml are inspected.
Computed import names and arbitrary launch helpers need manual review.
"""
from __future__ import annotations

import argparse
import ast
from graphlib import CycleError, TopologicalSorter
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET


RESOURCE_CALLS = {
    "FindPackageShare", "get_package_share_directory", "get_package_prefix",
    "_package_resource", "load_yaml",
}
RUNTIME_TAGS = {"depend", "exec_depend"}
DEPENDENCY_TAGS = RUNTIME_TAGS | {
    "build_depend", "build_export_depend", "buildtool_depend", "buildtool_export_depend",
}


def imported_modules(tree: ast.AST):
    """Include literal dynamic imports and aliases of importlib.import_module."""
    importlib_names = {"importlib"}
    dynamic_names = {"__import__"}
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name == "importlib":
                    importlib_names.add(alias.asname or alias.name)
                yield alias.name, node.lineno
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            if node.module == "importlib":
                dynamic_names.update(
                    alias.asname or alias.name for alias in node.names
                    if alias.name == "import_module"
                )
            if node.module:
                yield node.module, node.lineno
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not node.args:
            continue
        func = node.func
        dynamic = (
            isinstance(func, ast.Name) and func.id in dynamic_names
        ) or (
            isinstance(func, ast.Attribute) and func.attr == "import_module"
            and isinstance(func.value, ast.Name) and func.value.id in importlib_names
        )
        value = node.args[0]
        if dynamic and isinstance(value, ast.Constant) and isinstance(value.value, str):
            yield value.value, node.lineno


def resource_modules(tree: ast.AST):
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = getattr(node.func, "id", getattr(node.func, "attr", ""))
        values = [node.args[0]] if name in RESOURCE_CALLS and node.args else []
        values.extend(kw.value for kw in node.keywords if kw.arg in {"package", "package_name"})
        for value in values:
            if isinstance(value, ast.Constant) and isinstance(value.value, str):
                yield value.value, node.lineno


def cycle_message(graph: dict[str, set[str]]) -> str | None:
    try:
        tuple(TopologicalSorter({k: sorted(v) for k, v in sorted(graph.items())}).static_order())
    except CycleError as exc:
        return " -> ".join(exc.args[1])
    return None


def check_repository(root: Path, rules: dict) -> dict:
    errors: list[str] = []
    warnings: list[str] = []
    policy = rules["packages"]
    packages = {}
    runtime = {}
    declared = {}
    for path in sorted((root / "src").glob("*/package.xml")):
        manifest = ET.parse(path).getroot()
        name = manifest.findtext("name")
        if name in packages:
            errors.append(f"duplicate package: {name}")
        packages[name] = path.parent
        runtime[name] = {e.text for e in manifest if e.tag in RUNTIME_TAGS}
        declared[name] = {e.text for e in manifest if e.tag in DEPENDENCY_TAGS}
    for name in sorted(set(packages) ^ set(policy)):
        errors.append(f"package inventory differs from policy: {name}")
    local_names = set(packages) | set(policy)
    imports = {name: set() for name in packages}
    resources = {name: set() for name in packages}
    sites = {}
    python_files = 0
    sdk_debt = {(item["path"], item["module"]): item for item in rules.get("sdk_import_debt", [])}
    used_sdk_debt = set()

    def is_local(name):
        # Do not silently ignore a misspelled or newly introduced project package.
        return name in local_names or name.startswith("rebotarm")

    for name, directory in sorted(packages.items()):
        allowed = policy.get(name, {"imports": [], "resources": []})
        allowed_imports = set(allowed["imports"])
        allowed_runtime = allowed_imports | set(allowed["resources"])
        for dep in sorted(declared[name]):
            if is_local(dep) and dep not in allowed_runtime:
                errors.append(f"{name}/package.xml: forbidden dependency {dep}")
        for path in sorted(directory.rglob("*.py")):
            if "__pycache__" in path.parts:
                continue
            python_files += 1
            relative = path.relative_to(root).as_posix()
            try:
                tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=relative)
            except SyntaxError as exc:
                errors.append(f"{relative}:{exc.lineno}: {exc.msg}")
                continue
            if name == rules["compatibility_package"]:
                if any(isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) for n in ast.walk(tree)):
                    errors.append(f"{relative}: compatibility package must only forward existing implementations")
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call) or len(node.args) < 2:
                    continue
                method = getattr(node.func, "attr", "")
                owner_rules = rules.get(
                    "service_owners" if method == "create_service" else "publisher_owners", {}
                )
                if method not in {"create_service", "create_publisher"}:
                    continue
                topic_literals = "".join(
                    child.value for child in ast.walk(node.args[1])
                    if isinstance(child, ast.Constant) and isinstance(child.value, str)
                )
                for fragment, owners in owner_rules.items():
                    if fragment in topic_literals and name not in owners:
                        errors.append(f"{relative}:{node.lineno}: {method} {fragment} belongs to {', '.join(owners)}")
            for module, line in imported_modules(tree):
                dep = module.split(".")[0]
                site = f"{relative}:{line}"
                external = rules.get("external_modules", {}).get(dep)
                if external and external not in runtime[name]:
                    errors.append(f"{site}: external module {dep} needs depend/exec_depend {external}")
                if dep in rules["hardware_sdk_modules"] and name != rules["hardware_owner"]:
                    key = (relative, module)
                    if key in sdk_debt:
                        used_sdk_debt.add(key)
                        item = sdk_debt[key]
                        warnings.append(f"{item['issue']} (exit {item['exit_gate']}): {relative} imports {module}; {item['reason']}")
                    else:
                        errors.append(f"{site}: hardware SDK {dep} belongs to {rules['hardware_owner']}")
                if dep == name or not is_local(dep):
                    continue
                imports[name].add(dep)
                if dep not in allowed_imports:
                    errors.append(f"{site}: forbidden import {name} -> {dep}")
                if dep not in runtime[name]:
                    errors.append(f"{site}: imported package {dep} needs depend/exec_depend")
            for dep, line in resource_modules(tree):
                if dep != name and not is_local(dep) and dep not in runtime[name]:
                    errors.append(f"{relative}:{line}: external resource package {dep} needs depend/exec_depend")
                if dep != name and is_local(dep):
                    resources[name].add(dep)
                    sites.setdefault((name, dep), f"{relative}:{line}")
        # URDF/MJCF resources are dependencies too, even without Python imports.
        for pattern in ("*.urdf", "*.xacro", "*.xml"):
            for path in sorted(directory.rglob(pattern)):
                for dep in re.findall(r"package://([A-Za-z0-9_]+)/", path.read_text(encoding="utf-8-sig")):
                    if dep != name and is_local(dep):
                        resources[name].add(dep)
                        sites.setdefault((name, dep), path.relative_to(root).as_posix())
        for dep in sorted(resources[name]):
            if dep not in allowed_runtime:
                errors.append(f"{sites[name, dep]}: forbidden resource dependency {name} -> {dep}")

    debt = {(item["source"], item["target"]): item for item in rules["undeclared_resource_debt"]}
    used_debt = set()
    for name in sorted(resources):
        for dep in sorted(resources[name] - runtime[name]):
            edge = (name, dep)
            if edge in debt:
                item = debt[edge]
                used_debt.add(edge)
                warnings.append(
                    f"{item['issue']} (exit {item['exit_gate']}): {name} -> {dep} "
                    f"is not declared; {item['reason']}"
                )
            else:
                errors.append(f"{sites[edge]}: resource package {dep} needs depend/exec_depend")
    for source, target in sorted(set(debt) - used_debt):
        errors.append(f"remove resolved or stale resource exception: {source} -> {target}")
    for path, module in sorted(set(sdk_debt) - used_sdk_debt):
        errors.append(f"remove resolved or stale SDK exception: {path} -> {module}")

    manifests = {name: deps & set(packages) for name, deps in declared.items()}
    for label, graph in (("Python import", imports), ("package.xml", manifests)):
        cycle = cycle_message(graph)
        if cycle:
            errors.append(f"{label} cycle: {cycle}")
    deployment = {name: manifests[name] | imports[name] | resources[name] for name in packages}
    cycle = cycle_message(deployment)
    if cycle:
        warnings.append(f"deployment resource cycle (exit S1): {cycle}")
    return {
        "ok": not errors,
        "deployment_ready": not errors and not warnings,
        "package_count": len(packages),
        "python_file_count": python_files,
        "errors": sorted(set(errors)),
        "deployment_warnings": sorted(set(warnings)),
        "imports": {k: sorted(v) for k, v in sorted(imports.items())},
        "resources": {k: sorted(v) for k, v in sorted(resources.items())},
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--rules", type=Path, help="defaults to ROOT/tools/architecture_rules.json")
    parser.add_argument("--json", action="store_true", help="print the dependency graph and findings")
    parser.add_argument("--strict-deployment", action="store_true", help="fail on documented resource debt too")
    args = parser.parse_args(argv)
    try:
        rules = json.loads((args.rules or args.root / "tools/architecture_rules.json").read_text(encoding="utf-8"))
        if rules["schema_version"] != 1:
            raise ValueError("unsupported architecture rules schema")
        report = check_repository(args.root.resolve(), rules)
    except (OSError, ValueError, KeyError, ET.ParseError) as exc:
        print(f"architecture input error: {exc}", file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        print(f"{report['package_count']} packages, {report['python_file_count']} Python files")
        for message in report["errors"]:
            print(f"ERROR: {message}")
        for message in report["deployment_warnings"]:
            print(f"DEBT: {message}")
        print(f"boundaries={'PASS' if report['ok'] else 'FAIL'}; "
              f"deployment={'PASS' if report['deployment_ready'] else 'NOT READY'}")
    return 0 if report["deployment_ready" if args.strict_deployment else "ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
