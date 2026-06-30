"""
sync_pico.py — keep the Pico's /lib in lockstep with the repo.

The local firmware library directory:

    firmware/pico_micropython/lib/

is the single source of truth. Before any firmware runs, the Pico's /lib must
match it, and no root-level module may shadow a /lib module (MicroPython
sys.path is ['', '/lib'], so a root file wins over the same name in /lib — that
is the classic "deprecated firmware silently ran" trap).

Modes:
    --check   Report drift only. Never pushes, deletes, or writes anything.
    (default) Reconcile: push changed/new lib files, delete root shadows,
              rewrite the manifest. Add --prune to also delete /lib orphans.
    --deep    Compare by hashing the actual Pico files instead of trusting the
              on-device manifest (catches manual edits made on the Pico).

Drift detection uses a content hash with line endings normalized, so a CRLF vs
LF difference is not treated as a change. A manifest of last-pushed hashes lives
at /lib/.sync_manifest.json so the common "already in sync" case is fast.

Usage:
    python tools/sync_pico.py --check
    python tools/sync_pico.py --check --deep
    python tools/sync_pico.py            # reconcile
    python tools/sync_pico.py --prune    # reconcile + remove /lib orphans
"""

from pathlib import Path
import hashlib
import json
import re
import sys
import tempfile

from run_pico_and_pull import (
    REPO_ROOT,
    FIRMWARE_ROOT,
    get_mpremote_base,
    run_command,
    pico_path,
    upload_file_to_pico,
    remove_file_from_pico,
    find_pico_port,
)


LIB_DIR = FIRMWARE_ROOT / "lib"
MANIFEST_NAME = ".sync_manifest.json"
REMOTE_LIB_MANIFEST = "lib/" + MANIFEST_NAME

# Pico filesystem listing rows look like "    4650 hx711_gpio.py"; directories
# end with "/". The "ls :lib" header line has no leading size and is skipped.
LS_ROW_PATTERN = re.compile(r"^\s*(\d+)\s+(.+?)\s*$")


# ----------------------------
# Hashing (line-ending agnostic)
# ----------------------------

def normalized_hash(data):
    """SHA-1 of content with CRLF/CR collapsed to LF, so line endings don't
    register as drift."""
    canonical = data.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    return hashlib.sha1(canonical).hexdigest()


def local_lib_files():
    """Top-level non-hidden files in the local canonical lib directory."""
    if not LIB_DIR.is_dir():
        raise RuntimeError(f"Local lib directory not found: {LIB_DIR}")
    return sorted(
        p for p in LIB_DIR.iterdir()
        if p.is_file() and not p.name.startswith(".")
    )


def local_manifest():
    """{filename: normalized_hash} for every local lib file."""
    return {p.name: normalized_hash(p.read_bytes()) for p in local_lib_files()}


# ----------------------------
# Pico filesystem queries
# ----------------------------

def remote_listing(port, remote_dir=None):
    """
    Returns {filename: size} for a Pico directory (files only, no subdirs).
    remote_dir=None lists the filesystem root.
    """
    target = pico_path(remote_dir) if remote_dir else ":"
    return_code, output_text = run_command(
        get_mpremote_base(port) + ["fs", "ls", target],
        show_output=False,
    )

    if return_code != 0:
        raise RuntimeError(
            f"Could not list Pico path '{target}'. Is the Pico connected and "
            f"the port free?\n\n{output_text}"
        )

    files = {}
    for line in output_text.splitlines():
        match = LS_ROW_PATTERN.match(line)
        if not match:
            continue
        name = match.group(2)
        if name.endswith("/"):
            continue  # directory
        files[name] = int(match.group(1))
    return files


def read_remote_bytes(port, remote_path):
    """Copy a Pico file to a temp file and return its exact bytes, or None if
    the file does not exist."""
    with tempfile.TemporaryDirectory() as tmp:
        local = Path(tmp) / "remote_file"
        return_code, _ = run_command(
            get_mpremote_base(port) + ["fs", "cp", pico_path(remote_path), str(local)],
            show_output=False,
        )
        if return_code != 0 or not local.exists():
            return None
        return local.read_bytes()


def read_remote_manifest(port):
    """Parse /lib/.sync_manifest.json -> {filename: hash}; {} if missing/invalid."""
    data = read_remote_bytes(port, REMOTE_LIB_MANIFEST)
    if data is None:
        return {}
    try:
        parsed = json.loads(data.decode("utf-8"))
        if isinstance(parsed, dict):
            return parsed
    except (ValueError, UnicodeError):
        pass
    return {}


def remote_hashes(port, remote_lib_names):
    """Deep mode: hash the actual Pico /lib files."""
    hashes = {}
    for name in remote_lib_names:
        if name == MANIFEST_NAME:
            continue
        data = read_remote_bytes(port, "lib/" + name)
        if data is not None:
            hashes[name] = normalized_hash(data)
    return hashes


# ----------------------------
# Plan
# ----------------------------

def compute_plan(port, deep=False):
    """
    Build the reconciliation plan. Returns a dict describing drift.

    If no on-device manifest exists, escalates to a deep compare for this run so
    the first report is truthful rather than blindly re-pushing everything.
    """
    local = local_manifest()
    remote_lib = remote_listing(port, "lib")
    remote_root = remote_listing(port, None)

    manifest = read_remote_manifest(port)
    escalated = False
    if not deep and not manifest:
        deep = True
        escalated = True

    if deep:
        ref_hashes = remote_hashes(port, remote_lib.keys())
    else:
        ref_hashes = manifest

    new = [n for n in local if n not in remote_lib]
    changed = [
        n for n in local
        if n in remote_lib and local[n] != ref_hashes.get(n)
    ]
    to_push = sorted(set(new) | set(changed))

    # Root-level files that shadow a canonical lib module name.
    shadows = sorted(n for n in remote_root if n in local)

    # /lib files the repo doesn't know about (excluding the manifest itself).
    orphans = sorted(
        n for n in remote_lib
        if n not in local and n != MANIFEST_NAME
    )

    return {
        "local": local,
        "remote_lib": remote_lib,
        "remote_root": remote_root,
        "new": sorted(new),
        "changed": sorted(changed),
        "to_push": to_push,
        "shadows": shadows,
        "orphans": orphans,
        "deep": deep,
        "escalated": escalated,
    }


def plan_is_clean(plan, prune=False):
    if plan["to_push"] or plan["shadows"]:
        return False
    if prune and plan["orphans"]:
        return False
    return True


def print_report(plan, prune=False):
    mode = "deep (hashed Pico files)" if plan["deep"] else "fast (on-device manifest)"
    if plan["escalated"]:
        mode += " [auto-escalated: no manifest on Pico]"

    print("\nPico /lib sync report")
    print("  comparison mode:", mode)
    print("  local lib files:", len(plan["local"]))

    if plan["to_push"]:
        print("  -> push (changed/new):")
        for n in plan["to_push"]:
            tag = "new" if n in plan["new"] else "changed"
            print(f"       {n}  ({tag})")
    else:
        print("  -> push: none")

    if plan["shadows"]:
        print("  -> remove root shadows (root file hides /lib module):")
        for n in plan["shadows"]:
            print(f"       /{n}")
    else:
        print("  -> root shadows: none")

    if plan["orphans"]:
        verb = "remove" if prune else "orphan (use --prune to remove)"
        print(f"  -> /lib {verb}:")
        for n in plan["orphans"]:
            print(f"       lib/{n}")
    else:
        print("  -> /lib orphans: none")


# ----------------------------
# Apply
# ----------------------------

def write_remote_manifest(port, local):
    with tempfile.TemporaryDirectory() as tmp:
        local_manifest_path = Path(tmp) / MANIFEST_NAME
        local_manifest_path.write_text(
            json.dumps(local, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        upload_file_to_pico(local_manifest_path, REMOTE_LIB_MANIFEST, port)


def apply_plan(port, plan, prune=False):
    for name in plan["shadows"]:
        print(f"Removing root shadow: /{name}")
        remove_file_from_pico(name, port)

    for name in plan["to_push"]:
        print(f"Pushing lib/{name}")
        upload_file_to_pico(LIB_DIR / name, "lib/" + name, port)

    if prune:
        for name in plan["orphans"]:
            print(f"Removing /lib orphan: lib/{name}")
            remove_file_from_pico("lib/" + name, port)

    write_remote_manifest(port, plan["local"])


def ensure_pico_in_sync(port, deep=False, prune=False, check_only=False, verbose=True):
    """
    Programmatic entry point for the orchestrator.

    Returns (in_sync, plan). In non-check mode, reconciles and returns True after
    a successful apply (raising on failure). In check mode, never mutates the
    Pico and returns whether it is already clean.
    """
    plan = compute_plan(port, deep=deep)

    if check_only:
        if verbose:
            print_report(plan, prune=prune)
        return plan_is_clean(plan, prune=prune), plan

    if plan_is_clean(plan, prune=prune):
        if verbose:
            print(f"Pico /lib in sync ({len(plan['local'])} files).")
        # Make sure a manifest exists even when nothing else changed.
        if plan["escalated"]:
            write_remote_manifest(port, plan["local"])
        return True, plan

    if verbose:
        print_report(plan, prune=prune)
    apply_plan(port, plan, prune=prune)

    # Re-verify against the manifest we just wrote.
    verify = compute_plan(port, deep=False)
    if not plan_is_clean(verify, prune=prune):
        raise RuntimeError(
            "Pico /lib still out of sync after reconcile. Inspect manually with "
            "`python -m mpremote connect <port> fs ls lib`."
        )
    if verbose:
        print(f"Pico /lib reconciled and verified ({len(verify['local'])} files).")
    return True, verify


# ----------------------------
# CLI
# ----------------------------

def parse_args(argv):
    port = None
    if "--port" in argv:
        i = argv.index("--port")
        try:
            port = argv[i + 1]
        except IndexError:
            raise RuntimeError("--port requires a value, e.g. --port COM3")
    return {
        "port": port,
        "check_only": "--check" in argv,
        "deep": "--deep" in argv,
        "prune": "--prune" in argv,
    }


def main():
    args = parse_args(sys.argv[1:])

    port = args["port"]
    if port is None:
        port = find_pico_port()
    else:
        print(f"Using manually specified Pico port: {port}")

    in_sync, _ = ensure_pico_in_sync(
        port,
        deep=args["deep"],
        prune=args["prune"],
        check_only=args["check_only"],
    )

    if args["check_only"]:
        if in_sync:
            print("\nRESULT: Pico /lib is in sync with the repo.")
            sys.exit(0)
        else:
            print("\nRESULT: Pico /lib is OUT OF SYNC. Run without --check to reconcile.")
            sys.exit(1)


if __name__ == "__main__":
    main()
