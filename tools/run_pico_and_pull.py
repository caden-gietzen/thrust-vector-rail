from pathlib import Path
from datetime import datetime
import subprocess
import sys
import re
import json
import csv
import tempfile


REPO_ROOT = Path(__file__).resolve().parents[1]

FIRMWARE_ROOT = REPO_ROOT / "firmware" / "pico_micropython"
DATA_RAW_ROOT = REPO_ROOT / "data" / "raw"

SAVED_FILE_PATTERN = re.compile(r"Saved:\s*([^\s]+\.csv)")

DATASET_STATUS_FOLDERS = [
    "candidate",
    "accepted",
    "rejected",
    "diagnostics",
]

DEFAULT_REMOTE_CONFIG_PATH = "run_config.json"


def run_command(command, cwd=None, show_output=True):
    """
    Run a shell command and stream output live.

    Returns:
        return_code, full_stdout
    """
    if show_output:
        print("\nRunning command:")
        print(" ".join(str(part) for part in command))
        print()

    process = subprocess.Popen(
        command,
        cwd=cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    output_lines = []

    for line in process.stdout:
        if show_output:
            print(line, end="")
        output_lines.append(line)

    process.wait()

    return process.returncode, "".join(output_lines)


def get_mpremote_base(port=None):
    """
    Builds a robust mpremote command using the same Python interpreter
    running this script.

    Example:
        python -m mpremote connect COM3
    """
    base = [
        sys.executable,
        "-m",
        "mpremote",
    ]

    if port is not None:
        base += ["connect", port]

    return base


def list_candidate_ports():
    """
    Uses mpremote to list available serial devices.

    Returns:
        list of port strings, e.g. ["COM3", "COM5"]
    """
    command = get_mpremote_base() + ["connect", "list"]

    return_code, output_text = run_command(command, show_output=False)

    if return_code != 0:
        raise RuntimeError(
            "Could not list mpremote devices.\n"
            "Try checking that mpremote is installed:\n\n"
            "    python -m mpremote connect list\n"
        )

    ports = []

    for line in output_text.splitlines():
        line = line.strip()

        # Typical Windows output includes COM ports.
        if line.startswith("COM"):
            ports.append(line.split()[0])

    return ports


def test_pico_port(port):
    """
    Tests whether mpremote can talk to a MicroPython device on this port.
    """
    command = get_mpremote_base(port) + ["fs", "ls"]

    return_code, _ = run_command(command, show_output=False)

    return return_code == 0


def find_pico_port():
    """
    Automatically finds the first serial port that responds to mpremote.
    """
    ports = list_candidate_ports()

    if not ports:
        raise RuntimeError(
            "No candidate serial ports found.\n\n"
            "Check:\n"
            "  1. Pico is plugged in.\n"
            "  2. Pico is not in BOOTSEL mode.\n"
            "  3. Thonny / MicroPico / serial monitor is closed.\n"
            "  4. Device Manager shows a COM port."
        )

    print("Candidate ports:")
    for port in ports:
        print(f"  {port}")

    for port in ports:
        print(f"Testing {port}...")
        if test_pico_port(port):
            print(f"Using Pico port: {port}")
            return port

    raise RuntimeError(
        "Found serial ports, but none responded as a MicroPython Pico.\n\n"
        "Try manually testing:\n\n"
        "    python -m mpremote connect COM3 fs ls\n\n"
        "Also make sure no other program is connected to the Pico."
    )


def find_saved_csv_names(output_text):
    """
    Extracts all CSV filenames from lines like:

        Saved: servo_prps_set01_seed4001.csv
        Saved: servo_prps_set02_seed4002.csv
        Saved: servo_prps_set03_seed4003.csv

    Returns:
        list[str] of unique filenames in detected order
    """
    matches = SAVED_FILE_PATTERN.findall(output_text)

    saved_names = []
    seen = set()

    for match in matches:
        name = match.strip()
        name = Path(name).name

        if name not in seen:
            saved_names.append(name)
            seen.add(name)

    return saved_names


def get_data_output_dir(pico_script_path):
    """
    Maps:

        firmware/pico_micropython/hardware_validation/hx711_load_cell/load_cell_calibration.py

    to:

        data/raw/hardware_validation/hx711_load_cell/load_cell_calibration

    The candidate, accepted, rejected, and diagnostics folders are created
    inside this folder. New data is pulled into candidate by default.
    """
    pico_script_path = pico_script_path.resolve()

    try:
        relative_script_path = pico_script_path.relative_to(FIRMWARE_ROOT)
    except ValueError:
        raise RuntimeError(
            f"Script is not inside expected firmware root:\n"
            f"  script: {pico_script_path}\n"
            f"  firmware root: {FIRMWARE_ROOT}"
        )

    relative_folder = relative_script_path.parent
    script_folder = relative_script_path.stem

    return DATA_RAW_ROOT / relative_folder / script_folder


def ensure_dataset_status_folders(base_output_dir):
    """
    Ensures the standard dataset review folders exist.

    New data is written to candidate. After review, files can be moved to
    accepted, rejected, or diagnostics.
    """
    for folder_name in DATASET_STATUS_FOLDERS:
        folder_path = base_output_dir / folder_name
        folder_path.mkdir(parents=True, exist_ok=True)


def pico_path(remote_path):
    """
    Converts a Pico filesystem path into mpremote's colon path format.

    Examples:
        "run_config.json"  -> ":run_config.json"
        "/run_config.json" -> ":/run_config.json"
    """
    remote_path = str(remote_path).replace("\\", "/")
    if remote_path.startswith(":"):
        return remote_path
    return f":{remote_path}"


def pull_file_from_pico(remote_filename, local_output_path, port):
    """
    Pulls a file from the Pico filesystem into the target local path.
    """
    local_output_path.parent.mkdir(parents=True, exist_ok=True)

    command = get_mpremote_base(port) + [
        "fs",
        "cp",
        pico_path(remote_filename),
        str(local_output_path),
    ]

    return_code, _ = run_command(command)

    if return_code != 0:
        raise RuntimeError(f"Failed to pull {remote_filename} from Pico.")


def upload_file_to_pico(local_path, remote_filename, port):
    """
    Uploads a laptop file to the Pico filesystem.
    """
    command = get_mpremote_base(port) + [
        "fs",
        "cp",
        str(local_path),
        pico_path(remote_filename),
    ]

    return_code, _ = run_command(command)

    if return_code != 0:
        raise RuntimeError(f"Failed to upload {local_path} to Pico as {remote_filename}.")


def remove_file_from_pico(remote_filename, port, missing_ok=False):
    """
    Removes a file from the Pico filesystem.
    """
    command = get_mpremote_base(port) + [
        "fs",
        "rm",
        pico_path(remote_filename),
    ]

    return_code, output_text = run_command(command, show_output=not missing_ok)

    if return_code != 0 and not missing_ok:
        raise RuntimeError(
            f"Failed to remove {remote_filename} from the Pico.\n"
            "The local copy may still exist, but check the Pico filesystem "
            "before running many more tests.\n\n"
            f"mpremote output:\n{output_text}"
        )


def parse_args():
    """
    Usage:
        "    python tools/run_pico_and_pull.py path/to/pico_script.py --port COM3\n"
        "    python tools/run_pico_and_pull.py path/to/pico_script.py --plan path/to/plan.json\n"
        "    python tools/run_pico_and_pull.py path/to/pico_script.py --no-orchestrate\n"
        "    python tools/run_pico_and_pull.py path/to/pico_script.py --manifest\n"
        "    python tools/run_pico_and_pull.py path/to/pico_script.py --no-manifest\n"
    """
    if len(sys.argv) < 2:
        raise RuntimeError(
            "Usage:\n\n"
            "    python tools/run_pico_and_pull.py path/to/pico_script.py\n\n"
            "Optional:\n\n"
            "    python tools/run_pico_and_pull.py path/to/pico_script.py --port COM3\n"
            "    python tools/run_pico_and_pull.py path/to/pico_script.py --plan path/to/plan.json\n"
            "    python tools/run_pico_and_pull.py path/to/pico_script.py --no-orchestrate\n"
        )

    pico_script_path = Path(sys.argv[1]).resolve()
    port = None
    plan_path = None
    use_orchestration = True
    write_manifest = None

    if "--manifest" in sys.argv:
        write_manifest = True

    if "--no-manifest" in sys.argv:
        write_manifest = False

    if "--port" in sys.argv:
        port_index = sys.argv.index("--port")
        try:
            port = sys.argv[port_index + 1]
        except IndexError:
            raise RuntimeError("You used --port but did not provide a port, e.g. --port COM3")

    if "--plan" in sys.argv:
        plan_index = sys.argv.index("--plan")
        try:
            plan_path = Path(sys.argv[plan_index + 1]).resolve()
        except IndexError:
            raise RuntimeError("You used --plan but did not provide a JSON plan path.")

    if "--no-orchestrate" in sys.argv:
        use_orchestration = False

    return pico_script_path, port, plan_path, use_orchestration, write_manifest

def make_unique_local_csv_path(output_dir, remote_csv_name):
    """
    Creates a unique local CSV path using the laptop date and the next
    available index across the full dataset folder, not just candidate.

    This prevents name collisions across:
        candidate/
        accepted/
        accepted/train/
        accepted/validation/
        rejected/
        diagnostics/

    Example:
        remote_csv_name = thrust_prps_daq_voltage_local_1100_1350_seed2001.csv

    Output:
        candidate/2026_05_20_thrust_prps_daq_voltage_local_1100_1350_seed2001_00.csv
        candidate/2026_05_20_thrust_prps_daq_voltage_local_1100_1350_seed2001_01.csv
        ...
    """
    output_dir.mkdir(parents=True, exist_ok=True)

    date_string = datetime.now().strftime("%Y_%m_%d")
    remote_stem = Path(remote_csv_name).stem

    # output_dir is usually:
    #   data/raw/.../candidate
    #
    # base_output_dir should be:
    #   data/raw/.../
    #
    # Then we scan all review/status folders under that base.
    base_output_dir = output_dir.parent

    index = 0

    while True:
        candidate_name = f"{date_string}_{remote_stem}_{index:02d}.csv"
        candidate_path = output_dir / candidate_name

        duplicate_exists = any(
            path.name == candidate_name
            for path in base_output_dir.rglob(candidate_name)
            if path.is_file()
        )

        if not duplicate_exists:
            return candidate_path

        index += 1


def find_sidecar_orchestration_plan(pico_script_path):
    """
    Looks for a sidecar orchestration JSON next to the Pico script.

    Preferred:
        thrust_prps_daq_voltage.orchestrate.json

    Also accepted:
        thrust_prps_daq_voltage_orchestrate.json
    """
    candidates = [
        pico_script_path.with_suffix(".orchestrate.json"),
        pico_script_path.with_name(pico_script_path.stem + "_orchestrate.json"),
    ]

    for candidate in candidates:
        if candidate.exists():
            return candidate

    return None


def load_orchestration_plan(plan_path):
    with open(plan_path, "r", encoding="utf-8") as f:
        plan = json.load(f)

    has_explicit_segments = "segments" in plan
    has_repeated_sequence = "sequence" in plan and "repeat_sequence" in plan

    if not has_explicit_segments and not has_repeated_sequence:
        raise RuntimeError(
            "Orchestration JSON must contain either:\n"
            "  1. 'segments': [...]\n"
            "or\n"
            "  2. 'repeat_sequence': {...} and 'sequence': [...]"
        )

    if has_explicit_segments:
        if not isinstance(plan["segments"], list) or not plan["segments"]:
            raise RuntimeError("'segments' must be a non-empty list.")

        for i, segment in enumerate(plan["segments"], start=1):
            if "config" not in segment or not isinstance(segment["config"], dict):
                raise RuntimeError(f"Segment {i} must contain a dictionary field named 'config'.")

            if "name" not in segment:
                segment["name"] = f"segment_{i:03d}"

    if has_repeated_sequence:
        if not isinstance(plan["sequence"], list) or not plan["sequence"]:
            raise RuntimeError("'sequence' must be a non-empty list.")

        if not isinstance(plan["repeat_sequence"], dict):
            raise RuntimeError("'repeat_sequence' must be a dictionary.")

        for i, segment in enumerate(plan["sequence"], start=1):
            if "config" not in segment or not isinstance(segment["config"], dict):
                raise RuntimeError(f"Sequence item {i} must contain a dictionary field named 'config'.")

            if "name" not in segment:
                segment["name"] = f"sequence_{i:03d}"

    return plan


def expand_orchestration_segments(plan):
    """
    Supports two plan styles:

    1. Explicit one-shot list:
        {
            "segments": [...]
        }

    2. Repeated sequence:
        {
            "repeat_sequence": {...},
            "sequence": [...]
        }

    For Option B / same seed per pass:
        Pass 1:
            region A seed 2001
            region B seed 2001
            region C seed 2001

        Pass 2:
            region A seed 2002
            region B seed 2002
            region C seed 2002
    """
    if "segments" in plan:
        return plan["segments"]

    sequence = plan["sequence"]
    repeat = plan["repeat_sequence"]

    repeat_count = int(repeat["count"])
    base_seed = int(repeat["base_prps_seed"])
    seed_mode = repeat.get("seed_mode", "same_seed_per_pass")
    seed_increment_per_pass = int(repeat.get("seed_increment_per_pass", 1))
    run_order_seed_offset = int(repeat.get("run_order_seed_offset", 50000))

    expanded_segments = []

    for repeat_index in range(1, repeat_count + 1):
        pass_seed = base_seed + (repeat_index - 1) * seed_increment_per_pass

        for sequence_index, segment_template in enumerate(sequence, start=1):
            if seed_mode == "same_seed_per_pass":
                prps_seed = pass_seed

            elif seed_mode == "increment_every_segment":
                prps_seed = base_seed + (
                    (repeat_index - 1) * len(sequence)
                    + (sequence_index - 1)
                )

            else:
                raise RuntimeError(
                    "Unknown seed_mode: {}. Use 'same_seed_per_pass' "
                    "or 'increment_every_segment'.".format(seed_mode)
                )

            config = dict(segment_template["config"])

            # Enforce one bounded Pico run per orchestrated segment.
            config["NUM_ACQUISITION_SETS"] = 1
            config["BASE_PRPS_SEED"] = prps_seed
            config["RUN_ORDER_SEED_OFFSET"] = run_order_seed_offset

            segment_name = segment_template.get("name", f"sequence_{sequence_index:03d}")

            expanded_segments.append(
                {
                    "name": "pass{:03d}_seq{:03d}_{}_seed{}".format(
                        repeat_index,
                        sequence_index,
                        segment_name,
                        prps_seed,
                    ),
                    "config": config,
                    "repeat_index": repeat_index,
                    "sequence_index": sequence_index,
                    "prps_seed": prps_seed,
                }
            )

    return expanded_segments


def write_temp_segment_config(segment):
    """
    Writes only the current segment config to a temporary JSON file.

    This is intentionally not the whole orchestration plan. The laptop owns
    the full plan; the Pico only receives the current segment's config.
    """
    config = dict(segment["config"])

    temp_file = tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        suffix="_run_config.json",
        delete=False,
    )

    with temp_file:
        json.dump(config, temp_file, indent=2)
        temp_file.write("\n")

    return Path(temp_file.name)


def append_manifest_row(manifest_path, row):
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    file_exists = manifest_path.exists()

    fieldnames = [
        "timestamp",
        "segment_index",
        "segment_name",
        "status",
        "remote_csv",
        "local_csv",
        "pico_script",
        "plan_path",
        "notes",
    ]

    with open(manifest_path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)

        if not file_exists:
            writer.writeheader()

        writer.writerow(row)


def run_pico_script_once(pico_script_path, port):
    return run_command(
        get_mpremote_base(port) + [
            "run",
            str(pico_script_path),
        ]
    )


def pull_detected_csvs(output_text, output_dir, port):
    saved_csv_names = find_saved_csv_names(output_text)

    if not saved_csv_names:
        print("\nNo CSV output files detected.")
        return []

    print("\nDetected saved CSV files:")
    for name in saved_csv_names:
        print(f"  {name}")

    pulled = []

    for saved_csv_name in saved_csv_names:
        local_csv_path = make_unique_local_csv_path(output_dir, saved_csv_name)

        print("\nLocal CSV will be saved as:")
        print(f"  {local_csv_path.name}")

        pull_file_from_pico(saved_csv_name, local_csv_path, port)

        print("\nPulled CSV to:")
        print(f"  {local_csv_path}")

        remove_file_from_pico(saved_csv_name, port)

        print("\nRemoved CSV from Pico:")
        print(f"  {saved_csv_name}")

        pulled.append((saved_csv_name, local_csv_path))

    return pulled


def run_normal_mode(pico_script_path, port, output_dir):
    return_code, output_text = run_pico_script_once(pico_script_path, port)

    if return_code != 0:
        raise RuntimeError("Pico script failed. Not pulling CSV.")

    pulled = pull_detected_csvs(output_text, output_dir, port)

    if not pulled:
        print("Pico script completed successfully, but no files were pulled.")
        return []

    print("\nAll detected CSV files pulled:")
    for _, path in pulled:
        print(f"  {path}")

    return pulled


def run_orchestrated_mode(pico_script_path, plan_path, port, output_dir, cli_write_manifest=None):
    plan = load_orchestration_plan(plan_path)
    remote_config_path = plan.get("remote_config_path", DEFAULT_REMOTE_CONFIG_PATH)

    if cli_write_manifest is None:
        write_manifest = bool(plan.get("write_manifest", False))
    else:
        write_manifest = bool(cli_write_manifest)

    manifest_path = output_dir / "orchestration_manifest.csv"

    print("\nUsing orchestration plan:")
    print(f"  {plan_path}")
    print("Remote config path:")
    print(f"  {remote_config_path}")
    print("Manifest:")
    if write_manifest:
        print(f"  enabled -> {manifest_path}")
    else:
        print("  disabled")

    all_pulled = []

    segments = expand_orchestration_segments(plan)

    for segment_index, segment in enumerate(segments, start=1):
        segment_name = segment.get("name", f"segment_{segment_index:03d}")

        print("\n" + "=" * 60)
        print(f"ORCHESTRATED SEGMENT {segment_index} of {len(segments)}")
        print(f"Segment: {segment_name}")
        print("=" * 60)

        local_config_path = write_temp_segment_config(segment)

        try:
            # Remove stale config before uploading this segment's config.
            remove_file_from_pico(remote_config_path, port, missing_ok=True)
            upload_file_to_pico(local_config_path, remote_config_path, port)

            if write_manifest:
                append_manifest_row(
                    manifest_path,
                    {
                        "timestamp": datetime.now().isoformat(timespec="seconds"),
                        "segment_index": segment_index,
                        "segment_name": segment_name,
                        "status": "started",
                        "remote_csv": "",
                        "local_csv": "",
                        "pico_script": str(pico_script_path),
                        "plan_path": str(plan_path),
                        "notes": "",
                    },
                )

            return_code, output_text = run_pico_script_once(pico_script_path, port)

            if return_code != 0:
                if write_manifest:
                    append_manifest_row(
                        manifest_path,
                        {
                            "timestamp": datetime.now().isoformat(timespec="seconds"),
                            "segment_index": segment_index,
                            "segment_name": segment_name,
                            "status": "started",
                            "remote_csv": "",
                            "local_csv": "",
                            "pico_script": str(pico_script_path),
                            "plan_path": str(plan_path),
                            "notes": "",
                        },
                    )
                raise RuntimeError(f"Pico segment failed: {segment_name}")

            pulled = pull_detected_csvs(output_text, output_dir, port)

            if not pulled:
                if write_manifest:
                    append_manifest_row(
                        manifest_path,
                        {
                            "timestamp": datetime.now().isoformat(timespec="seconds"),
                            "segment_index": segment_index,
                            "segment_name": segment_name,
                            "status": "started",
                            "remote_csv": "",
                            "local_csv": "",
                            "pico_script": str(pico_script_path),
                            "plan_path": str(plan_path),
                            "notes": "",
                        },
                    )
                raise RuntimeError(f"Segment completed but no CSV was detected: {segment_name}")

            for remote_csv, local_csv in pulled:
                if write_manifest:
                    append_manifest_row(
                        manifest_path,
                        {
                            "timestamp": datetime.now().isoformat(timespec="seconds"),
                            "segment_index": segment_index,
                            "segment_name": segment_name,
                            "status": "started",
                            "remote_csv": "",
                            "local_csv": "",
                            "pico_script": str(pico_script_path),
                            "plan_path": str(plan_path),
                            "notes": "",
                        },
                    )

            all_pulled.extend(pulled)

        finally:
            # Remove the run config so a later normal run cannot accidentally
            # inherit stale orchestration settings.
            remove_file_from_pico(remote_config_path, port, missing_ok=True)

            try:
                local_config_path.unlink()
            except OSError:
                pass

    print("\nAll orchestrated segments complete. Pulled files:")
    for _, path in all_pulled:
        print(f"  {path}")

    return all_pulled



def main():
    pico_script_path, port, cli_plan_path, use_orchestration, cli_write_manifest = parse_args()

    if not pico_script_path.exists():
        raise FileNotFoundError(f"Pico script not found: {pico_script_path}")

    base_output_dir = get_data_output_dir(pico_script_path)
    ensure_dataset_status_folders(base_output_dir)

    output_dir = base_output_dir / "candidate"

    print("Pico script:")
    print(f"  {pico_script_path}")

    print("Data output folder:")
    print(f"  {output_dir}")

    if port is None:
        port = find_pico_port()
    else:
        print(f"Using manually specified Pico port: {port}")

    plan_path = cli_plan_path

    if use_orchestration and plan_path is None:
        plan_path = find_sidecar_orchestration_plan(pico_script_path)

    if use_orchestration and plan_path is not None:
        if not plan_path.exists():
            raise FileNotFoundError(f"Orchestration plan not found: {plan_path}")

        run_orchestrated_mode(
            pico_script_path=pico_script_path,
            plan_path=plan_path,
            port=port,
            output_dir=output_dir,
            cli_write_manifest=cli_write_manifest,
        )
    else:
        run_normal_mode(pico_script_path, port, output_dir)


if __name__ == "__main__":
    main()

