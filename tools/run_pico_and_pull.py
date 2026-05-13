from pathlib import Path
from datetime import datetime
import subprocess
import sys
import re



REPO_ROOT = Path(__file__).resolve().parents[1]

FIRMWARE_ROOT = REPO_ROOT / "firmware" / "pico_micropython"
DATA_RAW_ROOT = REPO_ROOT / "data" / "raw"

SAVED_FILE_PATTERN = re.compile(r"Saved:\s*([^\s]+\.csv)")

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

    return_code, output_text = run_command(command, show_output=False)

    if return_code == 0:
        return True

    return False


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

        # Defensive cleanup in case the print line has extra whitespace.
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

        data/raw/hardware_validation/hx711_load_cell/load_cell_calibration/candidate

    Data is always pulled into candidate first. After review, manually promote
    files to accepted, rejected, or diagnostics.
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

    return DATA_RAW_ROOT / relative_folder / script_folder / "candidate"


def pull_file_from_pico(remote_filename, local_output_path, port):
    """
    Pulls a file from the Pico filesystem into the target local path.

    mpremote remote files use a leading colon:
        :servo_sweep.csv
    """
    local_output_path.parent.mkdir(parents=True, exist_ok=True)

    command = get_mpremote_base(port) + [
        "fs",
        "cp",
        f":{remote_filename}",
        str(local_output_path),
    ]

    return_code, _ = run_command(command)

    if return_code != 0:
        raise RuntimeError(f"Failed to pull {remote_filename} from Pico.")


def remove_file_from_pico(remote_filename, port):
    """
    Removes a file from the Pico filesystem after it has been successfully
    copied to the laptop.

    This prevents the Pico flash storage from slowly filling with duplicate
    CSV files.
    """
    command = get_mpremote_base(port) + [
        "fs",
        "rm",
        f":{remote_filename}",
    ]

    return_code, _ = run_command(command)

    if return_code != 0:
        raise RuntimeError(
            f"Pulled {remote_filename}, but failed to remove it from the Pico.\n"
            "The local copy should still exist, but check the Pico filesystem "
            "before running many more tests."
        )


def parse_args():
    """
    Basic argument parsing without needing argparse.

    Usage:
        python tools/run_pico_and_pull.py path/to/script.py
        python tools/run_pico_and_pull.py path/to/script.py --port COM3
    """
    if len(sys.argv) < 2:
        raise RuntimeError(
            "Usage:\n\n"
            "    python tools/run_pico_and_pull.py path/to/pico_script.py\n\n"
            "Optional:\n\n"
            "    python tools/run_pico_and_pull.py path/to/pico_script.py --port COM3\n"
        )

    pico_script_path = Path(sys.argv[1]).resolve()
    port = None

    if "--port" in sys.argv:
        port_index = sys.argv.index("--port")

        try:
            port = sys.argv[port_index + 1]
        except IndexError:
            raise RuntimeError("You used --port but did not provide a port, e.g. --port COM3")

    return pico_script_path, port

def make_unique_local_csv_path(output_dir, remote_csv_name):
    """
    Creates a unique local CSV path using the laptop date and the next
    available index in the destination folder.

    Example:
        remote_csv_name = servo_sweep_log.csv

    Output:
        data/raw/.../2026_05_10_servo_sweep_log_00.csv
        data/raw/.../2026_05_10_servo_sweep_log_01.csv
        data/raw/.../2026_05_10_servo_sweep_log_02.csv
    """
    output_dir.mkdir(parents=True, exist_ok=True)

    date_string = datetime.now().strftime("%Y_%m_%d")
    remote_stem = Path(remote_csv_name).stem

    index = 0

    while True:
        candidate_name = f"{date_string}_{remote_stem}_{index:02d}.csv"
        candidate_path = output_dir / candidate_name

        if not candidate_path.exists():
            return candidate_path

        index += 1

def main():
    pico_script_path, port = parse_args()

    if not pico_script_path.exists():
        raise FileNotFoundError(f"Pico script not found: {pico_script_path}")

    output_dir = get_data_output_dir(pico_script_path)

    print("Pico script:")
    print(f"  {pico_script_path}")

    print("Data output folder:")
    print(f"  {output_dir}")

    if port is None:
        port = find_pico_port()
    else:
        print(f"Using manually specified Pico port: {port}")

    # Step 1: Run the MicroPython script on the Pico.
    return_code, output_text = run_command(
        get_mpremote_base(port) + [
            "run",
            str(pico_script_path),
        ]
    )

    if return_code != 0:
        raise RuntimeError("Pico script failed. Not pulling CSV.")

    # Step 2: Parse all filenames printed by the Pico, if any exist.
    saved_csv_names = find_saved_csv_names(output_text)

    if not saved_csv_names:
        print("\nNo CSV output files detected.")
        print("Pico script completed successfully, but no files were pulled.")
        return

    print("\nDetected saved CSV files:")
    for name in saved_csv_names:
        print(f"  {name}")

    pulled_paths = []

    # Step 3-5: Pull each CSV into the mirrored local data/raw folder,
    # then remove it from the Pico after successful copy.
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

        pulled_paths.append(local_csv_path)

    print("\nAll detected CSV files pulled:")
    for path in pulled_paths:
        print(f"  {path}")


if __name__ == "__main__":
    main()
