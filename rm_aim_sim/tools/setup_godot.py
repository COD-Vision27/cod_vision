#!/usr/bin/env python3
"""Download the pinned standalone Linux x86_64 Godot binary into ignored .tools/."""

import argparse
import hashlib
from pathlib import Path
import platform
import tempfile
import urllib.request
import zipfile

VERSION = "4.7.2-stable"
NAME = "Godot_v%s_linux.x86_64" % VERSION
SHA256 = "cadd3204e728a35d3f13adb7fd0d7902636b79f6b95c40c265eb73b6c35329e4"
URL = "https://github.com/godotengine/godot-builds/releases/download/%s/%s.zip" % (VERSION, NAME)


def main():
    if platform.system() != "Linux" or platform.machine() not in ("x86_64", "AMD64"):
        raise SystemExit("This helper supports Linux x86_64; download the correct Godot build manually.")
    destination = Path(__file__).resolve().parents[1] / ".tools"
    destination.mkdir(exist_ok=True)
    binary = destination / "godot"
    if binary.is_file() and hashlib.sha256(binary.read_bytes()).hexdigest() == "8d106cbe6144c2dc7e881d61d2429c1a8a76e6b22ef48bd5e48dcf934953f71e":
        print("Already verified:", binary)
        return
    parser = argparse.ArgumentParser()
    parser.add_argument("--archive", type=Path, help="Offline official ZIP (SHA256 is still verified)")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="rm_aim_sim_godot_") as temp:
        archive = args.archive or Path(temp) / "godot.zip"
        if args.archive is None:
            print("Downloading", URL, flush=True)
            with urllib.request.urlopen(URL, timeout=20) as source, archive.open("wb") as output:
                import shutil
                shutil.copyfileobj(source, output)
        actual = hashlib.sha256(archive.read_bytes()).hexdigest()
        if actual != SHA256:
            raise SystemExit("SHA256 mismatch: " + actual)
        with zipfile.ZipFile(archive) as contents:
            (destination / "godot").write_bytes(contents.read(NAME))
        (destination / "godot").chmod(0o755)
    print("Ready:", destination / "godot")


if __name__ == "__main__":
    main()
