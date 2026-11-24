#!/usr/bin/env bash
# One-time setup on Ubuntu 24.04. Run from the repo root: bash scripts/setup_ubuntu.sh
set -euo pipefail

echo "== system packages: ARM cross-compiler, newlib, QEMU =="
sudo apt-get update
sudo apt-get install -y build-essential gcc-arm-none-eabi libnewlib-arm-none-eabi \
    qemu-system-arm libglib2.0-dev pkg-config curl python3-venv

echo "== Python environment (.venv) =="
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt

echo "== build and test =="
make
make -C tools/qemu_plugin
make test
python3 scripts/measure_m4.py smoke --check

echo
echo "Done. Each new terminal: source .venv/bin/activate"
