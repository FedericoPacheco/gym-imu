#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: ./install-firmware.sh [--build-only]

Install firmware prerequisites, fetch missing external components, and build
the device firmware. By default, upload it to the connected ESP32-C3.

Options:
  --build-only  Build the firmware without uploading it
  -h, --help    Show this help
EOF
}

BUILD_ONLY=0
while (($#)); do
  case "$1" in
    --build-only)
      BUILD_ONLY=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [[ $EUID -eq 0 ]]; then
  echo "Run this script as your regular user; it will use sudo when needed." >&2
  exit 1
fi

for command in sudo apt-get; do
  if ! command -v "$command" >/dev/null 2>&1; then
    echo "Required command not found: $command" >&2
    exit 1
  fi
done

sudo -v
sudo apt-get update
sudo apt-get install -y python3 python3-setuptools python3-venv lcov curl git

for command in curl git python3; do
  if ! command -v "$command" >/dev/null 2>&1; then
    echo "Required command not found after installing prerequisites: $command" >&2
    exit 1
  fi
done

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

pio_command() {
  if command -v pio >/dev/null 2>&1; then
    command -v pio
  elif command -v platformio >/dev/null 2>&1; then
    command -v platformio
  elif [[ -x "$HOME/.platformio/penv/bin/pio" ]]; then
    printf '%s\n' "$HOME/.platformio/penv/bin/pio"
  elif [[ -x "$HOME/.platformio/penv/bin/platformio" ]]; then
    printf '%s\n' "$HOME/.platformio/penv/bin/platformio"
  else
    return 1
  fi
}

if ! PIO="$(pio_command)"; then
  installer_file="$(mktemp)"
  trap 'rm -f "$installer_file"' EXIT
  curl -fsSL \
    https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py \
    -o "$installer_file"
  python3 "$installer_file"
  rm -f "$installer_file"
  trap - EXIT
  export PATH="$HOME/.platformio/penv/bin:$PATH"
  PIO="$(pio_command)" || {
    echo "PlatformIO was installed but its command could not be found." >&2
    exit 1
  }
fi

if [[ -x "$HOME/.platformio/penv/bin" ]]; then
  export PATH="$HOME/.platformio/penv/bin:$PATH"
  bashrc="$HOME/.bashrc"
  path_line='export PATH="$HOME/.platformio/penv/bin:$PATH"'
  if [[ -f "$bashrc" ]] && ! grep -Fqx "$path_line" "$bashrc"; then
    printf '\n%s\n' "$path_line" >> "$bashrc"
  elif [[ ! -f "$bashrc" ]]; then
    printf '%s\n' "$path_line" > "$bashrc"
  fi
fi

"$PIO" platform install espressif32

if command -v code >/dev/null 2>&1; then
  code --install-extension platformio.platformio-ide
else
  echo "VS Code CLI not found; install the PlatformIO IDE extension in VS Code manually." >&2
fi

clone_component() {
  local url="$1"
  local directory="$2"
  if [[ -d "$directory" ]]; then
    echo "Keeping existing component: $directory"
  else
    git clone "$url" "$directory"
  fi
}

clone_component https://github.com/FedericoPacheco/esp32-MPU-driver components/MPU
clone_component https://github.com/FedericoPacheco/esp32-I2Cbus components/I2Cbus

chmod +x gen-lcov-report.sh

sudo usermod -a -G dialout "$(id -un)"
udev_rules="$(mktemp)"
trap 'rm -f "$udev_rules"' EXIT
curl -fsSL \
  https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules \
  -o "$udev_rules"
sudo install -m 644 "$udev_rules" /etc/udev/rules.d/99-platformio-udev.rules
rm -f "$udev_rules"
trap - EXIT
sudo udevadm control --reload-rules
sudo udevadm trigger

if ! id -nG "$(id -un)" | tr ' ' '\n' | grep -qx dialout; then
  echo "Added $(id -un) to dialout. Log out and back in, then rerun this script to upload." >&2
  exit 2
fi

if ((BUILD_ONLY)); then
  "$PIO" run -e device
else
  "$PIO" run -e device -t upload
fi
