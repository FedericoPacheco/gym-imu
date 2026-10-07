# Setup

OS: Linux (tested on Ubuntu 24.04/26.04)

Open the [project workspace](../.vscode/gym-imu.code-workspace) in VS Code.

## Firmware

### Automated

For a mostly automated firmware setup and upload on Ubuntu, run `./device/install-firmware.sh` from the repository root. Pass `--build-only` to build without uploading. The script installs system and PlatformIO dependencies, fetches the external components, configures serial-port access, and builds or uploads the firmware. If the VS Code CLI is unavailable, install the PlatformIO IDE extension manually. If the script reports that you were added to the `dialout` group, log out and back in, then rerun it.

Note: script generated with AI.

### Manual

1. Install python:

    1.1. Check version:

    ```bash
    python3 --version
    ```

    1.2. Install it if not present:

    ```bash
    sudo apt install python3
    ```

    1.3. Install other dependencies:

    ```bash
    sudo apt install python3-setuptools
    sudo apt install python3-venv
    ```

2. Install PlatformIO IDE:
    2.1. Install VSCode extension: PlatformIO IDE
    2.2. Install PlatformIO Core:

    ```bash
    curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py -o get-platformio.py
    python3 get-platformio.py
    ```

    2.3. Add PlatformIO to PATH:

    ```bash
    echo 'export PATH=$HOME/.platformio/penv/bin:$PATH' >> ~/.bashrc
    source ~/.bashrc
    ```

    2.4. Verify installation:

    ```bash
    platformio --version
    ```

    2.5. Install the pinned device dependencies from the `device` directory:

    ```bash
    cd device
    pio pkg install -e device
    ```

    2.6. Grant access to the serial port:

    ```bash
    sudo usermod -a -G dialout $USER
    ```

    2.7. Install udev rules:

    ```bash
    curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
    sudo service udev restart
    ```

    2.8. Install code coverage tool, make script executable, and install VS Code "open in browser" extension to open html report:

    ```bash
    sudo apt install lcov
    chmod +x gen-lcov-report.sh
    ```

3. Install C++ tools (formatter, static analysis, make):

    3.1. Install the VSCode extensions:
        - C/C++ Extension Pack (C/C++, Themes, CMake Tools)
        - Clangd

    3.2. Install libraries:

    ```bash
    sudo apt install clang clangd clang-tidy
    clang --version
    clangd --version
    clang-tidy --version
    ```

    Note: couldn't make clangd refactors work :(

4. Install external dependencies not available in PlatformIO:

    ```bash
        cd device/components
        git clone https://github.com/FedericoPacheco/esp32-MPU-driver MPU
        git clone https://github.com/FedericoPacheco/esp32-I2Cbus I2Cbus
    ```

5. Flash device with firmware: open PlatformIO > device > General:
     - Build
     - Upload / Upload and Monitor

## Enclosure (optional)

1. Install OpenSCAD:

    ```bash
        snap install openscad-nightly
    ```

2. Install the BOSL2 library (tools, shapes, and helpers to make OpenScad easier to use): <https://github.com/BelfrySCAD/BOSL2/?tab=readme-ov-file#installation>

3. Install NopSCADlib (parts for 3D printers and enclosures for electronics): <https://github.com/nophead/NopSCADlib/blob/master/docs/usage.md#installation>

4. Install VS Code Extension: OpenSCAD Language Support

5. Inside OpenSCAD, check "Design" > "Automatic Reload and Preview"

## Signal processing

1. Create and activate virtual environment:

    ```bash
    python3 -m venv venv
    source venv/bin/activate    # Linux
    .\venv\Scripts\activate.ps1 # Windows
    ```

2. Install dependencies:

    ```bash
    pip install -r requirements.txt
    ```

    This also installs the local `signal-utils` package in editable mode.

3. Install extension: Black Formatter by Microsoft
4. Install extension: Jupyter by Microsoft
5. Install extension: Edit CSV by janisdd
