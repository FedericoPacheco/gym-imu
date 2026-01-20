# IMU setup (Linux Ubuntu)

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

    2.5. Install esp32 dependencies:

    ```bash
    pio platform install espressif32
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

    3.3. Configure in VSCode:
    File > Preferences > Settings:
        - Search "Editor: Default Formatter" > Select "clangd".
        - Search "Format On Save" > Enable (check the box).
        - Search "C_Cpp: Code Analysis Clang Tidy: Enabled" > Enable (check the box).
        - Search "C_Cpp: Intelli Sense Engine" > Select "Disabled".
        - Search "Platformio-ide: Auto Rebuild Autocomplete Index" > Disable (uncheck the box).
        - Search "Clangd: Path" > Set to "/usr/bin/clangd".
        - Search "Clangd: Arguments" > Add:
          - `--background-index`
          - `--clang-tidy`
          - `--header-insertion=iwyu`
          - `--completion-style=detailed`
          - `--compile-commands-dir=${workspaceFolder}/.pio/build/device`
        - Search "Clangd: Fallback Flags" > Add:
          - "-I${workspaceFolder}/include"
          - "-I${workspaceFolder}/lib"
          - "-I${workspaceFolder}/components"

    Note: couldn't make clangd refactors work :(

4. Flash device with firmware:
    5.1. Plug the device while pressing the BOOT button.
    5.2. Open PlatformIO > seeed_xiao_esp32c3 > General > Upload
    5.3. Wait for the process to finish.
