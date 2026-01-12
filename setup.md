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
    pio platform update espressif32
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

3. Install C++ tools (formatter, static analysis, make):
    3.1. Install the VSCode extensions:
        - C/C++
        - Clang-Format
        - C/C++ Advanced Lint
        - CMake Tools
    3.2. Install clang:

    ```bash
    sudo apt install clang
    clang --version
    ```

    3.3. Configure in VSCode:
    File > Preferences > Settings:
        - Search "Editor: Default Formatter" > Select "Clang-Format".
        - Search "Format On Save" > Check the box to enable it.
        - Search "C_CppCode Analysis Clang Tidy: Enabled" > Check the box to enable it.

4. Flash device with firmware:
    5.1. Plug the device while pressing the BOOT button.
    5.2. Open PlatformIO > seeed_xiao_esp32c3 > General > Upload
    5.3. Wait for the process to finish.
