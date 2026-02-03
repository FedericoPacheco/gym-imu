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
            - `"-I${workspaceFolder}/include"`
            - `"-I${workspaceFolder}/lib"`
            - `"-I${workspaceFolder}/components"`

    Note: couldn't make clangd refactors work :(

4. Install external dependencies not available in PlatformIO:
    4.1. Create folder and download repos:

    ```bash
        cd device
        mkdir -p components
        git clone https://github.com/natanaeljr/esp32-MPU-driver.git MPU
        git clone https://github.com/natanaeljr/esp32-I2Cbus.git I2Cbus
    ```

    4.2. Configure driver:

    ```bash
        pio run -t menuconfig
    ```

    Select: MPU driver:
        - MPU chip model > MPU6050
        - Communication Protocol > I2C
        - Digital Motion Processor (DMP) > Enable
    Press "S" to save config

    4.3. If building error fails due to missing `i2c1` object, wrap every usage of `i2c1` with:

    ```c++
        #if SOC_I2C_NUM > 1
        // code using i2c1
        #endif
    ```

    Commit the changes:

    ```bash
        cd device/components/I2Cbus
        git add .
        git commit -m "fix(I2Cbus): resolve compatibility with esp32-c3"
    ```

    The esp32-c3 microcontroller only has one I2C bus.

    4.4. If the sensor fails to initialize, add another valid "who am i" value to `MPU.testConnection()`:

    ```c++
      ...
      #if defined CONFIG_MPU6000 || defined CONFIG_MPU6050 || defined CONFIG_MPU9150
        return (wai == 0x68 || wai == ...) ? ESP_OK : ESP_ERR_NOT_FOUND;
      ...
    ```

    Get the wai value by reviewing the logs on the serial monitor, or with `sensor.whoAmI()`.

    Commit the changes:

    ```bash
        cd device/components/MPU
        git add .
        git commit -m "fix(MPU): add support for fake/clone/counterfeit MPU6050 device"
    ```

5. Flash device with firmware:
    5.1. Plug the device while pressing the BOOT button.
    5.2. Open PlatformIO > device > General:
     - Build
     - Upload / Upload and Monitor
