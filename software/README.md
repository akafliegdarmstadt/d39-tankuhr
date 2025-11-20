# D39 FQI Software

## Install dependencies

Install the esp-idf toolchain:

```
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
git clone -b v5.5.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32,esp32c3
```

## Building

Set up the environment for the toolchain:

```
source ./export.sh
```

Build:

```
idf.py build
```

Flash:

```
idf.py flas
```

For serial output:

```
idf.py monitor
```

