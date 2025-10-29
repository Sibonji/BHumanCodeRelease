# white_lines_zed_detector

Booster K1 vision subsystem for stereoscopic white lines detection

!!! Should be compiled with:
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```

## Prerequisites

- CMake (version 3.10 or higher)
- OpenCV library
- C++ compiler with C++17 support
- ZED SDK

## Building the Project

1. Create a build directory:
```bash
mkdir build
cd build
```

2. Configure and build the project:
```bash
cmake ..
make
```

## Running the Program

Run with a live stream:
```bash
./white_lines_zed_detector --camera
```

Run the program by providing a path to a PNG image:

```bash
./white_lines_zed_detector path/to/your/image.png
```
