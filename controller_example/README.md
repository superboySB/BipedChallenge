# My Drake Examples

baesd on [`CMake Project with Drake Installed Using APT`](https://github.com/RobotLocomotion/drake-external-examples/tree/main/drake_cmake_installed_apt)

This example uses the [`cmake`](https://cmake.org/) build system with an
instance of Drake installed using the
[APT](https://manpages.ubuntu.com/manpages/focal/man8/apt.8.html) package
manager.

## Instructions

Install the `drake-dev` APT package by following the instructions found at:

<https://drake.mit.edu/from_binary.html#apt-packages-for-monthly-tagged-releases>

For this example, also install the `build-essential` and `cmake` APT packages:

```sh
sudo apt-get update
sudo apt-get --no-install-recommends install build-essential cmake 
```

The controller uses the SDL2 library. It can be installed using apt on Ubuntu 20:

```sh
sudo apt install libsdl2-dev
sudo apt install libsdl2-ttf-dev
```

## compile
```sh
mkdir build
cd build
cmake ..
make
```

The taskxx_controller executable files for each level will be compiled, and can be modified as needed.

## Introduction of content

The control program of biped_sim in src/biped is based on ALIP's dead-beat planning, and task space PFL (partial feedback linearization) control.

