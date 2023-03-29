# DUNE Quickstart

DUNE is a framework that is split into units called **Tasks**.

The source code for all Tasks can be found in the `src/` folder.<br>
Each Task has at least a `Task.cpp` file (the main file) and a `Task.cmake` file (for building).

To decide which Tasks are run, DUNE uses configuration files called `.ini` files.<br>
They are located in the `etc/` folder.

The files for the WIC Driver can be found at `src/Sensors/WIC/`, and the configuration file at `etc/testing/wic.ini`.<br>
More specifically, the class that wraps around the WIC SDK is located at `Sensors/WIC/CameraInterface.hpp`
