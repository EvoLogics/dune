# Compiling and Running

This page explains how to:

- Compile DUNE on the Jetson board
- Run DUNE on the Jetson board
- Receive the WIC video feed from the Jetson board on your local machine.

> **Warning**<br>
> Make sure you have completed the steps in [Setting Up](Setup.md) before continuing!


## Compiling DUNE

On the Jetson board:

```
cd ~/dev/dune/build
make
```

## Running DUNE and receiving the WIC video feed

- Start DUNE on the Jetson board:

  ```
  cd ~/dev/dune
  ./build/dune -c testing/wic -p Simulation
  ```
  
  You should start to see log messages appearing in the terminal where you are running DUNE.
  <details>
    <summary>Example of DUNE logs</summary>

    ```
    [2023/03/30 13:37:45] - MSG [Daemon] >> system name: 'wic-test' (65535)
    [2023/03/30 13:37:45] - MSG [Daemon] >> registered tasks: 5
    [2023/03/30 13:37:45] - MSG [Daemon] >> base folder: '/home/jetson/dev/dune/build'
    [2023/03/30 13:37:45] - MSG [Daemon] >> configuration folder: '/home/jetson/dev/dune/etc'
    [2023/03/30 13:37:45] - MSG [Daemon] >> web server folder: '/home/jetson/dev/dune/www'
    [2023/03/30 13:37:45] - MSG [Daemon] >> log folder: '/home/jetson/dev/dune/build/log/wic-test'
    [2023/03/30 13:37:45] - MSG [Daemon] >> library folder: '/home/jetson/dev/dune/build'
    [2023/03/30 13:37:45] - MSG [Daemon] >> execution profiles: Simulation
    [2023/03/30 13:37:45] - MSG [Sensors.WIC] >> increased task stack size to: 8388608
    [2023/03/30 13:37:45] - MSG [Daemon] >> daemon running with maximum priority: 99
    [2023/03/30 13:37:45] - MSG [Sensors.WIC] >> starting
    [2023/03/30 13:37:45] - MSG [Transports.HTTP] >> starting
    [2023/03/30 13:37:45] - MSG [Sensors.WIC] >> disconnected from WIC
    [2023/03/30 13:37:45] - MSG [Transports.Logging] >> starting
    [2023/03/30 13:37:45] - MSG [Transports.HTTP] >> listening on 0.0.0.0:8080
    [2023/03/30 13:37:45] - MSG [Transports.UDP] >> starting
    [2023/03/30 13:37:45] - MSG [Transports.UDP] >> listening on 0.0.0.0:6002
    [2023/03/30 13:37:45] - MSG [Transports.Logging] >> log started '20230330/133745'
    CameraCenter: Authentication success for camera iPORT-NTx-GigE-PT01-PB3IP01-128 00:11:1c:05:04:e1 [192.168.3.40]
    CameraCenter: Connecting WIC devices results:
            - Found interfaces: 9
            - Found cameras: 1
            - Authenticated cameras: 1
    Camera: Connecting to iPORT-NTx-GigE-PT01-PB3IP01-128 00:11:1c:05:04:e1 [192.168.3.40].
    CameraSerialSettings::SetDefault(): Setting default settings
    [2023/03/30 13:37:52] - MSG [Sensors.WIC] >> connected to WIC
    [2023/03/30 13:37:52] - MSG [Sensors.WIC] >> updating camera setting 'FFC' to 'Manual'
    [2023/03/30 13:37:53] - MSG [Sensors.WIC] >> set source frame width to 640
    [2023/03/30 13:37:53] - MSG [Sensors.WIC] >> set source frame height to 512
    [2023/03/30 13:37:53] - MSG [Sensors.WIC] >> set source framerate to 30
    [2023/03/30 13:37:53] - MSG [Sensors.WIC] >> set UDP destination to 192.168.3.153
    Camera: Starting Acquisition
    [2023/03/30 13:37:53] - MSG [Sensors.WIC] >> (re)starting pipeline
    [2023/03/30 13:37:53] - MSG [Sensors.WIC] >> pipeline restarted
    [2023/03/30 13:37:53] - MSG [Sensors.WIC] >> getting camera data
    ```
  </details>
  <br>

- On your local machine, start the Docker container to receive and display the video feed:

  ```
  cd docker/test-pipelines
  ./run.sh -r
  ```
  
  After a few seconds, a window should appear with the video feed as well as some framerate information. 


> **Note**<br>
> If no window appears:
>  - check you have performed step 2 in the [Setting Up](Setup.md) file.
>  - check the DUNE logs do not show an error.

