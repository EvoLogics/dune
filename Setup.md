# Setting Up :wrench:

This page explains the necessary setup steps for testing WIC with DUNE on a Jetson board.

The main idea is to clone DUNE on your local machine so that you can browse the code and make changes using your editor(s).
You then copy the DUNE source files to the Jetson board, compile and run DUNE there.

## Requirements

You will need the following:
- License file for the camera
- Zip archive of the WIC SDK 1.1.0 for ARM
- WIC 640 camera:
    - Ethernet version
- NVidia Jetson board:
    - Jetson Xavier NX (8GB version)
    - Ubuntu 16.04 or 18.04
- Local machine:
    - Has bash interpreter
    - Docker installed

> **Warning**<br>
> The local machine, the Jetson board and the WIC camera should be on the same network.


## Setup

1. Clone this repository, switch to the correct branch and update the submodules:
   ```
   git checkout test/workswell_wic_640
   git submodule update --init --recursive
   ```

1. In `etc/testing/wic.ini`, change `UDP Destination` to the local machine's IP address, then save the file[^2]. 

1. Turn on the Jetson board and make sure you are connected and that you know the IP address and the name of the user on the board.<br>

1. Make sure the WIC camera is powered on and connected to the Jetson board.

1. If you do not have a public SSH key, follow the instructions [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) to generate one.

1. Add your SSH key to the Jetson board[^1]:
   ```
   ssh-copy-id -i ~/.ssh/<PUBLIC_KEY_NAME>.pub <JETSON_USER>@<JETSON_IP>
   ```

1. Execute the script `programs/scripts/wic-helpers/jetson_install.sh` (run with `-h` to see usage).

1. Execute the script `programs/scripts/wic-helpers/jetson_update-dune.sh` (run with `-h` to see usage).

1. Create the DUNE build directory: SSH into the Jetson board and execute the following:
   ```
   cd ~/dev/dune
   mkdir build/
   cd build/
   cmake ..
   ```

1. On your local machine, execute the script `docker/test-pipelines/build.sh` to build the Docker image to receive the WIC video stream[^3].<br>

---

**Footnotes**

[^1]: The scripts used in this document make use of SSH; adding your key to the Jetson board prevents you from having to re-enter the board's password every time you SSH into it.
[^2]: DUNE will send the video stream to your machine so that you can see it. For that, it needs to know your IP address.
[^3]: This Docker image runs a GStreamer pipeline that receives and displays the video stream sent by DUNE from the Jetson board.
