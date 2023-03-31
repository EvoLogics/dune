# WIC Integration Issues

EvoLogics uses the [LSTS DUNE framework](https://github.com/LSTS/dune) for its autonomous devices.<br>
(See [DUNE Quickstart](DuneQuickstart.md) for a very short intro to DUNE.)<br>
We are writing a DUNE driver for the Workswell WIC infrared camera.<br>

The driver should:
 - Configure the camera and grab the video frames, using the [Workswell ARM 1.1.0 SDK](https://software.workswell.eu/wic_sdk/ARM/doc/).
 - Encode and stream the video data over UDP to a predefined address, using [GStreamer](https://gstreamer.freedesktop.org/).


## Issues

Before continuing, please follow the [Setting Up](Setup.md) and [Compiling and Running](CompilingAndRunning.md) guides.<br>
They explain how to create a working example of DUNE with the WIC driver, to which we can compare the non-working cases listed below.


### :heavy_exclamation_mark: Issue 1: Stack smashing

**Description**

In the working example above, only the bare minimum of Tasks are compiled necessary for the driver.
However, when all Tasks are compiled, DUNE crashes with a "stack smashing detected" error somewhere in the WIC SDK.
Note that the same amount of Tasks are run as in the working example, the only change was in compilation.

**To reproduce**

- Checkout branch `test/workswell_wic_640_stack_smashing`<br>
- Push, compile and run DUNE on the Jetson board as described in the guides.
- During connection to the WIC camera, DUNE should crash with the message:
  ```
  *** stack smashing detected ***: <unknown> terminated
  ```

**Stacktrace**
<details>
  <summary>GDB Stacktrace</summary>

  ```
  #0  0x0000007fb789b598 in __GI_raise (sig=sig@entry=6) at ../sysdeps/unix/sysv/linux/raise.c:51
  #1  0x0000007fb789c974 in __GI_abort () at abort.c:79
  #2  0x0000007fb78d572c in __libc_message (action=do_abort, fmt=fmt@entry=0x7fb7995350 "*** %s ***: %s terminated\n")
      at ../sysdeps/posix/libc_fatal.c:181
  #3  0x0000007fb794a3c4 in __GI___fortify_fail_abort (need_backtrace=need_backtrace@entry=false, msg=msg@entry=0x7fb7995328 "stack smashing detected") at fortify_fail.c:33
  #4  0x0000007fb794a378 in __stack_chk_fail () at stack_chk_fail.c:29
  #5  0x0000007fb7839164 in PvBaseLib::CapabilitiesGEV::CapabilitiesGEV(EbUtilsLib::Network::EthernetAddress) ()
      at /opt/pleora/ebus_sdk/linux-aarch64-arm/lib/libPvBase.so.5.0.0.4100
  #6  0x0000007fb783b494 in PvBaseLib::CapabilitiesGEV::GetCapabilities(EbUtilsLib::Network::EthernetAddress, unsigned char*) ()
      at /opt/pleora/ebus_sdk/linux-aarch64-arm/lib/libPvBase.so.5.0.0.4100
  #7  0x0000000000000000 in  ()
  ```
</details>
<br>

**What we have tried**

To get more information, we have disabled stack smashing checks (canaries) using the compiler option `-fno-stack-protector`.
Unfortunately, the issue persists, probably because the WIC library is compiled with this check enabled.

**Questions**

 - What causes this stack smashing?
 - How can we solve this?


### :heavy_exclamation_mark: Issue 2: AGC setting segfault

**Description**

Trying to change the AGC setting on the WIC camera from "PlateauHistogram" to "Manual" causes a segmentation fault.<br>
This does not happen for other settings (FFC, Palette, Range, etc...).

**To reproduce**

- Checkout branch `test/workswell_wic_640_agc_segfault`<br>
  This branch attempts to set the AGC Type on the WIC to "Manual" during initialization.
- Push, compile and run DUNE on the Jetson board.
- DUNE should crash with a segmentation fault.

<details>
  <summary>GDB Stacktrace</summary>

  ```
  #0  0x0000007fb7e92e68 in CameraSerialSettings::SetParameter(CameraSerialSettings::FunctionCodes, void*, CameraSerialSettings::RadiometricParameters, CameraSerialSettings::DigitalOutputModes) () at /opt/workswell/wic_sdk/lib/libWIC_SDK.so
  #1  0x0000007fb7e939fc in CameraSerialSettings::SetAGCType(CameraSerialSettings::AGCTypes) ()
      at /opt/workswell/wic_sdk/lib/libWIC_SDK.so
  #2  0x00000055556f56d4 in Sensors::WIC::CameraInterface::updateSettings(Sensors::WIC::CameraSettings const&) ()
  #3  0x00000055556f7070 in Sensors::WIC::Task::updateCameraSettings() ()
  #4  0x00000055556f77e8 in Sensors::WIC::Task::onResourceInitialization() ()
  #5  0x00000055556d71c8 in DUNE::Tasks::Task::run() ()
  #6  0x00000055556e80a8 in dune_concurrency_thread_entry_point ()
  #7  0x0000007fb7f88088 in start_thread (arg=0x7fb7fcf71f) at pthread_create.c:463
  #8  0x0000007fb79830cc in thread_start () at ../sysdeps/unix/sysv/linux/aarch64/clone.S:78
  ```
</details>
<br>

**Questions:**

- Why does changing the AGC setting cause a segmentation fault, while other settings are fine?


### :heavy_exclamation_mark: Issue 3: Fail to set 16 bit CMOS depth

**To reproduce:**

- Checkout branch `test/workswell_wic_640_16_bit_failure`<br>
  This branch attempts to set the CMOS bit depth to "16 bit YCbCr" instead of the default "14 bit RAW".
- Push, compile and run DUNE on the Jetson board.
- The following error should appear:
  ```
  ERR [Sensors.WIC] >> failed to update following camera settings: 'digital output depth' to '16 bit YCbCr'
  ```

**Questions:**

 - Why is it not possible to set 16 bit CMOS bit depth?

