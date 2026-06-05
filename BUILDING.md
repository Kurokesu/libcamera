<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->

# Building Kurokesu's libcamera and rpicam-apps forks from source

On Raspberry Pi, `libcamera` and `rpicam-apps` must be rebuilt together. Detailed instructions are available [here](https://www.raspberrypi.com/documentation/computers/camera_software.html#advanced-rpicam-apps), but for convenience, here is a shorter version.

Remove pre-installed `rpicam-apps`:

```bash
sudo apt remove --purge rpicam-apps
```

## libcamera

Install dependencies:

```bash
sudo apt install -y libboost-dev
sudo apt install -y libgnutls28-dev openssl libtiff5-dev pybind11-dev
sudo apt install -y qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5
sudo apt install -y meson cmake
sudo apt install -y python3-yaml python3-ply
sudo apt install -y libglib2.0-dev libgstreamer-plugins-base1.0-dev
```

Clone Kurokesu's `libcamera` fork:

```bash
cd ~
git clone https://github.com/Kurokesu/libcamera.git --branch kurokesu
cd libcamera/
```

Configure with `meson`:

```bash
meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=enabled -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
```

Build:

```bash
ninja -C build
```

Install:

```bash
sudo ninja -C build install
```

> [!TIP]
> On devices with 1 GB of memory or less, the build may exceed available memory. Append `-j 1` to limit it to a single process.

> [!WARNING]
> `libcamera` does not yet have a stable binary interface. Always build `rpicam-apps` after building `libcamera`.

## rpicam-apps

Install dependencies:

```bash
sudo apt install -y cmake libboost-program-options-dev libdrm-dev libexif-dev
sudo apt install -y libavcodec-dev libavdevice-dev libavformat-dev libswresample-dev
sudo apt install -y libepoxy-dev libpng-dev
```

Clone Kurokesu's `rpicam-apps` fork:

```bash
cd ~
git clone https://github.com/Kurokesu/rpicam-apps.git --branch kurokesu
cd rpicam-apps
```

Configure with `meson` (libav enabled by default):

```bash
meson setup build -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled
```

> [!IMPORTANT]
> On Raspberry Pi OS **Bookworm**, packaged `libav*` is **too old** for `rpicam-apps` newer than v1.9.0.

<details>
<summary>Bookworm libav workaround</summary>

Bookworm ships `libavcodec` **59.x** while newer `rpicam-apps` expects **libavcodec >= 60**, causing build errors like "libavcodec API version is too old" (see [Raspberry Pi forum thread](https://forums.raspberrypi.com/viewtopic.php?t=392649)).

- **Keep libav** by checking out `rpicam-apps` **v1.9.0** before running `meson setup` (drops the `kurokesu` branch changes, so AR0822 eHDR will **not** be available):
  ```bash
  git checkout v1.9.0
  ```
- **Disable libav** if building `rpicam-apps` > v1.9.0 (keeps AR0822 eHDR support):
  ```bash
  meson setup build -Denable_libav=disabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled
  ```

</details>

Build:

```bash
meson compile -C build
```

Install:

```bash
sudo meson install -C build
```

> [!TIP]
> This should automatically update `ldconfig` cache. If you have trouble accessing your new build, update manually:
>
> ```bash
> sudo ldconfig
> ```

Verify new binary is used:

```bash
rpicam-still --version
```

The output should contain the date and time of your local `rpicam-apps` build.
