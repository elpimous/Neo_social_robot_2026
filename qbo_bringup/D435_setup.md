# ✅ D435 Setup — Installing `librealsense` and `realsense2_camera` for ROS 2 Humble on JetPack 6.2

This guide explains how to install the RealSense SDK (`librealsense`) and the ROS 2 wrapper (`realsense2_camera`) to use the Intel RealSense D435 camera on an NVIDIA Jetson with JetPack 6.2 and ROS 2 Humble.

---

## 🧾 1. Install RealSense SDK (`librealsense`) on JetPack 6.2

### 📦 Install required dependencies

```bash
sudo apt update
sudo apt install -y git cmake build-essential libusb-1.0-0-dev
sudo apt install -y libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
```

### ⚙️ Set CUDA environment variables

Add to your `.bashrc` (or run directly):

```bash
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
export CUDACXX=/usr/local/cuda/bin/nvcc
source ~/.bashrc
```

### 🛠️ Clone and build `librealsense` (version `v2.56.3` compatible with JetPack 6.2)

```bash
cd ~
git clone https://github.com/IntelRealSense/librealsense.git -b v2.56.3
cd librealsense
mkdir build && cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_WITH_CUDA=ON \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_TOOLS=OFF \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DCMAKE_CUDA_ARCHITECTURES=87

make -j$(nproc)
sudo make install
```

---

## 🔧 Add UDEV rules for RealSense

```bash
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG video $USER
sudo usermod -aG plugdev $USER
```

Then **unplug and replug** the camera, and test:

```bash
lsusb
rs-enumerate-devices
```

You should see the D435 recognized and listed.

---

## 🤖 Install `realsense2_camera` for ROS 2 Humble

```bash
cd ~/qbo_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
cd realsense-ros
git fetch --all
git checkout -b realsense-v4.56.3 4.56.3

cd ~/qbo_ws
colcon build --symlink-install --packages-select realsense2_camera
colcon build
source install/setup.bash
```

---

## 🎥 Launch the D435 Camera Node

```bash
ros2 launch realsense2_camera rs_launch.py
```

You should see topics like:

```bash
/camera/color/image_raw
/camera/depth/image_rect_raw
/camera/imu
/camera/aligned_depth_to_color/image_raw
```

---

## 🖼️ Visualize in `rqt_image_view`

```bash
rqt_image_view
```

Then select `/camera/color/image_raw` from the dropdown menu to view the RGB stream.

---

✅ You're now ready to use the RealSense D435 with ROS 2 on your Jetson Orin NX!

---

## 🐞 Troubleshooting — Compilation errors in `realsense2_camera`

If you encounter build errors such as:

```text
error: ‘class rs2::motion_frame’ has no member named ‘get_combined_motion_data’
error: ‘class rs2::auto_calibrated_device’ has no member named ‘get_calibration_config’
```

It usually means that your ROS 2 workspace is compiling `realsense2_camera` against an incompatible version of `librealsense2`.

This happens if `ros-humble-librealsense2` is installed from the package manager, and it overrides the manually compiled SDK.

---

### ✅ Solution: Remove conflicting system package and recompile cleanly

1. **Check if the conflicting package is installed**:

```bash
dpkg -l | grep librealsense
```

If you see:

```text
ros-humble-librealsense2
```

→ this must be removed.

2. **Remove the conflicting version**:

```bash
sudo apt remove ros-humble-librealsense2
```

3. **Clean and rebuild your ROS 2 workspace**:

```bash
cd ~/qbo_ws
rm -rf build/ install/ log/
colcon build --symlink-install \
  --packages-select realsense2_camera \
  --cmake-args -DCMAKE_PREFIX_PATH=/usr/local
```

4. **Verify the correct library is linked**:

```bash
ldd install/realsense2_camera/lib/realsense2_camera/realsense2_camera_node | grep librealsense
```

Expected result:

```text
librealsense2.so => /usr/local/lib/librealsense2.so
```

---

### 💡 Tip

Always avoid mixing:
- `librealsense` compiled manually in `/usr/local`  
**with**
- ROS packages prebuilt from APT that ship their own version of `librealsense`.

They will conflict unless you're very strict with `CMAKE_PREFIX_PATH`.

---
