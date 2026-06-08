# ORB-SLAM3 with Azure Kinect DK on Ubuntu 24.04
### Complete RGB-D Inertial Setup Guide — With and Without ROS 2

> **Tested Environment**
> - OS: Ubuntu 24.04 LTS (Noble Numbat)
> - Camera: Azure Kinect DK (USB 3.0)
> - ROS 2: Jazzy Jalisco *(optional path)*
> - Architecture: x86\_64 / amd64
> - Shell: zsh

---

## Table of Contents

1. [System Preparation & Core Dependencies](#1-system-preparation--core-dependencies)
2. [Azure Kinect SDK Installation](#2-azure-kinect-sdk-installation)
3. [Build Pangolin](#3-build-pangolin)
4. [Build ORB-SLAM3](#4-build-orb-slam3)
5. [Camera Calibration & YAML Configuration](#5-camera-calibration--yaml-configuration)
6. [Path A — Native C++ (No ROS) ✅ Confirmed Working](#path-a--native-c-no-ros)
7. [Path B — ROS 2 Jazzy (RGB-D only)](#path-b--ros-2-jazzy)
8. [Troubleshooting Reference](#troubleshooting-reference)

---

## 1. System Preparation & Core Dependencies

Ubuntu 24.04 ships with **GCC 13**, **OpenCV 4.x**, and **Eigen 3.4** — all newer than what ORB-SLAM3 originally targeted. The steps below account for every known incompatibility.

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  libgtk2.0-dev pkg-config \
  libavcodec-dev libavformat-dev libswscale-dev \
  libglew-dev libepoxy-dev libtbb-dev \
  libjpeg-dev libpng-dev libtiff-dev \
  libsuitesparse-dev \
  qtdeclarative5-dev qt5-qmake \
  libqglviewer-dev-qt5 \
  libgoogle-glog-dev \
  libeigen3-dev
```

### 1.1 Install Sophus

The ROS 2 wrapper requires Sophus (a Lie group math library). Build it from source:

```bash
cd ~
git clone https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

---

## 2. Azure Kinect SDK Installation

Microsoft only publishes official packages for Ubuntu 18.04. The strategy is to install those `.deb` files manually — they work fine on 24.04.

### 2.1 Register the Microsoft Repository

```bash
curl -sSL https://packages.microsoft.com/config/ubuntu/18.04/prod.list \
  | sudo tee /etc/apt/sources.list.d/microsoft-prod.list

curl -sSL https://packages.microsoft.com/keys/microsoft.asc \
  | sudo apt-key add -

sudo apt update
```

### 2.2 Install the Runtime & Development Libraries

```bash
sudo apt install libk4a1.4 libk4a1.4-dev libk4arecord1.4 libk4arecord1.4-dev
```

### 2.3 Install the Tools (k4aviewer / k4arecorder)

The `k4a-tools` package requires `libsoundio1`, which is no longer in Ubuntu 24.04 repos. Install it manually:

```bash
# libsoundio2 satisfies the general audio dependency
sudo apt install libsoundio2 libsoundio-dev

# Manually provide the legacy libsoundio1 that k4a-tools specifically requires
wget http://archive.ubuntu.com/ubuntu/pool/universe/libs/libsoundio/libsoundio1_1.0.2-1_amd64.deb
sudo dpkg -i libsoundio1_1.0.2-1_amd64.deb
sudo apt install -f

sudo apt install k4a-tools
```

> **If you also see a `libssl1.1` error:**
> ```bash
> wget http://archive.ubuntu.com/ubuntu/pool/main/o/openssl/libssl1.1_1.1.1f-1ubuntu2_amd64.deb
> sudo dpkg -i libssl1.1_1.1.1f-1ubuntu2_amd64.deb
> ```

### 2.4 Configure udev Rules (enables running without sudo)

```bash
sudo nano /etc/udev/rules.d/99-k4a.rules
```

Paste the following:

```text
# Azure Kinect DK — USB device permissions
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="097a", MODE="0666", GROUP="plugdev", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="097b", MODE="0666", GROUP="plugdev", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="097c", MODE="0666", GROUP="plugdev", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="097d", MODE="0666", GROUP="plugdev", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="045e", ATTR{idProduct}=="097e", MODE="0666", GROUP="plugdev", TAG+="uaccess"
```

Apply the rules and add your user to the `plugdev` group:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger

sudo usermod -aG plugdev $USER
# Log out and back in for the group change to take effect
```

Unplug and re-plug the Kinect, then verify detection:

```bash
lsusb | grep -i Microsoft
k4aviewer   # should open without sudo
```

### 2.5 Update Firmware (Recommended)

Older firmware produces sync warnings. Download and flash the latest:

```bash
# Download from: https://github.com/microsoft/Azure-Kinect-Sensor-SDK/releases
# File: AzureKinectDK_Fw_1.6.110080014.bin

sudo AzureKinectFirmwareTool -u ~/Downloads/AzureKinectDK_Fw_1.6.110080014.bin
sudo AzureKinectFirmwareTool -q   # verify the new version
```

---

## 3. Build Pangolin

ORB-SLAM3's visualization requires a recent Pangolin build. The system package is often too old.

```bash
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

> **If `cmake ..` fails with missing `epoxy`:** `sudo apt install libepoxy-dev`, then re-run cmake.

---

## 4. Build ORB-SLAM3

### 4.1 Clone the Repository

```bash
cd ~
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3
```

### 4.2 Apply C++14 Fix (Required for Ubuntu 24.04)

GCC 13 + OpenCV 4 require at least C++14. The original repo targets C++11.

```bash
sed -i 's/++11/++14/g' CMakeLists.txt
```

Optionally verify the change:

```bash
grep "CXX_STANDARD" CMakeLists.txt
```

### 4.3 Place the Azure Kinect Driver Source

Copy your `rgbd_inertial_azure_kinect_dk.cc` file into the ORB-SLAM3 examples folder:

```bash
cp /path/to/rgbd_inertial_azure_kinect_dk.cc \
   ~/ORB_SLAM3/Examples/RGB-D-Inertial/
```

Then register it in `~/ORB_SLAM3/CMakeLists.txt` by adding at the bottom of the file:

```cmake
# ── Azure Kinect DK RGB-D Inertial ──────────────────────────────────────────
find_package(k4a REQUIRED)

add_executable(rgbd_inertial_azure_kinect_dk
    Examples/RGB-D-Inertial/rgbd_inertial_azure_kinect_dk.cc)

target_link_libraries(rgbd_inertial_azure_kinect_dk
    ${PROJECT_NAME}
    k4a::k4a
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    -lboost_serialization
    -lcrypto)

set_target_properties(rgbd_inertial_azure_kinect_dk PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D-Inertial)
```

### 4.4 Build

```bash
chmod +x build.sh
./build.sh
```

> **Thermal throttling tip (laptops):** If the build crashes your machine, edit `build.sh` to replace `make -j$(nproc)` with `make -j2`.

Verify the output:

```bash
ls ~/ORB_SLAM3/lib/                        # libORB_SLAM3.so
ls ~/ORB_SLAM3/Thirdparty/DBoW2/lib/      # libDBoW2.so
ls ~/ORB_SLAM3/Thirdparty/g2o/lib/        # libg2o.so
ls ~/ORB_SLAM3/Examples/RGB-D-Inertial/   # rgbd_inertial_azure_kinect_dk
```

### 4.5 Extract the Vocabulary

```bash
cd ~/ORB_SLAM3/Vocabulary
tar -xf ORBvoc.txt.tar.gz
ls -lh ORBvoc.txt   # should be ~140 MB
```

---

## 5. Camera Calibration & YAML Configuration

### 5.1 Set Up the Python Extraction Environment

`pyk4a` must be installed in a virtual environment (Ubuntu 24.04 enforces PEP 668):

```bash
cd ~/ORB_SLAM3/Examples
python3 -m venv kinnect
source kinnect/bin/activate
pip install pyk4a
```

Place `parameters.py` in the RGB-D-Inertial examples folder:

```bash
cp /path/to/parameters.py ~/ORB_SLAM3/Examples/RGB-D-Inertial/
```

### 5.2 Run the Extraction Script

With the camera plugged in via USB 3.0:

```bash
~/ORB_SLAM3/Examples/kinnect/bin/python \
  ~/ORB_SLAM3/Examples/RGB-D-Inertial/parameters.py
```

<details>
<summary>📄 parameters.py — Full Source</summary>

```python
import pyk4a
from pyk4a import PyK4A, CalibrationType
import numpy as np

def get_azure_kinect_parameters():
    """
    Connects to the Azure Kinect device, extracts camera intrinsics
    and IMU-to-Camera extrinsics, and prints them in YAML format.
    """
    # 1. Initialize and start the device
    k4a = PyK4A()
    try:
        k4a.start()
    except Exception as error:
        print(f"Failed to start the device: {error}")
        return

    # 2. Access the calibration data
    calibration = k4a.calibration

    # Extract Intrinsics for the Color Camera
    # Returns a 3x3 camera matrix
    camera_matrix = calibration.get_camera_matrix(CalibrationType.COLOR)
    # Returns distortion coefficients (k1, k2, p1, p2, k3...)
    distortion_coefficients = calibration.get_distortion_coefficients(CalibrationType.COLOR)

    # 3. Extract Extrinsics (IMU to Color Camera)
    # pyk4a returns a tuple: (rotation_matrix, translation_vector)
    try:
        extrinsics_data = calibration.get_extrinsic_parameters(
            CalibrationType.GYRO,
            CalibrationType.COLOR
        )
        rotation_part    = np.array(extrinsics_data[0]).reshape(3, 3)
        translation_part = np.array(extrinsics_data[1])
    except (AttributeError, IndexError) as error:
        print(f"Error retrieving extrinsic parameters: {error}")
        k4a.stop()
        return

    # 4. Construct the 4x4 Transformation Matrix (T_b_c)
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_part
    # Convert translation from millimetres (SDK default) to metres (SLAM requirement)
    transformation_matrix[:3, 3] = translation_part / 1000.0

    # 5. Output results for ORB-SLAM3 YAML configuration
    print("--- COPY THESE TO YOUR YAML ---")
    print(f"Camera1.fx: {camera_matrix[0, 0]}")
    print(f"Camera1.fy: {camera_matrix[1, 1]}")
    print(f"Camera1.cx: {camera_matrix[0, 2]}")
    print(f"Camera1.cy: {camera_matrix[1, 2]}")
    print(f"Camera1.k1: {distortion_coefficients[0]}")
    print(f"Camera1.k2: {distortion_coefficients[1]}")
    print(f"Camera1.p1: {distortion_coefficients[2]}")
    print(f"Camera1.p2: {distortion_coefficients[3]}")
    print(f"Camera1.k3: {distortion_coefficients[4]}")

    print("\n# IMU to Camera Transformation (T_b_c1)")
    print("IMU.T_b_c1: !!opencv-matrix")
    print("   rows: 4\n   cols: 4\n   dt: f")
    flat_matrix = transformation_matrix.flatten()
    print(f"   data: [{', '.join(map(str, flat_matrix))}]")

    k4a.stop()

if __name__ == "__main__":
    get_azure_kinect_parameters()
```

</details>

### 5.3 Create Your YAML File

Save as `~/ORB_SLAM3/Examples/RGB-D-Inertial/Kinnect_DK.yaml`, filling in your extracted values.

<details>
<summary>📄 Kinnect_DK.yaml — Full Configuration</summary>

```yaml
%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters for Azure Kinect DK
#--------------------------------------------------------------------------------------------
File.version: "1.0"

Camera.type: "PinHole"

# ── Fill in YOUR values from parameters.py ──────────────────────────────────
Camera1.fx: 611.04150390625
Camera1.fy: 610.7039794921875
Camera1.cx: 637.584228515625
Camera1.cy: 369.2159729003906

Camera1.k1:  0.6664037704467773
Camera1.k2: -2.7947685718536377
Camera1.p1:  0.0005224059568718076
Camera1.p2: -0.00040463212644681334
Camera1.k3:  1.5301569700241089

# Resolution — cx ≈ 637 confirms 1536P (1280×720)
Camera.width:  1280
Camera.height: 720

# Camera.fps MUST be an integer
Camera.fps: 30

# 1 = RGB colour order (Azure Kinect outputs BGR, converted in the driver)
Camera.RGB: 1

Stereo.ThDepth: 40.0
Stereo.b: 0.05

# Azure Kinect outputs depth in millimetres (uint16) — divide by 1000 for metres
RGBD.DepthMapFactor: 1000.0

#--------------------------------------------------------------------------------------------
# IMU Parameters — Azure Kinect DK (LSM6DSMUS)
#--------------------------------------------------------------------------------------------
# ── Fill in YOUR T_b_c1 matrix from parameters.py ───────────────────────────
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [0.011187894269824028, -0.9999364614486694, -0.0013695547822862864, -3.205295562744141e-05,
          0.005285903811454773, -0.0013104798272252083, 0.9999852180480957, -1.9444148540496826e-06,
          -0.9999234080314636, -0.011194968596100807, 0.005270920693874359, 4.181725025177002e-06,
          0.0, 0.0, 0.0, 1.0]

IMU.NoiseGyro:  0.00017   # rad/s/√Hz
IMU.NoiseAcc:   0.0028    # m/s²/√Hz
IMU.GyroWalk:   0.00001   # rad/s^1.5
IMU.AccWalk:    0.0008    # m/s^2.5
IMU.Frequency:  200.0

#--------------------------------------------------------------------------------------------
# ORB Extractor Parameters (tuned for 1536P)
#--------------------------------------------------------------------------------------------
ORBextractor.nFeatures:   1500
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels:     8
ORBextractor.iniThFAST:   20
ORBextractor.minThFAST:   7

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize:      0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth:    0.9
Viewer.PointSize:         2.0
Viewer.CameraSize:        0.08
Viewer.CameraLineWidth:   3.0
Viewer.ViewpointX:        0.0
Viewer.ViewpointY:       -0.7
Viewer.ViewpointZ:       -3.5
Viewer.ViewpointF:        500.0
```

> **Common YAML pitfalls:**
> - `Camera.fps` must be a plain integer (`30`), not a float (`30.0`) — ORB-SLAM3 will abort otherwise.
> - Do not mix tabs and spaces in the `IMU.T_b_c1` data block.

</details>

---

## Path A — Native C++ (No ROS)

> ✅ **This is the confirmed working path** for RGB-D Inertial with the Azure Kinect DK.

### A.1 The Driver Source File

Place `rgbd_inertial_azure_kinect_dk.cc` at `~/ORB_SLAM3/Examples/RGB-D-Inertial/` (see §4.3).

<details>
<summary>📄 rgbd_inertial_azure_kinect_dk.cc — Full Source</summary>

```cpp
/**
 * ORB-SLAM3 — Azure Kinect DK RGB-D Inertial Driver
 *
 * Copyright (C) 2017-2021 Carlos Campos et al., University of Zaragoza.
 * Licensed under GPLv3. See <http://www.gnu.org/licenses/>.
 */

#include <signal.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <k4a/k4a.h>
#include <System.h>

using namespace std;

namespace
{
std::atomic_bool g_continue_session{true};

void exit_loop_handler(int)
{
    cout << "Finishing session" << endl;
    g_continue_session = false;
}

bool read_capture(k4a_device_t device, k4a_capture_t &capture, int timeout_ms)
{
    switch (k4a_device_get_capture(device, &capture, timeout_ms))
    {
    case K4A_WAIT_RESULT_SUCCEEDED: return true;
    case K4A_WAIT_RESULT_TIMEOUT:   return false;
    default:
        throw runtime_error("Failed to read capture from Azure Kinect DK");
    }
}
} // namespace

int main(int argc, char **argv)
{
    if (argc < 3 || argc > 4)
    {
        cerr << "\nUsage: ./rgbd_inertial_azure_kinect_dk "
                "path_to_vocabulary path_to_settings [trajectory_file]\n";
        return 1;
    }

    string file_name;
    if (argc == 4)
        file_name = string(argv[argc - 1]);

    // Ensure Pangolin can display on Wayland/XCB systems
    if (getenv("QT_QPA_PLATFORM") == nullptr)
        setenv("QT_QPA_PLATFORM", "xcb", 1);

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, nullptr);

    // ── Open device ──────────────────────────────────────────────────────────
    if (k4a_device_get_installed_count() == 0)
    {
        cerr << "No Azure Kinect DK detected" << endl;
        return 1;
    }

    k4a_device_t device = nullptr;
    if (k4a_device_open(0, &device) != K4A_RESULT_SUCCEEDED)
    {
        cerr << "Failed to open Azure Kinect DK" << endl;
        return 1;
    }

    k4a_device_configuration_t config  = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format                = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution            = K4A_COLOR_RESOLUTION_1536P;
    config.depth_mode                  = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    config.camera_fps                  = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only    = true;

    k4a_calibration_t calibration;
    if (k4a_device_get_calibration(device, config.depth_mode,
                                   config.color_resolution, &calibration)
        != K4A_RESULT_SUCCEEDED)
    {
        cerr << "Failed to get calibration" << endl;
        k4a_device_close(device);
        return 1;
    }

    k4a_transformation_t transformation = k4a_transformation_create(&calibration);
    if (!transformation)
    {
        cerr << "Failed to create transformation" << endl;
        k4a_device_close(device);
        return 1;
    }

    if (k4a_device_start_cameras(device, &config) != K4A_RESULT_SUCCEEDED)
    {
        cerr << "Failed to start cameras" << endl;
        k4a_transformation_destroy(transformation);
        k4a_device_close(device);
        return 1;
    }

    if (k4a_device_start_imu(device) != K4A_RESULT_SUCCEEDED)
    {
        cerr << "Failed to start IMU" << endl;
        k4a_device_stop_cameras(device);
        k4a_transformation_destroy(transformation);
        k4a_device_close(device);
        return 1;
    }

    // ── IMU thread ───────────────────────────────────────────────────────────
    mutex imu_mutex;
    deque<k4a_imu_sample_t> imu_queue;

    thread imu_thread([&]() {
        while (g_continue_session)
        {
            k4a_imu_sample_t imu_sample{};
            switch (k4a_device_get_imu_sample(device, &imu_sample, 1000))
            {
            case K4A_WAIT_RESULT_SUCCEEDED:
            {
                lock_guard<mutex> lock(imu_mutex);
                imu_queue.push_back(imu_sample);
                break;
            }
            case K4A_WAIT_RESULT_TIMEOUT: break;
            default:
                g_continue_session = false;
                break;
            }
        }
    });

    // ── Initialise ORB-SLAM3 ─────────────────────────────────────────────────
    ORB_SLAM3::System SLAM(argv[1], argv[2],
                            ORB_SLAM3::System::IMU_RGBD, true, 0, file_name);
    float image_scale = SLAM.GetImageScale();

    // ── Main capture loop ────────────────────────────────────────────────────
    while (g_continue_session && !SLAM.isShutDown())
    {
        k4a_capture_t capture = nullptr;
        if (!read_capture(device, capture, 1000))
            continue;

        k4a_image_t color_image = k4a_capture_get_color_image(capture);
        k4a_image_t depth_image = k4a_capture_get_depth_image(capture);

        if (!color_image || !depth_image)
        {
            if (color_image) k4a_image_release(color_image);
            if (depth_image) k4a_image_release(depth_image);
            k4a_capture_release(capture);
            continue;
        }

        const int width  = k4a_image_get_width_pixels(color_image);
        const int height = k4a_image_get_height_pixels(color_image);

        // Align depth to colour camera plane
        k4a_image_t depth_to_color = nullptr;
        if (k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, width, height,
                             width * (int)sizeof(uint16_t), &depth_to_color)
            != K4A_RESULT_SUCCEEDED)
        {
            k4a_image_release(color_image);
            k4a_image_release(depth_image);
            k4a_capture_release(capture);
            continue;
        }

        if (k4a_transformation_depth_image_to_color_camera(
                transformation, depth_image, depth_to_color)
            != K4A_RESULT_SUCCEEDED)
        {
            k4a_image_release(depth_to_color);
            k4a_image_release(color_image);
            k4a_image_release(depth_image);
            k4a_capture_release(capture);
            continue;
        }

        // Convert BGRA → RGB
        cv::Mat im_bgra(height, width, CV_8UC4,
                        k4a_image_get_buffer(color_image),
                        k4a_image_get_stride_bytes(color_image));
        cv::Mat im;
        cv::cvtColor(im_bgra, im, cv::COLOR_BGRA2RGB);

        // Aligned depth as float (DepthMapFactor=1000 converts mm→m inside SLAM)
        cv::Mat depth_view(height, width, CV_16U,
                           k4a_image_get_buffer(depth_to_color),
                           k4a_image_get_stride_bytes(depth_to_color));
        cv::Mat depth = depth_view.clone();

        double timestamp =
            static_cast<double>(k4a_image_get_device_timestamp_usec(color_image))
            * 1e-6;

        // Drain IMU samples up to this frame's timestamp
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        {
            lock_guard<mutex> lock(imu_mutex);
            while (!imu_queue.empty())
            {
                const k4a_imu_sample_t &s = imu_queue.front();
                const double t = static_cast<double>(s.gyro_timestamp_usec) * 1e-6;
                if (t > timestamp) break;

                vImuMeas.emplace_back(
                    s.acc_sample.v[0],  s.acc_sample.v[1],  s.acc_sample.v[2],
                    s.gyro_sample.v[0], s.gyro_sample.v[1], s.gyro_sample.v[2],
                    t);
                imu_queue.pop_front();
            }
        }

        // Optional: scale images if YAML specifies Camera.newWidth/newHeight
        if (image_scale != 1.f)
        {
            const int w = static_cast<int>(im.cols * image_scale);
            const int h = static_cast<int>(im.rows * image_scale);
            cv::resize(im,    im,    cv::Size(w, h));
            cv::resize(depth, depth, cv::Size(w, h));
        }

        SLAM.TrackRGBD(im, depth, timestamp, vImuMeas);

        k4a_image_release(depth_to_color);
        k4a_image_release(color_image);
        k4a_image_release(depth_image);
        k4a_capture_release(capture);
    }

    // ── Shutdown ─────────────────────────────────────────────────────────────
    g_continue_session = false;
    if (imu_thread.joinable())
        imu_thread.join();

    SLAM.Shutdown();

    const string traj = file_name.empty() ? "KeyFrameTrajectory.txt" : file_name;
    SLAM.SaveKeyFrameTrajectoryTUM(traj);

    k4a_device_stop_imu(device);
    k4a_device_stop_cameras(device);
    k4a_transformation_destroy(transformation);
    k4a_device_close(device);

    return 0;
}
```

</details>

### A.2 Build

After placing the file and editing `CMakeLists.txt` (see §4.3):

```bash
cd ~/ORB_SLAM3/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc) rgbd_inertial_azure_kinect_dk
```

Or do a full rebuild:

```bash
cd ~/ORB_SLAM3
./build.sh
```

### A.3 Run

```bash
cd ~/ORB_SLAM3
export QT_QPA_PLATFORM=xcb   # required — forces Pangolin to use XCB instead of Wayland

./Examples/RGB-D-Inertial/rgbd_inertial_azure_kinect_dk \
  Vocabulary/ORBvoc.txt \
  Examples/RGB-D-Inertial/Kinnect_DK.yaml \
  KeyFrameTrajectory.txt
```

> **What to expect on startup:**
> 1. The vocabulary takes **5–15 seconds** to load — the terminal looks frozen, this is normal.
> 2. A Pangolin window opens with a black frame view and a 3D map pane.
> 3. Move the camera **slowly** in a well-lit environment. Green dots appear as keypoints are extracted. Once enough keypoints are found, the map initialises and poses begin tracking.
> 4. Press `Ctrl+C` to stop cleanly. The trajectory is saved to `KeyFrameTrajectory.txt`.

---

## Path B — ROS 2 Jazzy

> ⚠️ **Current limitation:** The ROS 2 wrapper (`orbslam3` package) only provides a pure `rgbd` node. The `rgbd_inertial` node (RGB-D + IMU fusion) has not been ported to this wrapper yet. Use **Path A** for full inertial capability.

### B.1 Create the ROS 2 Workspace

```bash
mkdir -p ~/kinect_ws/src
cd ~/kinect_ws/src
```

### B.2 Fix the Python / ament_package Environment

Ubuntu 24.04 (PEP 668) strips `PYTHONPATH` in CMake sub-processes. Create a permanent bridge:

```bash
mkdir -p ~/.local/lib/python3.12/site-packages
echo "/opt/ros/jazzy/lib/python3.12/site-packages" \
  > ~/.local/lib/python3.12/site-packages/ros_jazzy.pth
```

Verify in a **fresh terminal** (no sourcing needed):

```bash
python3 -c "import ament_package; print('OK')"
```

Add these lines to your `~/.zshrc` so they are always active:

```bash
source /opt/ros/jazzy/setup.zsh
export PYTHONPATH=$PYTHONPATH:/opt/ros/jazzy/lib/python3.12/site-packages
export ORB_SLAM3_ROOT_DIR=$HOME/ORB_SLAM3
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/ORB_SLAM3/lib
```

### B.3 Install the Azure Kinect ROS 2 Driver

```bash
cd ~/kinect_ws/src
git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git -b humble
```

Apply the two required fixes:

```bash
# Fix 1 — cv_bridge .h → .hpp (Jazzy removed legacy headers)
cd ~/kinect_ws/src/Azure_Kinect_ROS_Driver
find . -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) \
  -exec sed -i 's|cv_bridge/cv_bridge\.h|cv_bridge/cv_bridge.hpp|g' {} +
```

```bash
# Fix 2 — install path (Jazzy expects lib/<pkg>/, not bin/)
# In CMakeLists.txt, find:  install(TARGETS node  RUNTIME DESTINATION bin)
# Change to:
#   install(TARGETS node  DESTINATION lib/${PROJECT_NAME})
nano ~/kinect_ws/src/Azure_Kinect_ROS_Driver/CMakeLists.txt
```

Build:

```bash
cd ~/kinect_ws
source /opt/ros/jazzy/setup.zsh
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy || true
colcon build --symlink-install --packages-select azure_kinect_ros_driver
source install/setup.zsh
```

> **The `Cannot locate rosdep definition for [K4A]` error is safe to ignore** — CMake finds the SDK because you installed it manually in §2.2.

Test the driver:

```bash
ros2 launch azure_kinect_ros_driver driver.launch.py \
  color_resolution:=1536P \
  depth_mode:=WFOV_2X2BINNED \
  fps:=30
```

In a second terminal, confirm topics:

```bash
ros2 topic list
# Must include:
#   /rgb/image_raw               ← colour stream
#   /depth_to_rgb/image_raw      ← depth aligned to colour (critical)
#   /imu                         ← inertial data
```

### B.4 Set Up the ORB-SLAM3 ROS 2 Wrapper

```bash
cd ~/kinect_ws/src
git clone https://github.com/zang09/ORB_SLAM3_ROS2.git orbslam3_ros2
```

<details>
<summary>🔧 Fix 1 — Replace FindORB_SLAM3.cmake</summary>

Replace `~/kinect_ws/src/orbslam3_ros2/CMakeModules/FindORB_SLAM3.cmake` entirely:

```cmake
# FindORB_SLAM3.cmake — patched for local builds on Ubuntu 24.04
# Sets: ORB_SLAM3_FOUND, ORB_SLAM3_LIBRARIES, ORB_SLAM3_INCLUDE_DIRS

if(NOT ORB_SLAM3_ROOT_DIR)
    set(ORB_SLAM3_ROOT_DIR "$ENV{HOME}/ORB_SLAM3")
endif()

find_path(ORB_SLAM3_INCLUDE_DIR
    NAMES System.h
    PATHS ${ORB_SLAM3_ROOT_DIR}/include
    NO_DEFAULT_PATH
)

find_library(ORB_SLAM3_LIBRARY
    NAMES ORB_SLAM3
    PATHS ${ORB_SLAM3_ROOT_DIR}/lib
    NO_DEFAULT_PATH
)

find_path(DBoW2_INCLUDE_DIR
    NAMES DBoW2/BowVector.h
    PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2
    NO_DEFAULT_PATH
)

find_library(DBoW2_LIBRARY
    NAMES DBoW2
    PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib
    NO_DEFAULT_PATH
)

find_library(g2o_LIBRARY
    NAMES g2o
    PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib
    NO_DEFAULT_PATH
)

set(g2o_INCLUDE_DIR "${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ORB_SLAM3 DEFAULT_MSG
    ORB_SLAM3_LIBRARY
    ORB_SLAM3_INCLUDE_DIR
    DBoW2_INCLUDE_DIR
    DBoW2_LIBRARY
    g2o_LIBRARY
)

if(ORB_SLAM3_FOUND)
    set(ORB_SLAM3_LIBRARIES
        ${ORB_SLAM3_LIBRARY}
        ${DBoW2_LIBRARY}
        ${g2o_LIBRARY}
    )
    # IMPORTANT: include ORB_SLAM3_ROOT_DIR so that relative includes like
    # "Thirdparty/DBoW2/DBoW2/BowVector.h" in KeyFrame.h resolve correctly.
    set(ORB_SLAM3_INCLUDE_DIRS
        ${ORB_SLAM3_INCLUDE_DIR}
        ${ORB_SLAM3_ROOT_DIR}
        ${DBoW2_INCLUDE_DIR}
        ${g2o_INCLUDE_DIR}
    )
    message(STATUS "ORB_SLAM3 found at: ${ORB_SLAM3_ROOT_DIR}")
endif()

mark_as_advanced(ORB_SLAM3_INCLUDE_DIR ORB_SLAM3_LIBRARY
                 DBoW2_INCLUDE_DIR DBoW2_LIBRARY g2o_LIBRARY)
```

</details>

<details>
<summary>🔧 Fix 2 — cv_bridge header + OpenCV 4 constants</summary>

```bash
cd ~/kinect_ws/src/orbslam3_ros2

# cv_bridge .h → .hpp
find . -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) \
  -exec sed -i 's|cv_bridge/cv_bridge\.h|cv_bridge/cv_bridge.hpp|g' {} +

# Fix any .hpppppp typos from previous sed runs
find . -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) \
  -exec sed -i 's|cv_bridge/cv_bridge\.hpp*|cv_bridge/cv_bridge.hpp|g' {} +

# OpenCV 4 constant names
find . -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) \
  -exec sed -i 's/CV_LOAD_IMAGE_UNCHANGED/cv::IMREAD_UNCHANGED/g' {} +
find . -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) \
  -exec sed -i 's/CV_BGR2GRAY/cv::COLOR_BGR2GRAY/g' {} +
```

</details>

<details>
<summary>🔧 Fix 3 — Update CMakeLists.txt (PYTHONPATH + C++17)</summary>

Open `~/kinect_ws/src/orbslam3_ros2/CMakeLists.txt` and make these two changes:

```cmake
# Change this line (references old foxy path):
set(ENV{PYTHONPATH} "/opt/ros/foxy/lib/python3.8/site-packages/")

# To this (jazzy + python 3.12):
set(ENV{PYTHONPATH} "/opt/ros/jazzy/lib/python3.12/site-packages/")
```

```cmake
# Ensure C++17 is set (replace any existing CXX_STANDARD line):
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

</details>

Build the wrapper:

```bash
cd ~/kinect_ws
rm -rf build/orbslam3 install/orbslam3

export ORB_SLAM3_ROOT_DIR=$HOME/ORB_SLAM3

colcon build --symlink-install --packages-select orbslam3 \
  --parallel-workers 2 \
  --event-handlers console_direct+

source install/setup.zsh
```

<details>
<summary>📄 orbslam3_ros2/CMakeLists.txt — Full Reference (your working version)</summary>

```cmake
cmake_minimum_required(VERSION 3.5)
project(orbslam3)

# Updated for ROS 2 Jazzy / Ubuntu 24.04
set(ENV{PYTHONPATH} "/opt/ros/jazzy/lib/python3.12/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ORB_SLAM3 REQUIRED)

include_directories(
  include
  ${ORB_SLAM3_INCLUDE_DIRS}
  ${ORB_SLAM3_ROOT_DIR}
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
)

link_directories(include)

set(ORB_SLAM3_LIBS
  ${ORB_SLAM3_LIBRARY}
  ${DBoW2_LIBRARY}
  ${g2o_LIBRARY}
)

add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)
target_link_libraries(mono ${ORB_SLAM3_LIBS} ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES})

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)
target_link_libraries(rgbd ${ORB_SLAM3_LIBS} ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES})

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp)
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)
target_link_libraries(stereo ${ORB_SLAM3_LIBS} ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES})

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)
target_link_libraries(stereo-inertial ${ORB_SLAM3_LIBS} ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES})

install(TARGETS mono rgbd stereo stereo-inertial
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

</details>

### B.5 Launch Everything

**Terminal 1 — Kinect driver:**

```bash
cd ~/kinect_ws
source install/setup.zsh
ros2 launch azure_kinect_ros_driver driver.launch.py \
  color_resolution:=1536P \
  depth_mode:=WFOV_2X2BINNED \
  fps:=30
```

**Terminal 2 — ORB-SLAM3 (RGB-D only, no IMU fusion):**

```bash
source /opt/ros/jazzy/setup.zsh
source ~/kinect_ws/install/setup.zsh
export QT_QPA_PLATFORM=xcb

ros2 run orbslam3 rgbd \
  ~/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  ~/ORB_SLAM3/Examples/RGB-D-Inertial/Kinnect_DK.yaml \
  --ros-args \
  -r /camera/rgb/image_raw:=/rgb/image_raw \
  -r /camera/depth_registered/image_raw:=/depth_to_rgb/image_raw
```

**If you want IMU topics flowing (for future rgbd_inertial support), add:**

```bash
  -r /imu:=/imu
```

---

## Troubleshooting Reference

| Symptom | Cause | Fix |
|---|---|---|
| `libsoundio1` not installable | Dropped from Ubuntu 24.04 | Manually install from Ubuntu 18.04 archive (see §2.3) |
| `k4aviewer` needs `sudo` | udev rules not active or missing `plugdev` group | Add `TAG+="uaccess"` to rules; `usermod -aG plugdev $USER`; re-login |
| `Invalid RGB Camera Resolution: 1536P` | Case sensitivity in driver arg | Use `1536P` (uppercase P) |
| Pangolin window doesn't open | Wayland/XCB conflict | `export QT_QPA_PLATFORM=xcb` before running |
| `Camera.fps parameter must be integer, aborting` | Float value in YAML | Change `Camera.fps: 30.0` to `Camera.fps: 30` |
| ORB-SLAM3 `build.sh` crashes laptop | OOM / thermal throttling | Edit `build.sh` to use `make -j2` |
| `fatal error: cv_bridge/cv_bridge.h: No such file` | Jazzy removed `.h` legacy headers | `sed -i 's|cv_bridge.h|cv_bridge.hpp|'` on all affected files |
| `libexec directory does not exist` | Node installed to `bin/` not `lib/<pkg>/` | Change `RUNTIME DESTINATION bin` → `DESTINATION lib/${PROJECT_NAME}` in CMakeLists |
| `ModuleNotFoundError: No module named 'ament_package'` | CMake sub-process loses PYTHONPATH | Create `ros_jazzy.pth` bridge file (see §B.2) |
| `Could NOT find ORB_SLAM3` | Wrapper search paths are wrong | Replace `FindORB_SLAM3.cmake` (see §B.4 Fix 1) |
| `fatal error: Thirdparty/DBoW2/DBoW2/BowVector.h` | `ORB_SLAM3_ROOT_DIR` missing from include path | Add `${ORB_SLAM3_ROOT_DIR}` to `ORB_SLAM3_INCLUDE_DIRS` in FindORB_SLAM3.cmake |
| Blank viewer / "Waiting for images" | Topic remap mismatch | Use `/rgb/image_raw` + `/depth_to_rgb/image_raw` |
| Point cloud drifts/jitters | Depth not aligned to colour | Always use `/depth_to_rgb/image_raw`, never raw `/depth/image_raw` |
| IMU never initialises | Wrong noise params or timestamp drift | Use values in §5.3; confirm `IMU.Frequency: 200.0` |
| Firmware version warnings | Outdated camera firmware | Update with `AzureKinectFirmwareTool -u AzureKinectDK_Fw_1.6.110080014.bin` |
| `cv_bridge/cv_bridge.hpppppp` error | `sed` ran on already-patched files | Fix with `sed -i 's|cv_bridge\.hpp*|cv_bridge.hpp|g'` |

---

## Quick-Start Checklists

<details>
<summary>✅ Path A — Native C++ (No ROS)</summary>

- [ ] Core build tools + Sophus installed
- [ ] `libk4a1.4` + `libk4a1.4-dev` installed from Microsoft 18.04 repo
- [ ] `libsoundio1` amd64 installed manually; `k4a-tools` installed
- [ ] udev rules created with `TAG+="uaccess"`; user in `plugdev` group; re-logged in
- [ ] Pangolin built from source
- [ ] ORB-SLAM3 cloned; `sed` C++14 fix applied
- [ ] `rgbd_inertial_azure_kinect_dk.cc` placed in `Examples/RGB-D-Inertial/`
- [ ] ORB-SLAM3 `CMakeLists.txt` updated with `k4a` target
- [ ] `build.sh` completed successfully
- [ ] `ORBvoc.txt` extracted in `Vocabulary/`
- [ ] pyk4a venv created at `~/ORB_SLAM3/Examples/kinnect/`
- [ ] `parameters.py` run; values extracted
- [ ] `Kinnect_DK.yaml` created with your device's intrinsics; `Camera.fps: 30` (integer)
- [ ] `export QT_QPA_PLATFORM=xcb` set before running

</details>

<details>
<summary>✅ Path B — ROS 2 Jazzy</summary>

- [ ] Everything from Path A (except the non-ROS run step)
- [ ] Sophus installed from source
- [ ] Python `.pth` bridge created (`ros_jazzy.pth`)
- [ ] `~/.zshrc` updated with `source`, `PYTHONPATH`, `ORB_SLAM3_ROOT_DIR`, `LD_LIBRARY_PATH`
- [ ] `Azure_Kinect_ROS_Driver` cloned (humble branch)
- [ ] cv_bridge `.hpp` fix applied to driver
- [ ] Install path fix applied to driver's CMakeLists.txt
- [ ] Driver built and tested — `/rgb/image_raw` and `/depth_to_rgb/image_raw` confirmed
- [ ] `orbslam3_ros2` cloned into `~/kinect_ws/src/` as `orbslam3_ros2`
- [ ] `FindORB_SLAM3.cmake` replaced with patched version
- [ ] cv_bridge `.hpp` fix applied to wrapper source files
- [ ] PYTHONPATH line in CMakeLists.txt updated from `foxy` to `jazzy`
- [ ] C++17 confirmed in wrapper's CMakeLists.txt
- [ ] OpenCV 4 constants updated
- [ ] Wrapper built with `--parallel-workers 2`
- [ ] Driver launched with `color_resolution:=1536P` (uppercase P)
- [ ] `ros2 run orbslam3 rgbd` launches with correct topic remaps
- [ ] `export QT_QPA_PLATFORM=xcb` set before running

</details>

---

*Guide compiled from live debugging sessions on Ubuntu 24.04 Noble Numbat with HP Spectre x360, April 2026.*