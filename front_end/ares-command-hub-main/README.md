# A.R.E.S. Command Hub

Dashboard web para supervisar el rover A.R.E.S. con datos reales de ROS2. El frontend corre con Vite en el puerto `8080` y el backend FastAPI actua como bridge ROS2 en el puerto `8000`.

No usa `.venv`: el backend debe correr con el Python del sistema y el entorno de ROS2 Jazzy.

## Requisitos

- Ubuntu con ROS 2 Jazzy en `/opt/ros/jazzy`.
- Workspace compilado en `/home/snorlix/colcon_ws/install` si usas mensajes o paquetes del workspace.
- Node.js `^20.19.0` o `>=22.12.0`.
- npm.

## Paquetes apt

Instala las dependencias del backend con apt/ROS:

```bash
sudo apt update
sudo apt install -y \
  python3-fastapi \
  python3-uvicorn \
  python3-pydantic \
  python3-numpy \
  python3-opencv \
  ros-jazzy-cv-bridge \
  ros-jazzy-sensor-msgs-py
```

Si necesitas instalar Node.js 22:

```bash
sudo apt install -y ca-certificates curl gnupg
curl -fsSL https://deb.nodesource.com/setup_22.x | sudo -E bash -
sudo apt install -y nodejs
```

## Instalar frontend

```bash
cd /home/snorlix/colcon_ws/src/autonomous_rover/front_end/ares-command-hub-main
npm install
```

## Correr backend

En una terminal:

```bash
cd /home/snorlix/colcon_ws/src/autonomous_rover/front_end/ares-command-hub-main/backend
PYTHONNOUSERSITE=1 bash -lc 'source /opt/ros/jazzy/setup.bash && source /home/snorlix/colcon_ws/install/setup.bash && python3 -m uvicorn main:app --host 127.0.0.1 --port 8000'
```

`PYTHONNOUSERSITE=1` evita que Python cargue paquetes de `~/.local` que pueden romper `cv_bridge`/NumPy.

## Correr frontend

En otra terminal:

```bash
cd /home/snorlix/colcon_ws/src/autonomous_rover/front_end/ares-command-hub-main
npm run dev
```

Abre:

```text
http://127.0.0.1:8080
```

## Endpoints principales

- `GET /health`
- `GET /api/telemetry`
- `GET /api/rtabmap/status`
- `GET /ros/topics`
- `GET /ros/nodes`
- `GET /ros/topic/{topic_path}`
- `POST /ros/subscribe`
- `DELETE /ros/subscribe`
- `POST /api/navigation/goal`
- `POST /api/navigation/initial_pose`

## WebSockets

- `/ws/telemetry`
- `/ws/camera`
- `/ws/depth`
- `/ws/map`
- `/ws/path`
- `/ws/pointcloud`
- `/ws/rtabmap/status`
- `/ws/topic/{topic_path}`

Aliases RTAB-Map:

- `/ws/rtabmap/rgb`
- `/ws/rtabmap/depth`
- `/ws/rtabmap/odom`
- `/ws/rtabmap/map`
- `/ws/rtabmap/cloud`
- `/ws/rtabmap/imu`

## Topics default

- RGB preview: `/k4a/rgb/image_raw/compressed`
- Depth: `/k4a/depth_to_rgb/image_raw`
- Camera info: `/k4a/rgb/camera_info`
- Odom: `/rtabmap/odom`
- IMU: `/k4a/imu_filtered`
- Map: `/rtabmap/grid_prob_map`
- Path: `/sdv_trajectory`
- Point cloud: `/k4a/points2`
- Markers: `/all_available_paths`
- Scan: `/scan`
- Goal publish: `/goal_pose`, opcional `/goal`
- Initial pose publish: `/initialpose`

## Pruebas rapidas

```bash
cd /home/snorlix/colcon_ws/src/autonomous_rover/front_end/ares-command-hub-main
PYTHONNOUSERSITE=1 bash -lc 'source /opt/ros/jazzy/setup.bash && source /home/snorlix/colcon_ws/install/setup.bash && python3 -m py_compile backend/main.py'
node node_modules/typescript/bin/tsc --noEmit
npm run build
```
