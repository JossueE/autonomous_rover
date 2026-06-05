# person_tracker

A ROS 2 package for autonomous person following on a rover. It combines **YOLOv8** real-time detection with **OSNet Re-ID** to lock onto a specific individual and follow them reliably — even when other people cross the field of view. Velocity commands are published directly as `Twist` messages.

---

## How it works

### 1. Detection — YOLOv8
The node subscribes to the Azure Kinect RGB stream (`/k4a/rgb/image_raw`). Each frame is resized to 640×360 and passed to a YOLOv8 model. Only bounding boxes of the configured `target_class` (default: `"person"`) are kept.

### 2. Re-Identification — OSNet
Every detected person crop is fed to an **OSNet** feature extractor (`osnet_x0_25` by default). OSNet produces a 512-dimensional embedding that captures appearance features (clothing color, shape, texture).

- **Lock-on**: When the first person is detected, their embedding is saved as the *target embedding*.
- **Tracking**: In subsequent frames, the detection whose embedding has the highest cosine similarity to the target (above `reid_similarity_threshold`) is selected as the target — regardless of position or bounding box overlap.
- **EMA update**: The target embedding is softly updated each frame (`reid_ema_alpha = 0.9`) to handle gradual appearance changes (lighting, rotation).

### 3. Distance — Depth map
The node reads the aligned depth map (`/k4a/depth_to_rgb/image_raw`, 32FC1, in meters). It extracts the median depth of all valid pixels inside the target's bounding box to estimate the real distance.

### 4. Reactive control — Center Stage style
The controller outputs a `Twist` message to `/cmd_vel_safe` every frame:

| Error | Command |
|---|---|
| Person to the right of center | Rotate right |
| Person to the left of center | Rotate left |
| Person farther than 1.1 m | Move forward |
| Person closer than 1.1 m | Move backward |
| Person well-centered (< 5% from center) | No rotation |
| Large centering error (> 20% from center) | Suppress linear motion — turn first |

The angular and linear controls are independent proportional (P) controllers, clamped to configurable max speeds.

### 5. Recovery state machine
When the target disappears, the node enters a recovery sequence:

```
SEARCHING ──(first person detected)──────────────────────► TRACKING
TRACKING  ──(target lost)────────────────────────────────► LOST_WAITING
LOST_WAITING  ──(target reappears, < 5 s)────────────────► TRACKING
LOST_WAITING  ──(5 s elapsed, still missing)─────────────► LOST_REVERSING
LOST_REVERSING ──(target reappears)──────────────────────► TRACKING
LOST_REVERSING ──(3 s elapsed)───────────────────────────► LOST_TURNING_LEFT
LOST_TURNING_LEFT ──(target reappears)───────────────────► TRACKING
LOST_TURNING_LEFT ──(5 s elapsed)────────────────────────► LOST_TURNING_RIGHT
LOST_TURNING_RIGHT ──(target reappears)──────────────────► TRACKING
LOST_TURNING_RIGHT ──(5 s elapsed)───────────────────────► LOST_STOPPED
LOST_STOPPED ──(target reappears)────────────────────────► TRACKING
LOST_STOPPED ──(stays here until external reset)
```

All timings are configurable in the YAML file.

---

## Topics

### Subscribed

| Topic | Type | Description |
|---|---|---|
| `/k4a/rgb/image_raw` | `sensor_msgs/Image` | RGB stream from Azure Kinect |
| `/k4a/depth_to_rgb/image_raw` | `sensor_msgs/Image` | Aligned depth map (32FC1, meters) |
| `/k4a/rgb/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |
| `/k4a/imu` | `sensor_msgs/Imu` | IMU del Kinect — integra `angular_velocity.z` durante el pivote |

### Published

| Topic | Type | Description |
|---|---|---|
| `/cmd_vel_safe` | `geometry_msgs/Twist` | Velocity commands to the rover motor driver |
| `/person_tracker/person_detected` | `std_msgs/Bool` | `true` when the target is visible |
| `/person_tracker/person_bbox` | `std_msgs/Float32MultiArray` | `[x1, y1, x2, y2, conf, cx_norm, cy_norm]` |
| `/person_tracker/detections_image` | `sensor_msgs/Image` | Annotated debug image |

---

## Installation

### 1. Python dependencies

```bash
cd ~/rover_ws/src/person_tracker/person_tracker
pip install ultralytics opencv-python numpy

# torchreid prerequisites (Cython must be installed before torchreid):
pip install cython scipy Pillow h5py matplotlib

# torchreid itself (PyTorch already provided by ultralytics):
pip install git+https://github.com/KaiyangZhou/deep-person-reid.git
```

> On first run, OSNet will automatically download pretrained Market-1501 weights (~5 MB) from the torchreid model zoo. An internet connection is required the first time.

### 2. Build

```bash
cd ~/rover_ws
colcon build --packages-select azure_kinect_ros2_driver person_tracker --symlink-install
source install/setup.zsh   # o setup.bash según tu shell
```

> **`--symlink-install` es importante.** Sin esta bandera, los archivos `.py` y `.yaml` se *copian* a `install/` y cualquier edición posterior no se refleja hasta que recompiles. Con el symlink, editar el YAML o el código se ve al instante (basta con relanzar el nodo).

---

## Running

Setup mínimo: **ruedas + Kinect conectadas por USB**. Nada más se necesita (la IMU usada por el PIVOTING ya viene dentro del propio Kinect).

### 🚀 Lanzamiento todo-en-uno (recomendado)

Hay un launcher que orquesta los 3 componentes (driver de ruedas + Kinect/detector + ventana de visualización) en una sola terminal:

```bash
ros2 launch person_tracker bringup.launch.py                        # outdoor (default)
ros2 launch person_tracker bringup.launch.py mode:=indoor           # interiores
ros2 launch person_tracker bringup.launch.py motors_port:=/dev/WHEELS mode:=indoor
ros2 launch person_tracker bringup.launch.py mode:=indoor use_image_view:=false
```

Argumentos disponibles:

| Argumento | Default | Descripción |
|---|---|---|
| `mode` | `outdoor` | `outdoor` o `indoor` (selecciona el YAML de parámetros del detector) |
| `motors_port` | `/dev/ttyUSB0` | Puerto serial del driver ZLAC8015D (alternativas comunes: `/dev/WHEELS`) |
| `use_image_view` | `true` | Si lanza la ventana con las detecciones anotadas |

Si todo está bien, deberías ver en log:
```
[person_tracker] Modo: INDOOR → /home/.../person_tracker_params_indoor.yaml
```

---

### Lanzamiento manual en 3 terminales (debug)

Si prefieres lanzarlos por separado (útil para depurar cada pieza), abajo van los comandos uno por uno.

En cada terminal nueva, primero:

```bash
source ~/rover_ws/install/setup.zsh
export QT_QPA_PLATFORM=xcb   # solo si estás en Wayland (necesario para image_view)
```

### Terminal 1 — Driver de las ruedas (vía `hardware_bringup`)

```bash
ros2 launch rover_bringup hardware_bringup.launch.py motors_port:=/dev/WHEELS
```

Si no tienes el symlink `/dev/WHEELS` (motores directo a otra laptop), usa el puerto crudo:

```bash
ls /dev/ttyUSB*                            # ver qué puertos hay
ros2 launch rover_bringup hardware_bringup.launch.py motors_port:=/dev/ttyUSB0
```

> **Por qué usar el launch y no `ros2 run` directo:** el launch ya configura `unlock_driver:=true`, `accel_time_ms`, `decel_time_ms` y `wheels_separation:0.35` (crítico — sin este valor correcto, los giros salen mal). Además incluye `robot_state_publisher` para los TFs.
>
> El launch tiene las secciones de IMU/EKF/odometría **comentadas**, así que hoy en día solo lanza ruedas + state publisher — exactamente lo que necesitas.

### Terminal 2 — Kinect + detector

Hay dos **modos de navegación** preconfigurados, cada uno con su propio YAML de parámetros:

| Modo | Para | YAML | Velocidad lineal tope |
|---|---|---|---|
| `outdoor` (default) | Exteriores, espacios amplios | `person_tracker_params_outdoor.yaml` | 1.75 m/s |
| `indoor` | Laboratorio, interiores | `person_tracker_params_indoor.yaml` | 0.6 m/s |

```bash
# Modo exteriores (default — rápido, reactivo)
ros2 launch person_tracker run_all.launch.py
# equivalente:
ros2 launch person_tracker run_all.launch.py mode:=outdoor

# Modo interiores (suave, conservador, ideal para lab)
ros2 launch person_tracker run_all.launch.py mode:=indoor
```

Al arrancar verás en el log qué modo cargó:
```
[person_tracker] Modo: INDOOR → /home/.../config/person_tracker_params_indoor.yaml
```

Esto lanza:
1. El driver del Azure Kinect (720P @ 15 fps, depth NFOV_2x2BINNED, IMU @ ~1.6 kHz)
2. El nodo `person_tracker_node` con los parámetros del modo elegido

Verifica que esté corriendo bien:

```bash
ros2 topic hz /k4a/rgb/image_raw           # debe ser ~15 Hz
ros2 topic hz /k4a/imu                     # debe ser ~1600 Hz
ros2 topic echo /person_tracker/person_detected   # true cuando vea una persona
```

### Terminal 3 — Visualización (opcional)

```bash
ros2 run image_view image_view --ros-args -r image:=/person_tracker/detections_image
```

La imagen anotada muestra:
- Todas las detecciones de YOLO
- **Bounding box verde grueso** = la persona bloqueada por Re-ID (el objetivo)
- **Punto rojo** = centro del objetivo
- Esquina superior izquierda: estado actual de la FSM principal y distancia al objetivo

Para seguir las transiciones de la sub-FSM (corner+pivot) en tiempo real:

```bash
ros2 run rqt_console rqt_console
# o en una terminal:
ros2 topic echo /rosout | grep -E "State|SubState"
```

### Verificación rápida del eje del IMU

El PIVOTING integra `angular_velocity.z` del IMU del Kinect. Si la cámara está montada en orientación estándar, **Z es yaw** (giros) — ya configurado por defecto. Si los pivotes no funcionan bien, verifica girando físicamente el rover:

```bash
ros2 topic echo /k4a/imu --field angular_velocity
```

Al girar sobre el eje vertical, uno de los tres ejes (`x`, `y` o `z`) debe mostrar valores grandes. Ese es el yaw axis. Si no es `z`, cambia `imu_yaw_axis` en el YAML.

---

## Probar el rover sin el detector

Para verificar que los motores respondan, publica directamente al tópico `/cmd_vel_safe`:

```bash
# Avanzar a 0.2 m/s
ros2 topic pub /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"

# Parar
ros2 topic pub /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

> **Importante:** el `wheels_driver` se apaga automáticamente si detecta más de un publisher en `/cmd_vel_safe`. No corras el detector y un `topic pub` manual al mismo tiempo.

---

## Modos de navegación

El paquete tiene **dos perfiles** preconfigurados con sus propios archivos YAML. Eliges cuál usar con el argumento `mode:=...` al lanzar:

```bash
ros2 launch person_tracker run_all.launch.py mode:=outdoor    # default
ros2 launch person_tracker run_all.launch.py mode:=indoor
```

| Modo | Para | Archivo de parámetros | Vel. lineal tope |
|---|---|---|---|
| `outdoor` (default) | Exteriores, espacios amplios | [`config/person_tracker_params_outdoor.yaml`](config/person_tracker_params_outdoor.yaml) | 1.75 m/s |
| `indoor` | Laboratorio, interiores | [`config/person_tracker_params_indoor.yaml`](config/person_tracker_params_indoor.yaml) | 0.6 m/s |

Al arrancar, el log confirma qué perfil cargó:
```
[person_tracker] Modo: INDOOR → /home/.../config/person_tracker_params_indoor.yaml
```

Verifica en runtime con:
```bash
ros2 param get /person_tracker_node max_linear_speed
# 1.75 si outdoor, 0.6 si indoor
```

### Diferencias clave entre modos

| Parámetro | Outdoor | Indoor | Por qué |
|---|---|---|---|
| `target_distance_m` | 1.1 m | **0.75 m** | Sigue de cerca en lab (espacios chicos) |
| `max_linear_speed` | 1.75 m/s | **0.6 m/s** | Velocidad de paseo en lab |
| `max_reverse_speed` | 0.5 m/s | **0.25 m/s** | Reversa aún más cauta |
| `max_angular_speed` | 1.9 rad/s | **1.6 rad/s** | Giro rápido para alcanzar desplazamientos laterales |
| `kp_linear` | 1.2 | **0.6** | Control lineal menos reactivo (evita oscilación) |
| `kp_angular` | 3.1 | **2.8** | Casi igual: alcanza el tope angular rápido |
| `kd_angular` | 1.5 | **1.6** | Anticipa giros laterales rápidos |
| `cx_velocity_ema_alpha` | 0.6 | **0.5** | Menos suavizado: reacción más rápida en lab |
| `approach_speed` | 1.0 m/s | **0.5 m/s** | Fase APPROACHING más lenta |
| `pivot_max_angular_speed` | 1.9 | **1.6** | Pivote rápido al perder a la persona |
| `pivot_ramp_duration_s` | 0.3 s | **0.5 s** | Arranque del giro más gradual |
| `accel_base` / `accel_peak` | 0.3 / 0.7 | **0.3 / 0.6** | Aceleración casi igual (lab necesita reaccionar) |
| `decel_base` / `decel_peak` | 0.4 / 0.9 | **0.35 / 0.7** | Frenado un poco más suave que outdoor |
| `ramp_urgency_scale` | 2.0 m | **1.0 m** | Alcanza el pico con menos error de distancia |
| `emergency_distance` | 0.7 m | **0.5 m** | Disparo más cerca (target indoor = 0.75 m, no hay tanto colchón) |
| `emergency_decel` | 5.0 m/s² | **3.0 m/s²** | Frenado de emergencia adecuado a las velocidades indoor |
| `depth_max_valid_m` | 3.5 m | **3.0 m** | Rango de profundidad más corto |
| `reverse_speed` (recovery) | 0.15 | **0.10** | Retroceso de búsqueda muy lento |
| `search_turn_speed` (recovery) | 0.6 | **0.4** | Giros de búsqueda calmados |

### Qué se mantiene igual

- Modelo YOLO y umbral de confianza
- OSNet Re-ID (mismo modelo, mismo umbral de similitud, misma EMA)
- Tópicos de entrada (RGB, depth, info, IMU)
- Estructura de la FSM (`corner_threshold`, `recover_zone`, integración IMU)
- Zonas muertas de centrado

### Agregar más modos

Para añadir un modo nuevo (ej. `crowded`, `narrow_corridor`):

1. Copia `person_tracker_params_outdoor.yaml` → `person_tracker_params_<nombre>.yaml` y ajusta valores
2. Agrega el nombre a `VALID_MODES` en [`launch/person_tracker.launch.py`](launch/person_tracker.launch.py)
3. Recompila: `colcon build --packages-select person_tracker --symlink-install`
4. Lanza con `mode:=<nombre>`

---

## Parameters reference

Todos los parámetros se encuentran en `config/person_tracker_params_<modo>.yaml`. El nodo arranca con `parameters=[config_file]`, así que **el YAML es la fuente de verdad** — los defaults en código son solo fallback. Los valores mostrados abajo corresponden al modo **outdoor**; consulta la sección anterior para los valores indoor.

### Detección y Re-ID

| Parámetro | Default | Descripción |
|---|---|---|
| `model_path` | `"yolov8n.pt"` | Pesos YOLO (nano por velocidad) |
| `confidence_threshold` | `0.5` | Confianza mínima de YOLO |
| `device` | `"0"` | GPU CUDA `"0"` o `"cpu"` |
| `target_class` | `"person"` | Clase COCO a detectar |
| `publish_debug_image` | `true` | Publicar imagen anotada |
| `reid_model` | `"osnet_x0_25"` | Variante OSNet (`osnet_x0_25`/`osnet_x0_5`/`osnet_x1_0`) |
| `reid_similarity_threshold` | `0.7` | Similitud coseno mínima para aceptar match |
| `reid_ema_alpha` | `0.9` | Peso EMA del embedding objetivo (0=sin update, 1=nunca) |

### Tópicos de entrada

| Parámetro | Default | Descripción |
|---|---|---|
| `rgb_topic` | `/k4a/rgb/image_raw` | Stream RGB |
| `depth_topic` | `/k4a/depth_to_rgb/image_raw` | Profundidad alineada (32FC1, metros) |
| `camera_info_topic` | `/k4a/rgb/camera_info` | Intrínsecos de cámara |
| `imu_topic` | `/k4a/imu` | IMU del Kinect (usado durante PIVOTING) |
| `imu_yaw_axis` | `"z"` | Eje de yaw del IMU (`x`/`y`/`z`) |

### Control de seguimiento (sub-estado NORMAL)

| Parámetro | Default | Descripción |
|---|---|---|
| `target_distance_m` | `1.1` | Distancia de seguimiento (m) |
| `max_linear_speed` | `2.0` | Tope lineal (m/s) — las rampas evitan que sea brusco |
| `max_angular_speed` | `1.0` | Tope angular (rad/s) |
| `kp_linear` | `1.5` | Ganancia P de distancia |
| `kp_angular` | `2.5` | Ganancia P de centrado |
| `kd_angular` | `1.5` | Ganancia D angular (anticipa giros laterales) |
| `cx_velocity_ema_alpha` | `0.6` | Suavizado EMA de la velocidad lateral de la persona |
| `centering_deadzone` | `0.05` | Zona muerta de giro |
| `centering_suppress_linear_zone` | `0.45` | (legacy, sin uso con la FSM nueva) |

### Sub-FSM Corner + Pivot

| Parámetro | Default | Descripción |
|---|---|---|
| `corner_threshold` | `0.45` | `|error_x| >` este valor → entra a APPROACHING |
| `recover_zone` | `0.20` | `|error_x| <` este valor → vuelve a NORMAL (hysteresis) |
| `approach_speed` | `1.0` | m/s al avanzar hacia la "esquina" |
| `approach_max_duration_s` | `3.0` | Timeout: pasa a PIVOTING si no llegó |
| `pivot_max_angular_speed` | `1.0` | rad/s tope durante el pivote |
| `pivot_ramp_duration_s` | `0.3` | Ramp-up suave del pivote (0 → tope) |
| `pivot_max_angle_rad` | `2.5` | ~143° — si pivota tanto sin hallarla, se da por perdido |

### Rampas reactivas de velocidad lineal

| Parámetro | Default | Descripción |
|---|---|---|
| `accel_base` | `0.8` | m/s² aceleración base (situación tranquila) |
| `accel_peak` | `1.8` | m/s² pico (persona lejos → alcanzar rápido) |
| `decel_base` | `1.0` | m/s² frenado base |
| `decel_peak` | `2.0` | m/s² frenado firme (persona muy cerca) |
| `ramp_urgency_scale` | `2.0` | m de error de distancia para alcanzar el pico |

### Buffer de distancia centrada

| Parámetro | Default | Descripción |
|---|---|---|
| `center_dist_buffer_seconds` | `2.0` | Vigencia de muestras (s) |
| `center_dist_min_samples` | `3` | Muestras mínimas para confiar en el buffer |
| `depth_min_valid_m` / `depth_max_valid_m` | `0.5` / `3.5` | Rango válido de lecturas de profundidad |

### Recuperación FSM principal (objetivo perdido)

| Parámetro | Default | Descripción |
|---|---|---|
| `wait_timeout_s` | `5.0` | Espera antes de iniciar búsqueda |
| `reverse_duration_s` | `3.0` | Tiempo de retroceso |
| `search_turn_duration_s` | `5.0` | Tiempo de giro por lado durante búsqueda |
| `reverse_speed` | `0.15` | m/s de retroceso |
| `search_turn_speed` | `0.6` | rad/s durante búsqueda |

---

## Troubleshooting

**OSNet weights fail to download**
Run with internet access the first time, or manually download and point to the weights file path using the `torchreid` API.

**"No depth image received" warning**
Make sure `/k4a/depth_to_rgb/image_raw` is being published. Use `ros2 topic hz /k4a/depth_to_rgb/image_raw` to verify. Distance control is silently disabled until depth is available.

**Target keeps switching between people**
Lower `reid_similarity_threshold` (e.g. `0.6`) or use a heavier OSNet variant (`osnet_x0_5` or `osnet_x1_0`) for better discrimination.

**Rover oscillates / overshoots**
Reduce `kp_linear` and/or `kp_angular`. Increase `centering_deadzone` and the distance deadzone (hardcoded at ±0.1 m in the controller).

**CUDA out of memory**
Set `device: "cpu"` in the YAML, or switch to `osnet_x0_25` (smallest model).
