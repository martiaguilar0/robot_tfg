# Robot diferencial con navegación autónoma mediante fusión inercial, odométrica y vision artificial por marcadores 
---

## Índice

- [Descripción general](#descripción-general)
- [Arquitectura del sistema](#arquitectura-del-sistema)
- [Hardware](#hardware)
- [Estructura del repositorio](#estructura-del-repositorio)
- [Requisitos previos](#requisitos-previos)
- [Instalación](#instalación)
  - [1. Firmware (STM32)](#1-firmware-stm32)
  - [2. Workspace ROS2 (Jetson)](#2-workspace-ros2-jetson)
- [Configuración](#configuración)
- [Ejecución del sistema](#ejecución-del-sistema)
- [Grafo ROS2](#grafo-ros2)
- [Resultados](#resultados)
- [Recursos y calibración](#recursos-y-calibración)

---

## Descripción general

Este proyecto implementa un robot móvil diferencial completamente autónomo capaz de seguir waypoints predefinidos manteniendo una localización precisa. El sistema fusiona datos de tres fuentes — **odometría de ruedas**, **giroscopio IMU** y **marcadores de visión ArUco** — a través de un **Filtro de Kalman Extendido (EKF)** ejecutado en una NVIDIA Jetson.

El robot se divide en dos capas computacionales:

| Capa | Hardware | Rol |
|------|----------|-----|
| **Bajo nivel** | STM32F401 | Control PID de motores, lectura de encoders, adquisición IMU, telemetría serie |
| **Alto nivel** | NVIDIA Jetson | Middleware ROS2, EKF, localización por visión, navegación por waypoints |

La comunicación entre capas se realiza por UART a 460800 baudios.

---

## Arquitectura del sistema

El sistema se divide en dos capas que se comunican por UART a 460800 baudios.

**Capa de bajo nivel — STM32F401**

Gestiona el hardware en tiempo real: lee los encoders de cuadratura y la IMU ICM-20948 a ~102 Hz, ejecuta el control PID de motores a 1 kHz y empaqueta la telemetría para enviarla a la Jetson.

**Capa de alto nivel — NVIDIA Jetson (ROS2)**

Recibe la telemetría serie y la distribuye por topics. Tres fuentes de información convergen en el EKF:

- Encoders → odometría → `/odom` → **EKF**
- IMU → `/imu/raw` → **EKF**
- Cámara CSI → detección ArUco → `/vision/pose` → **EKF**

El EKF produce una estimación filtrada del estado `[x, y, θ, v_lin, v_ang]` en `/odometry/filtered`, que el nodo de navegación usa para generar comandos de velocidad (`/cmd_vel`) hacia los waypoints. La cinemática inversa convierte esos comandos en referencias de velocidad para cada motor, cerrando el lazo de control.

---

## Hardware

| Componente | Modelo | Detalles |
|------------|--------|---------|
| Microcontrolador | STM32F401RCTx | 84 MHz, control de motores y sensores |
| Computador embarcado | NVIDIA Jetson | Host ROS2, procesamiento de visión |
| IMU | ICM-20948 | 9-DOF (accel ±16g, giróscopo ±2000°/s, magnetómetro) |
| Encoders | Cuadratura | 1496 ticks/revolución por rueda |
| Cámara | Jetson CSI | 1280×720 @ 30 FPS vía GStreamer |
| Ruedas | — | Diámetro: 80 mm, distancia entre ejes: 223.5 mm |
| Driver de motores | — | Puente H controlado por PWM (TIM1) |
| Enlace serie | UART2 | 460800 baudios, DMA RX/TX |

### Parámetros físicos del robot

| Parámetro | Valor |
|-----------|-------|
| Diámetro de rueda | 0.08 m |
| Distancia entre ejes | 0.2235 m |
| Metros por tick de encoder | 0.000168 m |
| Ticks por revolución | 1496 |

### Marcadores ArUco

| ID Marcador | Posición en el mundo [x, y] (m) | Orientación |
|-------------|--------------------------------|-------------|
| 493 | [1.50, 0.00] | 0 rad |
| 494 | [-0.30, -1.20] | π rad |

Diccionario: `DICT_5X5_1000`, tamaño de marcador: 60 mm.

---

## Estructura del repositorio

```
robot_tfg/
├── firmware/                        # Firmware embebido STM32F401
│   ├── stm_fw_1.0.ioc               # Fichero de proyecto STM32CubeIDE
│   ├── Core/
│   │   ├── Src/
│   │   │   ├── main.c               # Bucle principal, PID, telemetría, init sensores
│   │   │   ├── stm32f4xx_it.c       # Manejadores de interrupciones
│   │   │   └── stm32f4xx_hal_*.c    # Drivers HAL de periféricos
│   │   └── Inc/
│   └── Drivers/                     # STM32 HAL / CMSIS
│
├── ros2_ws/                         # Workspace ROS2 (Colcon)
│   └── src/
│       ├── robot_driver/            # Paquete principal del driver (Python)
│       │   ├── robot_driver/
│       │   │   ├── stm32_bridge.py       # Puente serie ↔ ROS2
│       │   │   ├── inv_kinematics.py     # cmd_vel → velocidades de motor
│       │   │   ├── odometry.py           # Encoders → /odom
│       │   │   ├── ekf.py                # Filtro de Kalman Extendido
│       │   │   ├── vision_localization.py# Detección de marcadores ArUco
│       │   │   └── navigation.py         # Seguidor de waypoints
│       │   ├── launch/
│       │   │   └── robot_full.launch.py  # Lanzamiento completo del sistema
│       │   ├── config/
│       │   │   └── navigation_config.yaml# Waypoints y ganancias de control
│       │   ├── package.xml
│       │   └── setup.py
│       └── robot_msgs/              # Definiciones de mensajes ROS2 personalizados
│           └── msg/
│               └── EncoderTicks.msg # header, enc_l (int32), enc_r (int32)
│
├── resources/                       # Scripts de calibración y análisis de resultados
│   ├── calibration/
│   │   ├── vision/
│   │   │   ├── vision_calibration_results.py   # Calibración de cámara con tablero de ajedrez
│   │   │   ├── accuracy_check/                 # Precisión de detección a 50/100 cm
│   │   │   └── images/                         # 20 imágenes de calibración (000–019.jpg)
│   │   └── gyro/                               # Scripts de calibración de la IMU
│   │       ├── gyro_calibration.ino            # Sketch Arduino para adquisición del bias
│   │       └── gyro_analysis.py                # Análisis estadístico del giróscopo
│   └── results_analysis/
│       ├── ground_truth_tracker.py             # Extrae trayectoria de vídeo cenital
│       ├── plot_trajectories.py                # Gráficas ground truth vs EKF
│       └── gt_correction.py                    # Corrección de errores de homografía
│
└── tests_results/                   # Datos experimentales de los ensayos
    ├── config_a/
    │   ├── data/   gt_config_a.csv, ros_log_config_a.csv
    │   ├── charts/ gt_config_a.png, gt_ekf_config_a.png
    │   └── video/  config_a.mp4
    ├── config_b/
    │   └── ...
    └── config_c/
        └── ...
```

---

## Requisitos previos

### Firmware

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) >= 1.13
- Programador ST-Link o cable USB con soporte DFU
- Objetivo: **STM32F401RCTx** @ 84 MHz (cristal HSE de 25 MHz)

### ROS2 (NVIDIA Jetson)

- Ubuntu 20.04 / 22.04 (compatible con Jetson)
- **ROS2 Humble** (o Foxy)
- Python >= 3.8

**Dependencias Python:**

```bash
pip install opencv-contrib-python numpy scipy
```

**Dependencias de paquetes ROS2:**

```bash
sudo apt install ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-nav-msgs
```

---

## Instalación

### 1. Firmware (STM32)

1. Abrir STM32CubeIDE e importar el proyecto:
   ```
   File → Import → Existing Projects into Workspace → firmware/
   ```

2. Compilar el proyecto (`Ctrl+B` o `Project → Build All`).

3. Flashear en la placa:
   ```
   Run → Debug (ST-Link)  o  Run → Run
   ```
   Alternativamente, usar el fichero `.bin` con STM32CubeProgrammer apuntando al STM32F401.

4. Verificar que la placa transmite telemetría por UART2 a **460800 baudios** en el pin TX físico.

> **Formato del paquete de telemetría**: cabecera `0xABCD` (2 bytes) + payload (ticks de encoders, IMU, magnetómetro, batería, flags) + checksum XOR. Transmitido a ~102 Hz.

---

### 2. Workspace ROS2 (Jetson)

**Clonar el repositorio en la Jetson:**

```bash
git clone <url-del-repo> ~/robot_tfg
cd ~/robot_tfg/ros2_ws
```

**Cargar el entorno ROS2:**

```bash
source /opt/ros/humble/setup.bash
```

**Instalar dependencias Python:**

```bash
pip install opencv-contrib-python numpy scipy
```

**Compilar el workspace:**

```bash
colcon build --symlink-install
source install/setup.bash
```

**Verificar que los nodos están registrados:**

```bash
ros2 pkg list | grep robot
# Salida esperada: robot_driver  robot_msgs
```

---

## Configuración

Todos los parámetros en tiempo de ejecución se encuentran en [ros2_ws/src/robot_driver/config/navigation_config.yaml](ros2_ws/src/robot_driver/config/navigation_config.yaml).

```yaml
navigation:
  linear_speed: 0.15        # m/s — velocidad de avance
  angular_speed: 1.5        # rad/s — velocidad angular máxima
  distance_tolerance: 0.03  # m — umbral de llegada al waypoint
  angle_tolerance: 0.03     # rad — umbral de alineación de rumbo

  waypoints:                # Recorrido cuadrado 1.2 x 1.2 m, repetido 6 vueltas
    - [1.20,  0.00]
    - [1.20, -1.20]
    - [0.00, -1.20]
    - [0.00,  0.00]

  markers:
    493: [1.50,  0.00, 0.0]      # [x, y, rumbo] en el marco del mundo
    494: [-0.30, -1.20, 3.1416]
```

**Parámetros de ruido del EKF** (en [ekf.py](ros2_ws/src/robot_driver/robot_driver/ekf.py)):

| Fuente | Ruido (R) | Notas |
|--------|-----------|-------|
| Giróscopo IMU | 9999 | Peso bajo; solo usado para actualizaciones rápidas |
| Odometría | 0.01 | Fuente principal de velocidad |
| Visión (ArUco) | Dinámico | Derivado de la varianza de la medición; umbral Mahalanobis chi2=9.21 |

**Intrínsecos de la cámara** (precalibrados, en [vision_localization.py](ros2_ws/src/robot_driver/robot_driver/vision_localization.py)):

```
focal_x = 1092.06,  focal_y = 1095.36
principal_x = 707.16,  principal_y = 340.62
Distorsión: k1=-0.1242, k2=0.7327, ...
```

Para recalibrar, usar las imágenes de `resources/calibration/vision/images/` y ejecutar:

```bash
python resources/calibration/vision/vision_calibration_results.py
```

---

## Ejecución del sistema

### Sistema completo (comando único)

En la Jetson, tras compilar el workspace:

```bash
source ~/robot_tfg/ros2_ws/install/setup.bash
ros2 launch robot_driver robot_full.launch.py
```

Esto lanza los seis nodos simultáneamente:

| Nodo | Script | Rol |
|------|--------|-----|
| `stm32_bridge_node` | `stm32_bridge.py` | Interfaz serie <-> ROS2 |
| `inv_kinematics_node` | `inv_kinematics.py` | `cmd_vel` -> ticks de motor |
| `odometry_node` | `odometry.py` | Integración de encoders |
| `ekf_node` | `ekf.py` | Fusión sensorial (EKF) |
| `vision_localization_node` | `vision_localization.py` | Corrección de pose por ArUco |
| `navigation_node` | `navigation.py` | Seguidor de waypoints |

### Lanzamiento de nodos por separado

```bash
# Puente serie (debe arrancarse primero)
ros2 run robot_driver stm32_bridge_node

# Odometría a partir de encoders
ros2 run robot_driver odometry_node

# Filtro EKF
ros2 run robot_driver ekf_node

# Localización por visión
ros2 run robot_driver vision_localization_node

# Navegación (el robot empieza a moverse)
ros2 run robot_driver navigation_node
```

### Monitoreo de topics

```bash
# Pose filtrada
ros2 topic echo /odometry/filtered

# IMU en crudo
ros2 topic echo /imu/raw

# Ticks de encoders
ros2 topic echo /encoders/ticks

# Corrección por visión
ros2 topic echo /vision/pose

# Comandos de velocidad
ros2 topic echo /cmd_vel
```

---

## Grafo ROS2

### Topics publicados

| Topic | Tipo | Publicador | Frecuencia |
|-------|------|------------|------------|
| `/imu/raw` | `sensor_msgs/Imu` | stm32_bridge | ~102 Hz |
| `/mag/raw` | `sensor_msgs/MagneticField` | stm32_bridge | ~102 Hz |
| `/encoders/ticks` | `robot_msgs/EncoderTicks` | stm32_bridge | ~102 Hz |
| `/photo_trigger` | `std_msgs/Header` | stm32_bridge | ~102 Hz |
| `/motor_speed_target` | `geometry_msgs/Twist` | inv_kinematics | bajo demanda |
| `/odom` | `nav_msgs/Odometry` | odometry | ~102 Hz |
| `/odometry/filtered` | `nav_msgs/Odometry` | ekf | 50 Hz |
| `/vision/pose` | `geometry_msgs/PoseWithCovarianceStamped` | vision_localization | por eventos |
| `/cmd_vel` | `geometry_msgs/Twist` | navigation | ~10 Hz |

### Mensaje personalizado

```
robot_msgs/EncoderTicks
    std_msgs/Header header
    int32 enc_l          # Cuenta acumulada de ticks rueda izquierda
    int32 enc_r          # Cuenta acumulada de ticks rueda derecha
```

---

## Resultados

Se registraron tres configuraciones de ensayo. Cada una compara la trayectoria estimada por el EKF con la ground truth extraída de una cámara cenital.

| Config | Descripción | Datos |
|--------|-------------|-------|
| A | Solo odometría (sin visión) | [tests_results/config_a/](tests_results/config_a/) |
| B | Odometría + IMU | [tests_results/config_b/](tests_results/config_b/) |
| C | Fusión completa (Odom + IMU + Visión) | [tests_results/config_c/](tests_results/config_c/) |

**Generación de gráficas de trayectoria y análisis de error:**

```bash
# Extraer ground truth del vídeo cenital
python resources/results_analysis/ground_truth_tracker.py

# Comparar ground truth vs EKF
python resources/results_analysis/plot_trajectories.py
```

Salida: superposición de trayectorias 2D y gráficas de error frente a distancia con marcadores por vuelta.

---

## Recursos y calibración

### Calibración de la cámara

Se usa un patrón de tablero de ajedrez estándar. Se proporcionan 20 imágenes de calibración:

```bash
python resources/calibration/vision/vision_calibration_results.py
# Salida: matriz de cámara y coeficientes de distorsión en formato numpy
```

Verificación de precisión de detección a 50 cm y 100 cm:

```
resources/calibration/vision/accuracy_check/
```

### Calibración del bias del giróscopo

```bash
# Flashear gyro_calibration.ino en cualquier Arduino
# Luego analizar los datos registrados:
python resources/calibration/gyro/gyro_analysis.py
```

---

## Detalles del firmware

| Parámetro | Valor |
|-----------|-------|
| Reloj del sistema | 84 MHz (HSE 25 MHz + PLL) |
| Velocidad UART | 460800 bps |
| Bucle PID (TIM4) | 1 kHz |
| Frecuencia de telemetría | ~102.2 Hz |
| Resolución Timer 3 | 1 us |
| PWM motores | TIM1 CH1/CH2 |
| Interfaces encoder | TIM2 (izquierda), TIM5 (derecha) |
| Velocidad I2C1 | 400 kHz (modo rápido) |
| Ganancias PID | Kp=40, Ki=2 |
| Timeout watchdog | 500 ms (parada automática si se pierde comunicación) |

---

## Máquina de estados de localización por visión

El nodo de visión ejecuta una máquina de estados para garantizar mediciones fiables sin movimiento:

```
IDLE --(marcador < 0.5 m)--> PARANDO --(0.5 s asentamiento)--> MIDIENDO --(media 10 frames)--> COOLDOWN --(20 s)--> IDLE
```

Restricciones de medición:
- Distancia máxima de detección: **1.0 m**
- Ángulo lateral máximo: **30 grados**
- Covarianza de la actualización de pose: derivada de la varianza de las 10 muestras

---

