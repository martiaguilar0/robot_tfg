**Trabajo de Fin de Grado (TFG) — Diseño e implementación de un robot movil**

Repositorio: [github.com/martiaguilar0/robot_tfg](https://github.com/martiaguilar0/robot_tfg)

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
- [Flujo de datos ROS2](#flujo-de-datos-ros2)
- [Resultados y validación](#resultados-y-validación)
- [Recursos y calibración](#recursos-y-calibración)
- [Documentación adicional](#documentación-adicional)

---

## Descripción general

En este Trabajo de Fin de Grado (TFG) desarrollamos un robot móvil diferencial que sigue waypoints predefinidos. En el proyecto integramos tres fuentes de información —odometría de ruedas, IMU y visión por marcadores ArUco— y empleamos un Filtro de Kalman Extendido (EKF) para fusionarlas y estimar la pose del robot.

El robot está organizado en dos capas computacionales:

| Capa | Función | Rol |
|------|---------|-----|
| Bajo nivel | STM32F401 | Control PID de motores, lectura de encoders, adquisición IMU y telemetría serie |
| Alto nivel | NVIDIA Jetson | Middleware ROS2, procesamiento de visión, EKF y navegación |

Archivos clave del proyecto:

- Firmware: `firmware/stm_fw_1.0.ioc` y `firmware/Core/Src/main.c`
- ROS2: `ros2_ws/src/robot_driver/robot_driver/`, que contiene los nodos principales del sistema

La comunicación entre ambas capas se realiza por UART a 460800 baudios.

---

## Arquitectura del sistema

**Capa de bajo nivel — STM32F401**

- Adquisición de encoders de cuadratura
- Lectura de IMU ICM-20948
- Control PID de motores a 1 kHz
- Envía telemetría por UART2

**Capa de alto nivel — NVIDIA Jetson (ROS2)**

- Decodifica la telemetría del STM32
- Publica datos en topics ROS2
- Detecta marcadores ArUco desde la cámara CSI
- Fusiona odometría, IMU y visión con EKF
- Planifica trayectorias y genera `/cmd_vel`

### Flujo sensorial principal

- `/encoders/ticks` → `odometry.py` → `/odom`
- `/imu/raw` → `ekf.py`
- `/vision/pose` → `ekf.py`
- `/odometry/filtered` → `navigation.py`
- `/cmd_vel` → `inv_kinematics.py` → `/motor_speed_target`
- `/motor_speed_target` → `stm32_bridge.py` → STM32

---

## Hardware

| Componente | Modelo | Detalles |
|------------|--------|---------|
| Microcontrolador | STM32F401RCTx | 84 MHz |
| Ordenador de a bordo | NVIDIA Jetson | Host ROS2 y visión |
| IMU | ICM-20948 | Acelerómetro, giroscopio y magnetómetro |
| Encoders | Cuadratura | 1496 ticks/rev |
| Cámara | Waveshare IMX219 | 1280×720 @ 20 FPS |
| Ruedas | — | Diámetro 80 mm |
| Distancia entre ejes | — | 223.5 mm |
| Comunicación | UART2 | 460800 baudios |

### Parámetros físicos

- Diámetro de rueda: 0.08 m
- Distancia entre ejes: 0.2235 m
- Metros por tick de encoder: 0.000168 m

### Marcadores ArUco

- Diccionario: `DICT_5X5_1000`
- Tamaño del marcador: 60 mm

---

## Estructura del repositorio

El repositorio tiene dos bloques principales:

- `firmware/`: código STM32 y proyecto STM32CubeIDE
- `ros2_ws/`: workspace ROS2 con paquetes Python, configuración y mensajes

### Archivos importantes de firmware

- `firmware/stm_fw_1.0.ioc`: proyecto STM32CubeIDE
- `firmware/Core/Src/main.c`: bucle principal, inicialización de periféricos, PID, encoders, IMU y telemetría
- `firmware/Core/Src/stm32f4xx_it.c`: manejadores de interrupciones
- `firmware/Core/Startup/startup_stm32f401rctx.s`: vector de arranque

### Archivos importantes de ROS2

La carpeta principal donde están los nodos es `ros2_ws/src/robot_driver/robot_driver/`.

- `stm32_bridge.py`: puente serie entre STM32 y ROS2
- `odometry.py`: genera `/odom` desde los encoders
- `vision_localization.py`: detecta marcadores ArUco y publica `/vision/pose`
- `ekf.py`: fusiona odometría, IMU y visión para producir `/odometry/filtered`
- `navigation.py`: sigue waypoints y publica `/cmd_vel`
- `inv_kinematics.py`: convierte `/cmd_vel` en `/motor_speed_target`

Otros archivos clave:

- `ros2_ws/src/robot_driver/config/navigation_config.yaml`: parámetros de navegación y waypoints
- `ros2_ws/src/robot_driver/launch/robot_full.launch.py`: launch completo del sistema
- `ros2_ws/src/robot_msgs/msg/EncoderTicks.msg`: mensaje personalizado de encoders

```
robot_tfg/
├── firmware/
│   ├── stm_fw_1.0.ioc
│   ├── Core/
│   │   ├── Src/
│   │   │   ├── main.c
│   │   │   ├── stm32f4xx_it.c
│   │   │   ├── stm32f4xx_hal_msp.c
│   │   │   └── ...
│   │   └── Inc/
│   └── Startup/
│       └── startup_stm32f401rctx.s
├── ros2_ws/
│   └── src/
│       ├── robot_driver/
│       │   ├── robot_driver/
│       │   │   ├── stm32_bridge.py
│   │       │   ├── odometry.py
│   │       │   ├── vision_localization.py
│   │       │   ├── ekf.py
│   │       │   ├── navigation.py
│   │       │   ├── inv_kinematics.py
│   │       │   └── ...
│       │   ├── launch/
│       │   │   └── robot_full.launch.py
│       │   ├── config/
│       │   │   └── navigation_config.yaml
│       │   ├── package.xml
│       │   └── setup.py
│       └── robot_msgs/
│           └── msg/
│               └── EncoderTicks.msg
├── resources/
│   ├── calibration/
│   │   ├── gyro/
│   │   │   ├── gyro_calibration.ino
│   │   │   └── gyro_calibration_analysis.py
│   │   └── vision/
│   │       ├── vision_calibration_results.py
│   │       └── accuracy_check/
│   │           ├── vision_calibration_analysis.py
│   │           └── data/
│   └── results_analysis/
│       ├── ground_truth_tracker.py
│       ├── plot_trajectories.py
│       └── gt_correction.py
└── tests_results/
    ├── test_config_a/
    ├── test_config_b/
    └── test_config_c/
```

---

## Requisitos previos

### Firmware

- STM32CubeIDE
- Programador ST-Link
- MCU: STM32F401RCT6 @ 84 MHz

### ROS2 (Jetson)

- Ubuntu 20.04 o 22.04
- ROS2 Dashing
- Python 3.8+

**Dependencias Python**

```bash
pip install opencv-contrib-python numpy scipy pyserial
```

**Dependencias ROS2**

```bash
sudo apt install ros-dashing-sensor-msgs ros-dashing-geometry-msgs ros-dashing-nav-msgs
```

---

## Instalación

### 1. Firmware (STM32)

1. Abrir STM32CubeIDE.
2. Importar el proyecto desde `firmware/`.
3. Compilar.
4. Flashear en la placa con ST-Link o STM32CubeProgrammer.
5. Verificar transmisión serial a 460800 baudios.

### 2. Workspace ROS2 (Jetson)

```bash
git clone https://github.com/martiaguilar0/robot_tfg.git ~/robot_tfg
cd ~/robot_tfg/ros2_ws
source /opt/ros/dashing/setup.bash
pip install opencv-contrib-python numpy scipy pyserial
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep robot
```

---

## Configuración

Archivo de configuración:

- `ros2_ws/src/robot_driver/config/navigation_config.yaml`

Parámetros clave:

- `linear_speed`: velocidad lineal máxima (m/s)
- `angular_speed`: velocidad angular máxima (rad/s)
- `distance_tolerance`: umbral de llegada al waypoint (m)
- `waypoints`: recorrido en metros
- `markers`: posición y orientación de marcadores ArUco

**Marcadores definidos actualmente**

- 493 → `[1.50, 0.0, 0.0]`
- 494 → `[-0.3, -1.20, 3.14159265]`

---

## Ejecución del sistema

```bash
source /opt/ros/dashing/setup.bash
source ~/robot_tfg/ros2_ws/install/setup.bash
ros2 launch robot_driver robot_full.launch.py
```

Si la Jetson usa otro puerto serie, es necesario modificar `port='/dev/ttyTHS1'` en `ros2_ws/src/robot_driver/robot_driver/stm32_bridge.py`.

---

## Flujo de datos ROS2

- `/encoders/ticks` → `odometry.py` → `/odom`
- `/imu/raw` → `ekf.py`
- `/vision/pose` → `ekf.py`
- `/odometry/filtered` → `navigation.py`
- `/cmd_vel` → `inv_kinematics.py`
- `/motor_speed_target` → `stm32_bridge.py`

---

## Resultados y validación

Se ha validado una arquitectura híbrida de localización que combina:

- odometría de encoders
- datos IMU
- correcciones absolutas por marcadores ArUco
- fusión mediante EKF

Los datos experimentales y las gráficas de los ensayos están disponibles en `tests_results/`, y los scripts de análisis en `resources/results_analysis/`.

### Videos de funcionamiento

- [Configuración A](https://www.youtube.com/watch?v=_0mUWip__zQ)
- [Configuración B](https://www.youtube.com/watch?v=M9yNuGetR2c)
- [Configuración C](https://www.youtube.com/watch?v=wDx567-2WXk)
---

## Recursos y calibración

- `resources/calibration/gyro/` — calibración de la IMU
- `resources/calibration/vision/` — calibración de la cámara y análisis de precisión
- `resources/results_analysis/` — scripts de seguimiento y comparación de trayectorias

---


