# Matrice 100: plataforma de exteriores

Sección del repositorio de tesis dedicada al DJI Matrice 100. Aquí va la simulación en exteriores (objetos a escala real, como la estación de bomberos) y, en la fase 3, el despliegue en el dron físico con telemetría de batería para medir el consumo energético real.

Estado al 26 oct 2026: simulación en Gazebo subida (`drone_gazebo/`). Falta el despliegue en el dron físico.

## Contenido de `drone_gazebo/`

Paquete ROS 2 `drone_gazebo` (probado en el escritorio: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic; en Humble el launch usa copias del mundo y del modelo hechas para Fortress).

- `launch/matrice_flight.launch.py`: vuela el Matrice en Gazebo **reutilizando los nodos de `control_bebop`** (flight_manager, trajectory_planner, control_PID, sim_odometry con la geocerca, nodo_video, joystick) con `real_drone.yaml` + `sim.yaml` + `config/matrice.yaml`, más `ros_gz_bridge` y `pose_relay`. El dron arranca en (1.92, −10, 0.7). Argumentos: `use_joystick`, `headless`, `record_bag`, `launch_gui`, `launch_camera_view`.
- `launch/sim.launch.py`: teleoperación directa con `drone_teleop`.
- `config/matrice.yaml`: escala los parámetros del Bebop. Órbita de radio 20 m a 6 m de altura (centro (−1.92, 10), ~1 m/s, `lookat`) alrededor de la estación de bomberos; geocerca de ±35 m y techo de 15 m; `no_fly_boxes` con el edificio y margen de 2 m para que RETURN_HOME y RECOVERY lo rodeen (`route_around` en `bebop/control_bebop/control_bebop/safety.py`); sin visión ni deriva de odometría; cámara a 20 fps en `/camera/image_raw`.
- `config/bridge*.yaml`, `config/joy_params.yaml`: puente ROS↔Gazebo y mando.
- `worlds/drone_world.sdf` y `models/`: Matrice 100, estación de bomberos (`collapsed_fire_station`, `edificio_01`), plataforma de aterrizaje. Incluye los `.blend` originales de los modelos.

**Depende de `control_bebop`** (`exec_depend`): sin ese paquete en el workspace no arranca, y un cambio de tópicos o parámetros en `bebop/control_bebop` puede romper `matrice_flight.launch.py`. Las pruebas `test_route.py` del Bebop ya usan la geometría de la estación de bomberos.

```bash
cd ~/ros2_ws && colcon build --packages-select control_bebop drone_gazebo && source install/setup.bash
ros2 launch drone_gazebo matrice_flight.launch.py
```

Si todavía existe `~/ros2_ws/src/drone_gazebo` (la copia vieja), bórrala o renómbrala para que colcon no encuentre dos paquetes con el mismo nombre.

## Estructura

```
matrice100/
├── README.md          Este archivo
├── drone_gazebo/      Paquete ROS 2 de simulación del Matrice (mundo, modelos, launch, config)
└── docs/              Notas, parámetros identificados y resultados de esta plataforma
```

Más adelante, para el dron físico, se agregará un paquete puente con el SDK de DJI (equivalente a `real_drone_bridge` del Bebop) que publique la pose y la batería (voltaje y corriente).

## Pendientes

- Elegir el SDK/driver de ROS 2 para el Matrice 100 físico y la computadora a bordo.
- Definir qué telemetría de batería se registra para validar el modelo energético.
- Cámara: modelo y calibración de la cámara que llevará el Matrice.
