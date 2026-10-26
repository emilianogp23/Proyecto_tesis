# Matrice 100: plataforma de exteriores

Sección del repositorio de tesis dedicada al DJI Matrice 100. Aquí va la simulación en exteriores (objetos a escala real, como la estación de bomberos) y, en la fase 3, el despliegue en el dron físico con telemetría de batería para medir el consumo energético real.

Estado al 26 oct 2026: **estructura inicial**. El trabajo que ya existe vive solo en la computadora de escritorio (`~/ros2_ws/src/drone_gazebo`, fuera de git) y todavía no se ha subido aquí.

## Qué existe ya (fuera del repositorio)

Paquete `drone_gazebo` en el escritorio (Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic):

- `launch/matrice_flight.launch.py` (23 oct 2026): vuela el Matrice en Gazebo **reutilizando los nodos de `control_bebop`** (flight_manager, trajectory_planner, control_PID, etc.) sin copiarlos.
- `config/matrice.yaml`: escala los parámetros del Bebop. Órbita de radio 20 m a 6 m de altura alrededor de la estación de bomberos; geocerca de ±35 m; `no_fly_boxes` con el edificio para que RETURN_HOME y RECOVERY lo rodeen (`route_around` en `bebop/control_bebop/control_bebop/safety.py`).
- Mundo con la plataforma de despegue en el piso (z del mundo −0.472) y el dron centrado en su placa (1.92, −10).

Como depende de los nombres de tópicos y parámetros de `control_bebop`, cualquier cambio en `bebop/control_bebop` puede romper `matrice_flight.launch.py`. Las pruebas `test_route.py` del paquete del Bebop ya usan la geometría de la estación de bomberos.

## Estructura prevista

```
matrice100/
├── README.md          Este archivo
├── drone_gazebo/      Paquete ROS 2 de simulación del Matrice (mundo, modelo, launch, config)
└── docs/              Notas, parámetros identificados y resultados de esta plataforma
```

Más adelante, cuando se trabaje con el dron físico, se agregará un paquete puente con el SDK de DJI (equivalente a `real_drone_bridge` del Bebop) que publique la pose y la batería (voltaje y corriente).

## Cómo subir `drone_gazebo`

```bash
cd ~/ros2_ws/src
git clone https://github.com/emilianogp23/Proyecto_tesis.git   # si aún no está
cp -r drone_gazebo/* Proyecto_tesis/matrice100/drone_gazebo/
cd Proyecto_tesis && git add matrice100/drone_gazebo && git commit -m "Matrice 100: agregar drone_gazebo"
```

Después, borrar o renombrar la copia vieja de `~/ros2_ws/src/drone_gazebo` para que colcon no encuentre dos paquetes con el mismo nombre. Revisar antes que no haya archivos grandes (mallas, bags) de más de 100 MB, que GitHub rechaza.

## Pendientes

- Subir `drone_gazebo` (ver arriba) y documentar cómo lanzarlo.
- Elegir el SDK/driver de ROS 2 para el Matrice 100 físico y la computadora a bordo.
- Definir qué telemetría de batería se registra para validar el modelo energético.
- Cámara: modelo y calibración de la cámara que llevará el Matrice.
