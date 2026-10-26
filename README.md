# Proyecto_tesis

Implementar un optimizador de trayectorias para un vehículo aéreo no tripulado, equipado con cámara monocular, con el fin de maximizar la cantidad de información para la reconstrucción 3D de un objeto de interés, garantizando al mismo tiempo un consumo energético eficiente del vuelo, comparando su desempeño con trayectorias estándar.

Tesis de Maestría en Ingeniería Eléctrica, Universidad de Guanajuato. Resumen del proyecto, avance y siguientes pasos: **[CONTEXTO.md](CONTEXTO.md)**.

## Secciones

| Carpeta | Plataforma | Estado |
|---|---|---|
| [bebop/](bebop/) | Parrot Bebop 2, interiores | Control de vuelo en órbita, localización con ArUco, video y reconstrucción con COLMAP. Volado en real (oct 2026). Migrado de [Vuelo_bebop_v1](https://github.com/emilianogp23/Vuelo_bebop_v1). |
| [matrice100/](matrice100/) | DJI Matrice 100, exteriores | Estructura inicial. La simulación en Gazebo (`drone_gazebo`) está pendiente de subir. |

## Uso

Ambas secciones son paquetes ROS 2 dentro del mismo workspace de colcon:

```bash
cd ~/ros2_ws/src
git clone https://github.com/emilianogp23/Proyecto_tesis.git
cd ~/ros2_ws && colcon build --packages-select control_bebop && source install/setup.bash
```

Entornos: Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress (laptop) y Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic (escritorio). Cómo volar cada dron, en simulación y en real, está en el README de su carpeta.
