# CLAUDE.md

Tesis de maestría de Emiliano García Pérez (Universidad de Guanajuato): optimizador de trayectorias para un dron con cámara monocular que reconstruye un objeto en 3D gastando poca energía. Plataformas: Parrot Bebop 2 (real, interiores, ArUco) y DJI Matrice 100 (simulación en Gazebo, exteriores).

## Qué leer antes de trabajar

1. **[PLAN_IMPLEMENTACION.md](PLAN_IMPLEMENTACION.md)**: qué se va a construir, en qué orden, con las ecuaciones y el criterio de terminado de cada paso. **Es la fuente principal para cualquier tarea del optimizador.**
2. [CONTEXTO.md](CONTEXTO.md): la tesis completa, plataformas y avance.
3. [bebop/CONTEXTO.md](bebop/CONTEXTO.md): estado detallado del vuelo del Bebop (arquitectura ROS 2, geocerca, localización, marco del corral).

## Cómo trabajar un paso del plan

- Trabajar **un paso a la vez**, en el orden del plan. Si un paso depende de otro que no está hecho, decirlo antes de empezar.
- Antes de programar, resumir en pocas líneas qué se va a hacer y qué archivos se tocan.
- Cada módulo nuevo lleva sus pruebas en `optimizador/tests/`. Un paso está terminado solo cuando se cumple su «Listo cuando» del plan y las pruebas pasan.
- Al terminar: marcar la casilla del paso en el «Registro de avance» de `PLAN_IMPLEMENTACION.md` y anotar ahí cualquier valor que se haya fijado (por ejemplo `eta`, `n_reg`).
- Un paso por rama y por PR. No hacer commit ni push sin que Emiliano lo pida.
- Si al implementar algo del plan resulta imposible o mala idea, no improvisar otro diseño: explicar el problema y proponer el cambio para que Emiliano decida, y luego actualizar el plan.

## Reglas

- Responder en español, claro y directo, con explicaciones cortas (bloques breves y listas; Emiliano lee mejor así).
- **Nunca lanzar motores ni operar los drones.** Solo editar código, analizar bags, compilar y probar sin hardware. Los vuelos los hace Emiliano.
- Confirmar antes de cambios grandes; proponer cambios vía PR.
- Analizar los bags con datos reales antes de proponer cambios de ganancias del controlador.
- No subir videos `.avi`, `database.db` de COLMAP ni bags nuevos.
- `bebop/control_bebop/control_bebop/consumo_energ.py` (modelo de motores) ya no se usa: el modelo de energía es `optimizador/energia.py`.

## Código

- `optimizador/`: Python puro, **sin ROS**, para los dos drones. Unidades SI, marco del corral (x al frente al despegar, y a la izquierda, z arriba). Convenciones completas en el plan, §2.
- `bebop/control_bebop/`: paquete ROS 2 de vuelo del Bebop (ament_python). El Matrice reutiliza sus nodos: un cambio de tópicos o parámetros aquí afecta a los dos.
- `matrice100/drone_gazebo/`: simulación del Matrice.

## Comandos

```bash
# Optimizador
pip install -r optimizador/requirements.txt
python3 -m pytest optimizador/tests -q

# Bebop (ROS 2)
cd ~/ros2_ws && source /opt/ros/humble/setup.bash      # jazzy en el escritorio
colcon build --packages-select control_bebop && source install/setup.bash
cd src/Proyecto_tesis/bebop/control_bebop && python3 -m pytest test/ -q   # flake8/pep257 fallan por estilo antiguo
```
