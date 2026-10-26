# Contexto de la tesis (para futuros chats)

Última actualización: 26 oct 2026. Este archivo resume la tesis completa y dónde está cada cosa. El detalle técnico de cada plataforma está en su carpeta: [bebop/CONTEXTO.md](bebop/CONTEXTO.md) (estado al día del Bebop) y [matrice100/README.md](matrice100/README.md).

## 1. La tesis

**Optimización de Trayectorias para Reconstrucción 3D Monocular Energéticamente Eficiente en Vehículos Aéreos No Tripulados.** Emiliano García Pérez, Maestría en Ingeniería Eléctrica, Universidad de Guanajuato (Salamanca). Asesor: Dr. Juan Pablo Ignacio Ramírez Paredes. Propuesta del 27 jul 2026; entregables con fecha 30 abr 2027 (tablero de Trello "EGP Trayectorias UAV").

Objetivo: implementar un optimizador de trayectorias para un UAV con cámara monocular que maximice la información para reconstruir en 3D un objeto, con consumo energético eficiente, y comparar su desempeño con trayectorias estándar.

Objetivos específicos:
- **OE1.** Formular el problema de optimización del escaneo: costo energético contra ganancia de información (modelo de la cámara monocular y Next-Best-View).
- **OE2.** Evaluar al menos tres algoritmos de optimización de trayectorias (calidad de reconstrucción y tiempo de cómputo).
- **OE3.** Validar la estrategia secuencial en simulación, con menor costo energético que las trayectorias estándar.
- **OE4.** Implementar el algoritmo elegido en un dron físico y comparar la reconstrucción real contra la simulada.

Estrategia en dos vuelos: (1) trayectoria genérica (órbita) que da una reconstrucción parcial; (2) trayectoria optimizada con lo aprendido en el primero, eligiendo vistas NBV y conectándolas con el menor gasto de energía.

Entregables: documento de tesis, artículo científico, prototipo funcional (código y validación física) y una base de datos de trayectorias y reconstrucciones.

## 2. Plataformas y repositorios

| Plataforma | Carpeta | Uso |
|---|---|---|
| Parrot Bebop 2 | [bebop/](bebop/) | Prototipo en interiores: laboratorio con corral de ~5 × 5 m, localización con ArUco, primer vuelo genérico (órbita) ya volado en real. |
| DJI Matrice 100 | [matrice100/](matrice100/) | Exteriores y escala real: simulación de una órbita de 20 m alrededor de una estación de bomberos; dron físico para la fase 3 con telemetría de batería. |

- Este repositorio (`emilianogp23/Proyecto_tesis`) es el oficial desde el 26 oct 2026. Se clona en `~/ros2_ws/src/Proyecto_tesis`; colcon encuentra los paquetes dentro de `bebop/` (y de `matrice100/` cuando tengan `package.xml`).
- `emilianogp23/Vuelo_bebop_v1` queda como histórico: su contenido se copió a `bebop/` desde el commit `f198135` (23 oct 2026), sin los videos `.avi` de respaldo (~146 MB), que siguen allá.
- El Matrice reutiliza los nodos de `control_bebop` (flight_manager, trajectory_planner, control_PID) con otro YAML. Un cambio de tópicos o parámetros en `bebop/control_bebop` afecta a los dos.

Computadoras: laptop (Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress) y escritorio (Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic). El paquete de simulación del Matrice (`drone_gazebo`) solo está en el escritorio, fuera de git.

## 3. Avance contra el plan

| Fase / tarea (Trello) | Estado en el código |
|---|---|
| F1 Modelado de la cámara monocular | Cámara del Bebop calibrada (ChArUco, RMS 0.61 px, `bebop/control_bebop/config/bebop2_camera_calib.yaml`); falta validarla con puntos de control. **Falta** la función de incertidumbre por distancia y ángulo de incidencia. |
| F1 Modelado del consumo energético | Solo un borrador: `bebop/control_bebop/control_bebop/consumo_energ.py` (CasADi, modelo de motores con constantes en 0, no ejecutable). |
| F1 Problema de optimización (OE1) | No formulado en código. |
| F2 Arquitectura de simulación | Hecha en **Gazebo** + ROS 2 (Bebop y Matrice), no en Webots como dice la propuesta. Decidir si se queda Gazebo y ajustarlo en el documento. |
| F2 Primer vuelo genérico | Órbita circular con la cámara al centro (`yaw_mode: lookat`), geocerca, máquina de estados, mando PS5. Volada en real el 23 oct 2026: radio real 0.47 m contra 0.5 m, lock visual 100 %. Video para reconstrucción con `nodo_video`. |
| Reconstrucción 3D | Pipeline con COLMAP (`bebop/control_bebop/RECONSTRUCCION_3D.md`): nube dispersa y malla; la nube densa requiere COLMAP con CUDA. La propuesta habla de SLAM visual y mapa de ocupación: hoy es SfM fuera de línea. |
| F2 Segundo vuelo optimizado (NBV) | **No existe.** El trajectory_planner solo hace órbita; hay que extenderlo a puntos arbitrarios. |
| OE2 Tres algoritmos | Pendiente. |
| F3 Evaluación en simulación / hardware / campo | Pendiente. Ya hay bags de vuelos reales del Bebop (16 y 23 oct) en `bebop/control_bebop/Bags_respaldo/`, base de la "base de datos de trayectorias". |

## 4. Siguientes pasos sugeridos

1. Bebop: vuelos 3 y 4 de la guía (visión al mando en hover y en órbita) y textura en la caja para la reconstrucción (ver `bebop/CONTEXTO.md` §8).
2. Subir `drone_gazebo` a `matrice100/` (pasos en su README).
3. OE1: escribir el modelo de incertidumbre de la cámara y el modelo de energía con parámetros reales (Bebop y Matrice), y la función objetivo.
4. Extender el trajectory_planner a una lista de waypoints/vistas para el segundo vuelo y conectar la salida de COLMAP con un primer NBV.
5. Definir las métricas de comparación: energía (modelo y telemetría), cobertura y error de la nube contra el modelo, tiempo de vuelo y de cómputo.

## 5. Convenciones para trabajar con Emiliano

- Responder en español, claro y directo.
- No lanzar motores ni operar los drones desde aquí: solo analizar bags, editar código, compilar y probar sin hardware.
- Confirmar antes de commits, push o cambios grandes; proponer cambios vía PR.
- Analizar los bags con datos reales antes de proponer cambios de ganancias.
- Pruebas del Bebop: `cd bebop/control_bebop && python3 -m pytest test/ -q` (flake8/pep257 fallan por estilo antiguo).
