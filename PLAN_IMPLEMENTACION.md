# Plan de implementación del optimizador

Última actualización: 25 nov 2026. Este archivo es la guía para implementar la tesis paso a paso con Claude Code. Cada paso dice qué hacer, en qué archivos, cómo se hace y cuándo está terminado. Al terminar un paso, marca su casilla en el [registro de avance](#registro-de-avance).

Documentos de diseño de donde sale este plan (Claude Docs, para leer con calma; aquí está lo esencial para programar):

- D1, modelo de energía: https://claude.ai/artifact/1mqHFYSDphXtx87qsGqxA2
- D2, planteamiento de optimización con ecuaciones: https://claude.ai/artifact/M2Jr5iZ1iHQCmAHenzdF5F
- Resumen sin fórmulas (por qué cada decisión): https://claude.ai/artifact/Y8fr7XksbwSXmeJJF6BACz
- Por dónde empezar (versión corta de este plan): https://claude.ai/artifact/RNV9CLKh3k4WD19JwxziRk

---

## 1. El método en 10 líneas

1. **Primer vuelo:** espiral (u órbita) alrededor del objeto, ya implementada en `bebop/control_bebop`. Su video se reconstruye con COLMAP.
2. **Del primer modelo** se saca: puntos 3D con su incertidumbre, cámaras, un mapa de vóxeles (libre, ocupado, desconocido) y los **vóxeles objetivo** (lo que falta ver bien).
3. **Segundo vuelo: solo a los huecos.** Una regla NBV propone M vistas candidatas. El optimizador decide cuáles visitar, en qué orden, qué tan cerca, a qué velocidad y cuánto frena en cada vista. Las vistas se unen con tramos de mínimo jerk.
4. **Tres objetivos a minimizar:** energía (modelo físico de D1), precisión (percentil 95 de la incertidumbre por punto, con información de Fisher) y menos completitud (fracción de vóxeles objetivo que quedan cubiertos).
5. **11 restricciones:** espacio, distancia de seguridad, inclinación, empuje, descenso, suavidad, desenfoque, traslape, registrabilidad, localizabilidad y reserva de batería.
6. **Tres algoritmos** (NSGA-II, MOEA/D y una variante de MOPSO) con jMetalPy, más búsqueda al azar como línea base. Se comparan con HV, IGD+ y Wilcoxon sobre 20 a 30 semillas.
7. **Se comparan tres segundos vuelos:** otra espiral (`x_std`), NBV voraz sin optimizar (`x_nbv`) y la solución optimizada (`x*`).
8. **Predecir y comprobar:** antes de volar, el optimizador predice energía, precisión y completitud. Se vuelan varias rutas y se compara lo predicho con lo medido (batería del bag; COLMAP contra una referencia hecha con fotos de celular).
9. **Cada dron escoge su ruta según su física:** Bebop real en el corral (ArUco) y Matrice 100 en Gazebo (GPS). Solo cambian los parámetros del dron y la función de localizabilidad.
10. **Ablación:** optimizar con «energía proporcional a la distancia» y evaluar con el modelo físico, para mostrar que el modelo de energía cambia las decisiones.

---

## 2. Estructura del código nuevo

Todo el optimizador va en una carpeta nueva en la raíz, **en Python puro, sin ROS**, porque sirve para los dos drones y se prueba sin hardware.

```
optimizador/
  __init__.py
  parametros.py      # datos de cada dron y del escenario (paso 3)
  energia.py         # modelo de energía de D1 (paso 4)
  modelo_previo.py   # leer y alinear la reconstrucción del primer vuelo (paso 6)
  calibracion.py     # ajustar η y P_av con el vuelo de calibración (paso 7)
  observacion.py     # visibilidad, Fisher, σ_j, vóxeles y cobertura (paso 8)
  candidatas.py      # vistas NBV candidatas (paso 9)
  trayectoria.py     # decodificar x → ruta; espiral estándar; NBV voraz (paso 10)
  evaluar.py         # evaluate(x) → (F, G) (paso 11)
  problema_jmetal.py # el problema envuelto para jMetalPy (paso 12)
  experimentos.py    # corridas, métricas y estadística (pasos 13 y 14)
  exportar.py        # ruta → archivo que sigue el dron (paso 15)
  tests/
    test_energia.py, test_observacion.py, ...
  datos/             # pequeños archivos de prueba (no subir reconstrucciones completas)
```

Dependencias: `numpy`, `scipy`, `pycolmap`, `jmetalpy`, `matplotlib`, `pytest`. Ponerlas en `optimizador/requirements.txt`.

### Convenciones

- **Unidades SI** en todo: metros, segundos, newtons, watts, joules. Ángulos en radianes dentro del código.
- **Marco del corral** (igual que `bebop/CONTEXTO.md` §3): origen en la cruz de la zona segura, x al frente del dron al despegar, y a la izquierda, z arriba. En el Matrice, marco local ENU con origen en el despegue.
- **Trayectoria muestreada:** un `dataclass Trayectoria` con arreglos `t (N,)`, `p (N,3)`, `v (N,3)`, `a (N,3)`, `psi (N,)`, muestreados cada `dt = 0.05 s`. Todos los módulos reciben y devuelven este tipo.
- **Cuadros:** un `dataclass Cuadro` con `t`, posición `o (3,)` y rotación mundo→cámara `R (3,3)`.
- Docstrings y comentarios en español. Funciones puras siempre que se pueda (fáciles de probar).
- Cada módulo con su archivo de pruebas. Las pruebas no deben depender de archivos grandes: usar escenas sintéticas (una caja de puntos) o un recorte pequeño en `optimizador/datos/`.

---

## 3. Los pasos

Leyenda: 👤 = lo hace Emiliano (vuelos, cosas físicas). 🤖 = se puede pedir a Claude Code. Fechas aproximadas según el plan general (OE1 cierra el 18 dic 2026).

### Paso 1 — Subir el trabajo local a GitHub 👤🤖 (semana del 25 nov)

- **Qué:** la laptop todavía tiene la estructura vieja (`~/ros2_ws/src/control_bebop`, sin `Proyecto_tesis`). Lo hecho después del 26 oct (aterrizaje suave, `test_soft_landing.py`, YOLO, `consumo_energ.py`, cambios de `real_drone.yaml`, launches) no está en este repo.
- **Cómo:** comparar `~/ros2_ws/src/control_bebop` con `bebop/control_bebop`, copiar solo código, configuración y `resumen.json` de las reconstrucciones (no videos `.avi`, ni `database.db`, ni bags nuevos). Rama nueva y PR.
- **Listo cuando:** el PR está abierto, las pruebas de `bebop/control_bebop` pasan y, después del merge, la laptop usa `~/ros2_ws/src/Proyecto_tesis`.

### Paso 2 — Grabar la batería en los bags 👤🤖 (semana del 25 nov)

- **Qué:** el `ros2 bag record` de `bebop/control_bebop/launch/real_flight.launch.py` no incluye la batería.
- **Cómo:** con el dron encendido en el piso y los motores apagados, `ros2 topic list | grep -i batt` para ver el nombre del tópico de batería de `ros2_bebop_driver`. Agregarlo a la lista del `ExecuteProcess` de `ros2 bag record`. Si el driver da voltaje además del porcentaje, grabar los dos.
- **Listo cuando:** un bag de 1 minuto con el dron en el piso tiene mensajes de batería (`ros2 bag info`).
- **Nota:** el Bebop reporta el porcentaje en enteros. Con ~30 Wh, 1 % ≈ 0.3 Wh. Por eso los vuelos de comprobación se repiten.
- **Hecho el 25 nov, probado con el dron en el piso:** `ros2_bebop_driver` no publicaba la batería. Se agregó al parche (`bebop/patches/ros2_bebop_driver_altitude.patch`) como `/bebop/battery` (`std_msgs/UInt8`, porcentaje; el Bebop 2 no da voltaje). Ya está en el `ros2 bag record` de `bebop/control_bebop/launch/real_flight.launch.py`, y `bebop/control_bebop/scripts/analisis_bag.py` reporta el % inicial, el final y el gastado. Driver y control_bebop compilados en el escritorio. En la laptop hay que aplicar el parche nuevo.

### Paso 3 — Crear `optimizador/` y `parametros.py` 🤖 (semana del 25 nov)

- **Qué:** la carpeta, `requirements.txt`, `tests/` y los parámetros de cada dron.
- **Contenido de `parametros.py`:** un `dataclass Dron` y dos instancias.

| Parámetro | Bebop 2 | Matrice 100 | Fuente |
|---|---|---|---|
| masa `m` | 0.503 kg | ≈ 2.4 kg (con cámara) | Gazebo / ficha técnica, confirmar |
| área total de hélices `A` | 0.071 m² (4 × π·0.075²) | 0.342 m² (4 × π·0.165²) | diámetro de hélice |
| densidad del aire `rho` | 1.0 kg/m³ | 1.0 kg/m³ (ajustar al lugar) | Salamanca ≈ 1,720 m s. n. m. |
| arrastre `CDS` (C_D·S) | 0.01 m² | 0.06 m² | literatura, valores iniciales |
| eficiencia `eta` | 0.45 (inicial) | 0.45 (inicial) | se calibra (paso 7) |
| aviónica `P_av` | por calibrar | por calibrar | paso 7 |
| batería `E_nom` | ≈ 30 Wh | ≈ 100 Wh (TB47D) | ficha técnica |
| `theta_max`, `T_max`, `a_max`, `v_desc` | ver tabla de valores (§4) | ver §4 | |

- También un `dataclass Escenario`: geocerca, alturas, centro del objeto `c_o`, despegue `p_0`, cámara (intrínsecos de `bebop/control_bebop/config/bebop2_camera_calib.yaml`, inclinación, tiempo de exposición).
- **Listo cuando:** `pytest optimizador/tests` corre.

### Paso 4 — `energia.py`: modelo de energía de D1 🤖 (semana del 25 nov)

Para cada muestra de la trayectoria:

1. Empuje (vector), con arrastre parásito:
   `T·n̂ = m·(a + g·e_z) + ½·rho·CDS·‖v‖·v`  →  `T = ‖·‖`, `n̂ = (·)/T`
2. Componentes de la velocidad respecto al disco: `v_perp = v·n̂`, `v_par = ‖v − v_perp·n̂‖`.
3. Velocidad inducida en hover: `v_h = sqrt(T / (2·rho·A))`.
4. Velocidad inducida (Glauert), resolver con Newton-Raphson partiendo de `v_i = v_h`:
   `v_i = v_h² / sqrt(v_par² + (v_perp + v_i)²)`
   Derivada para Newton: `f(v_i) = v_i − v_h²/s`, `s = sqrt(v_par² + (v_perp+v_i)²)`, `f' = 1 + v_h²·(v_perp+v_i)/s³`. 20 a 50 iteraciones, vectorizado con numpy.
5. Potencia eléctrica: `P = P_av + (1/eta)·T·(v_i + v_perp)`.
6. Energía: `E = ∫ P dt` (regla del trapecio).

Además, la **aproximación polinomial a baja velocidad** (para comparar y para la tesis):
`P ≈ c0 + c1‖v_xy‖² + c2‖v‖³ + c3·m·(a·v) + c4·m·g·v_z + c5·m·a_z`, con
`c0 = P_av + P_h0/eta`, `c1 = −m·g/(4·eta·v_h0)`, `c2 = rho·CDS/(4·eta)`, `c3 = c4 = 1/(2·eta)`, `c5 = 3·v_h0/(2·eta)`, donde `P_h0 = m·g·v_h0` y `v_h0 = sqrt(m·g/(2·rho·A))`.

Y un modelo ingenuo `energia_distancia(tray, k)` = k × longitud recorrida, para la ablación (paso 18).

- **Pruebas (listo cuando pasan):**
  - Bebop en hover con `eta = 0.45`, `P_av = 0`: `v_h ≈ 5.9 m/s`, potencia ideal `≈ 29 W`, eléctrica `≈ 65 W`.
  - Matrice: la potencia en vuelo recto nivelado tiene su mínimo entre 6 y 11 m/s (esperado ≈ 8–9 m/s) y es menor que la de hover.
  - Subir cuesta más que bajar a la misma rapidez; acelerar cuesta más que ir a velocidad constante.
  - El polinomio coincide con el modelo completo (error < 5 %) a velocidades bajas (≤ 0.5 m/s en el Bebop).
  - Newton converge (residuo < 1e-9) para todo el rango de prueba.

### Paso 5 — Preparar el corral 👤 (semana del 25 nov)

- Mover la caja al centro del corral y revisar que despegar desde una esquina sea seguro con el mapa ArUco actual. Con la caja en (1, 0) las vistas del lado +x quedan a menos de 0.7 m de la geocerca.
- **Listo cuando:** hay coordenadas nuevas de caja y despegue, anotadas en `bebop/control_bebop/config/` y en `parametros.py`.
- **Extra (🤖, sin volar):** sacar el primer valor de `n_reg`: con pycolmap, en `reconstrucciones/vuelo_20261030_103040`, el número mínimo de puntos 3D con que se registró una imagen.

### Paso 6 — `modelo_previo.py`: leer y alinear el primer vuelo 🤖 (30 nov – 4 dic)

- **Qué:** leer la reconstrucción de COLMAP (`sparse/0`) con pycolmap y devolver un `ModeloPrevio` con:
  - puntos `X (P,3)`, normales `n (P,3)` (estimadas con PCA de vecinos o desde la dirección media de las cámaras que lo ven), número de cámaras por punto, y los puntos firmes `P_reg` (vistos en ≥ 3 cámaras);
  - cámaras del primer vuelo (`Cuadro`s);
  - la información previa por punto `Λ_j⁰` (se calcula con `observacion.py` usando esas cámaras; en este paso dejar el gancho);
  - reducir a 2,000–5,000 puntos (submuestreo por vóxel).
- **Alineación al corral:** COLMAP sale en escala arbitraria. Estimar la similitud (escala, rotación, traslación) que lleva los centros de cámara de COLMAP a las posiciones del dron en el bag en esos mismos instantes (`/vision/fused_pose` o `/bebop1/pose`), con el método de Umeyama. Reportar el error residual.
- **Mapa de vóxeles:** rejilla alrededor del objeto con tres estados: ocupado (tiene puntos), libre (atravesado por rayos de cámara a punto) y desconocido.
- **Listo cuando:** con el vuelo del 30 oct, la caja queda en metros, en su lugar del corral (±5 cm), y el residuo de Umeyama se reporta. Prueba sintética: una similitud conocida se recupera con error < 1e-6.

### Paso 7 — Vuelo de calibración y `calibracion.py` 👤🤖 (30 nov – 4 dic)

- 👤 Volar con la batería grabada: hover de 3 a 5 minutos y rectas a 0.2, 0.35 y 0.5 m/s.
- 🤖 `calibracion.py`: leer pose y batería del bag, derivar `v` y `a` (filtrar), predecir la energía con `energia.py` y ajustar `eta` y `P_av` por mínimos cuadrados contra la energía medida. Si no se pueden separar, fijar `P_av` en un valor documentado y ajustar solo `eta`.
- 🤖 Para el Matrice: el mismo ajuste con el dataset de Rodrigues et al. 2021 (Matrice 100 real con voltaje y corriente; `Rodrigues2021Inflight` en la bibliografía).
- **Listo cuando:** se reporta el error relativo de la energía predicha contra la medida en cada vuelo y los valores de `eta` y `P_av` quedan en `parametros.py`.

### Paso 8 — `observacion.py`: qué se ve y qué tan bien 🤖 (7 – 11 dic)

Para un conjunto de cuadros y los puntos del modelo previo:

1. **Visibilidad** `ν_jf = 1` si se cumplen las cuatro condiciones:
   - la proyección cae dentro de la imagen con margen de 20 px;
   - distancia `d_min ≤ d_jf ≤ d_max`;
   - ángulo de incidencia `α_jf ≤ α_max`, con `cos α = n_j · (o_f − X_j)/‖o_f − X_j‖`;
   - sin oclusión: el rayo cámara→punto no atraviesa vóxeles ocupados (recorrido de rejilla tipo Amanatides-Woo).
2. **Peso:** `w_jf = ν_jf · cos^γ(α_jf)`.
3. **Proyección estenopeica:** en coordenadas de cámara `(X, Y, Z) = R_f (X_j − o_f)`, píxel `(f_x X/Z + c_x, f_y Y/Z + c_y)`.
4. **Jacobiano** respecto a la posición del punto (2×3):
   `H_jf = [[f_x/Z, 0, −f_x X/Z²], [0, f_y/Z, −f_y Y/Z²]] · R_f`
5. **Información** de la observación: `Λ_jf = (w_jf/σ_px²) · H_jfᵀ H_jf`.
6. **Covarianza** del punto: `Σ_j = (Λ_j⁰ + ε·I + Σ_f Λ_jf)⁻¹`, y `σ_j = sqrt(tr Σ_j)`. `ε·I` evita la singularidad de un punto visto en una sola cámara; alternativamente limitar `σ_j ≤ σ_cap`.
7. **Precisión:** `f_P = percentil 95 de {σ_j}`.
8. **Vóxeles objetivo** `S`: vóxeles desconocidos vecinos de libres (frontera) más vóxeles ocupados débiles (vistos en menos de `n_min` cámaras o con `σ` alta). Para desconocidos, normal = dirección hacia afuera desde `c_o`.
9. **Cobertura:** un vóxel de `S` queda cubierto si lo ven al menos `n_min` cuadros (contando los del primer vuelo) con `α ≤ α_max`, y el máximo ángulo entre pares de rayos que llegan a él es `≥ β_min`. `f_C = (cubiertos)/|S|`.

- **Rendimiento:** esta función se llama decenas de miles de veces. Vectorizar con numpy por cuadro, precalcular la oclusión de puntos contra la rejilla y, si hace falta, cachear por cuadro.
- **Listo cuando:**
  - prueba sintética: un punto visto desde dos cámaras a 90° tiene `σ` mucho menor que visto desde dos cámaras casi paralelas; agregar cámaras nunca aumenta `σ`;
  - con el vuelo del 30 oct, pintar la nube por `σ_j` (archivo PLY o gráfica) y los puntos más inciertos salen donde se espera (techo de la caja, esquinas);
  - se reporta el tiempo por evaluación.

### Paso 9 — `candidatas.py`: vistas NBV 🤖 (7 – 11 dic)

- Agrupar los vóxeles objetivo y los puntos de mayor `σ` en regiones (por ejemplo, k-means sobre posición y normal).
- Por región, muestrear poses a distancias en `[d_min, d_max]` sobre la dirección de la normal (con dispersión angular), descartar las que violan geocerca, distancia de seguridad u oclusión, y calcular la **ganancia** `G_m` = vóxeles objetivo nuevos que vería + reducción de `Σ σ_j` de los puntos peor ubicados.
- Quedarse con las `M = 25` de mayor ganancia con supresión de no-máximos (separación angular mínima entre candidatas).
- Cada candidata: punto al que mira `X*_m`, dirección de vista unitaria `d_m`, ganancia `G_m`.
- **Listo cuando:** se dibujan las 25 flechas alrededor de la caja y tienen sentido a simple vista; la prueba verifica que todas respetan geocerca y distancia de seguridad.

### Paso 10 — `trayectoria.py`: de x a ruta 🤖 (14 – 18 dic)

**Vector de decisión** (M = 25 → 52 variables, todas continuas y acotadas):
`x = (κ_1..κ_M, ρ_1..ρ_M, v_c, λ)`

- **Cuáles y en qué orden (llaves aleatorias):** `A(x) = {m : κ_m > 0.5}`, ordenadas de menor a mayor `κ_m`.
- **Qué tan cerca:** `q_m = X*_m − ρ_m·d_m`, `ψ_m = atan2(d_m,y, d_m,x)`, con `d_min ≤ ρ_m ≤ d_max`.
- **Cómo se vuela:** `v_c` velocidad de crucero; en cada vista `‖v(t_m)‖ = λ·v_c`, `λ ∈ [0, 1]` (0 = parar y disparar, 1 = pasar sin frenar).
- **Ruta:** `p_0 → q_(1) → … → q_(K) → p_0`.
- **Tramos de mínimo jerk** (polinomio de quinto grado por eje) con posición, velocidad y aceleración fijas en los extremos. Velocidad en cada vista: `λ·v_c` en la dirección del punto anterior al siguiente; aceleración cero. Duración del tramo: longitud / `v_c` más el tiempo de rampa con `a_max`. En `p_0` velocidad cero.
- **Conexión segura:** si la recta entre dos vistas pasa a menos de `d_safe` del objeto, insertar un punto intermedio que lo rodee (sobre el cilindro o esfera de radio seguro alrededor de `c_o`).
- **Yaw:** `ψ_m` en cada vista; entre vistas, interpolación suave (por el camino angular más corto) que respete `ψ̇_max`. Matrice: también la inclinación del gimbal por vista; Bebop: inclinación de cámara fija.
- **Cuadros:** muestrear la ruta a `r_c` cuadros por segundo → lista de `Cuadro`.
- Funciones adicionales: `espiral_estandar(escenario, vueltas, ...)` (línea base `x_std`, de abajo hacia arriba) y `nbv_voraz(candidatas, k)` (las k de mayor ganancia, ordenadas por vecino más cercano desde `p_0`; línea base `x_nbv`).

| Variable | Bebop | Matrice |
|---|---|---|
| κ_m | 0–1 | 0–1 |
| ρ_m | 0.4–1.2 m | 5–30 m |
| v_c | 0.1–0.5 m/s | 2–10 m/s |
| λ | 0–1 | 0–1 |

- **Listo cuando:** gráfica 3D de la espiral, de una ruta al azar y de la NBV voraz; pruebas: la ruta pasa exactamente por cada `q_m`, la velocidad en las vistas es `λ·v_c`, `v` y `a` son continuas, y con `κ` todas ≤ 0.5 la ruta es despegue → aterrizaje sin vistas.

### Paso 11 — `evaluar.py`: la función evaluate 🤖 (14 – 18 dic)

`evaluate(x, contexto) → (F, G)` con:

- `F = (f_E/E_ref, f_P/f_P⁰, −f_C)`. `E_ref` = energía de volar la espiral estándar como segundo vuelo; `f_P⁰` = precisión que dejó el primer vuelo.
- `G = (g_1, …, g_11)`, todas de la forma `g_k ≤ 0`, con el peor caso sobre las muestras:

| | Restricción | Forma |
|---|---|---|
| g1 | espacio permitido | posición dentro de la geocerca y de `[z_min, z_max]` |
| g2 | distancia de seguridad | `d_safe − min_t dist(p(t), ocupados) ≤ 0` |
| g3 | inclinación | `max_t arccos(n̂·e_z) ≤ θ_max` |
| g4 | empuje | `max_t T ≤ T_max` |
| g5 | descenso | `max_t (−v_z) ≤ v_desc` |
| g6 | suavidad | `max ‖a‖ ≤ a_max` y `max ‖ψ̇‖ ≤ ψ̇_max` (0.15 rad/s en el Bebop) |
| g7 | desenfoque | `max_f t_exp·f_x·(‖v_t‖/d_f + ‖ψ̇‖) ≤ b_max` (v_t: velocidad perpendicular a la línea de vista) |
| g8 | traslape | `min_f |V_f ∩ V_f+1| / |V_f| ≥ o_min` |
| g9 | registrabilidad | en cada ventana de W cuadros, algún cuadro ve `≥ n_reg` puntos firmes |
| g10 | localizabilidad | tiempo máximo con `L_plat < L_min` `≤ t_gap` |
| g11 | batería | `f_E ≤ (SoC_0 − SoC_res)/100 · E_nom` |

- `L_plat` del Bebop: número de ArUco del mapa (`bebop/control_bebop/config/aruco_map.yaml`) dentro del cuadro y a distancia útil; `L_min = 2` (por `min_markers: 2`), `t_gap = 4 s`. Matrice: calidad de GPS (en simulación, siempre buena salvo zonas marcadas) o número de características visibles junto a la fachada.
- Violación total para comparar infactibles: `CV = Σ max(0, g_k)/s_k`, con una escala `s_k` por restricción.
- Debe poder cambiar de dron (Bebop/Matrice) y de modelo de energía (físico/distancia) por parámetro.
- **Listo cuando:** se evalúan `x_std` y `x_nbv` y dan números razonables (se espera que la NBV gaste menos y complete más); se reporta el tiempo por evaluación (objetivo: menos de ~0.2 s para que 20,000 evaluaciones quepan en una hora; si no, optimizar el paso 8).

**Con el paso 11, OE1 queda cerrado en código.**

### Paso 12 — NSGA-II con jMetalPy 🤖 (enero, después de vacaciones)

- `problema_jmetal.py`: una clase de problema de jMetalPy con 52 variables reales acotadas, 3 objetivos y 11 restricciones que llama a `evaluate`.
- Comparación con restricciones de Deb: factible gana a infactible; entre infactibles gana menor `CV`; entre factibles, dominancia de Pareto. Confirmar que el algoritmo de jMetalPy usado la aplica.
- **Siembra:** parte de la población inicial con NBV voraz de 5, 10 y 15 vistas, cada una con `λ = 0` y `λ = 1` y `ρ` a la mitad de su rango; el resto al azar.
- Variables fuera de rango: recortar al límite.
- **Listo cuando:** una corrida de NSGA-II termina, guarda el frente en un archivo y se grafica en 3D.

### Paso 13 — MOEA/D, MOPSO y búsqueda al azar 🤖 (enero)

- MOEA/D de jMetalPy (normaliza objetivos; pesos sobre los 3 objetivos).
- MOPSO: usar la variante disponible en jMetalPy (SMPSO u OMOPSO; verificar cuál y documentarlo).
- Búsqueda al azar con el mismo número de evaluaciones.
- Mismo presupuesto para todos (por ejemplo 20,000 evaluaciones) y misma siembra donde aplique.
- **Listo cuando:** los cuatro corren con la misma interfaz desde `experimentos.py`.

### Paso 14 — Comparación estadística 🤖 (enero – febrero)

- 20 a 30 semillas por algoritmo.
- **HV** con punto de referencia `(1.1, 1.1, 1.1)` sobre `(f_E/E_ref, f_P/f_P⁰, 1 − f_C)`.
- **IGD+** contra `Z` = unión de los no dominados de todas las corridas de todos los algoritmos.
- Wilcoxon al 5 % entre cada par. Tiempo de cómputo por algoritmo.
- **Elección de `x*`:** `argmin f_E` sobre el frente sujeto a `f_P ≤ f_P(x_std)` y `f_C ≥ f_C(x_std)`; si ninguna cumple, el punto rodilla (el más lejano al plano que une los extremos).
- **Listo cuando:** tabla de HV/IGD+ con medias, desviaciones y p-values, y gráficas de los frentes.

### Paso 15 — Que el dron siga la ruta 🤖 (OE3, febrero)

- `exportar.py`: ruta → JSON con `t, x, y, z, yaw` a 10 Hz en el marco del corral.
- En `bebop/control_bebop`: agregar al `trajectory_planner` un modo que lea ese archivo y publique `/goal` y `/goal_vel` (hoy solo hace órbita). Debe pasar por la misma máquina de estados y geocerca.
- **Listo cuando:** la ruta se sigue en Gazebo (Bebop) con error de seguimiento reportado, y en `preview_trajectory.launch.py` sin simulador.

### Paso 16 — Matrice en simulación 🤖 (OE3, febrero – marzo)

- Parámetros del Matrice en `parametros.py` ya calibrados (paso 7), escenario del edificio en `matrice100/drone_gazebo`.
- Primer vuelo simulado (órbita), reconstrucción con COLMAP, optimización con los parámetros del Matrice, segundo vuelo simulado.
- **Listo cuando:** las rutas óptimas del Bebop y del Matrice se muestran lado a lado y se reporta `λ` y `v_c` de cada una (se espera Bebop con `λ` bajo y Matrice con `λ` alto y `v_c` cerca de su velocidad de mínima potencia; comprobarlo, no asumirlo).

### Paso 17 — Predecir y comprobar 👤🤖 (OE4, marzo – abril)

- 👤 Referencia del objeto con fotos de celular, reconstruida en COLMAP y escalada con medidas de cinta (D6).
- 👤 Volar `x_std`, `x_nbv` y 3 o 4 puntos del frente (menos energía, rodilla, más calidad), 3 veces cada uno.
- 🤖 Por vuelo: energía medida (batería) contra `Ê`; completitud real (fracción de la referencia a menos de `τ` del modelo) contra `Ĉ`; error real (percentil 95 de la distancia a la referencia) contra `σ̂_95`.
- Reportar error relativo de energía y correlación de Spearman para la calidad (`σ̂` es incertidumbre, no error: lo que se defiende es que ordena bien las rutas).
- **Listo cuando:** figura «predicho contra medido», un punto por vuelo, con barras de incertidumbre de la medida.

### Paso 18 — Ablación del modelo de energía 🤖 (cuando estén los pasos 12 y 16)

- Optimizar con `energia_distancia` y evaluar las rutas resultantes con el modelo físico, en los dos drones.
- **Listo cuando:** se reporta cuánta energía de más gastan las rutas del modelo ingenuo. No necesita vuelos.

---

## 4. Valores por definir

| Símbolo | Qué es | Bebop | Matrice | Cómo fijarlo |
|---|---|---|---|---|
| r_c | cuadros por segundo | 1–2 fps | 0.5–1 fps | probar traslape con COLMAP |
| M | vistas candidatas | 25 | 25–40 | |
| d_min, d_max | rango de distancia (y de ρ_m) | 0.4–1.2 m | 5–30 m | enfoque y GSD |
| d_safe | distancia de seguridad | ≥ 0.3 m | ≥ 3 m | el 23 oct la órbita pasó a 15–25 cm de la caja |
| α_max | ángulo de vista útil | 60° | 60° | literatura MVS |
| γ | exponente del peso | 1–2 | 1–2 | sensibilidad |
| σ_px | ruido de reproyección | 1 px | 1 px | error de reproyección de COLMAP |
| n_min | vistas mínimas por vóxel | 3 | 3 | |
| β_min | ángulo de triangulación | 5–10° | 5–10° | sensibilidad |
| o_min | traslape | 0.7 | 0.7 | |
| n_reg | puntos firmes para registrar | ≈ 50 | ≈ 50 | inliers mínimos en tus reconstrucciones (paso 5) |
| W | ventana de registrabilidad | 3 s | 3 s | |
| b_max | desenfoque máximo | 1 px | 1 px | |
| t_exp | tiempo de exposición | por medir | por medir | metadatos o estimación |
| vóxel | tamaño de vóxel | 2–3 cm | 20–30 cm | |
| θ_max | inclinación máxima | `max_tilt` configurado | 35° | |
| T_max | empuje máximo | ≈ 2·m·g (confirmar) | por confirmar | |
| a_max | aceleración máxima | 1 m/s² | 2 m/s² | |
| v_desc | descenso máximo | 0.3 m/s | 1.5 m/s | |
| SoC_res | reserva de batería | 25 % | 25–30 % | |
| τ | tolerancia de completitud real | 1–2 cm | 10–20 cm | |

Cuando se fije un valor, ponerlo en `parametros.py` con un comentario de dónde salió.

## 5. Fuera de alcance (no implementar salvo que se decida)

- Replanear durante el vuelo (en línea): trabajo futuro, o al final solo en Gazebo.
- NeRF / Gaussian Splatting.
- Volar el Matrice real (solo si sobra tiempo).
- El modelo de motores de `bebop/control_bebop/control_bebop/consumo_energ.py`: se reemplaza por `optimizador/energia.py`; no seguir desarrollándolo.

## Registro de avance

- [ ] Paso 1 — Trabajo local subido a GitHub
- [x] Paso 2 — Batería en los bags (25 nov: `/bebop/battery` probado con el dron en el piso)
- [ ] Paso 3 — `optimizador/` y `parametros.py`
- [ ] Paso 4 — `energia.py`
- [ ] Paso 5 — Corral preparado (+ primer n_reg)
- [ ] Paso 6 — `modelo_previo.py`
- [ ] Paso 7 — Vuelo de calibración y `calibracion.py`
- [ ] Paso 8 — `observacion.py`
- [ ] Paso 9 — `candidatas.py`
- [ ] Paso 10 — `trayectoria.py`
- [ ] Paso 11 — `evaluar.py` (cierra OE1)
- [ ] Paso 12 — NSGA-II
- [ ] Paso 13 — MOEA/D, MOPSO y búsqueda al azar
- [ ] Paso 14 — Comparación estadística (cierra OE2)
- [ ] Paso 15 — El dron sigue la ruta
- [ ] Paso 16 — Matrice en simulación (cierra OE3)
- [ ] Paso 17 — Predecir y comprobar (cierra OE4)
- [ ] Paso 18 — Ablación del modelo de energía
