# REPORT.md

Guía para la redacción del informe final del TP y de qué evidencia experimental conviene recolectar durante la implementación de cada módulo para poder armarlo después sin rehacer trabajo.

Basado en la sección "Presentación del trabajo" de `TP Final - Robot Omnidireccional.pdf` (página 8) y los requerimientos específicos de cada módulo (páginas 5-7).

> **El informe es evaluado y debe desarrollar los métodos en profundidad.** Cada módulo exige experimentos con resultados, gráficos y análisis — no alcanza con describir la implementación. Capturar la evidencia mientras se implementa cada módulo (rosbags, gráficos, screenshots de RViz) evita tener que reproducir corridas al final.

## Estructura sugerida del informe

El spec recomienda esta organización (página 8). Conviene mantenerla 1:1:

1. Introducción
2. Modelo Cinemático
3. Control a lazo cerrado y seguimiento de trayectorias
4. Localización basada en EKF
5. Seguimiento de trayectorias a lazo cerrado utilizando localización basada en EKF
6. Sistema desarrollado
7. Conclusiones

---

## 1. Introducción

**Qué incluir**
- Qué es un robot omnidireccional / holonómico, por qué importa.
- Motivaciones y aplicaciones (transporte de carga, maniobrabilidad sin reorientación previa).
- Enfoque propuesto: simulación en CoppeliaSim + ROS 2, cinemática Mecanum, control proporcional a lazo cerrado, localización por EKF con landmarks.

**Qué guardar durante la implementación**
- Nada experimental. Se escribe al final.
- Mantener `docs/chronicle.md` actualizado con hitos del proyecto para enmarcar decisiones de diseño cuando se redacte la introducción.

---

## 2. Modelo Cinemático

**Qué incluir en el informe** (spec p. 8)
- Particularidades de la plataforma Mecanum: configuración 4-ruedas, rodillos a 45°, libertades de movimiento (3-DoF en el plano).
- Motivación para resolver un modelo cinemático específico.
- Tabla de parámetros usados: `r = 0.05 m`, `lx = ly = 0.175 m`, `N = 500 ticks/rev`. Referenciar al paper (eqs. 19-26, tabla página 8).
- Metodología de la estimación odométrica: paso de Δticks por rueda → Δpose en el frame `odom` (eq. 21 + integración con rotación al frame mundial).
- **Experimento 1 — Cinemática inversa (actuación)**: gráfico de consignas `(vx, vy, ωz)` enviadas por `/robot/cmd_vel` vs velocidades reales ejercidas por el robot según el simulador (ground-truth derivado de `odom → base_link_gt`).
- **Experimento 2 — Cinemática directa (odometría)**: gráfico de pose `(x, y, θ)` estimada por odometría vs pose real (ground-truth) a lo largo del tiempo. Mostrar también la trayectoria en el plano `x-y`.

**Qué guardar durante la implementación**
- **Rosbag por experimento** con: `/robot/cmd_vel`, `/robot/odometry`, `/tf` (incluye `map → odom` y `odom → base_link_gt`), `/robot/encoders` (debugging por si las cuentas no cierran).
- **Trayectorias de prueba** diseñadas para ejercitar cada DOF por separado y luego en combinación:
  1. Avance puro (`vx ≠ 0`) — valida la suma `ω₁+ω₂+ω₃+ω₄`.
  2. Strafing puro (`vy ≠ 0`) — valida la fila 2 de eq. 21. **Si esto falla, hay un bug en la cinemática directa** (caso típico: el código antiguo `pioneer_odometry.cpp` integraba con cinemática diferencial y descartaba `v_y`; ahora se corrige en `mecanum_odometry.cpp`).
  3. Rotación pura (`ωz ≠ 0`) — valida el denominador `4·(lx+ly)`.
  4. Trayectoria combinada (diagonal + rotación) — caso realista.
- Anotar parámetros y comando exacto usado para cada corrida (junto al bag).

**Tooling**
- `ros2 bag record -o experimentos/cinematica/<caso>/bag /robot/cmd_vel /robot/odometry /tf /robot/encoders`
- `plotjuggler` o `rqt_plot` para inspección rápida.
- Script Python con `rosbag2_py` + `matplotlib` para los gráficos del informe — versionarlo en `scripts/` así los gráficos son reproducibles.

---

## 3. Control a lazo cerrado y seguimiento de trayectoria

**Qué incluir en el informe** (spec p. 8)
- Diagrama de bloques del sistema de control a lazo cerrado (planta + sensor + controlador P + setpoint).
- Descripción del controlador proporcional por DOF: ecuación, qué entra como feedback (transformación `map → base_link`), qué sale (`/robot/cmd_vel`).
- Estrategia pursuit-based: cómo se define la trayectoria cuadrada de 2 m con orientación "opuesta al centro", cómo se interpolan los waypoints, criterio de "lo suficientemente cerca" para avanzar al siguiente.
- **Experimentos con distintos valores de Kp**:
  - Variar Kp lineal y Kp angular **por separado**.
  - Gráficos comparando setpoint vs pose real para cada valor.
  - Velocidades lineales y angular asignadas en cada momento durante el experimento.
- Análisis: oscilaciones, overshoot, error en estado estacionario, tiempo de convergencia.
- Justificación de los valores finales de Kp.

**Qué guardar durante la implementación**
- **Rosbag por cada valor de Kp probado** con: `/robot/cmd_vel`, `/robot/odometry`, `/tf`, y un tópico propio con el waypoint actual (recomendado publicar uno tipo `/control/current_setpoint` para facilitar el plotting posterior).
- **Tabla de configuraciones probadas** (markdown o CSV en `experimentos/control_p/`):
  - Una fila por corrida: `(Kp_lineal, Kp_angular, ruta del bag, observación cualitativa)`.
- Capturar al menos **3 valores de cada constante** (bajo / medio / alto) para mostrar sensibilidad sin combinar variables.

**Tooling**
- Exponer Kp como `ros2 param` así se pueden cambiar entre corridas sin recompilar.
- Script de graficado que superponga setpoint y pose real en `x-y` (vista cenital) más cada componente `(x, y, θ)` en función del tiempo.

---

## 4. Localización basada en EKF

**Qué incluir en el informe** (spec p. 8)
- **Modelos del filtro detallados**:
  - Estado `x⃗` (típicamente `[x, y, θ]ᵀ` en el frame `map`).
  - Entradas de control `u⃗` — 3-DoF para el omni: `[vx, vy, ωz]ᵀ` (de ahí `EKFilter(3, 3, 3, 2, 2)` del anexo, no la versión diferencial).
  - Mediciones `z⃗`: `[range, bearing]ᵀ` por landmark detectado.
  - Ruido del actuador `w⃗`, ruido del sensor `v⃗`.
  - Modelo de movimiento `f(x⃗, u⃗, w⃗)` y modelo de sensado `h(x⃗, v⃗)`.
  - Jacobianos correspondientes.
- Matrices de covarianza iniciales (Q, R, P) y justificación: la covarianza de predicción debe reflejar **mayor incertidumbre** que la de corrección — el spec espera que el sensor sea "más confiable".
- Detección de landmarks por LiDAR: método de clusterización para detectar postes Ø 0.1 m en `/robot/front_laser/scan`. Decisiones (umbral de distancia, mínimo de puntos por cluster, etc.).
- Asociación de datos: cómo se matchea cada landmark detectado (en frame del robot) con un poste del mapa conocido (`/posts`, frame `map`).
- **Experimentos de precisión**:
  - Robot movido manualmente con velocidades constantes (no usar el controlador de §3, para aislar la calidad de la localización).
  - Pose en el tiempo comparada contra ground-truth.
  - **Comparación clave**: solo odometría vs EKF. El EKF debería mostrar menos deriva acumulada, sobre todo en trayectorias largas.

**Qué guardar durante la implementación**
- **Rosbag de cada experimento manual** con: `/robot/cmd_vel`, `/robot/odometry`, `/robot/front_laser/scan` (para poder reproducir la clusterización offline), `/landmarks`, `/posts`, `/tf` (incluye `odom → base_link_gt` y nuestro `map → base_link_ekf`).
- **Trayectorias de prueba**: 2-3 trayectorias largas (líneas, vueltas, zig-zag) donde la deriva odométrica sea visible, para que el EKF tenga oportunidad de demostrar su valor.
- **Caso "pocos landmarks visibles"**: al menos una trayectoria donde el robot pase un tramo sin ver postes, para mostrar cómo crece la covarianza durante la predicción sin corrección.
- Guardar los valores iniciales de Q, R, P junto al bag; cualquier reajuste se registra en `docs/chronicle.md` con fecha y motivo.

**Tooling**
- RViz2 (config `coppeliaSim/tpfinal.rviz`) para visualizar landmarks detectados, mapa, y el elipsoide de covarianza.
- Script Python que extraiga `(x, y, θ)` de los tres frames — `base_link` por odometría, `base_link_ekf` por EKF, `base_link_gt` ground-truth — y los grafique en simultáneo.

---

## 5. Seguimiento de trayectoria a lazo cerrado con EKF

**Qué incluir en el informe** (spec p. 8)
- Cómo se conecta el controlador de §3 con la pose refinada del EKF: sustituir `map → base_link` por `map → base_link_ekf` como feedback.
- Reajuste eventual de Kp con el nuevo feedback — si fue necesario, mostrar experimentos antes/después.
- **Comparación trayectoria realizada vs trayectoria requerida** contra ground-truth.
- **Imágenes de RViz2 con elipsoide de covarianza** en distintas porciones del recorrido cuadrado (spec lo pide explícitamente).

**Qué guardar durante la implementación**
- **Rosbag del recorrido cuadrado completo** con todo lo de las secciones anteriores + el setpoint actual.
- **Screenshots de RViz2** en al menos 4-6 momentos del recorrido:
  - Al inicio (pose estimada bien conocida).
  - En cada esquina del cuadrado.
  - En un tramo intermedio con muchos landmarks visibles (covarianza pequeña).
  - En un tramo intermedio con pocos / sin landmarks (covarianza expandida).
- Guardar la config de RViz usada para los screenshots — los screenshots tienen que ser reproducibles.

**Tooling**
- Habilitar el display "Covariance" de RViz2 sobre un topic `nav_msgs/Odometry` con la covarianza poblada por el EKF (publicar uno tipo `/robot/odometry_ekf`).

---

## 6. Sistema desarrollado

**Qué incluir en el informe**
- Lista de paquetes ROS 2 implementados y para qué sirve cada uno.
- Diagrama de nodos / topics (exportado desde `rqt_graph`).
- Documentación de interacciones entre tópicos y mensajes.
- Instrucciones reproducibles para correr el sistema completo (qué terminal abre qué, en qué orden).

**Qué guardar durante la implementación**
- Exportar `rqt_graph` cada vez que se agrega un nodo nuevo, guardar PNG en `docs/architecture/`.
- README mínimo por paquete con cómo correrlo.
- Actualizar `docs/chronicle.md` con decisiones de diseño no obvias (ej: por qué un nodo es de Python y otro de C++, por qué se eligió tal frecuencia de publicación).

---

## 7. Conclusiones

**Qué incluir**
- Conclusiones extraídas de los experimentos y resultados.
- Problemas encontrados durante el desarrollo y soluciones aplicadas.
- Mejoras potenciales (EKF con ruido adaptativo, controlador PID, fusión con IMU, A* para planning).

**Qué guardar durante la implementación**
- Mantener una sección "problemas y soluciones" en `docs/chronicle.md` a medida que se va trabajando — al final se destila a las conclusiones sin tener que ejercitar la memoria.

---

## Convenciones recomendadas para la evidencia experimental

- **Ubicación de bags**: `experimentos/<modulo>/<caso>/bag/`
- **Configuración de cada corrida**: junto al bag, un `README.md` con parámetros usados, comando exacto, fecha, observaciones cualitativas.
- **Scripts de graficado**: versionados en `scripts/`. Los gráficos del informe deben poder regenerarse desde el bag con un solo comando.
- **Screenshots de RViz**: `experimentos/<modulo>/<caso>/screenshots/` con nombres descriptivos (`esquina_NE.png`, `sin_landmarks.png`, `kp_alto_oscila.png`).
- **Ground-truth**: siempre disponible vía `odom → base_link_gt`. Para ground-truth en frame `map`, componer la cadena `map → odom → base_link_gt` desde `/tf`. No usar `base_link_gt` como feedback de control — es solo para depuración y comparación.
- **Antes de borrar un bag o un experimento**: anotar en `docs/chronicle.md` por qué se descartó. Es habitual querer volver a una corrida vieja semanas después.
