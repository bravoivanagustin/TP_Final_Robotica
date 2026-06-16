# Decisiones Arquitecturales — Módulo 3 EKF

Fecha de sesión: 2026-06-15

---

## 1. Nombre del paquete: ekf_localizer (no robmovil_ekf)

**Decisión: el paquete propio se llama `ekf_localizer`.**

`robmovil_ekf` es el nombre del paquete de la cátedra que provee la biblioteca EKFilter. Registrar un paquete propio con ese mismo nombre provoca colisión en `colcon build`: CMake y `ament` no pueden distinguir cuál de los dos instalar, lo que resulta en errores de enlace no deterministas según el orden de descubrimiento de paquetes.

Alternativa descartada: usar `robmovil_ekf` como nombre del paquete propio → descartada porque rompe el build en presencia del paquete de la cátedra.

---

## 2. Implementación EKF: Eigen puro (sin kfilter)

**Decisión: el filtro de Kalman extendido se implementa directamente con matrices Eigen, sin usar la biblioteca kfilter.**

La consigna no exige ninguna biblioteca específica de EKF. La API de kfilter es desconocida (los headers no están disponibles en el repositorio) y requeriría heredar de una clase base con métodos virtuales de firma incierta, añadiendo una dependencia opaca. Con Eigen puro el código mapea directamente a las ecuaciones del informe (predict/update en forma matricial explícita), es autocontenido en el repo y tiene cero dependencias misteriosas.

Alternativa descartada: kfilter (EKFilter de la cátedra) → descartada porque la API no está documentada localmente y añade una dependencia cuya versión y comportamiento no pueden verificarse.

---

## 3. Entrada de control u: deltas de odometría en frame body

**Decisión: el vector de control es `u = [dxb, dyb, dth]` — deltas calculados en frame body, no velocidades escaladas por dt.**

`mecanum_odometry.cpp` ya integra los encoders y produce estos deltas en las líneas 103-110. Usarlos directamente como entrada al modelo de proceso `f(x, u)` es coherente con la derivación del Jacobiano `F`: las dimensiones son metros y radianes en cada paso, lo que evita errores de escala por `dt²` que aparecerían si se usaran velocidades y luego se multiplicaran nuevamente por dt dentro del EKF.

Alternativa descartada: usar velocidades `[vx, vy, omega]` multiplicadas por dt → descartada porque requiere un paso de conversión adicional y es más proclive a errores cuando dt varía bajo `use_sim_time`.

---

## 4. Mapa de postes: suscripción a /posts con QoS transient_local

**Decisión: las coordenadas de los 16 postes se obtienen suscribiéndose al tópico `/posts` (`geometry_msgs/PoseArray`, QoS `transient_local`), no de un archivo YAML hardcodeado.**

CoppeliaSim publica las posiciones reales de los postes en frame `map` a través de `/posts` con QoS `transient_local`, lo que garantiza que un nodo que se suscribe tarde recibe igualmente el mensaje. Usar este tópico asegura que las coordenadas del mapa del EKF coinciden exactamente con las de la simulación en todo momento.

Alternativa descartada: YAML hardcodeado con coordenadas de los postes → descartada porque puede quedar desactualizado si la escena de CoppeliaSim cambia, y requiere sincronización manual entre el archivo y la simulación.

---

## 5. Inicialización del estado EKF: via TF map→base_link

**Decisión: el estado inicial `[x, y, theta]` del EKF se obtiene realizando un `lookupTransform("map", "base_link", ...)` en el primer ciclo de predicción.**

`/robot/odometry` tiene `frame_id = "odom"` y su pose está en coordenadas del frame `odom`. Copiar esa pose directamente al estado EKF (que vive en frame `map`) solo sería correcto bajo la suposición `map ≡ odom` en `t = 0`, suposición que no está garantizada. Inicializar via la TF `map→base_link` es la forma canónica en ROS 2 y funciona correctamente para cualquier configuración de frames.

Alternativa descartada: leer `pose.pose` de `/robot/odometry` → descartada porque asume implícitamente que `odom` y `map` coinciden, lo que puede no ser cierto y produce un sesgo inicial silencioso en el estado EKF.

---

## 6. lookupTransform para TF estática (front_laser→base_link): tf2::TimePointZero

**Decisión: la transformación mecánica `front_laser→base_link` se consulta con `tf2::TimePointZero` (última disponible), no con el timestamp exacto del scan.**

`front_laser→base_link` es una TF estática que no cambia con el tiempo. Pedirla con el timestamp exacto del mensaje de scan puede lanzar `ExtrapolationException` al arranque si el TF buffer aún no tiene datos suficientes. Usar `TimePointZero` es más robusto y semánticamente correcto para transformaciones mecánicas fijas.

Alternativa descartada: usar el `stamp` del `LaserScan` en el `lookupTransform` → descartada porque puede fallar con `ExtrapolationException` durante los primeros ciclos cuando el buffer de TF no está completamente poblado.

---

## 7. Relación Q >> R: mayor incertidumbre en predicción que en medición

**Decisión: `sigma_dx = sigma_dy = 0.15 m`, `sigma_dth = 0.15 rad` (ruido de proceso Q) frente a `sigma_range = 0.05 m`, `sigma_bearing = 0.03 rad` (ruido de medición R).**

La consigna exige "mayor incertidumbre al momento de predecir". Con estos valores `Q[0,0] / R[0,0] ≈ 9x` y `Q[1,1] / R[1,1] ≈ 25x`. El robot Mecanum desliza lateralmente en condiciones reales (alta incertidumbre en `dyb`), mientras que la detección de postes cilíndricos mediante LiDAR es un sensor confiable con ruido bajo. La asimetría es físicamente justificada y no arbitraria.

Alternativa descartada: Q ≈ R o Q < R → descartada porque viola la premisa de la consigna y causaría que el EKF confíe más en el modelo cinemático que en el sensor, lo que degrada la localización cuando hay deslizamiento.

---

## 8. Escalado de Q por dt en cada paso de predicción

**Decisión: la matriz de covarianza de proceso Q se escala por `dt` en cada llamada a `predict()`, donde `dt` es el intervalo real entre mensajes de `/robot/odometry`.**

Bajo `use_sim_time = true` el intervalo entre mensajes no es constante. Una Q estática acumularía incertidumbre a una tasa incorrecta cuando `dt` varía, subestimando la covarianza en pasos cortos y sobreestimándola en pasos largos. Escalar `Q_efectiva = Q_nominal * dt` garantiza que la covarianza predicha crece proporcionalmente al tiempo real transcurrido en cada paso.

Alternativa descartada: Q fija sin escalar → descartada porque la covarianza acumulada dependería de la frecuencia de publicación del nodo de odometría, que puede variar, y no reflejaría la física del proceso correctamente.
