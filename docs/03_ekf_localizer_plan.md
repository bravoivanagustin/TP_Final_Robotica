# Módulo 3: Localización por EKF — Briefing de implementación

## 1. Visión general del módulo 3

El módulo 3 (`ekf_localizer`) refina la estimación de pose del robot fusionando la odometría del módulo 1 (que deriva) con observaciones de postes conocidos del mapa. La odometría sola acumula error (rozamiento, deslizamiento de las ruedas Mecanum invalidan el modelo cinemático); las referencias del entorno (`/posts`, 16 postes Ø 0.1 m en frame `map`) aportan información absoluta que corrige esa deriva.

El módulo se monta **encima** de los módulos 1 y 2, no los reemplaza. El módulo 1 sigue publicando `odom → base_link`; el módulo 3 agrega una TF paralela `map → base_link_ekf`. El módulo 2 se reconecta cambiando su param `base_frame:=base_link_ekf`, sin recompilar.

Dos nodos:
- **`landmark_detector`** — convierte `/robot/front_laser/scan` en `/landmarks` (range, bearing en frame `base_link`) por clusterización.
- **`ekf_node`** — corre el EKF: predice con `/robot/odometry` como control, corrige con `/landmarks` matcheados contra `/posts`, publica `map → base_link_ekf` y `/robot/odometry_ekf` (con covarianza poblada para RViz).

### Diagrama de flujo de datos

```
                      CoppeliaSim (omni_ekf.ttt)
   ┌──────────────────────────────────────────────────────────────┐
   │  /robot/front_laser/scan   /posts (PoseArray, map, 16)         │
   │  /robot/encoders           map→odom, map→odom→base_link_gt     │
   └───────┬───────────────────────┬──────────────────────┬────────┘
           │                        │                      │
           ▼                        │                      │
  ┌──────────────────┐             │                      │
  │ modelo_holonomico│  /robot/odometry + TF odom→base_link
  │ (MÓDULO 1)       │─────────────┼───────┐              │
  └──────────────────┘             │       │              │
                                   │       ▼              ▼
  ┌─────────────────────┐         │   ┌──────────────────────────┐
  │ landmark_detector    │◄─scan──┘   │        ekf_node           │
  │ (MÓDULO 3a)          │            │       (MÓDULO 3b)         │
  │                      │ /landmarks │  pred: u=/robot/odometry  │
  │  scan→clusters→r,β   ├───────────►│  corr: z=/landmarks       │
  └─────────────────────┘  (base_link)│        mapa=/posts        │
                                       └────┬──────────────┬───────┘
                                            │              │
                          TF map→base_link_ekf      /robot/odometry_ekf
                                            │              │ (cov poblada)
                                            ▼              ▼
                          ┌──────────────────────┐   ┌─────────┐
                          │ lazo_cerrado (MÓD 2) │   │ RViz2   │
                          │ base_frame:=          │   │ ellipse │
                          │   base_link_ekf       │   └─────────┘
                          │ → /robot/cmd_vel      │
                          └──────────────────────┘
```

TF tree resultante: `map → odom → base_link → front_laser` (módulos 0/1) + `map → base_link_ekf` (módulo 3) + `map → odom → base_link_gt` (sim, ground-truth).

---

## 2. Estructura de paquetes y archivos

Paquete único nuevo: **`ekf_localizer`** (sigue la convención `<dominio>` del proyecto, snake_case). Contiene los dos nodos. Estructura estándar ROS 2 como `lazo_cerrado/`.

```
ekf_localizer/
├── CMakeLists.txt
├── package.xml
├── include/ekf_localizer/
│   ├── LandmarkDetector.h
│   └── EkfLocalizer.h
├── src/
│   ├── LandmarkDetector.cpp
│   ├── landmark_detector_node.cpp
│   ├── EkfLocalizer.cpp
│   └── ekf_localizer_node.cpp
└── launch/
    ├── ekf_localizer.launch.py        # solo módulo 3 (detector + ekf), para experimentos manuales
    └── seguimiento_ekf.launch.py      # sistema completo (módulos 1+2+3) para el requerimiento 4
```

### Rol de cada archivo

| Archivo | Rol |
|---|---|
| `include/ekf_localizer/LandmarkDetector.h` | Declaración de `robmovil::LandmarkDetector : rclcpp::Node`. `#pragma once`, includes mínimos, bloque `//` describiendo responsabilidad y frames. |
| `src/LandmarkDetector.cpp` | Implementación del detector: callback de scan, clustering, range/bearing, publicación. |
| `src/landmark_detector_node.cpp` | `main()` mínimo: `init → spin(make_shared<robmovil::LandmarkDetector>()) → shutdown`. |
| `include/ekf_localizer/EkfLocalizer.h` | Declaración de `robmovil::EkfLocalizer : rclcpp::Node`. Estado, matrices, miembros TF, callbacks. |
| `src/EkfLocalizer.cpp` | Predicción, corrección, asociación de datos, publicación de TF y odometry_ekf. |
| `src/ekf_localizer_node.cpp` | `main()` mínimo. |
| `launch/*.launch.py` | Lanzamiento con `use_sim_time:=True`. |

Nota: el nombre del frame `base_link_ekf` exige que cada landmark detectado venga en `base_link` (frame del robot, no del laser). Decisión clave de arquitectura: **el detector NO asigna IDs** — entrega clusters genéricos en (range, bearing); la asociación de datos vive enteramente en `ekf_node` (que tiene el estado y el mapa). Esto sigue la separación del Taller 9 y mantiene el detector stateless respecto del mapa.

### CMakeLists.txt (contenido esperado)

Patrón de `lazo_cerrado`:

```cmake
cmake_minimum_required(VERSION 3.5)
project(ekf_localizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)        # LaserScan, PointCloud
find_package(nav_msgs REQUIRED)            # Odometry
find_package(robmovil_msgs REQUIRED)       # Landmark, LandmarkArray
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)              # matrices del EKF

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(landmark_detector src/landmark_detector_node.cpp src/LandmarkDetector.cpp)
ament_target_dependencies(landmark_detector
  rclcpp std_msgs geometry_msgs sensor_msgs robmovil_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(ekf_localizer src/ekf_localizer_node.cpp src/EkfLocalizer.cpp)
ament_target_dependencies(ekf_localizer
  rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs robmovil_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(ekf_localizer Eigen3::Eigen)

install(TARGETS landmark_detector ekf_localizer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include/ DESTINATION include)

ament_package()
```

**Decisión Eigen vs librería `robmovil_ekf` (EKFilter):** ver §4 (nota final). Si se usa `EKFilter`, reemplazar `find_package(Eigen3)` por la dependencia de ese paquete y enlazarla; las dimensiones se fijan en `EKFilter(3,3,3,2,2)`. El plan base asume **Eigen** (más control, sin dependencia del paquete del curso); la variante EKFilter se documenta abajo.

### package.xml (format 3)

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
<depend>robmovil_msgs</depend>
<depend>tf2</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
<depend>eigen3_cmake_module</depend>   <!-- + buildtool_depend eigen3_cmake_module -->
<depend>eigen</depend>
```

---

## 3. Nodo: `landmark_detector`

### Clase y herencia

`namespace robmovil { class LandmarkDetector : public rclcpp::Node }`. Hereda directo de `Node` (tiene estado: subs, pubs, TF, no necesita jerarquía). Constructor `Node("landmark_detector")`. Callback `on_laser_scan` (patrón `on_<topic_shortname>`). Auxiliares en camelCase: `updateLaserTf`, `publishPointcloud`.

### Tópicos

| Dirección | Tópico | Tipo | QoS |
|---|---|---|---|
| sub | `/robot/front_laser/scan` | `sensor_msgs/msg/LaserScan` | `rclcpp::QoS(10)` |
| pub | `/landmarks` | `robmovil_msgs/msg/LandmarkArray` | `rclcpp::QoS(10)` |
| pub | `/landmarks_pointcloud` | `sensor_msgs/msg/PointCloud` | `rclcpp::QoS(10)` |

`/landmarks_pointcloud` (centroides cartesianos, frame `base_link`) es solo para visualización en RViz; tipo legacy `PointCloud` igual que el Taller 9.

### Parámetros ROS 2

| Parámetro | Tipo | Default | Descripción |
|---|---|---|---|
| `robot_frame` | string | `"base_link"` | Frame destino (donde se expresan los landmarks). Origen del lookup. |
| `laser_frame` | string | `"front_laser"` | Frame del laser (¡no `"laser"` como el taller!). |
| `landmark_diameter` | double | `0.1` | Diámetro de poste (m). De aquí sale el umbral de cluster. |
| `cluster_distance_threshold` | double | `0.15` | Distancia euclídea máx entre puntos consecutivos para seguir en el mismo cluster. |
| `min_cluster_points` | int | `2` | Mínimo de puntos para aceptar un cluster (filtra ruido). |
| `max_cluster_points` | int | `30` | Máximo (descarta paredes/objetos grandes). |
| `max_range` | double | `8.0` | Rango máx aceptado (clamp adicional sobre `range_max`). |

Declarar todos con `this->declare_parameter(...)` en el constructor; strings con `std::string("...")` explícito. En el launch solo se sobreescribe `use_sim_time`.

### Lectura del scan y conversión a cartesiano

Para cada `i` con `ranges[i]` en `[range_min, min(range_max, max_range)]`:

```
theta_i = angle_min + i * angle_increment
x_l = ranges[i] * cos(theta_i)
y_l = ranges[i] * sin(theta_i)
```

Punto en frame laser: `tf2::Vector3 p(x_l, y_l, 0.0)`. Transformar a `base_link`:

```
p_robot = laser_transform * p     // laser_transform = TF front_laser → base_link
```

`laser_transform` se obtiene en `updateLaserTf(scan->header.stamp)`:

```cpp
auto tf = tf_buffer_->lookupTransform(robot_frame_, laser_frame_, scan->header.stamp);
tf2::fromMsg(tf.transform, laser_transform_);
```

Dentro de `try/catch (const tf2::TransformException& ex)`, con `RCLCPP_WARN_THROTTLE(*this->get_clock(), 2000, ...)` y `return` (descartar ese scan; reintentar en el próximo). Acumular los `p_robot` válidos en `std::vector<tf2::Vector3> cartesian`.

### Algoritmo de clustering

Clustering secuencial simple (no k-means/DBSCAN). El laser entrega puntos angularmente ordenados, así que puntos de un mismo poste son consecutivos.

```
landmark_points = []
para cada p en cartesian:
    si landmark_points vacío:
        landmark_points.push(p)
    sino:
        d = distancia_euclidea(p, landmark_points.back())
        si d < cluster_distance_threshold:
            landmark_points.push(p)
        sino:                              // fin del cluster actual
            emitir_landmark(landmark_points)
            landmark_points = [p]
al terminar el loop: emitir_landmark(landmark_points)  // último cluster
```

`emitir_landmark(pts)`:
```
si pts.size() < min_cluster_points  o  pts.size() > max_cluster_points: descartar
centroid = Σ pts / pts.size()
range   = sqrt(centroid.x² + centroid.y²)
bearing = atan2(centroid.y, centroid.x)
agregar Landmark{range, bearing} a LandmarkArray
agregar centroid a pointcloud
```

`cluster_distance_threshold` = diámetro + margen ≈ `landmark_diameter + 0.05`. Razón: dos lecturas sobre el mismo poste Ø 0.1 m nunca distan más que el diámetro; un salto mayor indica fin de objeto.

**Condiciones de validez de un landmark:** dentro de `[range_min, max_range]`; tamaño de cluster en `[min_cluster_points, max_cluster_points]`. Opcionalmente verificar que el ancho geométrico del cluster (`||pts.front() - pts.back()||`) sea `≲ 1.5 × landmark_diameter` para rechazar segmentos largos (paredes). Recomendado activarlo si en pruebas aparecen falsos postes.

### Header de salida

`msg.header.stamp = scan->header.stamp` (no `now()`, para no romper TF con `use_sim_time`). `msg.header.frame_id = robot_frame_` (`base_link`).

### Logging

- `RCLCPP_INFO` una sola vez al arrancar: parámetros efectivos (frames, umbral).
- `RCLCPP_WARN_THROTTLE(... 2000 ...)` en el catch de TF: `"TF front_laser→base_link no disponible aún"`.
- `RCLCPP_INFO_THROTTLE(... 2000 ...)` opcional con `n` landmarks detectados por scan, útil para depurar el umbral. Mensajes en español.

---

## 4. Nodo: `ekf_node`

### Clase y herencia

`namespace robmovil { class EkfLocalizer : public rclcpp::Node }`, constructor `Node("ekf_localizer")`. Estado interno (vector, covarianza, baseline de odometría), TF broadcaster + listener.

### Tópicos y TF

| Dirección | Nombre | Tipo | Frame | QoS |
|---|---|---|---|---|
| sub | `/robot/odometry` | `nav_msgs/msg/Odometry` | — | `QoS(10)` |
| sub | `/landmarks` | `robmovil_msgs/msg/LandmarkArray` | `base_link` | `QoS(10)` |
| sub | `/posts` | `geometry_msgs/msg/PoseArray` | `map` | `KeepLast(1).reliable().transient_local()` |
| pub TF | `map → base_link_ekf` | TF en `/tf` | — | broadcaster |
| pub | `/robot/odometry_ekf` | `nav_msgs/msg/Odometry` | header `map`, child `base_link_ekf` | `QoS(10)` |

`/posts` con **TransientLocal**: el simulador puede publicarlo una sola vez (latched) antes de que arranque el EKF. Si no se usa transient_local, el EKF nunca recibe el mapa. Guardar el mapa en `std::vector<std::pair<double,double>> posts_` (mx, my en `map`) en el callback `on_posts`.

### Parámetros ROS 2

| Parámetro | Tipo | Default | Descripción |
|---|---|---|---|
| `map_frame` | string | `"map"` | Frame padre de la TF publicada. |
| `ekf_frame` | string | `"base_link_ekf"` | Frame hijo (pose refinada). |
| `sigma_v` | double | `0.10` | Desvío del ruido de proceso en vx (m/s, por √s integrado). |
| `sigma_vy` | double | `0.10` | Desvío de proceso en vy. |
| `sigma_w` | double | `0.15` | Desvío de proceso en ωz (rad/s). |
| `sigma_range` | double | `0.05` | Desvío de medición de range (m). |
| `sigma_bearing` | double | `0.03` | Desvío de medición de bearing (rad). |
| `init_cov` | double | `0.01` | Diagonal inicial de P. |
| `association_max_dist` | double | `0.6` | Umbral de gating (m) para data association. |
| `min_range` | double | `0.2` | Descartar landmarks demasiado cercanos (numéricamente inestables). |

### Estado, control, medición

```
x⃗ = [x, y, θ]ᵀ        pose en frame map        (n=3)
u⃗ = [vx, vy, ωz]ᵀ     velocidades del cuerpo    (nu=3)  ← de /robot/odometry.twist
z⃗ = [range, bearing]ᵀ por landmark              (m=2)
w⃗ = [wvx, wvy, wω]ᵀ   ruido de actuador         (nw=3)
v⃗ = [vr, vβ]ᵀ         ruido de sensor           (nv=2)
```

La entrada de control sale del `twist.twist.linear.x/y` y `twist.twist.angular.z` de `/robot/odometry` (velocidades en frame del cuerpo, las que produce el módulo 1). El paso de tiempo `Δt` es la diferencia de stamps entre mensajes consecutivos de odometría.

### Modelo de movimiento `f(x⃗, u⃗, w⃗)`

Velocidades en frame del cuerpo rotadas a `map` por `θ` e integradas en `Δt`. Para el cuadrado y velocidades moderadas se integra con el `θ` previo (Euler); el ruido entra aditivo sobre el incremento:

```
x⁺  = x  + (vx·cosθ − vy·sinθ)·Δt + wvx
y⁺  = y  + (vx·sinθ + vy·cosθ)·Δt + wvy
θ⁺  = θ  + ωz·Δt + wω           (normalizar a (−π, π])
```

Variables en el código: `x_`, `y_`, `theta_` (miembros, trailing underscore, igual que `mecanum_odometry`); locales `vx`, `vy`, `wz`, `dt`, `cos_t`, `sin_t`.

### Jacobianos del movimiento

`F = ∂f/∂x` (3×3), evaluado en la estimación previa:

```
        ⎡ 1   0   −(vx·sinθ + vy·cosθ)·Δt ⎤
F  =    ⎢ 0   1    (vx·cosθ − vy·sinθ)·Δt ⎥
        ⎣ 0   0            1              ⎦
```

`W = ∂f/∂w` (3×3, ruido aditivo) = identidad `I₃`:

```
        ⎡ 1 0 0 ⎤
W  =    ⎢ 0 1 0 ⎥
        ⎣ 0 0 1 ⎦
```

### Modelo de sensado `h(x⃗, v⃗)`

Para un poste `j` del mapa en `(mxⱼ, myⱼ)` (frame `map`), la observación esperada desde la pose `(x, y, θ)`:

```
dx = mxⱼ − x
dy = myⱼ − y
q  = dx² + dy²

range_esperado   = sqrt(q)               + vr
bearing_esperado = atan2(dy, dx) − θ     + vβ   (normalizar a (−π, π])
```

El `bearing` es relativo al robot (el detector lo entrega así, frame `base_link`), por eso se resta `θ`.

### Jacobianos de sensado

`H = ∂h/∂x` (2×3), con `r = sqrt(q)`:

```
       ⎡ −dx/r     −dy/r      0  ⎤
H  =   ⎢                          ⎥
       ⎣  dy/q     −dx/q     −1  ⎦
```

`V = ∂h/∂v` (2×2, ruido aditivo) = `I₂`:

```
       ⎡ 1  0 ⎤
V  =   ⎢      ⎥
       ⎣ 0  1 ⎦
```

### Matrices de covarianza iniciales

El spec exige Q (proceso) **mayor** que R (sensor): el sensor es "más confiable" que el modelo cinemático (que viola sus asunciones por deslizamiento Mecanum).

```
P₀ = diag(init_cov, init_cov, init_cov) = diag(0.01, 0.01, 0.01)
       arrancamos relativamente seguros si el robot parte de pose conocida;
       subir a diag(1,1,1) si la pose inicial es incierta.

Q  = diag(σv², σvy², σw²)·Δt = diag(0.10², 0.10², 0.15²)·Δt
       ≈ diag(0.01, 0.01, 0.0225)·Δt   → incertidumbre de predicción ALTA

R  = diag(σrange², σbearing²) = diag(0.05², 0.03²)
       = diag(0.0025, 0.0009)            → incertidumbre de medición BAJA
```

Justificación numérica: con `Δt ≈ 0.05 s` (20 Hz odom), `Q ≈ diag(5e-4, 5e-4, 1.1e-3)` crece sin corrección; `R` es chico y constante, por lo que la corrección "tira" fuerte hacia la observación. Relación de orden Q≫R cumplida en el régimen donde hay postes. Estos son puntos de partida; tunear con los experimentos de §6.

### Asociación de datos

El detector no da IDs. Para cada landmark detectado `(r_obs, β_obs)`:

1. Convertir a punto en `map` usando la pose **predicha** `x̄`:
   ```
   lx = x̄ + r_obs·cos(β_obs + θ̄)
   ly = ȳ + r_obs·sin(β_obs + θ̄)
   ```
2. Buscar el poste `j` de `posts_` que minimice `||(lx,ly) − (mxⱼ,myⱼ)||` (nearest-neighbor).
3. **Gating:** si esa distancia mínima `> association_max_dist` (0.6 m), descartar la observación (probable falso positivo o ambigüedad). Si entra, usar `(mxⱼ, myⱼ)` como landmark del mapa para la corrección.
4. Descartar también si `r_obs < min_range`.

Nearest-neighbor con gating es suficiente: 16 postes separados, pose predicha buena tras los primeros frames. Alternativa Mahalanobis (usando S del filtro) documentada en §7.

### Paso de predicción

Disparado por el callback `on_odometry`. Variables a actualizar: `x_, y_, theta_`, `P_`.

```
on_odometry(msg):
    extraer vx, vy, wz de msg.twist.twist
    dt = stamp(msg) − last_stamp_;  si dt ≤ 0 → guard (refrescar baseline, no integrar); return
    aplicar f(): actualizar x_, y_, theta_  (normalizar theta_)
    construir F y Q(dt)
    P_ = F · P_ · Fᵀ + W · Q · Wᵀ        (W=I ⇒ + Q)
    last_stamp_ = stamp(msg)
    publicar TF y odometry_ekf (con la mejor estimación actual)
```

Guard de stamps duplicados copiado del patrón del módulo 1 (común al arrancar la sim).

### Paso de corrección

Disparado por `on_landmarks`. **Si `posts_` vacío o el array de landmarks vacío → no corregir** (solo queda la predicción; P crece, el elipsoide se agranda — exactamente el caso "pocos landmarks visibles" pedido por el informe). Para cada landmark asociado:

```
on_landmarks(msg):
    si posts_ vacío: return
    para cada landmark detectado (r_obs, β_obs):
        j = asociar(r_obs, β_obs);  si no matchea (gating): continue
        z      = [r_obs, β_obs]ᵀ
        h(x_)  = [range_esp, bearing_esp]ᵀ  con (mxⱼ, myⱼ)
        y_innov = z − h(x_);  normalizar y_innov[1] (bearing) a (−π, π]
        H = ∂h/∂x evaluado en x_ y (mxⱼ,myⱼ)
        S = H · P_ · Hᵀ + R          (V=I ⇒ + R)
        K = P_ · Hᵀ · S⁻¹            (3×2)
        [x_,y_,θ_] += K · y_innov;   normalizar θ_
        P_ = (I₃ − K·H) · P_         (o forma de Joseph para estabilidad numérica)
    publicar TF y odometry_ekf
```

Procesar landmarks secuencialmente (una actualización 2×2 por poste) es más simple y robusto que apilar todas en un bloque grande; cada `S` es 2×2 e invertible trivialmente. La normalización del residuo de bearing es **crítica**: sin ella, un salto de ±2π destruye la corrección.

### Publicar `map → base_link_ekf`

```cpp
geometry_msgs::msg::TransformStamped t;
t.header.stamp = <stamp del mensaje que disparó>;   // no now()
t.header.frame_id = map_frame_;        // "map"
t.child_frame_id  = ekf_frame_;        // "base_link_ekf"
t.transform.translation.x = x_;  .y = y_;  .z = 0.0;
tf2::Quaternion q; q.setRPY(0,0,theta_); t.transform.rotation = tf2::toMsg(q);
tf_broadcaster_->sendTransform(t);
```

Paralelamente publicar `/robot/odometry_ekf` (`nav_msgs/Odometry`, header `map`, child `base_link_ekf`) copiando `x_,y_,θ_` a `pose.pose` y los 9 elementos relevantes de `P_` (las componentes x,y,θ) a `pose.covariance` (matriz 6×6 row-major: índices 0=xx, 1=xy, 5=xθ, 6=yx, 7=yy, 11=yθ, 30=θx, 31=θy, 35=θθ). Esto alimenta el display "Covariance" de RViz2 → el elipsoide pedido por el informe.

**El stamp del TF y del odometry_ekf es el del mensaje que disparó la publicación**, nunca `this->now()`, para evitar warnings de extrapolación bajo `use_sim_time`.

### Nota sobre `EKFilter(3,3,3,2,2)` (variante con librería del curso)

Si en lugar de Eigen se usa el paquete `robmovil_ekf` (`EKFilter`), hay que **modificar `localizer_ekf.cpp`**:

```cpp
// de:  EKFilter(3, 2, 3, 2, 2)
// a:   EKFilter(3, 3, 3, 2, 2)
robmovil_ekf::LocalizerEKF::LocalizerEKF(void) : EKFilter(3, 3, 3, 2, 2)
```

El `nu=2 → 3` refleja que el control omnidireccional es `[vx, vy, ωz]` (3-DoF), no `[v, ω]` del diferencial. Con esa librería, `f/h/F/W/H/V` se implementan en los métodos virtuales `makeBaseA/makeBaseW/makeBaseH/makeBaseV/makeProcess/makeMeasure`, con exactamente las ecuaciones de arriba. El plan base usa Eigen porque da control total sobre el procesamiento secuencial de landmarks y el gating; EKFilter es válido si se prefiere reusar el material de cátedra.

---

## 5. Launch files

### `ekf_localizer.launch.py` (solo módulo 3 — experimentos manuales del informe §4)

Lanza el detector + el EKF. **No** lanza el controlador (el robot se mueve manual publicando `/robot/cmd_vel` con velocidades constantes). Asume que el módulo 1 corre por separado (o se incluye su launch).

Nodos:
- `Node(package='ekf_localizer', executable='landmark_detector', parameters=[{'use_sim_time': True}])`
- `Node(package='ekf_localizer', executable='ekf_localizer', parameters=[{'use_sim_time': True}])`
- (incluir o requerir) `modelo_holonomico/odometry.launch.py` para tener `/robot/odometry` y `odom→base_link`.

### `seguimiento_ekf.launch.py` (sistema completo — Requerimiento 4)

Compone los tres módulos. La integración con el módulo 2 es **un solo cambio de parámetro**:

```python
# módulo 1
IncludeLaunchDescription(modelo_holonomico/odometry.launch.py)
# módulo 3
Node(ekf_localizer/landmark_detector, params={use_sim_time:True})
Node(ekf_localizer/ekf_localizer,     params={use_sim_time:True})
# módulo 2 — reconectado al EKF
IncludeLaunchDescription(lazo_cerrado/lazo_cerrado.launch.py,
    launch_arguments={'base_frame': 'base_link_ekf', 'use_sim_time': 'true'})
```

`base_frame:=base_link_ekf` hace que el follower del módulo 2 lea `map → base_link_ekf` como feedback. Sin recompilar. Todos los nodos con `use_sim_time:=true` (obligatorio; sin él los stamps no coinciden con CoppeliaSim).

Orden de arranque: módulo 1 primero (publica la primera TF/odometría); luego EKF y detector; el follower del módulo 2 último. El follower ya publica Twist=0 mientras no haya TF; el EKF debe tener guard equivalente (no integrar hasta recibir la primera odometría; no corregir hasta recibir `/posts`).

---

## 6. Plan de implementación por pasos

**Paso 1 — Esqueleto del paquete (~30 min).**
Crear `ekf_localizer/` con CMakeLists, package.xml, los 4 `.cpp`/`.h` vacíos compilables (clases que solo crean subs/pubs y loguean al arrancar).
Validar: `colcon build --packages-select ekf_localizer` sin errores; `ros2 run ekf_localizer landmark_detector` y `ekf_localizer` arrancan e imprimen su log de inicio.

**Paso 2 — Detector: lectura de scan + TF (~45 min).**
Implementar `on_laser_scan`, `updateLaserTf`, conversión a cartesiano, y `publishPointcloud` (sin clustering aún: publicar todos los puntos).
Validar: con la sim corriendo, `ros2 topic echo /landmarks_pointcloud --once` muestra puntos; en RViz2 la nube cae sobre los obstáculos, en frame `base_link`. Si el lookup de TF falla, verificar `laser_frame:=front_laser`.

**Paso 3 — Detector: clustering + range/bearing (~1 h).**
Implementar el clustering secuencial, filtros de tamaño, cálculo de centroide y (range, bearing), publicación de `/landmarks`.
Validar: `ros2 topic echo /landmarks` → cantidad ≈ postes visibles frente al robot; `range`/`bearing` coherentes (poste al frente → bearing≈0). En RViz los centroides caen sobre los postes. Tunear `cluster_distance_threshold` y `min_cluster_points` hasta no ver clusters partidos ni falsos.

**Paso 4 — EKF: ingestión de mapa y odometría (~45 min).**
Implementar `on_posts` (guardar 16 postes) y `on_odometry` con guard de stamps + lectura de `vx,vy,wz,dt`. Sin filtro aún: copiar la pose inicial de la primera odometría.
Validar: `ros2 topic echo /robot/odometry_ekf` publica; log confirma "mapa recibido: 16 postes" (verificar transient_local: si dice 0, el QoS del sub de `/posts` está mal).

**Paso 5 — EKF: predicción (~1 h).**
Implementar `f()`, `F`, `Q`, actualización de `P_`, y publicación de TF `map→base_link_ekf`.
Validar: mover el robot manual. `base_link_ekf` debe coincidir con `base_link` (compuesto a `map`) ya que solo hay predicción ≈ misma integración que odometría. El elipsoide de `/robot/odometry_ekf` **crece monótonamente**. Test strafing puro (`vy≠0, vx=wz=0`): `base_link_ekf` se desplaza lateralmente.

**Paso 6 — EKF: asociación + corrección (~1.5 h).**
Implementar `h()`, `H`, `R`, asociación NN+gating, ganancia de Kalman, actualización de estado y `P_`, normalización de residuo de bearing.
Validar: con el robot quieto frente a postes, `base_link_ekf` converge hacia `base_link_gt`. El elipsoide **se contrae** al recibir landmarks. Inducir deriva (mover manual sin postes, luego volver a verlos): la pose EKF debe corregir. Confirmar que el residuo de bearing nunca da saltos de ~2π.

**Paso 7 — Experimentos de localización (informe §4) (~1.5 h).**
Grabar bags moviendo manual (líneas, vueltas, zig-zag) con deriva odométrica visible, incluyendo un tramo sin postes.
```bash
ros2 bag record -o experimentos/ekf/<caso>/bag \
    /robot/cmd_vel /robot/odometry /robot/front_laser/scan \
    /landmarks /posts /tf /tf_static
```
Validar: graficar `(x,y,θ)` de `base_link` (odom), `base_link_ekf` (EKF) y `base_link_gt` (GT). El EKF debe mostrar menos deriva que la odometría sola. Documentar crecimiento de covarianza en el tramo sin postes.

**Paso 8 — Integración con módulo 2 (informe §5) (~1 h).**
Escribir `seguimiento_ekf.launch.py` con `base_frame:=base_link_ekf`. Correr el cuadrado de 2 m.
Validar: el robot sigue el cuadrado usando la pose EKF como feedback. Capturar screenshots de RViz2 con el elipsoide en: inicio, las 4 esquinas, tramo con muchos postes (elipse chica) y tramo con pocos (elipse expandida).

Tiempo total estimado: ~9 h de implementación + tuning.

---

## 7. Riesgos y decisiones técnicas

**Mapa `/posts` no llega (riesgo alto).** Si el sub usa QoS default y el sim publica `/posts` latched una sola vez antes del EKF, nunca se recibe → la corrección jamás corre. Detección: log "0 postes". Mitigación: QoS `transient_local` en el sub de `/posts`. Es la causa de fallo silencioso más probable.

**Frame del laser equivocado.** El Taller 9 usa `laser_frame="laser"`; este robot usa `front_laser`. Con el default del taller, el lookup de TF falla siempre y `/landmarks` queda vacío. Detección: WARN_THROTTLE de TF constante. Mitigación: default `front_laser` y verificar con `ros2 run tf2_tools view_frames`.

**Residuo de bearing sin normalizar.** Un poste detrás del robot da `bearing≈±π`; restar `θ` puede producir un residuo de ~2π → la pose salta. Detección: saltos bruscos de `base_link_ekf`. Mitigación: normalizar `y_innov[1]` a `(−π, π]` siempre.

**Asociación de datos errónea con deriva grande.** Si la odometría derivó mucho antes de ver el primer poste, el NN puede matchear contra el poste vecino equivocado → corrección que empeora. Detección: la pose EKF "salta" a una ubicación errónea al ver postes. Mitigación: gating con `association_max_dist`; arrancar el EKF desde la pose inicial conocida. **Alternativa descartada: Mahalanobis** (usando `S` del filtro) — más robusta ante anisotropía de covarianza, descartada en versión base por complejidad; queda como mejora si el NN+gating produce mismatches.

**Saturación de velocidades del módulo 2 (heredado).** Con `kp_xy≥1.5` y error inicial grande, los comandos superan 2 m/s → la cinemática inversa amplifica → el simulador capa cada rueda por separado → el robot no se mueve como `(vx,vy,wz)`, contaminando la `u` que lee el EKF. Mitigación: mantener `kp_xy` moderado (≈0.8) o agregar saturación en el follower.

**Covarianza de `/robot/odometry` en cero.** El módulo 1 no puebla las covarianzas. El EKF **no** debe leerlas; genera Q propia desde parámetros. Riesgo solo si por error se intenta usar `msg.twist.covariance` como Q → sería todo ceros y `P_` colapsaría.

**Decisión: EKF disparado por odometría vs por timer.** Elegido **por odometría** (predicción en `on_odometry`, corrección en `on_landmarks`). Alternativa descartada: timer a frecuencia fija — da `Δt` constante pero introduce latencia y complejidad de sincronización; con `use_sim_time` los stamps dan el `Δt` correcto directamente.

**Decisión: detector sin IDs vs detector con asociación.** Elegido **detector sin IDs** (clusters genéricos), asociación en el EKF. Alternativa descartada: que el detector matchee contra `/posts` — acopla el detector al mapa y al estado, violando separación del Taller 9.

**Decisión: Eigen vs librería `EKFilter` del curso.** Elegido **Eigen** en el plan base por control total del procesamiento secuencial de landmarks y gating. `EKFilter(3,3,3,2,2)` es válido reutilizando el material de cátedra (con el cambio `nu: 2→3`); trade-off: menos flexibilidad en el lazo de corrección multi-landmark.
