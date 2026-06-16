# Etapa A — Nodo LiDAR: Detección de Landmarks (ekf_localizer)

## Objetivo

El nodo `landmark_detector` toma el scan de rango del LiDAR frontal (`/robot/front_laser/scan`)
y publica una lista de landmarks detectados en frame `base_link`. Cada landmark es un poste
candidato representado como `(range, bearing)` en el plano horizontal del robot.

Responsabilidades del nodo:

- Transformar los puntos del scan de frame `front_laser` a frame `base_link` via TF.
- Agrupar puntos consecutivos del scan mediante clustering secuencial.
- Calcular el centroide de cada cluster y derivar `range` y `bearing` en `base_link`.
- Publicar `/landmarks` (`robmovil_msgs/msg/LandmarkArray`) para consumo del `ekf_node`.
- Publicar `/landmark_markers` (`visualization_msgs/msg/MarkerArray`) para visualizacion en RViz2.

El nodo es **stateless** respecto al mapa: no conoce `/posts`, no asigna IDs a los postes.
La asociacion de datos (cluster → poste del mapa) vive exclusivamente en `ekf_node`.
Frame de salida: `base_link` (no `front_laser`), para que el EKF opere en frame del robot.

---

## Archivos a crear

Todos los paths son relativos a la raiz del repositorio:
`/Users/pablo/Documents/Estudio/Facultad/2025-2C/Robótica Móvil/TP_Final_Robotica/`

```
ekf_localizer/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── ekf_localizer/
│       ├── LandmarkDetector.h
│       └── EkfLocalizer.h          (esqueleto, sin implementar en esta etapa)
├── src/
│   ├── LandmarkDetector.cpp
│   ├── landmark_detector_node.cpp
│   ├── EkfLocalizer.cpp            (esqueleto, sin implementar en esta etapa)
│   └── ekf_localizer_node.cpp      (esqueleto, sin implementar en esta etapa)
└── launch/
    └── landmark_detector_test.launch.py
```

---

## Paso 1 — Esqueleto del paquete ekf_localizer compilable

### Descripcion

Crear el paquete ROS 2 completo con todos los archivos necesarios para que compile sin errores,
aunque los nodos solo arranquen y logueen un mensaje de inicio. Esto valida la infraestructura
(CMakeLists, package.xml, dependencias, instalacion de targets) antes de agregar logica.

### CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.5)
project(ekf_localizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robmovil_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(landmark_detector
  src/landmark_detector_node.cpp
  src/LandmarkDetector.cpp)
ament_target_dependencies(landmark_detector
  rclcpp std_msgs geometry_msgs sensor_msgs visualization_msgs
  robmovil_msgs tf2 tf2_ros tf2_geometry_msgs angles)

add_executable(ekf_localizer
  src/ekf_localizer_node.cpp
  src/EkfLocalizer.cpp)
ament_target_dependencies(ekf_localizer
  rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs
  robmovil_msgs tf2 tf2_ros tf2_geometry_msgs angles)
target_link_libraries(ekf_localizer Eigen3::Eigen)

install(TARGETS landmark_detector ekf_localizer
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include/
  DESTINATION include)

ament_package()
```

Diferencias respecto al plan original:

- `project(ekf_localizer)` — nombre corregido (el plan original usaba `robmovil_ekf`).
- `find_package(visualization_msgs REQUIRED)` — agregado explicitamente (necesario para `MarkerArray`).
- `find_package(angles REQUIRED)` — para `angles::normalize_angle` en el EKF.
- `find_package(eigen3_cmake_module REQUIRED)` — requerido para que `find_package(Eigen3)` funcione en ROS 2 Humble.

### package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ekf_localizer</name>
  <version>0.1.0</version>
  <description>EKF-based localization using LiDAR landmark detection.</description>
  <maintainer email="pbenajac9@gmail.com">pablo</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>eigen3_cmake_module</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>robmovil_msgs</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>angles</depend>
  <depend>eigen</depend>
</package>
```

### Criterio de validacion

```bash
cd <workspace_ros2>
colcon build --packages-select ekf_localizer
ros2 run ekf_localizer landmark_detector  # debe imprimir "[landmark_detector] Nodo iniciado."
ros2 run ekf_localizer ekf_localizer      # debe imprimir "[ekf_localizer] Nodo iniciado."
```

---

## Paso 2 — Parametros, subs/pubs y TF en el constructor

### Descripcion

Declarar todos los parametros ROS 2 del detector, crear el subscriber al scan, los dos
publishers de salida, e inicializar el `tf2_ros::Buffer` y el `tf2_ros::TransformListener`.
No se procesa ningun dato aun: el callback solo loguea que recibio un scan.

### include/ekf_localizer/LandmarkDetector.h

```cpp
#pragma once

// LandmarkDetector: convierte /robot/front_laser/scan en /landmarks (range, bearing
// en frame base_link). Detector stateless respecto al mapa — no conoce /posts.
// Frame de entrada: front_laser. Frame de salida: base_link.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <robmovil_msgs/msg/landmark_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

namespace robmovil {

class LandmarkDetector : public rclcpp::Node
{
public:
  LandmarkDetector();

private:
  // Parametros
  std::string robot_frame_;
  std::string laser_frame_;
  double landmark_diameter_;
  double cluster_distance_threshold_;
  int    min_cluster_points_;
  int    max_cluster_points_;
  double max_range_;

  // TF
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2::Transform                              laser_transform_;
  bool                                        laser_tf_ready_;

  // Comunicacion ROS 2
  std::string scan_topic_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    scan_sub_;
  rclcpp::Publisher<robmovil_msgs::msg::LandmarkArray>::SharedPtr landmarks_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  // Callbacks y auxiliares
  void on_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  bool updateLaserTf(const std_msgs::msg::Header& header);
  void publishMarkers(const robmovil_msgs::msg::LandmarkArray& landmarks,
                      const rclcpp::Time& stamp);
};

}  // namespace robmovil
```

### src/LandmarkDetector.cpp — constructor

```cpp
#include "ekf_localizer/LandmarkDetector.h"

using namespace robmovil;

LandmarkDetector::LandmarkDetector()
  : Node("landmark_detector"),
    laser_tf_ready_(false)
{
  // Parametros
  robot_frame_                = this->declare_parameter("robot_frame",
                                    std::string("base_link"));
  laser_frame_                = this->declare_parameter("laser_frame",
                                    std::string("front_laser"));
  landmark_diameter_          = this->declare_parameter("landmark_diameter", 0.1);
  cluster_distance_threshold_ = this->declare_parameter("cluster_distance_threshold", 0.15);
  min_cluster_points_         = this->declare_parameter("min_cluster_points", 2);
  max_cluster_points_         = this->declare_parameter("max_cluster_points", 30);
  max_range_                  = this->declare_parameter("max_range", 8.0);
  scan_topic_                 = this->declare_parameter<std::string>("scan_topic",
                                    std::string("/robot/front_laser/scan"));

  // TF
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subs / Pubs
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::QoS(10),
      std::bind(&LandmarkDetector::on_laser_scan, this, std::placeholders::_1));

  landmarks_pub_ = this->create_publisher<robmovil_msgs::msg::LandmarkArray>(
      "/landmarks", rclcpp::QoS(10));

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/landmark_markers", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(),
      "[landmark_detector] Iniciado. robot_frame=%s laser_frame=%s "
      "cluster_threshold=%.3f min_pts=%d max_pts=%d max_range=%.1f",
      robot_frame_.c_str(), laser_frame_.c_str(),
      cluster_distance_threshold_,
      min_cluster_points_, max_cluster_points_, max_range_);
}
```

Notas de estilo (siguiendo `mecanum_odometry.cpp`):

- Miembros con trailing underscore: `robot_frame_`, `laser_frame_`, `scan_sub_`, etc.
- Namespace `robmovil`, sin `using namespace std`.
- `RCLCPP_INFO` en el constructor con los parametros efectivos.
- `std::string("...")` explicito en `declare_parameter` para evitar ambiguedad con `const char*`.

### Criterio de validacion

```bash
ros2 run ekf_localizer landmark_detector \
    --ros-args -p laser_frame:=front_laser -p use_sim_time:=true
# Debe imprimir la linea de RCLCPP_INFO con los parametros.
ros2 topic list | grep landmarks   # debe aparecer /landmarks y /landmark_markers
```

---

## Paso 3 — Callback del scan: TF y conversion a cartesiano

### Descripcion

Implementar `on_laser_scan` y `updateLaserTf`. Por cada scan valido:

1. Obtener la TF `front_laser → base_link` con `tf2::TimePointZero` (TF estatica).
2. Filtrar rangos validos e iterar sobre los puntos del scan.
3. Convertir cada punto `(range, angle)` a coordenadas cartesianas en frame `front_laser`.
4. Aplicar la transformacion TF para obtener el punto en frame `base_link`.
5. Acumular los puntos validos en `std::vector<tf2::Vector3> cartesian`.

### updateLaserTf: TF estatica con TimePointZero

La TF `front_laser → base_link` es **estatica** (publicada por `robot_state_publisher`
o por el simulador en `/tf_static`). Para TFs estaticas se debe usar `tf2::TimePointZero`
en lugar de construir un `tf2::TimePoint` manual a partir de `stamp.sec`/`stamp.nanosec`.

```cpp
bool LandmarkDetector::updateLaserTf(const std_msgs::msg::Header& header)
{
  // Para TF estatica (front_laser → base_link) usar TimePointZero.
  // NO usar: tf2::TimePoint(std::chrono::seconds(header.stamp.sec) + ...)
  // porque falla si la TF aun no fue publicada al stamp exacto del scan.
  try {
    auto tf_stamped = tf_buffer_->lookupTransform(
        robot_frame_,            // frame destino: base_link
        laser_frame_,            // frame origen:  front_laser
        tf2::TimePointZero);     // tiempo: TF mas reciente disponible (estatica)
    tf2::fromMsg(tf_stamped.transform, laser_transform_);
    laser_tf_ready_ = true;
    return true;
  }
  catch (const tf2::TransformException& ex) {
    // Imprimir en orden: laser_frame_ → robot_frame_ (origen → destino)
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF %s→%s no disponible aun: %s",
        laser_frame_.c_str(), robot_frame_.c_str(), ex.what());
    return false;
  }
}
```

Razon del orden `laser_frame_ → robot_frame_` en el mensaje: es la transformacion que se
esta intentando obtener (origen → destino), lo que facilita depurar con `view_frames`.

### on_laser_scan: conversion a cartesiano

```cpp
void LandmarkDetector::on_laser_scan(
    const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  if (!updateLaserTf(scan->header)) return;

  std::vector<tf2::Vector3> cartesian;
  cartesian.reserve(scan->ranges.size());

  const double range_min    = scan->range_min;
  const double range_max_eff = std::min(
      static_cast<double>(scan->range_max), max_range_);

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    double r = scan->ranges[i];
    if (!std::isfinite(r) || r < range_min || r > range_max_eff) continue;

    double theta = scan->angle_min + i * scan->angle_increment;
    double xl = r * std::cos(theta);
    double yl = r * std::sin(theta);

    // Transformar de frame front_laser a frame base_link.
    tf2::Vector3 p_laser(xl, yl, 0.0);
    tf2::Vector3 p_robot = laser_transform_ * p_laser;
    cartesian.push_back(p_robot);
  }

  // Continua en el paso de clustering (Paso 4).
  // Por ahora: loguear cantidad de puntos validos y salir.
  RCLCPP_DEBUG(this->get_logger(),
      "Scan recibido: %zu puntos validos en base_link", cartesian.size());
}
```

### Criterio de validacion

Con la simulacion corriendo y el robot cargado:

```bash
ros2 run ekf_localizer landmark_detector --ros-args -p use_sim_time:=true
# No debe aparecer WARN de TF si el simulador esta corriendo.
# Si aparece: verificar que laser_frame=front_laser con:
ros2 run tf2_tools view_frames
# Debe existir el frame "front_laser" en el arbol TF.
```

---

## Paso 4 — Clustering secuencial y centroide a (range, bearing)

### Descripcion

Implementar el clustering secuencial sobre el vector `cartesian`. El scan LiDAR entrega
puntos en orden angular: los puntos de un mismo poste son angularmente consecutivos y
cercanos en distancia euclidea. Un salto mayor que `cluster_distance_threshold_` indica
el limite entre dos objetos distintos.

### Algoritmo de clustering

```cpp
// Dentro de on_laser_scan, despues de construir cartesian:

robmovil_msgs::msg::LandmarkArray landmarks_msg;
landmarks_msg.header.stamp    = scan->header.stamp;  // NO now()
landmarks_msg.header.frame_id = robot_frame_;        // "base_link"

std::vector<tf2::Vector3> cluster;

auto emit_landmark = [&](const std::vector<tf2::Vector3>& pts) {
  if (static_cast<int>(pts.size()) < min_cluster_points_) return;
  if (static_cast<int>(pts.size()) > max_cluster_points_) return;

  // Centroide del cluster
  tf2::Vector3 centroid(0.0, 0.0, 0.0);
  for (const auto& p : pts) centroid += p;
  centroid /= static_cast<double>(pts.size());

  double range   = std::sqrt(centroid.x() * centroid.x()
                           + centroid.y() * centroid.y());
  double bearing = std::atan2(centroid.y(), centroid.x());

  robmovil_msgs::msg::Landmark lm;
  lm.range   = static_cast<float>(range);
  lm.bearing = static_cast<float>(bearing);
  landmarks_msg.landmarks.push_back(lm);
};

for (const auto& p : cartesian) {
  if (cluster.empty()) {
    cluster.push_back(p);
  } else {
    tf2::Vector3 diff = p - cluster.back();
    double d = diff.length();
    if (d < cluster_distance_threshold_) {
      cluster.push_back(p);
    } else {
      emit_landmark(cluster);
      cluster.clear();
      cluster.push_back(p);
    }
  }
}
// Procesar el ultimo cluster
if (!cluster.empty()) emit_landmark(cluster);
```

### Umbral de cluster

`cluster_distance_threshold_` (default 0.15 m) cubre el diametro del poste (0.10 m) mas
un margen de 0.05 m por variacion de angulo discreto. Dos lecturas consecutivas sobre el
mismo poste nunca distan mas que el diametro; un salto mayor indica cambio de objeto.

Filtro opcional de ancho geometrico (activar si aparecen falsos postes por paredes):

```cpp
// Dentro de emit_landmark, antes de aceptar el cluster:
double ancho = (pts.front() - pts.back()).length();
if (ancho > 1.5 * landmark_diameter_) return;  // segmento demasiado largo -> pared
```

### Header de salida

```cpp
// CRITICO: usar el stamp del mensaje entrante, nunca this->now()
// Con use_sim_time=true, this->now() puede devolver tiempo diferente al del scan.
landmarks_msg.header.stamp    = scan->header.stamp;
landmarks_msg.header.frame_id = robot_frame_;   // "base_link"
```

### Criterio de validacion

```bash
ros2 topic echo /landmarks
# Con el robot frente a un poste:
# - range: distancia aproximada al poste (m)
# - bearing: ~0.0 rad si el poste esta al frente
# Con el robot quieto sin postes visibles: array vacio.
```

En RViz2: agregar `MarkerArray` en `/landmark_markers` → los marcadores deben
coincidir con la posicion de los postes visibles.

Tunear si hay problemas:

- Clusters partidos (un poste da 2 landmarks): bajar `cluster_distance_threshold_`.
- Paredes detectadas como landmark: subir `min_cluster_points_` o activar filtro de ancho.
- Postes lejanos no detectados: subir `max_range_`.

---

## Paso 5 — Publicacion de /landmarks y markers

### Descripcion

Publicar el `LandmarkArray` construido en el paso anterior y un `MarkerArray` con esferas
en la posicion cartesiana de cada centroide (solo para visualizacion en RViz2).

### Publicar /landmarks

```cpp
// Al final de on_laser_scan, despues del loop de clustering:
landmarks_pub_->publish(landmarks_msg);

RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "Landmarks detectados: %zu", landmarks_msg.landmarks.size());
```

### publishMarkers: MarkerArray en /landmark_markers

El tipo de publicacion es `MarkerArray` (no `PointCloud`). Cada landmark se visualiza
como una esfera roja en frame `base_link`.

```cpp
void LandmarkDetector::publishMarkers(
    const robmovil_msgs::msg::LandmarkArray& landmarks,
    const rclcpp::Time& stamp)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Primero: marker DELETE_ALL para limpiar los del scan anterior
  visualization_msgs::msg::Marker delete_all;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_all);

  for (size_t i = 0; i < landmarks.landmarks.size(); ++i) {
    const auto& lm = landmarks.landmarks[i];

    visualization_msgs::msg::Marker m;
    m.header.stamp    = stamp;
    m.header.frame_id = robot_frame_;   // "base_link"
    m.ns              = "landmarks";
    m.id              = static_cast<int>(i);
    m.type            = visualization_msgs::msg::Marker::SPHERE;
    m.action          = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = lm.range * std::cos(lm.bearing);
    m.pose.position.y = lm.range * std::sin(lm.bearing);
    m.pose.position.z = 0.3;   // altura visual sobre el suelo
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.12;
    m.color.r = 1.0; m.color.g = 0.2; m.color.b = 0.0; m.color.a = 0.9;
    m.lifetime = rclcpp::Duration::from_seconds(0.2);

    marker_array.markers.push_back(m);
  }

  markers_pub_->publish(marker_array);
}
```

Llamar a `publishMarkers` al final de `on_laser_scan` pasando `landmarks_msg` y
`rclcpp::Time(scan->header.stamp)`.

### Criterio de validacion

```bash
ros2 topic echo /landmark_markers --once
# Debe mostrar markers con ns="landmarks" y esferas rojas.
# En RViz2: agregar MarkerArray -> /landmark_markers
# Las esferas deben aparecer sobre los postes visibles en el sensor LaserScan.
```

---

## Paso 6 — Launch file y validacion end-to-end

### landmark_detector_test.launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ekf_localizer',
            executable='landmark_detector',
            name='landmark_detector',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_frame': 'base_link',
                'laser_frame': 'front_laser',
                'scan_topic': '/robot/front_laser/scan',
                'landmark_diameter': 0.1,
                'cluster_distance_threshold': 0.15,
                'min_cluster_points': 2,
                'max_cluster_points': 30,
                'max_range': 8.0,
            }],
        ),
    ])
```

Este launch es para pruebas manuales del detector en aislamiento. Los launches completos
del modulo 3 (`ekf_localizer.launch.py` y `seguimiento_ekf.launch.py`) se crean en la
Etapa B junto con el `ekf_node`.

### Procedimiento de validacion end-to-end

**Requisitos previos:** CoppeliaSim corriendo con `omni_ekf.ttt`, modulo 1 activo
(publica `/robot/odometry` y TF `odom→base_link`).

```bash
# Terminal 1: modulo 1
ros2 launch modelo_holonomico odometry.launch.py

# Terminal 2: detector
ros2 launch ekf_localizer landmark_detector_test.launch.py

# Terminal 3: verificacion
ros2 topic hz /landmarks          # debe ser ~10-20 Hz (igual que el scan)
ros2 topic echo /landmarks --once # ver campos range y bearing
```

En RViz2:
1. Agregar `LaserScan` en `/robot/front_laser/scan` (frame `front_laser`).
2. Agregar `MarkerArray` en `/landmark_markers` (frame `base_link`).
3. Mover el robot manualmente (`ros2 topic pub /robot/cmd_vel ...`).
4. Verificar que los marcadores siguen los postes visibles en el scan.

Mover el robot frente a un poste conocido y verificar:
- `range` coincide con la lectura directa del scan al poste (tolerancia ±0.05 m).
- `bearing ≈ 0` cuando el poste esta al frente.
- `bearing ≈ +π/4` cuando el poste esta 45° a la izquierda.
- Cuando el poste sale del campo de vision: desaparece del array.

---

## Riesgos

**Frame del laser equivocado (riesgo alto).**
El Taller 9 usaba `laser_frame="laser"`; este robot usa `"front_laser"`. Si se usa el
default incorrecto, el `lookupTransform` falla en todos los scans y `/landmarks` queda
permanentemente vacio. Sintoma: `RCLCPP_WARN_THROTTLE` continuo de TF.
Mitigacion: default ya corregido a `"front_laser"` en `declare_parameter`.
Verificar con `ros2 run tf2_tools view_frames` que el frame `front_laser` existe.

**TF estatica no disponible al arrancar.**
El `TransformListener` necesita algunos milisegundos para recibir `/tf_static` del
simulador. Los primeros scans pueden fallar el lookup. `laser_tf_ready_` y el
`try/catch` con `RCLCPP_WARN_THROTTLE` manejan esto graciosamente: el nodo descarta
esos scans y reintenta en el siguiente. No requiere accion del usuario.

**Clusters partidos por curvatura del scan.**
En un poste lejano (>5 m), el incremento angular del laser genera puntos mas espaciados.
La distancia euclidea entre puntos consecutivos puede superar `cluster_distance_threshold_`
incluso dentro del mismo poste. Sintoma: un poste da 2 o mas landmarks.
Mitigacion: aumentar `cluster_distance_threshold_` a 0.20 m, o reducir `max_range_`.

**Paredes detectadas como landmarks.**
Un segmento largo de pared puede acumular puntos dentro del umbral de distancia si el
robot esta en un angulo rasante. Sintoma: landmarks con `range` muy alto o con cantidades
de puntos mayor que `max_cluster_points_`.
Mitigacion: `max_cluster_points_` ya filtra los clusters grandes. Activar el filtro de
ancho geometrico (`> 1.5 * landmark_diameter_`) si persiste.

**Tipo de mensaje `robmovil_msgs/LandmarkArray`.**
Verificar que el paquete `robmovil_msgs` del workspace incluye `Landmark.msg` y
`LandmarkArray.msg` con los campos `range` y `bearing` (float32). Si los campos tienen
nombres distintos (e.g., `id`, `x`, `y`), adaptar el codigo de publicacion.

**Header con `now()` en lugar del stamp del scan.**
Si se usa `this->now()` en el header de `/landmarks` o `/landmark_markers`, con
`use_sim_time=true` el tiempo simulado puede diferir del stamp del scan, causando
errores de extrapolacion en TF en el `ekf_node` cuando intente usar ese timestamp.
El codigo presentado usa siempre `scan->header.stamp`.

---

## Estimacion: 3.5 h

| Paso | Descripcion | Tiempo estimado |
|------|-------------|-----------------|
| 1    | Esqueleto del paquete compilable | 30 min |
| 2    | Parametros, subs/pubs, TF en el constructor | 30 min |
| 3    | Callback del scan: TF con TimePointZero y conversion a cartesiano | 45 min |
| 4    | Clustering secuencial y calculo de (range, bearing) | 45 min |
| 5    | Publicacion de /landmarks y MarkerArray | 20 min |
| 6    | Launch file y validacion end-to-end con CoppeliaSim | 30 min |

Total: ~3.5 h. El tiempo variable es el Paso 4 (tunear umbrales de clustering contra
los postes reales del simulador puede tomar entre 20 y 60 minutos adicionales segun
la configuracion de la escena).
