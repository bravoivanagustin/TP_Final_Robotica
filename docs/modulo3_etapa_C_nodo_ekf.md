# Etapa C — Nodo EKF con Eigen puro (ítem 4 de la consigna)

> **Plan REESCRITO desde cero.** La versión anterior asumía la API de `kfilter`
> (`timeUpdateStep`, `measureUpdateStep`, `setDt`, `calculateP`, `setMapLandmark`).
> Esos métodos **no existen**: eran invenciones. La consigna no exige `kfilter`
> ni `robmovil_ekf`; este plan implementa el EKF directamente con **Eigen puro**,
> que es más claro, autocontenido y sin dependencias misteriosas. Toda la
> matemática del filtro vive dentro de la clase `EkfLocalizer : public rclcpp::Node`.

---

## Arquitectura — clase EkfLocalizer, relación con Etapa A como prerrequisito

El nodo `ekf_localizer` implementa un EKF de estado mínimo (pose 2D) que **fusiona**:

- **Predicción** con la odometría del Módulo 1 (`/robot/odometry`), usada como
  *control* en forma de **deltas de odometría en frame body** `u = [dxb, dyb, dth]`
  (igual que la integración de `mecanum_odometry.cpp:103-110`). Se pueden obtener
  como diferencia de pose consecutiva rotada al body, o equivalentemente como
  `twist * dt` (el twist de `/robot/odometry` está en m/s; ambos métodos son equivalentes).
- **Corrección** con las observaciones de postes (`/landmarks`, range + bearing en
  frame `base_link`) producidas por el nodo `landmark_detector` de la **Etapa A**,
  asociadas contra el mapa real de 16 postes (`/posts`, frame `map`).

**Prerrequisito explícito:** la Etapa A (`landmark_detector`) debe estar implementada
y publicando `/landmarks` (`robmovil_msgs/msg/LandmarkArray`, frame `base_link`) antes
de validar esta etapa. Sin el detector, el paso de corrección nunca se ejecuta y el
EKF degenera a pura odometría (la covarianza crece monótonamente sin contraerse).

Variables de estado y matrices (todo Eigen):

```cpp
Eigen::Vector3d x_;          // [px, py, theta] en frame map
Eigen::Matrix3d P_;          // covarianza del estado
Eigen::Matrix3d Q_base_;     // ruido de proceso base (sin escalar por dt)
Eigen::Matrix2d R_;          // ruido de medición [sigma_range^2, sigma_bearing^2]
```

El nodo hereda directo de `rclcpp::Node` (constructor `Node("ekf_localizer")`), sigue
el estilo de `mecanum_odometry.cpp` (namespace `robmovil`, miembros con trailing
underscore, `RCLCPP_INFO` en el constructor, callbacks `on_<topic>`, auxiliares en
camelCase). **Bajo `use_sim_time=true` nunca se usa `rclcpp::Clock::now()`**: todos
los `dt` y stamps salen del `header.stamp` de los mensajes.

### Flujo de disparo

| Callback | Tópico | Acción |
|---|---|---|
| `on_odometry` | `/robot/odometry` | **Predicción** (calcula `u` desde delta de pose odom, integra `x_`, propaga `P_`), luego `publishEkfPose` |
| `on_landmarks` | `/landmarks` | **Corrección** secuencial por landmark asociado, luego `publishEkfPose` |
| `on_posts` | `/posts` | Carga el mapa una vez (QoS transient_local) |

---

## Archivos a crear

```
ekf_localizer/
├── CMakeLists.txt
├── package.xml
├── include/ekf_localizer/
│   └── EkfLocalizer.h
├── src/
│   ├── EkfLocalizer.cpp
│   └── ekf_localizer_node.cpp
└── launch/
    └── ekf_localizer.launch.py
```

Paths absolutos:

- `/Users/pablo/Documents/Estudio/Facultad/2025-2C/Robótica Móvil/TP_Final_Robotica/ekf_localizer/include/ekf_localizer/EkfLocalizer.h`
- `/Users/pablo/Documents/Estudio/Facultad/2025-2C/Robótica Móvil/TP_Final_Robotica/ekf_localizer/src/EkfLocalizer.cpp`
- `/Users/pablo/Documents/Estudio/Facultad/2025-2C/Robótica Móvil/TP_Final_Robotica/ekf_localizer/src/ekf_localizer_node.cpp`
- `/Users/pablo/Documents/Estudio/Facultad/2025-2C/Robótica Móvil/TP_Final_Robotica/ekf_localizer/launch/ekf_localizer.launch.py`

(El detector `LandmarkDetector.{h,cpp}` y su nodo pertenecen a la Etapa A; este
paquete los comparte, pero esta etapa solo agrega los archivos del EKF.)

---

## Paso 1 — Esqueleto del paquete y dependencias

Dependencias del nodo EKF: `rclcpp`, `nav_msgs` (Odometry), `geometry_msgs`
(PoseArray, TransformStamped), `robmovil_msgs` (LandmarkArray), `tf2`, `tf2_ros`
(buffer/listener/broadcaster), `tf2_geometry_msgs` (conversiones), `angles`
(normalización de ángulos), `Eigen3` (matrices del filtro).

### CMakeLists.txt (fragmento del target EKF)

> **Nota:** este fragmento es un addendum al CMakeLists.txt completo definido en la
> Etapa A (que ya contiene `cmake_minimum_required`, `project()`, etc.). Solo muestra
> las líneas específicas del target EKF que se agregan/modifican.

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robmovil_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(ekf_localizer src/ekf_localizer_node.cpp src/EkfLocalizer.cpp)
ament_target_dependencies(ekf_localizer
  rclcpp geometry_msgs nav_msgs robmovil_msgs
  tf2 tf2_ros tf2_geometry_msgs angles)
target_link_libraries(ekf_localizer Eigen3::Eigen)

install(TARGETS ekf_localizer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include/ DESTINATION include)
```

### package.xml (format 3)

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<depend>rclcpp</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>robmovil_msgs</depend>
<depend>tf2</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
<depend>angles</depend>
<buildtool_depend>eigen3_cmake_module</buildtool_depend>
<depend>eigen</depend>
```

### EkfLocalizer.h (esqueleto completo)

```cpp
#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <robmovil_msgs/msg/landmark_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Dense>

namespace robmovil
{

// EKF de localización 2D. Estado x_=[px,py,theta] en frame map.
// Predice con la odometría del Módulo 1 (control = delta de pose en body),
// corrige con los postes detectados por la Etapa A (landmark_detector).
class EkfLocalizer : public rclcpp::Node
{
  public:
    EkfLocalizer();

    void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void on_landmarks(const robmovil_msgs::msg::LandmarkArray::SharedPtr msg);
    void on_posts(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  private:
    // --- Modelo del EKF (Eigen puro) ---
    Eigen::Vector3d f(const Eigen::Vector3d& x, const Eigen::Vector3d& u) const;
    Eigen::Matrix3d computeJf(const Eigen::Vector3d& x, const Eigen::Vector3d& u) const;
    Eigen::Vector2d h(const Eigen::Vector3d& x, double mx, double my) const;
    Eigen::Matrix<double, 2, 3> computeJh(const Eigen::Vector3d& x, double mx, double my) const;

    // --- Inicialización y publicación ---
    bool tryInitFromTf(const rclcpp::Time& stamp);
    void publishEkfPose(const rclcpp::Time& stamp);

    // --- Suscripciones / publicaciones ---
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<robmovil_msgs::msg::LandmarkArray>::SharedPtr landmarks_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr posts_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_ekf_pub_;

    // --- TF ---
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // --- Estado del filtro ---
    Eigen::Vector3d x_;
    Eigen::Matrix3d P_;
    Eigen::Matrix3d Q_base_;
    Eigen::Matrix2d R_;

    // --- Mapa de postes (mx,my en frame map) ---
    std::vector<Eigen::Vector2d> map_landmarks_;

    // --- Baseline de odometría para calcular el delta de control ---
    bool odom_initialized_ = false;
    double last_odom_x_ = 0.0, last_odom_y_ = 0.0, last_odom_theta_ = 0.0;
    rclcpp::Time last_odom_time_;

    // --- Inicialización del estado desde TF map->base_link ---
    bool state_initialized_ = false;

    // --- Parámetros ---
    std::string map_frame_;
    std::string base_frame_;
    std::string ekf_frame_;
    double association_max_dist_;
    double min_range_;
};

}  // namespace robmovil
```

### ekf_localizer_node.cpp

```cpp
#include "ekf_localizer/EkfLocalizer.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robmovil::EkfLocalizer>());
  rclcpp::shutdown();
  return 0;
}
```

**Validación:** `colcon build --packages-select ekf_localizer` compila sin errores;
`ros2 run ekf_localizer ekf_localizer` arranca y loguea sus parámetros efectivos.

---

## Paso 2 — Suscripción a /posts (transient_local) y subs/pubs en el constructor

`/posts` lo publica CoppeliaSim **una sola vez** (latched) con las posiciones reales
de los 16 postes en frame `map`. Si el sub usa QoS default, el EKF nunca recibe el
mapa y la corrección jamás corre. **QoS obligatorio: `transient_local` + `reliable`.**

```cpp
EkfLocalizer::EkfLocalizer()
  : Node("ekf_localizer")
{
  // Parámetros.
  map_frame_            = this->declare_parameter("map_frame", std::string("map"));
  base_frame_           = this->declare_parameter("base_frame", std::string("base_link"));
  ekf_frame_            = this->declare_parameter("ekf_frame", std::string("base_link_ekf"));
  double sigma_dx       = this->declare_parameter("sigma_dx", 0.05);
  double sigma_dy       = this->declare_parameter("sigma_dy", 0.05);
  double sigma_dth      = this->declare_parameter("sigma_dth", 0.05);
  double sigma_range    = this->declare_parameter("sigma_range", 0.05);
  double sigma_bearing  = this->declare_parameter("sigma_bearing", 0.03);
  double init_cov       = this->declare_parameter("init_cov", 0.01);
  association_max_dist_ = this->declare_parameter("association_max_dist", 0.6);
  min_range_            = this->declare_parameter("min_range", 0.2);

  // Matrices base. Q_base_ NO escala por dt: u ya son deltas (m/paso), no velocidades.
  Q_base_ = Eigen::Matrix3d::Zero();
  Q_base_(0, 0) = sigma_dx * sigma_dx;
  Q_base_(1, 1) = sigma_dy * sigma_dy;
  Q_base_(2, 2) = sigma_dth * sigma_dth;

  R_ = Eigen::Matrix2d::Zero();
  R_(0, 0) = sigma_range * sigma_range;
  R_(1, 1) = sigma_bearing * sigma_bearing;

  x_.setZero();
  P_ = Eigen::Matrix3d::Identity() * init_cov;

  // TF.
  tf_buffer_      = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Mapa de postes: latched (transient_local), KeepLast(1), reliable.
  auto posts_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  posts_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/posts", posts_qos,
      std::bind(&EkfLocalizer::on_posts, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/robot/odometry", rclcpp::QoS(10),
      std::bind(&EkfLocalizer::on_odometry, this, std::placeholders::_1));

  landmarks_sub_ = this->create_subscription<robmovil_msgs::msg::LandmarkArray>(
      "/landmarks", rclcpp::QoS(10),
      std::bind(&EkfLocalizer::on_landmarks, this, std::placeholders::_1));

  odom_ekf_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/robot/odometry_ekf", rclcpp::QoS(10));

  RCLCPP_INFO(this->get_logger(),
      "EkfLocalizer listo. map=%s base=%s ekf=%s gate=%.2f sigma_range=%.3f sigma_bearing=%.3f",
      map_frame_.c_str(), base_frame_.c_str(), ekf_frame_.c_str(),
      association_max_dist_, sigma_range, sigma_bearing);
}

void EkfLocalizer::on_posts(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  map_landmarks_.clear();
  map_landmarks_.reserve(msg->poses.size());
  for (const auto& p : msg->poses) {
    map_landmarks_.emplace_back(p.position.x, p.position.y);
  }
  RCLCPP_INFO(this->get_logger(), "Mapa recibido: %zu postes en frame %s.",
              map_landmarks_.size(), msg->header.frame_id.c_str());
}
```

**Validación:** con la sim corriendo, el log imprime "Mapa recibido: 16 postes".
Si imprime 0 o nunca aparece, el QoS de `/posts` está mal (falta `transient_local`).

---

## Paso 3 — Inicialización del estado EKF desde TF map→base_link

El estado **no** se inicializa desde `/robot/odometry.pose.pose` (que está en frame
`odom`, no `map`). Se inicializa desde la primera TF `map → base_link` vía
`tf_buffer_->lookupTransform`. Si al arrancar la TF aún no está disponible (warm-up
de la sim), se reintenta en cada `on_odometry` hasta lograrlo; mientras tanto no se
integra.

```cpp
bool EkfLocalizer::tryInitFromTf(const rclcpp::Time& stamp)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    // TimePointZero: tomar la última TF disponible (no exigir el stamp exacto).
    tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF %s -> %s no disponible aun para inicializar el estado: %s",
        map_frame_.c_str(), base_frame_.c_str(), ex.what());
    return false;
  }

  x_(0) = tf.transform.translation.x;
  x_(1) = tf.transform.translation.y;
  x_(2) = tf2::getYaw(tf.transform.rotation);
  state_initialized_ = true;

  RCLCPP_INFO(this->get_logger(),
      "Estado EKF inicializado desde TF: x=%.3f y=%.3f theta=%.3f",
      x_(0), x_(1), x_(2));
  (void)stamp;
  return true;
}
```

**Validación:** el log "Estado EKF inicializado desde TF" aparece una vez tras
arrancar el Módulo 1; los valores iniciales coinciden con la pose ground-truth.

---

## Paso 4 — Funciones f, h, computeJf, computeJh con Eigen

El **control** es el delta de odometría en frame body `u = [dxb, dyb, dth]`, idéntico
a los deltas que integra `mecanum_odometry.cpp:103-110`. La predicción rota ese delta
por el `theta` actual del estado EKF (no por el theta de la odometría) y lo suma.

```cpp
// Modelo de movimiento: integra el delta body u=[dxb,dyb,dth] sobre x=[px,py,theta].
// Idéntico a mecanum_odometry.cpp:108-110.
Eigen::Vector3d EkfLocalizer::f(const Eigen::Vector3d& x, const Eigen::Vector3d& u) const
{
  const double th  = x(2);
  const double c   = std::cos(th);
  const double s   = std::sin(th);
  const double dxb = u(0), dyb = u(1), dth = u(2);

  Eigen::Vector3d xp;
  xp(0) = x(0) + dxb * c - dyb * s;
  xp(1) = x(1) + dxb * s + dyb * c;
  xp(2) = angles::normalize_angle(x(2) + dth);
  return xp;
}

// Jacobiano del movimiento F = df/dx (3x3), evaluado en (x,u).
Eigen::Matrix3d EkfLocalizer::computeJf(const Eigen::Vector3d& x, const Eigen::Vector3d& u) const
{
  const double th  = x(2);
  const double c   = std::cos(th);
  const double s   = std::sin(th);
  const double dxb = u(0), dyb = u(1);

  Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
  // d(px)/d(theta) = -dxb*sin(th) - dyb*cos(th)
  F(0, 2) = -dxb * s - dyb * c;
  // d(py)/d(theta) =  dxb*cos(th) - dyb*sin(th)
  F(1, 2) =  dxb * c - dyb * s;
  return F;
}

// Modelo de sensado: range/bearing al poste (mx,my) desde la pose x.
// bearing relativo al robot (el detector entrega en base_link), por eso resta theta.
Eigen::Vector2d EkfLocalizer::h(const Eigen::Vector3d& x, double mx, double my) const
{
  const double dx = mx - x(0);
  const double dy = my - x(1);
  const double q  = dx * dx + dy * dy;

  Eigen::Vector2d z;
  z(0) = std::sqrt(q);
  z(1) = angles::normalize_angle(std::atan2(dy, dx) - x(2));
  return z;
}

// Jacobiano de sensado H = dh/dx (2x3), evaluado en x y (mx,my).
Eigen::Matrix<double, 2, 3> EkfLocalizer::computeJh(const Eigen::Vector3d& x, double mx, double my) const
{
  const double dx = mx - x(0);
  const double dy = my - x(1);
  const double q  = dx * dx + dy * dy;
  const double r  = std::sqrt(q);

  Eigen::Matrix<double, 2, 3> H;
  H(0, 0) = -dx / r;   H(0, 1) = -dy / r;   H(0, 2) =  0.0;
  H(1, 0) =  dy / q;   H(1, 1) = -dx / q;   H(1, 2) = -1.0;
  return H;
}
```

**Validación:** test unitario informal — con `u=[0,0,0]`, `f(x,u)==x` y `computeJf==I`.
Con un poste al frente (`mx>x, my==y, theta==0`), `h(...).bearing ≈ 0`.

---

## Paso 5 — on_odometry: predicción con Eigen

`u` se calcula como el **delta de pose de la odometría** entre mensajes consecutivos,
expresado en frame body, usando el `theta` de la odometría *previa* para rotar el
delta mundo→body. Esto es robusto frente a que la odometría arranque desde una pose
distinta de la del EKF (solo importa el incremento). `Q_base_` se usa directamente
sin escalar por dt, porque `u` ya son deltas de distancia (m/paso), no velocidades.

> **Nota:** el twist de `/robot/odometry` tiene unidades de m/s (velocidad instantánea
> en frame body). Multiplicar por dt da los mismos deltas `[dxb, dyb, dth]` que calcula
> internamente `mecanum_odometry.cpp` (líneas 103-110). Ambos métodos (twist×dt y
> diferencia de pose consecutiva rotada al body) son equivalentes; aquí se usa la
> diferencia de pose porque es directamente lo que expone `msg->pose.pose`, pero
> twist×dt produce el mismo resultado.

```cpp
void EkfLocalizer::on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);

  const double ox  = msg->pose.pose.position.x;
  const double oy  = msg->pose.pose.position.y;
  const double oth = tf2::getYaw(msg->pose.pose.orientation);

  // Primer mensaje: guardar baseline de odometria y salir.
  if (!odom_initialized_) {
    odom_initialized_ = true;
    last_odom_x_ = ox;  last_odom_y_ = oy;  last_odom_theta_ = oth;
    last_odom_time_ = stamp;
    return;
  }

  // Inicializar el estado desde TF map->base_link si todavia no se hizo.
  if (!state_initialized_) {
    if (!tryInitFromTf(stamp)) {
      // TF aun no disponible: refrescar baseline para no acumular un delta gigante.
      last_odom_x_ = ox;  last_odom_y_ = oy;  last_odom_theta_ = oth;
      last_odom_time_ = stamp;
      return;
    }
  }

  const double dt = (stamp - last_odom_time_).seconds();

  // Guard de stamps duplicados/regresivos (comun al arrancar la sim).
  if (dt <= 1e-6) {
    last_odom_x_ = ox;  last_odom_y_ = oy;  last_odom_theta_ = oth;
    last_odom_time_ = stamp;
    return;
  }

  // Delta de pose en frame mundo (odom).
  const double ddx_w = ox - last_odom_x_;
  const double ddy_w = oy - last_odom_y_;
  const double dth   = angles::normalize_angle(oth - last_odom_theta_);

  // Rotar el delta mundo -> body con el theta de odometria previo.
  const double c = std::cos(last_odom_theta_);
  const double s = std::sin(last_odom_theta_);
  Eigen::Vector3d u;
  u(0) =  c * ddx_w + s * ddy_w;   // dxb
  u(1) = -s * ddx_w + c * ddy_w;   // dyb
  u(2) =  dth;

  // --- Prediccion EKF ---
  Eigen::Matrix3d F = computeJf(x_, u);
  x_ = f(x_, u);
  P_ = F * P_ * F.transpose() + Q_base_;

  // Actualizar baseline.
  last_odom_x_ = ox;  last_odom_y_ = oy;  last_odom_theta_ = oth;
  last_odom_time_ = stamp;

  publishEkfPose(stamp);
}
```

**Validación:** mover el robot manualmente. `base_link_ekf` debe seguir a `base_link`
(compuesto a `map`) ya que solo hay predicción ≈ misma integración que la odometría.
El elipsoide de `/robot/odometry_ekf` **crece monótonamente** sin corrección. Test de
strafing puro (`vy≠0, vx=wz=0`): `base_link_ekf` se desplaza lateralmente.

---

## Paso 6 — on_landmarks: data association NN+gating y corrección con Eigen

Para cada landmark detectado se proyecta a `map` usando la pose **actual** del EKF,
se busca el poste más cercano (nearest-neighbor), se aplica gating por distancia, y se
corrige con una actualización 2×2 secuencial. **El residuo de bearing se normaliza
siempre a (−π, π]** con `angles::normalize_angle` (sin esto, un poste detrás del robot
con `bearing≈±π` produce un salto de ~2π que destruye la corrección).

```cpp
void EkfLocalizer::on_landmarks(const robmovil_msgs::msg::LandmarkArray::SharedPtr msg)
{
  // Sin mapa o sin estado inicializado: no se puede corregir (solo predice; P crece).
  if (map_landmarks_.empty() || !state_initialized_) {
    return;
  }

  const rclcpp::Time stamp(msg->header.stamp);

  for (const auto& lm : msg->landmarks) {
    const double r_obs = lm.range;
    const double b_obs = lm.bearing;

    // Descartar landmarks numericamente inestables (demasiado cercanos).
    if (r_obs < min_range_) {
      continue;
    }

    // Proyectar la observacion a frame map con la pose actual del EKF.
    const double lx = x_(0) + r_obs * std::cos(b_obs + x_(2));
    const double ly = x_(1) + r_obs * std::sin(b_obs + x_(2));

    // Nearest-neighbor contra el mapa.
    int best_j = -1;
    double best_d2 = std::numeric_limits<double>::max();
    for (size_t j = 0; j < map_landmarks_.size(); ++j) {
      const double dx = lx - map_landmarks_[j](0);
      const double dy = ly - map_landmarks_[j](1);
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) { best_d2 = d2; best_j = static_cast<int>(j); }
    }

    // Gating: si el mejor match supera el umbral, descartar (falso positivo/ambiguo).
    if (best_j < 0 || best_d2 > association_max_dist_ * association_max_dist_) {
      continue;
    }

    const double mx = map_landmarks_[best_j](0);
    const double my = map_landmarks_[best_j](1);

    // --- Correccion EKF (actualizacion 2x2 por landmark) ---
    const Eigen::Vector2d h_pred = h(x_, mx, my);
    const Eigen::Matrix<double, 2, 3> H = computeJh(x_, mx, my);

    const Eigen::Matrix2d S = H * P_ * H.transpose() + R_;
    const Eigen::Matrix<double, 3, 2> K = P_ * H.transpose() * S.inverse();

    Eigen::Vector2d innov;
    innov(0) = r_obs - h_pred(0);
    innov(1) = angles::normalize_angle(b_obs - h_pred(1));   // CRITICO: normalizar bearing

    x_ += K * innov;
    x_(2) = angles::normalize_angle(x_(2));
    P_ = (Eigen::Matrix3d::Identity() - K * H) * P_;
  }

  publishEkfPose(stamp);
}
```

**Validación:** con el robot quieto frente a postes, `base_link_ekf` converge hacia
`base_link_gt` y el elipsoide **se contrae** al recibir landmarks. Inducir deriva
(mover sin postes, luego volver a verlos): la pose EKF debe corregir. Confirmar que el
residuo de bearing nunca da saltos de ~2π. Verificar que el NN no matchea postes
vecinos equivocados (subir/bajar `association_max_dist` si hay mismatches).

---

## Paso 7 — publishEkfPose: TF map→base_link_ekf + /robot/odometry_ekf con covarianza

La consigna pide la TF `map → base_link_ekf`. El tópico `/robot/odometry_ekf`
(`nav_msgs/Odometry`, header `map`, child `base_link_ekf`) es **adicional** para el
informe: alimenta el display "Covariance" de RViz2 (el elipsoide). **El stamp es el
del mensaje que disparó la publicación, nunca `now()`** (evita warnings de
extrapolación bajo `use_sim_time`).

```cpp
void EkfLocalizer::publishEkfPose(const rclcpp::Time& stamp)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, x_(2));

  // --- TF map -> base_link_ekf ---
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp    = stamp;
  t.header.frame_id = map_frame_;     // "map"
  t.child_frame_id  = ekf_frame_;     // "base_link_ekf"
  t.transform.translation.x = x_(0);
  t.transform.translation.y = x_(1);
  t.transform.translation.z = 0.0;
  t.transform.rotation      = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);

  // --- /robot/odometry_ekf con covarianza poblada ---
  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = stamp;
  odom.header.frame_id = map_frame_;
  odom.child_frame_id  = ekf_frame_;
  odom.pose.pose.position.x = x_(0);
  odom.pose.pose.position.y = x_(1);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf2::toMsg(q);

  // Mapear P_ (3x3, [x,y,theta]) a la covarianza 6x6 row-major [x,y,z,roll,pitch,yaw].
  // Indices: 0=xx 1=xy 5=xth | 6=yx 7=yy 11=yth | 30=thx 31=thy 35=thth.
  auto& cov = odom.pose.covariance;
  cov[0]  = P_(0, 0);  cov[1]  = P_(0, 1);  cov[5]  = P_(0, 2);
  cov[6]  = P_(1, 0);  cov[7]  = P_(1, 1);  cov[11] = P_(1, 2);
  cov[30] = P_(2, 0);  cov[31] = P_(2, 1);  cov[35] = P_(2, 2);

  odom_ekf_pub_->publish(odom);
}
```

**Validación:** `ros2 run tf2_tools view_frames` muestra `map → base_link_ekf`;
`ros2 topic echo /robot/odometry_ekf` muestra pose + covarianza no nula; en RViz2 el
elipsoide aparece sobre el robot y cambia de tamaño según haya o no postes.

---

## Paso 8 — Launch file

`ekf_localizer.launch.py` lanza el EKF (y, si se desea para experimentos manuales, el
detector de la Etapa A). Todos los nodos con `use_sim_time:=True` (obligatorio para
que los stamps coincidan con CoppeliaSim).

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = {'use_sim_time': True}

    landmark_detector = Node(
        package='ekf_localizer',
        executable='landmark_detector',   # Etapa A (prerrequisito)
        name='landmark_detector',
        output='screen',
        parameters=[use_sim_time],
    )

    ekf = Node(
        package='ekf_localizer',
        executable='ekf_localizer',
        name='ekf_localizer',
        output='screen',
        parameters=[use_sim_time],
    )

    return LaunchDescription([landmark_detector, ekf])
```

**Validación:** `ros2 launch ekf_localizer ekf_localizer.launch.py` arranca ambos
nodos; con el Módulo 1 corriendo en paralelo, el EKF inicializa estado, recibe el mapa
y publica `map → base_link_ekf`.

---

## Riesgos

- **`/posts` no llega (alto).** Si el sub no usa `transient_local`, el sim publica el
  mapa latched antes de arrancar el EKF y nunca se recibe → la corrección jamás corre.
  Detección: log "Mapa recibido: 0 postes" o ausente. Mitigación: QoS
  `KeepLast(1).reliable().transient_local()`.
- **Detector de la Etapa A ausente o mal configurado (alto).** Sin `/landmarks` el EKF
  degenera a odometría pura (P crece sin contraerse). El frame del laser debe ser
  `front_laser` (no `laser`). Es prerrequisito explícito de esta etapa.
- **Inicialización desde el frame equivocado.** Inicializar desde
  `/robot/odometry.pose` (frame `odom`) en vez de TF `map → base_link` desplaza todo el
  filtro. Mitigación: `tryInitFromTf` con `tf2::TimePointZero` y reintento.
- **Residuo de bearing sin normalizar.** Salto de ~2π destruye la corrección.
  Mitigación: `angles::normalize_angle` en `innov(1)` y en `x_(2)`, siempre.
- **Asociación errónea con deriva grande.** Si la odometría derivó mucho antes del
  primer poste, el NN puede matchear el poste vecino. Mitigación: gating con
  `association_max_dist`; inicializar desde la TF (pose conocida). Alternativa:
  distancia de Mahalanobis con `S` del filtro (queda como mejora si el NN+gating falla).
- **`dt` mal calculado bajo `use_sim_time`.** Usar `rclcpp::Clock::now()` rompe los
  stamps. Mitigación: `dt` siempre desde `header.stamp`; guard `dt <= 1e-6`.
- **Q escalada por dt incorrectamente.** Error del diseño con `kfilter` (`makeBaseQ` una sola vez con factor dt). Con Eigen puro y deltas de odometría (m/paso, no m/s), `Q_base_` se usa directamente sin escalar por dt: `P_ = F * P_ * F.transpose() + Q_base_`.

---

## Estimación: 8 h

- Paso 1 (esqueleto + CMake + package.xml): 1 h
- Paso 2 (subs/pubs + `/posts` transient_local): 0.75 h
- Paso 3 (init desde TF): 0.5 h
- Paso 4 (f, h, computeJf, computeJh): 1 h
- Paso 5 (predicción): 1 h
- Paso 6 (asociación + corrección): 2 h
- Paso 7 (publishEkfPose + covarianza): 0.75 h
- Paso 8 (launch) + integración/tuning: 1 h
