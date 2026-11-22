#include <cmath>
#include <limits>
#include <angles/angles.h>
#include "localizer_ekf.h"

/* u = (vx, vy, w) es de dimension 3 en el modelo omnidireccional (anexo de la consigna) */
robmovil_ekf::LocalizerEKF::LocalizerEKF(void) : EKFilter(3, 3, 3, 2, 2)
{
  delta_t = 0;
  association_radius = 0.6;
  sigma_x = sigma_y = 0.02;
  sigma_theta = 0.01;
  sigma_range = 0.01;
  sigma_bearing = 0.005;

  reset(0, 0, 0, 0.01);
}

void robmovil_ekf::LocalizerEKF::reset(double x_init, double y_init, double theta_init, double init_cov)
{
  /* estado inicial */
  Vector x0(3);
  x0(1) = x_init;
  x0(2) = y_init;
  x0(3) = angles::normalize_angle(theta_init);

  /* covarianza inicial */
  Matrix P(3, 3);
  P(1,1) = init_cov;
  P(1,2) = 0;
  P(1,3) = 0;
  P(2,1) = 0;
  P(2,2) = init_cov;
  P(2,3) = 0;
  P(3,1) = 0;
  P(3,2) = 0;
  P(3,3) = init_cov;

  /* inicializa el filtro con estado y covarianza iniciales */
  init(x0, P); /* NOTA: esta llamada utiliza las referencias de x0 y P */
}

void robmovil_ekf::LocalizerEKF::set_noise(double sx, double sy, double sth, double sr, double sb)
{
  sigma_x = sx;
  sigma_y = sy;
  sigma_theta = sth;
  sigma_range = sr;
  sigma_bearing = sb;
}

void robmovil_ekf::LocalizerEKF::set_map(const std::vector<tf2::Vector3>& landmarks)
{
  map_landmarks = landmarks;
}

void robmovil_ekf::LocalizerEKF::set_delta_t(double delta)
{
  delta_t = delta;
}

void robmovil_ekf::LocalizerEKF::set_association_radius(double radius)
{
  association_radius = radius;
}

/** Notificacion de una nueva medicion para su actualizacion.
 *
 *  Se define la variable global correspondence_landmark con las coordenadas cartesianas
 *  del landmark perteneciente al mapa con el que la nueva medicion debera compararse */
bool robmovil_ekf::LocalizerEKF::set_measure(const Vector& new_measure_z)
{
  /* Convertir medicion actual a landmark, en representacion cartesiana y en referencia al frame map */
  tf2::Vector3 landmark_cartesian = measure2landmark(new_measure_z);

  /* Buscar en el mapa el landmark mas cercano a la observacion (establecimiento de correspondencia por nearest-neighbor) */
  return find_corresponding_landmark(landmark_cartesian, correspondence_landmark, association_radius);
}

/** Jacobiano de f con respecto del estado (valores constantes) */
void robmovil_ekf::LocalizerEKF::makeBaseA(void)
{
  A(1,1) = 1;
  A(1,2) = 0;
  A(2,1) = 0;
  A(2,2) = 1;
  A(3,1) = 0;
  A(3,2) = 0;
  A(3,3) = 1;
}

/** Jacobiano de f con respecto del estado
 *  La matriz se actualiza en cada ciclo de actualizacion de tiempo (prediccion),
 *  evaluada en el estado anterior x_t-1 */
void robmovil_ekf::LocalizerEKF::makeA(void)
{
  double theta = x(3);
  double vx = u(1);
  double vy = u(2);

  A(1,3) = -(vx * sin(theta) + vy * cos(theta)) * delta_t;
  A(2,3) =  (vx * cos(theta) - vy * sin(theta)) * delta_t;
}

/** Jacobiano de f respecto de w. El ruido es aditivo sobre el estado: W = I */
void robmovil_ekf::LocalizerEKF::makeBaseW(void)
{
  W(1,1) = 1;
  W(1,2) = 0;
  W(1,3) = 0;
  W(2,1) = 0;
  W(2,2) = 1;
  W(2,3) = 0;
  W(3,1) = 0;
  W(3,2) = 0;
  W(3,3) = 1;
}

/** covarianza de w (ruido en f), por paso de prediccion */
void robmovil_ekf::LocalizerEKF::makeBaseQ()
{
  Q(1,1) = pow(sigma_x, 2);
  Q(1,2) = 0;
  Q(1,3) = 0;
  Q(2,1) = 0;
  Q(2,2) = pow(sigma_y, 2);
  Q(2,3) = 0;
  Q(3,1) = 0;
  Q(3,2) = 0;
  Q(3,3) = pow(sigma_theta, 2);
}

/** Jacobiano de h con respecto del estado (valores constantes) **/
void robmovil_ekf::LocalizerEKF::makeBaseH(void)
{
  H(1,1) = 0;
  H(1,2) = 0;
  H(1,3) = 0;
  H(2,1) = 0;
  H(2,2) = 0;
  H(2,3) = -1;
}

/** Jacobiano de h con respecto del estado
 *  La matriz se actualiza en cada ciclo de actualizacion de mediciones */
void robmovil_ekf::LocalizerEKF::makeH(void)
{
  tf2::Vector3 robot_position(x(1), x(2), 0);

  /* Coordenadas de correspondence_landmark relativas a la posicion del robot (sin rotar) */
  tf2::Vector3 diff_robot_landmark = correspondence_landmark - robot_position;

  if (diff_robot_landmark.length2() < 0.001)
  {
    H(1,1) = H(2,2) = 1;
    H(1,2) = H(2,1) = 0;

    RCLCPP_WARN(rclcpp::get_logger("robmovil_ekf"), "Landmark too close to robot! Fake H used");
  } else {
    double dx = diff_robot_landmark.getX();
    double dy = diff_robot_landmark.getY();
    double q = dx * dx + dy * dy;
    double r = sqrt(q);

    H(1,1) = -dx / r;
    H(1,2) = -dy / r;
    H(2,1) =  dy / q;
    H(2,2) = -dx / q;
  }
}

/** Jacobiano de h respecto de v. El ruido es aditivo sobre la medicion: V = I **/
void robmovil_ekf::LocalizerEKF::makeBaseV(void)
{
  V(1,1) = 1;
  V(1,2) = 0;
  V(2,1) = 0;
  V(2,2) = 1;
}

/** Covarianza de v **/
void robmovil_ekf::LocalizerEKF::makeBaseR()
{
  R(1,1) = pow(sigma_range, 2);
  R(1,2) = 0;
  R(2,1) = 0;
  R(2,2) = pow(sigma_bearing, 2);
}

/** Modelo de movimiento o proceso: x_t = f(x_t-1, u_t-1).
 *
 *  Las velocidades (vx, vy) estan en el frame del robot: se rotan por theta
 *  para integrarlas en el frame map */
void robmovil_ekf::LocalizerEKF::makeProcess(void)
{
  LocalizerEKF::Vector x_old(x); // X_t-1

  double theta = x_old(3);
  double vx = u(1);
  double vy = u(2);
  double w = u(3);

  x(1) = x_old(1) + (vx * cos(theta) - vy * sin(theta)) * delta_t;
  x(2) = x_old(2) + (vx * sin(theta) + vy * cos(theta)) * delta_t;
  x(3) = angles::normalize_angle(x_old(3) + w * delta_t);
}

/** Modelo de sensado: z_t = h(x_t).
 *
 *  Se debe utilizar la variable global correspondence_landmark previamente definida
 *  para definir la variable z con lo que deberia haber medido el sensor */
void robmovil_ekf::LocalizerEKF::makeMeasure(void)
{
  z = landmark2measure(correspondence_landmark);
}

/** La innovacion del bearing es una diferencia de angulos: sin normalizar, una medicion cerca de
 *  +pi contra una prediccion cerca de -pi daria un error de casi 2*pi */
void robmovil_ekf::LocalizerEKF::makeDZ(void)
{
  dz(2) = angles::normalize_angle(dz(2));
}

/** Recibe una medicion de landmark reciente en el frame map y busca el landmark del mapa
 *  mas cercano teniendo un radio delta como umbral
 *
 *  NOTA: El landmark perteneciente al mapa al cual hace referencia el landmark medido debe ser devuelto
 *        por la referencia corresponding_landmark */
bool robmovil_ekf::LocalizerEKF::find_corresponding_landmark(const tf2::Vector3& measured_landmark, tf2::Vector3& corresponding_landmark, float delta_radio)
{
  bool found = false;

  float min_distance = std::numeric_limits<float>::max();

  for (int i = 0; i < map_landmarks.size(); i++)
  {
    float distance = measured_landmark.distance(map_landmarks[i]);

    if (distance < delta_radio && distance < min_distance)
    {
      min_distance = distance;
      corresponding_landmark = map_landmarks[i];
      found = true;
    }
  }

  return found;
}

/** Convierte una medicion en coordenadas polares (relativa al sistema de coordenadas del robot) a un
 * landmark absoluto en el sistema de coordenadas del mundo */
tf2::Vector3 robmovil_ekf::LocalizerEKF::measure2landmark(const LocalizerEKF::Vector& measure)
{
  /* Al utilizar el ultimo estado ESTIMADO para la conversion, el landmark calculado
   * corresponde a la prediccion de la posicion del landmark */

  double rho = measure(1);
  double phi = measure(2);

  double robot_x = x(1);
  double robot_y = x(2);
  double robot_theta = x(3);

  double absolute_angle = phi + robot_theta;

  tf2::Vector3 predicted_landmark;
  predicted_landmark.setX(rho * cos(absolute_angle));
  predicted_landmark.setY(rho * sin(absolute_angle));
  predicted_landmark.setZ(0);

  predicted_landmark = predicted_landmark + tf2::Vector3(robot_x, robot_y, 0);

  return predicted_landmark;
}

/** Convierte un landmark (en coordenadas cartesianas en referencia al mapa) a una medicion (coordenadas polares)
 *  en referencia al sistema de coordenadas del robot */
robmovil_ekf::LocalizerEKF::Vector robmovil_ekf::LocalizerEKF::landmark2measure(const tf2::Vector3& landmark)
{
  /* Pose del robot construida utilizando el estado estimado hasta este momento
   * (prior state estimate) */
  tf2::Transform transform_world_robot;
  transform_world_robot.setOrigin(tf2::Vector3(x(1),x(2),0));
  tf2::Quaternion q;
  q.setRPY(0, 0, x(3));
  transform_world_robot.setRotation(q);

  tf2::Vector3 relative_landmark = transform_world_robot.inverse() * landmark;

  LocalizerEKF::Vector measure(2);
  measure(1) = relative_landmark.length(); // Calculo del rho
  measure(2) = angles::normalize_angle(atan2(relative_landmark.getY(), relative_landmark.getX())); // Calculo del phi

  return measure;
}
