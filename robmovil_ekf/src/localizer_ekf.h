#ifndef __ROBMOVIL_EKF_LOCALIZEREKF_H__
#define __ROBMOVIL_EKF_LOCALIZEREKF_H__

#include <rclcpp/rclcpp.hpp>
#include <kalman/ekfilter.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace robmovil_ekf
{
  typedef Kalman::EKFilter<double,1,false,false,false> EKFilter;

  /**
   * Localizador EKF para la plataforma omnidireccional (Mecanum)
   *
   * Estado: X = (x, y, theta) en el frame map
   * Input: U = (vx, vy, w) velocidades en el frame del robot (twist de /robot/odometry)
   * Medicion: Z = (r, phi) de un poste, en el frame del robot
   * Se incluyen dos ruidos aditivos para modelo de proceso y sensado, respectivamente
   */
  class LocalizerEKF : public EKFilter
  {
    public:
      LocalizerEKF(void);

      /* Jacobiano de f() respecto a X */
      void makeBaseA(void);
      void makeA(void);

      /* Jacobiano de f() respecto a W */
      void makeBaseW(void);

      /* Covarianza de f() */
      void makeBaseQ(void);

      /* Jacobiano de h() con respecto a X */
      void makeBaseH(void);
      void makeH(void);

      /* Jacobiano de h() con respecto a V */
      void makeBaseV(void);

      /* Covarianza de h() */
      void makeBaseR(void);

      /* modelo de proceso */
      void makeProcess(void);

      /* modelo de sensado */
      void makeMeasure(void);

      /* normaliza la componente angular de la innovacion */
      void makeDZ(void);

      /* Desvios estandar de los ruidos. Tiene que llamarse antes del primer paso del filtro,
       * porque makeBaseQ() y makeBaseR() se evaluan una unica vez */
      void set_noise(double sigma_x, double sigma_y, double sigma_theta,
                     double sigma_range, double sigma_bearing);

      /* Reinicia el filtro con la pose x0 = (x, y, theta) y covarianza P0 = init_cov * I */
      void reset(double x, double y, double theta, double init_cov);

      /* Mapa de postes en coordenadas cartesianas del frame map (conocido a priori, desde /posts) */
      void set_map(const std::vector<tf2::Vector3>& landmarks);
      bool has_map(void) const { return !map_landmarks.empty(); }

      void set_delta_t(double delta); // seconds
      void set_association_radius(double radius); // metros
      bool set_measure(const Vector& measured_z);

    private:

      /* Encuentra el landmark del mapa mas cercano al provisto y lo devuelve por corresponding_landmark */
      bool find_corresponding_landmark(const tf2::Vector3& measured_landmark, tf2::Vector3& corresponding_landmark, float delta);

      /* Convierte una medicion en coordenadas polares (relativa al sistema de coordenadas del robot) a un
       * landmark absoluto en el sistema de coordenadas del mundo */
      tf2::Vector3 measure2landmark(const Vector& measure);

      /* Convierte un landmark (en coordenadas cartesianas en referencia al mapa) a una medicion (coordenadas polares)
       *  en referencia al sistema de coordenadas del robot */
      LocalizerEKF::Vector landmark2measure(const tf2::Vector3& landmark);

      /* Mapa de landmarks en coordenadas cartesianas del frame map */
      std::vector<tf2::Vector3> map_landmarks;

      /* Delta tiempo desde la ultima prediccion (time update) */
      double delta_t;

      /* Radio de busqueda de la asociacion por nearest-neighbor */
      double association_radius;

      /* Desvios estandar de w (proceso) y v (sensado) */
      double sigma_x, sigma_y, sigma_theta;
      double sigma_range, sigma_bearing;

      /* Landmark del mapa (en coordenadas cartesianas del frame map) el cual corresponde
       * con la ultima medicion ingresada (por set_measure) */
      tf2::Vector3 correspondence_landmark;
  };
}

#endif // __ROBMOVIL_EKF_LOCALIZEREKF_H__
