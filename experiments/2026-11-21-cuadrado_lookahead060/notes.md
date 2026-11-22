# Notas de la corrida — cuadrado_lookahead060

## Contexto
Barrido del parámetro `lookahead_distance` del `KinematicHolonomicController`.

Esta corrida usa `lookahead_distance=0.60 m` (doble del baseline). Subir lookahead
mete el goal mucho más adelante del robot → el error de posición "visto" por la ley
P es más grande → `vx/vy` comandado saturado por `kp_xy · error` queda cerca de
0.48 m/s pico. Hipótesis: más velocidad lineal → más slip lateral Mecanum → el GT
debería estar más adentro del cuadrado que en el baseline, y las esquinas van a
cortarse más (el cursor del pursuit avanza a waypoints del lado siguiente antes
de que el robot haya llegado a la esquina).

## Setup
<!-- Sim en Play, RViz abierto, mismo escena `coppeliaSim/omni_ekf.ttt`. -->

## Observaciones cualitativas
<!-- Rellenar después de correr. -->

## Análisis cuantitativo
<!-- Comparar vs baseline:
  - plots/xy_path.png: ¿el GT corta más las esquinas?
  - plots/body_velocity.png: ¿vx/vy máximos cercanos a 0.48 m/s?
  - plots/wheel_velocities.png: ¿gap ω_cmd vs ω_med más grande?
  - plots/tracking_error.png: picos en las esquinas; ¿odom cierra o se queda corto?
-->

## Conclusiones y próximos pasos
<!-- Si GT se desvía MÁS que el baseline y proporcional al aumento de velocidad →
     slip Mecanum es ~lineal con velocidad comandada (confirma la hipótesis de
     que bajar el gap requiere el EKF del módulo 3, no tunear el controller). -->
