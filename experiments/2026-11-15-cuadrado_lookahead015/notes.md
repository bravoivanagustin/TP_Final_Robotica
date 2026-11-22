# Notas de la corrida — cuadrado_lookahead015

## Contexto
Barrido del parámetro `lookahead_distance` del `KinematicHolonomicController` para
aislar cuánto del gap odom↔GT visto en el baseline (lookahead=0.30 m) se debe a la
magnitud de velocidad comandada y cuánto a otros efectos (slip de rotación en el
lugar durante pre-alineación, slip lateral en las esquinas).

Esta corrida usa `lookahead_distance=0.15 m` (mitad del baseline). Bajar lookahead
baja el error de posición "visto" por la ley P → baja `vx/vy` comandado → debería
bajar el slip lateral Mecanum. Trade-off esperado: la trayectoria se hace más lenta
y aparece algo de oscilación en los tramos rectos (el goal nunca se adelanta mucho
del robot, entonces la ley P modula más fino).

## Setup
<!-- Sim en Play, RViz abierto, mismo escena `coppeliaSim/omni_ekf.ttt`. -->

## Observaciones cualitativas
<!-- Rellenar después de correr. -->

## Análisis cuantitativo
<!-- Comparar vs baseline:
  - plots/xy_path.png: ¿el GT está más cerca del cuadrado?
  - plots/body_velocity.png: ¿vx/vy máximos cercanos a 0.12 m/s?
  - plots/wheel_velocities.png: ¿gap ω_cmd vs ω_med menor?
  - plots/tracking_error.png: ¿picos de las esquinas más chicos o más grandes?
-->

## Conclusiones y próximos pasos
<!-- Si el gap baja proporcional a la velocidad comandada → confirma hipótesis de slip
     proporcional a cmd. Si no → el slip es dominado por otro efecto (p. ej. pre-alineación). -->
