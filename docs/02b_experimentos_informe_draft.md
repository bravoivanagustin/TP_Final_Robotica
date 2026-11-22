# Borrador — Subsección "Experimentos" del Módulo 2 (informe TeX)

Draft reusable del contenido para la subsección `\subsection{Experimentos}` de
`\section{Control a lazo cerrado y seguimiento de trayectorias}` del informe
(`informe/Informe TP Final.tex`, actualmente con TODOs en líneas 238-243). Este
archivo **no toca el TeX** — es material para copiar/adaptar cuando se redacte.

Sidecar del deep-dive `docs/02_lazo_cerrado.md`. La implementación del
controlador, la estrategia pursuit y la trayectoria "opuesta al centro" están
cubiertas por las subsecciones 3.1 y 3.2 del TeX (ya escritas). Acá sólo se
cierra el loop con los experimentos y la justificación de las constantes.

## Tabla de cambios de este draft

| Fecha | Cambio | Motivo |
|---|---|---|
| 2026-11-21 | Primera versión del draft tras barrido Kp en escena v2 | Cerrar módulo 2 antes de pasar a módulo 3 |

---

## Diseño del barrido

La consigna §2 pide variar las constantes del controlador P "al menos 3 valores
cada una" y justificar la elección final. Se armaron entonces dos barridos
univariados:

- **Barrido de `kp_xy`** — valores `{0.4, 0.8, 1.6}` manteniendo `kp_yaw = 1.5`
  y `lookahead_distance = 0.30`. Mueve la velocidad traslacional comandada
  para un error de posición dado.
- **Barrido de `kp_yaw`** — valores `{0.75, 1.5, 3.0}` manteniendo `kp_xy = 0.8`
  y `lookahead_distance = 0.30`. Mueve la velocidad angular comandada para un
  error de orientación dado.

Ambos barridos comparten el punto central (`kp_xy=0.8`, `kp_yaw=1.5`,
`lookahead=0.30`), que se corre una sola vez como baseline.

Dos corridas adicionales fuera de lo que exige la consigna, incluidas para dar
contexto al barrido de Kp:

- **Barrido de `lookahead_distance`** — valores `{0.15, 0.60}`. Dado que el
  pursuit elige el waypoint objetivo a `lookahead_distance` del robot, variar
  este parámetro cambia la velocidad comandada por una vía distinta a `kp_xy`.
  Lo incluimos como control para distinguir "efecto de velocidad" de "efecto
  del ciclo de pursuit".
- **Best combo** — `lookahead = 0.15` + `kp_yaw = 3.0`. Combinación de las dos
  mejores palancas individuales, para testear si los efectos son aditivos.

Cada corrida ejecuta la trayectoria cuadrada completa (16 m de perímetro) sobre
la escena `coppeliaSim/omni_ekf_v2.ttt`. La métrica se computa en post-proceso
con `scripts/plot_run.py` (ver `experiments/README.md`).

### Qué se mide

Más informativo que el error pose-vs-setpoint que pide estrictamente la
consigna es el **error GT vs odometría**: la diferencia entre la pose real del
simulador (publicada en `odom → base_link_gt`, compuesta con `map → odom` para
llegar a `map`) y la pose estimada por el módulo 1 (publicada en
`odom → base_link`). Esta diferencia **exhibe el deslizamiento físico** de los
rodillos Mecanum, que es invisible para la odometría basada en encoders (las
ruedas rotan sin deslizar en su eje principal, pero los rodillos sí deslizan
en su eje libre — ver §Modelo Cinemático). Es el límite teórico de precisión
del módulo 1 y lo que motiva al módulo 3.

Las métricas que se reportan por corrida:

- `drift_fin` — módulo del error GT-odom al final de la corrida.
- `drift_med` — mediana del error a lo largo de toda la corrida. Más robusto
  que el promedio ante outliers de las esquinas.
- `drift_max` — error máximo observado.
- `yaw_err_fin` — error angular GT-odom al final, en grados.
- `duracion` — tiempo de simulación total hasta completar la trayectoria.

---

## Resultados

| Configuración                 | `kp_xy` | `kp_yaw` | `lookahead` | drift_fin | drift_med | drift_max | yaw_err_fin | duración |
|-------------------------------|---------|----------|-------------|-----------|-----------|-----------|-------------|----------|
| baseline                      | 0.8     | 1.5      | 0.30        | 1.158 m   | 0.885 m   | 1.158 m   | 27.1°       | 241 s    |
| `kp_xy = 0.4` (kpxy04)        | 0.4     | 1.5      | 0.30        | 0.899 m   | **0.601 m** | 0.900 m | 19.7°       | 421 s    |
| `kp_xy = 1.6` (kpxy16)        | 1.6     | 1.5      | 0.30        | 1.599 m   | 1.597 m   | 1.892 m   | 40.6°       | 181 s    |
| `kp_yaw = 0.75` (kpyaw075)    | 0.8     | 0.75     | 0.30        | 1.158 m   | 1.071 m   | 1.159 m   | 27.6°       | 301 s    |
| **`kp_yaw = 3.0` (kpyaw30)**  | 0.8     | 3.0      | 0.30        | **0.869 m** | 0.680 m | **0.869 m** | **19.2°** | 241 s    |
| `lookahead = 0.15`            | 0.8     | 1.5      | 0.15        | 1.047 m   | 0.660 m   | 1.047 m   | 23.3°       | 362 s    |
| `lookahead = 0.60`            | 0.8     | 1.5      | 0.60        | 1.725 m   | 1.527 m   | 1.840 m   | 42.9°       | 152 s    |
| bestcombo                     | 0.8     | 3.0      | 0.15        | 1.054 m   | 0.692 m   | 1.054 m   | 23.1°       | 391 s    |

Todas las corridas **cierran la trayectoria por odometría** (el controlador
alcanza el último waypoint dentro de las tolerancias de posición y yaw). La
diferencia entre corridas está en **dónde queda físicamente** el robot según
el GT.

---

## Análisis

### Efecto de `kp_xy` (ganancia traslacional)

El barrido confirma la intuición de que velocidades traslacionales más altas
generan más slip lateral en los rodillos Mecanum:

- `kp_xy = 0.4` vs. baseline (`0.8`): drift_med baja de 0.88 m a 0.60 m (−32%),
  drift_fin de 1.16 m a 0.90 m (−22%). El costo es el tiempo: la corrida tarda
  420 s vs. 240 s del baseline.
- `kp_xy = 1.6` vs. baseline: todas las métricas se degradan ~40-60%,
  yaw_err_fin salta de 27° a 40°, el GT **se sale del cuadrado de referencia**
  por sobreimpulso en las esquinas.

### Efecto de `kp_yaw` (ganancia angular)

El efecto es **asimétrico** — subirlo ayuda, bajarlo casi no se nota:

- `kp_yaw = 0.75` vs. baseline: drift_fin idéntico (1.16 m en ambos), pero
  drift_med sube de 0.88 a 1.07 m (+21%). El robot tarda más en alinear la
  orientación en cada esquina, lo cual acumula error durante la trayectoria
  pero termina en la misma zona del cuadrado.
- `kp_yaw = 3.0` vs. baseline: drift_fin cae de 1.16 a 0.87 m (−25%), drift_med
  de 0.88 a 0.68 m (−23%), yaw_err_fin de 27° a 19° (−29%). Es la mejor
  configuración univariada del barrido.

**Interpretación física**: con `kp_yaw` alto el robot pasa menos tiempo en el
régimen "rota mientras traslada", que es donde el slip Mecanum es peor (los
rodillos del lado interno del giro tienen que deslizar mucho más que los del
lado externo para mantener la consigna cinemática). Alcanzar el yaw objetivo
rápido hace que durante el pursuit de cada lado el error angular sea chico y
`wz` quede chico también.

### Equivalencia entre `lookahead_distance` y `kp_xy`

Las dos palancas controlan la velocidad comandada por caminos distintos: la
primera mueve el waypoint objetivo (por lo cual el error de posición es más
chico), la segunda escala el error. Los resultados confirman que son
cualitativamente equivalentes:

- `lookahead = 0.15` ≈ `kp_xy = 0.4`: drift_fin 1.05 m vs. 0.90 m.
- `lookahead = 0.60` ≈ `kp_xy = 1.6`: drift_fin 1.73 m vs. 1.60 m — ambos
  degradan el sistema de la misma manera.

Esto implica que para controlar la velocidad traslacional del robot da igual
cuál de las dos se toque; se elige `kp_xy` por ser la constante natural del
controlador P.

### El slip Mecanum no escala linealmente con la velocidad

Comparando las dos corridas "patológicas" (`kp_xy = 1.6`, `lookahead = 0.60`)
contra el baseline: la velocidad comandada es aproximadamente el doble, pero
el drift_fin aumenta ~1.5× (de 1.16 m a 1.6-1.7 m) **y** el yaw_err salta de
27° a 41-43° (+54%). Los `xy_path` muestran que el GT a alta velocidad se
escapa del cuadrado de referencia por sobreimpulso — el controlador comanda
giros bruscos en las esquinas que los rodillos no pueden ejecutar sin deslizar
masivamente. Es evidencia experimental del acoplamiento no lineal entre
rotación y traslación en la cinemática Mecanum.

### El "best combo" no suma sus mejoras

La hipótesis previa era que `kp_yaw = 3.0` y `lookahead = 0.15` atacan
mecanismos de slip ortogonales (rotacional y traslacional) y por lo tanto
combinados darían un drift menor que cualquiera de los dos solos. Los datos
desmienten esa hipótesis:

- `kp_yaw = 3.0` solo: drift_fin = 0.87 m.
- `lookahead = 0.15` solo: drift_fin = 1.05 m.
- Combinado: drift_fin = 1.05 m — **igual al lookahead solo, peor que kp_yaw solo**.

La lectura es que los dos tipos de slip están acoplados por la cinemática y no
son independientes: bajar la velocidad traslacional hace que `kp_yaw` alto no
tenga oportunidad de sacar ventaja porque el error angular transitorio es
mucho más chico desde el arranque.

---

## Justificación del tune final

**Configuración elegida: `kp_xy = 0.8, kp_yaw = 3.0, lookahead = 0.30`.**

Fundamentos:

1. **Mejor drift_fin del barrido** (0.87 m, −25% vs. baseline) y menor error
   angular (19°, −29% vs. baseline).
2. **Preferimos `kp_yaw = 3.0` sobre `kp_xy = 0.4`** aunque ambos tengan drift
   comparable, porque el primero no agrega tiempo de corrida (240 s vs. 420 s
   del `kp_xy = 0.4`). Doblar la duración por 0.08 m de ganancia en drift_med
   es un ratio pobre.
3. **Rechazamos el bestcombo** porque los datos muestran que no mejora sobre
   `kp_yaw = 3.0` solo.

El drift residual elegido es **0.87 m sobre 16 m de perímetro = 5.4%**,
consistente con los números que reporta la literatura para plataformas Mecanum
sin fusión sensorial (5-15%). Representa la cota de precisión del módulo 1
(odometría pura basada en encoders) y motiva la introducción del módulo 3
(EKF con landmarks del laser), que se espera comprima el drift un orden de
magnitud al corregir la pose contra referencias externas al robot.

---

## Figuras a incluir en el TeX

| # | Ruta | Contenido propuesto para la figura |
|---|---|---|
| 1 | Grid 2×4 con los 8 `xy_path.png` de las corridas v2 | Vista comparativa del efecto de cada tune sobre la trayectoria física |
| 2 | `experiments/2026-11-21-cuadrado_v2_kpyaw30/plots/goal_pose_vs_pose.png` | Ganador: pose real vs. setpoint pursuit en el tiempo |
| 3 | `experiments/2026-11-21-cuadrado_v2_kpyaw30/plots/body_velocity.png` | Ganador: `vx`, `vy`, `wz` publicados en `/robot/cmd_vel` |
| 4 | `experiments/2026-11-21-cuadrado_v2_kpyaw30/plots/tracking_error.png` | Ganador: error de pose en el tiempo |
| 5 | `experiments/2026-11-21-cuadrado_v2_kpxy16/plots/xy_path.png` | Caso patológico: el GT se sale del cuadrado |

La figura 1 es la más informativa — un grid con los 8 trayectos superpuestos
cada uno sobre la referencia hace evidente qué palancas rompen el sistema y
cuáles lo mejoran. Se puede armar con `matplotlib.subplots(2, 4)` leyendo los
bags directamente, o componiendo los PNGs ya generados.

---

## Pendientes antes de volcar al TeX

- Reemplazar los 4 TODOs de las líneas 238-243 por el contenido correspondiente
  de las secciones "Diseño del barrido", "Resultados", "Análisis" y
  "Justificación del tune final" de este documento.
- Generar la figura 1 (grid 2×4 de `xy_path`) — no existe como PNG único,
  requiere un script ad-hoc o una composición manual en el TeX con subfigure.
- Ajustar nombres de figuras (`\label`, `\caption`) y rutas relativas a
  `informe/`.
- Si se prefiere la tabla con otro ordenamiento (por drift ascendente en lugar
  de por tipo de barrido), reorganizar al pasarla al TeX.
