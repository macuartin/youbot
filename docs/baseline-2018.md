# Baseline 2017-2020

Estado congelado del trabajo original, para poder comparar contra la reconstrucción de 2026. Todas las cifras salen de documentos que existen, no de recuerdos.

## El trabajo

**Título**: Arquitectura de control visual de estacionamiento de precisión para un robot móvil en labores de pick & place.

**Institución**: Pontificia Universidad Javeriana, Bogotá. Maestría en Ingeniería Electrónica. Trabajo de Investigación de Maestría No. 1738.

**Director**: Julián David Colorado Montaño, Ph.D. **Codirector**: Iván Fernando Mondragón Bernal, Ph.D.

**Objetivo general**: desarrollar e implementar una arquitectura de control visual de estacionamiento de precisión para un robot móvil (AGV/manipulador) en labores de pick & place.

El robot es un KUKA youBot: plataforma omnidireccional mecanum más brazo manipulador de 5 grados de libertad, todos rotacionales, con pinza de dos dedos. El entorno de validación previsto era el sistema de manufactura flexible del Centro Tecnológico de Automatización Industrial (CTAI) de la misma universidad.

## Cronología

| Fecha | Hito | Fuente |
|---|---|---|
| 2017-12-07 | Anteproyecto presentado al comité | Anteproyecto V2 |
| 2018 (1er periodo) | Anteproyecto aprobado | Aprobación Anteproyecto |
| 2018-08-05 | Plan de trabajo semestral firmado por estudiante y director | Plan de Trabajo fdo. |
| 2018-10-13 | Informe de Avance #1 | Informe de Avance #1.docx |
| 2020-03-16 | Primer commit del repo `macuartin/youbot` | git log |
| 2020-10-08 | Último commit del repo | git log |

Elapsed entre el anteproyecto y el último commit: **34 meses**. Duración planificada en el anteproyecto: 36 semanas.

## Objetivos específicos y estado real al abandonar

El plan de trabajo firmado en agosto de 2018 declara el avance auto-reportado. La columna de la derecha es el estado verificado en 2026 leyendo el código que quedó.

| # | Objetivo | Auto-reportado ago-2018 | Verificado 2026 |
|---|---|---|---|
| OE1 | Control visual del AGV por extracción de características con códigos QR | 0% | 0%. No existe ni una línea de visión por computadora en ninguno de los dos repos. |
| OE2 | Modelo cinemático y dinámico del brazo por formulación Newton-Euler | 50% | Escrito casi entero, nunca validado. La dinámica directa no puede ejecutarse. |
| OE3 | Control de posición articular por cinemática inversa método Jacobiano | 0% | Escrito, pero el jacobiano devuelve ceros por un bug de tipos. |
| OE4 | Validación pick & place, 95% de éxito en el agarre | 0% | 0% |
| OE5 | Validación del estacionamiento en entorno industrial, error espacial ±10 cm | 0% | 0% |

**Objetivos validados: 0 de 5.**

## Producción de código

| Repo | Líneas Python | Commits | Rango |
|---|---|---|---|
| `macuartin/youbot` | 782 | 38 | 2020-03-16 a 2020-10-08 |
| `macuartin/newton-euler-robotic-model` | 190 | 2 | 2020-09 |
| **Total** | **972** | **40** | |

El código de 2018-2019 no sobrevive en control de versiones. Lo que hay en `legacy/` es la reescritura de 2020 sobre ROS 1 Kinetic y Python 2.7.

## Los cinco bugs

Ninguno es conceptual. Los cinco son de implementación, y los cinco son silenciosos: el programa corre y devuelve números. Los tres primeros se detectaron leyendo el código; el cuarto y el quinto aparecieron al reconstruir el modelo con la documentación de referencia en la mano.

### 1. El jacobiano en dtype entero

`legacy/youbot_mechanics/src/youbot_mechanics/jacobian.py`

```python
J = np.matrix([[0] * DoF] * 6)
```

Una lista de enteros de Python produce una matriz NumPy de dtype `int64`. Las seis asignaciones siguientes son valores en coma flotante, típicamente entre -1 y 1, y NumPy las trunca hacia cero al escribirlas.

Verificado:

```
>>> J = np.matrix([[0]*5]*6); J.dtype
dtype('int64')
>>> J[0,0] = -0.1234; J[0,0]
0
```

El jacobiano salía casi entero en ceros. Su pseudo-inversa, y por lo tanto toda la cinemática inversa de OE3, era ruido. No hay ningún error en pantalla: el programa corre y devuelve números.

**Fix**: `np.zeros((6, DoF))`.

### 2. La matriz de masa mal formada

`legacy/youbot_mechanics/src/youbot_mechanics/forward_dynamic.py`

La dinámica directa arma la matriz de masa llamando a `inverse_dynamic`, pero esa función solo escribe `tau[i, 0]`, o sea devuelve un vector columna (5,1) y no una matriz (5,5). El `np.linalg.inv(M)` de la línea siguiente no puede funcionar. Concuerda con que la sección "Algoritmo de Dinámica Directa" del Informe de Avance #1 quedó en blanco.

**Fix**: construir la matriz de masa por columnas de aceleración unitaria (método CRBA).

### 3. Aceleración articular contada dos veces

`legacy/youbot_mechanics/src/youbot_mechanics/inverse_dynamic.py`

En la propagación de aceleraciones, el término `np.dot(H, Qdd[i])` aparece sumado dos veces en la misma expresión: una vez directo y otra bajo el nombre `centripeta`.

```python
centripeta = np.dot(H, Qdd[i])
vd = np.dot(...) + np.dot(H, Qdd[i]) + coriolis + centripeta + centrifuga
```

Los torques salen sesgados en proporción a la aceleración articular. El error es silencioso: los números parecen razonables.

### 4. Centros de masa en centímetros usados como metros

`legacy/youbot_mechanics/src/youbot_mechanics/dynamics_operators.py`, función `centerOfMassDistance`.

La tabla de parámetros físicos del Informe de Avance #1 tabula los centros de masa sin declarar unidades. El eslabón 2 aparece con `Sx = 11.397`. Leído como metros, eso pone el centro de masa de un eslabón a once metros de su articulación, en un brazo cuya longitud total extendida es de 65,45 cm. La única lectura físicamente posible es centímetros: así los cinco eslabones caen dentro de su propia geometría.

El código toma los valores crudos:

```python
s = np.array([Params[0, 1], Params[0, 2], Params[0, 3]])
```

o sea con un factor 100 de error en los brazos de palanca de toda la dinámica.

### 5. `a` y `alpha` transpuestos entre la tabla y quien la consume

`legacy/youbot_mechanics/src/youbot_mechanics/homogeneus_matrix.py`

La función lee `alpha = DHi[2]` y `a = DHi[3]`, o sea espera filas en orden `(theta, d, alpha, a, sigma)`. Pero la tabla que le pasan, tanto en su propio bloque de prueba como en `legacy/youbot_bringup/config/youbot_mechanics.yaml`, está en orden `(theta, d, a, alpha)`:

```python
DH = [[0, 0.147, 0.033, np.pi/2, 0], ...]
```

Con esa transposición el eslabón 1 queda con `alpha = 0.033 rad` y `a = 1.5708 m`: un brazo de metro y medio. La forma de la matriz homogénea que construye la función es correcta; lo que está mal es el orden en que se la alimenta.

El repo `newton-euler-robotic-model`, escrito en septiembre de 2020 con SymPy, sí es consistente: declara `[type, theta, d, a, alpha]` y lee `alpha = DHi[4]`, `a = DHi[3]`. La derivación simbólica estaba bien; la numérica no.

### Bug latente: mezcla de frames en la cinemática inversa

`inverse_kinematic.py` resuelve `pinv(J) @ Xdif + Q0` usando el jacobiano en frame del efector final (formulación de Paul), pero `Xdif` viene del planificador de trayectorias, que trabaja en coordenadas cartesianas de la base. Son dos frames distintos. Nunca llegó a manifestarse porque el servicio ROS que debía llamar a esta función se quedó en `return True`.

## Discrepancias del modelo, resueltas

Ambas se zanjan contra [kirillin/youbot_arm_kinematics](https://github.com/kirillin/youbot_arm_kinematics), que publica `DH_A = (0.033, 0.155, 0.135, 0, 0)`, `DH_ALPHA = (pi/2, 0, 0, pi/2, 0)` y `DH_D = (0.147, 0, 0, 0, 0.218)`.

- `alpha4`: el Informe de Avance #1 tabula 270 grados y el código usa 90. **Gana el código**: la referencia publica `pi/2`. Los 270 grados son un error de transcripción del informe.
- `d5`: **0,2175 m**, no los 0,113 del bloque de prueba de `homogeneus_matrix.py`. Hay además una comprobación aritmética independiente que no necesita fuente externa: `0.147 + 0.155 + 0.135 + 0.2175 = 0.6545`, exactamente la longitud extendida de 65,45 cm que declara el propio Informe de Avance.

## Detalle de época

El generador de trayectorias de 2020 ya no ejecuta en NumPy 2: hace `traj[j+p, x] = ax[i] * ...` donde `ax[i]` es un array de forma `(1,)`, y asignar una secuencia a una posición escalar dejó de estar permitido. No es un error de formulación, es código que envejeció.

## Veredicto 2026

La pregunta que quedó abierta en 2018 era si la matemática del Informe de Avance era correcta. La respuesta, ahora verificada: **sí**.

El modelo reconstruido en `youbot/` coincide con `roboticstoolbox-python` (Peter Corke) como oráculo independiente a tolerancia 1e-9, sobre 25 configuraciones articulares aleatorias, en cinemática directa, jacobiano en frame base, jacobiano en frame del efector, torques de gravedad, matriz de masa y Newton-Euler completo con velocidad y aceleración no nulas. El generador de trayectorias de 2020 coincide coeficiente a coeficiente con `scipy.interpolate.CubicSpline`.

La formulación de 2018 era correcta. Lo que falló fueron cinco errores de implementación, ninguno de los cuales produce un mensaje de error.

Nota metodológica honesta: la reconstrucción de 2026 también tuvo su bug, y del mismo tipo. La primera versión de la dinámica mezcló la recursión de Newton-Euler de Craig, que asume DH modificada con el eje de la articulación en Z_i, con una cinemática en DH clásica, donde el eje está en Z_{i-1}. La diferencia está en tres detalles: dónde se suma la velocidad articular, en qué frame se expresa el vector entre orígenes, y sobre qué eje se proyecta el torque. Un brazo de prueba de dos eslabones con masas puntuales, cuya matriz de masa se calcula a mano en dos líneas, lo delató en el primer intento: I11 debía valer 5 y valía 1. La diferencia con 2018 no es que ahora no se cometan errores. Es que el error duró minutos en vez de años, porque había un oráculo contra el que comparar y un caso analítico que lo acorrala.

## Coste declarado

Presupuesto del anteproyecto: **$102.600.000 COP** (del orden de USD 34.000 al cambio de 2017).

| Ítem | Valor COP |
|---|---|
| Robot KUKA youBot | 60.000.000 |
| Cámara industrial uEye CP | 15.000.000 |
| Director | 10.800.000 |
| Codirector | 10.800.000 |
| Ingeniero investigador | 4.320.000 |
| Computadora | 1.500.000 |
| Router inalámbrico | 180.000 |

A eso hay que sumarle lo que el presupuesto no contabiliza y fue el coste real: presencia física en el laboratorio del CTAI, disponibilidad del robot, y sincronía con el horario de los directores.

## Por qué se abandonó

No fue técnico. La universidad y el director le plantearon al estudiante que trabajar y estudiar a la vez no era viable, en un momento en que dejar de trabajar no era una opción disponible. El trabajo se detuvo en octubre de 2020.

## Nota sobre los documentos originales

Los PDF y DOCX originales (anteproyecto, carta de aprobación, plan de trabajo firmado, informe de avance) **no se versionan en este repo**: contienen documento de identidad, teléfono y dirección particular. Se conservan fuera del repositorio. Las cifras de este documento se pueden contrastar contra ellos a petición.
