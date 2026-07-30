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

## Los tres bugs

Ninguno es conceptual. Los tres son de implementación, y los tres son del tipo que un revisor cansado no ve.

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

## Discrepancias del modelo pendientes de resolver

- `alpha4`: el Informe de Avance #1 tabula 270 grados, el código usa `1.57079` (90 grados).
- `d5`: `0.2175` en `legacy/youbot_bringup/config/youbot_mechanics.yaml` y en el Informe de Avance, pero `0.113` en el bloque de prueba de `homogeneus_matrix.py`.

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
