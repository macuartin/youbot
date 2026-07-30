# Fase 2: servo visual de estacionamiento y agarre

Cierra los objetivos específicos 1, 4 y 5 del trabajo de 2018. Reproducible con:

```bash
uv run python experiments/measure_detector_noise.py
uv run python experiments/parking_campaign.py
```

Resultados en `results/`.

## Desviación respecto del plan: no hay MuJoCo

El plan de la Fase 2 decía simular en MuJoCo. No se usa, y conviene decir por qué antes de nada.

El marcador de la estación es plano. Lo que la cámara ve de un plano es exactamente una homografía de su imagen frontal, así que sintetizar la vista es un `warpPerspective` de la imagen del ArUco a los cuatro píxeles proyectados. No hace falta un motor de render, y lo que se obtiene es **más** controlado: no hay iluminación, antialiasing ni materiales metiéndose entre la geometría y lo que se quiere medir. Sobre esas imágenes corre el detector real de OpenCV, así que la extracción de características no está simulada, es la de verdad.

Con eso, la campaña de 500 ensayos con detección real tarda 21 segundos. Renderizando cada fotograma en MuJoCo sería del orden de horas, y los fallos del render se confundirían con los del control.

En la Fase 4 tampoco acabó haciendo falta, y por el mismo razonamiento llevado un paso más allá: la destilación del controlador clásico en una política aprendida exige que maestro y alumno vean **los mismos píxeles**, y esos píxeles son justamente los que produce este renderizador. Meter MuJoCo habría cambiado la observación del alumno respecto de la del maestro, que es lo contrario de lo que un experimento de destilación necesita.

## Qué se implementó

- `youbot/vision.py`: cámara pinhole, geometría del marcador, matriz de interacción de Chaumette y Hutchinson, síntesis de la vista por homografía y detección con `cv2.aruco`.
- `youbot/parking.py`: cinemática de la plataforma omnidireccional, jacobiano del montaje de la cámara, y el lazo IBVS que estaciona.
- `youbot/control.py`: `attempt_grasp`, que resuelve posición más dirección de aproximación de la pinza con `scipy.optimize.least_squares` y cotas articulares.

Sobre el marcador: el objetivo original decía códigos QR. Se usan ArUco, que cumplen las dos funciones que el anteproyecto le pedía al QR (llevar un identificador de estación y aportar esquinas detectables) y son la familia que lo desplazó para estimación de pose. Para el control servo visual, cuatro esquinas de un cuadrado plano, son equivalentes.

## Validación: diferencia finita como oráculo

En la Fase 1 el oráculo era `roboticstoolbox`. Aquí es la diferencia finita: si la matriz de interacción es correcta, predice el cambio de las características de imagen ante un desplazamiento infinitesimal de la cámara. Se comprueban los seis grados de libertad por separado y combinaciones aleatorias, y luego el producto `L·J` completo, que es la matriz que usa la ley de control. Tolerancia 1e-5.

## Ruido del detector, medido y no supuesto

| Ruido de sensor (niveles de gris) | Fallos de detección | Residuo por esquina: media | p95 | max |
|---|---|---|---|---|
| 0 | 0 | 0,429 px | 0,739 | 0,933 |
| 2 | 0 | 0,419 px | 0,714 | 0,906 |
| 5 | 0 | 0,420 px | 0,717 | 1,141 |
| 10 | 0 | 0,425 px | 0,723 | 2,509 |
| 20 | 0 | 0,514 px | 0,889 | 5,916 |

El residuo apenas depende del ruido de sensor: lo domina el sesgo subpíxel del propio detector. En un sistema real la parte sistemática de ese sesgo se cancela, porque la imagen de referencia se captura con el mismo detector. Lo que queda, y es lo que se inyecta en la campaña, es la parte aleatoria: **0,5 px**.

## Objetivo específico 5: estacionamiento

500 ensayos, pose inicial uniforme en una ventana de aproximación de 35 cm en profundidad, 24 cm en lateral y 11,5 grados en guiñada, con 0,5 px de ruido de detección y 2% de error de ejecución en las ruedas.

| Métrica | Valor |
|---|---|
| Ensayos completados | 500 de 500 (ninguno perdió el marcador) |
| Error de posición, media | 1,439 mm |
| Error de posición, p95 | 3,123 mm |
| Error de posición, máximo | 5,341 mm |
| Error de guiñada, máximo | 0,508 grados |
| Pasos hasta detenerse, media | 183,2 |
| Criterio de 2018 (±10 cm) | **CUMPLE**, con casi 19 veces de margen |

### Corrección: el criterio de parada

La primera versión de la campaña reportaba una media de 573,5 iteraciones sobre un límite de 600. Ese número no significaba nada: era "casi todos los ensayos agotaron el tope".

La causa es que el criterio de convergencia era una tolerancia sobre la norma del error de características, fijada en 1e-3. Con 0,5 px de ruido de detección el suelo de ese error es del orden de 3,5e-3, o sea que **la condición de parada era inalcanzable con ruido, por construcción**. Los errores de posición publicados sí eran medidas reales de dónde acababa el robot, y no cambian; lo que no medía nada era el recuento de iteraciones.

La corrección es un criterio de asentamiento, que es el físicamente correcto: la maniobra termina cuando el vehículo deja de moverse. Y su umbral tampoco se elige a ojo. El jitter de la pose en régimen permanente, con el ruido medido, tiene un p95 de **1,75 mm**, así que el umbral tiene que estar en ese orden. Con 1 mm sobre una ventana de 20 pasos, los 500 ensayos terminan por criterio propio en 183 pasos de media.

Lo que se paga por detenerse antes está medido: la ley de control sin regla de parada llega a 0,25 mm, y con la regla de parada se queda en 1,3 mm. Un factor 5 de exactitud a cambio de que la maniobra termine. Sigue siendo 40 veces mejor que el presupuesto de 54 mm que impone la tarea.

## Objetivo específico 4: agarre

Éxito del 100% sobre los 500 ensayos. Criterio de 2018 (95%): **CUMPLE**.

## El resultado que contradice al anteproyecto

Aquí está lo interesante, y no era lo que se esperaba encontrar.

El primer criterio de agarre que se implementó era solo de posición: ¿alcanza el efector final el punto de recogida? Con ese criterio, **un error de estacionamiento de 30 cm sigue permitiendo el agarre**. El brazo tiene la articulación 1 libre en ±169 grados y simplemente gira para alcanzar el punto. La premisa del anteproyecto, que un mal estacionamiento impide la tarea del manipulador, no se sostenía.

No se sostenía porque alcanzar un punto no es agarrar. Una pinza de dos dedos tiene que llegar alineada con la cara de la pieza: si entra torcida, un dedo choca antes de que el otro cierre. Con la dirección de aproximación como restricción, la tarea son cinco restricciones para los cinco grados de libertad del brazo, y aparece el mecanismo real.

En el youBot la articulación 1 es un giro y las 2, 3 y 4 son cabeceos en un plano. El eje de la herramienta solo puede apuntar dentro del plano vertical que define la articulación 1, que es el mismo plano donde tiene que estar el punto de agarre. Un error lateral obliga a la articulación 1 a girar para alcanzar la pieza, y ese giro saca de ese plano la dirección de aproximación que la pieza exige. La desalineación resultante es:

```
desalineación ≈ arctan(error_lateral / alcance_horizontal)
```

Medido contra predicho, la ley acierta con menos de 0,2 grados de discrepancia en todo el rango de 0 a 20 cm. De ahí sale el error lateral máximo tolerable:

```
error_lateral_max = alcance · tan(tolerancia_angular) = 0,307 · tan(10°) = 5,41 cm
```

El barrido lo confirma: el agarre falla a partir de **5,50 cm** de error lateral. Y la sensibilidad a la guiñada es mucho menor, con el límite en 24 grados.

**El criterio de ±10 cm del anteproyecto de 2018 era 1,8 veces más laxo de lo que la tarea de pick & place admite.** Un AGV que cumpliera exactamente la especificación que el propio trabajo se puso fallaría el agarre. La especificación se fijó sobre el estacionamiento aislado, sin derivarla de la tarea que tenía que habilitar.

Que el servo visual la cumpla con 19 veces de margen es lo que salva el resultado: 5,4 mm de error máximo está muy por debajo de los 54 mm que la geometría del brazo permite. Pero eso es suerte del método, no de la especificación.

## Cuenca de atracción

El IBVS no converge desde cualquier parte: si el marcador no está en el campo de visión, no hay lazo que cerrar. Sobre una rejilla de ±30 cm de desviación lateral y ±30 grados de guiñada inicial, converge el 64%. La región es una banda diagonal, que es lo esperable: una desviación lateral acompañada de la guiñada que apunta la cámara hacia la estación mantiene el marcador en cuadro, mientras que lateral y guiñada en sentidos opuestos lo sacan.

Es una limitación que hay que declarar: el servo visual de estacionamiento no sustituye a la navegación, la remata. Necesita que el algoritmo de navegación deje al robot dentro de esa banda.

## Limitaciones

- Todo es cinemático. No hay dinámica de la plataforma, ni deslizamiento de las ruedas mecanum, ni retardo de control. El error de ejecución se modela como ruido multiplicativo del 2% sobre la velocidad comandada.
- El agarre se valida geométricamente: posición del efector final y alineación de la pinza. No hay física de contacto, así que "éxito de agarre" significa que la pinza llega bien colocada, no que la pieza se sostenga.
- La escena tiene un solo marcador, sin oclusiones y sin otros marcadores que confundir.
- El offset del montaje del brazo tiene unos 3 cm sin reconciliar entre el URDF y la tabla DH (ver `youbot/model.py`). Desplaza la pose nominal de agarre, pero no la pendiente de la ley `arctan(e/r)`, que es el resultado.
