# Fase 4: destilar el controlador clásico en una política aprendida

Reproducible con:

```bash
uv run --group vla python experiments/generate_demonstrations.py --episodes 120
uv run --group vla python experiments/train_policy.py --steps 3000
uv run --group vla python experiments/evaluate_policy.py --trials 40
```

## La pregunta

El survey de la Fase 3 respondió la pregunta original con cifras publicadas: un VLA generalista no alcanza la precisión que exige esta tarea. Pero esa comparación tiene un problema, y está declarado en el propio survey: las dos columnas no se midieron en el mismo banco.

Esta fase mide las dos en el mismo banco, con una pregunta más estrecha y más limpia:

> **¿Cuánta precisión se pierde al sustituir una ley de control analítica por una red entrenada sobre sus propias demostraciones, viendo los dos exactamente los mismos píxeles?**

No es la pregunta de si un VLA generaliza mejor. Este banco no puede responder eso y no lo pretende: la escena tiene un solo marcador, sin oclusiones, sin fondo y con una única pose de estacionamiento. La generalización es la fortaleza del VLA y aquí no se está probando.

## Cómo se resolvió el bloqueante de los datos

El survey identificaba los datos como el obstáculo: la documentación de SmolVLA recomienda del orden de 50 episodios teleoperados de la tarea concreta, y construir eso era un proyecto en sí mismo.

Ese obstáculo no existía. Las Fases 1 y 2 dejaron un controlador que resuelve la tarea, y **un controlador que funciona es un experto que genera demostraciones solo**. 120 episodios, 24.431 fotogramas, en 6,6 MB codificados en vídeo. Sin teleoperar nada.

Es clonación de comportamiento a partir de un experto algorítmico, que es un montaje estándar. Lo que lo hace interesante aquí es que el experto no es una heurística: es la reconstrucción validada de un trabajo de 2018, con su error caracterizado.

## Las tres decisiones que sostienen el experimento

**1. Maestro y alumno ven los mismos píxeles.** El experto corre con `cv2.aruco` dentro del lazo, sobre exactamente la misma imagen que se graba como observación, y no sobre una proyección geométrica con ruido añadido. Sin esto la destilación no significa nada, porque el alumno estaría aprendiendo a imitar decisiones tomadas con información que él no recibe.

Efecto secundario útil: eso valida de forma independiente el modelo de ruido de la Fase 2. Con ruido modelado de 0,5 px el error medio es 1,218 mm; con el detector real en el lazo, 1,492 mm. El mismo orden, así que los 0,5 px medidos eran un modelo fiel.

**2. La misma cámara para los dos, a 256x192.** Esta es la decisión que más fácilmente se hace mal. Si el experto viera 640x480 y la política 256x192, la comparación mediría resolución y se presentaría como si midiera aprendizaje.

La resolución se eligió con un barrido, no por comodidad:

| Resolución | Lado del marcador | Error medio del experto | Pasos |
|---|---|---|---|
| 640x480 | 133 px | 1,43 mm | 185 |
| 320x240 | 67 px | 5,32 mm | 208 |
| **256x192** | **53 px** | **4,83 mm** | **203** |
| 224x168 | 47 px | 9,29 mm | no se asienta |
| 160x120 | 33 px | 9,30 mm | no se asienta |

La precisión del experto se degrada con la resolución, y por debajo de 256x192 deja de asentarse: el marcador baja de 50 px y las esquinas subpíxel dejan de ser fiables. 256x192 es el punto más bajo que todavía resuelve la tarea, o sea el más exigente que se le puede pedir a la política sin regalarle píxeles.

Consecuencia: **el listón de la política no es el 1,43 mm de la Fase 2 sino el error del experto a esta resolución**, medido sobre las mismas 120 demostraciones en 5,84 mm de media y 9,25 mm máximo.

**3. La política no recibe la pose de la plataforma.** Dársela sería resolver la tarea sin mirar, que es exactamente lo contrario de por qué existe el servo visual basado en imagen: el robot no conoce su pose respecto de la estación. Como estado recibe la acción del paso anterior, que es lo único que un robot real sí conoce de sí mismo.

## Tampoco hace falta MuJoCo

El plan original contemplaba MuJoCo para esta fase. No se usa, por el mismo argumento de la Fase 2 llevado un paso más allá: la destilación exige que maestro y alumno vean los mismos píxeles, y esos píxeles son los que produce el renderizador por homografía. Meter un motor de render habría cambiado la observación del alumno respecto de la del maestro, que es lo contrario de lo que este experimento necesita.

## Coste de entrenamiento, medido

Contra lo que el plan preveía, **no hizo falta alquilar GPU**. SmolVLA se afina en el MPS de un MacBook Pro M1 de 16 GB:

| Métrica | Valor |
|---|---|
| Parámetros entrenables | 99,9M (el backbone VLM se reduce a 16 capas) |
| Velocidad, benchmark de 20 pasos | 1,75 s/paso |
| Velocidad, ritmo sostenido real | ~2,95 s/paso |
| Batch | 4 |
| Coste en dinero | 0 |

Detalle honesto: el benchmark corto proyectaba 1,75 s/paso y el ritmo sostenido resultó ser un 63% mayor. Los benchmarks de veinte pasos son optimistas, y conviene decirlo cuando se usan para decidir si algo cabe en local.

Evolución de la pérdida: 0,266 en el paso 1, 0,067 en el paso 400, y a partir de ahí oscilando entre 0,06 y 0,09 sin tendencia clara. La pérdida se estanca pronto, lo que era esperable en una tarea tan estrecha. Que se estanque la pérdida de entrenamiento no implica que se haya estancado el rendimiento en lazo cerrado: la pérdida es una regresión sobre acciones individuales y lo que se mide al final es el error acumulado de cientos de decisiones encadenadas. Por eso se conserva un checkpoint intermedio del paso 1.500, para medirlo en vez de suponerlo.

## Protocolo de evaluación

Los dos brazos corren sobre las mismas poses iniciales, la misma cámara, el mismo ruido de sensor y de actuación, y **el mismo criterio de parada**: la maniobra termina cuando el vehículo deja de moverse, con los mismos umbrales. Darle regla de parada a uno y no al otro sesgaría la comparación.

También comparten el criterio de fallo: si el detector no encuentra el marcador, la maniobra ha fracasado. Se aplica igual a los dos aunque la política no use las esquinas, porque es la condición de que el marcador siga en cuadro.

Al final se encadena `attempt_grasp`, igual que en la Fase 2, para traducir el error de estacionamiento en lo único que le importa a la tarea: si la pieza se puede coger. El presupuesto es el de la Fase 2, 5,41 cm de error lateral.

## Nota de entorno

`SmolVLAPolicy.from_pretrained` no puede leer su propio `config.json` en Python 3.14: draccus falla al registrar los argumentos del tipo `Dict[str, PolicyFeature] | None` con un `is not callable`. Se evita construyendo el config desde las features del dataset y pasándolo explícitamente. La misma función se comparte entre entrenamiento y evaluación, para que los dos vean idéntica configuración.

## Resultados

### Lazo cerrado, 40 ensayos sobre las mismas poses iniciales

| Métrica | Experto clásico | Destilada (chunk 50) |
|---|---|---|
| Ensayos completados | 40 de 40 | 40 de 40 |
| Error de posición, media | **4,86 mm** | 74,51 mm |
| Error de posición, p95 | 8,26 mm | 160,14 mm |
| Error de posición, máximo | 8,95 mm | 184,43 mm |
| Error de guiñada, media | 0,30° | 3,42° |
| Se asentaron por criterio propio | 100% | 38% |
| **Éxito de agarre** | **100%** | **55%** |

La política queda un factor 15 por detrás de su propio maestro. El 55% de agarre se explica por el presupuesto de la tarea derivado en la Fase 2: como el límite lateral son 54 mm y la política aterriza en 74,5 mm de media, aproximadamente la mitad de los ensayos cae dentro por poco.

### El diagnóstico en lazo abierto, que es lo que da la interpretación

Antes de leer nada de la tabla anterior hay que responder si la política reproduce las acciones del experto sobre los fotogramas **con los que se entrenó**. Si no lo hace, el error en lazo cerrado no informa sobre acumulación de error ni sobre control, sino sobre que el modelo no aprendió la tarea.

| Checkpoint | Error relativo | Correlación avance | Correlación lateral | Correlación giro |
|---|---|---|---|---|
| paso 1.500 | 122,8% | 0,876 | 0,384 | -0,063 |
| paso 2.500 | 90,6% | 0,940 | 0,486 | 0,255 |
| paso 3.000 | 98,1% | 0,821 | 0,527 | 0,168 |

Referencias: predecir siempre cero da 100,0% de error relativo; predecir la media del dataset, 103,5%.

En norma L2 la política empata con "predecir cero", lo cual leído solo suena a fracaso total. Pero las correlaciones dicen otra cosa: **aprendió bien la componente de avance (r entre 0,82 y 0,94) y mal las correcciones laterales y de giro** (0,38 a 0,53 y en torno a 0,17).

Eso localiza el fallo. Lo grueso, acercarse al marcador, es fácil de aprender. Lo fino, las correcciones que dependen de leer las esquinas del marcador con exactitud subpíxel, no se aprendió. Y son justamente las que dan la precisión.

No hay mejora monótona entre checkpoints. Con tres puntos y 64 muestras es evidencia débil, pero no apunta a que más pasos, en este régimen, lo resuelvan solos.

### La latencia real, y una medición que hubo que rectificar

La primera medida de latencia, 19,8 ms y 50 Hz efectivos, **estaba mal** y conviene explicar por qué porque el error es fácil de cometer.

SmolVLA usa action chunking: por defecto `chunk_size=50` y `n_action_steps=50`. Cada inferencia produce 50 acciones que se consumen de una cola. Cronometrar `select_action` en cada paso promedia una inferencia real con 49 lecturas de cola, y da un número unas cincuenta veces optimista.

Midiendo con `n_action_steps=1`, donde cada paso es una inferencia de verdad:

| Configuración | Latencia por decisión | Frecuencia real |
|---|---|---|
| Aparente, promediando sobre la cola | 19,8 ms | 50,5 Hz |
| **Real, una inferencia por paso** | **625 ms** | **1,6 Hz** |

1,6 Hz en el MPS de un M1. Queda muy por debajo de los 10 Hz que la literatura señala como frontera para despliegue industrial. El chunking entrega un lazo a 20 Hz, pero cada decisión se toma con información de hasta 2,5 segundos antes.

### Replanificar en cada paso lo empeora

La hipótesis natural era que esos 2,5 segundos de ejecución a ciegas explicaban el fallo. Se probó y **es falso**:

| Métrica | Chunk 50 (40 ensayos) | Chunk 1 (20 ensayos) |
|---|---|---|
| Marcador perdido | 0 | **10 de 20** |
| Error medio, de los completados | 74,5 mm | 119,3 mm |
| Se asentaron | 38% | 30% |

Replanificar cada paso no mejora: destruye la maniobra. La mitad de los ensayos pierde el marcador de vista, cosa que no pasaba ni una sola vez con el chunk por defecto.

La explicación encaja con el diagnóstico en lazo abierto. Si cada predicción individual es ruidosa, ejecutar un chunk de 50 actúa como suavizado y el vehículo avanza en una dirección coherente. Replanificar en cada paso mete el ruido de cada predicción directamente en el lazo, y el vehículo serpentea hasta salirse del campo de visión.

El 70% de agarre que aparece en esa columna no es una mejora: está calculado solo sobre los 10 ensayos que sobrevivieron, que son los fáciles. Es sesgo de selección y no debe compararse con el 55% de la otra columna.

## Qué se puede concluir y qué no

**Se puede concluir**, sobre este banco y con este presupuesto:

- Una política de 450M destilada de un controlador clásico, con 120 demostraciones y 3.000 pasos de entrenamiento en un portátil, queda un factor 15 por detrás de su maestro en precisión de estacionamiento, y convierte un 100% de éxito de agarre en un 55%.
- El fallo está localizado: aprende el avance y no las correcciones finas.
- La latencia real de inferencia en hardware de consumo es 1,6 Hz, y el action chunking, que es lo que hace usable esa latencia, es también lo que impide replanificar con frecuencia.

**No se puede concluir** que los VLA no sirvan para estacionamiento de precisión. El entrenamiento fueron 3.000 pasos frente a los 20.000 que recomienda la documentación de SmolVLA, con batch 4 impuesto por los 16 GB de memoria del equipo, sin búsqueda de hiperparámetros, sin planificador de tasa de aprendizaje y sin aumento de datos. Presentar este resultado como una propiedad del método sería atribuirle al modelo una limitación que podría ser del montaje.

## La hipótesis que queda abierta, y cómo se falsa

La sospecha, que este trabajo no resuelve, es que el límite no es de cómputo sino **de representación**.

El marcador ocupa 53 px en la observación de 256x192. El backbone visual tokeniza en parches de unos 16 px, así que el marcador entero abarca tres o cuatro parches. El controlador clásico obtiene su precisión del refinamiento subpíxel de las esquinas, diferencias de medio píxel. Una representación por parches de ese tamaño descarta esa información antes de que la red la vea. Si es así, más cómputo no lo arregla.

Encaja con la literatura: π0 afinado con presupuesto serio, en una GPU H20 de 96 GB, sigue reportando 2,2 cm de error de posición.

Dos experimentos la falsarían, y ninguno necesita hardware nuevo:

1. **Repetir a 640x480 para maestro y alumno.** El marcador pasa de 53 a 133 px, o sea de 3 parches a 8, y el maestro mejora a 1,43 mm. Si la hipótesis es correcta, el alumno debería mejorar proporcionalmente más que el maestro.
2. **Acumulación de gradiente para batch efectivo 32**, que cabe en 16 GB y prueba directamente si el batch pequeño era el problema.

Si tras eso el error sigue en la decena de centímetros, el límite representacional queda confirmado y el resultado pasa de inconcluso a publicable.

## Limitaciones

- La escena es visualmente pobre: un marcador, sin fondo, sin oclusiones, sin variación de iluminación. Mide precisión, no robustez visual.
- Una sola pose de estacionamiento. La política no está condicionada a una imagen objetivo, así que aprende *esa* pose y no la tarea general de estacionar.
- Todo es cinemático. No hay dinámica de la plataforma ni deslizamiento de ruedas mecanum.
- El experto está calibrado por su autor y la política se entrena sobre sus demostraciones, así que hereda sus sesgos por construcción. Eso es lo que hace justa la comparación de precisión y a la vez lo que impide sacar conclusiones sobre generalización.
