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

Pendiente: se rellena al terminar el entrenamiento.

## Limitaciones

- La escena es visualmente pobre: un marcador, sin fondo, sin oclusiones, sin variación de iluminación. Mide precisión, no robustez visual.
- Una sola pose de estacionamiento. La política no está condicionada a una imagen objetivo, así que aprende *esa* pose y no la tarea general de estacionar.
- Todo es cinemático. No hay dinámica de la plataforma ni deslizamiento de ruedas mecanum.
- El experto está calibrado por su autor y la política se entrena sobre sus demostraciones, así que hereda sus sesgos por construcción. Eso es lo que hace justa la comparación de precisión y a la vez lo que impide sacar conclusiones sobre generalización.
