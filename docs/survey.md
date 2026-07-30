# Fase 3: estado del arte de la visión por computadora para manipulación, 2018 a 2026

Survey acotado a la pregunta que hereda este trabajo: **¿reemplazan los modelos Vision-Language-Action la cadena clásica de servo visual más cinemática inversa más planificador de trayectorias en manipulación móvil, y a qué coste?**

## Nivel de evidencia

Distinción explícita, porque no todas las fuentes de aquí se leyeron igual:

- **Leído a fondo**: `arXiv:2509.23121` (transferencia de VLA a industria) y `arXiv:2509.13024` (DVDP, docking aprendido). De estos salen los números que sostienen el análisis.
- **Datos de tarjeta de modelo y documentación oficial**: OpenVLA, SmolVLA, GR00T N1, π0.
- **Localizado pero no leído a fondo**: `arXiv:2603.13966` (vla-eval), `arXiv:2603.15046` (AnoleVLA), `arXiv:2606.00253` (fine-tuning para manipulación móvil de 11 GDL). Se citan como señal de hacia dónde va el campo, no como respaldo de ninguna cifra.

Ninguna cifra de este documento es de memoria del modelo que lo escribe: el conocimiento base llega a mayo de 2026 y todo lo posterior, y también lo anterior, se verificó contra fuente.

## Qué cambió entre 2018 y 2026

El anteproyecto de 2018 daba por sentada una arquitectura: extraer características de un marcador, calcular la matriz de interacción, cerrar un lazo proporcional, y aparte resolver cinemática inversa y planificar trayectorias. Cada bloque diseñado a mano y con su justificación matemática.

Lo que apareció en el medio es la posibilidad de sustituir la cadena entera por una sola red que va de píxeles a acciones. La secuencia relevante:

| Modelo | Tamaño | Datos de entrenamiento | Referencia |
|---|---|---|---|
| OpenVLA | 7B | 970.000 episodios de Open X-Embodiment | [arXiv:2406.09246](https://arxiv.org/abs/2406.09246) |
| π0 | Gemma 2.6B + cabeza de acción 300M, flow matching | Múltiples plataformas, incluidos manipuladores móviles | [physicalintelligence.company](https://www.physicalintelligence.company/blog/pi0) |
| SmolVLA | 450M | 481 datasets de la comunidad LeRobot, 10,6M fotogramas | [huggingface.co/blog/smolvla](https://huggingface.co/blog/smolvla) |
| GR00T N1 | Backbone de razonamiento más módulo motor | Más de 20.000 horas de vídeo egocéntrico humano más teleoperación | [NVIDIA](https://developer.nvidia.com/isaac/gr00t) |

Dato que importa para este trabajo: **SmolVLA, con 450M de parámetros, saca 87,3% en LIBERO frente al 76,5% de OpenVLA con 7B y el 86,0% de π0 con 3,3B**. La escala dejó de ser el eje dominante, y eso es lo que abre la puerta a evaluar algo en un portátil.

El campo también empezó a construir su propia infraestructura de evaluación (`vla-eval`, `arXiv:2603.13966`), que es la señal habitual de que una línea madura: cuando hay demasiados modelos como para compararlos a ojo.

## La cifra que decide la pregunta

Aquí está el resultado del survey, y es bastante nítido.

[arXiv:2509.23121](https://arxiv.org/html/2509.23121v1), sobre transferencia de VLA a aplicaciones industriales, reporta que **π0 afinado sobre datos de la tarea alcanza 2,2 cm de error de posición y 12,4 grados de error de orientación** en tareas de colocación de precisión. El mismo trabajo cita, para contraste, que el servo visual clásico alinea la punta de un destornillador sobre tornillos M4 a M8 **con error medio de 0,8 a 1,3 mm**.

Un orden de magnitud largo de diferencia, en la misma clase de tarea.

Y no es solo precisión. El mismo trabajo señala que la latencia de inferencia de los VLA grandes suele quedar **por debajo de 10 Hz**, que el fine-tuning de su caso necesitó **una GPU H20 de 96 GB durante 10 horas**, y que las tres limitaciones para adopción industrial son escasez de datos industriales, dificultad con oclusiones y disposiciones espaciales complejas, y coste computacional.

## Cruce con el resultado propio de la Fase 2

Esto es lo que convierte el survey en un aporte y no en una lista de lecturas.

La Fase 2 de este trabajo midió, sobre el youBot, la ley que relaciona el error de estacionamiento con la viabilidad del agarre:

```
desalineación de la pinza = arctan(error_lateral / alcance_horizontal)
```

y de ahí el presupuesto de error de la tarea: **5,4 cm de error lateral máximo** y **10 grados de desalineación máxima** de la pinza. Con esos dos números se puede evaluar cualquier método publicado sin ejecutarlo:

| Método | Error de posición | Error de orientación | ¿Pasa el presupuesto de la tarea? |
|---|---|---|---|
| IBVS clásico con marcador (Fase 2 de este trabajo) | 1,27 mm media, 5,37 mm máx | 0,13 grados | Sí, con 10 veces de margen |
| Servo visual clásico de precisión (citado en 2509.23121) | 0,8 a 1,3 mm | no reportado | Sí |
| DVDP, docking aprendido sin marcador ([arXiv:2509.13024](https://arxiv.org/html/2509.13024)) | 44,5 mm | 4,5 grados | Al límite: 44,5 mm dan 8,2 grados de desalineación, contra 10 de tolerancia |
| π0 afinado, colocación de precisión (2509.23121) | 22 mm | 12,4 grados | **No**: los 12,4 grados de orientación ya exceden por sí solos la tolerancia de 10 |

La respuesta a la pregunta de investigación, con los datos publicados a julio de 2026, es **no**: un VLA generalista no reemplaza la cadena clásica en una tarea de estacionamiento de precisión seguida de pick & place. No por poco, y no por un detalle de implementación. El error de orientación de un π0 afinado excede por sí solo el presupuesto angular que la geometría de la pinza impone.

## Pero el eje de comparación no es la precisión

Sería deshonesto quedarse ahí, porque la precisión no es la razón por la que existen los VLA.

Los tres métodos de la tabla no compiten en el mismo eje. Ordenados por lo que hace falta preparar antes de que funcionen:

- **IBVS con marcador**: necesita un marcador colocado en cada estación, una imagen de referencia por estación, y que la navegación deje al robot dentro de la cuenca de atracción, que en la Fase 2 resultó ser el 64% de una rejilla de ±30 cm y ±30 grados. A cambio, milímetros.
- **DVDP**: no necesita marcador, solo una cámara RGB-D, y sus autores reivindican docking desde poses iniciales arbitrarias. A cambio, 4,45 cm y 73,2% de tasa de éxito.
- **VLA generalista**: no necesita marcador ni imagen de referencia ni una tarea definida de antemano; se le dice en lenguaje natural. A cambio, centímetros y decenas de grados.

Lo que se compra con cada orden de magnitud de precisión perdida es **no tener que preparar el entorno**. Y el trabajo de 2018 vivía en el extremo opuesto de esa curva: un sistema de manufactura flexible es precisamente un entorno que sí se puede preparar, donde poner un marcador en cada estación cuesta lo que cuesta imprimirlo.

Esa es la conclusión no obvia del survey. La pregunta "¿reemplaza el VLA al servo visual clásico?" está mal planteada. En un FMS, con estaciones fijas y tolerancias de milímetros, no hay nada que reemplazar: el marcador es barato y la precisión es obligatoria. Donde el VLA gana es exactamente donde el trabajo de 2018 no estaba: entornos que no se pueden instrumentar.

## Viabilidad de la Fase 4 en el hardware disponible

Restricción dura: MacBook Pro M1, 16 GB de memoria unificada.

**Lo que sí es ejecutable en local.** SmolVLA 450M corre en CPU y su documentación menciona explícitamente que se puede entrenar en un MacBook. Es el único de los cuatro que entra sin discusión.

**Lo que no.** π0 con 3,3B y OpenVLA con 7B no caben cómodamente en 16 GB compartidos con un simulador. Y el fine-tuning del caso industrial de referencia consumió una H20 de 96 GB durante 10 horas, que a precio de alquiler son del orden de 20 a 40 USD, asumible, pero no es el cuello de botella.

**El cuello de botella real no es la GPU, son los datos.** SmolVLA es un modelo base: su propia documentación recomienda grabar del orden de **50 episodios** de la tarea concreta para afinarlo. Para la tarea de este trabajo eso significa construir la escena en un simulador con render, teleoperar cincuenta demostraciones de estacionamiento y agarre sobre el youBot, y afinar. Es un proyecto en sí mismo, de un orden de magnitud más grande que las fases 1 y 2 juntas.

**Y evaluarlo en zero-shot no diría nada.** Ninguno de estos modelos ha visto un youBot ni esta estación. Un fallo en zero-shot no informa sobre la capacidad del método, solo sobre la ausencia de esa plataforma en su conjunto de entrenamiento. Sería un experimento sin contenido.

## Recomendación para la Fase 4

El plan contemplaba este punto de corte y hay datos para tomarlo.

Lo que se propone conservar de la Fase 4:

1. **El análisis de presupuesto de error**, que ya está hecho en la tabla de arriba. Es la contribución fuerte: convierte la ley medida en la Fase 2 en un criterio con el que evaluar cualquier método publicado, y da una respuesta cuantitativa a la pregunta de investigación sin necesidad de reentrenar nada.
2. **Caracterización local de SmolVLA medida, no supuesta**: latencia de inferencia por decisión y huella de memoria reales en el M1. Es honesto, es hands-on, y ataca el eje de despliegue, que es donde la literatura señala el problema (por debajo de 10 Hz) y donde nadie publica números de hardware de consumo.

Lo que se propone declarar fuera de alcance, con la razón:

3. **Afinar un VLA para esta tarea.** Requiere unas 50 demostraciones teleoperadas en un simulador con render que aún no existe en este repo. No es una limitación de presupuesto, es de alcance, y esconderlo detrás de una evaluación zero-shot sin sentido sería peor que declararlo.

## Fuentes

- [OpenVLA: An Open-Source Vision-Language-Action Model](https://arxiv.org/abs/2406.09246): modelo de 7B sobre 970.000 episodios de Open X-Embodiment.
- [Transferring Vision-Language-Action Models to Industry Applications](https://arxiv.org/html/2509.23121v1): π0 afinado con 2,2 cm y 12,4 grados; servo visual clásico con 0,8 a 1,3 mm; latencia por debajo de 10 Hz; H20 de 96 GB durante 10 horas.
- [DVDP: An End-to-End Policy for Mobile Robot Visual Docking with RGB-D Perception](https://arxiv.org/html/2509.13024): 44,5 mm, 4,5 grados, 73,2% de éxito, sin marcador, entrenado en una RTX 4090.
- [SmolVLA](https://huggingface.co/blog/smolvla): 450M, 481 datasets de LeRobot, 10,6M fotogramas, 87,3% en LIBERO, corre en CPU.
- [EA-CTFVS: An Environment-Agnostic Coarse-to-Fine Visual Servoing Method for Sub-Millimeter-Accurate Assembly](https://www.mdpi.com/2076-0825/13/8/294): el extremo de precisión de la familia clásica.
- [Robust Docking Maneuvers for Autonomous Trolley Collection](https://arxiv.org/html/2509.07413v2): servo visual de docking basado en optimización.
- `arXiv:2603.13966` (vla-eval), `arXiv:2603.15046` (AnoleVLA), `arXiv:2606.00253` (fine-tuning para manipulación móvil de 11 GDL): localizados, no leídos a fondo.

## Cobertura y sesgos

Las cifras industriales provienen de un solo grupo (proyecto de Liaoning Liaohe Lab) y de una sola tarea de colocación, así que no deben leerse como el estado del arte de la precisión alcanzable por un VLA sino como un punto de medida documentado. La familia clásica está representada por trabajos de docking y ensamblaje que eligen sus propias condiciones experimentales. Ninguna de las dos columnas de la tabla comparativa se midió en el mismo banco, y eso es una limitación real de este survey: son órdenes de magnitud comparables, no medidas equivalentes.
