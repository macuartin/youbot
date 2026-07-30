# youBot: de una tesis abandonada a un experimento sobre el coste de investigar

Este repo tuvo su primer commit en marzo de 2020 y el último en octubre de 2020. Era el código de un trabajo de investigación de maestría en la Pontificia Universidad Javeriana, "Arquitectura de control visual de estacionamiento de precisión para un robot móvil en labores de pick & place", que nunca se terminó. La razón no fue técnica: trabajar y estudiar a la vez no era viable y había una hija en camino.

En 2026 se retoma con dos objetivos.

**Técnico.** Cerrar los cinco objetivos específicos de 2018 en simulación, y usar ese pipeline clásico como brazo de control para una pregunta nueva: ¿reemplazan los modelos Vision-Language-Action la cadena de servo visual más cinemática inversa más planificador de trayectorias en manipulación móvil, y a qué coste?

**Medición.** Registrar el coste real de hacerlo con IA en el bucle, para poder comparar contra el coste de 2018 con números y no con impresiones. El experimento natural es difícil de mejorar: el mismo problema, la misma persona, ocho años de diferencia.

## Estado

| Objetivo de 2018 | Estado |
|---|---|
| OE1 Control visual del AGV con marcador | pendiente |
| OE2 Modelo cinemático y dinámico Newton-Euler | **validado** contra oráculo independiente a 1e-9 |
| OE3 Control articular por cinemática inversa | **validado**, seguimiento por debajo de 1 mm |
| OE4 Validación pick & place (95% de agarre) | pendiente |
| OE5 Validación de estacionamiento (±10 cm) | pendiente |

```bash
uv sync --group dev
uv run pytest
```

El oráculo es [`roboticstoolbox-python`](https://github.com/petercorke/robotics-toolbox-python): se construye el mismo robot desde la misma tabla DH y se comparan cinemática directa, los dos jacobianos, torques de gravedad, matriz de masa y Newton-Euler completo sobre 25 configuraciones aleatorias. Los tests que no dependen de él siguen corriendo si no está instalado.

## Estructura

```
youbot/     núcleo Python 3 sin ROS: cinemática, dinámica, trayectorias, servo visual
docs/       baseline histórico, survey del estado del arte
paper/      preprint
legacy/     los paquetes ROS 1 Kinetic / Python 2.7 de 2020, tal cual quedaron
EFFORT.md   bitácora de esfuerzo
```

`legacy/` no se toca. Es el objeto de estudio: ahí viven los tres bugs que mataron el trabajo original, documentados en [docs/baseline-2018.md](docs/baseline-2018.md).

## Contacto

[macuartin@gmail.com](mailto:macuartin@gmail.com)
