# Bitácora de esfuerzo

Registro append-only del coste real de reconstruir en 2026 un trabajo de maestría abandonado en 2020. Sirve de evidencia para la serie de posts sobre el colapso del coste de investigar.

## Reglas

1. **Una fila por sesión.** Se escribe al cerrar la sesión, no después.
2. **Nada retroactivo.** Si no se midió en el momento, no entra. Una celda vacía es un dato honesto; una estimación inventada no.
3. **Los fallos también se registran.** Sesiones que no produjeron nada, callejones sin salida y trabajo tirado a la basura llevan su fila igual. Una bitácora que solo cuenta los aciertos no es medición.
4. **Coste de tokens**: salida de `/cost` de Claude Code al final de la sesión.
5. **Coste externo**: GPU alquilada, APIs, servicios. En USD.

## Baseline contra el que se compara

Ver [docs/baseline-2018.md](docs/baseline-2018.md). Resumen: 34 meses, 0 de 5 objetivos validados, 972 líneas de código, presupuesto declarado de $102.600.000 COP.

## Sesiones

| # | Fecha | Fase | Wall-clock | Tokens (USD) | Externo (USD) | Entregable |
|---|---|---|---|---|---|---|
| 3 | 2026-07-30 | 3, más rework de la 2 | 15m 41s | 6,38 | 0,11 | Survey del estado del arte VLA con fuentes primarias (`docs/survey.md`). Cruce del presupuesto de error medido en la Fase 2 con las cifras publicadas: la pregunta de investigación queda respondida sin reentrenar nada. Corrección del criterio de parada de la Fase 2 (la tolerancia estaba por debajo del suelo de ruido). Detector real dentro del lazo, que valida el modelo de ruido de forma independiente. |
| 2 | 2026-07-30 | 2 | 29m 55s | 9,55 | 0 | Servo visual de estacionamiento (`vision.py`, `parking.py`) validado con diferencia finita, ruido del detector ArUco medido (0,5 px), agarre con restricción de aproximación, campaña de 500 ensayos y barridos de sensibilidad. 24 tests verdes. **OE1, OE4 y OE5 cerrados: los cinco objetivos de 2018 completos.** Hallazgo: el criterio de ±10 cm del anteproyecto es 1,8 veces más laxo que lo que la tarea admite. |
| 1 | 2026-07-30 | 0 y 1 | 59m 29s | ≤ 12,95 | 0,05 | Revisión del material de 2018-2020, diagnóstico de 5 bugs bloqueantes, plan de 7 fases, `docs/baseline-2018.md`, esta bitácora. Núcleo Python 3 sin ROS (`youbot/`): modelo, cinemática, dinámica Newton-Euler, trayectorias, control articular. Discrepancias DH resueltas contra implementación de referencia. 13 tests verdes con oráculo independiente a 1e-9. **OE2 y OE3 cerrados.** |

Detalle de la sesión 1, salida de `/cost`: 26m 57s de tiempo de API sobre 59m 29s de reloj de pared, 1406 líneas añadidas y 67 borradas en total, 2 búsquedas web (0,0523 USD). El 83% del consumo ocurrió con más de 150k de contexto, que es la parte cara.

**Cómo se obtienen las cifras.** Los contadores de `/cost` son acumulados de la sesión de Claude Code, no por tarea. Las filas de esta tabla salen de diferenciar dos lecturas del contador, y eso obliga a una precisión: la sesión 1 y la 2 son en realidad la misma sesión continua de Claude Code, partida aquí por fase.

**Por qué la fila 1 lleva `≤` y la 2 no.** La sesión arrancó con trabajo que no es de esta tesis: crear el repo privado del vault de Obsidian y montarle un auto-commit diario con launchd. Eso ocupó las primeras llamadas, con contexto pequeño, así que su peso es bajo pero no es cero y no se puede aislar a posteriori. La fila 1 lleva por tanto el total de la lectura, 12,95 USD, con `≤` para dejar claro que la tesis costó eso o menos.

La fila 2 sí es exacta: el trabajo ajeno quedó entero antes de la primera lectura, así que la diferencia entre lecturas (22,50 menos 12,95, y 1h 29m 24s menos 59m 29s) es atribuible por completo a la Fase 2. La medición salió mejor de lo que se prometió, no peor.

Las 2 búsquedas web son el único gasto externo hasta ahora, y no son un detalle: resolvieron las dos discrepancias de la tabla DH (`alpha4` y `d5`) que en 2018 se quedaron sin zanjar. Cinco céntimos.

## Acumulado

| Métrica | Valor |
|---|---|
| Tramos medidos | 3 |
| Wall-clock total | 1h 45m 05s |
| Coste total (USD) | ≤ 28,88 |
| Objetivos de 2018 validados | **5 de 5** |
| Líneas de código propio validado | 1.914 |

## Comparación con el baseline

| | 2017-2020 | 2026 |
|---|---|---|
| Elapsed | 34 meses | 1h 45m |
| Objetivos validados | 0 de 5 | 5 de 5 |
| Líneas de código | 972, ninguna validada | 1.914, con oráculos independientes |
| Coste directo | $102.600.000 COP declarados en el presupuesto | ≤ 28,88 USD |

**Esta tabla no es una comparación limpia y no debe presentarse como tal.** Las diferencias que no son la IA:

- En 2018 el autor estaba aprendiendo robótica de manipuladores por primera vez. En 2026 llega con ocho años más de oficio, y además con el marco teórico ya escrito por él mismo.
- El trabajo de 2018 incluía asignaturas, seminario de investigación y la validación en hardware real en el laboratorio del CTAI. Aquí no hay hardware: la validación es contra una librería de referencia, que es un criterio más débil que un robot físico.
- Los 34 meses son tiempo de calendario de alguien trabajando a jornada completa, no 34 meses de dedicación.
- El presupuesto de 2018 era una declaración institucional que incluía salarios de director y codirector, no dinero que el estudiante gastara de su bolsillo.

Lo que la tabla sí sostiene, y es suficiente: el trabajo técnico que quedó sin cerrar durante 34 meses se cerró en hora y tres cuartos, y los cinco bugs que lo bloqueaban se identificaron leyendo el código, no ejecutándolo.

Dato de método que conviene retener para el post: el 87% del consumo ocurrió por encima de 150k de contexto. Lo caro no es el número de llamadas, es la longitud de la sesión. Investigar así tiene una estructura de costes distinta a programar por tareas cortas.

## Notas de método

Los bugs propios también cuentan. En la sesión 1 la primera versión de la
dinámica mezcló la recursión de Newton-Euler para DH modificada con cinemática
en DH clásica. Lo detectó un caso analítico de dos eslabones en el primer
intento, antes de tocar el modelo real. Duración del error: minutos. Ese
contraste, y no la ausencia de errores, es lo que hay que medir.

En la sesión 2 hubo tres más, todos cazados por un test que falló: la pose del
marcador con la z hacia la cámara en vez de hacia dentro del plano (el detector
no lo encontraba), el borde blanco de la plantilla metiendo un desplazamiento
constante de 42 px, y un solver de agarre hecho a mano que se atascaba en los
límites articulares hasta que se cambió por `scipy.optimize.least_squares` con
cotas.

Y un cuarto de otra clase, que no es un bug sino un error de modelado: el primer
criterio de agarre solo miraba la posición, y con ese criterio la premisa entera
del trabajo de 2018 no se sostenía. El test que lo comprobaba falló, y en vez de
relajarlo hubo que reconocer que alcanzar un punto no es agarrar. De ahí salió el
resultado principal de la fase. Los tests que fallan cuando la hipótesis es
demasiado cómoda son los que más valen.
