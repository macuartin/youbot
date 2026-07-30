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
| 1 | 2026-07-30 | 0 | | | 0 | Revisión del material de 2018-2020, diagnóstico de 5 bugs bloqueantes, plan de 7 fases, rama `rescate-2026`, `docs/baseline-2018.md`, esta bitácora. |
| 2 | 2026-07-30 | 1 | | | 0 | Núcleo Python 3 sin ROS (`youbot/`): modelo, cinemática, dinámica Newton-Euler, trayectorias, control articular. Discrepancias DH resueltas contra implementación de referencia. 13 tests verdes, con oráculo independiente a 1e-9. **OE2 y OE3 cerrados.** |

## Acumulado

| Métrica | Valor |
|---|---|
| Sesiones | 2 |
| Wall-clock total | pendiente |
| Coste total (USD) | pendiente |
| Objetivos de 2018 validados | 2 de 5 |

## Notas de método

Los bugs propios también cuentan. En la sesión 2 la primera versión de la
dinámica mezcló la recursión de Newton-Euler para DH modificada con cinemática
en DH clásica. Lo detectó un caso analítico de dos eslabones en el primer
intento, antes de tocar el modelo real. Duración del error: minutos. Ese
contraste, y no la ausencia de errores, es lo que hay que medir.
