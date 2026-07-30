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
| 1 | 2026-07-30 | 0 | | | 0 | Revisión del material de 2018-2020, diagnóstico de los 3 bugs bloqueantes, plan de 7 fases, rama `rescate-2026`, `docs/baseline-2018.md`, esta bitácora. |

## Acumulado

| Métrica | Valor |
|---|---|
| Sesiones | 1 |
| Wall-clock total | pendiente |
| Coste total (USD) | pendiente |
| Objetivos de 2018 validados | 0 de 5 |
