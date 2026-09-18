# Laboratorio 4

---

## Entorno de ejecución

Todas las corridas de esta sección (Ejercicios A, B y C) se realizaron en el mismo equipo, para garantizar consistencia entre las mediciones de las distintas herramientas.

| Componente | Detalle |
| --- | --- |
| Modelo | Lenovo IdeaPad 3 14ITL6 |
| CPU | Intel Core i5-1135G7 (11ª gen), 4 núcleos / 8 hilos, hasta 4.2 GHz |
| Caché | L1d: 192 KiB · L1i: 128 KiB · L2: 5 MiB · L3: 8 MiB |
| RAM | 7.6 GiB total (5.1 GiB en uso al momento de la captura) |
| Swap | 4.0 GiB |
| SO | Ubuntu 24.04.2 LTS (noble) |
| Kernel | Linux 7.0.0-30-generic |
| Disco | 60 GB (29 GB disponibles) en NVMe |
| Compilador | g++ 13.3.0 |
| perf | 7.0.12 |

---

## 1. Comandos utilizados

Compilación estándar

```
make clean
make
```

Compilación conservando símbolos y marcos de pila (para perf/objdump)

```
make clean
make CXXFLAGS="-std=c++17 -O2 -g -Wall -Wextra -pedantic -fno-omit-frame-pointer"
```

Ejecución base

```
./point_cloud_collimation
./point_cloud_collimation --export
```

perf stat (Ejercicio B)

```
perf stat -e task-clock,instructions,cycles,branches,branch-misses,cache-references,cache-misses -o perf_stat_sin_export.txt ./point_cloud_collimation
perf stat -e task-clock,instructions,cycles,branches,branch-misses,cache-references,cache-misses -o perf_stat_con_export.txt ./point_cloud_collimation --export
```

perf record (una sola vez, reutilizado para B y C)

```
perf record -g -o perf_sin_export.data ./point_cloud_collimation
perf record -g -o perf_con_export.data ./point_cloud_collimation --export
```

perf report (Ejercicio B, a partir de las mismas capturas)

```
perf report -i perf_sin_export.data --stdio > perf_report_sin_export.txt
perf report -i perf_con_export.data --stdio > perf_report_con_export.txt
```

Valgrind Callgrind (Ejercicio B)

```
valgrind --tool=callgrind ./point_cloud_collimation
callgrind_annotate callgrind.out.* > callgrind_report_sin_export.txt
valgrind --tool=callgrind ./point_cloud_collimation --export
callgrind_annotate callgrind.out.* > callgrind_report_con_export.txt
```

Google Performance Tools (Ejercicio B)

Nota: se recompiló sin PIE porque google-pprof no resolvía símbolos con el binario PIE por defecto

```
g++ -std=c++17 -O2 -g -fno-omit-frame-pointer -no-pie -fno-pie -Wl,--no-as-needed -lprofiler -Wl,--as-needed $(pkg-config --libs gstreamer-1.0 gstreamer-app-1.0) point_cloud_collimation.cpp -o point_cloud_collimation
CPUPROFILE=point_cloud.prof ./point_cloud_collimation
google-pprof --text ./point_cloud_collimation point_cloud.prof > pprof_sin_export.txt
CPUPROFILE=point_cloud.prof ./point_cloud_collimation --export
google-pprof --text ./point_cloud_collimation point_cloud.prof > pprof_con_export.txt
```

Ensamblador / objdump (Ejercicio C)

```
g++ -std=c++17 -O2 -g -S -masm=intel $(pkg-config --cflags gstreamer-1.0 gstreamer-app-1.0) point_cloud_collimation.cpp -o point_cloud_collimation.s
objdump -drwCS -Mintel ./point_cloud_collimation > point_cloud_collimation.objdump
```

perf annotate (Ejercicio C, reutilizando perf_sin_export.data del mismo Ejercicio B)

```
perf annotate -i perf_sin_export.data --stdio > perf_annotate_full.txt
```

Bloque de GridIndex::nearest extraído de perf_annotate_full.txt con sed, ubicando su rango de líneas entre el inicio del símbolo y el siguiente símbolo

---

## 2. Ejercicio A: Comprensión del programa base

### Funciones principales identificadas

| Función | Rol |
| --- | --- |
| generate_h_rail_cloud | Genera el perfil objetivo (target) tipo riel en H |
| add_random_deformation | Aplica deformación no rígida (ondas sinusoidales + bumps gaussianos localizados) al perfil fuente |
| GridIndex (struct) | Estructura de indexación espacial (grid + unordered_map) para búsqueda de vecinos |
| GridIndex::nearest / nearest_neighbor_distances | Búsqueda de vecino más cercano |
| estimate_rigid_transform | Estima la rotación y traslación óptima (mínimos cuadrados) a partir de los pares emparejados |
| compare_profiles / profile_score | Comparación de perfiles mediante centroides y distancias |
| render_motion_frame | Renderizado de cuadros para el visor GStreamer |

### ¿Qué representan los centroides de ambos perfiles?

El centroide de cada nube es el punto promedio (x̄, ȳ) de todos sus puntos, y funciona como una medida resumen de la posición global de la nube en el canvas, independiente de su forma. En nuestros datos, el centroide del perfil objetivo (target) es constante en (4860.50, 5000.25) durante todo el proceso, mientras que el del perfil fuente (source) se desplaza en cada iteración del ICP: arranca en (5453.57, 4524.81) y converge hacia (4864.76, 4399.40).

### ¿Cómo cambia la distancia entre centroides durante las iteraciones?

El comportamiento no es monótono en todo el rango. Partiendo de centroid_distance = 760.12 en la iteración 0, el valor desciende de forma consistente hasta un mínimo de 541.59 en la iteración 21. A partir de la iteración 22 la distancia vuelve a subir hasta alcanzar un pico de 601.54 en la iteración 39, y de ahí en adelante desciende de forma gradual y monótona hasta 600.87 en la última iteración (45).

Este patrón se explica por el umbral de rechazo de vecinos (match_threshold = 420): en las primeras iteraciones solo se emparejan los puntos "fáciles" (cercanos por casualidad tras el ajuste inicial), lo que produce una convergencia rápida; conforme el algoritmo avanza, el conjunto de correspondencias válidas cambia, incorporando puntos de zonas más afectadas por la deformación no rígida, lo que desplaza la estimación hacia el pico de la iteración 39, tras el cual el conjunto de correspondencias se estabiliza y la métrica decrece suavemente hasta su valor final.

### Diferencia entre match_rmse, symmetric_chamfer_rmse y profile_score

match_rmse: se calcula dentro del bucle de ICP, únicamente sobre los pares que superan el umbral de 420 unidades, reutilizando las distancias al cuadrado ya obtenidas durante la búsqueda de vecinos.

symmetric_chamfer_rmse: se recalcula por separado en compare_profiles, considerando todos los puntos en ambas direcciones (source→target y target→source), sin descartar ninguno.

profile_score: combina el chamfer con la distancia entre centroides normalizada: (symmetric_chamfer_rmse + 0.25 · centroid_distance) / diagonal_del_canvas, y se usa como criterio de convergencia. Por eso su escala (~0.018–0.047) es mucho menor que la de las otras dos (~60–160 unidades del canvas).

### ¿Por qué la transformación recuperada no es idéntica a la esperada?

```
Expected source->target transform: theta=-18.00000 deg, tx=-1757.14529, ty=2390.34723
Recovered source->target transform: theta=-18.00595 deg, tx=-1720.41095, ty=1781.98006
```

El ángulo se recupera con un error mínimo (0.006°), pero la traslación en ty presenta un error de ~608 unidades, cercano al centroid_distance final (600.87). Esto es consistente con que la deformación no rígida aplicada (amplitude=60, rms=56.81 units) se distribuye de forma relativamente pareja alrededor de la nube, cancelando parcialmente su efecto sobre la rotación estimada. La traslación, en cambio, depende del desplazamiento promedio de los pares emparejados, y no cuenta con ese mecanismo de cancelación: cualquier componente neto de la deformación hacia un lado se traduce directamente en un sesgo sistemático de la traslación.

---

## 3. Ejercicio B: Perfilado con herramientas

### Hotspots identificados por cada herramienta

| Herramienta | Sin --export | Con --export |
| --- | --- | --- |
| perf report | GridIndex::nearest 97.25% (self) | GridIndex::nearest 88.62%; write_cloud_csv 7.53% |
| gperftools/pprof | GridIndex::nearest 94.4% (self) / 97.7% cumulativo | GridIndex::nearest 86.3% / 89.4% cumulativo; write_cloud_csv 7.7% cumulativo |
| Valgrind Callgrind | GridIndex::nearest ≈98.2% (repartido: 81.10% en el .cpp + 10.00% stl_vector.h + 3.09% hashtable.h + 1.99% hashtable_policy.h + 1.77% stl_algobase.h) | GridIndex::nearest ≈85%; aparecen __mpn_divrem 1.84%, lround 1.08%, draw_cloud 0.58% |

### perf stat — contadores generales

| Métrica | Sin --export | Con --export | Diferencia |
| --- | --- | --- | --- |
| task-clock | 41,560.97 ms | 45,215.73 ms | +8.8% |
| cycles | 125.18 B | 135.95 B | +8.6% |
| instructions | 208.44 B | 241.46 B | +15.8% |
| branches | 27.75 B | 34.16 B | +23.1% |
| branch-misses | 494.67 M | 509.68 M | +3.0% |
| cache-references | 7.16 B | 7.16 B | ≈0% |
| cache-misses | 61.70 M | 73.49 M | +19.1% |
| sys time | 0.095 s | 0.344 s | +262% |

### ¿Coinciden los resultados entre las tres herramientas?

Las tres identifican el mismo hotspot dominante, que es la conclusión central del análisis. La pequeña variación de porcentaje entre perf y gperftools (ambos basados en muestreo estadístico) se debe a diferencias en frecuencia de muestreo, mecanismo de captura y tratamiento de funciones inline. Valgrind Callgrind reporta cifras distintas porque no muestrea: instrumenta y cuenta cada instrucción de forma determinista, aunque esto no lo hace representativo del tiempo de ejecución real, ya que su instrumentación introduce un overhead considerable que altera el comportamiento temporal del programa.

Limitación metodológica importante: la corrida de gperftools se realizó con el binario compilado con -no-pie, necesario para que google-pprof resolviera correctamente los símbolos (limitación conocida de esta herramienta con binarios PIE modernos). Esto significa que sus tiempos absolutos no son estrictamente comparables con los de perf y Valgrind, ya que el cambio de PIE a no-PIE puede alterar el layout de memoria y el comportamiento de caché. Sin embargo, los porcentajes relativos de hotspot por función siguen siendo válidos y comparables.

### ¿Qué costo tiene exportar los archivos de reconstrucción?

El incremento de instructions en perf stat (+15.8%) coincide de forma casi exacta con el incremento observado en el conteo total de instrucciones (Ir) de Callgrind (207,960,832,635 → 240,252,814,507, +15.5%) — evidencia cruzada entre dos herramientas independientes. El salto más notorio, sin embargo, es el de sys time (+262%), reflejando las syscalls de escritura de los ~63 archivos generados (5 CSV + PPM por cuadro).

Las funciones nuevas que aparecen exclusivamente con --export: write_cloud_csv, draw_cloud/render_motion_frame, y un conjunto de funciones de formateo de punto flotante a texto (__mpn_divrem, hack_digit, __printf_fp_*) — el costo de convertir cada double a texto decimal para escribirlo en CSV.

### ¿Qué herramienta dio la evidencia más clara?

perf report ofrece un flujo rápido con una vista jerárquica útil, aunque con símbolos difíciles de interpretar en algunas ramas del árbol de llamadas. gperftools presentó una limitación técnica real (incompatibilidad con binarios PIE) que debió investigarse y resolverse, evidenciando menor compatibilidad con sistemas modernos. Valgrind Callgrind proporcionó el desglose más detallado, distinguiendo el costo por archivo fuente (incluyendo cuánto del tiempo de GridIndex::nearest corresponde a headers de la STL inlineados dentro de ella), a costa de un overhead de ejecución considerablemente mayor.

---

## 4. Ejercicio C: Perfilado mediante revisión de ensamblador

Nota metodológica: dos de las cinco funciones solicitadas (estimate_rigid_transform y add_random_deformation) no aparecen como símbolos independientes en el binario, ya que el compilador (-O2) las inlineó completamente dentro de main. Se ubicaron identificando fragmentos únicos de su código fuente (la llamada a atan2/sincos para la primera, las llamadas a sin para la segunda) dentro del desensamblado de main. El hecho de que una función sea inlineada no implica que sea poco relevante para el rendimiento: el inlining depende de tamaño, contexto de compilación y heurísticas del compilador. La evidencia de que GridIndex::nearest domina el costo proviene de los datos de perfilado del Ejercicio B, no del hecho de que conserve un símbolo separado.

### Instrucciones/llamadas costosas

| Instrucción/llamada | Función | Ocurrencias | Línea fuente asociada |
| --- | --- | --- | --- |
| sqrt@plt | compare_profiles (vía rmse_from_distances inlineada) | 3 | return std::sqrt(sum2 / static_cast<double>(distances.size())) |
| sin@plt | add_random_deformation (inlineada en main) | 2 | cálculo de wave_x y wave_y |
| sincos@plt | estimate_rigid_transform (inlineada en main) | 1 (combinada, no sin+cos separados) | tras theta = std::atan2(cross, dot) |
| div (entero) | GridIndex::nearest | 2 en el extracto (cb21, cb66) | { return __num % __den; } — cálculo de bucket del unordered_map |

### Patrón de acceso: contiguo vs. indirecto

GridIndex::nearest combina ambos patrones: el cálculo del índice de bucket mediante div es aritmético, pero el acceso a los nodos del unordered_map (mov r8, QWORD PTR [r9+0x18]) es indirecto vía punteros (recorrido de lista enlazada). En contraste, add_random_deformation accede a los puntos del std::vector de forma contigua (movsd xmm5, QWORD PTR [r15+0x8]).

### Saltos condicionales en bucles internos

Sí están presentes en GridIndex::nearest: instrucciones como jne cc20/jne cdd0 (en el desensamblado general) y, de forma más precisa según perf annotate, jne 0xca48 y jbe 0xca8e dentro del bucle principal de comparación de distancias — saltan hacia direcciones anteriores dentro del mismo rango de la función, correspondiendo al recorrido de puntos candidatos dentro de una celda y a la decisión de actualizar el mejor candidato encontrado.

### Regiones compute-bound vs. memory-bound

add_random_deformation y estimate_rigid_transform presentan cadenas de instrucciones aritméticas (mulsd, divsd, addsd, subsd) sobre registros xmm con pocos accesos a memoria intermedios — comportamiento compute-bound. GridIndex::nearest es más mixta: el cálculo de hash (div, acceso disperso por punteros) tiene un componente memory-bound por baja localidad, pero, como muestra el análisis de perf annotate a continuación, el costo dominante real está en el bucle de comparación de distancias, que combina acceso semi-indirecto a memoria con aritmética de punto flotante en registros.

### Análisis de perf annotate sobre GridIndex::nearest

La instrucción individual con mayor porcentaje de muestras dentro de GridIndex::nearest es:

```
15.69% : ca5b: subsd (%rax),%xmm0
```

Seguida de cerca por:

```
10.13% : ca75: comisd %xmm0,%xmm1
9.93% : ca64: mulsd %xmm0,%xmm0
8.88% : ca79: jbe 0xca8e
8.82% : ca95: jne 0xca48
8.76% : ca54: shlq $0x4,%rax
```

Estas seis son las instrucciones individuales más costosas, y forman parte de un único bucle interno (offsets ca48–ca95, 20 instrucciones en total) que, sumando todas sus instrucciones, concentra aproximadamente el 75% de las muestras totales de la función (74.78% exacto, sumando las 20 líneas del bloque). Este bucle recorre los puntos candidatos dentro de una celda del grid ya localizada, calculando para cada uno:

1. movslq (%rdx),%rax + shlq $0x4,%rax + addq %r9,%rax: obtiene la dirección del punto candidato a partir de un índice.
2. subsd (%rax),%xmm0 / subsd 0x8(%rax),%xmm1: calcula dx, dy (instrucción #1 en % de muestras).
3. mulsd/mulsd/addsd: eleva al cuadrado y suma (dx² + dy²).
4. comisd/jbe: compara contra la mejor distancia encontrada y decide actualizar.
5. jne 0xca48: cierra el bucle sobre el siguiente punto de la celda.

El costo dominante de GridIndex::nearest proviene de este bucle de comparación de distancias (~75% de las muestras), no del cálculo de hash para ubicar el bucket (div, solo 1.59% y 2.48% en sus dos apariciones). Este resultado coincide con el hotspot identificado en el Ejercicio B por las tres herramientas (perf, gperftools y Valgrind), y lo precisa a nivel de instrucción individual.

## Ejercicio D — perfilado mediante instrumentación manual

Instrumentación agregada con `std::chrono` en al menos 5 regiones del programa
(generación del perfil, deformación/ruido, construcción de `GridIndex`, búsqueda
de vecinos, estimación de transformación, métricas de comparación, exportación,
renderizado):

```cpp
auto t0 = std::chrono::steady_clock::now();
/* región medida */
auto t1 = std::chrono::steady_clock::now();
double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
std::cout << "region_x_ms=" << ms << "\n";
```

Ejecución con salida en CSV (`region,iteration,milliseconds`), repitiendo varias
corridas para promediar y reducir ruido de medición.

## Ejercicio E — propuesta de optimización

Cambio 1 — `cell_size` (90 → 120):

```bash
perf stat ./point_cloud_collimation
perf stat ./point_cloud_collimation --export
 
valgrind --tool=callgrind ./point_cloud_collimation
callgrind_annotate callgrind.out.* | less
```

Cambio 2 — criterio de convergencia (`VARIATION` 0.1% → 0.5%):

```bash
perf stat ./point_cloud_collimation
perf stat ./point_cloud_collimation --export
 
valgrind --tool=callgrind ./point_cloud_collimation
callgrind_annotate callgrind.out.* | less
```



## Ejercicio E — Propuesta de Optimización (`point_cloud_collimation`)

### Computadora usada para esta parte

| Componente | Detalle |
|---|---|
| Sistema operativo | Fedora Linux |
| CPU | 13th Gen Intel(R) Core(TM) i7-13700K |
| Arquitectura | Híbrida: 8 P-cores con Hyper-Threading (16 hilos lógicos) + 8 E-cores (sin HT) |
| CPUs lógicos totales | 24 |
| Frecuencia máxima P-cores | 5.3 GHz |
| Frecuencia máxima E-cores | 4.2 GHz |
| Memoria RAM | 31 GiB |

--

Se descartan los valores de `cpu_atom` en todas las tablas: su cobertura fue
menor al 0.04% en todas las corridas (ver paréntesis en la salida cruda de `perf`), es
decir, son ruido de medición y no una carga de trabajo real. Solo se usa
`cpu_core`, con cobertura de 99.9-100% en todas las corridas.
 
---
 
## Cambio número 1. Modificación del Cell_size de 90 a 120
 
La hipótesis es que para la búsqueda de un punto vecino, muchos puntos se agrupan en celdas; estas celdas contienen una cantidad de puntos dependiendo del tamaño, es decir, si se hacen celdas pequeñas, significa que van a existir más celdas por cómo se dividen los propios puntos, por ende, son muchas más celdas las cuales tiene que buscar en la búsqueda, a pesar incluso de poder alcanzar los 16 de radio que tiene como límite el código.
 
Haciendo la celda mucho más grande, deberían ser celdas con muchos más puntos, reduciendo así el radio máximo de la búsqueda. Es importante denotar que tampoco se hace un gran aumento del tamaño de las celdas, porque una celda de un tamaño inmenso significa que tiene una cantidad exagerada de puntos contenidos en una celda, lo cual tampoco es lo ideal.
 
### Evidencia antes (`cell_size=90`)
 
Expected source->target transform: theta=-18.00000 deg, tx=-1757.14529, ty=2390.34723
Recovered source->target transform: theta=-18.00595 deg, tx=-1720.41095, ty=1781.98006
Finished after 45 iterations with profile_score=0.01847086
 
| Métrica (`cpu_core`) | sin `--export` | con `--export` |
|---|---|---|
| instructions | 207,001,003,220 | 239,518,364,405 |
| cycles | 86,236,126,691 | 95,066,306,848 |
| branches | 27,275,328,136 | 33,627,719,411 |
| branch-misses | 486,398,735 (1.78%) | 508,427,796 (1.51%) |
| tiempo | 16.137382552 s | 17.858444185 s |

Validación cruzada con Callgrind (sin `--export`):

| Herramienta | Instrucciones totales |
|---|---|
| `perf stat` (`cpu_core/instructions`) | 207,001,003,220 |
| `valgrind --tool=callgrind` (`Ir`) | 207,131,928,312 |
| Diferencia | 130,925,092 (~0.06%) |

Desglose por función (`callgrind_annotate`), `cell_size=90`:

| Función / archivo | Ir | % |
|---|---|---|
| `GridIndex::nearest`  | 168,371,113,599 | 81.29% |
| `stl_vector.h` | 20,911,706,440 | 10.10% |
| `hashtable.h`  | 5,770,394,139 | 2.79% |
| `hashtable_policy.h` | 4,139,601,854 | 2.00% |
| `stl_algobase.h` | 3,680,330,298 | 1.78% |
| `stl_function.h` | 926,270,844 | 0.45% |

 
### Evidencia después (`cell_size=120`)
 
Expected source->target transform: theta=-18.00000 deg, tx=-1757.14529, ty=2390.34723
Recovered source->target transform: theta=-18.00595 deg, tx=-1720.41095, ty=1781.98006
Finished after 45 iterations with profile_score=0.01847086
 
| Métrica (`cpu_core`) | sin `--export` | con `--export` |
|---|---|---|
| instructions | 268,311,321,121 | 300,866,194,898 |
| cycles | 102,030,363,624 | 110,577,614,373 |
| branches | 34,334,838,043 | 40,691,621,133 |
| branch-misses | 424,884,865 (1.24%) | 446,568,959 (1.10%) |
| tiempo | 19.151438072 s | 21.006509671 s |

Validación cruzada con Callgrind (sin `--export`):

| Herramienta | Instrucciones totales |
|---|---|
| `perf stat` (`cpu_core/instructions`) | 268,311,321,121 |
| `valgrind --tool=callgrind` (`Ir`) | 268,425,428,727 |
| Diferencia | 114,107,606 (~0.04%) |

Desglose por función (`callgrind_annotate`), `cell_size=120`:

| Función / archivo | Ir | % |
|---|---|---|
| `GridIndex::nearest` | 224,614,461,128 | 83.68% |
| `stl_vector.h`  | 30,268,009,610 | 11.28% |
| `hashtable.h`  | 4,345,887,247 | 1.62% |
| `hashtable_policy.h` | 3,167,374,100 | 1.18% |
| `stl_algobase.h` | 1,951,996,404 | 0.73% |
| `stl_function.h` | 837,029,019 | 0.31% |

Comparando ambos desgloses: al subir `cell_size` de 90 a 120, la parte de instrucciones en `stl_vector.h` sube de 10.10% a 11.28%, mientras que la porción en `hashtable.h`/`hashtable_policy.h` si una  baja de 4.79% a 2.80%. Esto confirma que si es cierto que hay menos celdas que buscar, pero estas son más grandes por ende, más caras de revisar, aumentando el número de instrucciones
 
Mi hipótesis fue totalmente incorrecta. La justificación es la siguiente: es posible observar que, tanto usando export como sin usarlo, la duración total aumentó unos 3 segundos en ambos casos, junto con un incremento en las instrucciones ejecutadas por los núcleos de rendimiento, así también con los ciclos y los branches. Esto se debe a que, al aumentar el tamaño de la celda, cada celda contiene una mayor cantidad de puntos, y la búsqueda del vecino más cercano debe comparar la distancia contra todos los puntos dentro de la celda visitada. Es decir, aunque se reduce la cantidad de celdas y por lo tanto el radio de celdas que se debe revisar, el costo de revisar cada celda individual crece más de lo que se ahorra en disminuyendo el radio máximo, lo que termina agregando una mayor cantidad de instrucciones y ciclos, aumentando el tiempo de ejecución en vez de reducirlo.

---
 
## Cambio número 2. Modificación del criterio de convergencia de 0.1% a 0.5%
 
La hipótesis es que como el criterio de convergencia trata de que la transformación de la nube fuente se asemeje lo más posible a la nube objetivo obtenida de la búsqueda de vecino, que sucede, como el criterio actualmente se encuentra que la diferencia sea menor al 0.1%, significa que es necesario muchas iteraciones hasta cumplir el criterio, ahora si se sube que la diferencia sea del 0.5%, significa que la cantidad de iteraciones realizadas para que se cumpla el criterio es menor disminuyendo el tiempo, instrucciones, ciclos y branches del código, como contraparte, es probable que la precisión se vea perjudicada.
 
### Evidencia Después
 
Expected source->target transform: theta=-18.00000 deg, tx=-1757.14529, ty=2390.34723
Recovered source->target transform: theta=-17.07254 deg, tx=-1667.14488, ty=1675.26183
Finished after 36 iterations with profile_score=0.01858543
 
| Métrica (`cpu_core`) | sin `--export` | con `--export` |
|---|---|---|
| instructions | 161,060,380,467 | 187,955,610,021 |
| cycles | 69,655,475,781 | 77,032,841,878 |
| branches | 21,479,361,674 | 26,742,059,408 |
| branch-misses | 431,426,766 (2.01%) | 450,452,104 (1.68%) |
| tiempo | 13.161231565 s | 14.752698873 s |

Validación cruzada con Callgrind (sin `--export`):

| Herramienta | Instrucciones totales |
|---|---|
| `perf stat` (`cpu_core/instructions`) | 161,060,380,467 |
| `valgrind --tool=callgrind` (`Ir`) | 161,160,582,632 |
| Diferencia | 100,202,165 (~0.06%) |

esglose por función (`callgrind_annotate`), `VARIATION=0.5%`:

| Función / archivo | Ir | % |
|---|---|---|
| `GridIndex::nearest` | 129,328,324,585 | 80.25% |
| `stl_vector.h` | 15,509,107,218 | 9.62% |
| `hashtable.h` | 5,368,148,155 | 3.33% |
| `hashtable_policy.h` | 3,857,337,166 | 2.39% |
| `stl_algobase.h` | 3,510,520,630 | 2.18% |
| `stl_function.h` | 847,015,539 | 0.53% |


 
La hipótesis fue correcta, en este caso sí se cumplió el bajón de la duración siendo de 3 segundos menos para con y sin export, esto se debe a principalmente a lo mencionado, como se realizaron 9 iteraciones menos, eso es una disminución en las instrucciones, ciclos, branches que conllevaban al cálculo de la transformada, como el criterio de convergencia es menos exigente, eso permite que se llegue a cumplir tanto el profile_score como también en transform_step de una forma más rápida, como contraparte, la precisión se vio afectada, tanto el ángulo que terminó con un grado de diferencia, la traslación en X hubo 90 unidades de diferencia pero en la traslación en Y hubo una diferencia grande de unas 700 unidades, todo esto comparándolo a la precisión que tiene con 0.1% de criterio de convergencia donde el resultado es muy cercano a lo esperado. Hay que valorar si para el sistema en que se está utilizando el mecanismo ICP afecta mucho esa diferencia de precisión y podría terminar afectando su funcionamiento.

Al igual que en el Cambio 1, `perf` y Callgrind coinciden con una diferencia muy baja en el total de instrucciones.



## Uso de IA

Nota sobre utilización de herramientas de IA

Se hace uso de herramientas de IA como apoyo para comprender conceptos, generar ideas y mejorar la redacción de la documentación. La implementación, la validación y los resultados son responsabilidad del estudiante, quien asume la responsabilidad por el uso indebido o no descrito anteriormente.

Se adjuntan los enlaces compartidos de las conversaciones como evidencia.

Gerson: https://claude.ai/share/4789289c-86c7-4bba-838a-ca3628b3ef7b

Nicole: https://chatgpt.com/share/6aad6aff-5c2c-83e8-835d-f05c4bab4bec

Keilin: https://chatgpt.com/share/6aad704e-55e8-83e8-b9dd-5a009b43692d

Gonzalo: https://chatgpt.com/share/6aad727f-1db8-83e8-94b1-f0d1c4db6d14

## Profesor

Dr. Luis G. Leon Vega

## Autor

Gonzalo Alpizar Salas
Gerson Adrian Cordero Zuniga
Nicole Irina Corrales Rodriguez
Keilin Tatiana Loasiga Tellez

II Semestre 2026
