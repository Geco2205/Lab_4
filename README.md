# Laboratorio 4

## Comandos utilizados

## Preparación del entorno

```bash
g++ --version

### Si no existe:
sudo apt update
sudo apt install build-essential

### Herramientas de profiling
sudo apt update
sudo apt install linux-tools-common linux-tools-generic \
    valgrind kcachegrind google-perftools libgoogle-perftools-dev

### Para el visor con GStreamer
sudo apt install libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev

### Si perf no corre, verificar versión de kernel
uname -r
perf --version
```

### Compilación y ejecución base

```bash
cd point-cloud-collimation
make clean
make

./point_cloud_collimation
./point_cloud_collimation --export
./point_cloud_collimation --viewer   # requiere entorno gráfico
```

## Ejercicio A — comprensión del programa base

```bash
./point_cloud_collimation --export
```

Archivos revisados en `reconstruction/`: `target_profile.csv`, `source_initial_profile.csv`, `source_final_profile.csv`, `source_motion.csv`, `profile_metrics.csv`, `frame_*.ppm`.

## Ejercicio B — perfilado con herramientas

```bash
### perf stat: eventos generales
perf stat ./point_cloud_collimation
perf stat ./point_cloud_collimation --export

### perf record / report: perfil de muestreo
perf record -g ./point_cloud_collimation
perf report

### Valgrind Callgrind
valgrind --tool=callgrind ./point_cloud_collimation
callgrind_annotate callgrind.out.* | less
kcachegrind callgrind.out.*   # si hay entorno gráfico

### Google Performance Tools: compilar enlazando libprofiler
make clean
make CXXFLAGS="-std=c++17 -O2 -g -Wall -Wextra -pedantic \
    -fno-omit-frame-pointer" \
    GST_LIBS="-Wl,--no-as-needed -lprofiler -Wl,--as-needed \
    $(pkg-config --libs gstreamer-1.0 gstreamer-app-1.0)"

ldd ./point_cloud_collimation | grep profiler

CPUPROFILE=point_cloud.prof ./point_cloud_collimation
ls -lh point_cloud.prof
google-pprof --text ./point_cloud_collimation point_cloud.prof

### Si ldd no muestra libprofiler o no se genera el .prof:
LD_PRELOAD=/lib/x86_64-linux-gnu/libprofiler.so \
    CPUPROFILE=point_cloud.prof ./point_cloud_collimation
```

## Ejercicio C — perfilado mediante revisión de ensamblador

```bash
### Compilar conservando información de depuración
make clean
make CXXFLAGS="-std=c++17 -O2 -g -Wall -Wextra -pedantic \
    -fno-omit-frame-pointer"

### Generar ensamblador
g++ -std=c++17 -O2 -g -S -masm=intel \
    $(pkg-config --cflags gstreamer-1.0 gstreamer-app-1.0) \
    point_cloud_collimation.cpp -o point_cloud_collimation.s

### Desensamblar el binario
objdump -drwC -Mintel ./point_cloud_collimation \
    > point_cloud_collimation.objdump

### Conectar muestras de perf con el ensamblador
perf record -g ./point_cloud_collimation
perf annotate
```

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

**Cambio 1 — `cell_size` (90 → 120):**

```bash
perf stat ./point_cloud_collimation
perf stat ./point_cloud_collimation --export
```

**Cambio 2 — criterio de convergencia (`VARIATION` 0.1% → 0.5%):**

```bash
perf stat ./point_cloud_collimation
perf stat ./point_cloud_collimation --export
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
 
La hipótesis fue correcta, en este caso sí se cumplió el bajón de la duración siendo de 3 segundos menos para con y sin export, esto se debe a principalmente a lo mencionado, como se realizaron 9 iteraciones menos, eso es una disminución en las instrucciones, ciclos, branches que conllevaban al cálculo de la transformada, como el criterio de convergencia es menos exigente, eso permite que se llegue a cumplir tanto el profile_score como también en transform_step de una forma más rápida, como contraparte, la precisión se vio afectada, tanto el ángulo que terminó con un grado de diferencia, la traslación en X hubo 90 unidades de diferencia pero en la traslación en Y hubo una diferencia grande de unas 700 unidades, todo esto comparándolo a la precisión que tiene con 0.1% de criterio de convergencia donde el resultado es muy cercano a lo esperado. Hay que valorar si para el sistema en que se está utilizando el mecanismo ICP afecta mucho esa diferencia de precisión y podría terminar afectando su funcionamiento.



## Uso de IA

Nota sobre utilización de herramientas de IA

Se hace uso de herramientas de IA como apoyo para comprender conceptos, generar ideas y mejorar la redacción de la documentación. La implementación, la validación y los resultados son responsabilidad del estudiante, quien asume la responsabilidad por el uso indebido o no descrito anteriormente.

Se adjuntan los enlaces compartidos de las conversaciones como evidencia.

https://claude.ai/share/4789289c-86c7-4bba-838a-ca3628b3ef7b


## Profesor

Dr. Luis G. Leon Vega

## Autor

Gonzalo Alpizar Salas
Gerson Adrian Cordero Zuniga
Nicole Irina Corrales Rodriguez
Keilin Tatiana Loasiga Tellez

II Semestre 2026