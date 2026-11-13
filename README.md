# Control del brazo de 5 GDL con ESP32
Este proyecto adapta el brazo robótico HELENE, desarrollado por Felix Herbst, Sven Suppelt, Niklas Schäfer, Romol Chadda y Mario Kupnik, Measurement and Sensor Technology Group, Technische Universität Darmstadt. Fuente del diseño: https://github.com/TUDA-MUST/HELENE Artículo: Herbst, F. et al. (2025). “HELENE: Six-Axis Accessible Open-Source 3D-Printed Robotic Arm for Research and Education”. Hardware, 3(3), 7. https://doi.org/10.3390/hardware3030007 Nuestro equipo realizó adaptaciones mecánicas, sustituyó motores, rediseñó piezas e implementó electrónica y programación propias. Este proyecto no es el proyecto oficial de TU Darmstadt.

Firmware Arduino/PlatformIO para cinco motores paso a paso con TB6600. La interfaz envía y recibe una línea JSON terminada en `\n` por el puerto serie a 115200 baudios.

## Configuración inicial

### Parametros DH
|i|θ|d|a|α|
|:-:|:-----:|:-:|:-:|:----:|
|1|0|145|25|90|
|2|90|0|200|0|
|3|0|0|0|90|
|4|0|177|0|-90|
|5|0|0|0|90|

### Relaciones de transmición mecánica
|Junta|Planetario|Polea|Combinados|Pasos por grado|
|:--:|:--:|:--:|:--:|:--:|
|J1||1:3.5|1:3.5|1.94|
|J2|1:50|30:50|1:83.3|46.28|
|J3|1:50|30:52|1:86.6|48.1|
|J4||20:95|1:4.75|2.64|
|J5||20:60|1:3|1.67|

### Límites articulares 
Offset articular (Home):
|||
|--|--|
|J1|0|
|J2|195.0|
|J3|-80|
|J4|0|
|J5|0|

|J1|J2|J3|J4|J5|
|:--:|:--:|:--:|:--:|:--:|
|-160 a 160|0 a 210|0 a 160|-166 a 166|-180 a 180|


- El proyecto apunta a `esp32dev` (ESP32-WROOM DevKit clásico), con el pinout propuesto confirmado por el usuario. STEP: GPIO 26, 17, 21, 23, 33; DIR: GPIO 25, 4, 19, 22, 32; ENABLE compartido: GPIO 27.
- Si el cableado cambia o cada TB6600 usa ENABLE separado, ajusta `kStepPins`, `kDirectionPins` y `kEnablePins` en `src/main.cpp`.
- Configuración de microstepping: paso completo, M0/M1/M2 en LOW en cada TB6600. El firmware usa 200 pasos/vuelta de motor en todos los ejes; el tamaño NEMA no cambia ese cálculo.
- Reducciones motor:eje de salida para los cinco ejes: 3.5, 83.3, 86.6, 4.75 y 3. Los ángulos de `axes` son ángulos de articulación/eje de salida; los pasos calculados del motor incorporan la reducción.
- Con paso completo, el incremento angular por paso queda aproximadamente en 0.514°, 0.0216°, 0.0416°, 0.379° y 0.600° por eje, respectivamente. Las estimaciones reportadas se cuantizan a esos pasos.
- Los cinco sentidos comienzan en `+1`. Cambia `directionSign` a `-1` en el eje correspondiente si el sentido físico no coincide.
- Los límites iniciales son +/-10 grados respecto al origen lógico de encendido. Son una protección temporal para ensayos pequeños y deben ajustarse por eje tras revisar el rango mecánico real.
- ENABLE se activa al arrancar para mantener la pose y sigue activo para sostener torque. Ajusta el límite de corriente de cada DRV8825 al motor y a la refrigeración antes de energizarlos.
- El servo usa `kToolPin = 13`, cerrado=0° y abierto=55°, valores confirmados por el usuario. Cambia el GPIO si modificas el cableado.
- Alimenta el servo con una fuente adecuada para su corriente y une la tierra de esa fuente con GND del ESP32. No alimentes un servo de carga desde el pin 3V3 del ESP32.

## Referencia y limitaciones

Al encender, el firmware toma la pose actual como cero lógico y no mueve el brazo. `HOME` vuelve a asignar cero en la pose actual cuando los motores están detenidos. Sin encoders ni finales de carrera, esto no detecta movimiento manual, pérdida de pasos ni cambios de pose mientras el ESP32 está apagado. Las posiciones enviadas son cálculos basados en los pulsos ordenados, no mediciones del eje.

Algunas placas ESP32 se reinician cuando el HMI abre el puerto USB serie. Si sucede, el firmware volverá a tomar la pose del momento como cero lógico y las secuencias guardadas dejarán de representar coordenadas absolutas del robot.

`STOP` solicita una desaceleración usando FastAccelStepper; no es una parada de emergencia certificada. Conserva los drivers habilitados después de mover para mantener torque. Instala los TB6600 con alimentación y disipación adecuadas, y prueba inicialmente con corriente limitada y el brazo despejado.

## Compilar

Abre esta carpeta con PlatformIO y compila el entorno `esp32dev`. El proyecto usa FastAccelStepper para generar pulsos con los periféricos del ESP32, además de ArduinoJson y ESP32Servo. Si usas Arduino IDE, crea un sketch con `setup()` y `loop()` a partir de `src/main.cpp`, instala esas bibliotecas y selecciona una placa ESP32 compatible.

## Protocolo JSON

Cada objeto ocupa una línea. `id` es opcional y se devuelve para correlacionar respuestas.

```json
{"cmd":"MOVE","id":1,"axes":[2,1,-1,0,0],"feed":300}
{"cmd":"STATUS","id":2}
{"cmd":"STOP","id":3}
{"cmd":"HOME","id":4}
{"cmd":"TOOL","id":5,"tool":true}
```

- `MOVE`: cinco ángulos objetivo de articulación en grados y velocidad máxima en pasos/s (1–800). La ventana provisional es +/-10 grados por eje.
- `STATUS`: solicita posiciones actuales.
- `STOP`: descarta comandos aún en cola y desacelera los motores en movimiento.
- `HOME`: asigna cero a la pose actual; requiere que los cinco ejes estén detenidos.
- `TOOL`: mueve el servo a los ángulos configurados para abierto/cerrado.

Respuestas: `ack`, `error`, `ready`, `status`, `position`, `complete`, `stopping`, `homed` o `tool`. Los objetos de posición contienen por eje el conteo de pasos, ángulo articular estimado y equivalente de giro del motor.


## Protocolo de pruebas
### 1. Preparación y seguridad
- Deja el brazo despejado y, si es posible, apoya o descarga las articulaciones que puedan caer por gravedad. Ten a la mano una forma de cortar la alimentación de potencia de los motores. El botón STOP del programa desacelera los motores; no es un paro de emergencia.
- Revisa el manual y la tabla DIP de tus módulos TB6600: configura corriente según la corriente nominal del motor y el microstepping que elegiste. No te guíes solo por el nombre TB6600: el integrado admite varias resoluciones, pero el módulo determina cómo se seleccionan. Ficha técnica de Toshiba
- Antes de cambiar DIP, cablear o desconectar un motor, apaga la fuente. No conectes ni desconectes los cables del motor con el driver energizado.
- Comprueba el cableado STEP/DIR/ENABLE y la referencia eléctrica según el manual del módulo. Verifica que sus entradas acepten correctamente la señal de 3.3 V del ESP32; si no, usa una interfaz de nivel adecuada. No asumas que todos los módulos con la etiqueta TB6600 tienen la misma entrada.
- Confirma en el firmware que microsteps coincide con el ajuste DIP de cada driver. Si no coincide, el movimiento angular calculado no corresponderá al movimiento real.
### 2. Pruebas iniciales de movimiento
1. Con potencia de motores apagada, conecta USB, abre el monitor serial a 115200 baudios y envía:
   {"cmd":"STATUS","id":1}
   Confirma que llega la respuesta ready/`status` con cinco ejes.
2. Con el brazo despejado y la potencia activada, prueba un solo eje a la vez, empezando con un cambio de 1° y velocidad baja:
   {"cmd":"MOVE","id":2,"axes":[1,0,0,0,0],"feed":50}
   Después vuelve J1 a cero:
   {"cmd":"MOVE","id":3,"axes":[0,0,0,0,0],"feed":50}
   Repite cambiando a 1 únicamente la segunda, tercera, cuarta y quinta posición del arreglo para probar J2–J5. Si un eje se mueve al sentido opuesto al esperado, detén la prueba y revisa el sentido configurado antes de seguir.
3. Compara el ángulo ordenado con el giro observado. Si no coincide, revisa, en este orden: microstepping DIP frente a microsteps del firmware, relación de transmisión, sentido y cableado de fases. No intentes corregirlo ampliando límites.
4. Cuando cada eje responda individualmente, prueba recorridos algo mayores, todavía uno por uno. Observa si el motor pierde pasos, vibra, se calienta demasiado o si el driver se protege. Si sucede, corta potencia y revisa la corriente configurada, alimentación, cableado y carga antes de repetir.
5. Prueba el paro con un movimiento suficientemente largo para que puedas pulsarlo mientras se mueve. El firmware usa una desaceleración, así que el brazo puede continuar moviéndose un poco después del comando.
   {"cmd":"MOVE","id":10,"axes":[0,10,0,0,0],"feed":50}
   {"cmd":"STOP","id":11}
   Asegúrate de que ese ángulo esté dentro del rango mecánico seguro de tu brazo.
6. Prueba HOME solo cuando el brazo esté detenido y colocado en una pose que quieras tomar como cero lógico:
   {"cmd":"HOME","id":12}
   No busca una posición física; asigna cero a la pose actual.
### 3. Pruebas de la interfaz
1. Conexión y estado: conecta la interfaz y comprueba que muestra los cinco ejes y recibe posiciones.
2. Jogs articulares: mueve cada eje con incrementos pequeños; confirma sentido y que el gemelo digital se actualiza.
3. STOP: inicia un movimiento y activa el botón; confirma que deja de avanzar tras desacelerar.
4. Gripper: prueba abrir y cerrar con el brazo despejado. Comprueba que el servo tiene alimentación adecuada y referencia común con el ESP32 si así lo requiere tu montaje.
5. Cinemática directa e inversa: primero calcula una pose cercana a la actual; prueba movimientos pequeños y compara la respuesta visual y física.
6. Teach pendant: guarda una pose corta, muévete a otra y reproduce la guardada a baja velocidad. Después prueba una secuencia de dos puntos.
7. Límites: verifica que la interfaz y el firmware rechacen objetivos fuera de rango. En el firmware actual los límites son distintos por eje; comprueba que la interfaz no permita valores que el ESP32 vaya a rechazar y que ninguno de los rangos exceda los topes físicos.
8. Reconexión: detén el brazo, desconecta y reconecta la interfaz, y solicita STATUS. Recuerda que apagar el ESP32 borra la referencia de posición estimada: vuelve a colocar el brazo en una pose conocida antes de continuar.

## Tabla de fallos
| Área | Falla probable / síntoma | Causa posible | Solución probable |
|---|---|---|---|
| **Hardware** | El ESP32 se reinicia al mover motores | Caída de tensión, fuente insuficiente o ruido eléctrico | Verificar que la fuente soporte la corriente de los motores; separar la alimentación lógica y la de potencia según el diseño; revisar conexiones y tierras. |
| **Hardware** | Un motor no se mueve o pierde fuerza | Fuente desconectada, driver deshabilitado o corriente mal configurada | Comprobar alimentación del TB6600, estado de ENABLE, cableado y ajuste de corriente conforme al motor y al módulo. |
| **Hardware** | Motores o drivers se calientan demasiado | Corriente excesiva, ventilación insuficiente o carga elevada | Apagar y dejar enfriar; revisar la corriente configurada, disipación y carga mecánica. |
| **Electrónica** | El motor vibra, zumba o se mueve a tirones | Fases del motor conectadas incorrectamente, señal STEP/DIR deficiente o corriente inadecuada | Identificar correctamente los pares de bobinas; revisar señales, cableado y configuración del driver. |
| **Electrónica** | El movimiento es errático o el ESP32 pierde comunicación | Ruido electromagnético, cables largos o tierras mal conectadas | Separar cables de potencia y señal, reducir recorridos de cable, asegurar conexiones y aplicar el esquema de puesta a tierra recomendado para el montaje. |
| **Electrónica** | El driver no responde a las señales del ESP32 | Entradas del módulo incompatibles con la señal de 3.3 V, cableado incorrecto o ENABLE activo/inactivo al revés | Consultar el manual del módulo; verificar STEP, DIR, ENABLE y referencia eléctrica; usar una interfaz de nivel si el módulo la requiere. |
| **Electrónica** | El motor gira en sentido opuesto | DIR invertido o sentido configurado al revés en firmware | Cambiar la configuración de sentido de ese eje o corregir el cableado según el diseño. Probar primero con recorrido pequeño. |
| **Software** | La interfaz conecta, pero no hay movimiento | JSON mal formado, comando no reconocido, ejes incorrectos o driver deshabilitado | Revisar la respuesta `error` del ESP32; enviar `STATUS`; confirmar que `MOVE` incluya cinco ángulos y un `feed` válido. |
| **Software** | El ESP32 rechaza un movimiento | Objetivo fuera de límites o velocidad fuera del rango permitido | Revisar los límites del eje en firmware y HMI; enviar objetivos dentro del recorrido permitido y reducir `feed`. |
| **Software** | El brazo se mueve más o menos de lo esperado | Relación de transmisión o microstepping mal configurados | Comparar `microsteps` del firmware con los DIP del TB6600 y verificar la reducción motor/articulación de cada eje. |
| **Software** | La posición mostrada no coincide con la posición física | El firmware estima la posición a partir de los pasos; puede haber pasos perdidos o movimiento manual con el equipo apagado | Recolocar el brazo en una pose conocida y volver a establecer el cero lógico. Para conocer posición real se requieren encoders o sensores de referencia. |
| **Software** | El botón HOME no lleva el brazo a una pose fija | HOME solo asigna cero de software en la pose actual | Colocar manualmente el brazo en una pose de referencia antes de pulsarlo. Para homing automático, instalar finales de carrera o sensores de referencia y programar su búsqueda. |
| **Software** | STOP tarda en detener el brazo | STOP solicita desaceleración; no corta físicamente la potencia | Considerar la distancia de frenado al probar. Para una parada de emergencia real, instalar un circuito físico que interrumpa la energía de los motores de forma segura. |
| **Mecánica** | El brazo se atasca o no puede sostener una articulación | Interferencia, carga excesiva, fricción o par insuficiente | Apagar la potencia; revisar topes, alineación, rodamientos, tornillería y carga; probar el eje descargado antes de volver a montar la carga. |
| **Mecánica** | Hay holgura o el eje no repite la posición | Correas flojas, acoplamientos sueltos, engranajes desgastados o juego en piezas impresas | Revisar tensión de correas y fijaciones; comprobar desgaste, grietas y holguras; repetir la calibración tras cualquier ajuste. |
| **Mecánica** | El brazo golpea el bastidor o llega al tope | Límites de software incorrectos o cero lógico mal establecido | Detener y revisar el cero y los límites de cada articulación; establecer límites conservadores dentro del recorrido mecánico real. |
| **Mecánica** | El gripper no abre/cierra correctamente | Servo trabado, recorrido excesivo, alimentación insuficiente o mecanismo con fricción | Revisar el mecanismo y la fuente del servo; ajustar el recorrido angular gradualmente para que no fuerce los topes. |


