# Control del brazo de 5 GDL con ESP32

Firmware Arduino/PlatformIO para cinco motores paso a paso con DRV8825. La interfaz envía y recibe una línea JSON terminada en `\n` por el puerto serie a 115200 baudios.

## Configuración inicial

- El proyecto apunta a `esp32dev` (ESP32-WROOM DevKit clásico), con el pinout propuesto confirmado por el usuario. STEP: GPIO 25, 26, 27, 32 y 33; DIR: GPIO 16, 17, 18, 19 y 21; ENABLE compartido: GPIO 23.
- Si el cableado cambia o cada DRV8825 usa ENABLE separado, ajusta `kStepPins`, `kDirectionPins` y `kEnablePins` en `src/main.cpp`.
- Configuración de microstepping: paso completo, M0/M1/M2 en LOW en cada DRV8825. El firmware usa 200 pasos/vuelta de motor en todos los ejes; el tamaño NEMA no cambia ese cálculo.
- Reducciones motor:eje de salida para los cinco ejes: 3.5, 83.3, 43.3, 4.75 y 3. Los ángulos de `axes` son ángulos de articulación/eje de salida; los pasos calculados del motor incorporan la reducción.
- Con paso completo, el incremento angular por paso queda aproximadamente en 0.514°, 0.0216°, 0.0416°, 0.379° y 0.600° por eje, respectivamente. Las estimaciones reportadas se cuantizan a esos pasos.
- Los cinco sentidos comienzan en `+1`. Cambia `directionSign` a `-1` en el eje correspondiente si el sentido físico no coincide.
- Los límites iniciales son +/-10 grados respecto al origen lógico de encendido. Son una protección temporal para ensayos pequeños y deben ajustarse por eje tras revisar el rango mecánico real.
- ENABLE se activa al arrancar para mantener la pose y sigue activo para sostener torque. Ajusta el límite de corriente de cada DRV8825 al motor y a la refrigeración antes de energizarlos.
- El servo usa `kToolPin = 22`, cerrado=0° y abierto=55°, valores confirmados por el usuario. Cambia el GPIO si modificas el cableado.
- Alimenta el servo con una fuente adecuada para su corriente y une la tierra de esa fuente con GND del ESP32. No alimentes un servo de carga desde el pin 3V3 del ESP32.

## Referencia y limitaciones

Al encender, el firmware toma la pose actual como cero lógico y no mueve el brazo. `HOME` vuelve a asignar cero en la pose actual cuando los motores están detenidos. Sin encoders ni finales de carrera, esto no detecta movimiento manual, pérdida de pasos ni cambios de pose mientras el ESP32 está apagado. Las posiciones enviadas son cálculos basados en los pulsos ordenados, no mediciones del eje.

Algunas placas ESP32 se reinician cuando el HMI abre el puerto USB serie. Si sucede, el firmware volverá a tomar la pose del momento como cero lógico y las secuencias guardadas dejarán de representar coordenadas absolutas del robot.

`STOP` solicita una desaceleración usando AccelStepper; no es una parada de emergencia certificada. Conserva los drivers habilitados después de mover para mantener torque. Instala los DRV8825 con alimentación y disipación adecuadas, y prueba inicialmente con corriente limitada y el brazo despejado.

## Compilar

Abre esta carpeta con PlatformIO y compila el entorno `esp32dev`. Las dependencias declaradas son AccelStepper, ArduinoJson y ESP32Servo. Si usas Arduino IDE, crea un sketch con `setup()` y `loop()` a partir de `src/main.cpp`, instala esas bibliotecas y selecciona una placa ESP32 compatible.

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
