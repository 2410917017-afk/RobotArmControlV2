# Firmware RoboDK por USB serial (5 GDL)

Firmware independiente del HMI y del firmware original. El ESP32 se conecta a la computadora únicamente por USB serial a 115200 baudios. Los mensajes del controlador son JSON delimitado por líneas.

RoboDK ejecuta un driver local y se comunica con él por stdin/stdout. Este driver (`robodk_driver.py`) usa el puerto COM USB y traduce los comandos RoboDK al protocolo JSON del ESP32. RoboDK documenta `CONNECT COMx PUERTO DOF` y comandos como `MOVJ`/`CJNT` en [Robot Drivers](https://robodk.com/doc/en/Robot-Drivers.html) y [Driver commands](https://robodk.com/doc/en/PythonAPI/driver.html).

## Preparación

1. Revisar en `src/main.cpp` pines, reducciones, microstepping, sentidos, corriente de drivers y límites. **Los límites del firmware están configurados actualmente a ±360° en todos los ejes; sustituir por límites mecánicos conservadores antes de energizar/mover el brazo.**
2. Abrir `firmware_robodk` como proyecto PlatformIO y carga el firmware al ESP32. Es un proyecto separado del `platformio.ini` en la raíz.
3. Instalar `pyserial` en el mismo Python que ejecutará el driver de RoboDK: `python -m pip install -r requirements.txt`. En RoboDK revisar el intérprete configurado en Tools > Options > Python; si usa un Python integrado, instala allí la dependencia.
4. Conectar el ESP32 por USB y averigua el COM asignado en el Administrador de dispositivos de Windows. Cerrar Arduino Serial Monitor/PlatformIO Monitor y cualquier aplicación que tenga abierto ese COM; un puerto serial normalmente admite un solo cliente.
5. En RoboDK cargar o crear un modelo **de cinco ejes** con geometría, límites, orden, sentido y ceros articulares correctos. Ajusta base y TCP a tu brazo.
6. En RoboDK abrir la configuración de conexión del robot: elige `robodk_driver.py` como Driver path, escribe el puerto COM (por ejemplo `COM7`), velocidad `115200` y `5` DOF; conecta. Si la ventana de conexión solo presenta IP/puerto, usa COMx como dirección y 115200 como puerto/baud rate. El driver también permite consola: `CONNECT COM7 115200 5`.
7. Primero consulta posición; luego prueba movimientos articulares pequeños con el área despejada y a baja velocidad. RoboDK debe tener límites iguales o más conservadores que los del firmware.

Al abrir el puerto USB algunas placas reinician el ESP32 mediante DTR/RTS. El driver espera el evento JSON `ready` o una respuesta a `STATUS` hasta 15 s. Si no conecta, verifica COM, velocidad, dependencias, que el monitor serial esté cerrado y que el ESP32 haya arrancado sin error.

## Comandos admitidos por el driver

- `CONNECT`, `DISCONNECT`, `QUIT`, `PAUSE`, `STOP`, `CJNT`, `MOVJ`.
- `SETDO`: mapea una salida digital positiva a gripper abierto y cero a cerrado.
- `MOVL`, `MOVC`, búsqueda lineal, ajuste de velocidad y cambios de TCP se rechazan: no tienen aún una ejecución cartesiana equivalente en el firmware. No los uses esperando que el brazo trace líneas o círculos físicos.

`MOVJ` espera la finalización del movimiento antes de reportar `Ready`. En esta versión simple, la orden RoboDK `STOP` puede no ser procesada por el driver hasta que termine el `MOVJ` bloqueante. Mantén accesible un corte/paro físico; el paro software no es una función de seguridad certificada.

## Protocolo serial manual

115200 baudios, una línea JSON por orden:

```json
{"cmd":"STATUS","id":1}
{"cmd":"MOVE","id":2,"axes":[1,0,0,0,0],"feed":100}
{"cmd":"STOP","id":3}
{"cmd":"TOOL","id":4,"tool":true}
```

La posición reportada es una estimación del movimiento ordenado, no una lectura de encoder. Al encender, el ESP32 toma la pose actual como cero lógico.
