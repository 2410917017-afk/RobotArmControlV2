# HMI para brazo de 5 GDL

Aplicación PyQt6 dividida en módulos:

- `main.py` / `app.py`: interfaz manual, IK/FK, conexión, gripper y teach pendant.
- `serial_worker.py`: conexión serie en un QThread, con JSON por líneas y cola de salida.
- El ESP32 usa FastAccelStepper para generar pulsos desde los periféricos/temporizadores del ESP32; el HMI registra confirmaciones `ack` y conteos de pasos para ayudar a diagnosticar los movimientos.
- `kinematics.py`: cinemática directa DH estándar e inversa numérica DLS para XYZ + yaw/pitch.
- `digital_twin.py`: esquema isométrico ligero con QPainter y marcas para puntos teach.
- `teach_store.py`: persistencia local de secuencias JSON.
- `robot_config.py`: geometría DH estándar en milímetros, offsets y límites configurables. La quinta fila usa `a=0`, `d=0`, `alpha=-90°`, según el dato añadido para la quinta junta.

## Ejecutar

Usa Python 3.10 o posterior. En una terminal dentro de `hmi/`:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python main.py
```

Conecta el ESP32 por USB, selecciona su puerto y pulsa **Conectar**. El HMI consulta el estado cada segundo. La interfaz y el ESP32 intercambian líneas JSON a 115200 baudios.

## Control y modelo

- El ESP32 recibe `MOVE` con cinco ángulos articulares en grados (ejes de salida), no coordenadas de motor; la reducción está aplicada por el firmware.
- Jog articular y cartesiano envían destinos absolutos. La cinemática inversa calcula cinco ángulos y limita a los rangos configurados en `robot_config.py`, que por ahora reflejan el límite inicial +/-10° del firmware.
- Un robot de 5 GDL no puede imponer seis componentes de pose. La interfaz controla XYZ y la dirección del eje herramienta mediante yaw/pitch; no fija el roll.
- `HOME` reasigna cero de software en la pose detenida actual. No hay búsqueda física porque no hay sensores de referencia.
- El gemelo y las coordenadas XYZ son estimaciones abiertas derivadas de los ángulos comunicados por el ESP32. Las cuatro primeras filas DH vienen del código compartido y el usuario confirmó que están en milímetros y usan DH estándar. El TCP es cero por ahora; faltan el TCP real, offsets mecánicos y límites de recorrido para calibrar IK/FK.
- Las secuencias teach guardan cinco ángulos y estado abierto/cerrado del gripper. Repetirlas tras reiniciar requiere volver a alinear físicamente la pose de arranque, ya que el ESP32 vuelve a definirla como cero.
- El servo está configurado en el firmware con señal GPIO13, cerrado=0° y abierto=55°, según los datos confirmados.

