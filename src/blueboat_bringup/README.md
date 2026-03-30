# catamaran_bringup

## Qué es

`catamaran_bringup` es el paquete encargado de arrancar el sistema del catamarán.

Se encarga de:

* generar la descripción del robot (URDF desde Xacro),
* lanzar `ros2_control_node`,
* cargar el hardware interface,
* cargar los controladores.

---

## Modo de ejecución

El sistema puede arrancarse en dos modos:

* `real` --> para el robot real (por defecto se lanza este)
* `sim` --> para usarse con la simulacion en Stonefish

Ejemplo:

```bash
ros2 launch catamaran_bringup bringup.launch.py environment:=sim
```

---

## Estado

Actualmente orientado a pruebas del sistema y control básico del catamarán.
