# blueboat_teleop

Paquete ROS 2 para teleoperar BlueBoat con mando publicando `geometry_msgs/msg/Twist`
en `/cmd_vel`, pensado para encajar con la cadena:

`/cmd_vel -> body_velocity_controller -> body_force_controller -> thrusters`

## Lanzamiento

```bash
ros2 launch blueboat_teleop blueboat_teleop.launch.py
```

O junto con la simulación/controladores:

```bash
ros2 launch blueboat_sura blueboat_sura.launch.py environment:=sim use_teleop:=true
```

Con F310 por indice de dispositivo:

```bash
ros2 launch blueboat_teleop blueboat_teleop.launch.py joy_device_id:=0
```

Con perfil PS4:

```bash
ros2 launch blueboat_teleop blueboat_teleop.launch.py \
  config_file:=/home/mario-cirtesu/blueboat_ws/src/blueboat_teleop/config/ps4.yaml
```

## Comprobaciones rápidas

```bash
ros2 run joy joy_enumerate_devices
ros2 topic echo /joy --once
ros2 control list_controllers
```

En el perfil Logitech F310:

- `RB + LB`: activar/desactivar modo fuerza directa (`body_force_controller`).
- `RB + X`: activar/desactivar modo velocidad (`body_velocity_controller` encima de `body_force_controller`).
- `B`: reservado para futuro control de posición.

En modo velocidad, `body_force_controller` debe quedarse activo porque
`body_velocity_controller` escribe sobre sus interfaces encadenadas.
