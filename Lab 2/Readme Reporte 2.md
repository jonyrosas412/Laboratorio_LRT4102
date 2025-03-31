# Lab 2 - ROS y Turtlesim

## Objetivo
Crear un paquete ROS con nodos de comunicación básica (talker/listener) y verificar su funcionamiento.

### Procedimiento
**Creación del paquete**:
  ```bash
  catkin_create_pkg Practicas_lab rospy roscpp std_msgs
```
**Compilación** :

  ```bash
  catkin_make
  source devel/setup.bash
```
**Ejecución**
Para ejecutar los programas talker.py y listener.py ejecutamos las siguientes líneas de código en una ventana de terminator.
  ```bash
  roscore
```
Después de ejecutar roscore dividimos verticalmente y ejecutamos la siguiente línea: 
  ```bash
  rosrun Practicas_lab talker.py
```
Dividimos nuevamente en horizontal y ejecutamos la siguiente línea:
  ```bash
  rosrun Practicas_lab listener.py
```

Como resultado se obtuvo lo siguiente:
![image](https://github.com/user-attachments/assets/6a2187b8-6da9-4b24-93d7-cb0ab45597f1)
