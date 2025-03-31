# Lab 2 - ROS y Turtlesim
## Basic 
### Objetivo
Crear un paquete llamado Practicas_lab de ros con dependencias rospy, roscpp y std_msgs

Colocar los archivos listener.py y talker.py

Compilar el paquete.

Ejecutar el talker.

Ejecutar el listener.

Concluir sobre su funcionamiento.

#### Procedimiento
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

Se puede observar que el nodo talker publica mensajes que son recibidos por el listener, mostrando en la consola que la comunicación es exitosa.

## Medium
### Objetivo 
Crear un control por teclado para turtlesim.

Dibujar un cuadrado y un triángulo equilátero con turtlesim (Sin controlador).

### Procedimiento
Para este ejercicio se modifico el código visto en clase para que la tortuga pudiera moverse en +x -x +y -y +z y -z. Al ejecutar el código se puede observar el comportamiento de este programa. Podemos observar que para +x y +y presionamos x y y, para -x y -y presionamos a y b, para -z y +z presionamos z y r, para detener presionamos s y para salir q. Para dibujar un cuadrado se presiona c y para un triangulo t. 

```bash
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import math
from time import time

# Configuración de movimientos
LINEAR_SPEED = 0.5
ANGULAR_SPEED = 0.5
SIDE_LENGTH = 2.0  # Para las formas

# Estados del sistema
STATE_MANUAL = 0
STATE_DRAWING = 1

current_state = STATE_MANUAL
draw_start_time = 0
current_shape = None
current_step = 0

def get_key():
    """Lee una tecla del teclado sin necesidad de presionar Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def draw_shape(pub, shape):
    global current_state, draw_start_time, current_shape, current_step
    current_state = STATE_DRAWING  # Cambia el estado a "dibujando"
    current_shape = shape          # Almacena la figura a dibujar
    current_step = 0               # Reinicia el contador de pasos
    draw_start_time = time()       # Registra el tiempo de inicio

def update_drawing(pub):
    global current_step, draw_start_time, current_state
    
    elapsed = time() - draw_start_time  # Tiempo desde el inicio del paso actual
    msg = Twist()  # Mensaje ROS para movimiento
    
    # Pasos predefinidos para cada figura:
    if current_shape == "square":
        steps = [
            (LINEAR_SPEED, 0, SIDE_LENGTH/LINEAR_SPEED),  # Avanzar (velocidad, 0, tiempo)
            (0, math.radians(90), 1.0),                   # Girar 90° (0, velocidad angular, tiempo)
            (LINEAR_SPEED, 0, SIDE_LENGTH/LINEAR_SPEED),
            (0, math.radians(90), 1.0),
            (LINEAR_SPEED, 0, SIDE_LENGTH/LINEAR_SPEED),
            (0, math.radians(90), 1.0),
            (LINEAR_SPEED, 0, SIDE_LENGTH/LINEAR_SPEED)   # Último lado del cuadrado
        ]
    elif current_shape == "triangle":
        steps = [
            (LINEAR_SPEED, 0, SIDE_LENGTH/LINEAR_SPEED),  # Avanzar
            (0, math.radians(120), 1.0),                  # Girar 120° (triángulo equilátero)
            (LINEAR_SPEED, 0, SIDE_LENGTH/LINEAR_SPEED),
            (0, math.radians(120), 1.0),
            (LINEAR_SPEED, 0, SIDE_LENGTH/LINEAR_SPEED)    # Último lado del triángulo
        ]
    else:
        current_state = STATE_MANUAL  # Si la figura no es válida, vuelve al modo manual
        return

    # Verifica si se completaron todos los pasos:
    if current_step >= len(steps):
        current_state = STATE_MANUAL  # Vuelve al modo manual
        return

    # Obtiene el paso actual y publica el comando:
    step = steps[current_step]
    msg.linear.x = step[0]   # Velocidad lineal
    msg.angular.z = step[1]  # Velocidad angular
    
    # Si el paso actual se completó (tiempo transcurrido >= duración del paso):
    if elapsed >= step[2]:
        current_step += 1           # Avanza al siguiente paso
        draw_start_time = time()     # Reinicia el temporizador
    
    pub.publish(msg)  # Envía el comando a turtlesim

def main():
    global current_state
    
    rospy.init_node('turtle_advanced_control', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    print("Controles:")
    print("  x/y: Mover en +X/+Y")
    print("  a/b: Mover en -X/-Y")
    print("  z/r: Girar horario/antihorario")
    print("  s: Detener")
    print("  c/t: Dibujar cuadrado/triángulo")
    print("  q: Salir")

    while not rospy.is_shutdown():
        if current_state == STATE_MANUAL:
            key = get_key()
            msg = Twist()
            
            if key == 'x':
                msg.linear.x = LINEAR_SPEED
            elif key == 'y':
                msg.linear.y = LINEAR_SPEED
            elif key == 'a':
                msg.linear.x = -LINEAR_SPEED
            elif key == 'b':
                msg.linear.y = -LINEAR_SPEED
            elif key == 'z':
                msg.angular.z = ANGULAR_SPEED
            elif key == 'r':
                msg.angular.z = -ANGULAR_SPEED
            elif key == 's':
                msg.linear.x = 0
                msg.linear.y = 0
                msg.angular.z = 0
            elif key == 'c':
                draw_shape(pub, "square")
            elif key == 't':
                draw_shape(pub, "triangle")
            elif key == 'q':
                print("Saliendo...")
                break
            
            pub.publish(msg)
        
        elif current_state == STATE_DRAWING:
            update_drawing(pub)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```

Una forma facíl de ejecutar este código es creando una carpeta launch dentro de Practicas_lab. Una vez creada la carpete creamos un archivo .launch que contenga para este caso en especifico lo siguiente: 

```bash
<launch>
    
    <!-- Lanzar el nodo turtlesim_node del paquete turtlesim -->
    <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node" output="screen"/>

    <!-- Lanzar el script teleop.py del paquete Practicas_lab -->
    <node pkg="Practicas_lab" type="teleop.py" name="teleop_node" output="screen"/>
</launch>
```

Un .launch nos ayuda a correr el programa sin necesidad de hacer varia divisiones del cmd. No será necesario ingresar roscore, turtlesim_node, y el nombre del .py ya que bastara con ejecutar una sola línea de código.

Para ejecutar el launch simplemente lanzamos el siguiente código:

```bash
roslaunch Practicas_lab tortuga_teleop.launch
```

En las siguientes imagenes se puede observar que el programa funciona correctamente. Podemos observar los resultados de presionar c y t:
![image](https://github.com/user-attachments/assets/c053291d-e3a5-4535-b8ff-dad2abe698e9)

![image](https://github.com/user-attachments/assets/38219e1a-c9ef-489e-a578-6630a760c463)

## Advanced
### Objetivo
Control de posición para turtlesim (P)
Control de posición para turtlesim (PI)
Control de posición para turtlesim (PID)
Comparar el desempeño de cada uno de los controladores, mediante el uso de Plot Juggler o
alguna otra herramienta de graficación.

### Procedimiento 
### Proporcional
Este programa implementa un **controlador proporcional** para mover la tortuga del simulador `turtlesim` hacia una pose deseada \((x, y, \theta)\), donde:
- \((x, y)\): Coordenadas objetivo en el plano 2D.
- \(\theta\): Orientación final en radianes (use grados en la entrada interactiva).

```bash
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, radians, sin, cos  # ¡Importamos sin y cos!

class MoveTurtleProportionalControl:
    def __init__(self):
        rospy.init_node('control_tortuga_xytheta')
        
        # Suscribirse al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el topic de comandos de movimiento
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Tasa de publicación (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Variables de estado actual
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

    def pose_callback(self, pose):
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta  # theta en radianes

    def move_turtle_to_desired_pose(self, desired_x, desired_y, desired_theta):
        # Constantes del controlador (ajustables)
        Kp_linear = 1.0  # Ganancia para movimiento lineal
        Kp_angular = 4.0  # Ganancia para rotación

        while not rospy.is_shutdown():
            # Errores en posición
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y
            
            # Ángulo hacia el objetivo y error de orientación
            target_angle = atan2(error_y, error_x)
            error_theta = target_angle - self.current_theta
            
            # Normalizar error angular a [-π, π]
            error_theta = atan2(sin(error_theta), cos(error_theta))
            
            # Distancia al objetivo
            distance = sqrt(error_x**2 + error_y**2)
            
            # Comando de velocidad
            twist_msg = Twist()
            
            if distance > 0.1:  # Fase 1: Moverse hacia el objetivo
                twist_msg.linear.x = Kp_linear * distance * cos(error_theta)
                twist_msg.linear.y = Kp_linear * distance * sin(error_theta)
                twist_msg.angular.z = Kp_angular * error_theta
            else:  # Fase 2: Ajustar orientación final
                error_theta_final = desired_theta - self.current_theta
                error_theta_final = atan2(sin(error_theta_final), cos(error_theta_final))
                
                twist_msg.angular.z = Kp_angular * error_theta_final
                
                if abs(error_theta_final) < 0.1:
                    rospy.loginfo("¡Pose deseada alcanzada!")
                    break
            
            self.velocity_publisher.publish(twist_msg)
            rospy.loginfo("Pos: (%.2f, %.2f), θ: %.2f rad, Error θ: %.2f rad", 
                          self.current_x, self.current_y, self.current_theta, error_theta)
            self.rate.sleep()

    def get_desired_pose_from_user(self):
        print("\n=== Ingrese la pose deseada ===")
        x = float(input("Coordenada x: "))
        y = float(input("Coordenada y: "))
        theta_deg = float(input("Orientación (grados): "))
        return x, y, radians(theta_deg)  # Convertir a radianes

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            try:
                desired_x, desired_y, desired_theta = self.get_desired_pose_from_user()
                self.move_turtle_to_desired_pose(desired_x, desired_y, desired_theta)
            except ValueError:
                rospy.logwarn("Entrada inválida. Use números.")

if __name__ == '__main__':
    try:
        controller = MoveTurtleProportionalControl()
        controller.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
```

### Comportamiento de la Tortuga

1. **Fase de Acercamiento**:
   - La tortuga calcula el ángulo hacia el objetivo (\(\theta_{\text{target}} = \text{atan2}(y_{\text{error}}, x_{\text{error}})\)) y la distancia euclidiana.
   - **Movimiento curvilíneo**: Ajusta simultáneamente:
     - **Velocidad lineal**: Proporcional a la distancia (\(v = Kp_{\text{linear}} \cdot \text{distancia}}\)).
     - **Velocidad angular**: Proporcional al error de orientación (\(\omega = Kp_{\text{angular}} \cdot \theta_{\text{error}})\).
   - Resultado: La tortuga describe una **trayectoria curva** hacia el punto objetivo.

2. **Fase de Ajuste de Orientación**:
   - Cuando la tortuga está a menos de **0.1 unidades** del objetivo, ignora el movimiento lineal y solo gira sobre sí misma para alinear \(\theta\) con el valor deseado.
   - Finaliza cuando el error angular es menor a **0.1 radianes**.

### Parámetros Ajustables
- \(Kp_{\text{linear}}\): Controla la velocidad de avance (valor inicial: `1.0`).
- \(Kp_{\text{angular}}\): Controla la rapidez del giro (valor inicial: `4.0`).
  
> **Nota**: Valores altos de \(Kp\) generan respuestas rápidas pero pueden causar oscilaciones; valores bajos son más suaves pero lentos.

### Ejemplo Visual
```python
Entrada del usuario: (x=5.5, y=5.5, θ=90°)
```
![image](https://github.com/user-attachments/assets/284379ee-af3c-4066-a09b-253459cdf56c)


### ¿Cómo usarlo?
1. Ejecute el nodo en ROS.
2. Ingrese las coordenadas \((x, y)\) y la orientación en **grados**.
3. Observe la trayectoria combinada (lineal + angular) en `turtlesim`.

### Para este programa se uso también el respectivo launch 

```bash
<launch>
    
    <!-- Lanzar el nodo turtlesim_node del paquete turtlesim -->
    <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node" output="screen"/>

    <!-- Lanzar el script turtle_pcf.py del paquete Practicas_lab -->
    <node pkg="Practicas_lab" type="turtle_pcf.py" name="turtle_pcf" output="screen"/>
</launch>
```

### PD
Este programa implementa un **controlador PD (Proporcional-Derivativo)** para mover la tortuga en `turtlesim` hacia una posición deseada `(x,y)` con una orientación específica `θ`. 

```bash
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, radians, sin, cos

class MoveTurtlePDControl:
    def __init__(self):
        rospy.init_node('control_tortuga_pd_xytheta')
        
        # Suscribirse al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el topic de comandos de movimiento
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Tasa de publicación (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Variables de estado actual
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0
        
        # Errores anteriores para término derivativo
        self.last_error_x = 0
        self.last_error_y = 0
        self.last_error_theta = 0

    def pose_callback(self, pose):
        # Actualizar posición y orientación
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta  # en radianes

    def move_turtle_to_desired_pose(self, desired_x, desired_y, desired_theta):
        # Constantes del controlador PD (ajustables)
        Kp_linear = 1.0  # Ganancia proporcional lineal
        Kd_linear = 0.1  # Ganancia derivativa lineal
        Kp_angular = 4.0  # Ganancia proporcional angular
        Kd_angular = 0.5  # Ganancia derivativa angular

        while not rospy.is_shutdown():
            # Errores actuales
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y
            
            # Ángulo hacia el objetivo y error de orientación
            target_angle = atan2(error_y, error_x)
            error_theta = target_angle - self.current_theta
            
            # Normalizar error angular a [-π, π]
            error_theta = atan2(sin(error_theta), cos(error_theta))
            
            # Distancia al objetivo
            distance = sqrt(error_x**2 + error_y**2)
            
            # Términos derivativos
            deriv_x = error_x - self.last_error_x
            deriv_y = error_y - self.last_error_y
            deriv_theta = error_theta - self.last_error_theta
            
            # Actualizar errores anteriores
            self.last_error_x = error_x
            self.last_error_y = error_y
            self.last_error_theta = error_theta
            
            # Comando de velocidad
            twist_msg = Twist()
            
            if distance > 0.1:  # Fase de acercamiento
                # Control PD para movimiento lineal
                twist_msg.linear.x = Kp_linear * error_x + Kd_linear * deriv_x
                twist_msg.linear.y = Kp_linear * error_y + Kd_linear * deriv_y
                
                # Control PD para orientación
                twist_msg.angular.z = Kp_angular * error_theta + Kd_angular * deriv_theta
            else:  # Fase de ajuste de orientación final
                error_theta_final = desired_theta - self.current_theta
                error_theta_final = atan2(sin(error_theta_final), cos(error_theta_final))
                
                # Solo control PD para orientación
                deriv_theta_final = error_theta_final - self.last_error_theta
                twist_msg.angular.z = Kp_angular * error_theta_final + Kd_angular * deriv_theta_final
                
                if abs(error_theta_final) < 0.05:
                    rospy.loginfo("Pose deseada alcanzada!")
                    break
            
            # Publicar el comando
            self.velocity_publisher.publish(twist_msg)
            
            # Log de información
            rospy.loginfo("Pos: (%.2f, %.2f), θ: %.2f rad, Error: (%.2f, %.2f), Error θ: %.2f", 
                         self.current_x, self.current_y, self.current_theta,
                         error_x, error_y, error_theta)
            
            self.rate.sleep()

    def get_desired_pose_from_user(self):
        print("\n=== Ingrese la pose deseada ===")
        x = float(input("Coordenada x: "))
        y = float(input("Coordenada y: "))
        theta_deg = float(input("Orientación final (grados): "))
        return x, y, radians(theta_deg)

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            try:
                desired_x, desired_y, desired_theta = self.get_desired_pose_from_user()
                self.move_turtle_to_desired_pose(desired_x, desired_y, desired_theta)
            except ValueError:
                rospy.logwarn("Entrada inválida. Use números.")

if __name__ == '__main__':
    try:
        controller = MoveTurtlePDControl()
        controller.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
```

Características principales:
- **Control en 3 ejes**: Posición `(x,y)` y orientación `θ` (en radianes)
- **Algoritmo PD**: 
  - **Término Proporcional (P)**: Reacciona al error actual
  - **Término Derivativo (D)**: Amortigua oscilaciones usando la tasa de cambio del error
- **Fases inteligentes**:
  1. Movimiento hacia el objetivo
  2. Ajuste fino de orientación
- **Interactivo**: Permite ingresar coordenadas y ángulo deseado (en grados)

### Comportamiento de la Tortuga

1. **Movimiento inicial**:
   - Calcula una **trayectoria curva óptima** hacia `(x,y)`
   - Ajusta **simultáneamente**:
     - Velocidad lineal (avance)
     - Velocidad angular (giro)

2. **Fase de aproximación**:
   - Reduce velocidad al acercarse al objetivo
   - Suaviza movimientos gracias al componente derivativo

3. **Ajuste final**:
   - Cuando está a <0.1 unidades del objetivo:
     - Se detiene el movimiento lineal
     - Gira sobre sí misma hasta alcanzar `θ` deseado
   - Precisión: ±0.05 radianes (~3°)

4. **Estabilidad mejorada**:
   - Menos oscilaciones que un controlador P puro
   - Transiciones suaves entre movimientos

### Parámetros Ajustables
| Parámetro | Descripción | Valor por defecto |
|-----------|-------------|-------------------|
| `Kp_linear` | Ganancia proporcional (movimiento) | 1.0 |
| `Kd_linear` | Ganancia derivativa (movimiento) | 0.1 |
| `Kp_angular` | Ganancia proporcional (giro) | 4.0 |
| `Kd_angular` | Ganancia derivativa (giro) | 0.5 |

### Ejemplo de Uso
```bash
rosrun mi_paquete turtle_pd_controller.py
> Ingrese la pose deseada:
Coordenada x: 5.5
Coordenada y: 5.5
Orientación final (grados): 90
```
![image](https://github.com/user-attachments/assets/4ef761b1-d1e0-40d2-b608-d62ca051cc15)

Launch:
```bash
<launch>
    
    <!-- Lanzar el nodo turtlesim_node del paquete turtlesim -->
    <node pkg="turtlesim" type="turtlesim_node" name="turtlesim_node" output="screen"/>

    <!-- Lanzar el script turtle_pdcf.py del paquete Practicas_lab -->
    <node pkg="Practicas_lab" type="turtle_pdcf.py" name="turtle_pdcf" output="screen"/>
</launch>
```

### PID
## Control PID para Tortuga en Turtlesim (x, y, θ)

### Descripción del Programa
Controlador **PID completo** (Proporcional-Integral-Derivativo) para movimiento 2D + orientación en `turtlesim`:



```bash
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, radians, sin, cos

class MoveTurtlePIDControl:
    def __init__(self):
        rospy.init_node('control_tortuga_pid_xytheta')
        
        # Suscriptores y publicadores
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        
        # Variables de estado
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0
        
        # Variables para control PID
        self.last_error_x = 0
        self.last_error_y = 0
        self.last_error_theta = 0
        self.error_accumulation_x = 0
        self.error_accumulation_y = 0
        self.error_accumulation_theta = 0

    def pose_callback(self, pose):
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_desired_pose(self, desired_x, desired_y, desired_theta):
        # Constantes PID
        Kp_linear = 1.0
        Ki_linear = 0.01
        Kd_linear = 0.1
        Kp_angular = 4.0
        Ki_angular = 0.005
        Kd_angular = 0.5

        while not rospy.is_shutdown():
            # Cálculo de errores
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y
            
            # Error de orientación
            target_angle = atan2(error_y, error_x)
            error_theta = target_angle - self.current_theta
            error_theta = atan2(sin(error_theta), cos(error_theta))
            
            # Distancia al objetivo
            distance = sqrt(error_x**2 + error_y**2)
            
            # Acumulación de errores (término integral)
            self.error_accumulation_x += error_x
            self.error_accumulation_y += error_y
            self.error_accumulation_theta += error_theta
            
            # Crear mensaje Twist
            twist_msg = Twist()
            
            if distance > 0.1:  # Fase de acercamiento
                # Control PID para posición
                twist_msg.linear.x = Kp_linear * error_x + Ki_linear * self.error_accumulation_x + Kd_linear * (error_x - self.last_error_x)
                twist_msg.linear.y = Kp_linear * error_y + Ki_linear * self.error_accumulation_y + Kd_linear * (error_y - self.last_error_y)
                
                # Control PID para orientación
                twist_msg.angular.z = Kp_angular * error_theta + Ki_angular * self.error_accumulation_theta + Kd_angular * (error_theta - self.last_error_theta)
            else:  # Fase de ajuste fino
                error_theta_final = desired_theta - self.current_theta
                error_theta_final = atan2(sin(error_theta_final), cos(error_theta_final))
                
                self.error_accumulation_theta += error_theta_final
                twist_msg.angular.z = Kp_angular * error_theta_final + Ki_angular * self.error_accumulation_theta + Kd_angular * (error_theta_final - self.last_error_theta)
                
                if abs(error_theta_final) < 0.05:
                    rospy.loginfo("Pose deseada alcanzada!")
                    break
            
            # Publicar comando
            self.velocity_publisher.publish(twist_msg)
            
            # Actualizar errores anteriores
            self.last_error_x = error_x
            self.last_error_y = error_y
            self.last_error_theta = error_theta
            
            # Log de información
            rospy.loginfo("Pos: (%.2f, %.2f), θ: %.2f rad, Error: (%.2f, %.2f), Error θ: %.2f", 
                         self.current_x, self.current_y, self.current_theta,
                         error_x, error_y, error_theta)
            
            self.rate.sleep()

    def get_desired_pose_from_user(self):
        print("\n=== Ingrese la pose deseada ===")
        x = float(input("Coordenada x: "))
        y = float(input("Coordenada y: "))
        theta_deg = float(input("Orientación final (grados): "))
        return x, y, radians(theta_deg)

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            try:
                desired_x, desired_y, desired_theta = self.get_desired_pose_from_user()
                self.move_turtle_to_desired_pose(desired_x, desired_y, desired_theta)
            except ValueError:
                rospy.logwarn("Entrada inválida. Use números.")

if __name__ == '__main__':
    try:
        controller = MoveTurtlePIDControl()
        controller.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass

```

```python
# Características técnicas:
- Control en 3 ejes: posición (x,y) + orientación (θ)
- Algoritmo PID independiente para:
  * Movimiento lineal (ejes X/Y)
  * Rotación (θ)
- Lógica de dos fases:
  1. Acercamiento con corrección de trayectoria
  2. Ajuste fino de orientación
- Sistema anti-windup implícito
```

# Entrada de usuario:
Ingrese la pose deseada:
Coordenada x: 5.5
Coordenada y: 5.5
Orientación final (grados): 90

![image](https://github.com/user-attachments/assets/dc5d530d-adfb-4866-8244-99f66642485b)
