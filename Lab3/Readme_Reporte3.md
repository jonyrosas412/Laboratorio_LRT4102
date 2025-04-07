# Euclidian Distance

## Objetivo
El objetivo de este ejercicio es calcular y mostrar la **DTG** (Diferencia en las Coordenadas) y **ATG** (Ángulo hacia el Objetivo) del robot, sin moverlo, realizando un "spawn" en la posición objetivo con las coordenadas proporcionadas.

---

## Instrucciones

### 1. Calcular y mostrar en pantalla la **DTG** y **ATG**

- **DTG** se refiere a la diferencia entre las coordenadas actuales del robot y las coordenadas objetivo.
- **ATG** es el ángulo que debe girar el robot para alcanzar el objetivo, calculado con la función `atan2`.

### 2. No mover el robot, hacer **spawn** del mismo en la posición **Goal**

El código proporcionado realiza lo siguiente:
- **Suscripción** a la posición actual del robot.
- **Cálculo** de la **DTG** (diferencia entre las coordenadas actuales y las deseadas).
- **Cálculo** del **ATG** (ángulo hacia el objetivo utilizando `atan2`).
- **Teleportación** del robot a las coordenadas especificadas sin moverlo físicamente en el espacio, utilizando el servicio de **teleport_absolute**.

---

## Explicación del Código

### Variables
- **pose_actual**: Variable global que guarda la pose actual del robot.
- **x, y**: Coordenadas objetivo proporcionadas por el usuario.
- **angle_deg**: Ángulo en grados hacia el objetivo proporcionado por el usuario.

### Funciones

- **pose_callback(msg)**: Callback que actualiza la pose actual del robot.
- **get_valid_coordinate(prompt)**: Función para solicitar y validar las coordenadas X y Y dentro del rango [0, 11].
  
### Cálculos

1. **DTG**: 
    La diferencia entre las coordenadas actuales y las deseadas:
    ```python
    dx = x - pose_actual.x
    dy = y - pose_actual.y
    dtg = (dx, dy)
    ```

2. **ATG**:
    Utiliza la función `atan2` para calcular el ángulo que debe girar el robot hacia el objetivo:
    ```python
    atg_rad = math.atan2(dy, dx)
    atg_deg = math.degrees(atg_rad)
    ```

3. **Teleportación**:
    El robot es teleportado a las coordenadas objetivo especificadas con la orientación proporcionada:
    ```python
    teleport(x, y, angle_rad)
    ```

---

## Ejemplo de Ejecución

1. El usuario ingresa las coordenadas objetivo y el ángulo deseado:
    - Coordenada X: `3.5`
    - Coordenada Y: `7.0`
    - Ángulo: `45.0°`

2. El programa calcula la **DTG** y **ATG**:
    - **DTG (diferencia en coordenadas)**: `(3.5, 7.0)`
    - **DTG (distancia euclidiana)**: 7.80 unidades
    - **ATG**: 63.43° hacia el objetivo

3. El robot es teleportado a las nuevas coordenadas sin moverse físicamente en el espacio.

---

## 3. Explicar el mapeo necesario para las velocidades

Para que el robot se desplace de la posición actual a la posición objetivo, necesitamos calcular dos tipos de velocidad:

- **Velocidad Lineal**: La velocidad a la que el robot se mueve hacia el objetivo.
- **Velocidad Angular**: La velocidad a la que el robot debe girar para alinearse con la dirección hacia el objetivo.

### Mapeo de las Velocidades

1. **Velocidad Lineal (Lineal Speed)**
   
   La velocidad lineal depende de la distancia entre la posición actual y la posición objetivo. Cuanto mayor sea la distancia, mayor será la velocidad lineal. Se puede calcular utilizando la fórmula de la distancia euclidiana:

   \[
   \text{distancia} = \sqrt{(dx)^2 + (dy)^2}
   \]

   Donde `dx` y `dy` son las diferencias en las coordenadas X e Y respectivamente.

   La velocidad lineal se puede definir con un factor de escala `k_linear` como:

   \[
   \text{velocidad lineal} = k_{\text{linear}} \times \text{distancia}
   \]

2. **Velocidad Angular (Angular Speed)**

   La velocidad angular depende del ángulo de diferencia entre la orientación actual del robot y la dirección hacia el objetivo. Este ángulo es el **ATG** (Ángulo hacia el objetivo), calculado previamente utilizando la función `atan2`.

   La velocidad angular se puede definir con un factor de escala `k_angular` como:

   \[
   \text{velocidad angular} = k_{\text{angular}} \times |\text{ATG}|
   \]

   Donde `ATG` es el ángulo calculado en grados entre la orientación actual y la dirección deseada.

### 4. Usar un controlador (libre) para llevar a la tortuga a la posición deseada, hacerlo en bucle infinito.
# 4.- Usar un controlador (libre) para llevar a la tortuga a la posición deseada, hacerlo en bucle infinito.

Este código implementa un controlador proporcional para mover una tortuga en el simulador de ROS, **Turtlesim**, hacia una posición deseada. A continuación, se describe su funcionamiento.

### Estructura Principal

- **Inicialización de ROS**: Se inicia un nodo de ROS llamado `turtle_proportional_controller` para manejar la comunicación con el simulador.
  
- **Suscriptores y Publicadores**:
  - **`/turtle1/pose`**: Se suscribe a la posición y orientación actual de la tortuga.
  - **`/turtle1/cmd_vel`**: Publica los comandos de velocidad (lineales y angulares) para mover la tortuga.

### Lógica del Controlador

1. **Obtención de Objetivos**:
   - Se solicita al usuario ingresar las coordenadas `X` y `Y` del objetivo, junto con el ángulo final `theta`. Las entradas deben estar dentro de un rango específico (0-11 para las coordenadas, y un valor en grados para el ángulo).

2. **Cálculo de Errores**:
   - El controlador calcula los errores lineales y angulares necesarios para mover la tortuga desde su posición actual hasta el objetivo. Se utilizan las funciones trigonométricas `atan2` y `sqrt` para calcular los ángulos y las distancias.

3. **Movimiento a la Meta**:
   - El controlador sigue tres fases principales:
     1. **Rotación hacia el objetivo**: Si la tortuga no está orientada correctamente hacia el objetivo, se ajusta su orientación.
     2. **Movimiento hacia el objetivo**: Una vez orientada, la tortuga se mueve hacia las coordenadas deseadas.
     3. **Ajuste de orientación final**: Cuando la tortuga ha alcanzado la posición, ajusta su orientación para coincidir con el ángulo deseado.

4. **Control Proporcional**:
   - El código usa un controlador proporcional (con ganancias `Kp_linear` y `Kp_angular`) para ajustar las velocidades lineales y angulares en función de los errores calculados.

5. **Bucle Infinito**:
   - El controlador sigue ejecutándose en un bucle infinito, solicitando nuevos objetivos y moviendo la tortuga hasta alcanzarlos. Si la tortuga alcanza su meta, imprime un mensaje indicando el éxito.

### Funciones y Métodos

- **`pose_callback`**: Actualiza la posición y orientación actuales de la tortuga.
- **`get_valid_input`**: Solicita al usuario las coordenadas y el ángulo del objetivo, validando las entradas.
- **`calculate_errors`**: Calcula los errores lineales y angulares en relación con el objetivo.
- **`move_to_goal`**: Controla el movimiento de la tortuga hacia el objetivo, aplicando las fases de rotación, movimiento y ajuste de orientación.
- **`print_status`**: Muestra información detallada del estado de la tortuga en cada fase.
- **`run`**: Ejecuta el controlador en un bucle infinito, solicitando y procesando objetivos hasta que el programa sea interrumpido.

### Conclusiones

El código implementado para controlar el movimiento de la tortuga en el simulador **Turtlesim** utiliza un enfoque de **control proporcional** para mover la tortuga de su posición actual a un objetivo especificado por el usuario. A través de dos scripts en ROS, se abordan diferentes aspectos del control del robot, desde la **teleportación** hasta el **cálculo de errores** y el ajuste de la velocidad lineal y angular.

El primer script, por otro lado, se centra en **teletransportar** a la tortuga a una nueva posición sin moverla físicamente en el espacio. Utilizando el servicio de ROS `teleport_absolute`, la tortuga es posicionada en las coordenadas objetivo con la orientación solicitada, sin requerir una simulación de movimiento real. Además, este script calcula la **DTG** (diferencia de coordenadas) y el **ATG** (ángulo hacia el objetivo) para proporcionar al usuario retroalimentación detallada sobre el movimiento simulado. 

El segundo script, que utiliza un **controlador proporcional**, permite que la tortuga se mueva de manera eficiente hacia el objetivo. Se calcula la **distancia euclidiana (DTG)** y el **ángulo hacia el objetivo (ATG)**, utilizando funciones matemáticas como `atan2` y `sqrt`. El controlador ajusta la **velocidad angular** y **lineal** de la tortuga según la diferencia entre su posición actual y el objetivo. La implementación de un **bucle infinito** permite que la tortuga continúe moviéndose hasta que llegue al destino y se ajuste a la orientación final deseada. Este enfoque de control proporciona una **respuesta dinámica** y ajustable, adaptándose al error calculado en cada iteración.

Ambos enfoques son útiles en diferentes escenarios: el primero para simular un movimiento real hacia un objetivo con control dinámico y el segundo para pruebas rápidas o simulaciones en las que solo se necesita determinar la posición final. Ambos métodos se complementan bien, ya que permiten estudiar y validar algoritmos de control de manera eficiente antes de su implementación en un robot físico.

En resumen, estos scripts proveen un marco flexible para controlar el movimiento de la tortuga en un entorno simulado, ya sea ajustando su velocidad para llegar a un objetivo o teletransportándola instantáneamente a una nueva posición. Ambos métodos aprovechan las capacidades de ROS para el control de robots, permitiendo una interacción sencilla y efectiva con el simulador **Turtlesim**.



