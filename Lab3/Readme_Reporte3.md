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




