# Introducción a Python y Programación Orientada a Objetos (POO)

Este repositorio contiene ejemplos y explicaciones sobre los conceptos básicos de Python y el paradigma de Programación Orientada a Objetos (POO).

---

## Introducción a Python

Python es un lenguaje de programación interpretado, de alto nivel y multiparadigma. Es conocido por su sintaxis clara y legible, lo que lo hace ideal para principiantes y profesionales.

### Tipos de variables
En Python, las variables no necesitan ser declaradas con un tipo específico. El tipo de una variable se infiere automáticamente en tiempo de ejecución. Algunos tipos de datos comunes son:

**Enteros (`int`)**: Números sin decimales. Ejemplo: `x = 10`.

**Flotantes (`float`)**: Números con decimales. Ejemplo: `y = 3.14`.

**Cadenas (`str`)**: Texto. Ejemplo: `nombre = "Python"`.

**Booleanos (`bool`)**: Valores `True` o `False`. Ejemplo: `es_valido = True`.

**Listas (`list`)**: Colecciones ordenadas y mutables. Ejemplo: `numeros = [1, 2, 3]`.

**Tuplas (`tuple`)**: Colecciones ordenadas e inmutables. Ejemplo: `coordenadas = (4, 5)`.

**Diccionarios (`dict`)**: Colecciones de pares clave-valor. Ejemplo: `persona = {"nombre": "Juan", "edad": 25}`.

### Estructuras de control

#### Bucle `for`
El bucle `for` se utiliza para iterar sobre una secuencia (como una lista, tupla o cadena).

```python
for i in range(5):  # Itera desde 0 hasta 4
    print(i)
```
#### Bucle `while`
```python
contador = 0
while contador < 5:
    print(contador)
    contador += 1
```
#### Condicional `if`
```python
if edad >= 18:
    print("Eres mayor de edad")
else:
    print("Eres menor de edad")
```
## Programación Orientada a Objetos (POO) en Python
La Programación Orientada a Objetos (POO) es un paradigma de programación que organiza el código en "objetos", que son instancias de "clases". Estos objetos contienen datos (atributos) y comportamientos (métodos).

### Componentes de la POO
Clase: Es una plantilla o modelo para crear objetos.

Objeto: Es una instancia de una clase.

Atributos: Son variables que pertenecen a un objeto o clase.

Métodos: Son funciones que pertenecen a un objeto o clase.

Herencia: Permite crear una nueva clase a partir de una clase existente.

Encapsulamiento: Ocultar los detalles internos de una clase.

Polimorfismo: Capacidad de que diferentes clases puedan tener métodos con el mismo nombre pero comportamientos distintos.

#### Ejemplo de POO en Python
```python
class Animal:
    def __init__(self, nombre):
        self.nombre = nombre

    def hacer_sonido(self):
        print("Este animal hace un sonido.")

class Perro(Animal):
    def hacer_sonido(self):
        print("¡Guau guau!")

mi_perro = Perro("Rex")
mi_perro.hacer_sonido()  # Salida: ¡Guau guau!
```
## Ejercicios
### Ejercicio 1
Escribir un programa que lea un entero positivo “n” introducido por el usuario y después muestre
en pantalla la suma de todos los enteros desde 1 hasta n . La suma de los primeros enteros
positivos puede ser calculada de la siguiente forma:
![image](https://github.com/user-attachments/assets/6d22c819-6522-41d6-8c9d-be3fd2bd9e8a)
```python
import os      # Manejo de archivos y directorios
import sys     # Acceso a parámetros del sistema
import logging # Registro de eventos y errores
import json    # Manejo de datos en formato JSON
import time    # Control del tiempo y medición de ejecución

#Ingresar un número entero
n=int(input("Introduce un numero positivo: "))

#Hay que verificar que el numero entero sea positivo
if n<= 0:
    print("Introduce un numero entero positivo.")
else:
    suma=(n*(n+1))//2
#Mostramos el resultado de la suma
print(f"El resultado de la suma de los enteros desde 1 hasta {n} es: {suma}")

```
Se usa input() para leer el valor de n y int() para convertirlo a un número entero. Se verifica que n sea un número positivo. Si no lo es, se muestra un mensaje de error.Se aplica la fórmula para calcular la suma. Usamos // para asegurarnos de que el resultado sea un entero. Se imprime el resultado usando print().

### Ejercicio 2
Escribir un programa que pregunte al usuario por el número de horas trabajadas y el costo por hora.
Después debe mostrar por pantalla la paga que le corresponde.
```python
import os      # Manejo de archivos y directorios
import sys     # Acceso a parámetros del sistema
import logging # Registro de eventos y errores
import json    # Manejo de datos en formato JSON
import time    # Control del tiempo y medición de ejecución

# Solicitar al usuario el número de horas trabajadas
horas_trabajadas = float(input("Introduce el número de horas trabajadas: "))

# Solicitar al usuario el costo por hora
costo_por_hora = float(input("Introduce el costo por hora: "))

# Calcular la paga correspondiente
paga = horas_trabajadas * costo_por_hora

# Mostrar la paga por pantalla
print(f"La paga correspondiente es: {paga}")    
```
Se usa input() para leer los valores introducidos por el usuario. float() convierte la entrada en un número decimal, ya que las horas y el costo pueden tener valores fraccionarios. La paga se calcula multiplicando las horas trabajadas (horas_trabajadas) por el costo por hora (costo_por_hora). Se usa print() para mostrar la paga calculada. Se utiliza una f-string (f"...") para formatear el mensaje de salida.

### Ejericio 3
Crea una lista de nombre + sueldo por hora + horas trabajadas de al menos seis operadores.
Imprime el nombre y el sueldo a pagar de cada operador.
```python
import os      # Manejo de archivos y directorios
import sys     # Acceso a parámetros del sistema
import logging # Registro de eventos y errores
import json    # Manejo de datos en formato JSON
import time    # Control del tiempo y medición de ejecución

operadores = [
    {"nombre": "Juan", "sueldo_por_hora": 15.0, "horas_trabajadas": 40},
    {"nombre": "María", "sueldo_por_hora": 20.0, "horas_trabajadas": 35},
    {"nombre": "Pedro", "sueldo_por_hora": 18.5, "horas_trabajadas": 45},
    {"nombre": "Ana", "sueldo_por_hora": 22.0, "horas_trabajadas": 38},
    {"nombre": "Luis", "sueldo_por_hora": 16.0, "horas_trabajadas": 42},
    {"nombre": "Sofía", "sueldo_por_hora": 19.0, "horas_trabajadas": 37},
]

# Calcular y mostrar el sueldo a pagar para cada operador
for operador in operadores:
    nombre = operador["nombre"]
    sueldo_por_hora = operador["sueldo_por_hora"]
    horas_trabajadas = operador["horas_trabajadas"]
    sueldo_a_pagar = sueldo_por_hora * horas_trabajadas
    print(f"{nombre}: ${sueldo_a_pagar:.2f}")
```
Cada operador es un diccionario que contiene tres claves: nombre, sueldo_por_hora y horas_trabajadas. La lista operadores contiene seis diccionarios, cada uno representando a un operador. Se recorre la lista de operadores con un bucle for. Para cada operador, se obtienen sus datos (nombre, sueldo por hora y horas trabajadas). El sueldo a pagar se calcula multiplicando el sueldo por hora por las horas trabajadas. Se usa una f-string (f"...") para formatear la salida. El sueldo a pagar se muestra con dos decimales (:.2f) para simular un formato de moneda.

### Ejercicio 4 
Crea una lista llamada numeros que contenga al menos 10 números.
Calcula el promedio de los números pares y el producto de los números impares.
Imprime los resultados.

```python
import os      # Manejo de archivos y directorios
import sys     # Acceso a parámetros del sistema
import logging # Registro de eventos y errores
import json    # Manejo de datos en formato JSON
import time    # Control del tiempo y medición de ejecución

# Crear una lista con al menos 10 números
numeros = [2, 5, 8, 10, 3, 7, 12, 15, 6, 9]

# Inicializar variables para el cálculo
suma_pares = 0
contador_pares = 0
producto_impares = 1

# Recorrer la lista de números
for numero in numeros:
    if numero % 2 == 0:  # Verificar si el número es par
        suma_pares += numero
        contador_pares += 1
    else:  # Si el número es impar
        producto_impares *= numero

# Calcular el promedio de los números pares
if contador_pares > 0:
    promedio_pares = suma_pares / contador_pares
else:
    promedio_pares = 0  # Si no hay números pares

# Imprimir los resultados
print(f"Promedio de los números pares: {promedio_pares}")
print(f"Producto de los números impares: {producto_impares}")
```

La lista numeros contiene 10 números, algunos pares y otros impares. suma_pares: Almacena la suma de los números pares. Contador_pares: Cuenta cuántos números pares hay. producto_impares: Almacena el producto de los números impares. Se inicializa en 1 porque multiplicar por 0 daría siempre 0. Se usa un bucle for para recorrer cada número en la lista. Si el número es par (numero % 2 == 0), se suma a suma_pares y se incrementa contador_pares. Si el número es impar, se multiplica por producto_impares. El promedio de los números pares se calcula dividiendo suma_pares entre contador_pares. Si no hay números pares (contador_pares == 0), el promedio se establece en 0 para evitar divisiones por cero. Se usa print() para mostrar el promedio de los números pares y el producto de los números impares.

### Ejercicio 5 
Crea un programa que solicite al usuario adivinar un número secreto. El programa debe generar
un número aleatorio entre 1 y 10, y el usuario debe intentar adivinarlo. El programa debe
proporcionar pistas si el número ingresado por el usuario es demasiado alto o bajo. El bucle while
debe continuar hasta que el usuario adivine correctamente. Al final se debe imprimir en cuantos
intentos el usuario logró adivinar el número.
Pista:
import random
Generar un número aleatorio entre 1 y 10
numero_secreto = random.randint(1, 10)

```python
import os      # Manejo de archivos y directorios
import sys     # Acceso a parámetros del sistema
import logging # Registro de eventos y errores
import json    # Manejo de datos en formato JSON
import time    # Control del tiempo y medición de ejecución
import random 

# Generar un número aleatorio entre 1 y 10
numero_secreto = random.randint(1, 10)

# Inicializar el contador de intentos
intentos = 0

# Bucle para adivinar el número
while True:
    # Solicitar al usuario que ingrese un número
    intento = int(input("Adivina el número secreto (entre 1 y 10): "))
    intentos += 1  # Incrementar el contador de intentos

    # Comparar el número ingresado con el número secreto
    if intento < numero_secreto:
        print("El número es demasiado bajo. ¡Intenta de nuevo!")
    elif intento > numero_secreto:
        print("El número es demasiado alto. ¡Intenta de nuevo!")
    else:
        print(f"¡Felicidades! Adivinaste el número secreto {numero_secreto} en {intentos} intentos.")
        break  # Salir del bucle
```
Se usa random.randint(1, 10) para generar un número aleatorio entre 1 y 10. La variable intentos se inicializa en 0 y se incrementa en cada iteración del bucle. El bucle continúa indefinidamente (while True) hasta que el usuario adivine el número. En cada iteración, se solicita al usuario que ingrese un número y se compara con el número secreto. Si el número ingresado es menor que el número secreto, se muestra un mensaje indicando que el número es demasiado bajo. Si el número ingresado es mayor que el número secreto, se muestra un mensaje indicando que el número es demasiado alto. Si el número ingresado es igual al número secreto, se muestra un mensaje de felicitaciones y se imprime el número de intentos. El bucle se detiene con break.

### Ejercicio 6
Robot explorador
El programa debe generar una matriz de al menos 5x5.
El robot inicia su camino en la posición (0,0) de la matriz y debe salir en la posición (4,4) o la
máxima posición si se cambia el tamaño de matriz.
El numero y la posición de los obstáculos es aleatoria.
El robot solo puede avanzar, girar a la izquierda o a la derecha para buscar un camino libre, en el
eventual caso que el robot no pueda salir debe imprimir en pantalla “Imposible llegar al destino”
En caso de que el robot llegue a su destino final deberá imprimir el mapa, con los espacios libres y
obstáculos de la siguiente forma X obstáculo o libre
o o o X o
o o o o o
o o o o X
o o o o o
o X X X o
Deberá imprimir también la ruta que siguió.
Mostrar un segundo mapa con el “camino” seguido por el robot mediante flechas
Pista:
Flecha hacia arriba: ↑ (U+2191)
Flecha hacia abajo: ↓ (U+2193)
Flecha hacia la izquierda: ← (U+2190)
Flecha hacia la derecha: → (U+2192)
```python
import os      # Manejo de archivos y directorios
import sys     # Acceso a parámetros del sistema
import logging # Registro de eventos y errores
import json    # Manejo de datos en formato JSON
import time    # Control del tiempo y medición de ejecución
import random

# Tamaño de la matriz (5x5)
FILAS = 5
COLUMNAS = 5

# Símbolos para representar el mapa
LIBRE = "o"
OBSTACULO = "X"
INICIO = "S"
DESTINO = "D"
FLECHAS = {"arriba": "↑", "abajo": "↓", "izquierda": "←", "derecha": "→"}

# Generar una matriz vacía
def generar_matriz(filas, columnas):
    return [[LIBRE for _ in range(columnas)] for _ in range(filas)]

# Colocar obstáculos de forma aleatoria
def colocar_obstaculos(matriz, cantidad_obstaculos):
    for _ in range(cantidad_obstaculos):
        fila = random.randint(0, FILAS - 1)
        columna = random.randint(0, COLUMNAS - 1)
        # Asegurarse de no colocar obstáculos en el inicio o el destino
        if (fila, columna) != (0, 0) and (fila, columna) != (FILAS - 1, COLUMNAS - 1):
            matriz[fila][columna] = OBSTACULO

# Mostrar el mapa
def mostrar_mapa(matriz):
    for fila in matriz:
        print(" ".join(fila))
    print()

# Movimiento del robot
def mover_robot(matriz, ruta):
    fila, columna = 0, 0  # Posición inicial
    direccion = "derecha"  # Dirección inicial

    while (fila, columna) != (FILAS - 1, COLUMNAS - 1):
        # Guardar la posición actual en la ruta
        ruta.append((fila, columna))

        # Intentar avanzar en la dirección actual
        if direccion == "derecha" and columna + 1 < COLUMNAS and matriz[fila][columna + 1] == LIBRE:
            columna += 1
        elif direccion == "abajo" and fila + 1 < FILAS and matriz[fila + 1][columna] == LIBRE:
            fila += 1
        elif direccion == "izquierda" and columna - 1 >= 0 and matriz[fila][columna - 1] == LIBRE:
            columna -= 1
        elif direccion == "arriba" and fila - 1 >= 0 and matriz[fila - 1][columna] == LIBRE:
            fila -= 1
        else:
            # Girar a la derecha si no puede avanzar
            if direccion == "derecha":
                direccion = "abajo"
            elif direccion == "abajo":
                direccion = "izquierda"
            elif direccion == "izquierda":
                direccion = "arriba"
            elif direccion == "arriba":
                direccion = "derecha"

        # Si el robot está atrapado
        if (fila, columna) == ruta[-1]:
            print("Imposible llegar al destino")
            return False

    # Guardar la posición final en la ruta
    ruta.append((fila, columna))
    return True

# Mostrar la ruta del robot
def mostrar_ruta(matriz, ruta):
    # Crear una copia del mapa para mostrar la ruta
    mapa_ruta = [fila.copy() for fila in matriz]

    # Marcar la ruta con flechas
    for i in range(len(ruta) - 1):
        fila_actual, columna_actual = ruta[i]
        fila_siguiente, columna_siguiente = ruta[i + 1]

        if fila_siguiente > fila_actual:
            mapa_ruta[fila_actual][columna_actual] = FLECHAS["abajo"]
        elif fila_siguiente < fila_actual:
            mapa_ruta[fila_actual][columna_actual] = FLECHAS["arriba"]
        elif columna_siguiente > columna_actual:
            mapa_ruta[fila_actual][columna_actual] = FLECHAS["derecha"]
        elif columna_siguiente < columna_actual:
            mapa_ruta[fila_actual][columna_actual] = FLECHAS["izquierda"]

    # Mostrar el mapa con la ruta
    mostrar_mapa(mapa_ruta)

# Programa principal
def main():
    # Generar la matriz
    matriz = generar_matriz(FILAS, COLUMNAS)
    colocar_obstaculos(matriz, cantidad_obstaculos=5)  # Colocar 5 obstáculos

    # Marcar el inicio y el destino
    matriz[0][0] = INICIO
    matriz[FILAS - 1][COLUMNAS - 1] = DESTINO

    # Mostrar el mapa inicial
    print("Mapa inicial:")
    mostrar_mapa(matriz)

    # Ruta del robot
    ruta = []

    # Intentar mover el robot
    if mover_robot(matriz, ruta):
        print("El robot llegó al destino. Ruta seguida:")
        mostrar_ruta(matriz, ruta)

# Ejecutar el programa
if __name__ == "__main__":
    main()
```

Se genera una matriz de 5x5 con espacios libres (o). Se colocan obstáculos (X) de forma aleatoria, evitando el inicio (0, 0) y el destino (4, 4). El robot comienza en (0, 0) y se mueve en la dirección actual (derecha, abajo, izquierda, arriba). Si no puede avanzar, gira a la derecha. Si el robot queda atrapado, se muestra el mensaje "Imposible llegar al destino". Se guarda la ruta que sigue el robot en una lista. Se muestra el mapa con la ruta trazada usando flechas (↑, ↓, ←, →). Se imprime el mapa inicial con obstáculos. Si el robot llega al destino, se muestra el mapa con la ruta seguida. 

### Ejercicio: Gestión de Inventario de una Tienda
Una tienda quiere gestionar su inventario de productos. Para ello, debes implementar un sistema
en Python que permita:
Crear productos, cada uno con un nombre, precio y cantidad en stock.
Actualizar la cantidad en stock cuando se venden productos.
Mostrar la información de un producto con su disponibilidad.
Calcular el valor total del inventario (precio × cantidad de cada producto).

```python
class Producto:
    def __init__(self, nombre, precio, cantidad):
        self.nombre = nombre
        self.precio = precio
        self.cantidad = cantidad

    def __str__(self):
        return f"Producto: {self.nombre}, Precio: ${self.precio:.2f}, Cantidad: {self.cantidad}"

    def vender(self, cantidad_vendida):
        if cantidad_vendida <= self.cantidad:
            self.cantidad -= cantidad_vendida
            print(f"Se vendieron {cantidad_vendida} unidades de {self.nombre}.")
        else:
            print(f"No hay suficiente stock de {self.nombre}. Stock disponible: {self.cantidad}")

    def disponibilidad(self):
        return f"Disponibilidad de {self.nombre}: {self.cantidad} unidades"

class Inventario:
    def __init__(self):
        self.productos = []

    def agregar_producto(self, producto):
        self.productos.append(producto)
        print(f"Producto {producto.nombre} agregado al inventario.")

    def mostrar_informacion(self, nombre_producto):
        for producto in self.productos:
            if producto.nombre == nombre_producto:
                print(producto)
                return
        print(f"Producto {nombre_producto} no encontrado en el inventario.")

    def calcular_valor_total(self):
        valor_total = sum(producto.precio * producto.cantidad for producto in self.productos)
        return f"Valor total del inventario: ${valor_total:.2f}"

# Programa principal
def main():
    inventario = Inventario()

    # Crear productos
    producto1 = Producto("Laptop", 1200.0, 10)
    producto2 = Producto("Smartphone", 800.0, 20)
    producto3 = Producto("Tablet", 500.0, 15)

    # Agregar productos al inventario
    inventario.agregar_producto(producto1)
    inventario.agregar_producto(producto2)
    inventario.agregar_producto(producto3)

    # Mostrar información de un producto
    print("\nInformación de productos:")
    inventario.mostrar_informacion("Laptop")
    inventario.mostrar_informacion("Smartphone")
    inventario.mostrar_informacion("Tablet")

    # Vender productos
    print("\nVentas:")
    producto1.vender(3)
    producto2.vender(25)  # Intentar vender más de lo disponible
    producto3.vender(5)

    # Mostrar disponibilidad
    print("\nDisponibilidad:")
    print(producto1.disponibilidad())
    print(producto2.disponibilidad())
    print(producto3.disponibilidad())

    # Calcular valor total del inventario
    print("\nValor total del inventario:")
    print(inventario.calcular_valor_total())

# Ejecutar el programa
if __name__ == "__main__":
    main()
```
Clase Producto:
Representa un producto con atributos: nombre, precio y cantidad.

Métodos:
_ _str_ _: Devuelve una representación en cadena del producto.
vender: Reduce la cantidad en stock cuando se vende el producto.
disponibilidad: Muestra la cantidad disponible del producto.

Clase Inventario:
Gestiona una lista de productos.

Métodos:
agregar_producto: Añade un producto al inventario.
mostrar_informacion: Muestra la información de un producto específico.
calcular_valor_total: Calcula el valor total del inventario (precio × cantidad de cada producto).

Programa principal:
Crea productos y los agrega al inventario.
Muestra la información de los productos.
Realiza ventas y muestra la disponibilidad actualizada.
Calcula y muestra el valor total del inventario.
