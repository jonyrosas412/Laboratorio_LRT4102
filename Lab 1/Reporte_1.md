# Introducción a Python y Programación Orientada a Objetos (POO)

Este repositorio contiene ejemplos y explicaciones sobre los conceptos básicos de Python y el paradigma de Programación Orientada a Objetos (POO).

---

## Introducción a Python

Python es un lenguaje de programación interpretado, de alto nivel y multiparadigma. Es conocido por su sintaxis clara y legible, lo que lo hace ideal para principiantes y profesionales.

### Tipos de variables
En Python, las variables no necesitan ser declaradas con un tipo específico. El tipo de una variable se infiere automáticamente en tiempo de ejecución. Algunos tipos de datos comunes son:

Enteros (`int`): Números sin decimales. Ejemplo: `x = 10`.
Flotantes (`float`): Números con decimales. Ejemplo: `y = 3.14`.
Cadenas (`str`): Texto. Ejemplo: `nombre = "Python"`.
Booleanos (`bool`): Valores `True` o `False`. Ejemplo: `es_valido = True`.
Listas (`list`): Colecciones ordenadas y mutables. Ejemplo: `numeros = [1, 2, 3]`.
Tuplas (`tuple`): Colecciones ordenadas e inmutables. Ejemplo: `coordenadas = (4, 5)`.
Diccionarios (`dict`): Colecciones de pares clave-valor. Ejemplo: `persona = {"nombre": "Juan", "edad": 25}`.

### Estructuras de control

#### Bucle `for`
El bucle `for` se utiliza para iterar sobre una secuencia (como una lista, tupla o cadena).

```python
for i in range(5):  # Itera desde 0 hasta 4
    print(i)

#### Bucle `while`
contador = 0
while contador < 5:
    print(contador)
    contador += 1

#### Condicional `if`
if edad >= 18:
    print("Eres mayor de edad")
else:
    print("Eres menor de edad")

## Programación Orientada a Objetos (POO) en Python
La Programación Orientada a Objetos (POO) es un paradigma de programación que organiza el código en "objetos", que son instancias de "clases". Estos objetos contienen datos (atributos) y comportamientos (métodos).

###Componentes de la POO
Clase: Es una plantilla o modelo para crear objetos.

Objeto: Es una instancia de una clase.

Atributos: Son variables que pertenecen a un objeto o clase.

Métodos: Son funciones que pertenecen a un objeto o clase.

Herencia: Permite crear una nueva clase a partir de una clase existente.

Encapsulamiento: Ocultar los detalles internos de una clase.

Polimorfismo: Capacidad de que diferentes clases puedan tener métodos con el mismo nombre pero comportamientos distintos.
