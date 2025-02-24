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
print("El resultado de la suma de los enteros desde 1 hasta {n} es: {suma}")
![image](https://github.com/user-attachments/assets/a7d42c2f-74a4-4e39-b820-e744234c26c2)

