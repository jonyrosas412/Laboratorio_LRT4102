#!/usr/bin/env python3

import rospy
import math
from turtlesim.srv import TeleportAbsolute, SetPen
from turtlesim.msg import Pose

# Variable global para guardar la pose actual
pose_actual = None

# Callback para actualizar la posición actual
def pose_callback(msg):
    global pose_actual
    pose_actual = msg

def get_valid_coordinate(prompt):
    while True:
        try:
            value = float(input(prompt))
            if 0 <= value <= 11:
                return value
            else:
                print("Debe estar entre 0 y 11.")
        except ValueError:
            print("Entrada inválida. Intenta de nuevo.")

def main():
    global pose_actual
    rospy.init_node('turtle_teleport')

    # Suscripción a la pose de la tortuga
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    rospy.wait_for_service('/turtle1/set_pen')
    rospy.wait_for_service('/turtle1/teleport_absolute')

    set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
    teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)

    # Esperar a que se reciba la pose inicial
    while pose_actual is None and not rospy.is_shutdown():
        rospy.sleep(0.1)

    while not rospy.is_shutdown():
        # Solicitar coordenadas objetivo y orientación
        x = get_valid_coordinate("Ingresa la coordenada X de destino (0 - 11): ")
        y = get_valid_coordinate("Ingresa la coordenada Y de destino (0 - 11): ")

        while True:
            try:
                angle_deg = float(input("Ingresa el ángulo deseado en grados (0 - 360): "))
                if 0 <= angle_deg <= 360:
                    break
                else:
                    print("El ángulo debe estar entre 0 y 360 grados.")
            except ValueError:
                print("Entrada inválida. Intenta de nuevo.")

        # Calcular DTG (diferencia en coordenadas)
        dx = x - pose_actual.x
        dy = y - pose_actual.y
        dtg = (dx, dy)  # DTG ahora es una tupla con las diferencias en x e y

        # Calcular ATG con atan2 (correcto en todos los cuadrantes)
        atg_rad = math.atan2(dy, dx)
        atg_deg = math.degrees(atg_rad)

        print(f"\n Posición actual: X={pose_actual.x:.2f}, Y={pose_actual.y:.2f}")
        print(f" Posición objetivo: X={x:.2f}, Y={y:.2f}\n")
        print(f" DTG (diferencia en coordenadas): ({dtg[0]:.2f}, {dtg[1]:.2f})")
        print(f" DTG (distancia euclidiana): {math.sqrt(dx**2 + dy**2):.2f} \n")
        print(f" Angle to Goal (ATG): {atg_deg:.2f}°")
        

        angle_rad = math.radians(angle_deg)

        try:
            set_pen(0, 0, 0, 0, 1)  # Desactivar pluma
            teleport(x, y, angle_rad)  # Teletransportar tortuga
            print(f" Tortuga movida a X={x:.2f}, Y={y:.2f}, con orientación {angle_deg}°.")
        except rospy.ServiceException as e:
            print(f"Error en servicio: {e}")
            break

        # Preguntar si desea repetir
        repetir = input("\n¿Quieres mover la tortuga otra vez? (s/n): ").strip().lower()
        if repetir != 's':
            print("¡Hasta luego!")
            break

if __name__ == "__main__":
    main()
