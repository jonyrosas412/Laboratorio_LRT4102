#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, radians, sin, cos

class TurtleController:
    def __init__(self):
        rospy.init_node('turtle_proportional_controller')
        
        # Inicializar suscriptores y publicadores
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Variables de estado
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Constantes de control (ajustables)
        self.Kp_linear = 1.5   # Ganancia velocidad lineal
        self.Kp_angular = 4.0  # Ganancia velocidad angular
        self.rate = rospy.Rate(10)
        
        # Umbrales de error
        self.position_tolerance = 0.1
        self.angle_tolerance = 0.05

    def pose_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def get_valid_input(self):
        """Obtiene coordenadas válidas del usuario"""
        while True:
            try:
                x = float(input("Ingrese X objetivo (0-11): "))
                y = float(input("Ingrese Y objetivo (0-11): "))
                theta = float(input("Ángulo final deseado (grados): "))
                
                if 0 <= x <= 11 and 0 <= y <= 11:
                    return x, y, radians(theta)
                print("¡Coordenadas deben estar entre 0 y 11!")
            except ValueError:
                print("Entrada inválida. Intente nuevamente.")

    def calculate_errors(self, target_x, target_y):
        """Calcula DTG y errores angular/lineal"""
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        
        dtg = (dx, dy)
        distance = sqrt(dx**2 + dy**2)
        target_angle = atan2(dy, dx)
        angle_error = target_angle - self.current_theta
        
        # Normalizar error angular
        angle_error = atan2(sin(angle_error), cos(angle_error))
        
        return dtg, distance, angle_error, target_angle

    def move_to_goal(self, target_x, target_y, final_theta):
        """Control proporcional para alcanzar la pose deseada"""
        last_phase = 1
        
        while not rospy.is_shutdown():
            # Calcular errores
            dtg, distance, angle_error, target_angle = self.calculate_errors(target_x, target_y)
            final_angle_error = final_theta - self.current_theta
            final_angle_error = atan2(sin(final_angle_error), cos(final_angle_error))
            
            # Crear mensaje de velocidad
            cmd = Twist()
            
            # Fase 1: Rotar hacia el objetivo
            if distance > self.position_tolerance and abs(angle_error) > self.angle_tolerance:
                cmd.angular.z = self.Kp_angular * angle_error
                phase = 1
                
            # Fase 2: Movimiento hacia el objetivo
            elif distance > self.position_tolerance:
                cmd.linear.x = self.Kp_linear * distance * cos(angle_error)
                cmd.linear.y = self.Kp_linear * distance * sin(angle_error)
                phase = 2
                
            # Fase 3: Ajuste de orientación final
            else:
                cmd.angular.z = self.Kp_angular * final_angle_error
                phase = 3
                if abs(final_angle_error) < self.angle_tolerance:
                    rospy.loginfo("¡Pose objetivo alcanzada!")
                    return True
            
            # Publicar velocidades
            self.vel_pub.publish(cmd)
            
            # Mostrar información solo si cambia la fase
            if phase != last_phase:
                self.print_status(phase, dtg, distance, target_angle, final_angle_error)
                last_phase = phase
            
            self.rate.sleep()
        return False

    def print_status(self, phase, dtg, distance, target_angle, final_angle_error):
        """Muestra información del estado actual"""
        phase_names = {
            1: " ROTANDO HACIA OBJETIVO",
            2: " AVANZANDO",
            3: " AJUSTANDO ORIENTACIÓN FINAL"
        }
        
        print("\n" + "="*50)
        print(f"{phase_names[phase]}")
        print(f" Posición actual: ({self.current_x:.2f}, {self.current_y:.2f})")
        print(f" DTG (Δx, Δy): ({dtg[0]:.2f}, {dtg[1]:.2f})")
        print(f" DTG (Euclidiana): {distance:.2f}")
        print(f" ATG: {math.degrees(target_angle):.2f}°")
        print(f" Error orientación final: {math.degrees(final_angle_error):.2f}°")
        print("="*50 + "\n")

    def run(self):
        while not rospy.is_shutdown():
            try:
                print("\n=== NUEVO OBJETIVO ===")
                x, y, theta = self.get_valid_input()
                success = self.move_to_goal(x, y, theta)
                
                if success:
                    print("¡Objetivo alcanzado exitosamente!")
                else:
                    print("¡Movimiento cancelado!")
                    
            except rospy.ROSInterruptException:
                return
            except Exception as e:
                rospy.logerr(f"Error: {str(e)}")

if __name__ == '__main__':
    controller = TurtleController()
    controller.run()
