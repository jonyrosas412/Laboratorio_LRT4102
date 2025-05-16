#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Float32

class NavigationManager:
    def __init__(self):  # 
        rospy.init_node('navigation_manager')
        self.nav_pub = rospy.Publisher('/nav_commands', String, queue_size=10)
        rospy.Subscriber('/voice_commands', String, self.handle_voice)
        rospy.Subscriber('/qr_detected', String, self.handle_qr)
        rospy.Subscriber('/distancia', Float32, self.handle_distance)

        self.pasillo_objetivo = 0
        self.silla_objetivo = 0
        self.distancia_actual = 999.9
        self.umbral_detencion = 15.0  # cm

        self.etapa = "esperando"  # etapas: esperando, buscando_pasillo, girando, buscando_silla

    def handle_voice(self, msg):
        try:
            if "detener" in msg.data.lower():
                self.nav_pub.publish("detener")
                return
                
            if "pasillo" in msg.data and "silla" in msg.data:
                partes = msg.data.split()
                p = int(partes[partes.index("pasillo")+1])
                s = int(partes[partes.index("silla")+1])
                if 1 <= p <= 3 and 1 <= s <= 5:
                    self.pasillo_objetivo = p
                    self.silla_objetivo = s
                    self.nav_pub.publish(f"iniciar:{p}:{s}")
                    self.etapa = "buscando_pasillo"
        except Exception as e:
            rospy.logerr(f"Error en voz: {str(e)}")

    def handle_qr(self, msg):
        qr = msg.data
        rospy.loginfo(f"QR Detectado: {qr}")
    
        if "qr_pasillo_" in qr:
            p = int(qr.split("_")[2])
            if p == self.pasillo_objetivo:
                self.nav_pub.publish("girar_qr") 
                self.etapa = "girando"
            
        elif "qr_silla_" in qr:
            partes = qr.split("_")
            p = int(partes[2])
            s = int(partes[3])
            if p == self.pasillo_objetivo and s == self.silla_objetivo:
                self.nav_pub.publish("detener")
                self.etapa = "completado"

    def handle_distance(self, msg):
        self.distancia_actual = msg.data
        rospy.loginfo(f"Distancia: {msg.data:.2f} cm")
        if msg.data < self.umbral_detencion:
            rospy.logwarn(f"Objeto detectado a {msg.data:.2f} cm")
            # Ya no enviamos "detener" aquí
        elif msg.data >= self.umbral_detencion:
            rospy.loginfo("Objeto fuera de rango, el Arduino reanudará automáticamente.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':  
    NavigationManager().run()
