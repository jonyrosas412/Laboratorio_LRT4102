#!/usr/bin/env python3
import rospy
import speech_recognition as sr
from std_msgs.msg import String

class VoiceCommander:
    def __init__(self):
        rospy.init_node('voice_command_node')
        self.pub = rospy.Publisher('/voice_commands', String, queue_size=10)
        
        # Configuración optimizada del reconocedor
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 3000 
        self.recognizer.pause_threshold = 0.8  
        
  
        mic_list = sr.Microphone.list_microphone_names()
        self.microphone = sr.Microphone(device_index=mic_list.index([m for m in mic_list if 'webcam' in m.lower() or 'mic' in m.lower()][0]))
        
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
            rospy.loginfo("Micrófono calibrado. Di 'pasillo [número] silla [número]'")

    def listen(self):
        rate = rospy.Rate(2)  # Mayor frecuencia de muestreo
        while not rospy.is_shutdown():
            try:
                with self.microphone as source:
                    audio = self.recognizer.listen(source, timeout=2, phrase_time_limit=3)
                
                text = self.recognizer.recognize_google(audio, language='es-ES').lower()
                text = text.replace("uno", "1").replace("dos", "2").replace("tres", "3")
                
                if "pasillo" in text and "silla" in text:
                    self.pub.publish(text)
                    rospy.loginfo(f"Comando detectado: {text}")
                    
            except sr.WaitTimeoutError:
                continue
            except Exception as e:
                rospy.logwarn(f"Error temporal: {str(e)}")
            rate.sleep()

if __name__ == '__main__':
    VoiceCommander().listen()
