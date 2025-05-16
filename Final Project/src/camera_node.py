#!/usr/bin/env python3
import rospy
import cv2
from pyzbar import pyzbar
from std_msgs.msg import String

class QRDetector:
    def __init__(self):
        rospy.init_node('camera_node')
        self.pub = rospy.Publisher('/qr_detected', String, queue_size=10)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def detect(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret: continue
            
            qrs = pyzbar.decode(frame)
            for qr in qrs:
                data = qr.data.decode('utf-8')
                if data.startswith(("qr_pasillo_", "qr_silla_")):
                    self.pub.publish(data.lower())
                    cv2.rectangle(frame, (qr.rect.left, qr.rect.top), 
                                 (qr.rect.left + qr.rect.width, qr.rect.top + qr.rect.height), 
                                 (0, 255, 0), 2)
            
            cv2.imshow("QR", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rate.sleep()

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    QRDetector().detect()
