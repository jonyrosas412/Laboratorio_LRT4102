Código general:
 #define ROS_SERIAL_PROTOCOL_2
#define USE_OLD_SERIAL_PROTOCOL
#pragma message("Protocolo 2.0 activado")

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>


const int ENA = 5;    
const int IN1 = 3;    
const int IN2 = 4;    
const int ENB = 6;    
const int IN3 = 7;   
const int IN4 = 8;   


const int trigPin = 13;
const int echoPin = 12;
#define SENSOR_LINEA_IZQ A2
#define SENSOR_LINEA_DER A0
#define SENSOR_LINEA_CENT A1


const int VELOCIDAD_NORMAL = 50;
const int VELOCIDAD_GIRO = 70;
const float KP = 0.2;
const int UMBRAL_NEGRO = 900;
const float DISTANCIA_SEGURA = 30.0;
unsigned long tiempoGiro = 0;
const unsigned long TIEMPO_GIRO = 2000;


ros::NodeHandle nh;
std_msgs::Float32 distancia_msg;
std_msgs::String estado_msg;


void comandoCallback(const std_msgs::String& cmd);
void avanzar(int velocidad = -1);
void retroceder(int velocidad = -1);
void girarDerecha();
void girarIzquierda();
void detener();
float medirDistancia();
void seguirLineaMejorado();
void giroBruscoIzquierda();  


ros::Publisher pub_distancia("/distancia", &distancia_msg);
ros::Publisher pub_estado("/arduino_estado", &estado_msg);
ros::Subscriber<std_msgs::String> sub_comando("/nav_commands", &comandoCallback);


enum Modo {DETENIDO, SIGUIENDO_LINEA, GIRANDO, BUSCANDO_SILLA};
Modo modoActual = DETENIDO;
int pasilloObjetivo = 0;
int sillaObjetivo = 0;
bool modo_navegacion = false; 

void setup() {
 
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);
  pinMode(SENSOR_LINEA_IZQ, INPUT);
  pinMode(SENSOR_LINEA_DER, INPUT);
  pinMode(SENSOR_LINEA_CENT, INPUT);

  
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(pub_distancia);
  nh.advertise(pub_estado);
  nh.subscribe(sub_comando);

  
  while(!nh.connected()) {
    nh.spinOnce();
    delay(100);
  }
  estado_msg.data = "inicializado";
  pub_estado.publish(&estado_msg);
}

void loop() {

  nh.spinOnce();  

 
  if (modo_navegacion) {
    seguirLineaMejorado();
  }
}


void seguirLineaMejorado() {
 
  int rawL = analogRead(SENSOR_LINEA_IZQ);
  int rawC = analogRead(SENSOR_LINEA_CENT);
  int rawR = analogRead(SENSOR_LINEA_DER);

  
  static float whiteBase[3] = {0,0,0};
  static float blackBase[3] = {1023,1023,1023};
  static float threshold[3];
  static bool invertDetect[3];
  const float KP = 2.5; 
  const int BASE_SPEED = 120; 

 
  for (int i = 0; i < 3; i++) {
    int raw = (i==0? rawL : (i==1? rawC : rawR));
    if (raw > whiteBase[i]) whiteBase[i] = whiteBase[i]*0.99 + raw*0.01;
    if (raw < blackBase[i]) blackBase[i] = blackBase[i]*0.99 + raw*0.01;
    threshold[i] = (whiteBase[i] + blackBase[i]) * 0.5;
    invertDetect[i] = (whiteBase[i] > blackBase[i]);
  }

  
  float fL = normForce(rawL, 0, whiteBase, blackBase, threshold, invertDetect);
  float fC = normForce(rawC, 1, whiteBase, blackBase, threshold, invertDetect);
  float fR = normForce(rawR, 2, whiteBase, blackBase, threshold, invertDetect);

  
  float error = (fR - fL) * (1.0 - fC);

  
  int vL = constrain(int(BASE_SPEED - KP * error), 0, 255);
  int vR = constrain(int(BASE_SPEED + KP * error), 0, 255);

  
  if (fL < 0.05 && fC < 0.05 && fR < 0.05) {
    vL = vR = 0;
    // Opcional: notificar que se perdió la línea
    estado_msg.data = "linea_perdida";
    pub_estado.publish(&estado_msg);
  }


  analogWrite(ENA, vL);
  analogWrite(ENB, vR);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);


  
  Serial.print("fL:"); Serial.print(fL, 2);
  Serial.print(" fC:"); Serial.print(fC, 2);
  Serial.print(" fR:"); Serial.print(fR, 2);
  Serial.print(" err:"); Serial.print(error, 2);
  Serial.print(" vL:"); Serial.print(vL);
  Serial.print(" vR:"); Serial.println(vR);
  
}


float normForce(int raw, int idx, float whiteBase[], float blackBase[], float threshold[], bool invertDetect[]) {
  // Normaliza lectura a [0,1] según umbral y rangos adaptativos
  float wb = whiteBase[idx], bb = blackBase[idx], th = threshold[idx];
  float v = raw;
  if (invertDetect[idx]) {
    return constrain((v - th) / (wb - th), 0.0, 1.0);
  } else {
    return constrain((th - v) / (th - bb), 0.0, 1.0);
  }
}

void avanzar(int velocidad) {
  if(velocidad == -1) velocidad = VELOCIDAD_NORMAL;
  analogWrite(ENA, velocidad);
  analogWrite(ENB, velocidad);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
}

void retroceder(int velocidad) {
  if(velocidad == -1) velocidad = VELOCIDAD_NORMAL;
  analogWrite(ENA, velocidad);
  analogWrite(ENB, velocidad);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void girarDerecha() {
  analogWrite(ENA, VELOCIDAD_GIRO);
  analogWrite(ENB, VELOCIDAD_GIRO);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

void girarIzquierda() {
  analogWrite(ENA, VELOCIDAD_GIRO);
  analogWrite(ENB, VELOCIDAD_GIRO);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void detener() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

void comandoCallback(const std_msgs::String& cmd) {
  String comando = cmd.data;
  if (comando.startsWith("iniciar:")) {
    int sep1 = comando.indexOf(':');
    int sep2 = comando.lastIndexOf(':');
    pasilloObjetivo = comando.substring(sep1 + 1, sep2).toInt();
    sillaObjetivo = comando.substring(sep2 + 1).toInt();
    modoActual = SIGUIENDO_LINEA;
    modo_navegacion = true; 
  }
  else if (comando == "girar_qr") {
    if (medirDistancia() < 15.0) {
      giroBruscoIzquierda();
    }
  }
  else if (comando == "buscar_silla") {
    modoActual = BUSCANDO_SILLA;
    modo_navegacion = true;  
  }
  else if (comando == "detener") {
    modoActual = DETENIDO;
    modo_navegacion = false;  
    detener();
  }
}

void giroBruscoIzquierda() {
 
  analogWrite(ENA, VELOCIDAD_GIRO);
  analogWrite(ENB, VELOCIDAD_GIRO);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(500);  
  detener();  
}

float medirDistancia() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duracion = pulseIn(echoPin, HIGH, 30000);
  return (duracion == 0) ? 999.9 : duracion * 0.034 / 2;
}
