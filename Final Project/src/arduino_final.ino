Código general:
 #define ROS_SERIAL_PROTOCOL_2
#define USE_OLD_SERIAL_PROTOCOL
#pragma message("Protocolo 2.0 activado")

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

// Configuración de pines
const int ENA = 5;    // PWM Motor Izquierdo
const int IN1 = 3;    // Control 1A
const int IN2 = 4;    // Control 2A
const int ENB = 6;    // PWM Motor Derecho
const int IN3 = 7;    // Control 3B
const int IN4 = 8;    // Control 4B

// Sensores
const int trigPin = 13;
const int echoPin = 12;
#define SENSOR_LINEA_IZQ A2
#define SENSOR_LINEA_DER A0
#define SENSOR_LINEA_CENT A1

// Constantes
const int VELOCIDAD_NORMAL = 50;
const int VELOCIDAD_GIRO = 70;
const float KP = 0.2;
const int UMBRAL_NEGRO = 900;
const float DISTANCIA_SEGURA = 30.0;
unsigned long tiempoGiro = 0;
const unsigned long TIEMPO_GIRO = 2000;

// Variables globales
ros::NodeHandle nh;
std_msgs::Float32 distancia_msg;
std_msgs::String estado_msg;

// Declaración adelantada de funciones
void comandoCallback(const std_msgs::String& cmd);
void avanzar(int velocidad = -1);
void retroceder(int velocidad = -1);
void girarDerecha();
void girarIzquierda();
void detener();
float medirDistancia();
void seguirLineaMejorado();
void giroBruscoIzquierda();  // Declaración de la nueva función

// Publicadores y suscriptores
ros::Publisher pub_distancia("/distancia", &distancia_msg);
ros::Publisher pub_estado("/arduino_estado", &estado_msg);
ros::Subscriber<std_msgs::String> sub_comando("/nav_commands", &comandoCallback);

// Estados
enum Modo {DETENIDO, SIGUIENDO_LINEA, GIRANDO, BUSCANDO_SILLA};
Modo modoActual = DETENIDO;
int pasilloObjetivo = 0;
int sillaObjetivo = 0;
bool modo_navegacion = false; // Asegurarse de declarar esta variable como bool

void setup() {
  // Inicializar pines
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);
  pinMode(SENSOR_LINEA_IZQ, INPUT);
  pinMode(SENSOR_LINEA_DER, INPUT);
  pinMode(SENSOR_LINEA_CENT, INPUT);

  // Inicializar ROS
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(pub_distancia);
  nh.advertise(pub_estado);
  nh.subscribe(sub_comando);

  // Esperar conexión
  while(!nh.connected()) {
    nh.spinOnce();
    delay(100);
  }
  estado_msg.data = "inicializado";
  pub_estado.publish(&estado_msg);
}

void loop() {
  // Revisa si hay comandos desde ROS
  nh.spinOnce();  // Necesario para procesar los mensajes ROS

  // Si está en modo de navegación, seguir la línea
  if (modo_navegacion) {
    seguirLineaMejorado();
  }
}

// Reemplaza la función seguirLineaMejorado() con este código:
void seguirLineaMejorado() {
  // 1) Leer sensores (usando los pines definidos en el código general)
  int rawL = analogRead(SENSOR_LINEA_IZQ);
  int rawC = analogRead(SENSOR_LINEA_CENT);
  int rawR = analogRead(SENSOR_LINEA_DER);

  // Variables para el seguidor adaptativo (añadir al inicio del código)
  static float whiteBase[3] = {0,0,0};
  static float blackBase[3] = {1023,1023,1023};
  static float threshold[3];
  static bool invertDetect[3];
  const float KP = 2.5; // Ajusta según necesidad
  const int BASE_SPEED = 120; // Velocidad base (puedes usar VELOCIDAD_NORMAL si prefieres)

  // 2) Actualizar adaptativo blanco/negro y umbral
  for (int i = 0; i < 3; i++) {
    int raw = (i==0? rawL : (i==1? rawC : rawR));
    if (raw > whiteBase[i]) whiteBase[i] = whiteBase[i]*0.99 + raw*0.01;
    if (raw < blackBase[i]) blackBase[i] = blackBase[i]*0.99 + raw*0.01;
    threshold[i] = (whiteBase[i] + blackBase[i]) * 0.5;
    invertDetect[i] = (whiteBase[i] > blackBase[i]);
  }

  // 3) Normalizar lecturas
  float fL = normForce(rawL, 0, whiteBase, blackBase, threshold, invertDetect);
  float fC = normForce(rawC, 1, whiteBase, blackBase, threshold, invertDetect);
  float fR = normForce(rawR, 2, whiteBase, blackBase, threshold, invertDetect);

  // 4) Error proporcional y atenuado por fC
  float error = (fR - fL) * (1.0 - fC);

  // 5) Velocidades resultantes
  int vL = constrain(int(BASE_SPEED - KP * error), 0, 255);
  int vR = constrain(int(BASE_SPEED + KP * error), 0, 255);

  // 6) Si la cinta desaparece completamente, parar
  if (fL < 0.05 && fC < 0.05 && fR < 0.05) {
    vL = vR = 0;
    // Opcional: notificar que se perdió la línea
    estado_msg.data = "linea_perdida";
    pub_estado.publish(&estado_msg);
  }

  // 7) Aplicar velocidades (usando el sistema de control del código general)
  analogWrite(ENA, vL);
  analogWrite(ENB, vR);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);

  // Debug opcional (puedes descomentar si necesitas)
  
  Serial.print("fL:"); Serial.print(fL, 2);
  Serial.print(" fC:"); Serial.print(fC, 2);
  Serial.print(" fR:"); Serial.print(fR, 2);
  Serial.print(" err:"); Serial.print(error, 2);
  Serial.print(" vL:"); Serial.print(vL);
  Serial.print(" vR:"); Serial.println(vR);
  
}

// Añade esta función auxiliar en cualquier parte del código (antes de usarla):
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
    modo_navegacion = true;  // Asegúrate de que la navegación esté habilitada
  }
  else if (comando == "girar_qr") {
    if (medirDistancia() < 15.0) {
      giroBruscoIzquierda();
    }
  }
  else if (comando == "buscar_silla") {
    modoActual = BUSCANDO_SILLA;
    modo_navegacion = true;  // Habilitar la navegación después del giro
  }
  else if (comando == "detener") {
    modoActual = DETENIDO;
    modo_navegacion = false;  // Detener la navegación
    detener();
  }
}

void giroBruscoIzquierda() {
  // Realiza el giro brusco a la izquierda
  analogWrite(ENA, VELOCIDAD_GIRO);
  analogWrite(ENB, VELOCIDAD_GIRO);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(500);  // Ajusta este tiempo si es necesario para un giro más brusco
  detener();   // Detenerse después del giro brusco
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
