
## 1. Arduino Firmware

This module is responsible for:

- Controlling the Elegoo UNO R3 motors to move forward, turn, and stop.  
- Reading the ultrasonic sensor to measure distances and avoid obstacles.  
- Following a line (black on white or vice versa) with adaptive calibration.  
- Communicating state and distance to ROS, and receiving navigation commands from ROS.


### 1.1 Headers and Protocols

```cpp
#define ROS_SERIAL_PROTOCOL_2
#define USE_OLD_SERIAL_PROTOCOL
#pragma message("Protocol 2.0 activated")

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
````

* **ROS\_SERIAL\_PROTOCOL\_2 / USE\_OLD\_SERIAL\_PROTOCOL**
  Forces use of `rosserial` version 2.0, which improves message reliability.
* **ros.h**
  Core library for Arduino ↔ ROS communication.
* **std\_msgs/Float32.h** and **std\_msgs/String.h**
  Message types for publishing distance (`Float32`) and status updates (`String`).

---

### 1.2 Pin Configuration

```cpp
// Motors (L298N driver)
const int ENA = 5;    // PWM Left Motor
const int IN1 = 3;    // Direction Left Motor A
const int IN2 = 4;    // Direction Left Motor B
const int ENB = 6;    // PWM Right Motor
const int IN3 = 7;    // Direction Right Motor A
const int IN4 = 8;    // Direction Right Motor B

// Ultrasonic Sensor HC-SR04
const int trigPin = 13;
const int echoPin = 12;

// Line Follower Sensors (three phototransistors)
#define SENSOR_LINEA_IZQ  A2
#define SENSOR_LINEA_CENT A1
#define SENSOR_LINEA_DER  A0
```

* **ENA / ENB**
  PWM pins to set each motor’s speed.
* **IN1–IN4**
  Digital pins to set each motor’s rotation direction.
* **trigPin / echoPin**
  Trigger and echo pins for the ultrasonic sensor.
* **SENSOR\_LINEA\_**\*
  Analog inputs for reflectance readings.

---

### 1.3 Control Constants

```cpp
const int VELOCIDAD_NORMAL   = 50;    // Base PWM for forward motion
const int VELOCIDAD_GIRO     = 70;    // PWM for turns
const float KP               = 0.2;   // Proportional gain (basic)
const int UMBRAL_NEGRO       = 900;   // Raw threshold for pure black
const float DISTANCIA_SEGURA = 30.0;  // cm, safety distance in front

unsigned long tiempoGiro      = 0;
const unsigned long TIEMPO_GIRO = 2000; // ms for sharp turn
```

* **VELOCIDAD\_NORMAL/GIRO**
  Tunable based on mechanical response.
* **KP**
  Proportional gain used in line-following control.
* **UMBRAL\_NEGRO**
  Static “black” calibration threshold.
* **DISTANCIA\_SEGURA**
  Minimum frontal distance before obstacle avoidance.

---

### 1.4 Global Variables & ROS Setup

```cpp
ros::NodeHandle nh;
std_msgs::Float32 distancia_msg;
std_msgs::String  estado_msg;

// Publishers
ros::Publisher pub_distancia("/distancia", &distancia_msg);
ros::Publisher pub_estado   ("/arduino_estado", &estado_msg);

// Subscriber for navigation commands
ros::Subscriber<std_msgs::String> sub_comando("/nav_commands", &comandoCallback);

// Operation modes
enum Modo { DETENIDO, SIGUIENDO_LINEA, GIRANDO, BUSCANDO_SILLA };
Modo modoActual       = DETENIDO;
bool modo_navegacion  = false;
int pasilloObjetivo   = 0;
int sillaObjetivo     = 0;
```

* **nh**
  Manages the ROS node.
* **pub\_distancia**
  Publishes ultrasonic readings (in cm).
* **pub\_estado**
  Reports internal states (“initialized”, “line\_lost”, etc.).
* **sub\_comando**
  Receives strings (`iniciar:lane:chair`, `turn_qr`, `stop`).
* **Modo**
  Internal state machine for sequencing behaviors.

---

### 1.5 `setup()`

```cpp
void setup() {
  // Configure pin modes
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(SENSOR_LINEA_IZQ,  INPUT);
  pinMode(SENSOR_LINEA_CENT, INPUT);
  pinMode(SENSOR_LINEA_DER,  INPUT);

  // Initialize ROS at 57600 baud
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(pub_distancia);
  nh.advertise(pub_estado);
  nh.subscribe(sub_comando);

  // Wait until ROS is connected
  while (!nh.connected()) {
    nh.spinOnce();
    delay(100);
  }

  // Publish initial state
  estado_msg.data = "initialized";
  pub_estado.publish(&estado_msg);
}
```

1. **pinMode**: sets each pin direction.
2. **rosserial**: initializes connection, advertises publishers, and subscribes.
3. **Active wait**: blocks `setup()` until ROS connection is established.
4. **First message**: confirms the Arduino node is ready.

---

### 1.6 `loop()`

```cpp
void loop() {
  nh.spinOnce();           // Handle incoming ROS messages
  if (modo_navegacion) {   
    seguirLineaMejorado(); // Continuously run the line follower
  }
}
```

* **nh.spinOnce()**
  Processes incoming messages (commands).
* **modo\_navegacion**
  Only true after receiving the “iniciar” command.

---

### 1.7 Improved Line-Following

```cpp
void seguirLineaMejorado() {
  // 1) Read raw values from each sensor
  int rawL = analogRead(SENSOR_LINEA_IZQ);
  int rawC = analogRead(SENSOR_LINEA_CENT);
  int rawR = analogRead(SENSOR_LINEA_DER);

  // 2) Adaptive white/black calibration
  static float whiteBase[3] = {0,0,0};
  static float blackBase[3] = {1023,1023,1023};
  static float threshold[3];
  static bool  invertDetect[3];

  for (int i = 0; i < 3; i++) {
    int raw = (i==0? rawL : (i==1? rawC : rawR));
    whiteBase[i] = max(whiteBase[i]*0.99 + raw*0.01, whiteBase[i]);
    blackBase[i] = min(blackBase[i]*0.99 + raw*0.01, blackBase[i]);
    threshold[i] = (whiteBase[i] + blackBase[i]) * 0.5;
    invertDetect[i] = (whiteBase[i] > blackBase[i]);
  }

  // 3) Normalize with helper function
  float fL = normForce(rawL, 0, whiteBase, blackBase, threshold, invertDetect);
  float fC = normForce(rawC, 1, whiteBase, blackBase, threshold, invertDetect);
  float fR = normForce(rawR, 2, whiteBase, blackBase, threshold, invertDetect);

  // 4) Compute proportional error
  float error = (fR - fL) * (1.0 - fC);

  // 5) Calculate PWM outputs
  int vL = constrain(int(120 - KP * error), 0, 255);
  int vR = constrain(int(120 + KP * error), 0, 255);

  // 6) Detect line loss
  if (fL < 0.05 && fC < 0.05 && fR < 0.05) {
    vL = vR = 0;
    estado_msg.data = "line_lost";
    pub_estado.publish(&estado_msg);
  }

  // 7) Apply PWM and directions
  analogWrite(ENA, vL);
  analogWrite(ENB, vR);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
}
```

* **Adaptive bases**: continuously recalibrate to changing light conditions.
* **normForce**: maps raw readings to \[0…1] using thresholds.
* **Error**: lateral deviation scaled by center sensor.
* **constrain**: ensures PWM remains within valid range.
* **Line lost**: stops motors and publishes status.

---

### 1.8 Helper Function `normForce`

```cpp
float normForce(int raw, int idx,
                float whiteBase[], float blackBase[],
                float threshold[], bool invertDetect[]) {
  float wb = whiteBase[idx], bb = blackBase[idx], th = threshold[idx];
  if (invertDetect[idx]) {
    return constrain((raw - th) / (wb - th), 0.0, 1.0);
  } else {
    return constrain((th - raw) / (th - bb), 0.0, 1.0);
  }
}
```

* Chooses formula based on whether the sensor is interpreting white on black or vice versa.

---

### 1.9 Basic Movement Functions

```cpp
void avanzar(int v = -1) { /* Forward both motors */ }
void retroceder(int v = -1) { /* Reverse both motors */ }
void girarDerecha()    { /* Left forward + Right reverse */ }
void girarIzquierda()  { /* Left reverse + Right forward */ }
void detener()        { /* Stop both motors */ }
```

* Simplify commands issued by the callback handler.

---

### 1.10 ROS Command Callback

```cpp
void comandoCallback(const std_msgs::String& cmd) {
  String c = cmd.data;
  if (c.startsWith("iniciar:")) {
    // Extract lane and chair, enable navigation
  }
  else if (c == "girar_qr") {
    // If ultrasonic < 15cm, perform sharp turn
  }
  else if (c == "buscar_silla") {
    // Switch to searching for chair
  }
  else if (c == "detener") {
    // Disable navigation and stop motors
  }
}
```

* **“iniciar\:L\:C”**: begins line-following toward the lane QR.
* **“girar\_qr”**: executes on lane QR detection, using ultrasonic distance.
* **“buscar\_silla”**: after turning, continues until chair QR.
* **“detener”**: ends mission and stops movement.


```
```
## 2. Vision Node (`camera_node.py`)

This node captures video from a USB camera, detects specific QR codes, and publishes their data to ROS so other nodes can react accordingly.

---

### 2.1 Headers and Imports

```python
#!/usr/bin/env python3
import rospy
import cv2
from pyzbar import pyzbar
from std_msgs.msg import String
````

* **#!/usr/bin/env python3**
  Allows the script to run as a ROS node under Python 3.
* **rospy**
  ROS client library for Python, used to initialize nodes, publish, and subscribe.
* **cv2**
  OpenCV module for image capture and processing.
* **pyzbar**
  Library for decoding barcodes and QR codes.
* **std\_msgs.msg.String**
  ROS message type for publishing decoded QR data.

---

### 2.2 Class `QRDetector`

```python
class QRDetector:
    def __init__(self):
        rospy.init_node('camera_node')
        self.pub = rospy.Publisher('/qr_detected', String, queue_size=10)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
```

1. **rospy.init\_node('camera\_node')**
   Initializes a ROS node named `camera_node`.
2. **Publisher `/qr_detected`**
   Advertises a topic of type `String` for decoded QR strings.
3. **cv2.VideoCapture(0)**
   Opens the default camera (`/dev/video0`).
4. **cap.set(...)**
   Forces a capture resolution of 640×480 for a balance between speed and detail.

---

### 2.3 Method `detect()`

```python
def detect(self):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ret, frame = self.cap.read()
        if not ret:
            continue
```

* **rospy.Rate(10)**
  Runs the loop at 10 Hz (every 100 ms).
* **cap.read()**
  Reads a frame; if it fails, the loop continues without processing.

---

#### 2.3.1 QR Decoding

```python
        qrs = pyzbar.decode(frame)
        for qr in qrs:
            data = qr.data.decode('utf-8')
            if data.startswith(("qr_pasillo_", "qr_silla_")):
                self.pub.publish(data.lower())
                cv2.rectangle(
                    frame,
                    (qr.rect.left, qr.rect.top),
                    (qr.rect.left + qr.rect.width, qr.rect.top + qr.rect.height),
                    (0, 255, 0), 2
                )
```

1. **pyzbar.decode(frame)**
   Returns a list of decoded objects, each with `.data` (raw bytes) and `.rect` (bounding box).
2. **Filter by prefix**
   Only processes codes beginning with `qr_pasillo_` or `qr_silla_`.
3. **Publish**
   Converts the decoded data to lowercase and publishes it on `/qr_detected`.
4. **Draw rectangle**
   Visual feedback: draws a green box around each detected QR code.

---

#### 2.3.2 Display and Shutdown

```python
        cv2.imshow("QR", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        rate.sleep()

    self.cap.release()
    cv2.destroyAllWindows()
```

* **cv2.imshow("QR", frame)**
  Displays the video feed with detection overlays.
* **cv2.waitKey(1)**
  Waits 1 ms for a key press; quits the loop if ‘q’ is pressed.
* **rate.sleep()**
  Enforces the 10 Hz rate.
* **Resource cleanup**
  Releases the camera and closes all OpenCV windows when shutting down.

---

### 2.4 Execution Flow

1. **Initialization**

   * Configure camera and advertise `/qr_detected`.
2. **Main loop**

   * Capture a frame.
   * Decode all visible QR codes.
   * For each valid QR, publish its string and draw a rectangle.
   * Show the frame and listen for ‘q’ to exit.
3. **Cleanup**

   * Release hardware and destroy windows.
   * 
## 3. Voice Command Node (`voice_command_node.py`)

This node listens for spoken navigation commands via a microphone, converts them to text using Google’s Speech Recognition API, and publishes valid commands to ROS.


### 3.1 Headers and Imports

```python
#!/usr/bin/env python3
import rospy
import speech_recognition as sr
from std_msgs.msg import String
````

* **#!/usr/bin/env python3**
  Allows execution as a ROS node under Python 3.
* **rospy**
  ROS client library for Python.
* **speech\_recognition**
  Library to capture and transcribe audio.
* **std\_msgs.msg.String**
  ROS message type for publishing recognized commands.

---

### 3.2 Class `VoiceCommander`

```python
class VoiceCommander:
    def __init__(self):
        rospy.init_node('voice_command_node')
        self.pub = rospy.Publisher('/voice_commands', String, queue_size=10)
        
        # Recognizer configuration
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = 3000
        self.recognizer.pause_threshold  = 0.8
        
        # Automatically select a suitable microphone
        mic_list = sr.Microphone.list_microphone_names()
        idx = [m for m in mic_list if 'webcam' in m.lower() or 'mic' in m.lower()][0]
        self.microphone = sr.Microphone(device_index=mic_list.index(idx))
        
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
            rospy.loginfo("Microphone calibrated. Say 'pasillo [number] silla [number]'.")
```

1. **rospy.init\_node**
   Initializes the node as `voice_command_node`.
2. **Publisher `/voice_commands`**
   Prepares to send recognized text commands.
3. **Recognizer settings**

   * `energy_threshold=3000`: higher sensitivity to quieter speech.
   * `pause_threshold=0.8`: shorter wait after phrase end.
4. **Microphone selection**
   Scans available devices and picks one containing “webcam” or “mic” in its name.
5. **Ambient noise adjustment**
   Captures 1 second of background noise for calibration.

---

### 3.3 Method `listen()`

```python
def listen(self):
    rate = rospy.Rate(2)  # 2 Hz polling
    while not rospy.is_shutdown():
        try:
            with self.microphone as source:
                audio = self.recognizer.listen(source, timeout=2, phrase_time_limit=3)
            
            text = self.recognizer.recognize_google(audio, language='es-ES').lower()
            text = (text.replace("uno","1")
                        .replace("dos","2")
                        .replace("tres","3"))
            
            if "pasillo" in text and "silla" in text:
                self.pub.publish(text)
                rospy.loginfo(f"Detected command: {text}")
        except sr.WaitTimeoutError:
            continue
        except Exception as e:
            rospy.logwarn(f"Temporary error: {e}")
        rate.sleep()
```

1. **rospy.Rate(2)**
   Checks for new audio twice per second.
2. **recognizer.listen**

   * `timeout=2s`: fail if no speech starts within 2 seconds.
   * `phrase_time_limit=3s`: maximum recording length.
3. **recognize\_google**
   Sends audio to Google’s API for transcription in Spanish (`es-ES`).
4. **Post-processing**
   Converts “uno”, “dos”, “tres” to numeric digits for consistency.
5. **Command filter**
   Only publishes texts containing both “pasillo” and “silla”.
6. **Error handling**

   * Ignores timeouts and logs other exceptions as warnings.

---

### 3.4 Execution Entry Point

```python
if __name__ == '__main__':
    VoiceCommander().listen()
```

* Instantiates `VoiceCommander` and starts the listening loop until ROS shutdown or manual interrupt.


## 4. Navigation Manager (`navigation_manager.py`)

This node orchestrates the workflow by receiving voice commands, QR detections, and distance measurements, then publishing navigation directives to the Arduino.


### 4.1 Headers and Imports

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Float32
````

* **#!/usr/bin/env python3**
  Allows the script to run as a Python 3 ROS node.
* **rospy**
  ROS client library for Python.
* **std\_msgs.msg.String / Float32**
  Message types for incoming text commands, QR data, and ultrasonic distance values.

---

### 4.2 Class `NavigationManager`

```python
class NavigationManager:
    def __init__(self):
        rospy.init_node('navigation_manager')
        self.nav_pub = rospy.Publisher('/nav_commands', String, queue_size=10)
        rospy.Subscriber('/voice_commands', String, self.handle_voice)
        rospy.Subscriber('/qr_detected',    String, self.handle_qr)
        rospy.Subscriber('/distancia',      Float32, self.handle_distance)

        self.pasillo_objetivo  = 0
        self.silla_objetivo    = 0
        self.distancia_actual  = 999.9
        self.umbral_detencion  = 15.0   # cm safety threshold

        self.etapa = "esperando"       # stages: esperando, buscando_pasillo, girando, buscando_silla, completado
```

* **Publisher `/nav_commands`**
  Sends commands such as `iniciar:2:3`, `girar_qr`, `detener` to the Arduino node.
* **Subscribers**

  * `/voice_commands` → `handle_voice`: receives “pasillo X silla Y” or “detener”.
  * `/qr_detected`    → `handle_qr`: processes QR detections.
  * `/distancia`      → `handle_distance`: monitors front obstacle proximity.
* **State Variables**

  * `pasillo_objetivo`, `silla_objetivo`: numeric targets from voice.
  * `distancia_actual`, `umbral_detencion`: for obstacle-awareness.
  * `etapa`: controls current mission phase.

---

### 4.3 Voice Handling: `handle_voice()`

```python
def handle_voice(self, msg):
    text = msg.data.lower()
    if "detener" in text:
        self.nav_pub.publish("detener")
        return

    if "pasillo" in text and "silla" in text:
        parts = text.split()
        p = int(parts[parts.index("pasillo")+1])
        s = int(parts[parts.index("silla")+1])
        if 1 <= p <= 3 and 1 <= s <= 5:
            self.pasillo_objetivo = p
            self.silla_objetivo   = s
            self.nav_pub.publish(f"iniciar:{p}:{s}")
            self.etapa = "buscando_pasillo"
```

1. **Stop command**
   If user says “detener,” immediately publish `"detener"` and exit.
2. **Start command**

   * Parses numbers after “pasillo” and “silla.”
   * Validates ranges (e.g. halls 1–3, chairs 1–5).
   * Saves targets and sends `"iniciar:p:s"` to Arduino.
   * Advances to the “buscando\_pasillo” stage.

---

### 4.4 QR Handling: `handle_qr()`

```python
def handle_qr(self, msg):
    qr = msg.data
    rospy.loginfo(f"QR detected: {qr}")

    if "qr_pasillo_" in qr:
        p = int(qr.split("_")[2])
        if p == self.pasillo_objetivo:
            self.nav_pub.publish("girar_qr")
            self.etapa = "girando"

    elif "qr_silla_" in qr:
        parts = qr.split("_")
        p, s = int(parts[2]), int(parts[3])
        if p == self.pasillo_objetivo and s == self.silla_objetivo:
            self.nav_pub.publish("detener")
            self.etapa = "completado"
```

1. **Hall QR**

   * Extracts the hall number.
   * If it matches the target, sends `"girar_qr"` to initiate a sharp turn.
   * Updates stage to “girando.”
2. **Chair QR**

   * Extracts hall and chair numbers.
   * If both match the targets, publishes `"detener"` and marks “completado.”

---

### 4.5 Distance Handling: `handle_distance()`

```python
def handle_distance(self, msg):
    self.distancia_actual = msg.data
    rospy.loginfo(f"Distance: {msg.data:.2f} cm")
    if msg.data < self.umbral_detencion:
        rospy.logwarn(f"Object at {msg.data:.2f} cm — avoiding collision.")
    else:
        rospy.loginfo("No obstacle within threshold.")
```

* Logs every distance reading.
* Warns if the robot gets closer than 15 cm, alerting users but does not issue a stop here (Arduino handles collision avoidance).

---

### 4.6 Main Loop: `run()`

```python
def run(self):
    rospy.spin()
```

* Enters a blocking loop, waiting for callbacks until shutdown.

---

### 4.7 Execution Entry Point

```python
if __name__ == '__main__':
    NavigationManager().run()
```

* Creates a `NavigationManager` instance and starts processing events.


## 5. Launch File (`robot_system.launch`)

This launch file ties together all nodes—Arduino serial bridge, vision, voice, and navigation—so you can start the entire system with a single command.

```xml
<launch>
    <!-- Voice command node: listens for “pasillo X silla Y” via microphone -->
    <node pkg="robot_guia"
          type="voice_command_node.py"
          name="voice_cmd"
          output="screen"/>

    <!-- Vision node: captures camera feed and detects QR codes -->
    <node pkg="robot_guia"
          type="camera_node.py"
          name="cam_pub"
          output="screen"/>

    <!-- Navigation manager: orchestrates voice, QR, distance → navigation commands -->
    <node pkg="robot_guia"
          type="navigation_manager.py"
          name="nav_manager"
          output="screen"/>

    <!-- ROS–Arduino bridge: communicates with the Elegoo UNO via rosserial -->
    <node pkg="rosserial_python"
          type="serial_node.py"
          name="arduino_node"
          output="screen">
        <!-- Serial port to which the Arduino is connected -->
        <param name="port"  value="/dev/ttyUSB0"/>
        <!-- Must match the baud rate set in Arduino setup() -->
        <param name="baud"  value="57600"/>
    </node>
</launch>
````

### 5.1 Structure and Parameters

1. **`<launch>`**
   Top-level tag that groups all node definitions into one file.

2. **Voice Command Node**

   * **pkg**: `robot_guia` (your custom ROS package).
   * **type**: the Python script `voice_command_node.py`.
   * **name**: the ROS node name `voice_cmd`.
   * **output="screen"**: log output appears in your console.

3. **Vision Node**

   * Mirrors the same attributes as voice but runs `camera_node.py`.
   * Node name `cam_pub` makes logs identifiable in your terminal.

4. **Navigation Manager Node**

   * Runs `navigation_manager.py` under the name `nav_manager`.
   * Subscribes to `/voice_commands`, `/qr_detected`, and `/distancia`.
   * Publishes `/nav_commands` to the Arduino.

5. **ROS–Serial Bridge**

   * Package `rosserial_python` provides `serial_node.py` which handles ROS messages over a serial link.
   * **`port`**: set to the USB device (e.g. `/dev/ttyUSB0`). Adjust if your Arduino enumerates differently.
   * **`baud`**: must match the Arduino’s `nh.getHardware()->setBaud(57600)` call.

### 5.2 How It Works

* When you run `roslaunch robot_guia robot_system.launch`, ROS will:

  1. Start the **voice\_cmd** node and begin listening for spoken commands.
  2. Start the **cam\_pub** node and begin detecting QR codes in the camera feed.
  3. Start the **nav\_manager** node to coordinate inputs and issue `/nav_commands`.
  4. Start the **arduino\_node** bridge, exposing the Arduino firmware as a ROS node that publishes distance and state, and subscribes to navigation commands.

With this setup, all four components run concurrently. You speak “pasillo 2 silla 3,” the voice node publishes it, the navigation manager processes it and sends `iniciar:2:3` over serial, the Arduino begins line-following, the vision node detects QR codes and so on—fully automated.

You’ve covered the core components thoroughly. A few additional sections you might add to make the README even more complete:


## 6. Installation & Dependencies

- **ROS Melodic/Noetic**  
- Python packages: `opencv-python`, `pyzbar`, `speech_recognition`, `pyaudio` (for mic access)  
- Arduino library: `rosserial_arduino`  
- On Ubuntu you can install:  
  ```bash
  sudo apt-get update
  sudo apt-get install ros-noetic-rosserial-arduino ros-noetic-rosserial-python \
                       python3-opencv python3-pyzbar python3-speechrecognition \
                       python3-pyaudio


---

## 7. Build & Run

1. **Compile the Arduino sketch**

   * Import the `.ino` into the Arduino IDE, install `rosserial` library, flash to the UNO at 57600 baud.
2. **Build ROS workspace**

   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```
3. **Launch everything**

   ```bash
   roslaunch robot_guia robot_system.launch
   ```

---

## 8. Configuration

* **Serial Port**: If your board appears as `/dev/ttyACM0` instead of `/dev/ttyUSB0`, edit the `<param name="port">` in `robot_system.launch`.
* **Camera Index**: To use a different camera, adjust `cv2.VideoCapture(0)` (e.g. `1`, `2`).
* **Speech Mic**: If speech node picks the wrong device, we can force a device index in `voice_command_node.py`.

---

## 9. Topics & Message Examples

| Topic             | Message Type       | Example Payload              |
| ----------------- | ------------------ | ---------------------------- |
| `/voice_commands` | `std_msgs/String`  | `"pasillo 1 silla 4"`        |
| `/qr_detected`    | `std_msgs/String`  | `"qr_pasillo_1"`             |
| `/distancia`      | `std_msgs/Float32` | `23.47`                      |
| `/nav_commands`   | `std_msgs/String`  | `"iniciar:1:4"`, `"detener"` |

---

## 10. Troubleshooting

* **No connection to Arduino**

  * Check that the USB cable is data-capable, not just charging.
  * Ensure the baud rates match (both set to 57600).
* **QR codes not detected**

  * Print `qr.data` in console to verify format.
  * Increase camera resolution or brightness.
* **Voice node hangs or misrecognizes**

  * Run `arecord -l` / `pactl list sources` to confirm mic name.
  * Tweak `energy_threshold` and `pause_threshold`.

---

## 11. Future Improvements

* **Dynamic path planning** using `move_base`.
* **ROS action interface** for goal feedback.
* **Web dashboard** (e.g. `rqt`) for live status and manual override.
* **Logging & replay**: record all `/nav_commands` and sensor data for offline analysis.

---

## 12. Authors


* **Authors**: Alejandra Cobos Avila, Jonathan Eliasib Rosas Tlaczani, Jorge Ignacio Serrano Baez




