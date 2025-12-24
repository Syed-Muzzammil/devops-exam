Experiment 2: Raspberry Pi Interface with LED 
Control 
AIM: To interface an LED with a Raspberry Pi and control it (turn ON and OFF) using a Python program and GPIO pins.

Procedure 
1. Hardware Setup 
o Insert the SD card with Raspberry Pi OS and boot the Pi. 
o Assemble the circuit. 
2. Software Preparation 
o Open a terminal and update packages: 
o sudo apt update && sudo apt upgrade -y 
o Ensure the GPIO library is installed (pre-installed on Raspberry Pi OS): 
o sudo apt install python3-rpi.gpio 
3. Programming 
o Create a Python file: nano led_on_off.py. 
o Enter the code (see below), save and exit (Ctrl+O, Enter, Ctrl+X). 
4. Run the Program: python3 led_on_off.py

Source Code:

import RPi.GPIO as GPIO 
import time 
# Disable warnings (e.g., "GPIO already in use") 
GPIO.setwarnings(False) 
# Use BCM pin numbering 
GPIO.setmode(GPIO.BCM) 
# Set GPIO 17 as output
LED_PIN = 17 
GPIO.setup(LED_PIN, GPIO.OUT) 
try: 
while True: 
GPIO.output(LED_PIN, True) # LED ON 
time.sleep(1) # 1 second delay 
GPIO.output(LED_PIN, False) # LED OFF 
time.sleep(1) # 1 second delay 
except KeyboardInterrupt: 
# Gracefully clean up on Ctrl+C 
GPIO.cleanup()

Result and Conclusion: 
(Result: The LED turns ON for 2 seconds and then turns OFF, demonstrating successful 
control through the Raspberry Pi GPIO pin. 
(Conclusion: This experiment verifies that the Raspberry Pi can directly interface with basic 
output devices and be programmed using Python to control hardware, forming the foundation 
for more complex IoT and embedded applications.



Experiment 3: Raspberry Pi Interface with IR 
(Obstacle) Sensor 
AIM 
To detect the presence of an object using an Infrared (IR) obstacle sensor and indicate detection by printing a message on the Raspberry Pi terminal

Procedure 
1. Hardware: Connect VCC to 5 V, GND to Pi ground, OUT to GPIO 18. 
2. Software: 
o Update OS packages: 
o sudo apt update && sudo apt upgrade -y 
o Ensure GPIO library is present (usually pre-installed): 
o sudo apt install python3-rpi.gpio 
3. Programming: Create a file: nano ir_sensor.py and insert the code below.
4. Run: python3 ir_sensor.py

Source Code 

import RPi.GPIO as GPIO 
import time 
GPIO.setmode(GPIO.BCM) 
GPIO.setup(18, GPIO.IN) 
print("IR Sensor Test - Press Ctrl+C to stop") 
try: 
while True: 
if GPIO.input(18) == GPIO.LOW: # LOW when object is close 
print("Object Detected") 
else: 
print("No Object") 
time.sleep(0.5) 
except KeyboardInterrupt: 
GPIO.cleanup()

Result & Conclusion 
( Result: The terminal displays “Object Detected” whenever an object is within the IR sensor’s 
range. 
( Conclusion: Raspberry Pi successfully receives a digital signal from an IR obstacle sensor, 
demonstrating digital input interfacing











Experiment 4: Raspberry Pi Interface with 
Ultrasonic Sensor (HC-SR0️4️) 
AIM: To measure the distance to an object using the HC-SR0️4️ ultrasonic sensor and display it on the terminal.

Procedure 
1. Assemble circuit with voltage divider on Echo (1 kΩ + 2 kΩ). 
2. Update system if needed. 
3. Install GPIO library (usually present). 
4. Create program: nano ultrasonic.py. 
5. Run: python3 ultrasonic.py

Source Code 
import RPi.GPIO as GPIO 
import time 
GPIO.setmode(GPIO.BCM) 
GPIO.setwarnings(False) 
TRIG = 20 
ECHO = 21 
GPIO.setup(TRIG, GPIO.OUT) 
GPIO.setup(ECHO, GPIO.IN) 
try: 
print("Press Ctrl+C to stop\n") 
while True: 
# Send trigger pulse 
GPIO.output(TRIG, False) 
time.sleep(0.0002) 
GPIO.output(TRIG, True) 
time.sleep(0.00001) 
GPIO.output(TRIG, False) 
# Capture echo times 
while GPIO.input(ECHO) == 0: 
start = time.time() 
while GPIO.input(ECHO) == 1: 
end = time.time() 
# Calculate distance in cm 
distance = (end - start) * 34300 / 2 
# Use .format() for printing 
print("Distance = {:.1f} cm".format(distance)) 
time.sleep(0.5) 
except KeyboardInterrupt: 
print("\nMeasurement stopped by user.") 
GPIO.cleanup()

Explanation Highlights 
( TRIG is pulsed HIGH for 10 µs to start measurement. 
( The sensor sets ECHO HIGH while sound travels; timing that interval gives round-trip time. 
( distance = (duration * 34300)/2 converts to cm (speed of sound ≈343 m/s). 
Result & Conclusion 
(Result: Terminal displays distance to object in centimeters. 
(Conclusion: Raspberry Pi can measure real-world distances using ultrasonic time-of-flight 
Sensing

Experiment 5: Raspberry Pi Interface with DHT11
Temperature & Humidity Sensor 
AIM: To read temperature and humidity data from a DHT11 sensor and display it on the Raspberry Pi terminal.

Procedure:
1. Connect the sensor as above (use a pull-up resistor if not a module). 
2. Install required Python library: 
3. sudo apt update 
4. sudo apt upgrade 
5. pip3 install --break-system-packages dht11 
6. Create program: nano dht11_read.py. 
7. Run: python3 dht11_read.py

Source Code: 
import RPi.GPIO as GPIO 
import dht11 
import time 
# GPIO setup 
GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BCM) 
GPIO.cleanup() 
# Setup sensor 
instance = dht11.DHT11(pin=21) # GPIO21 
while True: 
result = instance.read() 
if result.is_valid(): 
print("Temperature: {} C Humidity: {} %".format(result.temperature, result.humidity)) 
else: 
print("Waiting for valid data...") 
time.sleep(2)

Explanation of Key Lines 
( import Adafruit_DHT – Imports the dedicated library to communicate with DHT sensors. 
( sensor = Adafruit_DHT.DHT11 – Specifies sensor type. 
( read_retry – Automatically retries a few times for a stable reading. 
( Conditional block prints the readings if successful. 
Result & Conclusion 
( Result: Terminal displays ambient temperature (°C) and relative humidity (%). 
( Conclusion: Confirms Raspberry Pi can interface with single-wire digital sensors for environmental monitoring, a building block for IoT weather-station projects.








Experiment 6: Ultrasonic Sensor and Relay Interface with Raspberry Pi

AIM: To measure the distance of an object using an HC-SR04 ultrasonic sensor and automatically energize or de-energize a relay based on the measured distance.

Procedure 
1. Connect the sensor and relay as per the table. 
2. Export GPIO pins using RPi.GPIO and set mode to BCM. 
3. Send a 10 µs pulse to TRIG; measure time until ECHO goes LOW. 
4. Calculate distance: distance = (time * 34300) / 2. 
5. If distance < threshold (e.g., 20 cm) turn the relay ON, else OFF.

Source Code 
import RPi.GPIO as GPIO 
import time 
GPIO.setmode(GPIO.BCM) 
TRIG = 23 
ECHO = 24 
RELAY = 18 
GPIO.setup(TRIG, GPIO.OUT) 
GPIO.setup(ECHO, GPIO.IN) 
GPIO.setup(RELAY, GPIO.OUT) 
try: 
while True: 
GPIO.output(TRIG, False) 
time.sleep(0.05) 
GPIO.output(TRIG, True) 
time.sleep(0.00001) 
GPIO.output(TRIG, False) 
while GPIO.input(ECHO) == 0: 
start = time.time() 
while GPIO.input(ECHO) == 1: 
end = time.time() 
distance = (end - start) * 17150 
print(f"Distance: {distance:.1f} cm") 
GPIO.output(RELAY, GPIO.HIGH if distance < 20 else GPIO.LOW) 
except KeyboardInterrupt: 
GPIO.cleanup() 

Result & Conclusion 
The relay switched ON whenever an object came within the preset distance (20 cm) and switched 
OFF when the object moved away. This demonstrates successful distance-based actuation using an ultrasonic sensor.







Experiment 7: IR Sensor and Relay Interface with Raspberry Pi

AIM: To detect the presence of an object using an infrared proximity sensor and activate a buzzer when the object is detected.

Procedure 
1. Connect IR sensor output to GPIO 17 and buzzer to GPIO 27. 
2. Configure GPIO; set buzzer as output and IR pin as input with pull-down.
3. Continuously read sensor output. 
4. If logic LOW (object detected), drive buzzer HIGH.	
Source Code 
import RPi.GPIO as GPIO 
import time 
GPIO.setmode(GPIO.BCM) 
IR_PIN = 17 
BUZZER = 27 
GPIO.setup(IR_PIN, GPIO.IN) 
GPIO.setup(BUZZER, GPIO.OUT) 
try: 
while True: 
if GPIO.input(IR_PIN) == GPIO.LOW: # Object detected 
GPIO.output(BUZZER, GPIO.HIGH) 
else: 
GPIO.output(BUZZER, GPIO.LOW) 
time.sleep(0.1) 
except KeyboardInterrupt: 
GPIO.cleanup() 

Result & Conclusion 
The buzzer activated whenever an obstacle was detected by the IR sensor, proving effective object detection and alert generation.









Experiment 8: DHT11 Sensor and Relay Interface with Raspberry Pi

AIM: To sense ambient temperature and humidity using a DHT11 sensor and control a relay when the temperature exceeds a threshold

Procedure 
1. Wire the DHT11 and relay as listed. 
2. Use the Adafruit_DHT library to read temperature and humidity. 
3. If temperature > threshold (e.g., 30 °C), turn relay ON; else OFF.

Source Code 
import Adafruit_DHT 
import RPi.GPIO as GPIO 
import time 
GPIO.setmode(GPIO.BCM) 
RELAY = 20 
GPIO.setup(RELAY, GPIO.OUT) 
sensor = Adafruit_DHT.DHT11 
pin = 21 
try: 
while True: 
humidity, temperature = Adafruit_DHT.read_retry(sensor, pin) 
if humidity is not None and temperature is not None: 
print(f"Temp={temperature:.1f}°C Humidity={humidity:.1f}%") 
GPIO.output(RELAY, GPIO.HIGH if temperature > 30 else GPIO.LOW) 
else: 
print("Sensor failure. Check wiring.") 
time.sleep(2) 
except KeyboardInterrupt: 
GPIO.cleanup() 

Result & Conclusion 
The relay turned ON whenever the temperature crossed the preset 30 °C threshold and OFF otherwise. This verifies correct sensor interfacing and environmental control


---

# 🔬 **EXPERIMENT 9**

## IoT Data Processing using **ESP32 + DHT22 + ThingSpeak** (Wokwi)

---

## **1. Aim**

To measure temperature and humidity using a DHT22 sensor connected to ESP32, process the data locally to generate alerts, and upload the data to ThingSpeak cloud for real-time and historical visualization.

---

## **2. Requirements**

### Hardware (Simulated in Wokwi)

* ESP32 Dev Module
* DHT22 Temperature & Humidity Sensor
* LED (Alert indicator)

### Software

* Wokwi Simulator
* Arduino ESP32 framework
* ThingSpeak cloud platform

---

## **3. Theory**

The ESP32 reads temperature and humidity from the DHT22 sensor.
The measured data is compared with predefined threshold values.
If the temperature or humidity crosses the threshold, an alert is generated and an LED is turned ON.
The sensor data and alert status are sent to ThingSpeak using HTTP for visualization and storage.

---

## **4. Step-by-Step Procedure**

### **Step 1: ThingSpeak Setup**

1. Create a ThingSpeak account
2. Create a new channel
3. Enable fields:

   * Field 1 → Temperature (°C)
   * Field 2 → Humidity (%)
   * Field 3 → Alert (0/1)
4. Note down:

   * Channel ID
   * Write API Key

---

### **Step 2: Wokwi Setup**

1. Go to [https://wokwi.com](https://wokwi.com)
2. Create **ESP32 (Arduino)** project
3. Add:

   * ESP32
   * DHT22
   * LED

---

### **Step 3: Connections**

| Component  | ESP32 Pin |
| ---------- | --------- |
| DHT22 VCC  | 5V        |
| DHT22 DATA | GPIO 15   |
| DHT22 GND  | GND       |
| LED +      | GPIO 2    |
| LED –      | GND       |

---

### **Step 4: Logic Implemented**

* Read temperature & humidity
* Compare with thresholds
* Generate alert
* Send data to ThingSpeak every 20 seconds

---

## **5. Correct Code (ESP32 – Arduino)**

```cpp
#include <WiFi.h>
#include <HTTPClient.h>
#include <DHTesp.h>

const char* ssid = "Wokwi-GUEST";
const char* password = "";

const char* server = "api.thingspeak.com";
const char* writeAPIKey = "YOUR_WRITE_API_KEY";

#define DHT_PIN 15
#define LED_ALERT_PIN 2

#define TEMP_HIGH_THRESHOLD 30.0
#define TEMP_LOW_THRESHOLD 18.0
#define HUM_HIGH_THRESHOLD 80.0

DHTesp dht;

void setup() {
  Serial.begin(115200);
  pinMode(LED_ALERT_PIN, OUTPUT);
  dht.setup(DHT_PIN, DHTesp::DHT22);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void loop() {
  static unsigned long lastSend = 0;
  if (millis() - lastSend < 20000) return;
  lastSend = millis();

  TempAndHumidity data = dht.getTempAndHumidity();
  float temp = data.temperature;
  float hum = data.humidity;

  int alert = 0;
  digitalWrite(LED_ALERT_PIN, LOW);

  if (temp > TEMP_HIGH_THRESHOLD || temp < TEMP_LOW_THRESHOLD || hum > HUM_HIGH_THRESHOLD) {
    alert = 1;
    digitalWrite(LED_ALERT_PIN, HIGH);
  }

  HTTPClient http;
  String url = "http://api.thingspeak.com/update?api_key=" +
               String(writeAPIKey) +
               "&field1=" + String(temp) +
               "&field2=" + String(hum) +
               "&field3=" + String(alert);

  http.begin(url);
  http.GET();
  http.end();
}
```

---

## **6. Output**

* Serial Monitor shows temperature, humidity, and alert status
* LED turns ON when threshold is crossed
* ThingSpeak displays real-time and historical graphs

---

## **7. Result**

Thus, temperature and humidity were successfully monitored using ESP32 and visualized on ThingSpeak cloud.

---

---

# 🔬 **EXPERIMENT 10**

## IoT Data Processing using **Raspberry Pi Pico W + HC-SR04 + ThingSpeak** (Wokwi)

---

## **1. Aim**

To measure distance using HC-SR04 ultrasonic sensor with Raspberry Pi Pico W, generate alerts when an object is too close, and upload data to ThingSpeak for visualization.

---

## **2. Requirements**

### Hardware

* Raspberry Pi Pico W
* HC-SR04 Ultrasonic Sensor
* LED

### Software

* Wokwi Simulator
* MicroPython
* ThingSpeak

---

## **3. Theory**

The HC-SR04 sensor works on the **time-of-flight principle**.
The Pico W triggers an ultrasonic pulse and measures the echo time.
Distance is calculated using the speed of sound.
If the distance is below a threshold, an alert is generated and sent to the cloud.

---

## **4. Step-by-Step Procedure**

### **Step 1: ThingSpeak Setup**

1. Create a channel
2. Enable:

   * Field 1 → Distance (cm)
   * Field 2 → Alert (0/1)
3. Note Channel ID & Write API Key

---

### **Step 2: Wokwi Setup**

1. New Project → Raspberry Pi Pico W
2. Framework → MicroPython
3. Add HC-SR04 and LED

---

### **Step 3: Connections**

| Component       | Pico W Pin |
| --------------- | ---------- |
| HC-SR04 Trigger | GP15       |
| HC-SR04 Echo    | GP14       |
| LED             | GP13       |
| VCC             | 5V         |
| GND             | GND        |

---

## **5. Correct Code (MicroPython)**

```python
import network
import urequests
import time
import machine

ssid = "Wokwi-GUEST"
password = ""

server = "api.thingspeak.com"
write_api_key = "YOUR_WRITE_API_KEY"

trigger = machine.Pin(15, machine.Pin.OUT)
echo = machine.Pin(14, machine.Pin.IN)
led = machine.Pin(13, machine.Pin.OUT)

DISTANCE_THRESHOLD = 10.0

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    while not wlan.isconnected():
        time.sleep(0.5)

def measure_distance():
    trigger.off()
    time.sleep_us(2)
    trigger.on()
    time.sleep_us(10)
    trigger.off()

    while echo.value() == 0:
        start = time.ticks_us()
    while echo.value() == 1:
        end = time.ticks_us()

    duration = end - start
    distance = (duration * 0.0343) / 2
    return distance

connect_wifi()
last_send = 0

while True:
    if time.time() - last_send < 20:
        time.sleep(1)
        continue

    last_send = time.time()
    distance = measure_distance()

    alert = 0
    led.off()

    if distance < DISTANCE_THRESHOLD:
        alert = 1
        led.on()

    url = f"http://{server}/update?api_key={write_api_key}&field1={distance:.1f}&field2={alert}"
    urequests.get(url)
```

---

## **6. Output**

* Console shows distance values
* LED turns ON when object is close
* ThingSpeak displays distance graph

---

## **7. Result**

Thus, distance measurement and alert generation using Pico W and HC-SR04 was successfully implemented and visualized on ThingSpeak.

---

