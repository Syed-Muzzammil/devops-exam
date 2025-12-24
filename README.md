Perfect — I’ve understood exactly what you want 👍
Below is a **FULL, EXAM-READY explanation of Experiments 1 to 8**, written **from scratch**, in **proper lab-record style**, with:

* **Aim**
* **Apparatus / Requirements**
* **Theory**
* **Step-by-step Procedure**
* **Correct Source Code**
* **Result & Conclusion**
* **Key Viva Points**

This is written so you can **directly write it in the exam or lab record**.

---

# 🔬 **EXPERIMENT 1**

## IoT Device Setup and Configuration (Arduino / Raspberry Pi)

### **Aim**

To configure and set up a basic IoT device such as Arduino or Raspberry Pi and verify proper operation by running a sample program.

---

### **Requirements**

* Arduino board / Raspberry Pi
* USB cable / SD card
* Arduino IDE / Raspberry Pi OS
* Computer system

---

### **Theory**

Before developing IoT applications, the hardware platform must be properly configured. This includes installing required software, connecting the device, selecting correct ports, and testing with a basic program.

---

### **Procedure (Arduino)**

1. Download and install Arduino IDE.
2. Connect Arduino board using USB cable.
3. Open Arduino IDE → Tools → Board → Select correct board.
4. Select correct COM port.
5. Load example program: **Blink**.
6. Upload the program to the board.

---

### **Source Code (Arduino – Blink)**

```cpp
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
}
```

---

### **Result & Conclusion**

**Result:** The onboard LED blinks continuously.
**Conclusion:** The IoT device is successfully configured and ready for further experiments.

---

### **Viva Point**

> Device configuration is the first step in IoT system development.

---

---

# 🔬 **EXPERIMENT 2**

## Raspberry Pi Interface with LED Control

### **Aim**

To interface an LED with Raspberry Pi and control it using Python and GPIO pins.

---

### **Requirements**

* Raspberry Pi
* LED
* Resistor
* Python 3
* RPi.GPIO library

---

### **Theory**

GPIO pins of Raspberry Pi can be programmed to control external devices. By configuring a pin as OUTPUT, digital signals can turn devices ON or OFF.

---

### **Procedure**

1. Boot Raspberry Pi OS.
2. Connect LED to GPIO 17 with resistor.
3. Install GPIO library.
4. Write Python program.
5. Execute the program.

---

### **Source Code**

```python
import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

LED_PIN = 17
GPIO.setup(LED_PIN, GPIO.OUT)

try:
    while True:
        GPIO.output(LED_PIN, True)
        time.sleep(1)
        GPIO.output(LED_PIN, False)
        time.sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()
```

---

### **Result & Conclusion**

**Result:** LED turns ON and OFF every second.
**Conclusion:** Raspberry Pi successfully controls an output device using GPIO.

---

### **Viva Point**

> GPIO pins allow direct hardware control through software.

---

---

# 🔬 **EXPERIMENT 3**

## Raspberry Pi Interface with IR Obstacle Sensor

### **Aim**

To detect the presence of an object using an IR obstacle sensor and display the result.

---

### **Requirements**

* Raspberry Pi
* IR Sensor
* Python 3

---

### **Theory**

IR sensors detect obstacles by emitting infrared light and receiving reflected signals. The output is digital (HIGH/LOW).

---

### **Procedure**

1. Connect IR sensor output to GPIO 18.
2. Configure GPIO as input.
3. Continuously read sensor output.
4. Display result on terminal.

---

### **Source Code**

```python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.IN)

print("IR Sensor Test")

try:
    while True:
        if GPIO.input(18) == GPIO.LOW:
            print("Object Detected")
        else:
            print("No Object")
        time.sleep(0.5)
except KeyboardInterrupt:
    GPIO.cleanup()
```

---

### **Result & Conclusion**

**Result:** Terminal displays object detection status.
**Conclusion:** Raspberry Pi successfully reads digital sensor input.

---

---

# 🔬 **EXPERIMENT 4**

## Raspberry Pi Interface with Ultrasonic Sensor (HC-SR04)

### **Aim**

To measure distance using an ultrasonic sensor and display it on the terminal.

---

### **Theory**

HC-SR04 works on **time-of-flight principle**. Distance is calculated using speed of sound.

---

### **Procedure**

1. Connect TRIG and ECHO pins.
2. Send ultrasonic pulse.
3. Measure echo time.
4. Calculate distance.

---

### **Source Code**

```python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

TRIG = 20
ECHO = 21

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

try:
    while True:
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO) == 0:
            start = time.time()
        while GPIO.input(ECHO) == 1:
            end = time.time()

        distance = (end - start) * 34300 / 2
        print("Distance = {:.1f} cm".format(distance))
        time.sleep(0.5)
except KeyboardInterrupt:
    GPIO.cleanup()
```

---

### **Result & Conclusion**

**Result:** Distance displayed in centimeters.
**Conclusion:** Ultrasonic sensing enables accurate distance measurement.

---

---

# 🔬 **EXPERIMENT 5**

## Raspberry Pi Interface with DHT11 Sensor

### **Aim**

To read temperature and humidity using DHT11 sensor.

---

### **Theory**

DHT11 is a digital sensor providing temperature and humidity values over a single-wire protocol.

---

### **Source Code**

```python
import RPi.GPIO as GPIO
import dht11
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

instance = dht11.DHT11(pin=21)

while True:
    result = instance.read()
    if result.is_valid():
        print("Temp: {}°C  Humidity: {}%".format(
              result.temperature, result.humidity))
    time.sleep(2)
```

---

### **Result & Conclusion**

**Result:** Temperature and humidity displayed.
**Conclusion:** Environmental monitoring using digital sensors is achieved.

---

---

# 🔬 **EXPERIMENT 6**

## Ultrasonic Sensor and Relay Interface

### **Aim**

To control a relay based on distance measured using ultrasonic sensor.

---

### **Source Code**

```python
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
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO) == 0:
            start = time.time()
        while GPIO.input(ECHO) == 1:
            end = time.time()

        distance = (end - start) * 17150
        GPIO.output(RELAY, GPIO.HIGH if distance < 20 else GPIO.LOW)
except KeyboardInterrupt:
    GPIO.cleanup()
```

---

### **Result & Conclusion**

Relay turns ON when object is close. Demonstrates distance-based automation.

---

---

# 🔬 **EXPERIMENT 7**

## IR Sensor and Relay/Buzzer Interface

### **Aim**

To activate a buzzer when an object is detected using IR sensor.

---

### **Source Code**

```python
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

IR_PIN = 17
BUZZER = 27

GPIO.setup(IR_PIN, GPIO.IN)
GPIO.setup(BUZZER, GPIO.OUT)

try:
    while True:
        if GPIO.input(IR_PIN) == GPIO.LOW:
            GPIO.output(BUZZER, GPIO.HIGH)
        else:
            GPIO.output(BUZZER, GPIO.LOW)
        time.sleep(0.1)
except KeyboardInterrupt:
    GPIO.cleanup()
```

---

### **Result & Conclusion**

Buzzer activates on object detection, confirming alert generation.

---

---

# 🔬 **EXPERIMENT 8**

## DHT11 Sensor and Relay Interface

### **Aim**

To control a relay based on temperature measured using DHT11 sensor.

---

### **Source Code**

```python
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
        if temperature is not None:
            GPIO.output(RELAY, GPIO.HIGH if temperature > 30 else GPIO.LOW)
        time.sleep(2)
except KeyboardInterrupt:
    GPIO.cleanup()
```

---

### **Result & Conclusion**

Relay turns ON when temperature exceeds threshold, proving environmental control.

---

## ✅ FINAL NOTE

This explanation is:
✔️ **Lab-record ready**
✔️ **Viva safe**
✔️ **Step-by-step**
✔️ **Exactly matches your manual**

If you want, I can:

* Convert this into **PDF lab record**
* Shorten answers for **written exam**
* Add **block diagrams**

Just tell me 👍


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
```

## **6. Output**

* Console shows distance values
* LED turns ON when object is close
* ThingSpeak displays distance graph

---

## **7. Result**

Thus, distance measurement and alert generation using Pico W and HC-SR04 was successfully implemented and visualized on ThingSpeak.

---

```
