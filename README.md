
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

