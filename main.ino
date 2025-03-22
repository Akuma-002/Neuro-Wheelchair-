#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_VL53L0X.h>

// WiFi Credentials
const char* ssid = "SSS";        
const char* password = "12345678"; 

// Web Server Setup
WebServer server(80);

// Motor & Servo Pins
#define BREAK_LIGHT 32
#define LED_PIN 2      // Indicator LED
#define SERVO_PIN 13  
#define LEFT_MOTOR_IN1 4
#define LEFT_MOTOR_IN2 5
#define RIGHT_MOTOR_IN3 18
#define RIGHT_MOTOR_IN4 19
Servo steeringServo;  

// VL53L0X Sensor
Adafruit_VL53L0X lox;

// EEG Processing
#define SAMPLE_RATE 256     
#define BAUD_RATE 115200    
#define INPUT_PIN 34        // ESP32 ADC Pin for EEG

void setup() {
    Serial.begin(BAUD_RATE);
    analogReadResolution(12);  // Set ADC resolution for ESP32
    
     pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN3, OUTPUT);
    pinMode(RIGHT_MOTOR_IN4, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN3, LOW);
    digitalWrite(RIGHT_MOTOR_IN4, LOW);
    steeringServo.attach(SERVO_PIN);
    steeringServo.write(90);  // Set servo to center (90°)
    pinMode(BREAK_LIGHT, OUTPUT);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nConnected to WiFi!");
    Serial.println(WiFi.localIP());

    // Web Routes
    server.on("/", handleRoot);
    server.on("/forward", handleForward);
    server.on("/stop", handleStop);
    server.on("/left", handleLeft);
    server.on("/right", handleRight);
    
    server.begin();
    Serial.println("Web server started!");

    // VL53L0X Init
    Wire.begin();
    if (!lox.begin()) {
        Serial.println("Failed to detect VL53L0X sensor!");
        while (1);
    }
        digitalWrite(BREAK_LIGHT, LOW);

}

void loop() {
  
    server.handleClient(); 
    checkObstacle();  // Check for obstacles
    processEEG();     // Process EEG signals
}

// 📌 Obstacle Detection: Stop if object < 10cm
void checkObstacle() {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    // Serial.print("Distance: ");
    // Serial.println(measure.RangeMilliMeter);

    if (measure.RangeStatus == 0 && measure.RangeMilliMeter < 100) {  
        Serial.println("🚨 Obstacle Detected! Beeping...");
        handleStop();  
    }
}


// 📌 EEG Processing
void processEEG() {
    static unsigned long past = 0;
    unsigned long present = micros();
    unsigned long interval = present - past;
    past = present;

    static long timer = 0;
    timer -= interval;

    if (timer < 0) {
        timer += 1000000 / SAMPLE_RATE;
        float sensor_value = analogRead(INPUT_PIN) * (3.3 / 4095.0);
        float signal = EEGFilter(sensor_value);
        Serial.println(signal, 6);
    }
}

// 📌 Web Control Functions
void handleRoot() {
    server.send(200, "text/html", R"rawliteral(
        <!DOCTYPE html>
        <html>
        <head>
            <title>ESP32 Car Control</title>
            <style>
                body { text-align: center; font-family: Arial, sans-serif; margin-top: 50px; }
                h1 { color: #333; }
                button { font-size: 20px; padding: 15px 30px; margin: 10px; cursor: pointer; }
                .control { display: flex; flex-direction: column; align-items: center; }
                .row { display: flex; justify-content: center; }
            </style>
        </head>
        <body>
            <h1>ESP32 Remote Car</h1>
            <div class="control">
                <div class="row">
                    <button onclick="controlCar('forward')" style="background: green; color: white;">Forward</button>
                </div>
                <div class="row">
                    <button onclick="controlCar('left')" style="background: blue; color: white;">Left</button>
                    <button onclick="controlCar('stop')" style="background: gray; color: white;">Stop</button>
                    <button onclick="controlCar('right')" style="background: blue; color: white;">Right</button>
                </div>
            </div>
            <script>
                function controlCar(action) { fetch('/' + action); }
            </script>
        </body>
        </html>
    )rawliteral");
}


void handleForward() {
    digitalWrite(BREAK_LIGHT, LOW);
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN3, HIGH);
    digitalWrite(RIGHT_MOTOR_IN4, LOW);
    digitalWrite(LED_PIN, HIGH);
    server.send(200, "text/plain", "Forward");
}

void handleStop() {
    digitalWrite(BREAK_LIGHT, HIGH);
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN3, LOW);
    digitalWrite(RIGHT_MOTOR_IN4, LOW);
    digitalWrite(LED_PIN, LOW);
    server.send(200, "text/plain", "Stop");
}


void handleLeft() {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN3, HIGH);
    digitalWrite(RIGHT_MOTOR_IN4, LOW);
    server.send(200, "text/plain", "Left");
}

void handleRight() {
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN3, LOW);
    digitalWrite(RIGHT_MOTOR_IN4, LOW);
    server.send(200, "text/plain", "Right");
}


// 📌 EEG Butterworth Filter
float EEGFilter(float input) {
    float output = input;
    static float z1_1 = 0, z2_1 = 0;
    static float z1_2 = 0, z2_2 = 0;
    static float z1_3 = 0, z2_3 = 0;
    static float z1_4 = 0, z2_4 = 0;

    // First biquad section
    float x = output - (-0.95391350 * z1_1) - (0.25311356 * z2_1);
    output = (0.00735282 * x) + (0.01470564 * z1_1) + (0.00735282 * z2_1);
    z2_1 = z1_1;
    z1_1 = x;

    // Second biquad section
    x = output - (-1.20596630 * z1_2) - (0.60558332 * z2_2);
    output = (1.00000000 * x) + (2.00000000 * z1_2) + (1.00000000 * z2_2);
    z2_2 = z1_2;
    z1_2 = x;

    // Third biquad section
    x = output - (-1.97690645 * z1_3) - (0.97706395 * z2_3);
    output = (1.00000000 * x) + (-2.00000000 * z1_3) + (1.00000000 * z2_3);
    z2_3 = z1_3;
    z1_3 = x;

    // Fourth biquad section
    x = output - (-1.99071687 * z1_4) - (0.99086813 * z2_4);
    output = (1.00000000 * x) + (-2.00000000 * z1_4) + (1.00000000 * z2_4);
    z2_4 = z1_4;
    z1_4 = x;

    return output;
}
