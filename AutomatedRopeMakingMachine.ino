#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HX711.h>
#include <AccelStepper.h>
#include <Servo.h>
#include <Encoder.h>

// Pin Definitions
const int SERVO_PIN = 12;             // Servo motor
const int LOAD_CELL_DOUT_PIN = 2;     // Load cell HX711 DOUT
const int LOAD_CELL_SCK_PIN = 7;      // Load cell HX711 SCK
const int ENCODER_CLK_PIN = 9;        // Rotary encoder CLK
const int ENCODER_DT_PIN = 8;         // Rotary encoder DT
const int ENCODER_SW_PIN = 13;        // Rotary encoder SW
const int BTS7960_LPWM_PIN = 11;      // Motor driver LPWM
const int BTS7960_RPWM_PIN = 10;      // Motor driver RPWM
const int NEMA_STEP_PIN = 6;          // Stepper motor STEP pin
const int NEMA_DIR_PIN = 5;           // Stepper motor DIR pin
const int REN = 3;                    // Additional motor control pin (e.g., enable)
const int LEN = 4;                    // Additional motor control pin
const int EMERGENCY_STOP_BTN = A0;    // Emergency stop button

// Initialize Libraries and Components
HX711 scale;
Servo myServo;
LiquidCrystal_I2C lcd(0x27, 16, 2);     // LCD display: address 0x27, 16x2
AccelStepper stepper(AccelStepper::DRIVER, NEMA_STEP_PIN, NEMA_DIR_PIN);
Encoder myEnc(ENCODER_CLK_PIN, ENCODER_DT_PIN);

// Control and Configuration Variables
volatile int encoderPos = 0;           // Position of rotary encoder
int lastEncoderPos = -1;               // Last position of rotary encoder
bool encoderPressed = false;           // Flag for encoder button press
bool mode = false;                     // Mode: false = tension, true = length
float setTension = 0;                  // Target tension
float setRopeLength = 0;               // Target rope length
bool running = false;                  // System running flag
unsigned long lastDebounceTime = 0;    // Debounce timing for encoder button
const unsigned long DEBOUNCE_DELAY = 50;     // Debounce delay (ms)
const unsigned long LONG_PRESS_DURATION = 1000; // Long press duration for mode switching (ms)
float calibrationFactor = 2000;             // Calibration factor for scale
float weight = 0;                       // Weight reading from load cell
float tent = 0;                         // Calculated tension in the rope
int motorSpeed = 20;                    // Motor speed (PWM value)
int ropeLength = 0;                     // Measured rope length

void setup() {
    Serial.begin(9600);
    lcd.init();
    lcd.backlight();

    // Pin Setup
    pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
    pinMode(BTS7960_LPWM_PIN, OUTPUT);
    pinMode(BTS7960_RPWM_PIN, OUTPUT);
    pinMode(NEMA_STEP_PIN, OUTPUT);
    pinMode(NEMA_DIR_PIN, OUTPUT);
    pinMode(REN, OUTPUT);
    pinMode(LEN, OUTPUT);
    pinMode(EMERGENCY_STOP_BTN, INPUT);

    // Initialize Components
    lcd.print("Initializing...");
    myServo.attach(SERVO_PIN);
    stepper.setMaxSpeed(1000);
    stepper.setAcceleration(500);
    scale.begin(LOAD_CELL_DOUT_PIN, LOAD_CELL_SCK_PIN);
    scale.set_scale();
    scale.tare();                       // Reset scale to zero
    
    Serial.println("Setup complete");
    lcd.clear();
    lcd.print("Set Tension:");
}

// Main loop
void loop() {
    readEncoder();       // Update encoder position for tension/length setting
    handleButton();      // Handle encoder button (short/long press for start/mode toggle)
    monitorOperation();  // Monitor tension and length while system is running
}

void readEncoder() {
    // Get encoder position and update desired values based on current mode
    encoderPos = myEnc.read();
    float position = encoderPos * (360.0 / 360.0); // Degrees per pulse (assuming 360 pulses per revolution)

    if (lastEncoderPos != encoderPos) {
        lastEncoderPos = encoderPos;
        if (!mode) { // Tension mode
            setTension = position;
            lcd.setCursor(0, 1);
            lcd.print("Tension: ");
            lcd.print(setTension);
            lcd.print(" ");
            Serial.print("Set Tension: ");
            Serial.println(setTension);
        } else {     // Length mode
            setRopeLength = position;
            lcd.setCursor(0, 1);
            lcd.print("Length: ");
            lcd.print(setRopeLength);
            lcd.print(" ");
            Serial.print("Set Length: ");
            Serial.println(setRopeLength);
        }
    }
}

void handleButton() {
    int buttonState = digitalRead(ENCODER_SW_PIN);
    if (buttonState == LOW && (millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
        lastDebounceTime = millis();
        if (!encoderPressed) {
            encoderPressed = true;
            unsigned long pressDuration = millis() - lastDebounceTime;

            // Check for long press
            if (pressDuration > LONG_PRESS_DURATION) {
                mode = !mode; // Toggle between tension and length mode
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print(mode ? "Set Length:" : "Set Tension:");
            } else if (!running) {
                startSystem(); // Start the system if not already running
            }
        }
    } else if (buttonState == HIGH) {
        encoderPressed = false;
    }
}

void monitorOperation() {
    if (running) {
        updateWeightAndTension();

        if (tent >= setTension || ropeLength >= setRopeLength) {
            stopSystem(); // Stop if either target is reached
        } else {
            stepper.run(); // Continue if within targets
        }

        lcd.setCursor(0, 1);
        lcd.print("Length: ");
        lcd.print(ropeLength);
    }
}

void startSystem() {
    running = true;
    ropeLength = 0;
    stepper.setCurrentPosition(0);
    lcd.clear();
    lcd.print("Running...");
    analogWrite(BTS7960_LPWM_PIN, motorSpeed); // Set motor speed
    analogWrite(BTS7960_RPWM_PIN, motorSpeed);
    Serial.print("Motor Speed: ");
    Serial.println(motorSpeed);
}

void stopSystem() {
    running = false;
    analogWrite(BTS7960_LPWM_PIN, 0); // Stop motor
    analogWrite(BTS7960_RPWM_PIN, 0);
    stepper.stop(); // Stop stepper motor
    lcd.clear();
    lcd.print("Stopped");
    Serial.println("System Stopped");
}

void updateWeightAndTension() {
    scale.set_scale(calibrationFactor);
    weight = scale.get_units(10);
    tent = (weight / 1000) * 10; // Convert grams to Newtons
    lcd.setCursor(0, 0);
    lcd.print("Tension: ");
    lcd.print(tent, 1);
    lcd.print("N");
}
