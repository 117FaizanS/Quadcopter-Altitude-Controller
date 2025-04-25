// Pin definitions
const int triggerPin = 8;  // Trigger pin for the ultrasonic sensor
const int echoPin = 7;     // Echo pin for the ultrasonic sensor
const int PWMPin = 10;     // Pin for the motor PWM signal

// Target height and control parameters
const int target_height = 10;  // Target height in centimeters
const float Kp = 30.0;         // Proportional Gain
const float Ki = 0.1;         // Integral Gain
const float Kd = 0.0001;         // Derivative Gain
const int minPWM = 0;         // Minimum PWM value
const int maxPWM = 255;       // Maximum PWM value

// Variables
long duration, cm, lastValidCm = 0;
int PWM_offset = 150;
int PWM_signal = 0;
int error = 0;
float integral = 0;
float derivative = 0;
int lastError = 0;
unsigned long lastTime = millis();
bool running = false;

void setup() {
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
    pinMode(PWMPin, OUTPUT);

    Serial.begin(9600);
    Serial.println("System initialized. Send 'start' to begin or 'stop' to halt.");
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        if (command == "start") {
            running = true;
            integral = 0;  // Reset integral term
            lastError = 0; // Reset last error
            lastTime = millis(); // Reset last time
            Serial.println("Starting system...");
        } else if (command == "stop") {
            running = false;
            analogWrite(PWMPin, 0); // Stop the motor
            Serial.println("Stopping system...");
        }
    }

    if (running) {
        unsigned long currentTime = millis();
        unsigned long elapsedTime = currentTime - lastTime;

        digitalWrite(triggerPin, LOW);
        delayMicroseconds(2);
        digitalWrite(triggerPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(triggerPin, LOW);

        duration = pulseIn(echoPin, HIGH);
        cm = microsecondsToCentimeters(duration);

        error = target_height - cm;
        integral += error * (elapsedTime / 1000.0); // Accumulate error over time
        derivative = (error - lastError) / (elapsedTime / 1000.0);
        PWM_signal = Kp * error + Ki * integral + Kd * derivative + PWM_offset;
        PWM_signal = constrain(PWM_signal, minPWM, maxPWM);

        analogWrite(PWMPin, PWM_signal);

        // Debugging Output
        Serial.print("Height: "); Serial.print(cm);
        Serial.print(" cm, Error: "); Serial.print(error);
        Serial.print(", Integral: "); Serial.print(integral);
        Serial.print(", PWM: "); Serial.println(PWM_signal);

        lastError = error;
        lastTime = currentTime;
    }
}

// Conversion of ultrasonic sensor reading ot centimeters
long microsecondsToCentimeters(long microseconds) {
    return microseconds / 29 / 2; // Speed of sound conversion
}
