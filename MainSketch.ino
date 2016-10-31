#include <NewPing.h>
#include <Servo.h>

const int LEFT_MOTOR_ENABLE_PIN = 3;
const int LEFT_MOTOR_INPUT1_PIN = 1;
const int LEFT_MOTOR_INPUT2_PIN = 2;

const int RIGHT_MOTOR_ENABLE_PIN = 5;
const int RIGHT_MOTOR_INPUT1_PIN = 4;
const int RIGHT_MOTOR_INPUT2_PIN = 0;

const int RIGHT_IR_PIN = 6;
const int LEFT_IR_PIN = 7;

const int COLOR_S2_PIN = 11;
const int COLOR_S3_PIN = 8;
const int COLOR_INPUT_PIN = 13;

const int SONAR_TRIGGER_PIN = 10;
const int SONAR_ECHO_PIN = 12;

const int SERVO_PIN = 9;

enum ColorEnum {
    White,
    Red,
    Green,
    Blue,
    Yellow,
    Orange
};

char const * getColorName(ColorEnum color) {
    switch (color) {
        case White: return "White";
        case Red: return "Red";
        case Green: return "Green";
        case Blue: return "Blue";
        case Yellow: return "Yellow";
        case Orange: return "Orange";
    }
}

class IRSensorClass {
private:
    int SENSOR_PIN;
public:
    IRSensorClass(int pin) {
        SENSOR_PIN = pin;
        pinMode(SENSOR_PIN, INPUT);
    }

    int GetState() {
        return digitalRead(SENSOR_PIN);
    }
};

class ColorSensorClass {
private:
    int SENSOR_IN_PIN;
    int S2_PIN;
    int S3_PIN;

    unsigned long GetState(int s2PinState, int s3PinState) {
        digitalWrite(S2_PIN, s2PinState);
        digitalWrite(S3_PIN, s3PinState);
        return pulseIn(SENSOR_IN_PIN, LOW);   
    }

    float dist(const float a[3], const float b[3]) {
        float res = 0;
        for (int i = 0; i < 3; i++)
            res += fabs(a[i] - b[i]);
        return res;
    }
public:
    ColorSensorClass(int sensorPin, int s2Pin, int s3Pin) {
        SENSOR_IN_PIN = sensorPin;
        S2_PIN = s2Pin;
        S3_PIN = s3Pin;
        pinMode(SENSOR_IN_PIN, INPUT);
        pinMode(S2_PIN, OUTPUT);
        pinMode(S3_PIN, OUTPUT);                
    }

    ColorEnum GetState() {
        unsigned long redRaw, greenRaw, blueRaw, minValue, maxValue;
        redRaw = GetState(LOW, LOW);
        greenRaw = GetState(HIGH, HIGH);
        blueRaw = GetState(LOW, HIGH);
        maxValue = max(redRaw, max(greenRaw, blueRaw)) + 100;
        
        float minDist = 100, norm[3] = {redRaw / (float)maxValue, greenRaw / (float)maxValue, blueRaw / (float)maxValue};
        Serial.print("Red: ");
        Serial.print(norm[0]);
        Serial.print(" Green: ");
        Serial.print(norm[1]);
        Serial.print(" Blue: ");
        Serial.println(norm[2]);
        
        ColorEnum res;
        
        float red[] = {0, 1, 1};
        if (dist(norm, red) < minDist) {
            minDist = dist(norm, red);
            res = Red;
        }
        
        float green[] = {1, 0, 1};
        if (dist(norm, green) < minDist) {
            minDist = dist(norm, green);
            res = Green;
        }
        
        float blue[] = {1, 1, 0};
        if (dist(norm, blue) < minDist) {
            minDist = dist(norm, blue);
            res = Blue;
        }
        
        float yellow[] = {0, 0, 1}; // #ffff00
        if (dist(norm, yellow) < minDist) {
            minDist = dist(norm, yellow);
            res = Yellow;
        }
        
        float orange[] = {0, 1 - 0xa5 / 255.0, 1}; // #ffa500
        if (dist(norm, orange) < minDist) {
            minDist = dist(norm, orange);
            res = Orange;
        }
        
        float white[] = {0, 0, 0};
        if (dist(norm, white) < minDist) {
            minDist = dist(norm, white);
            res = White;
        }
        
        Serial.print("Color is ");
        Serial.println(getColorName(res));
    }
};

class DistanceSensorClass {
private:
    NewPing* sonar;
public:
    DistanceSensorClass(int trigPin, int echoPin, int maxDistance) {
        sonar = new NewPing(trigPin, echoPin, maxDistance);
    }

    int GetDistance() {
        return sonar->convert_cm(sonar->ping_median(5));
    }
};

class SensorManagerClass {
private:
    IRSensorClass* LeftIRSensor;
    IRSensorClass* RightIRSensor;
    ColorSensorClass* ColorSensor;
    DistanceSensorClass* DistanceSensor;
public:
    SensorManagerClass(IRSensorClass* leftIRSensor, IRSensorClass* rightIRSensor, ColorSensorClass* colorSensor, DistanceSensorClass* distanceSensor) {
        LeftIRSensor = leftIRSensor;
        RightIRSensor = rightIRSensor;
        ColorSensor = colorSensor;
        DistanceSensor = distanceSensor;
    }

    IRSensorClass* GetLeftIRSensor() {
        return LeftIRSensor;
    }

    IRSensorClass* GetRightIRSensor() {
        return RightIRSensor;
    }

    ColorSensorClass* GetColorSensor() {
        return ColorSensor;
    }

    DistanceSensorClass* GetDistanceSensor() {
        return DistanceSensor;
    }
};

class MotorClass {
private:
    int EnablePin;
    int Input1Pin;
    int Input2Pin;
public:
    const int MinAnalogValue = 0;
    const int MaxAnalogValue = 255;

    MotorClass(int enablePin, int input1Pin, int input2Pin) {
        EnablePin = enablePin;
        Input1Pin = input1Pin;
        Input2Pin = input2Pin;

        pinMode(EnablePin, OUTPUT);
        pinMode(Input1Pin, OUTPUT);
        pinMode(Input2Pin, OUTPUT);
    }    

    void SetSpeed(float speed) {
        int value = MinAnalogValue + abs(speed) * (MaxAnalogValue - MinAnalogValue);
        //analogWrite(EnablePin, value);
        digitalWrite(EnablePin, HIGH);
        SetDirection(signbit(speed));
    }

    void SetDirection(int direction) {  
        digitalWrite(Input1Pin, direction ? HIGH : LOW);
        digitalWrite(Input2Pin, direction ? LOW : HIGH);
    }

    void DisableMotor() {
        digitalWrite(Input1Pin, LOW);
        digitalWrite(Input2Pin, LOW);  
    }
};

class ChassisClass {
private:
    Servo servo;
public:
    ChassisClass(int pin) {
        servo.attach(pin);
    }

    void SetAngle(int angle) {
        servo.write(angle);
    }

    void Drop() {
        SetAngle(100);
    }

    void Lift() {
        SetAngle(10);
    }
};

class MovementClass {
private:
    MotorClass* LeftMotor;
    MotorClass* RightMotor;
    ChassisClass* Chassis;
public:
    MovementClass(MotorClass* leftMotor, MotorClass* rightMotor, ChassisClass* chassisClass) {
        LeftMotor = leftMotor;
        RightMotor = rightMotor;
        Chassis = chassisClass;
    }

    void SetSpeedAndGoForward(float speed) {
        LeftMotor->SetSpeed(speed);
        RightMotor->SetSpeed(speed);
    }

    void SetRotateRadiusAndSpeed(float radius, float speed) {
        // TODO: Написать код который бы распределял скорость вращения между моторами, дифференциал типо.
    }

    ChassisClass* GetChassis() {
        return Chassis;
    }

    MotorClass* GetLeftMotor() {
        return LeftMotor;
    }

    MotorClass* GetRightMotor() {
        return RightMotor;
    }
};

class BaseStrategyClass {
protected:
    SensorManagerClass* SensorManager;
    MovementClass* Movement;
public:
    BaseStrategyClass(SensorManagerClass* sensorManager, MovementClass* movement) {
        Movement = movement;
        SensorManager = sensorManager;
    }

    virtual void Loop() = 0;
};

class SimpleLineStrategyClass : public BaseStrategyClass {
public:
    using BaseStrategyClass::BaseStrategyClass;
    void Loop() {
        Movement->GetLeftMotor()->SetSpeed(SensorManager->GetLeftIRSensor()->GetState() ? 1 : -1);
        Movement->GetRightMotor()->SetSpeed(SensorManager->GetRightIRSensor()->GetState() ? 1 : -1);
    }
};

class DebugColorSensorClass : public BaseStrategyClass {
public:
    using BaseStrategyClass::BaseStrategyClass;
    void Loop() {
        SensorManager->GetColorSensor()->GetState();
        delay(500);
    }
};

class DebugMotorsClass : public BaseStrategyClass {
public:
    using BaseStrategyClass::BaseStrategyClass;
    void Loop() {
        Movement->GetLeftMotor()->SetSpeed(1);
        Movement->GetRightMotor()->SetSpeed(1);
        delay(2000);
        Movement->GetLeftMotor()->DisableMotor();
        Movement->GetRightMotor()->DisableMotor();
        delay(2000);
        Movement->GetLeftMotor()->SetSpeed(-1);
        Movement->GetRightMotor()->SetSpeed(-1);
        delay(2000);
    }
};

class DebugSonarClass : public BaseStrategyClass {
    using BaseStrategyClass::BaseStrategyClass;
    void Loop() {
        Serial.print( SensorManager->GetDistanceSensor()->GetDistance() );
        Serial.println( "cm" );
        delay(300);
    }
};

class DebugChassisClass : public BaseStrategyClass {
    using BaseStrategyClass::BaseStrategyClass;
    void Loop() {
        Movement->GetChassis()->Drop();
        delay(5000);
        Movement->GetChassis()->Lift();
        delay(5000);
    }
};

MovementClass* MainMovement;

SensorManagerClass* MainSensorManager;

BaseStrategyClass* MainStrategy;

void setup() {
    MainMovement =      new MovementClass(new MotorClass(LEFT_MOTOR_ENABLE_PIN,  LEFT_MOTOR_INPUT1_PIN,  LEFT_MOTOR_INPUT2_PIN),
                                          new MotorClass(RIGHT_MOTOR_ENABLE_PIN, RIGHT_MOTOR_INPUT1_PIN, RIGHT_MOTOR_INPUT2_PIN),
                                          new ChassisClass(SERVO_PIN));
                            
    MainSensorManager = new SensorManagerClass(
                                    new IRSensorClass(LEFT_IR_PIN), 
                                    new IRSensorClass(RIGHT_IR_PIN),
                                    new ColorSensorClass(COLOR_INPUT_PIN, COLOR_S2_PIN, COLOR_S3_PIN),
                                    new DistanceSensorClass(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, 300));

    MainStrategy = new SimpleLineStrategyClass(MainSensorManager, MainMovement);
}

void loop() {
    MainStrategy->Loop();
}
