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
    Red,
    Green,
    Blue,
    Yellow,
    Orange
};

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

    int GetState(int s2PinState, int s3PinState) {
        digitalWrite(S2_PIN, s2PinState);
        digitalWrite(S3_PIN, s3PinState);
        return pulseIn(SENSOR_IN_PIN, LOW);   
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
        int red;
        int blue;
        int green;

        red = GetState(LOW, LOW);
        blue = GetState(LOW, HIGH);
        green = GetState(HIGH, HIGH);

        float t_yellow = abs(red*1.0f/green);

        ColorEnum res;

        if (blue < green && blue < red) {
            res = Blue;
            Serial.println("Color is Blue");
        } else if (t_yellow + 0.1 > 0.8 && t_yellow - 0.1f < 0.8) {
            res = Yellow;
            Serial.println("Color is Yellow");            
        } else if (red < green) {
            res = Red;
            Serial.println("Color is Red");
        } else {
            res = Green;
            Serial.println("Color is Green");
        }

        Serial.print("Red: ");
        Serial.print(red);
        Serial.print(" Green: ");
        Serial.print(green);
        Serial.print(" Blue: ");
        Serial.print(blue);
        Serial.print(" Yellow: ");
        Serial.println(t_yellow);
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
