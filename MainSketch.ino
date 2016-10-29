const int LEFT_MOTOR_ENABLE_PIN = 3;
const int LEFT_MOTOR_INPUT1_PIN = 1;
const int LEFT_MOTOR_INPUT2_PIN = 2;

const int RIGHT_MOTOR_ENABLE_PIN = 5;
const int RIGHT_MOTOR_INPUT1_PIN = 4;
const int RIGHT_MOTOR_INPUT2_PIN = 0;

const int RIGHT_IR_PIN = 6;
const int LEFT_IR_PIN = 7;

const int COLOR_S2_PIN = 9;
const int COLOR_S3_PIN = 8;
const int COLOR_INPUT_PIN = 11;

const int ULTRASONIC_TRIGGER_PIN = 10;

enum ColorEnum {
    Red,
    Green,
    Blue,
    Yellow
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

class SensorManagerClass {
private:
    IRSensorClass* LeftIRSensor;
    IRSensorClass* RightIRSensor;
    ColorSensorClass* ColorSensor;
public:
    SensorManagerClass(IRSensorClass* leftIRSensor, IRSensorClass* rightIRSensor, ColorSensorClass* colorSensor) {
        LeftIRSensor = leftIRSensor;
        RightIRSensor = rightIRSensor;
        ColorSensor = colorSensor;
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
};

class MotorClass {
private:
    int EnablePin;
    int Input1Pin;
    int Input2Pin;
public:
    const int MinAnalogValue = 0;
    const int MaxAnalogValue = 255;

    MotorClass(int enabePin, int input1Pin, int input2Pin) {
        EnablePin = enabePin;
        Input1Pin = input1Pin;
        input2Pin = input2Pin;
    }    

    void SetSpeed(float speed) {
        int value = MinAnalogValue + abs(speed) * (MaxAnalogValue - MinAnalogValue);
        analogWrite(EnablePin, value);
        SetDirection(signbit(speed));
    }

    void SetDirection(int direction) {
        digitalWrite(Input1Pin, direction);
        digitalWrite(Input2Pin, !direction);
    }

    void DisableMotor() {
        digitalWrite(Input1Pin, LOW);
        digitalWrite(Input2Pin, LOW);
    }
};

class MovementClass {
private:
    MotorClass* LeftMotor;
    MotorClass* RightMotor;
public:
    MovementClass(MotorClass* leftMotor, MotorClass* rightMotor) {
        LeftMotor = leftMotor;
        RightMotor = rightMotor;
    }

    void SetSpeedAndGoForward(float speed) {
        LeftMotor->SetSpeed(speed);
        RightMotor->SetSpeed(speed);
    }

    void SetRotateRadiusAndSpeed(float radius, float speed) {
        // TODO: Написать код который бы распределял скорость вращения между моторами, дифференциал типо.
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
        Movement->GetLeftMotor()->SetSpeed(SensorManager->GetLeftIRSensor()->GetState());
        Movement->GetRightMotor()->SetSpeed(SensorManager->GetRightIRSensor()->GetState());
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

MovementClass* MainMovement;

SensorManagerClass* MainSensorManager;

BaseStrategyClass* MainStrategy;

void setup() {
    MainMovement =      new MovementClass(new MotorClass(LEFT_MOTOR_ENABLE_PIN,  LEFT_MOTOR_INPUT1_PIN,  LEFT_MOTOR_INPUT2_PIN),
                                     new MotorClass(RIGHT_MOTOR_ENABLE_PIN, RIGHT_MOTOR_INPUT1_PIN, RIGHT_MOTOR_INPUT2_PIN));
                            
    MainSensorManager = new SensorManagerClass(
                                    new IRSensorClass(LEFT_IR_PIN), 
                                    new IRSensorClass(RIGHT_IR_PIN),
                                    new ColorSensorClass(COLOR_INPUT_PIN, COLOR_S2_PIN, COLOR_S3_PIN));

    MainStrategy = new DebugColorSensorClass(MainSensorManager, MainMovement);

    Serial.begin(9600);
}

void loop() {
    MainStrategy->Loop();
}
