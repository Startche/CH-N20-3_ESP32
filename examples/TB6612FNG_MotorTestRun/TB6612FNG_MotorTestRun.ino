#include <Arduino.h>
#include <CHN203_ESP32.h>
#include <SparkFun_TB6612.h>

/*
 * T6621FNG setup.
 */

// Pins for all inputs, keep in mind the PWM defines must be on PWM pins.
#define AIN1 2
#define AIN2 4
#define PWMA 5
#define BIN1 12
#define BIN2 14
#define PWMB 27
#define STBY 16

class T6621FNG : Motor, GenericMotor {
    public:
        T6621FNG(uint8_t in1, uint8_t in2, uint8_t pwm, float offset, uint8_t stby)
            : Motor(in1, in2, pwm, offset, stby) {};
        ~T6621FNG() {};

        void drive(float speed) override {
            Motor::drive((int)(255 * speed));
        }

        void brake() override {
            Motor::brake();
        }
};

T6621FNG motor1 = T6621FNG(AIN1, AIN2, PWMA, 1, STBY);
T6621FNG motor2 = T6621FNG(BIN1, BIN2, PWMB, 1, STBY);

void setup() {
}

void loop() {
    const float OMEGA = (0.25) * (2*M_PI);

    while (1) {
        float speed = sin(OMEGA * millis() / 1000);
        motor1.drive(speed);
        motor2.drive(speed);
        delay(10);
    }
}
