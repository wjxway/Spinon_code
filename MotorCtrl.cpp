#include "MotorCtrl.hpp"
#include "Utilities/FastIO.hpp"
#include "Utilities/PinDefs.hpp"
#include "Utilities/DebugDefs.hpp"
#include <Wire.h>

uint8_t Motor::Init()
{
    // pin modes
    pinMode(MOTOR_ALERT_PIN, INPUT_PULLUP);
    pinMode(MOTOR_SPD_FB_PIN, INPUT_PULLUP);
    pinMode(MOTOR_SPD_CTRL_PIN, OUTPUT);
    pinMode(MOTOR_BRAKE_PIN, OUTPUT);

    // default state
    clrbit(MOTOR_BRAKE_PIN);
    clrbit(MOTOR_SPD_CTRL_PIN);

    // i2c start
    Wire.begin(MOTOR_SDA_PIN, MOTOR_SCL_PIN, 200000);

    // check if successful
    uint8_t temp = 1;
    // initial config
    for (uint8_t addr = 0; addr < 23; addr++)
    {
        temp &= Config_register(addr + 2, Default_config[addr]);
    }

    // ledc start
    ledcSetup(Motor_LEDC_PWM_channel, PWM_frequency, PWM_resolution);
    ledcAttachPin(MOTOR_SPD_CTRL_PIN, Motor_LEDC_PWM_channel);
    ledcWrite(Motor_LEDC_PWM_channel, 0);

    return temp;
}

uint8_t Motor::Config_register(uint8_t address, uint8_t value)
{
    uint8_t state = 0;

    // write i2c device address
    Wire.beginTransmission(Motor_address);
    // write register address
    Wire.write(address);
    // write data
    Wire.write(value);
    // end transmission
    state = Wire.endTransmission();

    // not sure whether necessary, but let's add it anyways
    delayMicroseconds(100);

    return state;
}

void Motor::Set_speed(uint32_t duty)
{
    Last_set_speed = duty;
    ledcWrite(Motor_LEDC_PWM_channel, duty);
}

uint32_t Motor::Measure_speed()
{
    // set a time out (in us) or else it will freeze the core when at 0 speed.
    constexpr unsigned long time_out = 5000;

    // starting time
    unsigned long t_start = micros();
    // wait for the pin to go low
    while (fastread(MOTOR_SPD_FB_PIN) && (micros() - t_start < time_out))
    {
    }
    // wait for the pin to go high
    while ((!fastread(MOTOR_SPD_FB_PIN)) && (micros() - t_start < time_out))
    {
    }
    // directly return if timed out
    if (micros() - t_start > time_out)
        return 0;
    
    // start recording time
    t_start = micros();
    // wait for the pin to go high
    while (fastread(MOTOR_SPD_FB_PIN) && (micros() - t_start < time_out))
    {
    }

    // save time frame
    t_start=micros()-t_start;

    // return 0 if timed out
    if (t_start > time_out)
        return 0;
    // if not, compute the rotation speed in rps using the pulse width
    else
        return 4000000u/t_start;
}

void Motor::Active_brake()
{
    Last_set_speed = 0;
    setbit(MOTOR_BRAKE_PIN);
}

void Motor::Active_brake_release()
{
    clrbit(MOTOR_BRAKE_PIN);
}

void IRAM_ATTR Motor::Alert_ISR(void *arg)
{
    Active_brake();
}