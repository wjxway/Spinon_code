/**
 * @file TestRMT_1.ino
 * @brief main file
 */
#include <string>

#include <DebugDefs.hpp>
#include <FastIO.hpp>
#include <MotorCtrl.hpp>
#include <IrCommunication.hpp>
#include <Localization.hpp>
#include "Tasks.hpp"

uint64_t rec_finish_time = 0;

// Blink the LED
void blink_led(int n)
{
    for (size_t i = 0; i < n; i++)
    {
        LIT_R;
        LIT_G;
        LIT_B;

        delay(100);

        QUENCH_R;
        QUENCH_G;
        QUENCH_B;

        delay(100);
    }
}

void real_setup(void *pvParameters)
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    DEBUG_C(Serial.println("Setup start"));
    DEBUG_C(Serial.print("Robot #"));
    DEBUG_C(Serial.println(This_robot_ID));

    // please keep it here! or the RMT might be buggy!
    delay(10);

    // test output
    pinMode(DEBUG_PIN_1, OUTPUT);
    pinMode(DEBUG_PIN_2, OUTPUT);

    // LED output
    pinMode(LED_PIN_R, OUTPUT);
    pinMode(LED_PIN_G, OUTPUT);
    pinMode(LED_PIN_B, OUTPUT);

    digitalWrite(LED_PIN_R, HIGH);
    digitalWrite(LED_PIN_G, HIGH);
    digitalWrite(LED_PIN_B, HIGH);

    blink_led(5);

    DEBUG_C(Serial.println("Pin setup finished"));

    // LED_PWM_init();

    // // only setup first channel
    // ledcSetup(1, 40000, 10);
    // ledcAttachPin(LED_PIN_R, 1);
    // ledcWrite(1, (1 << 10) - 1);

    // Motor::Init();
    // Motor::Set_speed(0);
    // LED_set(0, float(0) / float((1 << Motor::PWM_resolution) - 1));

    // DEBUG_C(Serial.println("Motor started"));

    // IR::TX::Init();

    // DEBUG_C(Serial.println("TX inited"));

    // // this is the data task that has type 4 and transmit {0x1234, 0xFEDC}
    // IR::TX::Add_to_schedule(4, {0x0123, 0xFEDC}, 2, -1, 2);

    // DEBUG_C(Serial.println("TX data set"));

    IR::RX::Init();

    DEBUG_C(Serial.println("RX inited"));

    IR::Localization::Init();
    DEBUG_C(Serial.println("Localization inited"));

    DEBUG_C(Serial.println("Init finished, launching tasks!"));

    // // monitor the performance of cores
    // xTaskCreatePinnedToCore(
    //     Idle_stats_task,
    //     "idle0",
    //     10000,
    //     NULL,
    //     1,
    //     NULL,
    //     0);

    // quench LED!
    xTaskCreatePinnedToCore(
        LED_off_task,
        "LED_off_task",
        10000,
        NULL,
        2,
        NULL,
        0);

    // Light LED based on position
    TaskHandle_t LED_control_handle;

    auto task_status = xTaskCreatePinnedToCore(
        LED_control_task,
        "LED_control_task",
        50000,
        NULL,
        8,
        &LED_control_handle,
        0);

    // trigger LED_control_task when localization is updated.
    IR::Localization::Add_Localization_Notification(LED_control_handle);

    // // turn on motor based on signal
    // TaskHandle_t Motor_test_handle;

    // auto task_status = xTaskCreatePinnedToCore(
    //     Motor_test_task,
    //     "Motor_test_task",
    //     50000,
    //     NULL,
    //     8,
    //     &Motor_test_handle,
    //     0);

    // IR::RX::Add_RX_Notification(Motor_test_handle);

    // // buffer data when new localization is executed
    // TaskHandle_t Buffer_data_handle;

    // auto task_status = xTaskCreatePinnedToCore(
    //     Buffer_data_task,
    //     "Buffer_data_task",
    //     50000,
    //     NULL,
    //     8,
    //     &Buffer_data_handle,
    //     0);

    // // trigger buffer data when localization is updated.
    // IR::Localization::Add_Localization_Notification(Buffer_data_handle);

    if (task_status == pdTRUE)
    {
        Serial.println("All tasks launched!");
    }
    else
    {
        Serial.println("Task cannot be allocated!");
    }

    // remove this task after use
    vTaskDelete(NULL);
}

void setup()
{
    xTaskCreatePinnedToCore(
        real_setup,
        "setup_task",
        10000,
        NULL,
        20,
        NULL,
        0);
}

// delete loop() immediately
void loop()
{
    vTaskDelete(NULL);
}
