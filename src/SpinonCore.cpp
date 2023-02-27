/**
 * @file TestRMT_1.ino
 * @brief main file
 */
#include <cstring>
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
    DEBUG_C(Serial.println("Setup start!"));
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

    DEBUG_C(Serial.println("Pin setup finished!"));

    Motor::Init();
    Motor::Set_speed(0);
    Motor::Active_brake();

    // LED_set(0, float(0) / float((1 << Motor::PWM_resolution) - 1));

    DEBUG_C(Serial.println("Motor started"));

    // IR::TX::Init();

    // DEBUG_C(Serial.println("TX inited"));

    // // this is the data task that has type 4 and transmit {0x1234, 0xFEDC}
    // IR::TX::Add_to_schedule(4, {0x0123, 0xFEDC}, 2, -1, 2);

    // DEBUG_C(Serial.println("TX data set"));

    IR::RX::Init();

    DEBUG_C(Serial.println("RX inited!"));

    IR::Localization::Init();

    DEBUG_C(Serial.println("Localization inited!"));

    DEBUG_C(Serial.println("Init finished, launching tasks!"));

    BaseType_t task_status = pdTRUE, task_status_temp;

    // // monitor the performance of cores
    // xTaskCreatePinnedToCore(
    //     Idle_stats_task,
    //     "idle0",
    //     8000,
    //     NULL,
    //     1,
    //     NULL,
    //     0);

    // quench LED!
    task_status_temp = xTaskCreatePinnedToCore(
        LED_off_task,
        "LED_off_task",
        8000,
        NULL,
        2,
        NULL,
        0);
    task_status = (task_status_temp == pdTRUE) ? task_status : pdFALSE;

    // // test motor thrust - speed curve
    // TaskHandle_t motor_test_handle;

    // task_status_temp = xTaskCreatePinnedToCore(
    //     Motor_test_task,
    //     "Motor_test_task",
    //     8000,
    //     NULL,
    //     8,
    //     &motor_test_handle,
    //     0);

    // IR::RX::Add_RX_Notification(motor_test_handle);

    // task_status = (task_status_temp == pdTRUE) ? task_status : pdFALSE;

    // // lit LED based on position
    // TaskHandle_t LED_control_handle;

    // LED_PWM_init(7U);

    // task_status = xTaskCreatePinnedToCore(
    //     LED_control_task,
    //     "LED_control_task",
    //     12000,
    //     NULL,
    //     8,
    //     &LED_control_handle,
    //     0);

    // // trigger LED_control_task when localization is updated.
    // IR::Localization::Add_Localization_Notification(LED_control_handle);

    // buffer data when new timing is obtained
    TaskHandle_t Buffer_raw_data_handle;

    task_status_temp = xTaskCreatePinnedToCore(
        Buffer_raw_data_task,
        "Buffer_raw_data_task",
        8000,
        NULL,
        8,
        &Buffer_raw_data_handle,
        0);
    task_status = (task_status_temp == pdTRUE) ? task_status : pdFALSE;

    // trigger buffer data when new timing is obtained.
    IR::RX::Add_RX_Notification(Buffer_raw_data_handle);

    // buffer data when new localization is executed
    TaskHandle_t Buffer_data_handle;

    task_status_temp = xTaskCreatePinnedToCore(
        Buffer_data_task,
        "Buffer_data_task",
        8000,
        NULL,
        8,
        &Buffer_data_handle,
        0);
    task_status = (task_status_temp == pdTRUE) ? task_status : pdFALSE;

    // trigger buffer data when localization is updated.
    IR::Localization::Add_Localization_Notification(Buffer_data_handle);

    // lit LED and control motor based on position
    TaskHandle_t Motor_control_handle;

    LED_PWM_init(3U);

    task_status_temp = xTaskCreatePinnedToCore(
        Motor_control_task,
        "Motor_control_task",
        12000,
        NULL,
        8,
        &Motor_control_handle,
        0);
    task_status = (task_status_temp == pdTRUE) ? task_status : pdFALSE;

    // trigger Motor_control when localization is updated.
    IR::Localization::Add_Localization_Notification(Motor_control_handle);

    // monitor motor's state
    task_status_temp = xTaskCreatePinnedToCore(
        Motor_monitor_task,
        "Motor_monitor_task",
        8000,
        NULL,
        3,
        NULL,
        0);
    task_status = (task_status_temp == pdTRUE) ? task_status : pdFALSE;

    if (task_status == pdTRUE)
    {
        Serial.println("All tasks launched!");
    }
    else
    {
        Serial.println("Task cannot be allocated...");
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
