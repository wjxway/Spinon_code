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

        delay(60);

        QUENCH_R;
        QUENCH_G;
        QUENCH_B;

        delay(60);
    }
}

SemaphoreHandle_t init_sem;

void real_setup_core_1(void *pvParameters)
{
    IR::RX::Init();

    // give semaphore to indicate successful execution of RX setup
    xSemaphoreGive(init_sem);
    vTaskDelete(NULL);
}

void real_setup_core_0(void *pvParameters)
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    DEBUG_C(Serial.println("\nSetup start!"));

    // set global parameters
    // run it **ONCE** after calibration!
    // Write_global_parameters(11U, 17.9F, -0.03F, 0.0436332F, 0.18F, 39.0518F, 0.0555407F, -43.9161F);
    // Write_global_parameters(12U, 17.5F, 0.05F, 0.0261799F, 0.18F, 26.6286F, 0.00152896F, -5.96417F);

    // init global parameters
    if (!Init_global_parameters())
    {
        // if init failure, we lit red LED and then loop a random task
        LIT_R;

        xTaskCreatePinnedToCore(
            Idle_stats_task,
            "idle0",
            8000,
            NULL,
            21,
            NULL,
            0);
    }

    target_point[1]=(This_robot_ID == 12) ? -50.0F : -350.0F;
    // target_point[1]=-350.0F;

    DEBUG_C(Serial.println("Global parameters initialized!"));
    DEBUG_C(Serial.print("Robot #"));
    DEBUG_C(Serial.println(This_robot_ID));

    // please keep it here! or the RMT might be buggy!
    delay(500);

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

    blink_led(3);
    LED_PWM_init(2U);

    DEBUG_C(Serial.println("Pin setup finished!"));

    Motor::Init();
    Motor::Set_speed(0);
    Motor::Active_brake();
    DEBUG_C(Serial.println("Motor started!"));

    if (This_robot_ID == 12)
    {
        IR::TX::Init();
        DEBUG_C(Serial.println("TX inited!"));
        // this is the data task that has type 2 and transmit {0x0123}
        // the content is meaningless...
        IR::TX::Add_to_schedule(2, std::vector<uint16_t>{0x0123}, 1, -1, 1);
        DEBUG_C(Serial.println("TX data set!"));
    }

    // launch the RX init task on core 1, lock core 0 init task before
    // proceeding.
    init_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(
        real_setup_core_1,
        "set_core_1",
        10000,
        NULL,
        15,
        NULL,
        1);
    vTaskDelay(10);
    // block till setup on core 1 is finished.
    xSemaphoreTake(init_sem, portMAX_DELAY);
    vSemaphoreDelete(init_sem);

    IR::Localization::Init();
    DEBUG_C(Serial.println("Localization inited!"));

    DEBUG_C(Serial.println("Init finished, launching tasks!"));

    BaseType_t task_status = pdTRUE, task_status_temp;

    // // relay received messages to computer
    // TaskHandle_t Message_relay_task_handle;
    // task_status_temp = xTaskCreatePinnedToCore(
    //     Message_relay_task,
    //     "Message_relay_task",
    //     8000,
    //     NULL,
    //     5,
    //     &Message_relay_task_handle,
    //     0);
    // task_status = (task_status_temp == pdTRUE) ? task_status : pdFALSE;
    // // trigger buffer data when localization is updated.
    // IR::RX::Add_RX_Notification(Message_relay_task_handle);

    // // monitor the performance of cores
    // xTaskCreatePinnedToCore(
    //     Idle_stats_task,
    //     "idle0",
    //     8000,
    //     NULL,
    //     1,
    //     NULL,
    //     0);

    // // monitor the performance of cores
    // xTaskCreatePinnedToCore(
    //     Idle_stats_task,
    //     "idle1",
    //     8000,
    //     NULL,
    //     1,
    //     NULL,
    //     1);

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
        DEBUG_C(Serial.println("All tasks launched!"));
    }
    else
    {
        DEBUG_C(Serial.println("Task cannot be allocated..."));
    }

    // remove this task after use
    vTaskDelete(NULL);
}

void setup()
{
    vTaskDelay(10);

    xTaskCreatePinnedToCore(
        real_setup_core_0,
        "set_core_0",
        10000,
        NULL,
        20,
        NULL,
        0);

    vTaskDelete(NULL);
}

// delete loop() immediately
void loop()
{
    vTaskDelete(NULL);
}
