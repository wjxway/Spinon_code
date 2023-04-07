#include <cstring>
#include <DebugDefs.hpp>
#include <FastIO.hpp>
#include <IrCommunication.hpp>
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

    IR::RX::Init();

    DEBUG_C(Serial.println("RX inited!"));

    DEBUG_C(Serial.println("Init finished, launching tasks!"));

    BaseType_t task_status = pdTRUE, task_status_temp;

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
