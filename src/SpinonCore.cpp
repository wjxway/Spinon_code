#include <cstring>
#include <DebugDefs.hpp>
#include <FastIO.hpp>
#include <IrCommunication.hpp>
#include "Tasks.hpp"


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

    blink_led(3);

    DEBUG_C(Serial.println("Pin setup finished!"));

#if DRONE_MODE == 0
    Serial.println("Calibration mode!");
#else
    Serial.println("Relay mode!");
#endif

#if DRONE_MODE == 0
    IR::TX::Init();
    DEBUG_C(Serial.println("TX inited!"));
    // this is the data task that has type 2 and transmit {0x0123}
    // the content is meaningless...
    IR::TX::Add_to_schedule(4, std::vector<uint16_t>{0x0123}, 1);
    DEBUG_C(Serial.println("TX data set!"));
#endif

    // launch the RX init task on core 1, lock core 0 init task before
    // proceeding.
    init_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(
        real_setup_core_1,
        "set_core_1",
        10000,
        NULL,
        20,
        NULL,
        1);
    vTaskDelay(10);
    // block till setup on core 1 is finished.
    xSemaphoreTake(init_sem, portMAX_DELAY);
    vSemaphoreDelete(init_sem);

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

#if DRONE_MODE == 0
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
#endif

#if DRONE_MODE == 1
    // buffer data when new timing is obtained
    TaskHandle_t Message_relay_handle;

    task_status_temp = xTaskCreatePinnedToCore(
        Message_relay_task,
        "Message_relay_task",
        8000,
        NULL,
        8,
        &Message_relay_handle,
        0);
    task_status = (task_status_temp == pdTRUE) ? task_status : pdFALSE;

    // trigger buffer data when new timing is obtained.
    IR::RX::Add_RX_Notification(Message_relay_handle);
#endif

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
        15,
        NULL,
        0);

    vTaskDelete(NULL);
}

// delete loop() immediately
void loop()
{
    vTaskDelete(NULL);
}
