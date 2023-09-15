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
#include <EKFTask.hpp>
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
    DEBUG_C(Serial.println("\nSetup start!"));

    // set global parameters
    // run it **ONCE** after calibration!
    // Write_global_parameters(11U, 20.0F, -0.03F, 0.0436332F, 0.18F, 39.0518F, 0.0555407F, -43.9161F);
    // Write_global_parameters(12U, 17.0F, 0.05F, 0.0261799F, 0.18F, 26.6286F, 0.00152896F, -5.96417F);
    // Write_global_parameters(13U, 18.5F, -0.01F, -0.0279253F, 0.18F, 25.1755F, 0.027526F, -0.09398F);
    // Write_global_parameters(14U, 18.0F, 0.00F, 0.0087F, 0.18F, 29.7381F, 0.0479783F, -15.9752F);

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

    // target_point[1]=(This_robot_ID == 12) ? 0.0F : -300.0F;
    // target_point[1] = 100.0F;

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

    if (This_robot_ID == 13)
    {
        target_point[1] == 0.0F;
        IR::TX::Init();
        DEBUG_C(Serial.println("TX inited!"));
        // this is the data task that has type 2 and transmit {0x0123}
        // the content is meaningless...
        IR::TX::Add_to_schedule(2, std::vector<uint16_t>{0x0123}, 1, -1, 1);
        DEBUG_C(Serial.println("TX data set!"));
    }
    else
    {
        target_point[1] = -300.0F;
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

    EKF::Init();

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
    // // IR::RX::Add_RX_Notification(motor_test_handle);
    // task_status = (task_status_temp == pdTRUE) ? task_status : pdFALSE;

    // quench LED!
    task_status_temp = xTaskCreatePinnedToCore(
        LED_off_task,
        "LED_off_task",
        5000,
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
        5000,
        NULL,
        8,
        &Buffer_data_handle,
        0);
    task_status = (task_status_temp == pdTRUE) ? task_status : pdFALSE;
    // trigger buffer data when localization is updated.
    IR::Localization::Add_localization_notification(Buffer_data_handle);

    // // buffer raw timing data
    // TaskHandle_t Buffer_raw_data_handle;
    // task_status_temp = xTaskCreatePinnedToCore(
    //     Buffer_raw_data_task,
    //     "Buffer_raw_data_task",
    //     8000,
    //     NULL,
    //     8,
    //     &Buffer_raw_data_handle,
    //     0);
    // task_status = (task_status_temp == pdTRUE) ? task_status : pdFALSE;
    // // trigger buffer data when localization is updated.
    // IR::RX::Add_RX_Notification(Buffer_raw_data_handle);

    // buffer EKF data
    TaskHandle_t Buffer_EKF_handle;
    task_status_temp = xTaskCreatePinnedToCore(
        Buffer_EKF_task,
        "Buffer_EKF_task",
        5000,
        NULL,
        8,
        &Buffer_EKF_handle,
        0);
    task_status = (task_status_temp == pdTRUE) ? task_status : pdFALSE;
    // trigger buffer data when localization is updated.
    EKF::Add_localization_notification(Buffer_EKF_handle);

    // lit LED and control motor based on position
    TaskHandle_t Motor_control_handle_opt;
    task_status_temp = xTaskCreatePinnedToCore(
        Motor_control_task_opt,
        "Motor_control_task_opt",
        5000,
        NULL,
        8,
        &Motor_control_handle_opt,
        0);
    task_status = (task_status_temp == pdTRUE) ? task_status : pdFALSE;
    // trigger Motor_control when localization is updated.
    IR::Localization::Add_localization_notification(Motor_control_handle_opt);

    // lit LED and control motor based on position
    TaskHandle_t Motor_control_handle_EKF;
    task_status_temp = xTaskCreatePinnedToCore(
        Motor_control_task_EKF,
        "Motor_control_task_EKF",
        5000,
        NULL,
        8,
        &Motor_control_handle_EKF,
        0);
    task_status = (task_status_temp == pdTRUE) ? task_status : pdFALSE;
    // trigger Motor_control when localization is updated.
    EKF::Add_localization_notification(Motor_control_handle_EKF);

    // monitor motor's state
    task_status_temp = xTaskCreatePinnedToCore(
        Motor_monitor_task,
        "Motor_monitor_task",
        5000,
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

    // // when to_set_thrust is 1, we set the motor to spd_low, but this is actually when the speed is the highest.
    // // Motor_buffer.push(Motor_info_1{lmt.spd_low, lmt.spd_high, meas_spd[0], meas_spd[1], lmt.thrust_angle, static_cast<uint32_t>(t_now)});
    // EKF::Motor_info_t tempinfo;
    // tempinfo.spd_low = 12;
    // tempinfo.spd_high = 23;
    // tempinfo.spd_FB_low = 130;
    // tempinfo.spd_FB_high = 140;
    // tempinfo.thrust_angle = 0;
    // tempinfo.stop_time = 1024000;
    // EKF::Push_to_motor_buffer(tempinfo);

    // EKF::Notify_Localization_Task();

    // vTaskDelay(pdMS_TO_TICKS(100));
    // float err;
    // auto st = EKF::Get_state(1020000,Robot_mass,&err);
    // Serial.println(st.state[EKF::state_para::x]);
    // Serial.println(st.state[EKF::state_para::y]);
    // Serial.println(st.state[EKF::state_para::z]);
    // Serial.println(err);
    // Serial.println(esp_timer_get_time());

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
