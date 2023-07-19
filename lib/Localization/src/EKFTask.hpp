/**
 * @file EKFCore.hpp
 * @brief core for EKF filtering
 */
#ifndef EKFTASK_HPP__
#define EKFTASK_HPP__

#include "Arduino.h"

namespace EKF
{
    // transmitted int * Position_scaling_factor => actual position in cm
    constexpr float Position_scaling_factor = 0.1f;

    /**
     * @brief priority of Localization task, it should be marginally smaller
     * than Preprocess task, but still higher than the rest.
     */
    constexpr uint32_t Localization_task_priority = RX::Preprocess_task_priority - 1;

    struct Motor_info_t
    {
        uint16_t spd_low = 0;  // speed low end
        uint16_t spd_high = 0; // speed high end

        uint32_t spd_FB_low = 0;  // speed feedback low end
        uint32_t spd_FB_high = 0; // speed feedback high end

        float thrust_angle = 0; // at which angle the thrust is high

        int64_t stop_time = 0; // when this motor cycle stops
    };

    /**
     * @brief Notify this task when data is updated!
     *
     * @param handle Task handle to be added
     * @return bool true if successfully added, false if already exists.
     */
    bool Add_Localization_Notification(const TaskHandle_t &handle);

    /**
     * @brief No longer notify this task when data is updated!
     *
     * @param handle Task handle to be removed
     * @return bool true if successfully removed, false if never exist.
     */
    bool Remove_Localization_Notification(const TaskHandle_t &handle);

    /**
     * @brief get handle to localization task
     *
     * @note should only be accessed by the motor task or motor interrupt, because they trigger each localization update.
     */
    TaskHandle_t Get_Localization_Task_Handle();
} // EKF namespace

#endif