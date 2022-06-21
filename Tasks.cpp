#include "Tasks.hpp"
#include "RobotDefs.hpp"
#include "FastIO.hpp"

SPIClass *hspi = nullptr;
xSemaphoreHandle HSPI_MUTEX=nullptr;

/**
 * @brief a FreeRTOS queue for storing timing data
 */
extern xQueueHandle RMT_RX_time_queue;

/**
 * @brief a FreeRTOS queue for storing real data
 */
extern xQueueHandle RMT_RX_data_queue;


void Time_comm_task(void *pvParameters)
{
    // buffer
    uint64_t buffer[RMT_RX_CHANNEL_COUNT + 2] = {};
    while (1)
    {
#if RMT_RX_CHANNEL_COUNT
        // check queues
        xQueueReceive(RMT_RX_time_queue, (void *)buffer, portMAX_DELAY);

        // use MUTEX to ensure that only one task can use hspi at a time
        xSemaphoreTake(HSPI_MUTEX, portMAX_DELAY);
        delaylow100ns(HSS_PIN);
        // transfer the data out via spi
        hspi->transfer((uint8_t *)buffer, sizeof(buffer));
        delaylow100ns(HSS_PIN);
        setbit(HSS_PIN);
        xSemaphoreGive(HSPI_MUTEX);

#else
        // if no receiver, just delete the task.
        vTaskDelete(NULL);
#endif
    }
}

void Data_comm_task(void *pvParameters)
{
    // buffer, just 10 bytes for test
    uint8_t buffer[10] = {0};
    while (1)
    {
#if RMT_RX_CHANNEL_COUNT
        // check queues
        xQueueReceive(RMT_RX_data_queue, (void *)buffer, portMAX_DELAY);

        // use MUTEX to ensure that only one task can use hspi at a time
        xSemaphoreTake(HSPI_MUTEX, portMAX_DELAY);
        delaylow100ns(HSS_PIN);
        // transfer the data out via spi
        hspi->transfer((uint8_t *)buffer, sizeof(buffer));
        delaylow100ns(HSS_PIN);
        setbit(HSS_PIN);
        xSemaphoreGive(HSPI_MUTEX);

#else
        // if no receiver, just delete the task.
        vTaskDelete(NULL);
#endif
    }
}
