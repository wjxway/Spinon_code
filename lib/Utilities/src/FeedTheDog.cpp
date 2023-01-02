#include "Arduino.h"
#include "FeedTheDog.hpp"
#include "esp_task_wdt.h"
#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"

void Feed_the_dog()
{
    //feed dog 0
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE; //write enable
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
    //feed dog 1
    TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE; //write enable
    TIMERG1.wdt_feed = 1;
    TIMERG1.wdt_wprotect = 0;
}