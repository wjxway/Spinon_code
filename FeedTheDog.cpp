#include "FeedTheDog.h"

void FeedTheDog()
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