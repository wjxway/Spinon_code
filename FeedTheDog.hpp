#ifndef _FEEDTHEDOG_HPP_
#define _FEEDTHEDOG_HPP_
#pragma once

#include "Arduino.h"
#include "esp_task_wdt.h"
#include "soc/timer_group_reg.h"
#include "soc/timer_group_struct.h"

//feed dog to prevent reboot
void Feed_the_dog();

#endif