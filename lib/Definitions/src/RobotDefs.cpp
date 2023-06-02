#include "RobotDefs.hpp"
#include "Arduino.h"
#include <Preferences.h>

#if DEBUG_ENABLED
#define DEBUG_print(c) Serial.print(c)
#define DEBUG_println(c) Serial.println(c)
#else
#define DEBUG_print(c) 0
#define DEBUG_println(c) 0
#endif

#define Read_NVM(type, var, nvmvar)            \
    if (!Global_Prefs.isKey(#nvmvar))          \
    {                                          \
        Global_Prefs.end();                    \
        DEBUG_print(#var);                     \
        DEBUG_println(" is not initialized!"); \
        return false;                          \
    }                                          \
    var = Global_Prefs.get##type(#nvmvar);

#define Write_NVM(type, var, nvmvar)                \
    if (!Global_Prefs.put##type(#nvmvar, nvmvar))   \
    {                                               \
        Global_Prefs.end();                         \
        Serial.print(#var);                         \
        Serial.println(" storage failure!");        \
        return false;                               \
    }                                               \
    var = nvmvar;                                   \
    Serial.print(#var);                             \
    Serial.print(" has been successfully set to "); \
    Serial.println(nvmvar);

uint32_t This_robot_ID = 0U;
float Robot_mass = 17.5F;

namespace IR
{
    namespace RX
    {
        float LR_angle_compensation = 0.0F;
        float Elevation_angle_compensation = 0.0F;
        float Orientation_compensation = 0.18F;

        float Distance_param_a = 35.0F;
        float Distance_param_b = 0.0F;
        float Distance_param_c = -30.0F;
    } // namespace RX
} // namespace IR

bool Init_global_parameters()
{
    // open NVM namespace
    Preferences Global_Prefs;
    if (!Global_Prefs.begin("Global_Prefs", true))
    {
        DEBUG_println("namespace open failure!");
        return false;
    }

    // read values
    Read_NVM(UInt,This_robot_ID, TRID);
    Read_NVM(Float,Robot_mass, Rmass);
    Read_NVM(Float,IR::RX::LR_angle_compensation, LRAcomp);
    Read_NVM(Float,IR::RX::Elevation_angle_compensation, EAcomp);
    Read_NVM(Float,IR::RX::Orientation_compensation, Ocomp);
    Read_NVM(Float,IR::RX::Distance_param_a, DparamA);
    Read_NVM(Float,IR::RX::Distance_param_b, DparamB);
    Read_NVM(Float,IR::RX::Distance_param_c, DparamC);

    Global_Prefs.end();
    return true;
}

bool Write_global_parameters(uint32_t TRID, float Rmass, float LRAcomp, float EAcomp, float Ocomp, float DparamA, float DparamB, float DparamC)
{
    // open NVM namespace
    Preferences Global_Prefs;
    if (!Global_Prefs.begin("Global_Prefs", false))
    {
        Global_Prefs.end();
        Serial.println("namespace open/creation failure!");
        return false;
    }

    // read values
    Write_NVM(UInt,This_robot_ID, TRID);
    Write_NVM(Float,Robot_mass, Rmass);
    Write_NVM(Float,IR::RX::LR_angle_compensation, LRAcomp);
    Write_NVM(Float,IR::RX::Elevation_angle_compensation, EAcomp);
    Write_NVM(Float,IR::RX::Orientation_compensation, Ocomp);
    Write_NVM(Float,IR::RX::Distance_param_a, DparamA);
    Write_NVM(Float,IR::RX::Distance_param_b, DparamB);
    Write_NVM(Float,IR::RX::Distance_param_c, DparamC);

    Global_Prefs.end();
    return true;
}

#undef DEBUG_print
#undef DEBUG_println
#undef Read_NVM
#undef Write_NVM