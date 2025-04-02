#include "stdbool.h"

// Uncomment the following line if using high power board version ( look for HPWR silk )
#define HP_board

#ifndef POWER_DEFAULTS
#define POWER_DEFAULTS
// battery discharged voltage level, Volts
static const float DEFAULT_UVLO_LEVEL = 18.0f;
// battery discharged hysteresis, Volts
static const float DEFAULT_UVLO_HYST = 1.0f;
// battery charged voltage level, Volts
static const float DEFAULT_SRC_CHARGED_LEVEL = 25.2f;
// Nominal charge current, Ampere
static const float DEFAULT_NOMINAL_CHARGE_CURRENT = 3.0f;
#endif

#ifdef __cplusplus
extern "C" {
#endif
void load_config();

float get_uvlo_level();
float get_uvlo_hyst();
float get_src_charged_level();
float get_nom_chrg_curr();
bool is_bus_off_while_charging();

#ifdef __cplusplus
}
#endif

// user can prioritize discharging of power source 2 or 3. uncomment the desired source (only one!)
//#define default_prime 0 // the 2 channel
//#define default_prime 1 // the 3 channel
