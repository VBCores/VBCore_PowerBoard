#include "stdbool.h"

#ifdef __cplusplus
extern "C" {
#endif
void load_config();

float get_uvlo_level();
float get_uvlo_hyst();
float get_src_charged_level();
float get_nom_chrg_curr();
bool is_bus_off_while_charging();
bool is_app_running();

#ifdef __cplusplus
}
#endif

// user can prioritize discharging of power source 2 or 3. uncomment the desired source (only one!)
//#define default_prime 0 // the 2 channel
//#define default_prime 1 // the 3 channel
