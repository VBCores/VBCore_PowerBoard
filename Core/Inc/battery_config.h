// Uncomment the following line if using high power board version ( look for HPWR silk )
//#define HP_board

// Define to automatically disable power bus when charging device detected. Otherwise user controls power bus
#define bus_off_charging

#define undervoltage_lockout_voltage            18.0f   // Volts
#define undervoltage_lockout_hysteresis         1.0f    // Volts
#define charged_battery_voltage                 25.2f   // Volts
#define nominal_charge_current                  3.0f    // Amps

// user can prioritize discharging of power source 2 or 3. uncomment the desired source (only one!)
//#define default_prime 0 // the 2 channel
//#define default_prime 1 // the 3 channel