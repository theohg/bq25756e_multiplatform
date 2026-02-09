/**
 * @file basic_charging.ino
 * @brief Basic BQ25756E charger initialization and monitoring example.
 *
 * Demonstrates how to configure the BQ25756E buck-boost battery charger
 * and monitor charge status in a loop.
 */

#include <Wire.h>
#include <bq25756e.h>

// Hardware parameters
#define BQ25756E_ADDR         0x6A   // 7-bit I2C address (default)
#define SWITCHING_FREQ        500    // Switching frequency [kHz]
#define MAX_CHARGE_CURRENT    5000   // Max charge current [mA]
#define MAX_INPUT_CURRENT     5000   // Max input current [mA]
#define MIN_INPUT_VOLTAGE     4200   // Min input voltage [mV]
#define MAX_INPUT_VOLTAGE     36000  // Max input voltage [mV]

BQ25756E charger(BQ25756E_ADDR, SWITCHING_FREQ,
                 MAX_CHARGE_CURRENT, MAX_INPUT_CURRENT,
                 MIN_INPUT_VOLTAGE, MAX_INPUT_VOLTAGE);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    charger.setDebugStream(&Serial);

    Serial.println("Initializing BQ25756E charger...");

    // Build a configuration
    BQ25756E_Config cfg;
    cfg.chargeVoltageLimit        = 1536;   // FB voltage limit [mV] (maps to battery voltage via divider)
    cfg.chargeCurrentLimit        = 2000;   // Fast-charge current [mA]
    cfg.inputCurrentDPMLimit      = 3000;   // Input current DPM [mA]
    cfg.inputVoltageDPMLimit      = 4200;   // Input voltage DPM [mV]
    cfg.prechargeCurrentLimit     = 500;    // Pre-charge current [mA]
    cfg.terminationCurrentLimit   = 250;    // Termination current [mA]
    cfg.terminationControlEnabled = true;
    cfg.fastChargeThreshold       = 0b10;   // 66.7% x VFB_REG
    cfg.prechargeControlEnabled   = true;
    cfg.topOffTimer               = 0b01;   // 15 min
    cfg.watchdogTimer             = 0b00;   // Disabled
    cfg.safetyTimerEnabled        = true;
    cfg.safetyTimer               = 0b10;   // 12 hr
    cfg.safetyTimerSpeed          = false;
    cfg.constantVoltageTimer      = 0b0000; // Disabled
    cfg.autoRechargeThreshold     = 0b00;   // 93% x VFB_REG
    cfg.watchdogTimerResetEnabled = false;
    cfg.CEPinEnabled              = true;
    cfg.ChargeBehaviorWatchdogExpired = false;
    cfg.highZModeEnabled          = false;
    cfg.batteryLoadEnabled        = false;
    cfg.chargeEnabled             = true;
    cfg.enableMPPT                = false;
    cfg.verbose                   = true;

    charger.init(cfg);
    Serial.println("BQ25756E initialized.\n");
}

void loop() {
    // Read charge cycle status
    uint8_t chargeStatus = charger.getChargeCycleStatus();
    Serial.print("Charge status: ");
    switch (chargeStatus) {
        case 0: Serial.println("Not charging");           break;
        case 1: Serial.println("Trickle charge");         break;
        case 2: Serial.println("Pre-charge");             break;
        case 3: Serial.println("Fast charge (CC)");       break;
        case 4: Serial.println("Taper charge (CV)");      break;
        case 6: Serial.println("Top-off timer charge");   break;
        case 7: Serial.println("Charge termination done");break;
        default: Serial.println("Unknown");               break;
    }

    // Read battery voltage
    Serial.print("VBAT: ");
    Serial.print(charger.getVBATADC());
    Serial.println(" mV");

    // Read charge current
    Serial.print("IBAT: ");
    Serial.print(charger.getIBATADC());
    Serial.println(" mA");

    // Read input voltage
    Serial.print("VAC:  ");
    Serial.print(charger.getVACADC());
    Serial.println(" mV");

    // Check faults
    uint8_t faults = charger.getFaultStatus();
    if (faults) {
        Serial.print("FAULT: 0x");
        Serial.println(faults, HEX);
    }

    Serial.println("---");
    delay(2000);
}
