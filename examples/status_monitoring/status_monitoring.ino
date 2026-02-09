/**
 * @file status_monitoring.ino
 * @brief BQ25756E ADC readings and fault monitoring example.
 *
 * Reads all available ADC channels and status registers from the BQ25756E,
 * printing a periodic summary to Serial.
 */

#include <Wire.h>
#include <bq25756e.h>

#define BQ25756E_ADDR         0x6A
#define SWITCHING_FREQ        500
#define MAX_CHARGE_CURRENT    5000
#define MAX_INPUT_CURRENT     5000
#define MIN_INPUT_VOLTAGE     4200
#define MAX_INPUT_VOLTAGE     36000

BQ25756E charger(BQ25756E_ADDR, SWITCHING_FREQ,
                 MAX_CHARGE_CURRENT, MAX_INPUT_CURRENT,
                 MIN_INPUT_VOLTAGE, MAX_INPUT_VOLTAGE);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    charger.setDebugStream(&Serial);

    // Minimal config -- just enable ADC and charging
    BQ25756E_Config cfg;
    cfg.chargeVoltageLimit        = 1536;
    cfg.chargeCurrentLimit        = 2000;
    cfg.inputCurrentDPMLimit      = 3000;
    cfg.inputVoltageDPMLimit      = 4200;
    cfg.prechargeCurrentLimit     = 500;
    cfg.terminationCurrentLimit   = 250;
    cfg.terminationControlEnabled = true;
    cfg.fastChargeThreshold       = 0b10;
    cfg.prechargeControlEnabled   = true;
    cfg.topOffTimer               = 0b00;
    cfg.watchdogTimer             = 0b00;
    cfg.safetyTimerEnabled        = false;
    cfg.safetyTimer               = 0b00;
    cfg.safetyTimerSpeed          = false;
    cfg.constantVoltageTimer      = 0b0000;
    cfg.autoRechargeThreshold     = 0b00;
    cfg.watchdogTimerResetEnabled = false;
    cfg.CEPinEnabled              = true;
    cfg.ChargeBehaviorWatchdogExpired = false;
    cfg.highZModeEnabled          = false;
    cfg.batteryLoadEnabled        = false;
    cfg.chargeEnabled             = true;
    cfg.enableMPPT                = false;
    cfg.verbose                   = false;

    charger.init(cfg);
    Serial.println("BQ25756E status monitor ready.\n");
}

void loop() {
    Serial.println("===== ADC Readings =====");

    Serial.print("IAC  (input current):  ");
    Serial.print(charger.getIACADC());
    Serial.println(" mA");

    Serial.print("IBAT (battery current): ");
    Serial.print(charger.getIBATADC());
    Serial.println(" mA");

    Serial.print("VAC  (input voltage):  ");
    Serial.print(charger.getVACADC());
    Serial.println(" mV");

    Serial.print("VBAT (battery voltage): ");
    Serial.print(charger.getVBATADC());
    Serial.println(" mV");

    Serial.print("VFB  (feedback voltage): ");
    Serial.print(charger.getVFBADC());
    Serial.println(" mV");

    Serial.println("\n===== Status =====");

    Serial.print("Charger Status 1: 0x");
    Serial.println(charger.getChargerStatus1(), HEX);

    Serial.print("Charger Status 2: 0x");
    Serial.println(charger.getChargerStatus2(), HEX);

    Serial.print("Charger Status 3: 0x");
    Serial.println(charger.getChargerStatus3(), HEX);

    uint8_t faults = charger.getFaultStatus();
    Serial.print("Fault Status:     0x");
    Serial.println(faults, HEX);

    if (faults) {
        if (faults & 0x80) Serial.println("  -> VAC Under-Voltage");
        if (faults & 0x40) Serial.println("  -> VAC Over-Voltage");
        if (faults & 0x20) Serial.println("  -> IBAT Over-Current");
        if (faults & 0x10) Serial.println("  -> VBAT Over-Voltage");
        if (faults & 0x08) Serial.println("  -> Thermal Shutdown");
        if (faults & 0x04) Serial.println("  -> Safety Timer Expired");
        if (faults & 0x02) Serial.println("  -> DRV_SUP Pin Fault");
    }

    Serial.print("Part Info: 0x");
    Serial.println(charger.getPartInformation(), HEX);

    Serial.println("\n");
    delay(3000);
}
