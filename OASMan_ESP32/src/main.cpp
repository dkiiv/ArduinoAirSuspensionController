// OASMan ESP32

#include <Preferences.h> // have to include it here or it isn't found in the shared libs
#include <user_defines.h>
#include "input_type.h"
#include "components/wheel.h"
#include "components/compressor.h"
#include "components/AuxillaryOutput.h"
#include "bitmaps.h"
#include "manifoldSaveData.h"
#include "airSuspensionUtil.h"
#include "tasks/tasks.h"
#include <directdownload.h>

#include <SPIFFS.h>

extern bool ADS1115D_exists;

// #define FORCE_UPDATE_TEST
void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println(F("Startup!"));

    SPIFFS.begin(true);

    beginSaveData();

    // Check if in update mode and ignore everything else and just start the web server.
    if (getupdateMode())
    {
        setupdateMode(false);
        Serial.println("Gonna try to download update");
        downloadUpdate(getwifiSSID(), getwifiPassword());
        return;
    }

#ifdef FORCE_UPDATE_TEST
    for (int i = 0; i < 10; i++)
    {
        log_i("T-%d", 10 - i);
        delay(1000);
    }
    log_i("TEST updating");
    setupdateMode(true);
    delay(100);
    ESP.restart();
#endif

    // clearPressureData();

    // trainAIModels();

    delay(200); // wait for voltage stabilize

    setupADCReadMutex();
    setupWheelLockSem();

    setupManifold();

    delay(20);

    pressureInputs[0] = pressureSensorInput0;
    pressureInputs[1] = pressureSensorInput1;
    pressureInputs[2] = pressureSensorInput2;
    pressureInputs[3] = pressureSensorInput3;
    if (ADS1115D_exists) {
        pressureInputs[4] = pressureSensorInput4_adsd1115d;
    } else {
        pressureInputs[4] = pressureSensorInput4_directesp32;
    }

    wheel[WHEEL_FRONT_PASSENGER] = new Wheel(FRONT_PASSENGER_IN, FRONT_PASSENGER_OUT, pressureInputs[getpressureInputFrontPassenger()], levelInputFrontPassenger, WHEEL_FRONT_PASSENGER);
    wheel[WHEEL_REAR_PASSENGER] = new Wheel(REAR_PASSENGER_IN, REAR_PASSENGER_OUT, pressureInputs[getpressureInputRearPassenger()], levelInputRearPassenger, WHEEL_REAR_PASSENGER);
    wheel[WHEEL_FRONT_DRIVER] = new Wheel(FRONT_DRIVER_IN, FRONT_DRIVER_OUT, pressureInputs[getpressureInputFrontDriver()], levelInputFrontDriver, WHEEL_FRONT_DRIVER);
    wheel[WHEEL_REAR_DRIVER] = new Wheel(REAR_DRIVER_IN, REAR_DRIVER_OUT, pressureInputs[getpressureInputRearDriver()], levelInputRearDriver, WHEEL_REAR_DRIVER);

    compressor = new Compressor(compressorRelayPin, pressureInputs[getpressureInputTank()]);
    auxillaryOutput = new AuxillaryOutput(auxillaryOutputPin);
    rfReceiver = new RfReceiver();

    if (getlearnPressureSensors())
    {
        setlearnPressureSensors(false);
        PressureSensorCalibration::learnPressureSensorsRoutine();
    }

    accessoryWireSetup();
    ebrakeWireSetup();

    setup_tasks();

#if TEST_MODE == false
    // only want to rise on start if it was a full boot and not a quick reboot
    if (getinternalReboot() == false && getsafetyMode() == false)
    {
        if (getriseOnStart() == true)
        {
            // TODO: make base profile work (look in other spots in app for this)
            // readProfile(getbaseProfile());// TODO: add functionality for this in the controller
            loadProfileAirUp(RIDE_HEIGHT_PRESET_NUMBER);
        }
    }
#endif

    setinternalReboot(false);

    setupLEDs();

    Serial.println(F("Startup Complete"));
}

void loop()
{
    accessoryWireLoop();
    ebrakeWireLoop();
    if (getinternalReboot() == true)
    {
        ESP.restart();
    }

#ifdef DEBUG_HEIGHT_RAW
    // Height-sensor diagnostic (manifold_v4_diag env): prints RAW pre-calibration readings
    // plus the stored per-wheel cal points every ~500ms. raw is mapped 0.5-4.5V -> 0-100
    // and is NOT clamped: >100 or <0 means out-of-range voltage at the ADC pin (floating
    // input drifts high / lost sensor ground), which is electrical, never a cal artifact.
    // CAL min==max (degenerate cal) means the app display is passing raw through unfiltered.
    {
        static const char *const cornerNames[4] = {"FP", "RP", "FD", "RD"};
        static uint8_t dbgDivider = 0;
        if (++dbgDivider >= 5) // loop ticks every 100ms -> print every ~500ms
        {
            dbgDivider = 0;
            Serial.print(F("HRAW"));
            for (int i = 0; i < 4; i++)
            {
                float raw = wheel[i]->readLevelSensorRaw();
                Serial.printf(" %s=%.1f(%.2fV)", cornerNames[i], raw, 0.5f + (raw / 100.0f) * 4.0f);
            }
            Serial.print(F(" CAL"));
            for (int i = 0; i < 4; i++)
            {
                Serial.printf(" %s[%.1f..%.1f]", cornerNames[i], getheightCalMin(i), getheightCalMax(i));
            }
            Serial.print(F(" HM="));
            Serial.println(getheightSensorMode() ? 1 : 0);
        }
    }
#endif

    delay(100);
}
