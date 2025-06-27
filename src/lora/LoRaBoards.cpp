/**
 * @file      boards.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  ShenZhen XinYuan Electronic Technology Co., Ltd
 * @date      2024-04-24
 * @last-update 2025-05-26
 *
 */

#include "LoRaBoards.h"

#include "soc/rtc.h"
#ifdef ENABLE_BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#endif

#if defined(ARDUINO_ARCH_ESP32)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "hal/gpio_hal.h"
#endif
#include "driver/gpio.h"
#endif // ARDUINO_ARCH_ESP32

static DevInfo_t devInfo;

#ifdef HAS_GPS
static bool find_gps = false;
String gps_model = "None";
#endif

uint32_t deviceOnline = 0x00;

#ifdef HAS_PMU
XPowersLibInterface *PMU = NULL;
bool pmuInterrupt;

static void setPmuFlag()
{
    pmuInterrupt = true;
}
#endif

static void enable_slow_clock();

bool beginPower()
{
#ifdef HAS_PMU
    if (!PMU)
    {
        PMU = new XPowersAXP2101(PMU_WIRE_PORT);
        if (!PMU->init())
        {
            Serial.println("Warning: Failed to find AXP2101 power management");
            delete PMU;
            PMU = NULL;
        }
        else
        {
            Serial.println("AXP2101 PMU init succeeded, using AXP2101 PMU");
        }
    }

    if (!PMU)
    {
        PMU = new XPowersAXP192(PMU_WIRE_PORT);
        if (!PMU->init())
        {
            Serial.println("Warning: Failed to find AXP192 power management");
            delete PMU;
            PMU = NULL;
        }
        else
        {
            Serial.println("AXP192 PMU init succeeded, using AXP192 PMU");
        }
    }

    if (!PMU)
    {
        return false;
    }

    deviceOnline |= POWERMANAGE_ONLINE;

    PMU->setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);

    pinMode(PMU_IRQ, INPUT_PULLUP);
    attachInterrupt(PMU_IRQ, setPmuFlag, FALLING);

    if (PMU->getChipModel() == XPOWERS_AXP192)
    {

        PMU->setProtectedChannel(XPOWERS_DCDC3);

        // lora
        PMU->setPowerChannelVoltage(XPOWERS_LDO2, 3300);
        // gps
        PMU->setPowerChannelVoltage(XPOWERS_LDO3, 3300);
        // oled
        PMU->setPowerChannelVoltage(XPOWERS_DCDC1, 3300);

        PMU->enablePowerOutput(XPOWERS_LDO2);
        PMU->enablePowerOutput(XPOWERS_LDO3);

        // protected oled power source
        PMU->setProtectedChannel(XPOWERS_DCDC1);
        // protected esp32 power source
        PMU->setProtectedChannel(XPOWERS_DCDC3);
        // enable oled power
        PMU->enablePowerOutput(XPOWERS_DCDC1);

        // disable not use channel
        PMU->disablePowerOutput(XPOWERS_DCDC2);

        PMU->disableIRQ(XPOWERS_AXP192_ALL_IRQ);

        PMU->enableIRQ(XPOWERS_AXP192_VBUS_REMOVE_IRQ |
                       XPOWERS_AXP192_VBUS_INSERT_IRQ |
                       XPOWERS_AXP192_BAT_CHG_DONE_IRQ |
                       XPOWERS_AXP192_BAT_CHG_START_IRQ |
                       XPOWERS_AXP192_BAT_REMOVE_IRQ |
                       XPOWERS_AXP192_BAT_INSERT_IRQ |
                       XPOWERS_AXP192_PKEY_SHORT_IRQ);
    }
    else if (PMU->getChipModel() == XPOWERS_AXP2101)
    {

#if defined(CONFIG_IDF_TARGET_ESP32)
        // Unuse power channel
        PMU->disablePowerOutput(XPOWERS_DCDC2);
        PMU->disablePowerOutput(XPOWERS_DCDC3);
        PMU->disablePowerOutput(XPOWERS_DCDC4);
        PMU->disablePowerOutput(XPOWERS_DCDC5);
        PMU->disablePowerOutput(XPOWERS_ALDO1);
        PMU->disablePowerOutput(XPOWERS_ALDO4);
        PMU->disablePowerOutput(XPOWERS_BLDO1);
        PMU->disablePowerOutput(XPOWERS_BLDO2);
        PMU->disablePowerOutput(XPOWERS_DLDO1);
        PMU->disablePowerOutput(XPOWERS_DLDO2);
        PMU->disablePowerOutput(XPOWERS_CPULDO);

        // GNSS RTC PowerVDD 3300mV
        PMU->setPowerChannelVoltage(XPOWERS_VBACKUP, 3300);
        PMU->enablePowerOutput(XPOWERS_VBACKUP);

        // ESP32 VDD 3300mV
        //  ! No need to set, automatically open , Don't close it
        //  PMU->setPowerChannelVoltage(XPOWERS_DCDC1, 3300);
        //  PMU->setProtectedChannel(XPOWERS_DCDC1);
        PMU->setProtectedChannel(XPOWERS_DCDC1);

        // LoRa VDD 3300mV
        PMU->setPowerChannelVoltage(XPOWERS_ALDO2, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO2);

        // GNSS VDD 3300mV
        PMU->setPowerChannelVoltage(XPOWERS_ALDO3, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO3);

#endif /*CONFIG_IDF_TARGET_ESP32*/

        // Set constant current charge current limit
        PMU->setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);

        // Set charge cut-off voltage
        PMU->setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);

        // Disable all interrupts
        PMU->disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
        // Clear all interrupt flags
        PMU->clearIrqStatus();
        // Enable the required interrupt function
        PMU->enableIRQ(
            XPOWERS_AXP2101_BAT_INSERT_IRQ | XPOWERS_AXP2101_BAT_REMOVE_IRQ |    // BATTERY
            XPOWERS_AXP2101_VBUS_INSERT_IRQ | XPOWERS_AXP2101_VBUS_REMOVE_IRQ |  // VBUS
            XPOWERS_AXP2101_PKEY_SHORT_IRQ | XPOWERS_AXP2101_PKEY_LONG_IRQ |     // POWER KEY
            XPOWERS_AXP2101_BAT_CHG_DONE_IRQ | XPOWERS_AXP2101_BAT_CHG_START_IRQ // CHARGE
            // XPOWERS_AXP2101_PKEY_NEGATIVE_IRQ | XPOWERS_AXP2101_PKEY_POSITIVE_IRQ   |   //POWER KEY
        );
    }

    PMU->enableSystemVoltageMeasure();
    PMU->enableVbusVoltageMeasure();
    PMU->enableBattVoltageMeasure();

    Serial.printf("=========================================\n");
    if (PMU->isChannelAvailable(XPOWERS_DCDC1))
    {
        Serial.printf("DC1  : %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_DCDC1) ? "+" : "-", PMU->getPowerChannelVoltage(XPOWERS_DCDC1));
    }
    if (PMU->isChannelAvailable(XPOWERS_DCDC2))
    {
        Serial.printf("DC2  : %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_DCDC2) ? "+" : "-", PMU->getPowerChannelVoltage(XPOWERS_DCDC2));
    }
    if (PMU->isChannelAvailable(XPOWERS_DCDC3))
    {
        Serial.printf("DC3  : %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_DCDC3) ? "+" : "-", PMU->getPowerChannelVoltage(XPOWERS_DCDC3));
    }
    if (PMU->isChannelAvailable(XPOWERS_DCDC4))
    {
        Serial.printf("DC4  : %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_DCDC4) ? "+" : "-", PMU->getPowerChannelVoltage(XPOWERS_DCDC4));
    }
    if (PMU->isChannelAvailable(XPOWERS_DCDC5))
    {
        Serial.printf("DC5  : %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_DCDC5) ? "+" : "-", PMU->getPowerChannelVoltage(XPOWERS_DCDC5));
    }
    if (PMU->isChannelAvailable(XPOWERS_LDO2))
    {
        Serial.printf("LDO2 : %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_LDO2) ? "+" : "-", PMU->getPowerChannelVoltage(XPOWERS_LDO2));
    }
    if (PMU->isChannelAvailable(XPOWERS_LDO3))
    {
        Serial.printf("LDO3 : %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_LDO3) ? "+" : "-", PMU->getPowerChannelVoltage(XPOWERS_LDO3));
    }
    if (PMU->isChannelAvailable(XPOWERS_ALDO1))
    {
        Serial.printf("ALDO1: %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_ALDO1) ? "+" : "-", PMU->getPowerChannelVoltage(XPOWERS_ALDO1));
    }
    if (PMU->isChannelAvailable(XPOWERS_ALDO2))
    {
        Serial.printf("ALDO2: %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_ALDO2) ? "+" : "-", PMU->getPowerChannelVoltage(XPOWERS_ALDO2));
    }
    if (PMU->isChannelAvailable(XPOWERS_ALDO3))
    {
        Serial.printf("ALDO3: %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_ALDO3) ? "+" : "-", PMU->getPowerChannelVoltage(XPOWERS_ALDO3));
    }
    if (PMU->isChannelAvailable(XPOWERS_ALDO4))
    {
        Serial.printf("ALDO4: %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_ALDO4) ? "+" : "-", PMU->getPowerChannelVoltage(XPOWERS_ALDO4));
    }
    if (PMU->isChannelAvailable(XPOWERS_BLDO1))
    {
        Serial.printf("BLDO1: %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_BLDO1) ? "+" : "-", PMU->getPowerChannelVoltage(XPOWERS_BLDO1));
    }
    if (PMU->isChannelAvailable(XPOWERS_BLDO2))
    {
        Serial.printf("BLDO2: %s   Voltage: %04u mV \n", PMU->isPowerChannelEnable(XPOWERS_BLDO2) ? "+" : "-", PMU->getPowerChannelVoltage(XPOWERS_BLDO2));
    }
    Serial.printf("=========================================\n");

    // Set the time of pressing the button to turn off
    PMU->setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
    uint8_t opt = PMU->getPowerKeyPressOffTime();
    Serial.print("PowerKeyPressOffTime:");
    switch (opt)
    {
    case XPOWERS_POWEROFF_4S:
        Serial.println("4 Second");
        break;
    case XPOWERS_POWEROFF_6S:
        Serial.println("6 Second");
        break;
    case XPOWERS_POWEROFF_8S:
        Serial.println("8 Second");
        break;
    case XPOWERS_POWEROFF_10S:
        Serial.println("10 Second");
        break;
    default:
        break;
    }
#endif
    return true;
}

void disablePeripherals()
{

#ifdef HAS_PMU
    if (!PMU)
        return;

    PMU->setChargingLedMode(XPOWERS_CHG_LED_OFF);
    // Disable the PMU measurement section
    PMU->disableSystemVoltageMeasure();
    PMU->disableVbusVoltageMeasure();
    PMU->disableBattVoltageMeasure();
    PMU->disableTemperatureMeasure();
    PMU->disableBattDetection();

    if (PMU->getChipModel() == XPOWERS_AXP2101)
    {

        // Disable all PMU interrupts
        PMU->disableIRQ(XPOWERS_AXP2101_ALL_IRQ);
        // Clear the PMU interrupt status before sleeping, otherwise the sleep current will increase
        PMU->clearIrqStatus();
        // GNSS RTC Power , Turning off GPS backup voltage and current can further reduce ~ 100 uA
        PMU->disablePowerOutput(XPOWERS_VBACKUP);
        // LoRa VDD
        PMU->disablePowerOutput(XPOWERS_ALDO2);
        // GNSS VDD
        PMU->disablePowerOutput(XPOWERS_ALDO3);
    }
    else if (PMU->getChipModel() == XPOWERS_AXP192)
    {

        // Disable all PMU interrupts
        PMU->disableIRQ(XPOWERS_AXP192_ALL_IRQ);
        // Clear the PMU interrupt status before sleeping, otherwise the sleep current will increase
        PMU->clearIrqStatus();
        // LoRa VDD
        PMU->disablePowerOutput(XPOWERS_LDO2);
        // GNSS VDD
        PMU->disablePowerOutput(XPOWERS_LDO3);
    }
#endif
}

void loopPMU(void (*pressed_cb)(void))
{
#ifdef HAS_PMU
    if (!PMU)
    {
        return;
    }
    if (!pmuInterrupt)
    {
        return;
    }

    pmuInterrupt = false;
    // Get PMU Interrupt Status Register
    uint32_t status = PMU->getIrqStatus();
    Serial.print("STATUS => HEX:");
    Serial.print(status, HEX);
    Serial.print(" BIN:");
    Serial.println(status, BIN);

    if (PMU->isVbusInsertIrq())
    {
        Serial.println("isVbusInsert");
    }
    if (PMU->isVbusRemoveIrq())
    {
        Serial.println("isVbusRemove");
    }
    if (PMU->isBatInsertIrq())
    {
        Serial.println("isBatInsert");
    }
    if (PMU->isBatRemoveIrq())
    {
        Serial.println("isBatRemove");
    }
    if (PMU->isPekeyShortPressIrq())
    {
        Serial.println("isPekeyShortPress");
        if (pressed_cb)
        {
            pressed_cb();
        }
    }
    if (PMU->isPekeyLongPressIrq())
    {
        Serial.println("isPekeyLongPress");
    }
    if (PMU->isBatChargeDoneIrq())
    {
        Serial.println("isBatChargeDone");
    }
    if (PMU->isBatChargeStartIrq())
    {
        Serial.println("isBatChargeStart");
    }
    // Clear PMU Interrupt Status Register
    PMU->clearIrqStatus();
#endif
}

void beginWiFi()
{
#ifdef ARDUINO_ARCH_ESP32
    if (!WiFi.softAP(BOARD_VARIANT_NAME))
    {
        log_e("Soft AP creation failed.");
    }
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
#endif
}

void printWakeupReason()
{
#ifdef ARDUINO_ARCH_ESP32
    Serial.print("Reset reason:");
    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();
    switch (wakeup_reason)
    {
    case ESP_SLEEP_WAKEUP_UNDEFINED:
        Serial.println(" In case of deep sleep, reset was not caused by exit from deep sleep");
        break;
    case ESP_SLEEP_WAKEUP_ALL:
        break;
    case ESP_SLEEP_WAKEUP_EXT0:
        Serial.println("Wakeup caused by external signal using RTC_IO");
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
        Serial.println("Wakeup caused by external signal using RTC_CNTL");
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        Serial.println("Wakeup caused by timer");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        Serial.println("Wakeup caused by touchpad");
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        Serial.println("Wakeup caused by ULP program");
        break;
    default:
        Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
        break;
    }
#endif
}

void getChipInfo()
{
    Serial.println("-----------------------------------");

    printWakeupReason();

    if (psramFound())
    {
        uint32_t psram = ESP.getPsramSize();
        devInfo.psramSize = psram / 1024.0 / 1024.0;
        Serial.printf("PSRAM is enable! PSRAM: %.2fMB\n", devInfo.psramSize);
        deviceOnline |= PSRAM_ONLINE;
    }
    else
    {
        Serial.println("PSRAM is disable!");
        devInfo.psramSize = 0;
    }

    Serial.print("Flash:");
    devInfo.flashSize = ESP.getFlashChipSize() / 1024.0 / 1024.0;
    devInfo.flashSpeed = ESP.getFlashChipSpeed() / 1000 / 1000;
    devInfo.chipModel = ESP.getChipModel();
    devInfo.chipModelRev = ESP.getChipRevision();
    devInfo.chipFreq = ESP.getCpuFreqMHz();

    Serial.print(devInfo.flashSize);
    Serial.println(" MB");
    Serial.print("Flash speed:");
    Serial.print(devInfo.flashSpeed);
    Serial.println(" M");
    Serial.print("Model:");

    Serial.println(devInfo.chipModel);
    Serial.print("Chip Revision:");
    Serial.println(devInfo.chipModelRev);
    Serial.print("Freq:");
    Serial.print(devInfo.chipFreq);
    Serial.println(" MHZ");
    Serial.print("SDK Ver:");
    Serial.println(ESP.getSdkVersion());
    Serial.print("DATE:");
    Serial.println(__DATE__);
    Serial.print("TIME:");
    Serial.println(__TIME__);

    uint8_t mac[6];
    char macStr[18] = {0};
    esp_efuse_mac_get_default(mac);
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.print("EFUSE MAC: ");
    Serial.print(macStr);
    Serial.println();

    Serial.println("-----------------------------------");
}

void setupBoards(bool disable_u8g2)
{
    Serial.begin(115200);

    // while (!Serial);

    Serial.println("setupBoards");

    getChipInfo();

    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);

    SerialGPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

#ifdef BOARD_LED
    /*
     * T-Beam LED defaults to low level as turn on,
     * so it needs to be forced to pull up
     * * * * */
#if LED_ON == LOW
#if defined(ARDUINO_ARCH_ESP32)
    gpio_hold_dis((gpio_num_t)BOARD_LED);
#endif // ARDUINO_ARCH_ESP32
#endif

    pinMode(BOARD_LED, OUTPUT);
    digitalWrite(BOARD_LED, LED_ON);
#endif

#ifdef RADIO_DIO2_PIN
    pinMode(RADIO_DIO2_PIN, INPUT);
#endif

    beginPower();

    // Perform an I2C scan after power-on operation
#ifdef I2C_SDA
    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.println("Scan Wire...");
    scanDevices(&Wire);
#endif

    // scanWiFi();

    // beginWiFi();

#ifdef HAS_GPS

    uint32_t baudrate[] = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 4800};
    if (!find_gps)
    {
        // Restore factory settings
        for (int i = 0; i < sizeof(baudrate) / sizeof(baudrate[0]); ++i)
        {
            Serial.printf("Update baudrate : %u\n", baudrate[i]);
            SerialGPS.updateBaudRate(baudrate[i]);
            if (recoveryGPS())
            {
                Serial.println("UBlox GNSS init succeeded, using UBlox GNSS Module\n");
                gps_model = "UBlox";
                find_gps = true;
                break;
            }
        }
    }
    else
    {
        gps_model = "L76K";
    }

    if (find_gps)
    {
        deviceOnline |= GPS_ONLINE;
    }

#endif
    Serial.println("init done . ");
}

void printResult(bool radio_online)
{
    Serial.print("Radio        : ");
    Serial.println((radio_online) ? "+" : "-");

#if defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S3)

    Serial.print("PSRAM        : ");
    Serial.println((psramFound()) ? "+" : "-");

#ifdef HAS_PMU
    Serial.print("Power        : ");
    Serial.println((PMU) ? "+" : "-");
#endif

#ifdef HAS_GPS
    Serial.print("GPS          : ");
    Serial.println((find_gps) ? "+" : "-");
#endif

#endif
}

#ifdef BOARD_LED
static uint8_t ledState = LOW;
static const uint32_t debounceDelay = 50;
static uint32_t lastDebounceTime = 0;
#endif

void flashLed()
{
#ifdef BOARD_LED
    if ((millis() - lastDebounceTime) > debounceDelay)
    {
        ledState = !ledState;
        if (ledState)
        {
            digitalWrite(BOARD_LED, LED_ON);
        }
        else
        {
            digitalWrite(BOARD_LED, !LED_ON);
        }
        lastDebounceTime = millis();
    }
#endif
}

void scanDevices(TwoWire *w)
{
    uint8_t err, addr;
    int nDevices = 0;
    uint32_t start = 0;

    Serial.println("I2C Devices scanning");
    for (addr = 1; addr < 127; addr++)
    {
        start = millis();
        w->beginTransmission(addr);
        delay(2);
        err = w->endTransmission();
        if (err == 0)
        {
            nDevices++;
            switch (addr)
            {
            case 0x77:
            case 0x76:
                Serial.println("\tFind BMX280 Sensor!");
                deviceOnline |= BME280_ONLINE;
                break;
            case 0x34:
                Serial.println("\tFind AXP192/AXP2101 PMU!");
                deviceOnline |= POWERMANAGE_ONLINE;
                break;
            case 0x3C:
                Serial.println("\tFind SSD1306/SH1106 dispaly!");
                deviceOnline |= DISPLAY_ONLINE;
                break;
            case 0x51:
                Serial.println("\tFind PCF8563 RTC!");
                deviceOnline |= PCF8563_ONLINE;
                break;
            case 0x1C:
                Serial.println("\tFind QMC6310 MAG Sensor!");
                deviceOnline |= QMC6310_ONLINE;
                break;
            default:
                Serial.print("\tI2C device found at address 0x");
                if (addr < 16)
                {
                    Serial.print("0");
                }
                Serial.print(addr, HEX);
                Serial.println(" !");
                break;
            }
        }
        else if (err == 4)
        {
            Serial.print("Unknow error at address 0x");
            if (addr < 16)
            {
                Serial.print("0");
            }
            Serial.println(addr, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");

    Serial.println("Scan devices done.");
    Serial.println("\n");
}

#ifdef HAS_GPS

bool l76kProbe()
{
    bool result = false;
    uint32_t startTimeout;
    SerialGPS.write("$PCAS03,0,0,0,0,0,0,0,0,0,0,,,0,0*02\r\n");
    delay(5);
    // Get version information
    startTimeout = millis() + 3000;
    Serial.print("Try to init L76K . Wait stop .");
    // SerialGPS.flush();
    while (SerialGPS.available())
    {
        int c = SerialGPS.read();
        // Serial.write(c);
        // Serial.print(".");
        // Serial.flush();
        // SerialGPS.flush();
        if (millis() > startTimeout)
        {
            Serial.println("Wait L76K stop NMEA timeout!");
            return false;
        }
    };
    Serial.println();
    SerialGPS.flush();
    delay(200);

    SerialGPS.write("$PCAS06,0*1B\r\n");
    startTimeout = millis() + 500;
    String ver = "";
    while (!SerialGPS.available())
    {
        if (millis() > startTimeout)
        {
            Serial.println("Get L76K timeout!");
            return false;
        }
    }
    SerialGPS.setTimeout(10);
    ver = SerialGPS.readStringUntil('\n');
    if (ver.startsWith("$GPTXT,01,01,02"))
    {
        Serial.println("L76K GNSS init succeeded, using L76K GNSS Module\n");
        result = true;
    }
    delay(500);

    // Initialize the L76K Chip, use GPS + GLONASS
    SerialGPS.write("$PCAS04,5*1C\r\n");
    delay(250);
    // only ask for RMC and GGA
    SerialGPS.write("$PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0*02\r\n");
    delay(250);
    // Switch to Vehicle Mode, since SoftRF enables Aviation < 2g
    SerialGPS.write("$PCAS11,3*1E\r\n");
    return result;
}

bool beginGPS()
{
    SerialGPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    bool result = false;
    for (int i = 0; i < 3; ++i)
    {
        result = l76kProbe();
        if (result)
        {
            return result;
        }
    }
    return result;
}

static int getAck(uint8_t *buffer, uint16_t size, uint8_t requestedClass, uint8_t requestedID)
{
    uint16_t ubxFrameCounter = 0;
    bool ubxFrame = 0;
    uint32_t startTime = millis();
    uint16_t needRead;

    while (millis() - startTime < 800)
    {
        while (SerialGPS.available())
        {
            int c = SerialGPS.read();
            switch (ubxFrameCounter)
            {
            case 0:
                if (c == 0xB5)
                {
                    ubxFrameCounter++;
                }
                break;
            case 1:
                if (c == 0x62)
                {
                    ubxFrameCounter++;
                }
                else
                {
                    ubxFrameCounter = 0;
                }
                break;
            case 2:
                if (c == requestedClass)
                {
                    ubxFrameCounter++;
                }
                else
                {
                    ubxFrameCounter = 0;
                }
                break;
            case 3:
                if (c == requestedID)
                {
                    ubxFrameCounter++;
                }
                else
                {
                    ubxFrameCounter = 0;
                }
                break;
            case 4:
                needRead = c;
                ubxFrameCounter++;
                break;
            case 5:
                needRead |= (c << 8);
                ubxFrameCounter++;
                break;
            case 6:
                if (needRead >= size)
                {
                    ubxFrameCounter = 0;
                    break;
                }
                if (SerialGPS.readBytes(buffer, needRead) != needRead)
                {
                    ubxFrameCounter = 0;
                }
                else
                {
                    return needRead;
                }
                break;

            default:
                break;
            }
        }
    }
    return 0;
}

bool recoveryGPS()
{
    uint8_t buffer[256];
    uint8_t cfg_clear1[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x1C, 0xA2};
    uint8_t cfg_clear2[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x1B, 0xA1};
    uint8_t cfg_clear3[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x03, 0x1D, 0xB3};
    SerialGPS.write(cfg_clear1, sizeof(cfg_clear1));

    if (getAck(buffer, 256, 0x05, 0x01))
    {
        Serial.println("Get ack successes!");
    }
    SerialGPS.write(cfg_clear2, sizeof(cfg_clear2));
    if (getAck(buffer, 256, 0x05, 0x01))
    {
        Serial.println("Get ack successes!");
    }
    SerialGPS.write(cfg_clear3, sizeof(cfg_clear3));
    if (getAck(buffer, 256, 0x05, 0x01))
    {
        Serial.println("Get ack successes!");
    }
    // UBX-CFG-RATE, Size 8, 'Navigation/measurement rate settings'
    uint8_t cfg_rate[] = {0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30};
    SerialGPS.write(cfg_rate, sizeof(cfg_rate));
    if (getAck(buffer, 256, 0x06, 0x08))
    {
        Serial.println("Get ack successes!");
    }
    else
    {
        return false;
    }
    return true;
}

#endif

#if defined(ARDUINO_ARCH_ESP32)

// NCP18XH103F03RB: https://item.szlcsc.com/14214.html
//  #define NTC_PIN 14 // NTC connection pins
#define SERIES_RESISTOR 10000      // Series resistance value (10kΩ)
#define B_COEFFICIENT 3950         // B value, set according to the NTC specification
#define ROOM_TEMP 298.15           // 25°C absolute temperature (K)
#define ROOM_TEMP_RESISTANCE 10000 // Resistance of NTC at 25°C (10kΩ)

float getTempForNTC()
{
    static float temperature = 0.0f;
    return temperature;
}

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

void setupBLE()
{
#ifdef ENABLE_BLE

    uint8_t mac[6];
    char macStr[18] = {0};
    esp_efuse_mac_get_default(mac);
    sprintf(macStr, "%02X:%02X", mac[0], mac[1]);

    String dev = BOARD_VARIANT_NAME;
    dev.concat('-');
    dev.concat(macStr);

    Serial.print("Starting BLE:");
    Serial.println(dev);

    BLEDevice::init(dev.c_str());
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE);

    pCharacteristic->setValue("Hello World");
    pService->start();
    // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("Characteristic defined! Now you can read it in your phone!");
#endif
}

#define CALIBRATE_ONE(cali_clk) calibrate_one(cali_clk, #cali_clk)

static uint32_t calibrate_one(rtc_cal_sel_t cal_clk, const char *name)
{
    const uint32_t cal_count = 1000;
    const float factor = (1 << 19) * 1000.0f;
    uint32_t cali_val;
    for (int i = 0; i < 5; ++i)
    {
        cali_val = rtc_clk_cal(cal_clk, cal_count);
    }
    return cali_val;
}

static void enable_slow_clock()
{
    rtc_clk_32k_enable(true);
    CALIBRATE_ONE(RTC_CAL_RTC_MUX);
    uint32_t cal_32k = CALIBRATE_ONE(RTC_CAL_32K_XTAL);
    if (cal_32k == 0)
    {
        Serial.printf("32K XTAL OSC has not started up");
    }
    else
    {
        rtc_clk_slow_freq_set(RTC_SLOW_FREQ_32K_XTAL);
        Serial.println("Switching RTC Source to 32.768Khz succeeded, using 32K XTAL");
        CALIBRATE_ONE(RTC_CAL_RTC_MUX);
        CALIBRATE_ONE(RTC_CAL_32K_XTAL);
    }
    CALIBRATE_ONE(RTC_CAL_RTC_MUX);
    CALIBRATE_ONE(RTC_CAL_32K_XTAL);
    if (rtc_clk_slow_freq_get() != RTC_SLOW_FREQ_32K_XTAL)
    {
        Serial.println("Warning: Failed to set rtc clk to 32.768Khz !!! ");
        return;
    }
    deviceOnline |= OSC32768_ONLINE;
}

void scanWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    Serial.println("WiFi Scan start");
    // WiFi.scanNetworks will return the number of networks found.
    int n = WiFi.scanNetworks();
    Serial.println("WiFi Scan done");
    if (n == 0)
    {
        Serial.println("no networks found");
    }
    else
    {
        Serial.print(n);
        Serial.println(" networks found");
        Serial.println("Nr | SSID                             | RSSI | CH | Encryption");
        for (int i = 0; i < n; ++i)
        {
            // Print SSID and RSSI for each network found
            Serial.printf("%2d", i + 1);
            Serial.print(" | ");
            Serial.printf("%-32.32s", WiFi.SSID(i).c_str());
            Serial.print(" | ");
            Serial.printf("%4ld", WiFi.RSSI(i));
            Serial.print(" | ");
            Serial.printf("%2ld", WiFi.channel(i));
            Serial.print(" | ");
            switch (WiFi.encryptionType(i))
            {
            case WIFI_AUTH_OPEN:
                Serial.print("open");
                break;
            case WIFI_AUTH_WEP:
                Serial.print("WEP");
                break;
            case WIFI_AUTH_WPA_PSK:
                Serial.print("WPA");
                break;
            case WIFI_AUTH_WPA2_PSK:
                Serial.print("WPA2");
                break;
            case WIFI_AUTH_WPA_WPA2_PSK:
                Serial.print("WPA+WPA2");
                break;
            case WIFI_AUTH_WPA2_ENTERPRISE:
                Serial.print("WPA2-EAP");
                break;
            case WIFI_AUTH_WPA3_PSK:
                Serial.print("WPA3");
                break;
            case WIFI_AUTH_WPA2_WPA3_PSK:
                Serial.print("WPA2+WPA3");
                break;
            case WIFI_AUTH_WAPI_PSK:
                Serial.print("WAPI");
                break;
            default:
                Serial.print("unknown");
            }
            Serial.println();
            delay(10);
        }
    }
    Serial.println("");

    // Delete the scan result to free memory for code below.
    WiFi.scanDelete();
}

#endif /*ARDUINO_ARCH_ESP32*/
