#include "FridgeController.h"

#include "sdkconfig.h"
#include <esp_system.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_system.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

#include "String.h"

static const char *LOGTAG = "fridge";

/*
* DS18B20 sensor (sparkfun cable uses black/red/white wires)
*  The pinout for this sensor is as follows: RED=Vcc BLACK=GND WHITE=SIG
*  RED: +3V output from Huzzah board (or 5V is ok)
*  BLACK: GND
*  WHITE: GPIO21, pullup 4.7kOhm to 3V (NOT 5V!!!!)
*/

#define MAX_DEVICES 2
#define DS18B20_RESOLUTION (DS18B20_RESOLUTION_12_BIT)
#define SAMPLE_PERIOD 2000 // milliseconds

/**
 * Configuration is for Adafruit Huzzah ESP32 board with Wroom32
 * ---------------------------------------------------------------------------
 * 
 * Pin13-GPIO13: output and connected to internal red LED on board
 * Pin14-GPIO14: output, fan-heater side
 * Pin15-GPIO15: output, fan-cooler side
 * Pin27-GPIO27: output, external status LED
 * Pin32-GPIO32: output, PWM, peltier element FET
 *
 */

#define GPIO_ONBOARDLED GPIO_NUM_13
#define GPIO_FANHOT GPIO_NUM_14
#define GPIO_FANCOLD GPIO_NUM_32
#define GPIO_LED GPIO_NUM_27
#define GPIO_PELTIER GPIO_NUM_15
#define GPIO_DS18B20 GPIO_NUM_21

FridgeController::FridgeController()
{
}

FridgeController::~FridgeController()
{
}

uint64_t PinBit(uint64_t pin)
{
    return BIT(pin);
}

void FridgeController::init(bool power, float targetTemperature)
{

    mfTargetTemperature = targetTemperature;
    mbIsPower = power;

    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = PinBit(GPIO_ONBOARDLED) | PinBit(GPIO_FANCOLD) | PinBit(GPIO_FANHOT) | PinBit(GPIO_PELTIER) | PinBit(GPIO_LED);
    //disable pull-down mode
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    //disable pull-up mode
    io_conf.pull_up_en = (gpio_pullup_t)0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    /*//2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; //frequency = 500Hz,
    pwm_config.cmpr_a = 0;     //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;     //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_FANHOT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_FANCOLD);*/

    //------ init onewire -------------------------

    // Create a 1-Wire bus

    mOwb = owb_rmt_initialize(&mRmtDriverInfo, GPIO_DS18B20, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(mOwb, true); // enable CRC check for ROM code

    /*uint64_t rom_code = 0x3b000006f247ee28; // sparkfun DS18B20 sensor
    OneWireBus_ROMCode known_device;
    memcpy(&known_device, &rom_code, sizeof(rom_code));

    char rom_code_s[17];
    owb_string_from_rom_code(known_device, rom_code_s, sizeof(rom_code_s));
    bool is_present = false;
    owb_verify_rom(mOwb, known_device, &is_present);
    ESP_LOGI(LOGTAG, "Device %s is %s", rom_code_s, is_present ? "present" : "not present");

    ESP_LOGI(LOGTAG, "Single device optimisations enabled");*/

    ds18b20_init_solo(&mDs18b20, mOwb); // only one device on bus
    ds18b20_use_crc(&mDs18b20, true);   // enable CRC check for temperature readings
    ds18b20_set_resolution(&mDs18b20, DS18B20_RESOLUTION);

    /*while (1)
    {
        printf("\nTemperature readings (degrees C):\n");
        float temp = ds18b20_convert_and_read_temp(&mDs18b20);
        printf("   %.3f\n", temp);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }*/
}

void FridgeController::Fan(bool onoff)
{
    gpio_set_level(GPIO_FANHOT, onoff);
}

void FridgeController::Power(bool onoff)
{
    mbIsPower = onoff;
}

bool FridgeController::SetDeadBand(float deadBand)
{
    if ((deadBand < 0.1) || (deadBand > 10.0)) // sanity checks, adjust to your needs
        return false;

    mfDeadBand = deadBand;
    return true;
}

/*void FridgeController::Led(uint8_t brightness){

    if (brightness == 0)
    {
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    }
    else
    {
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (float)brightness);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    }
};*/

void FridgeController::Peltier(bool onoff)
{
    gpio_set_level(GPIO_PELTIER, onoff);
    mbIsPeltier = onoff;
};

void FridgeController::Run()
{

    while (true)
    {
        if (!MeasureActualTemperature())
        {
            mbIsPower = false;
            mbIsError = true;
        }

        if (mbIsPower && !mbIsError)
        {
            float delta = mfActualTemperature - mfTargetTemperature;
            if (mbIsPeltier && (delta <= -(mfDeadBand / 2.0)))
            {
                Peltier(false);
                Fan(false);
            }
            if (!mbIsPeltier && (delta >= (mfDeadBand / 2.0)))
            {
                Peltier(true);
                Fan(true);
            }
        }
        else
        {
            Peltier(false);
            Fan(false);
        }

        if (mbIsPower) {
            ticksPowerOn++;
        }
        if (mbIsPeltier) {
            ticksCooling++;
        } 

        vTaskDelay(1000 / portTICK_PERIOD_MS); // 1 sec delay
    }
};

bool FridgeController::SetTargetTemperature(float targetTemperature)
{
    if ((targetTemperature < 5.0) || (targetTemperature > 30.0)) // sanity checks - adjust to your needs
        return false;

    mfTargetTemperature = targetTemperature;
    return true;
};

/*
* https://github.com/DavidAntliff/esp32-ds18b20-example
*
*/
bool FridgeController::MeasureActualTemperature()
{
    float temperature = ds18b20_convert_and_read_temp(&mDs18b20);
    ESP_LOGI(LOGTAG, "Temperature: %.3f", temperature);
    if (temperature == DS18B20_INVALID_READING)  {
        return false;
    }
    mfActualTemperature = temperature;
    return true;
};
