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
*  RED: +5V
*  BLACK: GND
*  WHITE: GPIO21 
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
    owb_rmt_driver_info rmt_driver_info;
    mOwb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(mOwb, true); // enable CRC check for ROM code

    // Find all connected devices
    /*ESP_LOGI(LOGTAG, "find devices");
    OneWireBus_ROMCode device_rom_codes[MAX_DEVICES]; // = {0};
    memset(&device_rom_codes, 0, sizeof(device_rom_codes));
    int num_devices = 0;
    OneWireBus_SearchState search_state; // = {0};
     memset(&device_rom_codes, 0, sizeof(search_state));


    bool found = false;
    owb_search_first(mOwb, &search_state, &found);
    while (found)
    {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        ESP_LOGI(LOGTAG, "  %d : %s", num_devices, rom_code_s);
        device_rom_codes[num_devices] = search_state.rom_code;
        ++num_devices;
        owb_search_next(mOwb, &search_state, &found);
    }

    ESP_LOGI(LOGTAG, "Found %d devices", num_devices);

    //uint64_t rom_code = 0x0001162e87ccee28;  // pink
    //uint64_t rom_code = 0xf402162c6149ee28;  // green
    //uint64_t rom_code = 0x1502162ca5b2ee28;  // orange
    //uint64_t rom_code = owb_read_rom(mOwb);

    // Known ROM code (LSB first):

    */

    // ROM CODE OF Temperature Sensor 3b000006f247ee28
    /*OneWireBus_ROMCode known_device;
    known_device.fields.family[0] = (uint8_t)0x28;
    known_device.fields.serial_number[0] = (uint8_t)0xee;
    known_device.fields.serial_number[1] = (uint8_t)0x47;
    known_device.fields.serial_number[2] = (uint8_t)0xf2;
    known_device.fields.serial_number[3] = (uint8_t)0x06;
    known_device.fields.serial_number[4] = (uint8_t)0x00;
    known_device.fields.serial_number[5] = (uint8_t)0x00;
    known_device.fields.crc[0] = (uint8_t)0x00;  */

    uint64_t rom_code = 0x3b000006f247ee28; // sparkfun DS18B20 sensor
    OneWireBus_ROMCode known_device;
    memcpy(&known_device, &rom_code, sizeof(rom_code));

    char rom_code_s[17];
    owb_string_from_rom_code(known_device, rom_code_s, sizeof(rom_code_s));
    bool is_present = false;
    owb_verify_rom(mOwb, known_device, &is_present);
    ESP_LOGI(LOGTAG, "Device %s is %s", rom_code_s, is_present ? "present" : "not present");

    ESP_LOGI(LOGTAG, "Single device optimisations enabled");

    DS18B20_Info ds18b20_info;
    ds18b20_init_solo(&ds18b20_info, mOwb); // only one device on bus
    ds18b20_use_crc(&ds18b20_info, true);   // enable CRC check for temperature readings
    ds18b20_set_resolution(&ds18b20_info, DS18B20_RESOLUTION);

    /*
    // Create a DS18B20 device on the 1-Wire bus

    for (int i = 0; i < MAX_DEVICES; ++i)
    {
        devices[i] = &(devices_static[i]);
    }

    for (int i = 0; i < num_devices; ++i)
    {
        DS18B20_Info *ds18b20_info = devices[i];

        if (num_devices == 1)
        {
            ESP_LOGI(LOGTAG, "Single device optimisations enabled");
            ds18b20_init_solo(ds18b20_info, mOwb); // only one device on bus
        }
        else
        {
            ds18b20_init(ds18b20_info, mOwb, device_rom_codes[i]); // associate with bus and device
        }
        ds18b20_use_crc(ds18b20_info, true); // enable CRC check for temperature readings
        ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);
    } */
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
    if ((deadBand < 0.5) || (deadBand > 10.0)) // sanity checks, adjust to your needs
        return false;

    mfDeadBand = deadBand;
    return true;
}

void FridgeController::Led(uint8_t brightness){

    /*if (brightness == 0)
    {
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    }
    else
    {
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (float)brightness);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    }*/
};

void FridgeController::Peltier(bool onoff)
{
    gpio_set_level(GPIO_PELTIER, onoff);
    mbIsPeltier = onoff;
};

void FridgeController::Run()
{

    //    // Read temperatures from all sensors sequentially
    //    while (1)
    //    {
    //        printf("\nTemperature readings (degrees C):\n");
    //        for (int i = 0; i < num_devices; ++i)
    //        {
    //            float temp = ds18b20_get_temp(devices[i]);
    //            printf("  %d: %.3f\n", i, temp);
    //        }
    //        vTaskDelay(1000 / portTICK_PERIOD_MS);
    //    }

    // Read temperatures more efficiently by starting conversions on all devices at the same time

    int sample_count = 0;

    while (1)
    {
        TickType_t start_ticks = xTaskGetTickCount();

        /*      ds18b20_convert_all(mOwb);

            // In this application all devices use the same resolution,
            // so use the first device to determine the delay
            ds18b20_wait_for_conversion(devices[0]);

            // Read the results immediately after conversion otherwise it may fail
            // (using printf before reading may take too long)
            float temps[MAX_DEVICES];
            for (int i = 0; i < num_devices; ++i)
            {
                temps[i] = ds18b20_read_temp(devices[i]);
            }

            // Print results in a separate loop, after all have been read
            ESP_LOGI(LOGTAG, "  Temperature readings (degrees C): sample %d", ++sample_count);
            for (int i = 0; i < num_devices; ++i)
            {
                if (temps[i] == DS18B20_INVALID_READING)
                {
                    ++crc_errors[i];
                }

                ESP_LOGI(LOGTAG, "  %d: %.1f    %d errors", i, temps[i], crc_errors[i]);
            } */

        // Make up periodic delay to approximately one sample period per measurement
        ESP_LOGI(LOGTAG, "sample iteration.");
        if ((xTaskGetTickCount() - start_ticks) < (SAMPLE_PERIOD / portTICK_PERIOD_MS))
        {
            vTaskDelay(SAMPLE_PERIOD / portTICK_PERIOD_MS - (xTaskGetTickCount() - start_ticks));
        }
    }

    //-------------------------------

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
    return false;
};
