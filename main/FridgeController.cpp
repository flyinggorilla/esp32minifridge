#include "FridgeController.h"

#include "sdkconfig.h"
#include <esp_system.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_system.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "String.h"

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
#define GPIO_FANCOLD GPIO_NUM_15
#define GPIO_LED GPIO_NUM_27
#define GPIO_PELTIER GPIO_NUM_32

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

void FridgeController::init()
{
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

    printf("initializing mcpwm gpio...\n");


    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; //frequency = 500Hz,
    pwm_config.cmpr_a = 0;       //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;       //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_FANHOT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_FANCOLD);
}

void FridgeController::FanHot(unsigned short speed)
{

    //gpio_set_level(GPIO_FANHOT, onoff);
    if (speed == 0)
    {
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    }
    else
    {
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (float)speed);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    }
}

