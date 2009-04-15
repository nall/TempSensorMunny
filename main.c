/* Name: main.c
 * Author: Jon Nall
 * Copyright: Copyright (c) 2009, Jon Nall
 * License: MIT License
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include "DS18B20.h"

#define GREEN_BITS 1 << 3
#define BLUE_BITS 1 << 4
#define RED_BITS 1 << 2

/* The average room temperature range is probably 0C-36C
 * We have 6 colors to choose from (8, but not counting "black"
 * and "white")
 *
 * BLUE  GREEN   RED    Description
 *   0      0      0    Off
 *   0      0      1    Red
 *   0      1      0    Green
 *   0      1      1    Yellow-ish
 *   1      0      0    Blue
 *   1      0      1    Purple
 *   1      1      0    Cyan
 *   1      1      1    White
 */


#define LED_BLACK  0
#define LED_RED    1
#define LED_GREEN  2
#define LED_YELLOW 3
#define LED_BLUE   4
#define LED_PURPLE 5
#define LED_CYAN   6
#define LED_WHITE  7

static void show_error();
static void update_leds(const uint8_t temp);

volatile uint8_t blink_led = 0;
volatile uint8_t led_value = 0;

#define IOCLK_IN_MS_AFTER_PRESCALING 27 // Why is this 55 / 2? 
#define TEMP_DELAY_IN_MS (5000) // Every 5 seconds


ISR(TIM0_OVF_vect)
{
    static uint32_t temp_count = 0;
    static int8_t last_temp = 0x80;

    if(temp_count == 0)
    {
        int16_t full_temp = 0;
        uint8_t error = 0;
        ds18b20_read_temperature(full_temp, error);
        if(error == EXIT_FAILURE)
        {
            show_error();
        }
        int8_t cur_temp = (full_temp >> 4);
        update_leds(cur_temp);

        blink_led = 1;

        last_temp = cur_temp;
    }

    // This routine gets called about once every 0.4ms
    // This routine gets called around once every 13.56ms
    temp_count += IOCLK_IN_MS_AFTER_PRESCALING;
    if(temp_count >= TEMP_DELAY_IN_MS)
    {
        temp_count = 0;
    }
}

static void show_error() // Blink red quickly
{
    PORTB &= ~0x1C;
    PORTB |= RED_BITS;

    cli();
    while(1)
    {
        _delay_ms(200);
        PORTB ^= RED_BITS;
    }
}

static void update_leds(const uint8_t temperature)
{
    if(temperature < 19)
    {
        led_value = LED_BLUE;
    }
    else if(temperature < 22)
    {
        led_value = LED_CYAN;
    }
    else if(temperature < 25)
    {
        led_value = LED_PURPLE;
    }
    else if(temperature < 27)
    {
        led_value = LED_GREEN;
    }
    else if(temperature < 30)
    {
        led_value = LED_YELLOW;
    }
    else
    {
        led_value = LED_RED;
    }

    led_value <<= 2;
}

int main(void)
{
    // Setup various AVR config
    {
        DDRB = _BV(4) | _BV(3) | _BV(2); // Enable RGB as outputs

        // ACSR &= ~(_BV(3));  // Disable A-D interrupts
        // ACSR |= _BV(7);     // Disable A-D logic entirely

        // 4.8MHz clock
        CLKPR = _BV(7); // Enable writing clock pre-scaler
        CLKPR = 0;      // Set prescaler to unity to run at 4.8MHz
    }

    // Initialize the 1-wire library
    onewire_init();

    // Set the resolution to 9 bits and store it in EEPROM
    {
        uint8_t error = 0;
        ds18b20_set_resolution(DS18B20_9BIT_RESOLUTION, 1, error);
        if(error == EXIT_FAILURE)
        {
            show_error();
        }
    }


    {
        GTCCR = _BV(TSM) | _BV(PSR10);

        TCCR0A = 0;
        TCCR0B = 5; // clk_io / 1024 ===> (4.8MHz / 1024) = 4.6kHz
                    // Counting from 0->255 this means the overflow will
                    // occur at ~18Hz -> every 54ms


        TIMSK0 = _BV(TOIE0); // Enable overflow interrupt enable

        sei();
        GTCCR &= ~_BV(TSM);
    }

    while(1)
    {
        // Sleep until it's time to change the temp
        while(blink_led == 0)
        {
            set_sleep_mode(SLEEP_MODE_IDLE);
            sleep_mode();
        }

        PORTB &= ~0x1C;
        PORTB |= led_value;

        TCNT0 = 0;
        blink_led = 0;
    }

    return 0;   /* never reached */
}
