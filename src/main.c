#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
//Before periph to get away with gpio macro
#include "libmspware/driverlib.h"

//specific libmsp pieces
#include <libmsp/watchdog.h>
#include <libmsp/clock.h>
#include <libmsp/gpio.h>
#include <libmsp/periph.h>
#include <libmsp/sleep.h>
#include <libmsp/mem.h>
#include <libmsp/uart.h>
#include <libmspuartlink/uartlink.h>
#include <libhmc/magnetometer.h>

#if BOARD_MAJOR == 1 && BOARD_MINOR == 1
#include <libfxl/fxl6408.h>
#endif// BOARD_{MAJOR,MINOR}

//Other stuff
#include <libio/console.h>
#include <libchain/chain.h>
#include <libcapybara/capybara.h>
#include <libcapybara/reconfig.h>
#include <libcapybara/power.h>

#include "proximity.h"
#include "pins.h"
//Left here for now...
//#include <libwispbase/wisp-base.h>

#ifdef CONFIG_LIBEDB_PRINTF
#include <libedb/edb.h>
#endif

#ifdef CONFIG_EDB
#include <libedb/edb.h>
#else
#define WATCHPOINT(...)
#endif

TASK(1, task_init)
TASK(2, task_hmc, PREBURST, HIGHP, LOWP)
TASK(3, task_all, BURST)

void _capybara_handler(void) {
    msp_watchdog_disable();
    msp_gpio_unlock();
    __enable_interrupt();
// Don't wait if we're on continuous power
#ifndef CNTPWR
    capybara_wait_for_supply();
#if BOARD_MAJOR == 1 && BOARD_MINOR == 1
    capybara_wait_for_vcap();
#endif // BOARD_{MAJOR,MINOR}
#endif
    capybara_config_pins();
    msp_clock_setup();
// Set up deep_discharge stop
#ifndef CNTPWR
#if BOARD_MAJOR == 1 && BOARD_MINOR == 0
    capybara_shutdown_on_deep_discharge();
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
    capybara_wait_for_supply();
    if (capybara_shutdown_on_deep_discharge() == CB_ERROR_ALREADY_DEEPLY_DISCHARGED) {
        capybara_shutdown();
    }
#endif //BOARD.{MAJOR,MINOR}
#endif //CNTPWR

#if BOARD_MAJOR == 1 && BOARD_MINOR == 0
    GPIO(PORT_SENSE_SW, OUT) &= ~BIT(PIN_SENSE_SW);
    GPIO(PORT_SENSE_SW, DIR) |= BIT(PIN_SENSE_SW);

    GPIO(PORT_RADIO_SW, OUT) &= ~BIT(PIN_RADIO_SW);
    GPIO(PORT_RADIO_SW, DIR) |= BIT(PIN_RADIO_SW);

    P3OUT &= ~BIT5;
    P3DIR |= BIT5;

    P3OUT &= ~BIT0;
    P3DIR |= BIT0;
    GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG);
    GPIO(PORT_DEBUG, DIR) |= BIT(PIN_DEBUG);
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
    INIT_CONSOLE();
    //LOG2("i2c init\r\n");
    i2c_setup();

    //LOG2("fxl init\r\n");
    fxl_init();

    //LOG2("RADIO_SW\r\n");

    fxl_out(BIT_PHOTO_SW);
    fxl_out(BIT_RADIO_SW);
    fxl_out(BIT_RADIO_RST);
    fxl_out(BIT_APDS_SW);
    fxl_pull_up(BIT_CCS_WAKE);
    P3OUT &= ~(BIT5 | BIT6 | BIT7);
    P3DIR |= (BIT5 | BIT6 | BIT7);

    // SENSE_SW is present but is not electrically correct: do not use.
#else // BOARD_{MAJOR,MINOR}
#error Unsupported board: do not know what pins to configure (see BOARD var)
#endif // BOARD_{MAJOR,MINOR}


#if PWRCFG == PRECHRG || PWRCFG == TEST
   if(prechg_status){
    capybara_config_banks(prechg_config.banks);
    //capybara_wait_for_banks();
    msp_sleep(30);
    capybara_wait_for_supply();
   }
   if(burst_status == 2){
        prechg_status = 0;
        burst_status = 0;
   }
#endif //PWRCFG

#if PWRCFG == FXDLRG
    //Use MEDP2 for SE version and MEDHIGHP for TE version
    base_config.banks = MEDP2;
#elif PWRCFG == FXDSML
    base_config.banks = LOWP;
#endif

#ifndef CNTPWR
    capybara_config_banks(base_config.banks);
    capybara_wait_for_supply();
#endif
    LOG2("Gesture Test\r\n");
}

void delay(uint32_t cycles)
{
    unsigned i;
    for (i = 0; i < cycles / (1U << 15); ++i)
        __delay_cycles(1U << 15);
}


void init()
{
  _capybara_handler();
  LOG2("Done handler\r\n");
}

void task_hmc()
{
  magnet_t temp;
  magnetometer_read(&temp);
  LOG("Magneto vals = %u %u %u \r\n",temp.x,temp.y,temp.z);

  if(temp.x > 0xFF){
    TRANSITION_TO(task_all);
  }
  else{
    TRANSITION_TO(task_sample);
  }
}

void task_all()
{
  LOG("In task all!\r\n");
  delay(5000000);
  delay(5000000);

  TRANSITION_TO(task_sample);
}


ENTRY_TASK(task_init)
INIT_FUNC(init)

