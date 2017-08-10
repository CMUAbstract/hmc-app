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
#include <libmspmath/math.h> 


#if BOARD_MAJOR == 1 && BOARD_MINOR == 1
#include <libfxl/fxl6408.h>
#endif// BOARD_{MAJOR,MINOR}

//Other stuff
#include <libio/console.h>
#include <libchain/chain.h>
#include <libcapybara/capybara.h>
#include <libcapybara/reconfig.h>
#include <libcapybara/power.h>
#include <libapds/proximity.h>

#include "pins.h"

#define SERIES_LEN 8

// 2 ^ (WINDOW_DIV) + 1 = WINDOW_LEN
#define WINDOW_LEN 5
#define WINDOW_DIV 2

#define EVENT_THRESH 1000
#define ALERT_TH_LO_1 -9000
#define ALERT_TH_LO_2 -7000
#define ALERT_TH_HI_1 9000
#define ALERT_TH_HI_2 7000

#ifdef CONFIG_LIBEDB_PRINTF
#include <libedb/edb.h>
#endif

#ifdef CONFIG_EDB
#include <libedb/edb.h>
#else
#define WATCHPOINT(...)
#endif

#define TESTRUN 0

//#define CNTPWR
#define PRECHRG 1
#define FXDLRG 2
#define FXDRSP 3
#define RECFG 4
#define TEST 5
#define CNT 0

#define PWRCFG PRECHRG
#define SEND_GEST 1
#define LOG_PROX 0
#define DEFAULT_CFG 							0b111
#define PROX_ONLY 0

#define FXL_ADDR 0x43 // 100 0011
#define FXL_REG_ID        0x01

#if  PWRCFG == PRECHRG
TASK(1, task_init)
TASK(2, task_hmc, PREBURST, HIGHP, LOWP)
TASK(3, task_dist_meas_report, BURST)
#elif PWRCFG == RECFG
TASK(1, task_init)
TASK(2, task_hmc, CONFIGD, LOWP)
TASK(3, task_dist_meas_report, HIGHP)
#else
TASK(1, task_init)
TASK(2, task_hmc)
TASK(3, task_dist_meas_report)
#endif

struct msg_mag_val{
  CHAN_FIELD(uint16_t, mag_val);
};

CHANNEL(task_hmc,task_dist_meas_report,msg_mag_val);

typedef enum __attribute__((packed)) {
    RADIO_CMD_SET_ADV_PAYLOAD = 0,
} radio_cmd_t;

typedef struct __attribute__((packed)) {
    radio_cmd_t cmd;
    uint8_t series[SERIES_LEN];
} radio_pkt_t;

static radio_pkt_t radio_pkt;

int abs_int(int in){
  LOG2("Int size = %i \r\n",sizeof(int));
  int out;
  if(in < 0)
    out = in * -1;
  else
    out = in;
  return out;
}

void i2c_setup(void) {
  /*
  * Select Port 1
  * Set Pin 6, 7 to input Secondary Module Function:
  *   (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL)
  */
    GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P1,
    GPIO_PIN6 + GPIO_PIN7,
    GPIO_SECONDARY_MODULE_FUNCTION
  );

  EUSCI_B_I2C_initMasterParam param = {0};
  param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
  param.i2cClk = CS_getSMCLK();
  param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_400KBPS;
  param.byteCounterThreshold = 0;
  param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;

  EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);
}

static inline void radio_on()
{
#if BOARD_MAJOR == 1 && BOARD_MINOR == 0

#if PORT_RADIO_SW != PORT_RADIO_RST // we assume this below
#error Unexpected pin config: RAD_SW and RAD_RST not on same port
#endif // PORT_RADIO_SW != PORT_RADIO_RST

    GPIO(PORT_RADIO_SW, OUT) |= BIT(PIN_RADIO_SW) | BIT(PIN_RADIO_RST);
    GPIO(PORT_RADIO_RST, OUT) &= ~BIT(PIN_RADIO_RST);

#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
    fxl_set(BIT_RADIO_SW | BIT_RADIO_RST);
    fxl_clear(BIT_RADIO_RST);

#else // BOARD_{MAJOR,MINOR}
#error Unsupported board: do not know how to turn off radio (see BOARD var)
#endif // BOARD_{MAJOR,MINOR}
}

static inline void radio_off()
{
#if BOARD_MAJOR == 1 && BOARD_MINOR == 0
    GPIO(PORT_RADIO_SW, OUT) &= ~BIT(PIN_RADIO_SW);
#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1
    fxl_clear(BIT_RADIO_SW);
#else // BOARD_{MAJOR,MINOR}
#error Unsupported board: do not know how to turn on radio (see BOARD var)
#endif // BOARD_{MAJOR,MINOR}
}

void fxl_test(void){
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, FXL_ADDR);

  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

  EUSCI_B_I2C_enable(EUSCI_B0_BASE);

  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, FXL_REG_ID);

  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);

  EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);

  uint8_t id  = EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);

  EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);

  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  LOG2("[fxl test] FXL Id = %x \r\n",id);
}

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
    LOG2("\r\n[main] Start\r\n******** \r\ni2c init\r\n");
    i2c_setup();

    LOG2("[main] fxl full init\r\n");
    fxl_init();
    fxl_out(BIT_PHOTO_SW);
    fxl_out(BIT_RADIO_SW);
    fxl_out(BIT_RADIO_RST);
    fxl_out(BIT_APDS_SW);
    fxl_pull_up(BIT_CCS_WAKE);
    
    // Prep debug pins
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
    base_config.banks = 0xC;
#elif PWRCFG == FXDSML
    base_config.banks = LOWP;
#endif

#ifndef CNTPWR
    capybara_config_banks(base_config.banks);
    capybara_wait_for_supply();
#endif
    // Do non-essential inits here so we finish bank init before we run out of
    // energy and faceplant
    // Init magneto here
    //magnetometer_init();
    
    //PRINTF("Mag Test\r\n");
}

void capybara_transition()
{
    // need to explore exactly how we want BURST tasks to be followed --> should
    // we ever shutdown to reconfigure? Or should we always ride the burst wave
    // until we're out of energy?
#if (PWRCFG == PRECHRG) || (PWRCFG == RECFG) || (PWRCFG == TEST)

    // Check previous burst state and register a finished burst
    if(burst_status){
        burst_status = 2;
        //return;
    }
    task_cfg_spec_t curpwrcfg = curctx->task->spec_cfg;
    switch(curpwrcfg){
        case BURST:
            if(!prechg_status){
              PRINTF("Error! Running w/out precharge!\r\n");
            }
            prechg_status = 0;
            capybara_config_banks(prechg_config.banks);
            burst_status = 1;
            break;

        case PREBURST:
            if(!prechg_status){
                prechg_config.banks = curctx->task->precfg->banks;
                capybara_config_banks(prechg_config.banks);
                // Mark that we finished the config_banks_command
                prechg_status = 1;
                capybara_shutdown();
                capybara_wait_for_supply();
            }
            //intentional fall through

        case CONFIGD:

            if(base_config.banks != curctx->task->opcfg->banks){
                // Temp:
                P3OUT |= BIT7;
                base_config.banks = curctx->task->opcfg->banks;
                capybara_config_banks(base_config.banks);
                // Temp:
                capybara_shutdown();
                capybara_wait_for_supply();
                P3OUT &= ~BIT7;
            }
            //Another intentional fall through

        default:
            break;
    }
#endif
   //LOG("Running task %u \r\n",curctx->task->idx);

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

void task_init()
{ capybara_transition();
  LOG("In task init\r\n");
  TRANSITION_TO(task_hmc);
}

volatile int mag_init_flag = 0;

void task_hmc()
{ capybara_transition();
  magnet_t temp;
  int prev,diff=0;
  //GPIO(3,OUT) |= BIT(5);
  LOG("[main] In task hmc\r\n");
  LOG2("[main] Magneto init!\r\n");
  //GPIO(3,OUT) &= ~BIT(5);
  LOG2("[main] Reading magneto!\r\n");
  if(!mag_init_flag){
    mag_init_flag = 1;
    magnetometer_init();
  }
  //for(unsigned i = 0; i < WINDOW_LEN; i++){
    //prev = temp.z;
    GPIO(3,OUT) |= BIT(5);
    magnetometer_read(&temp);
   
    uint16_t out;
    out = abs_int(temp.x);
    
    //sqrX = mult16(out,out);
    //PRINTF("out= %u X=%i\r\n",out,temp.x);
    out += abs_int(temp.y);
    //sqrY = mult16(out,out);
    //PRINTF("out= %u Y=%i\r\n",out,temp.y);
    out += abs_int(temp.z);
    //PRINTF("out= %u Z=%i\r\n",out,temp.z);
    //sqrZ = mult16(out,out);
    uint16_t mag;
    //mag = sqrt16(sqrY + sqrZ); 
#if TESTRUN 
    PRINTF("magnitude = %u\r\n",out);
#else
   // Threshold check
   /*
    if(!(temp.z > ALERT_TH_HI_2 && temp.z < ALERT_TH_HI_1) && 
              !(temp.z > ALERT_TH_LO_2 && temp.z < ALERT_TH_LO_1)){ */
    PRINTF("magnitude = %u\r\n",out);
    if(out > 50000){
      CHAN_OUT1(uint16_t,mag_val,out,CH(task_hmc,task_dist_meas_report));
      TRANSITION_TO(task_dist_meas_report);
    }
    //PRINTF("%i %i %i\r\n",temp.x,temp.y,temp.z);
   /*
   GPIO(3,OUT) &= ~BIT(5);
    if(i){
      diff += abs_int(temp.z - prev);
    }  
    */
    //PRINTF("%i %i\r\n",window[i].z,i);
    //LOG2("[main] Magneto vals = %i %i %i\r\n",window[i].x,window[i].y,window[i].z);
  //}

 /* for(unsigned i = WINDOW_LEN - 1; i > 0; i--){
    int cur_diff  = window[i].z - window[i-1].z;
    diff += cur_diff;
    //PRINTF("[main] Current Diff = %i, running diff = %i \r\n",cur_diff,diff);
  }
  */
  //diff = diff >> WINDOW_DIV;
  //PRINTF("Avg div = %i \r\n",diff);
  //if(diff > EVENT_THRESH){
  //if(0){
  //delay(240000);
    //PRINTF("Avg div = %i \r\n",diff);
//  }
  //else{
#endif //TESTRUN
    LOG("In transition_to!\r\n");
    TRANSITION_TO(task_hmc);
  //}
}

void task_dist_meas_report()
{ capybara_transition();
  int mag_val;
  mag_val = *CHAN_IN1(int,mag_val,CH(task_hmc,task_dist_meas_report));
  PRINTF("Got mag_val %u \r\n",mag_val);
  LOG("[main] In task dist meas + report\r\n");
  fxl_test();
  LOG2("[main] Post fxl_test\r\n"); fxl_set(BIT_APDS_SW);
  msp_sleep(30);
  LOG2("[main] Post fxl_set\r\n");
  proximity_init();
  enableProximitySensor();
  enableGesture();
  disableGesture();
  //delay(240000);
  LOG("Running prox init!\r\n");
  proximity_init();
  enableProximitySensor();
  //delay(240000);
  LOG("Reading prox!\r\n");
  uint8_t test = 0;
  uint8_t max = 0;
  for(unsigned i = 0; i < 64; i++){
    GPIO(3,OUT) |= BIT(6);
    test = readProximity();
    __delay_cycles(24000);
    if(test > max)
      max = test;
    //PRINTF("[main] proxVal = %u\r\n",test);
    GPIO(3,OUT) &= ~BIT(6);
  }
  PRINTF("[main] Max prox reading = %u\r\n",max);
  // Delay 5 sec so we can see the output
  //__delay_cycles(40000000);
  uint8_t len = 8;
  //for(int i = 1; i < len; i++)
  //  radio_pkt.series[i] = test;
  fxl_init();
  fxl_out(BIT_PHOTO_SW);
  fxl_out(BIT_RADIO_SW);
  fxl_out(BIT_RADIO_RST);
  fxl_out(BIT_APDS_SW);
  fxl_pull_up(BIT_CCS_WAKE);

  fxl_clear(BIT_APDS_SW);
  // Turn on LED before sending packet
  GPIO(3,OUT) |= BIT(7);
  __delay_cycles(4000000);
  GPIO(3,OUT) &= ~BIT(7);

  LOG("Initializing radio!\r\n");
  radio_pkt.series[0] = 0xAA;
  radio_pkt.series[1] = 0xEE;
  radio_pkt.series[2] = mag_val;
  radio_pkt.series[3] = max;
  radio_on();
  msp_sleep(400); // ~15ms @ ACLK/8
  //msp_sleep(64); // ~15ms @ ACLK/8
  LOG("Opening uart link!\r\n");
  uartlink_open_tx();
  uartlink_send((uint8_t *)&radio_pkt.cmd, sizeof(radio_pkt.cmd) + len);
  uartlink_close();

  // TODO: wait until radio is finished; for now, wait for 0.25sec
  msp_sleep(2048);
  radio_off();

  // Turn on LED? This is shaping up to be pretty cheap
  PRINTF("RAdio finish!\r\n");
  GPIO(3,OUT) &= BIT(6);
  TRANSITION_TO(task_hmc);
}




ENTRY_TASK(task_init)
INIT_FUNC(init)

