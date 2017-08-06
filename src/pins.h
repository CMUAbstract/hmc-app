#ifndef PINS_H
#define PINS_H

// Ugly workaround to make the pretty GPIO macro work for OUT register
// (a control bit for TAxCCTLx uses the name 'OUT')
#undef OUT

#define BIT_INNER(idx) BIT ## idx
#define BIT(idx) BIT_INNER(idx)

#define GPIO_INNER(port, reg) P ## port ## reg
#define GPIO(port, reg) GPIO_INNER(port, reg)

#if defined(BOARD_WISP)
#define     PORT_LED_1           4
#define     PIN_LED_1            0
#define     PORT_LED_2           J
#define     PIN_LED_2            6

#define     PORT_AUX            3
#define 		PIN_AUX_0						3
#define     PIN_AUX_1           4
#define     PIN_AUX_2           5


#define     PORT_AUX3           1
#define     PIN_AUX_3           4

#elif defined(BOARD_MSP_TS430)

#define     PORT_AUX            3
#define     PIN_AUX_3           4
#define     PIN_AUX_4           5
#define			PIN_AUX_5					  6


#elif defined(BOARD_CAPYBARA)

#define PORT_DEBUG							3
#define PIN_DEBUG_1							4
#define PIN_DEBUG_2							5
#define PIN_DEBUG_3							6
#define PIN_DEBUG               6

#define PORT_LOAD 3
#define PIN_LOAD  4

#define PORT_CAPYBARA_CFG 3
#define PIN_CAPYBARA_CFG  5

#if BOARD_MAJOR == 1 && BOARD_MINOR == 0
#define PORT_SENSE_SW 3
#define PIN_SENSE_SW  7


#define PORT_RADIO_SW 3
#define PIN_RADIO_SW  2

#elif BOARD_MAJOR == 1 && BOARD_MINOR == 1

#define PORT_PHOTO_SENSE 2
#define PIN_PHOTO_SENSE 3 // GPIO extender pins
#define BIT_CCS_WAKE  (1 << 2)
#define BIT_SENSE_SW  (1 << 3)
#define BIT_PHOTO_SW  (1 << 4)
#define BIT_APDS_SW   (1 << 5)
#define BIT_RADIO_RST (1 << 6)
#define BIT_RADIO_SW  (1 << 7)

#endif // BOARD.{MAJOR,MINOR}

#elif defined(BOARD_SPRITE_APP_SOCKET_RHA)

#define     PORT_LED_1           1
#define     PIN_LED_1            0
#define     PORT_LED_2           1
#define     PIN_LED_2            2
#define     PORT_LED_3           1
#define     PIN_LED_3            0


#endif // BOARD_*

#endif

