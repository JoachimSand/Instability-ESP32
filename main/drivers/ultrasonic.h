#ifndef ULTRASONIC_H
#define ULTRASONIC_H


#define RMT_TX_CHANNEL 1 /* RMT channel for transmitter */
#define RMT_TX_GPIO_NUM PIN_TRIGGER /* GPIO number for transmitter signal */
#define RMT_RX_CHANNEL 0 /* RMT channel for receiver */
#define RMT_RX_GPIO_NUM PIN_ECHO /* GPIO number for receiver */
#define RMT_CLK_DIV 100 /* RMT counter clock divider */
#define RMT_TX_CARRIER_EN 0 /* Disable carrier */
#define rmt_item32_tIMEOUT_US 9500 /*!< RMT receiver timeout value(us) */

#define RMT_TICK_10_US (80000000/RMT_CLK_DIV/100000) /* RMT counter value for 10 us.(Source clock is APB clock) */
#define ITEM_DURATION(d) ((d & 0x7fff)*10/RMT_TICK_10_US)

#define PIN_TRIGGER GPIO_NUM_16
#define PIN_ECHO GPIO_NUM_17


void init_ultrasonic();

float read_ultrasonic();

#endif
