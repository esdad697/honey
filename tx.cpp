#include "cc1100_raspi.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <getopt.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#define PACKAGE    "CC1100 SW"
#define VERSION_SW "0.9.6"

#define INTERVAL_1S_TIMER 1000


//--------------------------[Global CC1100 variables]--------------------------
uint8_t Tx_fifo[FIFOBUFFER], Rx_fifo[FIFOBUFFER];
uint8_t My_addr, Tx_addr, Rx_addr, Pktlen, pktlen, Lqi, Rssi;
uint8_t rx_addr,sender,lqi;
 int8_t rssi_dbm;

int cc1100_channel_select;

uint32_t prev_millis_1s_timer = 0;

uint8_t cc1100_debug = 0;								//set CC1100 lib in no-debug output mode
uint8_t tx_retries = 1;
uint8_t rx_demo_addr = 3;
int interval = 1000;

CC1100 cc1100;

//-------------------------- [End] --------------------------

//|============================ Main ============================|
int main(int argc, char *argv[]) {
	cc1100_debug = 1;
	My_addr = 1;
	rx_demo_addr = 3; // Addr of receiver. Ignored in broadcast mode.
	interval = 3000;

	//------------- welcome message ------------------------
	printf("Raspberry CC1101 SPI Library test\n");

	//------------- hardware setup ------------------------
	wiringPiSetup();			//setup wiringPi library

	cc1100.begin(My_addr);			//setup cc1000 RF IC
	cc1100.sidle();
	cc1100.receive();

	cc1100.show_main_settings();             //shows setting debug messages to UART
	cc1100.show_register_settings();
	//------------------------- Main Loop ------------------------
	for (;;)
	{
		delay(1);                                           //delay to reduce system load

		if (millis() - prev_millis_1s_timer >= interval)    // one second update timer
		{
    			Rx_addr = rx_demo_addr;                     //receiver address

    			uint32_t time_stamp = millis();             //generate time stamp

    			Tx_fifo[3] = (uint8_t)(time_stamp >> 24);   //split 32-Bit timestamp to 4 byte array
    			Tx_fifo[4] = (uint8_t)(time_stamp >> 16);
    			Tx_fifo[5] = (uint8_t)(time_stamp >> 8);
    			Tx_fifo[6] = (uint8_t)(time_stamp);

    			Pktlen = 0x07;                              //set packet len to 0x13

    			uint8_t res = cc1100.sent_packet(My_addr, Rx_addr, Tx_fifo, Pktlen, tx_retries);
    			
    			if( res == 1)    //sents package over air. ACK is received via GPIO polling
    			{
    				printf("transmitted tx_timestamp: %ums \r\n\r\n", time_stamp);
			}
			prev_millis_1s_timer = millis();
  		}

	}
    printf("finished!\n");
    return 0;
}
//-------------------- END OF MAIN --------------------------------
