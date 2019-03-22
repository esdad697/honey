#include "cc1100_raspi.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <sys/select.h>

#include <getopt.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#define PACKAGE    "CC1100 SW"
#define VERSION_SW "0.9.1"

//--------------------------[Global CC1100 variables]--------------------------
uint8_t Tx_fifo[FIFOBUFFER], Rx_fifo[FIFOBUFFER];
uint8_t My_addr, Tx_addr, Rx_addr, Pktlen, pktlen, Lqi, Rssi;
uint8_t rx_addr,sender,lqi;
 int8_t rssi_dbm;


uint8_t cc1100_debug = 0;								//set CC1100 lib in no-debug output mode

CC1100 cc1100;

//-------------------------- [End] --------------------------

//|============================ Main ============================|
int main(int argc, char *argv[]) {
	cc1100_debug = 1;
	My_addr = 3;

	//------------- welcome message ------------------------
	printf("Raspberry CC1101 SPI Library test\n");

	//------------- hardware setup ------------------------
	wiringPiSetup();			//setup wiringPi library

	cc1100.begin(My_addr);			//setup cc1000 RF IC
	cc1100.sidle();
	
	cc1100.show_main_settings();             //shows setting debug messages to UART
    cc1100.show_register_settings();

	cc1100.receive();
	//------------------------- Main Loop ------------------------
	for (;;) {
		delay(1);                            //delay to reduce system load
		
		if (cc1100.packet_available())		 //checks if a packed is available
		{
		  cc1100.get_payload(Rx_fifo, pktlen, rx_addr, sender, rssi_dbm, lqi); //stores the payload data to Rx_fifo
		}
	}
	return 0;
}
//-------------------- END OF MAIN --------------------------------
