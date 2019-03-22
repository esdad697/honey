CC1101
======

Raspberry Pi test program for SmartRF Studio.<br />
Note: Raspi need wiringPi<br />

Hardware connection
===================

check cc1101_raspi.h for Pin description

CC1101 Vdd = 3.3V
CC1101 max. digital voltage level = 3.3V (not 5V tolerant)

```
CC1101<->Raspi

Vdd    -    3.3V (P1-01)
SI     -    MOSI (P1-19)
SO     -    MISO (P1-21)
CS     -    SS   (P1-24)
SCLK   -    SCK  (P1-23)
GDO2   -    GPIO (P1-22)
GDO0   -    not used in this demo
GND    -    P1-25
```

General description of RF packet
================================

```
-> pkt_len [1byte] | rx_addr [1byte] | tx_addr [1byte] | payload data [1..60bytes]
```

pkt_len = count of bytes which shall transfered over air (rx_addr + tx_addr + payload data)<br />
rx_addr = address of device, which shall receive the message (0x00 = broadcast to all devices)<br />
tx_addr = transmitter or my address. the receiver should know who has sent a message.<br />
payload = 1 to 60 bytes payload data.<br />

TX Bytes example:<br />
-> 0x06 0x03 0x01 0x00 0x01 0x02 0x03<br />

Basic configuration
===================

use **uint8_t CC1100::begin(volatile uint8_t &My_addr)** always as first configuration step.

Device address
--------------
Not used in this implementation. Be sure address filtering is not enabled (PKTCTRL1_ADR_CHK = 0) so the ADDR register is don’t care. 


SmartRF Studio
----------------
Copy register output directly in cc1100_raspi.cpp


Raspberry Pi
============

How to compile Raspi Demo files
-------------------------------

be sure first, that you have already wiringPi installed on your Raspberry Pi hardware. 

'm' makes tx and rx <br />

tx is transmitter <br />

rx is receiver <br />

t and r are single character commands that call tx and rx <br />

