/*-------------------------------------------------------------------------------
'                     CC1100 Raspberry Pi Library
'                     ---------------------------
'
'
'
'
'
'  module contains helper code from other people. Thx for that
'-----------------------------------------------------------------------------*/
#include "cc1100_raspi.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

extern uint8_t cc1100_debug;

static uint8_t PATABLE_POWER[8] = {0x00,0x51,0x00,0x00,0x00,0x00,0x00,0x00};

static uint8_t STUDIO_REGISTERS[CFG_REGISTER] = {
                    0x06,  // IOCFG2        GDO2 Output Pin Configuration
                    0x2E,  // IOCFG1        GDO1 Output Pin Configuration
                    0x06,  // IOCFG0        GDO0 Output Pin Configuration
    0x47,  // FIFOTHR             RX FIFO and TX FIFO Thresholds
    0xD3,  // SYNC1               Sync Word, High Byte
    0x91,  // SYNC0               Sync Word, Low Byte
    0xFF,  // PKTLEN              Packet Length
    0x04,  // PKTCTRL1            Packet Automation Control
    0x05,  // PKTCTRL0            Packet Automation Control
    0x00,  // ADDR                Device Address
    0x00,  // CHANNR              Channel Number
    0x06,  // FSCTRL1             Frequency Synthesizer Control
    0x00,  // FSCTRL0             Frequency Synthesizer Control
    0x0D,  // FREQ2               Frequency Control Word, High Byte
    0x44,  // FREQ1               Frequency Control Word, Middle Byte
    0xEC,  // FREQ0               Frequency Control Word, Low Byte
    0xF5,  // MDMCFG4             Modem Configuration
    0x83,  // MDMCFG3             Modem Configuration
    0x33,  // MDMCFG2             Modem Configuration
    0x22,  // MDMCFG1             Modem Configuration
    0xF8,  // MDMCFG0             Modem Configuration
    0x15,  // DEVIATN             Modem Deviation Setting
    0x07,  // MCSM2               Main Radio Control State Machine Configuration
    0x0C,  // MCSM1               Main Radio Control State Machine Configuration
    0x18,  // MCSM0               Main Radio Control State Machine Configuration
    0x14,  // FOCCFG              Frequency Offset Compensation Configuration
    0x6C,  // BSCFG               Bit Synchronization Configuration
    0x03,  // AGCCTRL2            AGC Control
    0x40,  // AGCCTRL1            AGC Control
    0x92,  // AGCCTRL0            AGC Control
    0x87,  // WOREVT1             High Byte Event0 Timeout
    0x6B,  // WOREVT0             Low Byte Event0 Timeout
    0xFB,  // WORCTRL             Wake On Radio Control
    0x56,  // FREND1              Front End RX Configuration
    0x11,  // FREND0              Front End TX Configuration
    0xE9,  // FSCAL3              Frequency Synthesizer Calibration
    0x2A,  // FSCAL2              Frequency Synthesizer Calibration
    0x00,  // FSCAL1              Frequency Synthesizer Calibration
    0x1F,  // FSCAL0              Frequency Synthesizer Calibration
    0x41,  // RCCTRL1             RC Oscillator Configuration
    0x00,  // RCCTRL0             RC Oscillator Configuration
    0x59,  // FSTEST              Frequency Synthesizer Calibration Control
    0x7F,  // PTEST               Production Test
    0x3F,  // AGCTEST             AGC Test
    0x81,  // TEST2               Various Test Settings
    0x35,  // TEST1               Various Test Settings
    0x09,  // TEST0               Various Test Settings
};

//----------------------------------[END]---------------------------------------

//-------------------------[CC1100 reset function]------------------------------
void CC1100::reset(void)                  // reset defined in cc1100 datasheet
{
    digitalWrite(SS_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(SS_PIN, HIGH);
    delayMicroseconds(40);

    spi_write_strobe(SRES);
    delay(1);
}
//-----------------------------[END]--------------------------------------------

//------------------------[set Power Down]--------------------------------------
void CC1100::powerdown(void)
{
    sidle();
    spi_write_strobe(SPWD);               // CC1100 Power Down
}
//-----------------------------[end]--------------------------------------------

//---------------------------[WakeUp]-------------------------------------------
void CC1100::wakeup(void)
{
    digitalWrite(SS_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(SS_PIN, HIGH);
    delayMicroseconds(10);
    receive();                            // go to RX Mode
}
//-----------------------------[end]--------------------------------------------

//---------------------[CC1100 set debug level]---------------------------------
uint8_t CC1100::set_debug_level(uint8_t set_debug_level = 1)  //default ON
{
    debug_level = set_debug_level;        //set debug level of CC1101 outputs

    return debug_level;
}
//-----------------------------[end]--------------------------------------------

//---------------------[CC1100 get debug level]---------------------------------
uint8_t CC1100::get_debug_level(void)
{
    return debug_level;
}
//-----------------------------[end]--------------------------------------------

//----------------------[CC1100 init functions]---------------------------------
uint8_t CC1100::begin(volatile uint8_t &My_addr)
{
    uint8_t partnum, version;

    pinMode(GDO0, INPUT);                 //setup AVR GPIO ports
    pinMode(GDO2, INPUT);

    set_debug_level(set_debug_level());   //set debug level of CC1101 outputs

    if(debug_level > 0){
          printf("Init CC1100...\r\n");
    }

    spi_begin();                          //inits SPI Interface
    reset();                              //CC1100 init reset

    spi_write_strobe(SFTX);delayMicroseconds(100);//flush the TX_fifo content
    spi_write_strobe(SFRX);delayMicroseconds(100);//flush the RX_fifo content

    partnum = spi_read_register(PARTNUM); //reads CC1100 partnumber
    version = spi_read_register(VERSION); //reads CC1100 version number

    //checks if valid Chip ID is found. Usualy 0x03 or 0x14. if not -> abort
    if(version == 0x00 || version == 0xFF){
        if(debug_level > 0){
            printf("no CC11xx found!\r\n");
        }
        return FALSE;
    }

    if(debug_level > 0){
          printf("Partnumber: 0x%02X\r\n", partnum);
          printf("Version   : 0x%02X\r\n", version);
    }



    spi_write_burst(WRITE_BURST,STUDIO_REGISTERS,CFG_REGISTER);
    spi_write_burst(PATABLE_BURST,PATABLE_POWER,8);

    //set my receiver address
    set_myaddr(My_addr);                  //My_Addr from EEPROM to global variable

    if(debug_level > 0){
          printf("...done!\r\n");
    }

    receive();                                  //set CC1100 in receive mode

    return TRUE;
}
//-------------------------------[end]------------------------------------------

//-----------------[finish's the CC1100 operation]------------------------------
void CC1100::end(void)
{
    powerdown();                          //power down CC1100
}
//-------------------------------[end]------------------------------------------

//-----------------------[show all CC1100 registers]----------------------------
void CC1100::show_register_settings(void)
{
    if(debug_level > 0){
        uint8_t config_reg_verify[CFG_REGISTER],Patable_verify[CFG_REGISTER];

        spi_read_burst(READ_BURST,config_reg_verify,CFG_REGISTER);  //reads all 47 config register from cc1100
        spi_read_burst(PATABLE_BURST,Patable_verify,8);             //reads output power settings from cc1100

        //show_main_settings();
        printf("Config Register:\r\n");

        for(uint8_t i = 0 ; i < CFG_REGISTER; i++)  //showes rx_buffer for debug
        {
            printf("0x%02X ", config_reg_verify[i]);
            if(i==9 || i==19 || i==29 || i==39) //just for beautiful output style
            {
                printf("\r\n");
            }
        }
        printf("\r\n");
        printf("PaTable:\r\n");

        for(uint8_t i = 0 ; i < 8; i++)         //showes rx_buffer for debug
        {
            printf("0x%02X ", Patable_verify[i]);
        }
        printf("\r\n");
    }
}
//-------------------------------[end]------------------------------------------

//--------------------------[show settings]-------------------------------------
void CC1100::show_main_settings(void)
{
     extern volatile uint8_t My_addr;

     printf("My_Addr: %d\r\n", My_addr);
}
//-------------------------------[end]------------------------------------------

//----------------------------[idle mode]---------------------------------------
uint8_t CC1100::sidle(void)
{
    uint8_t marcstate;

    spi_write_strobe(SIDLE);              //sets to idle first. must be in

    marcstate = 0xFF;                     //set unknown/dummy state value

    while(marcstate != 0x01)              //0x01 = sidle
    {
        marcstate = (spi_read_register(MARCSTATE) & 0x1F); //read out state of cc1100 to be sure in RX
        //printf("marcstate_rx: 0x%02X\r", marcstate);
    }
    //Serial.println();
    delayMicroseconds(100);
    return TRUE;
}
//-------------------------------[end]------------------------------------------

//---------------------------[transmit mode]------------------------------------
uint8_t CC1100::transmit(void)
{
    uint8_t marcstate;

    sidle();                              //sets to idle first.
    spi_write_strobe(STX);                //sends the data over air

    marcstate = 0xFF;                     //set unknown/dummy state value

    while(marcstate != 0x01)              //0x01 = ILDE after sending data
    {
        marcstate = (spi_read_register(MARCSTATE) & 0x1F); //read out state of cc1100 to be sure in IDLE and TX is finished
        //printf("marcstate_tx: 0x%02X ",marcstate);
    }
    //printf("\r\n");
    delayMicroseconds(100);
    return TRUE;
}
///-------------------------------[end]------------------------------------------

//---------------------------[receive mode]-------------------------------------
uint8_t CC1100::receive(void)
{
    uint8_t marcstate;

    sidle();                              //sets to idle first.
    spi_write_strobe(SRX);                //writes receive strobe (receive mode)

    marcstate = 0xFF;                     //set unknown/dummy state value

    while(marcstate != 0x0D)              //0x0D = RX
    {
        marcstate = (spi_read_register(MARCSTATE) & 0x1F); //read out state of cc1100 to be sure in RX
        //printf("marcstate_rx: 0x%02X\r", marcstate);
    }
    //printf("\r\n");
    delayMicroseconds(100);
    return TRUE;
}
//-------------------------------[end]------------------------------------------

//-------------------------[tx_payload_burst]-----------------------------------
uint8_t CC1100::tx_payload_burst(uint8_t my_addr, uint8_t rx_addr,
                              uint8_t *txbuffer, uint8_t length)
{
    txbuffer[0] = length-1;
    txbuffer[1] = rx_addr;
    txbuffer[2] = my_addr;

    spi_write_burst(TXFIFO_BURST,txbuffer,length); //writes TX_Buffer +1 because of pktlen must be also transfered

    if(debug_level > 0){
        printf("TX_FIFO: ");
        for(uint8_t i = 0 ; i < length; i++)       //TX_fifo debug out
        {
             printf("0x%02X ", txbuffer[i]);
        }
        printf("\r\n");
  }
  return TRUE;
}
//-------------------------------[end]------------------------------------------

//------------------[rx_payload_burst - package received]-----------------------
uint8_t CC1100::rx_payload_burst(uint8_t rxbuffer[], uint8_t &pktlen)
{
    uint8_t bytes_in_RXFIFO = 0;
    uint8_t res = 0;

    bytes_in_RXFIFO = spi_read_register(RXBYTES);              //reads the number of bytes in RXFIFO

    if((bytes_in_RXFIFO & 0x7F) && !(bytes_in_RXFIFO & 0x80))  //if bytes in buffer and no RX Overflow
    {
        spi_read_burst(RXFIFO_BURST, rxbuffer, bytes_in_RXFIFO);
        pktlen = rxbuffer[0];
        res = TRUE;
    }
    else
    {
        if(debug_level > 0){
            printf("no bytes in RX buffer or RX Overflow!: ");printf("0x%02X \r\n", bytes_in_RXFIFO);
        }
        sidle();                                                  //set to IDLE
        spi_write_strobe(SFRX);delayMicroseconds(100);            //flush RX Buffer
        receive();                                                //set to receive mode
        res = FALSE;
    }

    return res;
}
//-------------------------------[end]------------------------------------------

//---------------------------[sent packet]--------------------------------------
uint8_t CC1100::sent_packet(uint8_t my_addr, uint8_t rx_addr, uint8_t *txbuffer,
                            uint8_t pktlen,  uint8_t tx_retries)
{
    uint8_t pktlen_ack, rssi, lqi;                              //default package len for ACK
    uint8_t rxbuffer[FIFOBUFFER];
    uint8_t tx_retries_count = 0;
    uint8_t from_sender;
    uint16_t ackWaitCounter = 0;

    if(pktlen > (FIFOBUFFER - 1))
    {
        printf("ERROR: package size overflow\r\n");
        return FALSE;
    }

    tx_payload_burst(my_addr, rx_addr, txbuffer, pktlen);   //loads the data in cc1100 buffer
    transmit();                                             //sents data over air
    receive();                                              //receive mode
    return TRUE;
}
//-------------------------------[end]------------------------------------------

//--------------------------[sent ACKNOWLEDGE]------------------------------------
void CC1100::sent_acknowledge(uint8_t my_addr, uint8_t tx_addr)
{
    uint8_t pktlen = 0x06;                                      //complete Pktlen for ACK packet
    uint8_t tx_buffer[0x06];                                    //tx buffer array init

    tx_buffer[3] = 'A'; tx_buffer[4] = 'c'; tx_buffer[5] = 'k'; //fill buffer with ACK Payload

    tx_payload_burst(my_addr, tx_addr, tx_buffer, pktlen);      //load payload to CC1100
    transmit();                                                 //sent package over the air
    receive();                                                  //set CC1100 in receive mode

    if(debug_level > 0){                                        //debut output
        printf("Ack_sent!\r\n");
    }
}
//-------------------------------[end]------------------------------------------
//----------------------[check if Packet is received]---------------------------
uint8_t CC1100::packet_available()
{
    if(digitalRead(GDO2) == TRUE)                           //if RF package received
    {
        if(spi_read_register(IOCFG2) == 0x06)               //if sync word detect mode is used
        {
            while(digitalRead(GDO2) == TRUE){               //wait till sync word is fully received
                ;//printf("!\r\n");
            }                                                  //for sync word receive
        }

        if(debug_level > 0){
            printf("Pkt->:\r\n");
        }

        return TRUE;
    }
    return FALSE;
}
//-------------------------------[end]------------------------------------------

//------------------[check Payload for ACK or Data]-----------------------------
uint8_t CC1100::get_payload(uint8_t rxbuffer[], uint8_t &pktlen, uint8_t &my_addr,
                            uint8_t &sender, int8_t &rssi_dbm, uint8_t &lqi)
{
    uint8_t crc;

    rx_fifo_erase(rxbuffer);                               //delete rx_fifo bufffer

    if(rx_payload_burst(rxbuffer, pktlen) == FALSE)        //read package in buffer
    {
        rx_fifo_erase(rxbuffer);                           //delete rx_fifo bufffer
        return FALSE;                                    //exit
    }
    else
    {
        printf("RX\r\n");
        //return TRUE;
        
        my_addr = rxbuffer[1];                             //set receiver address to my_addr
        sender = rxbuffer[2];

        rssi_dbm = rssi_convert(rxbuffer[pktlen + 1]); //converts receiver strength to dBm
        lqi = lqi_convert(rxbuffer[pktlen + 2]);       //get rf quialtiy indicator
        crc = check_crc(lqi);                          //get packet CRC

        
        if(debug_level > 0){                           //debug output messages

            printf("RX_FIFO:");
            for(uint8_t i = 0 ; i < pktlen + 1; i++)   //showes rx_buffer for debug
            {
                printf("0x%02X ", rxbuffer[i]);
            }
            printf("| 0x%02X 0x%02X |", rxbuffer[pktlen+1], rxbuffer[pktlen+2]);
            printf("\r\n");

            printf("RSSI:%d ", rssi_dbm);
            printf("LQI:");printf("0x%02X ", lqi);
            printf("CRC:");printf("0x%02X ", crc);
            printf("\r\n");
        }
        
     return TRUE;
    }

}
//-------------------------------[end]------------------------------------------

//-------------------------[check ACKNOWLEDGE]------------------------------------
uint8_t CC1100::check_acknowledge(uint8_t *rxbuffer, uint8_t pktlen, uint8_t sender, uint8_t my_addr)
{
    int8_t rssi_dbm;
    uint8_t crc, lqi;

    if((pktlen == 0x05 && \
        rxbuffer[1] == my_addr || rxbuffer[1] == BROADCAST_ADDRESS) && \
        rxbuffer[2] == sender && \
        rxbuffer[3] == 'A' && rxbuffer[4] == 'c' && rxbuffer[5] == 'k')   //acknowledge received!
        {
            if(rxbuffer[1] == BROADCAST_ADDRESS){                           //if receiver address BROADCAST_ADDRESS skip acknowledge
                if(debug_level > 0){
                    printf("BROADCAST ACK\r\n");
                }
                return FALSE;
            }
            rssi_dbm = rssi_convert(rxbuffer[pktlen + 1]);
            lqi = lqi_convert(rxbuffer[pktlen + 2]);
            crc = check_crc(lqi);

            if(debug_level > 0){
                printf("ACK! ");
                printf("RSSI:%i ",rssi_dbm);
                printf("LQI:0x%02X ",lqi);
                printf("CRC:0x%02X\r\n",crc);
            }
            return TRUE;
        }
    return FALSE;
}
//-------------------------------[end]------------------------------------------

//------------[check if Packet is received within defined time in ms]-----------
uint8_t CC1100::wait_for_packet(uint16_t milliseconds)
{
    for(uint16_t i = 0; i < milliseconds; i++)
        {
            delay(1);                 //delay till system has data available
            if (packet_available())
            {
                return TRUE;
            }
        }
    return FALSE;
}
//-------------------------------[end]------------------------------------------

//--------------------------[tx_fifo_erase]-------------------------------------
void CC1100::tx_fifo_erase(uint8_t *txbuffer)
{
    memset(txbuffer, 0, sizeof(FIFOBUFFER));  //erased the TX_fifo array content to "0"
}
//-------------------------------[end]------------------------------------------

//--------------------------[rx_fifo_erase]-------------------------------------
void CC1100::rx_fifo_erase(uint8_t *rxbuffer)
{
    memset(rxbuffer, 0, sizeof(FIFOBUFFER)); //erased the RX_fifo array content to "0"
}
//-------------------------------[end]------------------------------------------

//------------------------[set CC1100 address]----------------------------------
void CC1100::set_myaddr(uint8_t addr)
{
    spi_write_register(ADDR,addr);          //stores MyAddr in the CC1100
}
//-------------------------------[end]------------------------------------------

//---------------------------[set channel]--------------------------------------
void CC1100::set_channel(uint8_t channel)
{
    spi_write_register(CHANNR,channel);   //stores the new channel # in the CC1100

    return;
}
//-------------------------------[end]------------------------------------------

void CC1100::set_mode(uint8_t mode)
{

    spi_write_burst(WRITE_BURST,STUDIO_REGISTERS,CFG_REGISTER);
    
    return;
}
//------------------------------------------end]-----------------------------------

//--------------------------[rssi_convert]--------------------------------------
int8_t CC1100::rssi_convert(uint8_t Rssi_hex)
{
    int8_t rssi_dbm;
    int16_t Rssi_dec;

    Rssi_dec = Rssi_hex;        //convert unsigned to signed

    if(Rssi_dec >= 128){
        rssi_dbm=((Rssi_dec-256)/2)-RSSI_OFFSET_868MHZ;
    }
    else{
        if(Rssi_dec<128){
            rssi_dbm=((Rssi_dec)/2)-RSSI_OFFSET_868MHZ;
        }
    }
    return rssi_dbm;
}
//-------------------------------[end]------------------------------------------

//----------------------------[lqi convert]-------------------------------------
uint8_t CC1100::lqi_convert(uint8_t lqi)
{
    return (lqi & 0x7F);
}
//-------------------------------[end]------------------------------------------

//----------------------------[check crc]---------------------------------------
uint8_t CC1100::check_crc(uint8_t lqi)
{
    return (lqi & 0x80);
}
//-------------------------------[end]------------------------------------------

//|==================== SPI Initialisation for CC1100 =========================|
void CC1100::spi_begin(void)
{
     int x = 0;
     //printf ("init SPI bus... ");
     if ((x = wiringPiSPISetup (0, 8000000)) < 0)  //4MHz SPI speed
     {
          if(debug_level > 0){
          printf ("ERROR: wiringPiSPISetup failed!\r\n");
          }
     }
     else{
          //printf ("wiringSPI is up\r\n");
          }
}
//------------------[write register]--------------------------------
void CC1100::spi_write_register(uint8_t spi_instr, uint8_t value)
{
     uint8_t tbuf[2] = {0};
     tbuf[0] = spi_instr | WRITE_SINGLE_BYTE;
     tbuf[1] = value;
     uint8_t len = 2;
     wiringPiSPIDataRW (0, tbuf, len) ;

     return;
}
//|============================ Ein Register lesen ============================|
uint8_t CC1100::spi_read_register(uint8_t spi_instr)
{
     uint8_t value;
     uint8_t rbuf[2] = {0};
     rbuf[0] = spi_instr | READ_SINGLE_BYTE;
     uint8_t len = 2;
     wiringPiSPIDataRW (0, rbuf, len) ;
     value = rbuf[1];
     //printf("SPI_arr_0: 0x%02X\n", rbuf[0]);
     //printf("SPI_arr_1: 0x%02X\n", rbuf[1]);
     return value;
}
//|========================= ein Kommando schreiben ========================|
void CC1100::spi_write_strobe(uint8_t spi_instr)
{
     uint8_t tbuf[1] = {0};
     tbuf[0] = spi_instr;
     //printf("SPI_data: 0x%02X\n", tbuf[0]);
     wiringPiSPIDataRW (0, tbuf, 1) ;
 }
//|======= Mehrere hintereinanderliegende Register auf einmal lesen =======|
void CC1100::spi_read_burst(uint8_t spi_instr, uint8_t *pArr, uint8_t len)
{
     uint8_t rbuf[len + 1];
     rbuf[0] = spi_instr | READ_BURST;
     wiringPiSPIDataRW (0, rbuf, len + 1) ;
     for (uint8_t i=0; i<len ;i++ )
     {
          pArr[i] = rbuf[i+1];
          //printf("SPI_arr_read: 0x%02X\n", pArr[i]);
     }
}
//|======= Mehrere hintereinanderliegende Register auf einmal schreiben =======|
void CC1100::spi_write_burst(uint8_t spi_instr, uint8_t *pArr, uint8_t len)
{
     uint8_t tbuf[len + 1];
     tbuf[0] = spi_instr | WRITE_BURST;
     for (uint8_t i=0; i<len ;i++ )
     {
          tbuf[i+1] = pArr[i];
          //printf("SPI_arr_write: 0x%02X\n", tbuf[i+1]);
     }
     wiringPiSPIDataRW (0, tbuf, len + 1) ;
}
//|================================= END =======================================|
