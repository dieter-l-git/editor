#include <stdlib.h>
extern "C" {
 #include "openplc.h"
}
#include "Arduino.h"
#include "../examples/Baremetal/defines.h"

// OpenPLC HAL for KC868-A6 from KINKONY
// 2024 by Dieter Lambrecht
/******************PINOUT CONFIGURATION***************************************************************************
Digital In:  P0, P1, P2, P3, P4, P5, P6, P7                                                     (%IX0.0 - %IX0.7)

Digital Out: P0, P1, P2, P3, P4, P5                                                             (%QX0.0 - %QX0.5)

Analog In:   36, 39, 34, 35                                                                     (%IW0 - %IW3)

Analog Out:  26, 25                                                                             (%QW0 - %QW1)

Vendor Documentation:   https://www.kincony.com/esp32-6-channel-relay-module-kc868-a6.html
                        https://www.kincony.com/kc868-a6-hardware-design-details.html
*****************************************************************************************************************/


//#define HASDISPLAY
//#define SERIALDEBUG

#include <PCF8574.h>
#ifdef HASDISPLAY 
#include <SSD1306Wire.h>
#include "../examples/Baremetal/additional/font2.h"
#endif


//---- Pin definitions and other defines for the KC868-A6 board -----
#define ANALOG_A1         36      //--- Analog Inputs
#define ANALOG_A2         39
#define ANALOG_A3         34
#define ANALOG_A4         35

#define ONEWIRE_1         32      //--- OneWire pins
#define ONEWIRE_2         33

#define DAC1_OUT          26      //--- Analog Outputs
#define DAC2_OUT          25

#define I2C_SDA           4       //--- I2C pins
#define I2C_SCL           15

#define RS485_RXD         14      //--- RS485 pins
#define RS485_TXD         27

#define RS232_RXD         16      //--- DB9 Serial pins
#define RS232_TXD         17

#define RS232_PCB_RXD     13      //--- Onboard Serial pins
#define RS232_PCB_TXD     12

#define SPI_CS            5       //--- SPI Signals
#define SPI_MOSI          23
#define SPI_MISO          19
#define SPI_SCK           18      

#define LORA_RST          21      //--- RST Signal for Lora Module
#define LORA_DIO          2       //--- DIO Signal for Lora Module

#define NRF_CE            22      //--- CE Signal for nRF24L01 Module

#define WIFIRESET_PIN     0       //--- Wifi reset pin

#define I2C_ADDR_INPUT    0x22    //--- Addr of Input PCF8574
#define I2C_ADDR_RELAY    0x24    //--- Addr of Relay PCF8574
#define I2C_ADDR_DISPLAY  0x3C    //--- Addr of SSD1306 OLED Display

#define OLED_RESET        -1      //--- no reset pin 
#define OLED_WIDTH        128     //--- OLED display width, in pixels
#define OLED_HEIGHT       64      //--- OLED display height, in pixels

//--- define the correct settings for the RS485/ModBusRTU
#ifndef MBSERIAL_TXPIN                  
    #define MBSERIAL_TXPIN -1     //--- not needed for the driver chip on the board
#endif
#ifndef MBSERIAL_IFACE
    #define MBSERIAL_IFACE Serial2
#endif
#ifndef MBSERIAL_BAUD
    #define MBSERIAL_BAUD 57600
#endif
#ifndef MBSERIAL_SLAVE
    #define MBSERIAL_SLAVE 1
#endif



// Create the I/O pin masks (defined within editor GUI when compiling for board)
uint8_t pinMask_DIN[] = {PINMASK_DIN};
uint8_t pinMask_AIN[] = {PINMASK_AIN};
uint8_t pinMask_DOUT[] = {PINMASK_DOUT};
uint8_t pinMask_AOUT[] = {PINMASK_AOUT};

//---- define Input and Relays PCF8574
PCF8574 D_INPUTS(I2C_ADDR_INPUT,I2C_SDA,I2C_SCL);
PCF8574 D_RELAYS(I2C_ADDR_RELAY,I2C_SDA,I2C_SCL);

//---- define Display
#ifdef HASDISPLAY
#define SCREEN_DURATION 5000
long timeSinceLastScreenSwitch = 0;
uint8_t statusflag = 0;
uint8_t ScreenMode = 0;
SSD1306Wire display(I2C_ADDR_DISPLAY, I2C_SDA,I2C_SCL);
void drawLogo();
void drawOutputState(uint8_t outputnumber, uint8_t value);
void drawInputState(uint8_t inputnumber, uint8_t value);
void drawStatus();
#endif

void hardwareInit()
{   
    #ifdef SERIALDEBUG
    //--- Init USB serial for debugging
    Serial.begin(115200);
    Serial.println();
    Serial.println("This is KC868-A6 with < > OPENPLC");
    //Serial.println(sizeof(screens));
    //Serial.println(sizeof(Screen));
    //Serial.println(screenLength);
    #endif

    //--- Init RS485-ModBus
    MBSERIAL_IFACE.begin(MBSERIAL_BAUD);
    //MBSERIAL_IFACE.println("This is KC868-A6 with < > OPENPLC");

    //--- Init the Display
    #ifdef HASDISPLAY
    display.init();
    display.flipScreenVertically();
    display.clear();
    drawLogo();
    display.display();
    #endif

    //--- wifi reset button
    pinMode(WIFIRESET_PIN, INPUT_PULLUP);

    //--- Set the pins from the Input-PCF8574 to INPUT
    for (int i = 0; i < NUM_DISCRETE_INPUT; i++){
        D_INPUTS.pinMode(i, INPUT);
    }
    D_INPUTS.begin();
    //--- Set the pinMode for the Analog Inputs    
    for (int i = 0; i < NUM_ANALOG_INPUT; i++)
    {
        pinMode(pinMask_AIN[i], INPUT);
    }
    //--- Set the pins from the Relay-PCF8574 to OUTPUT and HIGH
    for (int i = 0; i < NUM_DISCRETE_OUTPUT; i++){
        D_RELAYS.pinMode(i, OUTPUT);
        //--- relays are active LOW
        D_RELAYS.digitalWrite(i, HIGH);
    }
    D_RELAYS.begin();

    for (int i = 0; i < NUM_ANALOG_OUTPUT; i++)
    {
        ;   // Do nothing as we use the DAC outputs to get 0-10V
    }
}



void updateInputBuffers()
{
    for (int i = 0; i < NUM_DISCRETE_INPUT; i++)
    {
        if (bool_input[i/8][i%8] != NULL) {
            *bool_input[i/8][i%8] = D_INPUTS.digitalRead(pinMask_DIN[i]);
        }
            
    }
    #ifdef HASDISPLAY
    if (statusflag == 1) drawStatus();
    #endif

    for (int i = 0; i < NUM_ANALOG_INPUT; i++)
    {
        if (int_input[i] != NULL)
            *int_input[i] = (analogRead(pinMask_AIN[i]) * 64);
    }
}

void updateOutputBuffers()
{   
#ifdef HASDISPLAY
    if (millis() - timeSinceLastScreenSwitch > SCREEN_DURATION) {
        ScreenMode = (ScreenMode + 1)  % 2;
        Serial.println(ScreenMode);
        timeSinceLastScreenSwitch = millis();
        //rssisample = WiFi.RSSI();
       
        if (ScreenMode == 0){
            display.clear();
            drawLogo();
            display.display();
        } else {
            statusflag=1;
            /*
            display.clear();
            drawStatus();
            display.display();
            */
        }
    }
#endif

    for (int i = 0; i < NUM_DISCRETE_OUTPUT; i++)
    {
        if (bool_output[i/8][i%8] != NULL){
            D_RELAYS.digitalWrite(pinMask_DOUT[i], !(*bool_output[i/8][i%8]));
        }
            
    }
    #ifdef HASDISPLAY
    if (statusflag == 1) drawStatus();
    #endif

    for (int i = 0; i < NUM_ANALOG_OUTPUT; i++)
    {
        if (int_output[i] != NULL)
        {
            dacWrite(pinMask_AOUT[i], (*int_output[i] / 256));
        }
    }
}

#ifdef HASDISPLAY
void drawLogo() {
  statusflag = 0;  
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(DejaVu_Sans_Bold_18);
  display.drawString(0, 20, "< >");
  display.setFont(DejaVu_Sans_16);
  display.drawString(34, 22, " OPEN");
  display.setFont(DejaVu_Sans_Bold_16);
  display.drawString(86, 22, "PLC");
}

void drawOutputState(uint8_t outputnumber, uint8_t value){
  if (value != HIGH) {  //Inverted state
    display.drawRect(40 + (outputnumber*10), 50 , 8, 8);
  } else {
    display.fillRect(40 + (outputnumber*10), 50 , 8, 8);
  } 
}

void drawInputState(uint8_t inputnumber, uint8_t value){
  if (value != HIGH) {
    display.drawCircle(44 + (inputnumber*10), 34 , 4);
  } else {
    display.fillCircle(44 + (inputnumber*10), 34 , 4);
  } 
}

void drawStatus(){
  //statusflag = 1;
  display.clear();
  display.setFont(DejaVu_Sans_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawHorizontalLine(0, 25, 128);
  display.drawString(0,29,"Inputs:");
  for (int i = 0; i < NUM_DISCRETE_INPUT; i++){
    drawInputState(pinMask_DIN[i], !(*bool_input[i/8][i%8]));
  }
  display.drawHorizontalLine(0, 44, 128);
  display.drawString(0,48,"Relays:");
  for (int i = 0; i < NUM_DISCRETE_OUTPUT; i++){
    drawOutputState(pinMask_DOUT[i], *bool_output[i/8][i%8]);
  }
  display.drawHorizontalLine(0, 63, 128);
  
  display.display();

}

#endif