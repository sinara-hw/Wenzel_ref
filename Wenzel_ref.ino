/*
  Board: Wenzel rev 1.1
*/

/**********************************************************************/

String FW_VERSION = "1.0"; // current revision number firmware
String HW_VERSION = "1.1"; // current revision number hardware

#define PLL_REF_FREQ_MHZ  (100) 
#define GPSDO_LOOPBACK    (1)

/**********************************************************************/

#include <SPI.h>

#include <SoftWire.h>
#include <SoftwareSerial.h>
#define IC20_ADDR 0x68 // default IC20 address

/**********************************************************************/

/*
  Hardware nets:
  A5  SCL (IC20)
  A4  SDA (IC20)
  A2  SPEC/GPSDOn
  D16 PLL_MOSI
  D15 PLL_SCK
  D14 GND (PLL_MISO)
  D11 PLL_LE
  D10 RGMII_MDC     (When GPSDO is used: UART RX (GPSDO TX))
  D9  RGMII_MDIO    (When GPSDO is used: UART TX (GPSDO RX))
  D8  RGMII_RSTn
  D4  PLL_LOCK      (Input, High when PLL locked)
  D3  OCXO_READY    (Input, High when OCXO heating done)
  D2  SYNCn         (Output, toggle once locked)
  D1  PLL_CTRL_TX   (Default serial, UART to EEM connector)
  D0  PLL_CTRL_RX   (Default serial, UART to EEM connector)
*/

const uint8_t Pin__PLL_LE       = 11;
const uint8_t Pin__RGMII_MDC    = 10;     
const uint8_t Pin__RGMII_MDIO   = 9;
const uint8_t Pin__RGMII_RSTn   = 8;
const uint8_t Pin__PLL_LOCK     = 4;
const uint8_t Pin__OCXO_READY   = 3;
const uint8_t Pin__SYNCn        = 2;
const uint8_t Pin__SPEC_GPSDOn  = A2; 
const uint8_t Pin__IC20_SDA     = A4; // not a hardware I2C
const uint8_t Pin__IC20_SCL     = A5; // not a hardware I2C

bool ShieldDetected_GPSDO;
bool ShieldDetected_SPEC;
bool PLL_LockState;
bool OCXO_ReadyState;

SoftwareSerial * GPSDO_SoftSerial;


// GPSDO Buffering
// all messages from GPSDO are passed through wenzel, with 'GPSDO' prefix
#define GPSDO_BUFFER_SIZE     (1024)
struct{
  // circular buffer to process GPSDO outputs
  volatile char       Buffer[GPSDO_BUFFER_SIZE];
  volatile uint16_t   Buffer_head   = 0;
  volatile uint16_t   Buffer_tail   = 0;
  volatile uint16_t   Buffer_size   = 0;
  volatile bool       Msg_Complete  = false;
  // to differentiate GPSDO/Wenzel messages
  char                Prefix[6]     = {'G','P','S','D','O','~'};
  uint8_t             Prefix_Len    = 6;
  uint16_t            PrintCnt      = 0;
} GPSDO_Ctrl;

// Serial Buffering
#define SERIAL_BUFFER_SIZE    (128)
struct{
  char          Buffer[SERIAL_BUFFER_SIZE];
  uint16_t      Buffer_size   = 0;
  char          BufChar;
} Serial_Ctrl;

/**********************************************************************/

ISR(TIMER1_OVF_vect){ // triggered every 400us (once per ~4bits in 9600 baud)
  static uint8_t inactivity_timer = 0;
  // Software serial has extremely small buffer, and is horribly slow
  // this is a countermeasure
  if(GPSDO_SoftSerial != NULL && GPSDO_SoftSerial->available() > 0 ){
    GPSDO_Ctrl.Buffer[GPSDO_Ctrl.Buffer_tail] = GPSDO_SoftSerial->read();
    if(GPSDO_Ctrl.Buffer_size < GPSDO_BUFFER_SIZE){ // overflow protection
      ++GPSDO_Ctrl.Buffer_tail;
      ++GPSDO_Ctrl.Buffer_size;
      if(GPSDO_Ctrl.Buffer_tail >= GPSDO_BUFFER_SIZE){
        GPSDO_Ctrl.Buffer_tail = 0;
      }
    }
    inactivity_timer = 0;
  }else{
    ++inactivity_timer;

    if( inactivity_timer == 150){ // nothing received for 60ms, assume free time to process
      GPSDO_Ctrl.Msg_Complete = true;
    }else if(inactivity_timer > 150){
      // keep it from overflowing and re-triggering
      --inactivity_timer;
    }
  }
}

/**********************************************************************/

uint8_t Configure_IC20(void){
  // Overwrite IC20 registers via I2C (MPM54304GMN-XXXX)
  // prototype batch had -0002 instead of -0001 on IC20
  // Difference on regs 0x4E and 0x4F
  bool Rewrite_registers = false;
  uint8_t TimeoutCounter;

  // start I2C
  SoftWire sw(Pin__IC20_SDA, Pin__IC20_SCL);
  char swTxBuffer[16];
  char swRxBuffer[16];
  int reg;
  sw.setTxBuffer(swTxBuffer, sizeof(swTxBuffer));
  sw.setRxBuffer(swRxBuffer, sizeof(swRxBuffer));
  sw.setDelay_us(5);
  sw.setTimeout(1000);
  sw.begin();

  // Read MTP registers 0x4E 0x4F
  sw.beginTransmission(IC20_ADDR);
  sw.write(0x4E);
  sw.endTransmission(false);
  sw.requestFrom(IC20_ADDR, 1 , true);
  reg = sw.read();
  if( reg != 0x40 ) Rewrite_registers = true; // 0x0E/0x4E == 40h
  
  sw.beginTransmission(IC20_ADDR);
  sw.write(0x4F);
  sw.endTransmission(false);
  sw.requestFrom(IC20_ADDR, 1 , true);
  reg = sw.read();
  if( reg != 0x01 ) Rewrite_registers = true; // 0x0F/0x4F == 01h

  if (Rewrite_registers){
    // If still as -0002 variant: write I2C registers

    // double check reg 0x4F for sanity
    sw.beginTransmission(IC20_ADDR);
    sw.write(0x4F);
    sw.endTransmission(false);
    sw.requestFrom(IC20_ADDR, 1 , true);
    reg = sw.read();

    if (reg != 0x01){
      // 0x0E 40h
      reg = 0x40;
      sw.beginTransmission(IC20_ADDR);
      sw.write(0x0E);
      sw.write(reg);
      sw.endTransmission();

      // 0x0F 01h
      reg = 0x01;
      sw.beginTransmission(IC20_ADDR);
      sw.write(0x0F);
      sw.write(reg);
      sw.endTransmission();
  
      // Read I2C registers to confirm
      sw.beginTransmission(IC20_ADDR);
      sw.write(0x0E);
      sw.endTransmission(false);
      sw.requestFrom(IC20_ADDR, 1 , true);
      reg = sw.read();

      if (reg != 0x40){
        return 1; // error
      }

      sw.beginTransmission(IC20_ADDR);
      sw.write(0x0F);
      sw.endTransmission(false);
      sw.requestFrom(IC20_ADDR, 1 , true);
      reg = sw.read();

      if (reg != 0x01){
        return 1; // error
      }
      
      // Write MTP
      // - Set MTP_Program bit to 1
      // - Check MTP burning power supply, must be above 5.1V
      // - perform write, there will be a 100ms delay/timeout
      //   IC20 will clear MTP_Program bit
      sw.beginTransmission(IC20_ADDR);
      sw.write(0x0E);
      sw.endTransmission(false);
      sw.requestFrom(IC20_ADDR, 1 , true);
      reg = sw.read();

      reg |= (1<<5);
      sw.beginTransmission(IC20_ADDR);
      sw.write(0x0E);
      sw.write(reg);
      sw.endTransmission();
      
      delay(200);

      TimeoutCounter = 20;
      while(true){
        sw.beginTransmission(IC20_ADDR);
        sw.write(0x0E);
        sw.endTransmission(false);
        sw.requestFrom(IC20_ADDR, 1 , true);
        reg = sw.read();

        if( (reg & (1<<5)) == 0 ){
          break;
        }else{
          if( TimeoutCounter == 0 ){
            return 1; // error
          }
          TimeoutCounter -= 1;
          delay(100);
        }
      }
    }else{
      // Registers correct, do nothing.
    }
  }else{
    // Registers correct, do nothing.
  }

  return 0;
}

uint8_t Configure_PLL(void){
  // PLL Setup (ADF4002BCPZ-RL7)
  /*  24bit input shift reg, MSB first
  *   data latched on CLK rising edge, CLK low in stby (CPOL 0, CPHA 0 = SPI MODE0)
  *   C2/C1 control target reg (last two transmitted bits - C1 is last)
  *     0 0 -R Counter
  *     0 1 -N Counter
  *     1 0 -Function Latch
  *     1 1 -Initialization Latch
  */
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);//divide the clock by 8

  uint8_t PLLreg_R[3];
  uint8_t PLLreg_N[3];
  uint8_t PLLreg_F[3];
  uint8_t PLLreg_I[3];
  
  // endiannes fixed by reordering [2/1/0] to [0/1/2] below
  PLLreg_R[0] = 0x10; // LDP set, APB2/APB1 2.9ns
  PLLreg_R[1] = 0x00; // 
  PLLreg_R[2] = 0x04; // R = 1

  #if (PLL_REF_FREQ_MHZ == (10))    // 10Mhz rf
    PLLreg_N[0] = 0x00; // G1 = 0
    PLLreg_N[1] = 0x01; // N=1
    PLLreg_N[2] = 0x01;
  #elif (PLL_REF_FREQ_MHZ == (100))  // 100Mhz rf
    PLLreg_N[0] = 0x00; // G1 = 0
    PLLreg_N[1] = 0x0A; // N=10
    PLLreg_N[2] = 0x01;
  #else
    #error "UNSUPPORTED"
  #endif 
  
  PLLreg_F[0] = 0x1F; // PD2 = 0, CP6-2 = 11111
  PLLreg_F[1] = 0x80; // CPI1=1, TC4-1=0000, F4/F5 = 00 (fastlock off), F3=0
  PLLreg_F[2] = 0xD2; // F2 = 1, M3/2/1 = 101, PD1 = 0, F1=0

  PLLreg_I[0] = 0x1F; // PD2 = 0, CP6-2 = 11111
  PLLreg_I[1] = 0x80; // CPI1=1, TC4-1=0000, F4/F5 = 00 (fastlock off), F3=0
  PLLreg_I[2] = 0xD3; // F2 = 1, M3/2/1 = 101, PD1 = 0, F1=0


  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(Pin__PLL_LE, LOW);
  delay(2);
  SPI.transfer(PLLreg_I, 3);
  delay(2);
  digitalWrite(Pin__PLL_LE, HIGH);
  SPI.endTransaction();

  delay(2);

  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(Pin__PLL_LE, LOW);
  delay(2);
  SPI.transfer(PLLreg_F, 3);
  delay(2);
  digitalWrite(Pin__PLL_LE, HIGH);
  SPI.endTransaction();
  
  delay(2);

  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(Pin__PLL_LE, LOW);
  delay(2);
  SPI.transfer(PLLreg_R, 3);
  delay(2);
  digitalWrite(Pin__PLL_LE, HIGH);
  SPI.endTransaction();

  delay(2);

  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(Pin__PLL_LE, LOW);
  delay(2);
  SPI.transfer(PLLreg_N, 3);
  delay(2);
  digitalWrite(Pin__PLL_LE, HIGH);
  SPI.endTransaction();

  delay(10);

  // Initial SYNCn pulse
  digitalWrite(Pin__SYNCn, LOW);
  delay(10);
  digitalWrite(Pin__SYNCn, HIGH);

  return 0;
}

uint8_t DetectShield(void){
  static int ADC_val;
  static uint8_t Values_Changed;
  analogRead(Pin__SPEC_GPSDOn); // dummy read
  ADC_val = analogRead(Pin__SPEC_GPSDOn);
  // external 5k pulldown default, GND /w GPSDO, 3V3 /w SPEC 
  // internal 20-50k pullup, default float: 0.3 to 0.66V  (dec 93 to 205) 
  Values_Changed = 0;
  if( ADC_val <= 60 ){
    if( ShieldDetected_GPSDO == false ) Values_Changed = 1;
    ShieldDetected_GPSDO = true;
    ShieldDetected_SPEC = false;
  }else if( ADC_val >= 850 ){
    if( ShieldDetected_SPEC == false ) Values_Changed = 1;
    ShieldDetected_GPSDO = false;
    ShieldDetected_SPEC = true;
  }else{
    // float expected 120-264, in practice goes up to 550
    if( (ShieldDetected_GPSDO == true) || (ShieldDetected_SPEC == true) ) Values_Changed = 1;
    ShieldDetected_GPSDO = false;
    ShieldDetected_SPEC = false;
  }

  // Reconfigure GPIOs as needed
  if( Values_Changed ){
    if( ShieldDetected_GPSDO ){
      pinMode(Pin__RGMII_MDIO,  OUTPUT); 

      // Start soft serial
      if( GPSDO_SoftSerial ){
        delete GPSDO_SoftSerial;
        GPSDO_SoftSerial = NULL;
      }
      GPSDO_SoftSerial = new SoftwareSerial(Pin__RGMII_MDC, Pin__RGMII_MDIO);
      GPSDO_SoftSerial->begin(9600);
    }else if( ShieldDetected_SPEC ){
      pinMode(Pin__RGMII_MDIO,  INPUT); 
      // stop soft serial
      if( GPSDO_SoftSerial ){
        delete GPSDO_SoftSerial;
        GPSDO_SoftSerial = NULL;
      }
    }else{
      pinMode(Pin__RGMII_MDIO,  INPUT); 
      // stop soft serial
      if( GPSDO_SoftSerial ){
        delete GPSDO_SoftSerial;
        GPSDO_SoftSerial = NULL;
      }
    }
  }
  return Values_Changed;
}

/**********************************************************************/

void setup (void){
  PLL_LockState = false;
  OCXO_ReadyState = false;
  ShieldDetected_GPSDO = false;
  ShieldDetected_SPEC = false;
  GPSDO_SoftSerial = NULL;

  // GPIO Setup
  pinMode(Pin__PLL_LE,      OUTPUT);
  pinMode(Pin__RGMII_MDC,   INPUT); 
  pinMode(Pin__RGMII_MDIO,  INPUT); 
  pinMode(Pin__RGMII_RSTn,  INPUT); 
  pinMode(Pin__PLL_LOCK,    INPUT); 
  pinMode(Pin__OCXO_READY,  INPUT); 
  pinMode(Pin__SYNCn,       OUTPUT); 
  pinMode(Pin__SPEC_GPSDOn, INPUT_PULLUP); 

  digitalWrite(Pin__PLL_LE, HIGH);
  digitalWrite(Pin__SYNCn,  HIGH);

  analogReference(INTERNAL); // 2.56V

  // Configure board hardware
  uint8_t Config_result_IC20  = Configure_IC20();
  uint8_t Config_result_PLL   = Configure_PLL();
  DetectShield();

  // USB Setup
  Serial.begin(9600);
  // USART Setup
  Serial1.begin(9600);
  
  // Start TIM1 to capture GPSDO transmissions
  //  clocked by OCXO (8MHz)        
  //  counts to 3200 for easier calc (overflow every 400us)
  //  FastPWM to trigger OVF IRQ at OCR1A
  TCCR1A = 0;    // reset timer 1            
  TCCR1B = 0;  
  // configure timer1 for Fast PWM (mode 15), output disconnected
  TCCR1A |= ((1 << WGM11) | (1 << WGM10));
  TCCR1B |= ((1 << WGM13) | (1 << WGM12)); 
  OCR1A = 3199; 
  TIMSK1 |= ((1 << TOIE1)); //interrupt on Capture (1PPS) and overflow (timekeeping)  
  TCCR1B |= ((1 << CS10)); // start timer 1
  sei();

  // Serial is via USB; wait for enumeration
  //while (!Serial);
  // optional connection via backplane directly to TX0/RX0
  //delay(500);

  Serial.print(F("\n\n"));
  Serial.print(F("Wenzel-class freq. reference.")); 
  Serial.print(F("\n fw ver "));
  Serial.print(FW_VERSION); 
  Serial.print(F("\n hw ver "));
  Serial.print(HW_VERSION); 
  Serial.print(F("\n\n"));

  Serial1.print(F("\n\n"));
  Serial1.print(F("Wenzel-class freq. reference.")); 
  Serial1.print(F("\n fw ver "));
  Serial1.print(FW_VERSION); 
  Serial1.print(F("\n hw ver "));
  Serial1.print(HW_VERSION); 
  Serial1.print(F("\n\n"));

  

  // report any startup errors
  if(Config_result_IC20){
    Serial.print("Errors occured during IC20 configuration (MPM54304GMN-XXXX)"); 
    Serial.print(F("\n\n"));

    Serial1.print("Errors occured during IC20 configuration (MPM54304GMN-XXXX)"); 
    Serial1.print(F("\n\n"));
  }
  if(Config_result_PLL){
    Serial.print("Errors occured during PLL configuration (ADF4002BCPZ-RL7)"); 
    Serial.print(F("\n\n"));

    Serial1.print("Errors occured during PLL configuration (ADF4002BCPZ-RL7)"); 
    Serial1.print(F("\n\n"));
  }

  // Report currently mounted shield
  if( ShieldDetected_GPSDO ){
    Serial.print(F("Shield mounted: GPSDO")); 
    Serial.print(F("\n\n"));

    Serial1.print(F("Shield mounted: GPSDO")); 
    Serial1.print(F("\n\n"));
  }else if( ShieldDetected_SPEC ){
    Serial.print(F("Shield mounted: SPEC")); 
    Serial.print(F("\nNOT YET IMPLEMENTED!")); //TODO
    Serial.print(F("\n\n"));

    Serial1.print(F("Shield mounted: SPEC")); 
    Serial1.print(F("\nNOT YET IMPLEMENTED!")); //TODO
    Serial1.print(F("\n\n"));
  }else{
    Serial.print(F("Shield mounted: None")); 
    Serial.print(F("\n\n"));

    Serial1.print(F("Shield mounted: None")); 
    Serial1.print(F("\n\n"));
  }
}

/**********************************************************************/

void loop(){
  static char tempChar;
  static uint16_t cnt;
  static uint16_t tempint;

  // Detect PLL/OCXO changes
  if( digitalRead(Pin__PLL_LOCK) ){
    PLL_LockState = true;
  }else{
    PLL_LockState = false;
  }
  if( digitalRead(Pin__OCXO_READY) ){
    OCXO_ReadyState = true;
  }else{
    OCXO_ReadyState = false;
  }

  // Detect shield being plugged/unplugged
  if( DetectShield() ){ 
    if( ShieldDetected_GPSDO ){
      Serial.print(F("Shield mounted: GPSDO")); 
      Serial.print(F("\n\n"));

      Serial1.print(F("Shield mounted: GPSDO")); 
      Serial1.print(F("\n\n"));
    }else if( ShieldDetected_SPEC ){
      Serial.print(F("Shield mounted: SPEC")); 
      Serial.print(F("\nNOT YET IMPLEMENTED!"));  //TODO
      Serial.print(F("\n\n"));
      
      Serial1.print(F("Shield mounted: SPEC")); 
      Serial1.print(F("\nNOT YET IMPLEMENTED!"));  //TODO
      Serial1.print(F("\n\n"));
    }else{
      Serial.print(F("Shield mounted: None")); 
      Serial.print(F("\n\n"));

      Serial1.print(F("Shield mounted: None")); 
      Serial1.print(F("\n\n"));
    }
  }

  // Handle shields as needed
  if( ShieldDetected_GPSDO ){
    // Process any GPSDO messages
    while( GPSDO_Ctrl.Msg_Complete && GPSDO_Ctrl.Buffer_size > 0){
      tempint = 0;
      while(tempint < GPSDO_Ctrl.Prefix_Len){
        Serial.print((char)(GPSDO_Ctrl.Prefix[tempint]));
        Serial1.print((char)(GPSDO_Ctrl.Prefix[tempint]));
        ++tempint;
      }
      // Add a tab for clarity
      Serial.print("\t");
      Serial1.print("\t");

      // pass the message
      while(1){
        tempChar = GPSDO_Ctrl.Buffer[GPSDO_Ctrl.Buffer_head];
        Serial.print(tempChar);
        Serial1.print(tempChar);
        ++GPSDO_Ctrl.Buffer_head;
        if(GPSDO_Ctrl.Buffer_head >= GPSDO_BUFFER_SIZE) GPSDO_Ctrl.Buffer_head = 0;
        --GPSDO_Ctrl.Buffer_size;
        if(GPSDO_Ctrl.Buffer_size == 0 || tempChar == '\n') break;
      }

      // check if any more messages are queued
      if(GPSDO_Ctrl.Buffer_size == 0){
        GPSDO_Ctrl.Msg_Complete = false;
        GPSDO_Ctrl.Buffer_head = GPSDO_Ctrl.Buffer_tail;
        if( tempChar != '\n' ){
          Serial.print("\n");
          Serial1.print("\n");
        }
        break;
      } 
    }
  }else if( ShieldDetected_SPEC ){
    //TODO implement
  }

  // Handle USB
  while (Serial.available()) {
    // get the new byte:
    Serial_Ctrl.BufChar = (uint8_t)Serial.read();
    if(Serial_Ctrl.Buffer_size >= SERIAL_BUFFER_SIZE) Serial_Ctrl.Buffer_size=SERIAL_BUFFER_SIZE-1; // overflow protection
    Serial_Ctrl.Buffer[Serial_Ctrl.Buffer_size] = Serial_Ctrl.BufChar;
    ++Serial_Ctrl.Buffer_size;

    // if the incoming character is a newline, parse text
    if (Serial_Ctrl.BufChar == '\n') {
      parseReceivedText();
      Serial_Ctrl.Buffer_size = 0;
    }
  }
  // Handle Serial
  while (Serial1.available()) {
    // get the new byte:
    Serial_Ctrl.BufChar = (uint8_t)Serial1.read();
    if(Serial_Ctrl.Buffer_size >= SERIAL_BUFFER_SIZE) Serial_Ctrl.Buffer_size=SERIAL_BUFFER_SIZE-1; // overflow protection
    Serial_Ctrl.Buffer[Serial_Ctrl.Buffer_size] = Serial_Ctrl.BufChar;
    ++Serial_Ctrl.Buffer_size;

    // if the incoming character is a newline, parse text
    if (Serial_Ctrl.BufChar == '\n') {
      parseReceivedText();
      Serial_Ctrl.Buffer_size = 0;
    }
  }
}

/**********************************************************************/

void parseReceivedText(void){ 
  bool GPSDO_PrefixDetected = false;
  uint8_t cnt = 0;

  // Check if 'GPSDO' prefix is present, if so pass the message
  if( Serial_Ctrl.Buffer_size > GPSDO_Ctrl.Prefix_Len ){
    GPSDO_PrefixDetected = true;
    while( cnt<GPSDO_Ctrl.Prefix_Len ){
      if(Serial_Ctrl.Buffer[cnt] != GPSDO_Ctrl.Prefix[cnt]){
        GPSDO_PrefixDetected = false;
        break;
      }
      ++cnt;
    }
  }
  if( GPSDO_PrefixDetected ){
    if( ShieldDetected_GPSDO ){
      if( Serial_Ctrl.Buffer_size == GPSDO_Ctrl.Prefix_Len+1 ){
        // Empty payload
        Serial.println("Cannot send empty message.\n");
        Serial1.println("Cannot send empty message.\n");
      }else{
        #if GPSDO_LOOPBACK
        cnt = 0;
        while( cnt<GPSDO_Ctrl.Prefix_Len ){
          Serial.print((char)(GPSDO_Ctrl.Prefix[cnt]));
          Serial1.print((char)(GPSDO_Ctrl.Prefix[cnt]));
          ++cnt;
        }
        Serial.print("\t");
        Serial1.print("\t");
        cnt = GPSDO_Ctrl.Prefix_Len;
        while(cnt < Serial_Ctrl.Buffer_size){
          Serial.print((char)(Serial_Ctrl.Buffer[cnt]));
          Serial1.print((char)(Serial_Ctrl.Buffer[cnt]));
          ++cnt;
        }
        #endif

        cnt = GPSDO_Ctrl.Prefix_Len;
        while(cnt < Serial_Ctrl.Buffer_size){
          GPSDO_SoftSerial->print((char)(Serial_Ctrl.Buffer[cnt]));
          ++cnt;
        }
        GPSDO_SoftSerial->flush();
      }
    }else{
      Serial.println("GPSDO Shield must be connected first.\n");
      Serial1.println("GPSDO Shield must be connected first.\n");
    }
  }else{
    // Message directed to Wenzel itself, decode
    switch (Serial_Ctrl.Buffer[0]){
      case 'H' :
      case 'h' : Serial_help();             break;
      case 'L' :
      case 'l' : Serial_Lockstate();        break;
      case 'O' :
      case 'o' : Serial_OCXOstate();        break;
      case 'S' :
      case 's' : Serial_Shield();           break;
      default  : Serial_ErrorMessage();     break;
    }
  }
}

void Serial_help(){
  if(Serial_Ctrl.Buffer_size > 2) Serial_ErrorMessage();
  else{
    Serial.println("\n Supported commands:");
    Serial.println("  h                 -print this help message");
    Serial.println("  l                 -return state of PLL Lock");
    Serial.println("  o                 -return state of OCXO heater");
    Serial.println("  s                 -return which shield is plugged connected");  
    Serial.println("  GPSDO~<...>       -forward commands to GPSDO shield");

    Serial1.println("\n Supported commands:");
    Serial1.println("  h                 -print this help message");
    Serial1.println("  l                 -return state of PLL Lock");
    Serial1.println("  o                 -return state of OCXO heater");
    Serial1.println("  s                 -return which shield is plugged connected");  
    Serial1.println("  GPSDO~<...>       -forward commands to GPSDO shield");
  }
}

void Serial_Lockstate(){
  if(Serial_Ctrl.Buffer_size > 2) Serial_ErrorMessage();
  else{
    if(PLL_LockState){
      Serial.print(F("PLL Locked")); 
      Serial.print(F("\n\n"));

      Serial1.print(F("PLL Locked")); 
      Serial1.print(F("\n\n"));
    }else{
      Serial.print(F("PLL NOT Locked")); 
      Serial.print(F("\n\n"));

      Serial1.print(F("PLL NOT Locked")); 
      Serial1.print(F("\n\n"));
    }
  }
}

void Serial_OCXOstate(){
  if(Serial_Ctrl.Buffer_size > 2) Serial_ErrorMessage();
  else{
    if(OCXO_ReadyState){
      Serial.print(F("OCXO Heater OFF")); 
      Serial.print(F("\n\n"));

      Serial1.print(F("OCXO Heater OFF")); 
      Serial1.print(F("\n\n"));
    }else{
      Serial.print(F("OCXO Heater ON")); 
      Serial.print(F("\n\n"));

      Serial1.print(F("OCXO Heater ON")); 
      Serial1.print(F("\n\n"));
    }
  }
}

void Serial_Shield(){
  if(Serial_Ctrl.Buffer_size > 2) Serial_ErrorMessage();
  else{
    if( ShieldDetected_GPSDO ){
      Serial.print(F("Shield mounted: GPSDO")); 
      Serial.print(F("\n\n"));

      Serial1.print(F("Shield mounted: GPSDO")); 
      Serial1.print(F("\n\n"));
    }else if( ShieldDetected_SPEC ){
      Serial.print(F("Shield mounted: SPEC")); 
      Serial.print(F("\n\n"));
      
      Serial1.print(F("Shield mounted: SPEC")); 
      Serial1.print(F("\n\n"));
    }else{
      Serial.print(F("Shield mounted: None")); 
      Serial.print(F("\n\n"));
      
      Serial1.print(F("Shield mounted: None")); 
      Serial1.print(F("\n\n"));
    }
  }
}

void Serial_ErrorMessage(){
  Serial.println("Unknown command. Type <h> for help.\n");
  
  Serial1.println("Unknown command. Type <h> for help.\n");
}  

/**********************************************************************/