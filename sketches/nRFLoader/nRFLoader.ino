// nRFLoader - a sketch to enable OTA upgrade of devices running the nRFLoader bootloader
//
// (c) 2022, Andrew Williams
// -------------------------
// Hardware Setup - I2C connection to a 2x16 LCD
// and an ESP24L01 Radio connected to SPI and pins 9&10
// Serial Port: Baud 115200
// ----------------------------------------------------
#include <SPI.h>
#include "RF24.h"
#include <printf.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Configuration Parameters
#define BAUDRATE (115200)

// Hardware configuration:
// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);
// I2C LCD adapter
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3, POSITIVE);


// Global Parameters ------------------------------------------
// ------------------------------------------------------------
#define LOG_NORMAL (0x00)
#define LOG_VERBOSE (0x01)

uint8_t gLogLevel = LOG_NORMAL;
// For easy of copying, storing the nRF address (3 bytes) as the
// three LSB of a uint32_t
typedef uint32_t tDeviceId;

// Addresses for this Device (Server) and the client device
tDeviceId addr_server= 0xC0DEC1; // Primary broadcast and receive for the nRFLoader
tDeviceId addr_p2p   = 0xC0DE02; // Pipe 2 - dedicated P2P address
tDeviceId addr_client= 0x123456; // Client device address

// nRF Radio Frequency
uint8_t gRFchan;
uint8_t gNRF_message[32];

// Timestamp for any timeout event
unsigned long gWaitTimeout=0;
unsigned long gHeartBeatTimeout=0;

// A firmware write takes 3 nRF messages to complete (setup/write/bind)
// This structure maintains a cached copy of the transaction, in case
// of failure, and a need to re-transmit.
struct sWriteCmd {
  uint8_t addrL;
  uint8_t addrH;
  bool erase;
  uint8_t csum;
  uint8_t data[32];
} gWriteCmd;

// Serial Mode..
//    IDLE  - waiting on first byte of SYNC command
//    SYNCED - waiting for packet if (0x01 .. 0x03) all else back to IDLE
//    Parsing Packet type 1 - Setup <AddrL><AddrH><csum><erase>
//    Parsing Packet type 2 - 32 bytes of payload
//    Parsing Packet type 3 - 'R' = RESET
//    Parsing Packet type 4 - Audit <AddrL><AddrH><sizeL><sizeH><csumL><csumH><store>
//    Parsing Packet type 5 - (DeviceId)
//    Parsing Packet type 42 - SYNC request .. if see SYNC cmd, reply with SYNC response

// Counter for clocking bytes in from the Serial port
uint8_t gREAD_counter = 0;

// STATE = W4SERIAL | W4NRF | MODE
#define STATE_IDLE   (0x80|0x40|0x00)
#define STATE_LOG    (0x80|0x40|0x01)
#define STATE_DEVID  (0x80|0x40|0x02)
#define STATE_SEARCH (0x00|0x40|0x03)
#define STATE_BIND   (0x00|0x40|0x04)
#define STATE_HXHDR  (0x80     |0x05)
#define STATE_HXDATA (0x80     |0x06)
#define STATE_SETUP  (0x00|0x40|0x07)
#define STATE_WRITE  (0x00|0x40|0x08)
#define STATE_COMMIT (0x00|0x40|0x09)
#define STATE_FINISH (0x80|0x00|0x0A)
#define STATE_AUDIT  (0x00|0x40|0x0B)
#define STATE_RESET  (0x80|0x00|0x0C)

uint8_t gState = STATE_IDLE;
#define W4Radio(x) ((x&0x40)>0)
#define W4Serial(x) ((x&0x80)>0)
#define mode(x)    (x&0x0F)

#define major 0
#define minor 3

// Eye Candy - idle display
char info_line[17];   // Text on 2nd line (changes every 2 seconds)
unsigned long splash_timeout = 0; // Timeout value for IDLE anmation
uint8_t phase = 0;                // Offset of string being displayed
uint8_t dir = 1;                  // Directino of travel for animation

// Splash Page: Initial strings during Arduino setup()
void splash() {
  // Splash Page
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("_nRF  WHISPERER_"));
  sprintf(info_line," LabRat  Lights ");
}

// void idleRadio()
// Common code to restore the radio to an idle state
//   Pipe 1 incoming broadcasts
//   Pipe 2 incoming P2P channel

void idleRadio() {
      radio.openReadingPipe(1,addr_server);
      radio.openReadingPipe(2,addr_p2p);
}

// Configure the NRF24L01
//
void configRadio() {
      radio.setChannel(gRFchan);
      radio.setPayloadSize(32);
      radio.setAddressWidth(3);
      radio.setAutoAck(false);
      radio.setRetries(0x0F,0x0F); // Not time critical -so wait longer
      radio.setDataRate(RF24_2MBPS);
      radio.setCRCLength(RF24_CRC_16);
      radio.setPALevel(RF24_PA_LOW);
      idleRadio();
      radio.maskIRQ(true, true, true);
}

// ** EYE CANDY ** Customer Heart Icons for the Hitachi Display
// custom characters for the heartbeat
byte Heart[] = {
  B00000,
  B01010,
  B11111,
  B11111,
  B01110,
  B00100,
  B00000,
  B00000
};

byte Heart2[] = {
  B00000,
  B01010,
  B10101,
  B10001,
  B01010,
  B00100,
  B00000,
  B00000
};


// ---------------
// Arduino Setup :
// ---------------
void setup() {
  // Serial port for communication with the python nrfLoad.py application
  Serial.begin(BAUDRATE);
  // Debugging Setup
  printf_begin();

  // I2C LCD
  lcd.begin(16,2);
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.createChar(0, Heart);  // Blinking Heart Icon
  lcd.createChar(1, Heart2);
  splash();

  // Default gRFchan - update to be an option to be passed in.
  gRFchan=82;
  addr_client = 0x00;
  gState = STATE_IDLE;

  // Setup the NRF Radio sub-system
  radio.begin();
  configRadio();
  radio.startListening(); // Note  ENRXADDR_P0 = 0
  delay(1000);
}

// Ugly Debug Code
void dumpMsg(uint8_t * msg,unsigned char num_param) {
  char tempstr[21];
  switch (num_param) {
    case 1:snprintf(tempstr,20,"%2.2X",msg[0]); break;
    case 2:snprintf(tempstr,20,"%2.2X%2.2X",msg[0],msg[1]); break;
    case 3:snprintf(tempstr,20,"%2.2X%2.2X%2.2X",msg[0],msg[1],msg[2]); break;
    case 4:snprintf(tempstr,20,"%2.2X%2.2X%2.2X%2.2X",msg[0],msg[1],msg[2],msg[3]); break;
    case 5:snprintf(tempstr,20,"%2.2X%2.2X%2.2X%2.2X%2.2X",msg[0],msg[1],msg[2],msg[3],msg[4]); break;
    case 7:snprintf(tempstr,20,"%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X",msg[0],msg[1],msg[2],msg[3],msg[4],msg[5],msg[6]); break;
    case 8:snprintf(tempstr,20,"%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X",msg[0],msg[1],msg[2],msg[3],msg[4],msg[5],msg[6],msg[7]); break;
    default: snprintf(tempstr,20,"Unsupported");break;
  }
  lcd.print(tempstr);
}

bool SendSetup(){
  uint8_t * msg = gNRF_message;
  bool retCode = false;

        // Build the message
        msg[0] = 0x80; // String to erase
        msg[1] = gWriteCmd.addrL;
        msg[2] = gWriteCmd.addrH;
        msg[3] = gWriteCmd.erase; // No ERASE

        // Send the message
        radio.stopListening(); // Ready to Write - EN_RXADDRP0 = 1
        radio.setAutoAck(0,true);

        retCode =  radio.write(msg,32); // Want to get AA working here

        // Continue listening
        radio.setAutoAck(0,false);
        radio.startListening(); // EN_RXADDRP0 = 0

        lcd.setCursor(0,1);
        if (gLogLevel == LOG_VERBOSE) {
          //0200 Erase :.0..
          //0123456789ABCDEF
          dumpMsg(&(msg[2]),1);
          dumpMsg(&(msg[1]),1);

          lcd.print(" Erase  : ");
          lcd.print(retCode);
          lcd.print(" ");
        } else {
          lcd.print(">>>             ");
        }

        return retCode;
}

bool SendReboot() {
   uint8_t * msg = gNRF_message;
   bool retCode =false;

        radio.stopListening();
        radio.setAutoAck(0,true);

        msg[0] = 0x86;

        if (radio.write(msg,32)) { // Want to get AA working here
          retCode= true;
        }

        gState = STATE_DEVID;
        radio.setAutoAck(0,false);
        radio.startListening();
        if (gLogLevel == LOG_VERBOSE) {
          lcd.setCursor(0,1);
          lcd.print("Reset           ");
        }
        return retCode;
}

bool SendWrite() {
  uint8_t * msg = gNRF_message;
  bool retCode = false;

        radio.stopListening();
        radio.setAutoAck(0,true);

        msg[0] = 0x81; // String to erase
        memcpy(&(msg[1]),gWriteCmd.data,30);

        if (radio.write(msg,32)) { // Want to get AA working here
          retCode = true;
        }
        radio.setAutoAck(0,false);
        radio.startListening();


        if (gLogLevel == LOG_VERBOSE) {
          lcd.setCursor(5,1);
          //0200 Write :.0..
          //0123456789ABCDEF
          lcd.print("Write  : ");
          lcd.print(retCode);
          lcd.print(" ");
        } else {
          lcd.setCursor(0,1);
          lcd.print("     >>>        ");
        }
        return retCode;
}

bool SendCommit() {
  uint8_t * msg = gNRF_message;
  bool retCode = false;

        radio.stopListening();
        radio.setAutoAck(0,true);

        msg[0] = 0x82;
        msg[1] = 1;
        msg[2] = gWriteCmd.csum;
        memcpy(&(msg[3]),&(gWriteCmd.data[30]),2);

        if (radio.write(msg,32)) { // Want to get AA working here
          retCode = true;
        }

        radio.setAutoAck(0,false);
        radio.startListening();

        if (gLogLevel == LOG_VERBOSE) {
          lcd.setCursor(5,1);
          //0200 Commit :.0.
          //0123456789ABCDEF
          lcd.print("Commit : ");
          lcd.print(retCode);
          lcd.print(" ");
        }else {
          lcd.setCursor(0,1);
          lcd.print("          >>>   ");
        }
        return retCode;
}

bool SendAudit() {
  uint8_t * msg = gNRF_message;
  bool retCode = false;

        radio.stopListening();
        radio.setAutoAck(0,true);

        msg[0] = 0x83;
        memcpy(&(msg[1]),gWriteCmd.data,7);

        if (radio.write(msg,32)) { // Want to get AA working here
          retCode = true;
        }

        radio.setAutoAck(0,false);
        radio.startListening();
        if (gLogLevel == LOG_VERBOSE) {
          lcd.setCursor(0,1);
          lcd.print("Audit..         ");
        } else {
          lcd.setCursor(14,1);
          lcd.print("||");
        }
        return retCode;
}

uint32_t last_beacon =0;
bool sendBeacon() {
  uint8_t * msg = gNRF_message;
  bool retCode = false;
  uint32_t now = millis();
  if (now-last_beacon>1000) {
        last_beacon= now;
        msg[0] = 0x85;
        msg[2] = millis()&0xff; // ensure unique packets

        radio.stopListening();
        radio.openWritingPipe(addr_server);
        delay(1);
        radio.setAutoAck(0,false);  // Should already be false

        gWaitTimeout=millis();
        retCode = radio.write(msg,32);

        radio.openWritingPipe(addr_client);
        radio.startListening();
  } else {
    retCode =true;
  }
  return retCode;
}


bool SendHeartBeat() {
  uint8_t * msg = gNRF_message;
  bool retCode = false;

        radio.stopListening();
        radio.setAutoAck(0,true);

        msg[0] = 0x84;
        msg[1] = 0x2A; // 42 .. the ultimate question;
        msg[2] = millis()&0xff; // ensure unique packets

        if (radio.write(msg,32)) { // Want to get AA working here
          retCode = true;
        }

        radio.setAutoAck(0,false);
        radio.startListening();

        return retCode;
}

bool sendBindRequest() {
     uint8_t * msg = gNRF_message;
     bool retCode = false;

         msg[0] = 0x87;

        // Allocate a pipe and send that address to the client
        // (for now use the default)
        // Format <0x87><DevId0><DevId1><DevId2><P2P0><P2P1><P2P2>

        memcpy(&(msg[1]),&addr_client,3);
        memcpy(&(msg[4]),&addr_p2p,3);

        msg[16]=millis()&0xff;

        radio.stopListening();   // Ready to write EN_RXADDR_P0 = 1
        radio.openWritingPipe(addr_server);
        delay(1);
        radio.setAutoAck(0,false);// As a broadcast first ...
        gWaitTimeout=millis();
        //delay(500);
        retCode = radio.write(msg,32); // Want to get AA working here

        radio.openWritingPipe(addr_client);
        radio.setAutoAck(2,true);// From now on ... BOUND to P2P
        radio.startListening(); // EN_RXADDR_P0 = 0

        lcd.setCursor(0,1);
        if(gLogLevel == LOG_VERBOSE) {
          uint32_t tempAddr;
          radio.qryAddrReg(0x0C,(char *)&tempAddr);
          lcd.print(">>");
          lcd.print(tempAddr,HEX);
        }
        return retCode;
}

// Timeout values for various states
unsigned char heart_beat_count = 0;
unsigned long serial_timeout = 0;
unsigned char retry_count = 0;
uint8_t poll_count = 0;
// --------------------------------
// NRF_GPIO handler - toggle GPIO on Rx packets
// Allow for timing calculations
// --------------------------------
uint8_t pollRadio () {
  uint8_t pipe;
  uint8_t inbuf[32];

  if (radio.available(&pipe)) {
    uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
    radio.read(inbuf, bytes);               // fetch payload from FIFO
    if ((pipe == 0)|| (pipe==7)) {
      // Ignore - we aren't listening on 0
    }

    //DEBUG CODE - Print out any received messages
    #ifdef DEBUG
        lcd.setCursor(0,1);
        lcd.print("R");
        lcd.print(pipe);
        lcd.print("[");
        dumpMsg(inbuf,5);
    #endif
    if (inbuf[0] != 0x85) {
       heart_beat_count = 0; // receive ANYTHING other than BEACON will reset the heartbeat
       gHeartBeatTimeout = millis();
    }

    switch (gState) {
      case STATE_IDLE:
        if (inbuf[0] == 0x88) { // This is a BIND request
          // To Do .. add a scrolling marquee showing device id's
        }
        break;
      case STATE_SEARCH:
        if (inbuf[0] == 0x88) { // This is a BIND request
          // Is this the device I am waiting for?
          uint32_t * tempAddr= (uint32_t *) &(inbuf[1]);
          if ((*tempAddr&0xFFFFFF) == addr_client) {
            // Device Was heard...
            sendBindRequest();
            gState = STATE_BIND;
          }
        }
        break;
      case STATE_BIND:
        if (inbuf[0] == 0x88) { // This is a BIND request
           // Display for giggles
           uint32_t * tempAddr= (uint32_t *) &(inbuf[1]);
           if ((*tempAddr&0xFFFFFF) == addr_client) {
              // Device Was heard...
              sendBindRequest();
            }
        }

        if (inbuf[0] == 0x87) { // This is a BIND reply
          gState = STATE_HXHDR;
          radio.openWritingPipe(addr_client);
          delay(1);
          Serial.write(0x01);
          gWaitTimeout=millis();
        }
        break;
      case STATE_SETUP:
        if (inbuf[0] == 0x80) { // This is a SETUP response
          if (inbuf[1] == 0x01) {
            // ACK
            gState = STATE_HXHDR; // Back to Parsing input?
            gWaitTimeout=millis();
            Serial.write(0x01);
            Serial.flush();
          } else {
            // NACK - re-send the SETUP request
            SendSetup();
          }
        }
        break;
      case STATE_WRITE:
        if (inbuf[0] == 0x81) { // This is a WRITE response
          if (inbuf[1] == 0x01) {
            // ACK
            //Build and send the COMMIT request
             gState = STATE_COMMIT;
             while (!SendCommit());
          } else {
            // NACK - re-send the SETUP request
            while (!SendWrite());
          }
        }
        break;
      case STATE_COMMIT:
        if (inbuf[0] == 0x82) { // This is a COMMIT response
          if (inbuf[1] == 0x01) {
            // ACK
             gState = STATE_HXHDR;
             gWaitTimeout=millis();
             Serial.write(0x01);
             Serial.flush();
          } else {
            // NACK - re-send the SETUP request
            gState = STATE_SETUP;
            SendSetup();
          }
        }
        break;
      case STATE_AUDIT:
        // Check return code
        if (inbuf[0] == 0x83) { // This is a SETUP response
          if (inbuf[1] == 0x01) {
             // ACK
             gState = STATE_HXHDR; // Wait on RESET
             gWaitTimeout=millis();
             Serial.write(0x01);
             Serial.flush();
          } else {
            // NACK - error out
             gState = STATE_DEVID;
             Serial.write(0x04);
             Serial.flush();
          }
        }
        break;
      default: // Unexpected reply
        break;
    }// switch (gState)
  } else {
     // On Radio Message Timeout..
     if (W4Radio(gState)) {
        if (gState!=STATE_IDLE) {
           // If failure to hear back after 5 retries
           uint8_t RETRY_LIMIT = 5;

           if (gState == STATE_SEARCH) {
              RETRY_LIMIT = 10;
           }
           if (retry_count > RETRY_LIMIT) {
            // Exit with error
            idleRadio();
            addr_client =0;
            gState = STATE_IDLE;
            retry_count = 0;
            Serial.write(0x07);
            splash();
           }

           // Has there been seven mised HeartBeats?
           if (heart_beat_count == 7) {
             idleRadio();
           }

           if (millis()-gWaitTimeout > 1000) {
             switch (gState) {
                case STATE_SEARCH:
                  sendBeacon();
                  break;
                case STATE_BIND:
                  sendBindRequest();
                  break;
                case STATE_SETUP:
                  SendSetup();
                  break;
                case STATE_WRITE:
                  SendWrite();
                  break;
                case STATE_COMMIT:
                  SendCommit();
                  break;
                default:
                  // do nothing
                  break;
             } // switch gState
             retry_count++;
             gWaitTimeout= millis();
          } //1 second Timeout
        }// Not STATE_IDLE
     } //W4Radio
  } //No Radio Payload
} // pollRadio


char  poll_idle[4] ={'-',0xcd,'|','/'};
char poll_bound[4]={0,1,0,' '};

void DisplayState() {
   lcd.home (); // set cursor to 0,0

   if (gState==STATE_IDLE) {
     show_idle();
     return;
   }
   lcd.print("nRF:");

   switch (gState) {
    case STATE_IDLE:
    case STATE_LOG:
    case STATE_DEVID:
        lcd.print("                ");
        break;
    case STATE_SEARCH: {
       char temp[17];
          lcd.print("Searching  ");
               //nRF:56789ABCDE
          lcd.setCursor(0,1);
          sprintf(temp," device: %6.6lX ",addr_client);
          //0123456789ABCDEF
          // device: 1D0002
          lcd.print(temp);
       }
       break;
    case STATE_BIND:{
          char temp[17];
          lcd.print("BINDING...  ");
               //nRF:56789ABCDE
          lcd.setCursor(0,1);
          sprintf(temp," device: %6.6lX ",addr_client);
          //0123456789ABCDEF
          // device: 1D0002
          lcd.print(temp);
        }
        break;
    default:
          lcd.print("F/W Upload");
               //nRF:56789ABCDE
          break;
    }

   // Show the "I'm Alive" animation
   lcd.setCursor(15,0);
   /* Show the polling cursor */
   poll_count = (poll_count+1)%32;
   if ((heart_beat_count==0) && (mode(gState)>mode(STATE_BIND))){
      lcd.print(poll_bound[poll_count/8]);
   } else {
      lcd.print(poll_idle[poll_count/8]);
   }
}

// Helper routine for parsing HEX to binary
unsigned char x2i(char input) {
    if (input >= '0' && input <= '9') {
        return input- '0';
    } else if (input >= 'A' && input <= 'F') {
        return input -('A' - 10);
    } else if (input >= 'a' && input <= 'f') {
        return input- ('a' - 10);
    }
    return -1;
}

void show_idle() {
  if ((millis()-splash_timeout) >100) {
    // Clear previous Character
    lcd.setCursor(phase%16,1);
    lcd.write(info_line[phase%16]);
    phase = (phase+dir);
    // Show ICON
    lcd.setCursor(phase,1);
    lcd.write(255);

    phase %=16;

    //
    if (phase == 15) {
      dir=-1;
      sprintf(info_line,"  version  %d.%d  ",major,minor);
      delay(2000);
    }
    if (phase == 0) {
      dir=1;
      sprintf(info_line," LabRat  Lights ");
                       //0123456789ABCDEF
      delay(2000);
    }
    splash_timeout = millis();
  }
}

// Main Loop
//    - Check for Serial Input
//    - Determine Parsing of Serial based on character received
//    -
//
void loop() {
  int inch =0x00;
  // Are there any Radio Payload Pending?
  if (W4Radio(gState))
     pollRadio();
  // Is there any incoming Serial Pending?

  if (W4Serial(gState)) { // In a state that requires reading the file?
    if ( Serial.available() ) {
      inch = Serial.read();
      serial_timeout = millis();
      if (gState==STATE_IDLE) {
           // Only thing we do is wait on the SYNC
           if (inch == 0x42){
              gState = STATE_LOG;
              Serial.write(0x42);
           }
      } else {
        if (gREAD_counter == 0) { // No data pending
          switch (gState) {
            case STATE_LOG:
              if (inch==0x06){
                gREAD_counter = 1;
              } // Intentional FALL through
            case STATE_DEVID: // Wait for type 05 record
              if (inch==0x05) {// nRF target device ID
                    gREAD_counter = 3;
                    gState = STATE_DEVID;
                }
                break;
            case STATE_HXHDR: {
                switch (inch)  { // Parse record and update state accordingly
                   case 0x01 :
                      // Stay in STATE_HXHDR state
                      gREAD_counter = 4;
                      break;
                   case 0x02 :
                      // Change to parsing 32 byte payload
                      gState=STATE_HXDATA;
                      gREAD_counter = 32;
                      break;
                   case 0x03 :
                      gState = STATE_RESET;
                      gREAD_counter = 1;
                      break;
                   case 0x04 :
                      gState = STATE_FINISH; // This is the AUDIT request
                      gREAD_counter = 7;
                      break;
                   default:
                     // Invalid Record type
                     Serial.write(0x02);
                     gState = STATE_IDLE;
                     splash();
                     idleRadio();
                     break;
                }// switch (inch)
                break;
              } // case STATE_HXHDR
              break;
            } // switch gState
        } else {
          // Count is not at ZERO
          switch(gState) {
            case STATE_HXHDR:
             switch (gREAD_counter) {
                 case 4: gWriteCmd.addrL = inch; break;
                 case 3: gWriteCmd.addrH = inch; break;
                 case 2: gWriteCmd.csum  = inch; break;
                 case 1: gWriteCmd.erase = (inch == 'E'); break;
                 default: break;
             }
             gREAD_counter--;
             if (gREAD_counter == 0) {
                 SendSetup();
                 retry_count = 0;
                 gState = STATE_SETUP;
             }
             break;
          case STATE_HXDATA:
             gWriteCmd.data[32-gREAD_counter] = inch;
             gREAD_counter--;
             if (gREAD_counter == 0) {
                 SendWrite();
                 gState = STATE_WRITE;
             }
             break;
          case STATE_RESET:
             gREAD_counter--;
             if (gREAD_counter == 0) {
                SendReboot();
                Serial.write(0x01);
                Serial.flush();
                gState = STATE_IDLE;
                idleRadio();
                delay(2000);
                splash();
                addr_client =0;
             }
             break;
          case STATE_FINISH:
             gWriteCmd.data[7-gREAD_counter] = inch;
             gREAD_counter--;
             if (gREAD_counter == 0) {
                 SendAudit();
                 gState=STATE_AUDIT; // Wait for AUDIT ACK
             }
             break;
          case STATE_DEVID:
             addr_client=addr_client<<8 | (inch&0xFF);
             gREAD_counter--;
             if (gREAD_counter == 0) {
                 // Attempt to Bind to the client
                 gState = STATE_SEARCH; // Wait for Device Reply
                 sendBeacon();
                 //sendBindRequest();
             }
             break;
          case STATE_LOG:
             gLogLevel = (inch>0);
             gREAD_counter--;
             if (gREAD_counter == 0) {
                 // Attempt to Bind to the client

                 gState = STATE_DEVID; // Wait for DEVICE_ID
                 //Serial.write(gLogLevel+0x10);
                 Serial.write(0x01); // Confirm message
             }
             break;
          default:
             break;  // Invalid state

          } // Switch
        } // Else Count !=0
      } // Not IDLE
    } else { // No Serial Available
       // Is Serial expected?
       if (W4Serial(gState) && (gState != STATE_IDLE)) {
         if (millis()-gHeartBeatTimeout > 1500) {
           if (mode(gState) > mode(STATE_BIND)) {
             SendHeartBeat();
             heart_beat_count++;
             gHeartBeatTimeout=millis();
             lcd.setCursor(0,1);
             lcd.print(F("- signal lost  -"));
           }
         }
         if ((millis()-serial_timeout)>10000) {
              Serial.write(0x06); // Timeout on reading from file
              Serial.flush();
              gState = STATE_IDLE;
              idleRadio();
              addr_client =0;
              lcd.setCursor(4,0);
              lcd.print(millis()-serial_timeout);
              splash();
         }
       }
    }
  } // Should we be checking for Serial
  DisplayState();
}
