
#include <SPI.h>
#include "RF24.h"
#include <printf.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>


LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3, POSITIVE);

#define BAUDRATE (115200)

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 9 & 10 */
RF24 radio(9,10);

typedef byte t_nrf_addr[5];
#define MAX_PIPE (5)

/* Addressed for this Device (Server) and the client device */

byte addr_server[]  = {0xC1,0xDE,0xC0};
byte addr_p2p[]     = {0x01,0xde,0xc0}; // Pipe 1
byte addr_client[]  = {0x00,0x00,0x1D};
byte addr_yell[]    = {0xBB,0xDE,0xC0};

#define PIPE_STATE_IDLE    (0x00)
#define PIPE_STATE_BUSY   (0x01)
#define PIPE_STATE_WAITING (0x02)

unsigned long startWaitTime=0;

typedef struct sPipeState {
  uint8_t state;
  uint8_t txaddr[3];   // Address of the target device bound to this pipe
  uint8_t payload[32]; // Copy of the last payload sent
} tPipeState;

struct sNRFData {
  uint8_t channel;
  tPipeState pipe; // Only allowing a single connection at a time..
} gNRFData;

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
//    Parsing Packet type 42 - SYNC request .. if see SYNC cmd, reply with SYNC response

#define TTY_STATE_IDLE     (0x00)
#define TTY_STATE_NSYNC    (0x01)
#define TTY_STATE_W4_SETUP (0x02)
#define TTY_STATE_W4_DATA  (0x03)
#define TTY_STATE_W4_RESET (0x04)
#define TTY_STATE_W4_SYNC  (0x05)
#define TTY_STATE_W4_AUDIT (0x06)

uint8_t tty_state = TTY_STATE_IDLE;
uint8_t in_count = 0;

#define NRF_STATE_IDLE          (0x00)
#define NRF_STATE_W4_BIND_ACK   (0x01)
#define NRF_STATE_BOUND         (0x02)
#define NRF_STATE_W4_SETUP_ACK  (0x03)
#define NRF_STATE_W4_WRITE_ACK  (0x04)
#define NRF_STATE_W4_COMMIT_ACK (0x05)
#define NRF_STATE_W4_AUDIT_ACK  (0x06)
#define NRF_STATE_W4_HEART_ACK  (0x07)

uint8_t nrf_state = NRF_STATE_IDLE;

void configRadio() {

      radio.setChannel(gNRFData.channel);
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

void idleRadio() {
       // Pipe 0 reserved for Writing
      radio.openWritingPipe(addr_yell);
      
      radio.openReadingPipe(1,addr_server);
      radio.openReadingPipe(2,addr_p2p);

      nrf_state = NRF_STATE_IDLE;
}


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

void setup() {
  Serial.begin(BAUDRATE);
  printf_begin();

// I2C LCD 
  lcd.begin(20,4);
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.clear();
  lcd.createChar(0, Heart);
  lcd.createChar(1, Heart2);
  
  gNRFData.channel=82;
  gNRFData.pipe.state = PIPE_STATE_IDLE;
  memset (gNRFData.pipe.txaddr,0x00,1);
  // Setup the NRF Radio sub-system
  radio.begin();
  configRadio();
  radio.startListening(); // Note  ENRXADDR_P0 = 0
}

void dumpMsg(uint8_t * msg,unsigned char num_param) {
  char tempstr[21];
  switch (num_param) {
    case 1:snprintf(tempstr,20,"%2.2X ",msg[0]); break;
    case 2:snprintf(tempstr,20,"%2.2X%2.2X ",msg[0],msg[1]); break;
    case 3:snprintf(tempstr,20,"%2.2X%2.2X%2.2X ",msg[0],msg[1],msg[2]); break;
    case 4:snprintf(tempstr,20,"%2.2X%2.2X%2.2X%2.2X ",msg[0],msg[1],msg[2],msg[3]); break;
    case 5:snprintf(tempstr,20,"%2.2X%2.2X%2.2X%2.2X%2.2X ",msg[0],msg[1],msg[2],msg[3],msg[4]); break;
    case 7:snprintf(tempstr,20,"%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X ",msg[0],msg[1],msg[2],msg[3],msg[4],msg[5],msg[6]); break;
    case 8:snprintf(tempstr,20,"%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X ",msg[0],msg[1],msg[2],msg[3],msg[4],msg[5],msg[6],msg[7]); break;
           
    default: snprintf(tempstr,20,"Unsupported");break;
                
  }
  lcd.print(tempstr);
}

bool SendSetup(){
  uint8_t * msg = gNRFData.pipe.payload;
  bool retCode = false;
       
        msg[0] = 0x80; // String to erase
        msg[1] = gWriteCmd.addrL;
        msg[2] = gWriteCmd.addrH;
        msg[3] = gWriteCmd.erase; // No ERASE 
          
        radio.stopListening(); // Ready to Write - EN_RXADDRP0 = 1
        radio.setAutoAck(0,true);  
        
        retCode =  radio.write(msg,32); // Want to get AA working here
        radio.setAutoAck(0,false);  
        radio.startListening(); // EN_RXADDRP0 = 0
        
        lcd.setCursor(0,1);
        #ifdef DEBUG
            lcd.print("TxS");     
            if (retCode == false) {
              lcd.print("**FAILED**");  
            } else { 
              dumpMsg(msg,5);
            }
        #else
          lcd.print("Erase ");
          dumpMsg(&(msg[1]),2);
        #endif
        return retCode;
}

bool SendReboot() {
   uint8_t * msg = gNRFData.pipe.payload;
   bool retCode =false;
   
        radio.stopListening();
        radio.setAutoAck(0,true); 
   
        msg[0] = 0x86;
                
        if (radio.write(msg,32)) { // Want to get AA working here
          retCode= true;
        }
        
        gNRFData.pipe.state = PIPE_STATE_IDLE;
        radio.setAutoAck(0,false); 
        radio.startListening();
        return retCode;
}

bool SendWrite() {
  uint8_t * msg = gNRFData.pipe.payload;
  bool retCode = false;

        lcd.setCursor(0,1);
        lcd.print("Writing.. ");
    
        radio.stopListening();
        radio.setAutoAck(0,true);             
        
        msg[0] = 0x81; // String to erase
        memcpy(&(msg[1]),gWriteCmd.data,30);

        if (radio.write(msg,32)) { // Want to get AA working here
          retCode = true;
        }
        radio.setAutoAck(0,false);       
        radio.startListening();
           
        return retCode;  
}

bool SendCommit() {
  uint8_t * msg = gNRFData.pipe.payload;
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

        lcd.setCursor(0,1);        
        #ifdef DEBUG
          lcd.print("TX=");      
          dumpMsg(msg,5);
        #else
          lcd.print("Commit    ");
        #endif
        return retCode;   
}

bool SendAudit() {
  uint8_t * msg = gNRFData.pipe.payload;
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
        
        lcd.setCursor(0,1);
        #ifdef DEBUG
          lcd.print("TX=");
          dumpMsg(msg,5);
        #else
          lcd.print("Audit..   ");
        #endif
        return retCode;   
}

bool SendHeartBeat() {
  uint8_t * msg = gNRFData.pipe.payload;
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
       
        #ifdef DEBUG
          lcd.setCursor(0,1);
          lcd.print("HB=");
          dumpMsg(msg,5);
        #endif
        return retCode;   
}

bool ReplyBind() {
     uint8_t * msg = gNRFData.pipe.payload;
     bool retCode = false;

         msg[0] = 0x87;
        // Allocate a pipe and send that address to the client
        // (for now use the default)
        memcpy(&(msg[1]),addr_p2p,3);

        msg[16]=millis()&0xff;
   
        
        radio.stopListening();   // Ready to write EN_RXADDR_P0 = 1
        radio.setAutoAck(0,false);// As a broadcast first ...
          
        radio.openWritingPipe(gNRFData.pipe.txaddr); // Who I'm sending it to
        startWaitTime=millis();
        delay(500);
               
        retCode = radio.write(msg,32); // Want to get AA working here
 
        radio.setAutoAck(2,true);// From now on ... BOUND to P2P    
        radio.startListening(); // EN_RXADDR_P0 = 0
        
        lcd.setCursor(0,1);
        #ifdef DEBUG
          lcd.print("Tx[");       
          if (retCode == false) {
            lcd.print("--FAILED--");
          } else {
              dumpMsg(msg,5);
          }
        #else
          lcd.print("Binding   ");
        #endif 
        return retCode;
}

unsigned char heart_beat_count = 0;
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
      heart_beat_count = 0; // receive ANYTHING will reset the heartbeat
      switch (gNRFData.pipe.state) {
        case PIPE_STATE_WAITING: {
             if (nrf_state== NRF_STATE_W4_BIND_ACK) { // IF BINDING whith command pending
                if (inbuf[0] == 0x87) { // This is a BIND reply
                   gNRFData.pipe.state = PIPE_STATE_BUSY;    
              
                   switch (nrf_state) { 
                    case NRF_STATE_W4_SETUP_ACK:     
                        while (!SendSetup()); 
                        break;
                    case NRF_STATE_W4_WRITE_ACK:
                        while (!SendWrite());
                        break;
                    case NRF_STATE_W4_COMMIT_ACK:
                        while (!SendCommit());
                        break;
                    case NRF_STATE_W4_AUDIT_ACK:
                        while (!SendAudit());
                        break;
                    default: break; // Do Nothing
                  } // CASE
               }  
            }  
          }
          break;
        case PIPE_STATE_IDLE:{
            if (inbuf[0] == 0x88) { // Device Announce
              gNRFData.pipe.state = PIPE_STATE_BUSY;
              memcpy(gNRFData.pipe.txaddr, &(inbuf[1]),3);
              memcpy(addr_client,&(inbuf[1]),3); // Redundant - can be optimized
              ReplyBind();
              nrf_state = NRF_STATE_W4_BIND_ACK;     
            }
          }
          break;
        case PIPE_STATE_BUSY: {
          // Handing if already BOUND
          switch (nrf_state) {
              case NRF_STATE_BOUND:
                 // If Serial state is in W4 .. we need to do the action
              case NRF_STATE_IDLE: 
                if (inbuf[0] == 0x88) { // This is a BIND request
                  memcpy(gNRFData.pipe.txaddr, &(inbuf[1]),3);
                  memcpy(addr_client,&(inbuf[1]),3); // Redundant - can be optimized
                  gNRFData.pipe.state = PIPE_STATE_BUSY;
                  ReplyBind(); // Broadcast BOUND request
                  nrf_state = NRF_STATE_W4_BIND_ACK;  
                } 
                break;
              case NRF_STATE_W4_BIND_ACK:
                if (inbuf[0] == 0x87) { // This is a BIND reply
                  gNRFData.pipe.state = PIPE_STATE_BUSY;
                  nrf_state = NRF_STATE_BOUND;
                  startWaitTime=millis();   
                }
                break;
              case NRF_STATE_W4_SETUP_ACK:
                if (inbuf[0] == 0x80) { // This is a SETUP response
                  if (inbuf[1] == 0x01) {
                    // ACK
                    nrf_state = NRF_STATE_BOUND;
                    startWaitTime=millis();
                    Serial.write(0x01);
                    Serial.flush();
                  } else {
                    // NACK - re-send the SETUP request
                    while (!SendSetup());
                  }
                }
                break;
              case NRF_STATE_W4_WRITE_ACK:
                if (inbuf[0] == 0x81) { // This is a WRITE response
                  if (inbuf[1] == 0x01) {
                    // ACK
                    //Build and send the COMMIT request
                     nrf_state = NRF_STATE_W4_COMMIT_ACK;
                     while (!SendCommit());
                  } else {
                    // NACK - re-send the SETUP request
                    while (!SendWrite());
                  }
                }
                break;
              case NRF_STATE_W4_COMMIT_ACK:
                if (inbuf[0] == 0x82) { // This is a COMMIT response
                  if (inbuf[1] == 0x01) {
                    // ACK
                     nrf_state = NRF_STATE_BOUND;
                     startWaitTime=millis();
                     Serial.write(0x01);
                     Serial.flush();
                  } else {
                    // NACK - re-send the SETUP request
                    nrf_state = NRF_STATE_W4_SETUP_ACK;
                    tty_state = TTY_STATE_NSYNC;
                    while (!SendSetup());
                  }
                }
                break;
              case NRF_STATE_W4_AUDIT_ACK:
                if (inbuf[0] == 0x83) { // This is a SETUP response
                  if (inbuf[1] == 0x01) { 
                     // ACK
                     nrf_state = NRF_STATE_BOUND;
                     startWaitTime=millis();
                     Serial.write(0x01);
                     Serial.flush();
                  } else {
                    // NACK - re-send the AUDIT request
                     nrf_state = NRF_STATE_BOUND;
                     startWaitTime=millis();                 
                     Serial.write(0x00);
                     Serial.flush();
                  }
                }       
                break;
              case NRF_STATE_W4_HEART_ACK:
                // Any response shows life..
                nrf_state = NRF_STATE_BOUND;
                startWaitTime=millis();
                heart_beat_count = 0;
                poll_count = (poll_count+1)%32;
                break;
            } // end of SWITCH (nrf_state)
          } // end of BOUND handling
          break;
        // ---------
      }// end of CASE
 
  }else {
     switch (nrf_state) {
        case NRF_STATE_BOUND: // check for idle device timeout
          //if (millis()-startWaitTime > 7000) {
          if (heart_beat_count == 7) {
            idleRadio();
          }
          if (millis()-startWaitTime > 1500) {
            SendHeartBeat();
            nrf_state = NRF_STATE_W4_HEART_ACK;
            startWaitTime=millis();
          }
          break;
        case NRF_STATE_W4_BIND_ACK:
          if (millis()-startWaitTime > 1000) {
            ReplyBind();
          }
          break;
        case NRF_STATE_W4_HEART_ACK:
          if (millis()-startWaitTime > 1000) {
            nrf_state = NRF_STATE_BOUND;
            heart_beat_count++; 
          }
          break;
      // Add other timeout handlers here
        default:
                // do nothing
                break;
     }
  }
}



char  poll_idle[4] ={'-',0xcd,'|','/'};
char poll_bound[4]={0,1,0,' '};

#ifdef DEBUG
  void DisplayState(uint8_t state,uint8_t nrf) {
    lcd.home (); // set cursor to 0,0
    uint8_t tempAA = radio.qryAutoAck();
    uint8_t tempEnRx = radio.qryEnRxaddr();
    uint8_t tempConfig = 0x00;
  
    switch(state) {
      case TTY_STATE_IDLE :    lcd.print("IDLE :"); break;
      case TTY_STATE_NSYNC:    lcd.print("NSYNC:"); break;
      case TTY_STATE_W4_SETUP: lcd.print("SETUP:"); break;
      case TTY_STATE_W4_DATA:  lcd.print("DATA :"); break;
      case TTY_STATE_W4_RESET: lcd.print("RESET:"); break;
      case TTY_STATE_W4_SYNC:  lcd.print("SYNC :"); break;
      default:                 lcd.print("???  :"); break;
    }
   
    
    switch(nrf) {
      case NRF_STATE_IDLE  :        lcd.print("IDLE  "); break;
      case NRF_STATE_W4_BIND_ACK:   lcd.print("B ACK "); break;
      case NRF_STATE_BOUND :        lcd.print("BOUND "); break;
      case NRF_STATE_W4_SETUP_ACK : lcd.print("S ACK "); break;
      case NRF_STATE_W4_WRITE_ACK : lcd.print("W ACK "); break;
      case NRF_STATE_W4_COMMIT_ACK: lcd.print("C ACK "); break;
      case NRF_STATE_W4_AUDIT_ACK:  lcd.print("A ACK "); break;
      case NRF_STATE_W4_HEART_ACK:  lcd.print("H ACK "); break;
      default:                      lcd.print("UNKNOWN   "); break;
    }
    
     if (radio.qryCE()) {
      lcd.print("^");
    } else {
      lcd.print("v");
    }
    char tempstr[10];
    snprintf(tempstr,10,"%2.2x ", radio.qryEnRxaddr());
    lcd.print(tempstr);
    
    radio.qryAddrReg(0x00, tempstr);
    tempConfig = (uint8_t) tempstr[0];
    if (tempConfig & 01) {
      lcd.print("R ");
    } else {
      lcd.print("T ");
    }
     
  
  /*
    radio.qryAddrReg(0x07, tempstr);
    dumpMsg(tempstr,1);
  
    radio.qryAddrReg(0x17, tempstr);
    dumpMsg(tempstr,1);
  */
  
    lcd.setCursor(19,0);
    if (gNRFData.pipe.state == PIPE_STATE_BUSY) 
      lcd.print("*");
    else
      lcd.print("-");
    
    lcd.setCursor(0,1);
     
    radio.qryAddrReg(0x0A, tempstr);
    
    if (tempAA&0x01) 
       lcd.print("P0+");
    else
       lcd.print("P0:");
       
    dumpMsg(tempstr,3);
  
    radio.qryAddrReg(0x10,tempstr);
    dumpMsg(tempstr,3);
    
    lcd.setCursor(0,2);
  
    if (tempAA&0x02) 
       lcd.print("P1+");
    else
       lcd.print("P1:");
       
    radio.qryAddrReg(0x0B,tempstr);
    dumpMsg(tempstr,3);
    
    if (tempAA&0x04) 
       lcd.print("P2+");
    else
       lcd.print("P2:");
    radio.qryAddrReg(0x0C,tempstr);
    dumpMsg(tempstr,1);
        
    lcd.setCursor(19,3);
    poll_count = (poll_count+1)%32;
    if (nrf_state==NRF_STATE_IDLE) {
        lcd.print(poll_idle[poll_count/8]);
    } else {
        lcd.print(poll_bound[poll_count/8]);   
    }
  }
#else
  void DisplayState(uint8_t state,uint8_t nrf) {
     lcd.home (); // set cursor to 0,0
  
     lcd.print("nRF:");
     switch(nrf) {
      case NRF_STATE_IDLE  :        
        lcd.print("IDLE        ");
        lcd.setCursor(0,1);
        lcd.print("             ");
        break;
      case NRF_STATE_W4_BIND_ACK:   
        lcd.print("Connecting "); 
        break;
      default :                     
        lcd.print("DevID:");
        dumpMsg(gNRFData.pipe.txaddr,3);
        break;
     }
     lcd.setCursor(19,0);
     
     poll_count = (poll_count+1)%32;
     if ((heart_beat_count==0) && (nrf_state != NRF_STATE_IDLE)){
        lcd.print(poll_bound[poll_count/8]);
     } else {
        lcd.print(poll_idle[poll_count/8]);  
     }
  }
#endif
void loop() {
  int inch =0x00;
  // Are there any Radio Payload Pending?
  pollRadio();
  // Is there any incoming Serial Pending?
  if ( Serial.available() ) {
    inch = Serial.read();
 
    switch (tty_state) {
      case TTY_STATE_IDLE:
         if (inch == 0x42){
            tty_state = TTY_STATE_W4_SYNC;
         }      
         break;
      case TTY_STATE_NSYNC:
         in_count = 0;
         switch (inch)  {
           case 0x01 : 
              tty_state = TTY_STATE_W4_SETUP;
              in_count = 4;
              break;
           case 0x02 : 
              tty_state = TTY_STATE_W4_DATA; 
              in_count = 32;
              break;
           case 0x03 : 
              tty_state = TTY_STATE_W4_RESET; 
              in_count = 1; 
              break;
           case 0x04 :
              tty_state = TTY_STATE_W4_AUDIT;
              in_count = 7;
              break;
           default:
              tty_state = TTY_STATE_IDLE; // Invalid input
              break;                    
         }
         break;
      case TTY_STATE_W4_SETUP:
         switch (in_count) {
             case 4: gWriteCmd.addrL = inch; break;
             case 3: gWriteCmd.addrH = inch; break;
             case 2: gWriteCmd.csum  = inch; break;
             case 1: gWriteCmd.erase = (inch == 'E'); break;
             default: break;
         }
         in_count--;
         if (in_count == 0) {
             if (gNRFData.pipe.state == PIPE_STATE_BUSY) {       
               while (!SendSetup());
             }
             nrf_state = NRF_STATE_W4_SETUP_ACK;
             tty_state = TTY_STATE_NSYNC;
         }
         break;
      case TTY_STATE_W4_DATA:
         gWriteCmd.data[32-in_count] = inch;
         in_count--;
         if (in_count == 0) {
             if (gNRFData.pipe.state == PIPE_STATE_BUSY) {
                SendWrite();
             }
             nrf_state = NRF_STATE_W4_WRITE_ACK;
             tty_state = TTY_STATE_NSYNC;
         }
         break;     
      case TTY_STATE_W4_RESET:
         in_count--;
         if (in_count == 0) {
             if (gNRFData.pipe.state == PIPE_STATE_BUSY) {
                SendReboot();
             }
            Serial.write(0x01);
            Serial.flush();
            nrf_state = NRF_STATE_IDLE;
            tty_state = TTY_STATE_IDLE;          
         }
         break;

      case TTY_STATE_W4_SYNC:
         if (inch == 0x42) {
            tty_state = TTY_STATE_NSYNC;
            Serial.write(0x42); // Reply- we are In SYNC
            Serial.flush();
         } else {
            tty_state = TTY_STATE_IDLE;
         }

      case TTY_STATE_W4_AUDIT:
         gWriteCmd.data[7-in_count] = inch;
         in_count--;
         if (in_count == 0) {
             if (gNRFData.pipe.state == PIPE_STATE_BUSY) {
                SendAudit();
             }
             nrf_state = NRF_STATE_W4_AUDIT_ACK;
             tty_state = TTY_STATE_NSYNC;
         }
         break;
      default:
         break;  // Invalid state
      
    }
  }
  DisplayState(tty_state, nrf_state);
}
