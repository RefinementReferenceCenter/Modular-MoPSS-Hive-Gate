//------------------------------------------------------------------------------
/*
--- Adafruit ItsyBitsy M0 pin mapping ---

 A0-JP9
 A1-JP10
 A2-JP2
 A3-JP3
 A4-JP4
 A5-JP5

 D2-
 D3-
 D4-SD chip select
 D5-
 D7-
 D9-
D10-JP8
D11-WiFi Busy JP7
D12-WiFi Reset JP6
D13-WiFi CS

--- Experimental Setup ---

^^^^^^^\                                                     /^^^^^^^^
       |                                                     |
h c    |     |R|    |I| | D |    |I|    | D | |I|    |R|     |    t  c
o a  ––|–––––|F|––––|R|–| O |––––|R|––––| O |–|R|––––|F|–––––|––  e  a
m g    |     |I|    | | | O |    | |    | O | | |    |I|     |    s  g
e e  ––|–––––|D|––––| |–| R |––––| |––––| R |–| |––––|D|–––––|––  t  e
       |     |1|    |1| | 1 |    |3|    | 2 | |2|    |2|     |
       |                                                     |
_______/                  |----- 10cm ----|                  \________

*/

//------------------------------------------------------------------------------
#include <Wire.h>             //I2C communication
#include <SD.h>               //Access to SD card
#include <RTCZero.h>          //realtimeclock on MKR Boards
#include <WiFiNINA.h>         //Wifi Chipset modified version from Adafruit
#include <Adafruit_DotStar.h> //controlling the onboard dotstar RGB LED

//----- declaring variables ----------------------------------------------------
//Current Version of the program
//##############################################################################
//##############################################################################
const char VERSION[9] = "05102020"; //obsolete with git?
const char HARDWARE_REV[] = "v2.0";
//##############################################################################
//##############################################################################

//Sensors
const int sensor1 = A1; //IR 1 door 1
const int sensor2 = A0; //IR 2 door 2
const int sensor3 = 10; //IR 3 middle

//use volatile if interrupt based
volatile uint8_t sensor1_triggered = 0;     //IR 1 door 1
volatile uint8_t sensor2_triggered = 0;     //IR 2 door 2
/*volatile*/ uint8_t sensor3_triggered = 0; //IR 3 middle

uint8_t IR_door1_buffer[10] = {};   //10 reads, every 50ms = 0.5s buffer length
uint8_t IR_door2_buffer[10] = {};
uint8_t IR_middle_buffer[10] = {};
uint8_t sb = 0;                     //sensor buffer counter
uint8_t IR_door1_buffer_sum = 0;    //holds count of IR triggers in the last 0.5s
uint8_t IR_door2_buffer_sum = 0;
uint8_t IR_middle_buffer_sum = 0;
unsigned long IRsensor_time;       //time when IR sensors 1,2,3 were last checked

//LEDs
const uint8_t LED1 = 2;
volatile uint8_t ledstate = LOW;
Adafruit_DotStar strip(1, 41, 40, DOTSTAR_BRG); //create dotstar object

//SD
const uint8_t SDcs = 4; //ChipSelect pin for SD (SPI)
File dataFile;          //create file object

//WiFi
char ssid[] = "Buchhaim";           //network name
char pass[] = "2416AdZk3881QPnh+";  //WPA key
int status = WL_IDLE_STATUS;        //initialize for first check if wifi up
#define SPIWIFI       SPI  // The SPI port

//RTC - Real Time Clock
const uint8_t GMT = 2;  //current timezone (Sommerzeit)
RTCZero rtc;            //create rtc object

//Door Modules
const uint8_t door1 = 0x10;         //I2C address door module 1
const uint8_t door2 = 0x11;         //I2C address door module 2

uint8_t door1_open = 0;           //flag if door is open
uint8_t door2_open = 0;
uint8_t door1_moving = 0;         //flag if door is moving
uint8_t door2_moving = 0;
unsigned long door1_time;         //stores time when door starts moving
unsigned long door2_time;
uint16_t rotate1_duration;        //duration until door has finished movement
uint16_t rotate2_duration;
uint16_t door1_counter = 0;       //counter how often door 2 was opened (used to reset door position)
uint16_t door2_counter = 0;       //counter how often door 2 was opened (used to reset door position)
uint8_t door1_reset = 0;          //flag is set if door needs to be "re-set"
uint8_t door2_reset = 0;          //flag is set if door needs to be "re-set"

//RFID
const uint8_t reader1 = 0x09;     //I2C address RFID module 1
const uint8_t reader2 = 0x08;     //I2C address RFID module 2

unsigned long RFIDtime;           //used to measure time before switching to next antenna
uint8_t RFIDtoggle = 0;           //flag used to switch to next antenna

uint8_t tag[6] = {};                  //global variable to store returned tag data (limitation of C to return arrays)
uint8_t tag1_present = 0;             //flag that indicates if tag was present during read cycle
uint8_t tag2_present = 0;
uint8_t reader1_cycle = 0;            //toggles flag if a read cycle has just happened (not automatically cleared)
uint8_t reader2_cycle = 0;
uint8_t currenttag1[6] = {};          //saves id of the tag that was read during the current read cycle
uint8_t currenttag2[6] = {};
uint8_t lasttag1[6] = {};             //saves id of the tag that was read during the previous read cycle
uint8_t lasttag2[6] = {};

//TODO: test switching modes
uint8_t RFIDmode = 1;           //select mode to operate in: 1-alternate, 2-reader1, 3-reader2
uint8_t RFIDmode_firstrun = 1;  //to make sure the correct reader is turned on/off

//Experiment variables
//0 = no transition, 1 = origin door opened, 2 = origin door closed, 3 = checks mice present, 4 = target door closing
uint8_t transition_to_tc = 0;         //track transition phase to target cage
uint8_t transition_to_hc = 0;         //track transition phase to home cage
unsigned long transition_time = 0;    //time when both doors are closed, when mouse is in the middle
uint8_t failsafe = 0;                 //flag if failsafe is triggered (ONLY FOR TRAINING PHASE 2)

uint16_t day = 1;             // counts days, necessary to raise nosepokes_needed
unsigned long starttime;      // start of programm, necessary to raise nosepokes_needed
uint8_t next_raise;           // when to raise threshold next time
unsigned long rtccheck_time;  //time the rtc was checked last

//Mice tags
//uint8_t mice = 12; //TODO: use mice variable to automatically adjust several arrays
const uint8_t mouse_library[13][6] = {
  {0x00,0x00,0x00,0x00,0x00,0x00}, //mouse 0
  {0xC7,0x65,0xF7,0x90,0x2E,0xE1}, //mouse 1  8167 sw_we
  {0x80,0x68,0xF7,0x90,0x2E,0xE1}, //mouse 2  8864 sw_ge
  {0x35,0x67,0xF7,0x90,0x2E,0xE1}, //mouse 3  8533 sw_ro
  {0x8E,0x6B,0xF7,0x90,0x2E,0xE1}, //mouse 4  9646 we_sw
  {0x93,0x74,0xF7,0x90,0x2E,0xE1}, //mouse 5  1955 ge_sw
  {0x68,0x69,0xF7,0x90,0x2E,0xE1}, //mouse 6  9096 ge_we
  {0x24,0x73,0xF7,0x90,0x2E,0xE1}, //mouse 7  1588 ge_ro
  {0xAE,0x66,0xF7,0x90,0x2E,0xE1}, //mouse 8  8398 ro_sw
  {0x5B,0x6F,0xF7,0x90,0x2E,0xE1}, //mouse 9  0619 ro_we
  {0x76,0x65,0xF7,0x90,0x2E,0xE1}, //mouse 10 8086 ro_ge
  {0xA1,0x82,0x42,0xDD,0x3E,0xF3}, //mouse 11 polymorphmaus
  {0x0E,0x67,0xF7,0x90,0x2E,0xE1}  //mouse 12 bleistiftmaus
};

uint8_t update_current_mouse1 = 1;    //flag for updating current mouse at reader 1
uint8_t update_current_mouse2 = 1;    //flag for updating current mouse at reader 2
uint8_t current_mouse1 = 0;           //placeholder simple number for tag at reader 1
uint8_t current_mouse2 = 0;           //placeholder simple number for tag at reader 2

//##############################################################################
//#####   U S E R   C O N F I G  ###############################################
//##############################################################################

//if set to 1, the MoPSS will wait for a wifi connection and synchronization with network time before continuing
const uint8_t enable_wifi = 1;

//if set to 1, the MoPSS will wait until a serial connection via USB is established
//before continuing. Also prints what is written to uSD to Serial as well.
const uint8_t is_testing = 1;

//Experiment variables
const uint32_t warn_time =    60*60*24*1; //time after which a warning is logged
const uint32_t exclude_time = 60*60*24*2; //time after which mouse is excluded

//warning label 2 is handled automatically, DON'T SET!
uint8_t mouse_participation[13] = { //0 = does not participate; 1 = regular participation; 2 = warning; 3 = excluded from experiment
  1, //mouse 0
  1, //mouse 1  8167 sw_we
  1, //mouse 2  8864 sw_ge
  1, //mouse 3  8533 sw_ro
  1, //mouse 4  9646 we_sw
  1, //mouse 5  1955 ge_sw
  1, //mouse 6  9096 ge_we
  1, //mouse 7  1588 ge_ro
  1, //mouse 8  8398 ro_sw
  1, //mouse 9  0619 ro_we
  1, //mouse 10 8086 ro_ge
  1, //mouse 11 polymorphmaus
  1}; //mouse 12 bleistiftmaus

//12 mice, 3 values. 0: started poke, 1: successfully poked, 2: seen at R2
//if first value of last seen time is set to 0, starttime will be used
//only set first value of each mouse, accepted input is time in unixtime format
uint32_t mouse_last_seen[13] = {
0, //mouse 0 (ignored)
0, //mouse 1  8167 sw_we
0, //mouse 2  8864 sw_ge
0, //mouse 3  8533 sw_ro
0, //mouse 4  9646 we_sw
0, //mouse 5  1955 ge_sw
0, //mouse 6  9096 ge_we
0, //mouse 7  1588 ge_ro
0, //mouse 8  8398 ro_sw
0, //mouse 9  0619 ro_we
0, //mouse 10 8086 ro_ge
0, //mouse 11 polymorphmaus
0}; //mouse 12 bleistiftmaus

//door management
const uint16_t door1_needed_rotation = 4700;  //3200 = 1 revolution
const uint16_t door2_needed_rotation = 4000;
const uint16_t door1_stays_open_min = 500;    //minimum open time
const uint16_t door2_stays_open_min = 500;
const uint16_t door1_stays_open_max = 10000;  //maximum open time
const uint16_t door2_stays_open_max = 10000;
const uint16_t door1_speed = 170;             //min 0, max ~230
const uint16_t door2_speed = 150;
const uint16_t door1_reset_up = 6300;         //distance to rotate for reset up
const uint16_t door2_reset_up = 5300;
const uint16_t door1_reset_down = 5350;       //distance to rotate for reset down
const uint16_t door2_reset_down = 4400;

//##############################################################################
//#####   S E T U P   ##########################################################
//##############################################################################
void setup()
{
  //----- DEBUGGING ------------------------------------------------------------
  pinMode(7,OUTPUT); //to allow port toggle for timing purposes
  
  //Set up RGB LED on board, and turn it off -----------------------------------
  strip.begin(); //Initialize pins for output
  strip.show();  //Turn all LEDs off ASAP
  
  //----- communication --------------------------------------------------------
  //start Serial communication
  Serial.begin(115200);
  if(is_testing == 1)
  {
    while(!Serial); //wait for serial connection
  }
  
  //start I2C
  Wire.begin(); //atsamd can't multimaster
  //disable RFID readers and wait until they acknowledge
  disableReader(reader1);
  disableReader(reader2);
  
  //----- Sensors --------------------------------------------------------------
  //setup input mode for pins (redundant, for clarity)
  pinMode(sensor1,INPUT); //IR 1 door 1
  pinMode(sensor2,INPUT); //IR 2 door 2
  pinMode(sensor3,INPUT); //IR 3 middle
  
  //attach interrupt to all pins (must not analogRead interruptpins or else interrupt stops working!)
  attachInterrupt(digitalPinToInterrupt(sensor1), sensor1_ISR, RISING); //IR 1 door 1
  attachInterrupt(digitalPinToInterrupt(sensor2), sensor2_ISR, RISING); //IR 2 middle
  //attachInterrupt(digitalPinToInterrupt(sensor3), sensor3_ISR, RISING); //IR 3 door 2
  
  //----- Doors ----------------------------------------------------------------
  rotate(door1, door1_reset_up, 1, door1_speed);    //open door too much
  rotate(door2, door2_reset_up, 1, door2_speed);
  rotate(door1, door1_reset_down, 0, door1_speed);  //close door to correct height
  rotate(door2, door2_reset_down, 0, door2_speed);
  
  //----- WiFi -----------------------------------------------------------------
  if(enable_wifi == 1)
	{
		Serial.println("----- WiFi Setup -----"); // check if the WiFi module works
		digitalWrite(LED1,HIGH); //turn on LED to show a connection attempt with wifi
    
    WiFi.setPins(13, 11, 12, -1, &SPIWIFI);
    
		if(WiFi.status() == WL_NO_MODULE)
    {
			Serial.println("WiFi chip not working/disconnected, program stopped!");
			criticalerror(); //don't continue
		}
    
    //attempt to connect to WiFi network:
    Serial.print("Connecting to SSID: ");
    Serial.println(ssid);
    uint8_t numberOfTries = 0;
    while(status != WL_CONNECTED)
    {
      numberOfTries++;
      Serial.print("Waiting to connect, attempt No.: ");
      Serial.println(numberOfTries);

      //Connect to WPA/WPA2 network
      status = WiFi.begin(ssid, pass);

      //wait 10 seconds for each connection attempt and slowly blink LED:
      delay(2000);
      digitalWrite(LED1,LOW);
      delay(500);
      digitalWrite(LED1,HIGH);
      delay(2000);
      digitalWrite(LED1,LOW);
      delay(500);
      digitalWrite(LED1,HIGH);
      delay(2000);
      digitalWrite(LED1,LOW);
      delay(500);
      digitalWrite(LED1,HIGH);
      delay(2000);
      digitalWrite(LED1,LOW);
      delay(500);
      digitalWrite(LED1,HIGH);
    }
    Serial.println("Successfully connected!");
    
    //----- Real Time Clock ------------------------------------------------------
    Serial.println("----- RTC Setup -----");
    
    digitalWrite(LED1,HIGH); //turn on LED
    
    rtc.begin();
    unsigned long epoch = 0; //stores the time in seconds since beginning
    
    //repeatedly contact NTP server
    Serial.println("Attempting to reach NTP server");
    numberOfTries = 0;
    while(epoch == 0)
    {
      Serial.print("Attempt No. ");
      Serial.println(numberOfTries);
      numberOfTries++;
      delay(500);
      digitalWrite(LED1,LOW);
      delay(500);
      digitalWrite(LED1,HIGH);

      epoch = WiFi.getTime();
    }
    
    rtc.setEpoch(epoch); //set time received from ntp server
    
    Serial.print("Success! Time received: ");
    Serial.println(nicetime());
    
    digitalWrite(LED1,LOW); //turn off LED
    
    //disable wifi module after fetching time to conserve power (~83mA)
    WiFi.end();
  } //enablewifibracket
  
  //Set time to zero if WiFi isn't enabled
  if(enable_wifi == 0)
  {
    rtc.begin();
    Serial.println("WiFi disabled, setting time to 0");
    rtc.setEpoch(0); //1.1.2000 00:00:00 946684800
  }
  
  //----- Setup SD Card --------------------------------------------------------
  Serial.println("----- SD-Card Setup -----");
  // see if the card is present and can be initialized:
  if (!SD.begin(SDcs))
  {
    Serial.println("Card failed, or not present, program stopped!");
    //Stop program if uSD was not detected/faulty (needs to be FAT32 format):
    criticalerror();
  }
  else
  {
    Serial.println("SD card initialized successfully!");
  }
  
  //----- Setup log file, and write initial configuration ----------------------
  //open file, or create if empty
  dataFile = SD.open("RFIDLOG.TXT", FILE_WRITE);
  
  //TODO: add option to query RFID reader for version and resonant frequency
  //write current version to SD and some other startup/system related information
  dataFile.println("");
  dataFile.print("# Modular MoPSS Hive version: ");
  dataFile.println(VERSION);
  
  dataFile.print("# Door Module 1 version: ");
  dataFile.println("not yet implemented");
  dataFile.print("# Door Module 2 version: ");
  dataFile.println("not yet implemented");

  dataFile.print("# RFID Module 1 version: ");
  dataFile.println("not yet implemented");
  dataFile.print("# RFID Module 2 version: ");
  dataFile.println("not yet implemented");

  dataFile.print("# System start @ ");
  dataFile.print(nicetime());
  dataFile.print(" ");
  dataFile.print(rtc.getDay());
  dataFile.print("-");
  dataFile.print(rtc.getMonth());
  dataFile.print("-");
  dataFile.println(rtc.getYear());
  dataFile.print("# Unixtime: ");
  dataFile.println(rtc.getEpoch());
  dataFile.println();
  
  dataFile.flush();

  //---- get start time and intialize mouse_last_seen --------------------------
  starttime = rtc.getEpoch();
  
  //initialize last seen time with starttime unless provided in setup section
  for(uint8_t i = 1; i < 13; i++) //exculde mouse 0 from time
  {
    if(mouse_last_seen[i] == 0)
    {
      mouse_last_seen[i] = starttime;
    }
  }
  
} //end of setup

//##############################################################################
//#####   L O O P   ############################################################
//##############################################################################
void loop()
{
  //----------------------------------------------------------------------------
  //record RFID tags -----------------------------------------------------------
  //----------------------------------------------------------------------------
  //takes 198us from start to finishing send, and further 15.2us until rfid-module turns off RFID reader
  //>=80ms are required to cold-start a tag for a successful read (at reduced range)
  //>=90ms for full range, increasing further only seems to increase range due to noise
  //rather than requirements of the tag and coil for energizing (100ms is chosen as a compromise)
  //REG_PORT_OUTSET0 = PORT_PA21;
  //REG_PORT_OUTCLR0 = PORT_PA21;
  
  // make a string for assembling the data to log:
  String RFIDdataString = "";
  
  //switch between RFID readers
  switch(RFIDmode)
  {
    //alternately switch between both readers
    case 1:
      if((millis() - RFIDtime) >= 100)
      {
        RFIDtime = millis();
        
        if(RFIDtoggle == 1)
        {
          RFIDtoggle = 0; //toggle the toggle

          //enable reader2, disable reader1
          switchReaders(reader2,reader1);
          
          //fetch data reader1 collected during on time saved in variable: tag
          tag1_present = fetchtag(reader1, 1);
          reader1_cycle = 1;
          
          //copy received tag to current tag
          for(uint8_t i = 0; i < sizeof(tag); i++)
          {
            currenttag1[i] = tag[i];
          }
          
          //compare current and last tag
          //0 = no change, 1 = new tag entered, 2 = switch (two present successively), 3 = tag left
          uint8_t tag1_switch = compareTags(currenttag1,lasttag1);
          
          //create datastring that is written to uSD
          RFIDdataString = createRFIDDataString(currenttag1, lasttag1, tag1_present, tag1_switch, "R1");
          
          //copy currenttag to lasttag
          for(uint8_t i = 0; i < sizeof(currenttag1); i++)
          {
            lasttag1[i] = currenttag1[i];
          }
        }
        else
        {
          RFIDtoggle = 1; //toggle the toggle
          
          //enable reader1, disable reader2
          switchReaders(reader1,reader2);
          
          //fetch data reader2 collected during on time saved in variable: tag
          tag2_present = fetchtag(reader2, 1);
          reader2_cycle = 1;
          
          //copy received tag to current tag
          for(uint8_t i = 0; i < sizeof(tag); i++)
          {
            currenttag2[i] = tag[i];
          }
          //compare current and last tag
          //0 = no change, 1 = new tag entered, 2 = switch (two present successively), 3 = tag left
          uint8_t tag2_switch = compareTags(currenttag2,lasttag2);
          
          //create datastring that is written to uSD
          RFIDdataString = createRFIDDataString(currenttag2, lasttag2, tag2_present, tag2_switch, "R2");
          
          //copy currenttag to lasttag
          for(uint8_t i = 0; i < sizeof(currenttag2); i++)
          {
            lasttag2[i] = currenttag2[i];
          }
        }
      }
    break;
    
    //enable only reader1
    case 2:
      //do once
      if(RFIDmode_firstrun == 1)
      {
        RFIDmode_firstrun = 0;
        
        //dis/enable correct readers
        enableReader(reader1);
        disableReader(reader2);
      }
      
      //poll for tag reads every 40ms
      if((millis() - RFIDtime) >= 100)
      {
        RFIDtime = millis();
      
        //fetch tag
        tag1_present = fetchtag(reader1, 0);
      
        //copy received tag to current tag
        for(uint8_t i = 0; i < sizeof(tag); i++)
        {
          currenttag1[i] = tag[i];
        }
      
        //compare current and last tag
        //0 = no change, 1 = new tag entered, 2 = switch (two present successively), 3 = tag left
        uint8_t tag1_switch = compareTags(currenttag1, lasttag1);
      
        //create datastring that is written to uSD
        RFIDdataString = createRFIDDataString(currenttag1, lasttag1, tag1_present, tag1_switch, "R1s");
      
        //copy currenttag to lasttag
        for(uint8_t i = 0; i < sizeof(currenttag1); i++)
        {
          lasttag1[i] = currenttag1[i];
        }
      }
    break;
    
    //enable only reader2
    case 3:
      //do once
      if(RFIDmode_firstrun == 1)
      {
        RFIDmode_firstrun = 0;
        
        //dis/enable correct readers
        enableReader(reader2);
        disableReader(reader1);
      }
      
      //poll for tag reads every 40ms
      if((millis() - RFIDtime) >= 100)
      {
        RFIDtime = millis();
      
        //fetch tag
        tag2_present = fetchtag(reader2, 0);
      
        //copy received tag to current tag
        for(uint8_t i = 0; i < sizeof(tag); i++)
        {
          currenttag2[i] = tag[i];
        }
      
        //compare current and last tag
        //0 = no change, 1 = new tag entered, 2 = switch (two present successively), 3 = tag left
        uint8_t tag2_switch = compareTags(currenttag2,lasttag2);
      
        //create datastring that is written to uSD
        RFIDdataString = createRFIDDataString(currenttag2, lasttag2, tag2_present, tag2_switch, "R2s");
      
        //copy currenttag to lasttag
        for(uint8_t i = 0; i < sizeof(currenttag2); i++)
        {
          lasttag2[i] = currenttag2[i];
        }
      }
    break;
  }
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //do stuff (continuously) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  String SENSORDataString = ""; //create/clear data string
  
  //----------------------------------------------------------------------------
  //check tags at reader 2 -----------------------------------------------------
  //----------------------------------------------------------------------------
  //check current tag against mouse_library, every read cycle if tag present
  if(reader2_cycle && tag2_present)
  {
    reader2_cycle = 0;
    
    //check current tag against library
    current_mouse2 = 0;
    for(uint8_t h = 1; h < 13; h++) //iterate through all tags
    {
      uint8_t tc = 0;
      for(uint8_t i = 0; i < sizeof(currenttag2); i++) //compare byte by byte
      {
        if(currenttag2[i] == mouse_library[h][i])
        {
          tc++;
        }
        else
        {
          break; //stop comparing current tag at first mismatch
        }
      }
      if(tc == 6) //if all 6 bytes are identical, matching tag found
      {
        current_mouse2 = h; //assign detected mouse to variable, mouse 0 is no detection
        break;  //stop looking after first match
      }
    }
    mouse_last_seen[current_mouse2] = rtc.getEpoch();
  }
  
  //----------------------------------------------------------------------------
  //read IR sensors 1+2+3 periodically -----------------------------------------
  //----------------------------------------------------------------------------
  if((millis() - IRsensor_time) >= 50)
  {
    IRsensor_time = millis();
    
    //write sensor Value to buffer
    sensor3_triggered = digitalRead(sensor3); //not triggered by interrupt, set flag here
  
    IR_door1_buffer[sb] = digitalRead(sensor1); //triggered by interrupt, read here as well
    IR_door2_buffer[sb] = digitalRead(sensor2); //triggered by interrupt, read here as well
    IR_middle_buffer[sb] = sensor3_triggered;
    
    //count up buffer position, start again at 0 if end of array is reached
    sb++;
    if(sb > 9)
    {
      sb = 0;
    }
    
    //calculate sum i.e. times IR sensor was interrupted
    IR_door1_buffer_sum = 0;  //reset
    IR_door2_buffer_sum = 0;
    IR_middle_buffer_sum = 0;
    for(uint8_t i = 0; i < 10; i++)
    {
      IR_door1_buffer_sum += IR_door1_buffer[i];
      IR_door2_buffer_sum += IR_door2_buffer[i];
      IR_middle_buffer_sum += IR_middle_buffer[i];
    }
  }
  
  //----------------------------------------------------------------------------
  //manage transitions ---------------------------------------------------------
  //----------------------------------------------------------------------------
  if(sensor1_triggered || (transition_to_hc == 3))
  {
    sensor1_triggered = 0; //flag needs to be cleared (every time)
    
    if(!door1_open && !door1_moving && !door2_open && !door2_moving && //all closed and not moving
      (transition_to_tc == 0) && //not in transition towards testcage
      (((transition_to_hc == 0) && (IR_middle_buffer_sum == 0)) || (transition_to_hc == 3))) //not transitioning to hc and middle must be empty OR already transitioning to hc
    {
      //----- door management
      if(door1_counter >= 50)
      {
        rotate1_duration = rotate(door1, door1_reset_up, 1, door1_speed); //open door too much
        SENSORDataString = createSENSORDataString("D1", "Opening_reset", SENSORDataString); //generate datastring
        door1_counter = 0;
        door1_reset = 1;
      }
      else
      {
        rotate1_duration = rotate(door1, door1_needed_rotation, 1, door1_speed); //open door too much
        SENSORDataString = createSENSORDataString("D1", "Opening", SENSORDataString); //generate datastring
        door1_counter++;
      }
      
      door1_time = millis();
      door1_moving = 1; //set flag, door is opening
      
      //----- transition management
      if(transition_to_hc == 0) //only start transition to tc if not on its way back (transtioin to hc)
      {
        transition_to_tc = 1; //phase 1 origin door opening
      }
      if(transition_to_hc == 3)
      {
        transition_to_hc = 4; //phase 4 target door opening/open
      }
    }
  }
  
  //DOOR 1 CLOSING
  if(door1_open && !door1_moving && //open and not moving
    ((millis() - door1_time) >= (door1_stays_open_min+rotate1_duration)) && //minimum open time passed
      (((IR_door1_buffer_sum <= 3) && //door not blocked
      ((transition_to_tc == 1) || ((IR_middle_buffer_sum == 0) && (transition_to_hc == 4)))) || //starting a transition i.e. has yet to enter middle OR has to leave middle from transition
        ((millis() - door1_time) >= (door1_stays_open_max+rotate1_duration)))) //OR maximum time passed: takes precedence above all conditions
  {
    //----- door management
    if(door1_reset)
    {
      rotate1_duration = rotate(door1, door1_reset_down, 0, door1_speed); //close door to correct height after reset_up
      SENSORDataString = createSENSORDataString("D1", "Closing_reset", SENSORDataString); //maximum logging
      door1_reset = 0;
    }
    else
    {
      rotate1_duration = rotate(door1, door1_needed_rotation, 0, door1_speed); //close door
      SENSORDataString = createSENSORDataString("D1", "Closing", SENSORDataString); //maximum logging
    }
    door1_time = millis();
    door1_moving = 1;
  }
    
  //DOOR 1 OPEN/CLOSED create log entry when door has finished moving
  if(door1_moving && ((millis() - door1_time) >= rotate1_duration))
  {
    //----- door management
    door1_moving = 0; //door has finished moving, calculated from rotate-duration
    if(!door1_open) //if door has finished moving and was open before, it is now closed
    {
      SENSORDataString = createSENSORDataString("D1", "Opened", SENSORDataString); //maximum logging
      door1_open = 1;
    }
    else
    {
      SENSORDataString = createSENSORDataString("D1", "Closed", SENSORDataString); //generate datastring
      door1_open = 0;
      
      //----- transition management
      if(transition_to_tc == 1)
      {
        transition_to_tc = 2; //phase 2 origin door closed
        transition_time = millis();
      }
      transition_to_hc = 0; //phase 5/0 target door closed
    }
  }
  
  //----------------------------------------------------------------------------
  //check mice in transition (both doors closed) -------------------------------
  //transitioning to TestCage
  if(transition_to_tc == 2)
  {
    //for 1s we will read the IR, and afterwards decide if a mouse was present or not.
    if((millis() - transition_time) >= 1000) //training phase 4 (training phase 3 >= 0)
    {
      //if no mouse is/was present, abort
      if(IR_middle_buffer_sum == 0)
      {
        transition_to_tc = 0; //reset
      }
      //if 1 mouse is/was present, continue
      if(IR_middle_buffer_sum > 0)
      {
        transition_to_tc = 3; //phase 3, open door of target cage
      }
    }
  }
  //transitioning to HomeCage (here we don't check for multiple mice, as the way back is always free)
  if(transition_to_hc == 2)
  {
    //for 1000ms we will read the RFID tags/IR of R2, and afterwards decide if a mouse was present or not.
    if((millis() - transition_time) >= 1000) //training phase 4 (training phase 3 >= 0)
    {
      //if no mouse is/was present, abort
      if(IR_middle_buffer_sum == 0)
      {
        transition_to_hc = 0; //abort transition
      }
      //if a mouse is/was present, continue
      if(IR_middle_buffer_sum > 0)
      {
        transition_to_hc = 3; //phase 3, open door towards target cage
      }
    }
  }
  
  //----------------------------------------------------------------------------
  //IR sensor or The Way Back --------------------------------------------------
  //DOOR 2 OPENING
  if(sensor2_triggered  || (transition_to_tc == 3))
  {
    sensor2_triggered = 0; //flag needs to be cleared (every time)
    
    if(!door2_open && !door2_moving && !door1_moving && !door1_open && //all closed and not moving
      (transition_to_hc == 0) && //not transitioning to homecage
      (((transition_to_tc == 0) && (IR_middle_buffer_sum == 0)) || (transition_to_tc == 3))) //not transitioning to tc and middle must be empty OR already transitioning to tc
    {
      //----- transition management
      if(transition_to_tc == 0) //log/transition only if not already transitioning
      {
        SENSORDataString = createSENSORDataString("IR", "T", SENSORDataString); //create IR triggered log entry
        transition_to_hc = 1; //phase 1 origin door opens
      }
      if(transition_to_tc == 3)
      {
        transition_to_tc = 4; //phase 4 target door opens
      }
      //----- door management
      if(door2_counter >= 50)
      {
        rotate2_duration = rotate(door2, door2_reset_up, 1, door2_speed); //open door too much
        SENSORDataString = createSENSORDataString("D2", "Opening_reset", SENSORDataString); //generate datastring
        door2_counter = 0;
        door2_reset = 1;
      }
      else
      {
        rotate2_duration = rotate(door2, door2_needed_rotation, 1, door2_speed); //open door
        SENSORDataString = createSENSORDataString("D2", "Opening", SENSORDataString); //generate datastring
        door2_counter++;
      }
      door2_time = millis(); //opening time
      door2_moving = 1; //set flag, door is opening
    }
  }
  
  //DOOR 2 CLOSING
  if(door2_open && !door2_moving && //open and not moving
    ((millis() - door2_time) >= (door2_stays_open_min+rotate2_duration)) && //minimum open time passed
      (((IR_door2_buffer_sum <= 3) && //door not blocked
      ((transition_to_hc == 1) || ((IR_middle_buffer_sum == 0) && (transition_to_tc == 4)))) || //starting a transition i.e. has yet to enter middle OR has to leave middle from transition
        ((millis() - door2_time) >= (door2_stays_open_max+rotate2_duration)))) //OR maximum time passed: takes precedence above all conditions
  {
    //----- door management
    if(door2_reset)
    {
      rotate2_duration = rotate(door2, door2_reset_down, 0, door2_speed); //close door to correct height after reset_up
      SENSORDataString = createSENSORDataString("D2", "Closing_reset", SENSORDataString); //maximum logging
      door2_reset = 0;
    }
    else
    {
      rotate2_duration = rotate(door2, door2_needed_rotation, 0, door2_speed); //close door
      SENSORDataString = createSENSORDataString("D2", "Closing", SENSORDataString); //maximum logging
    }
    door2_time = millis();
    door2_moving = 1;
  }
  
  //DOOR 2 OPEN/CLOSED create log entry when door has finished moving
  if(door2_moving && ((millis() - door2_time) >= rotate2_duration))
  {
    //----- door management
    door2_moving = 0; //door has finished moving, calculated from rotate-duration
    if(!door2_open) //if door has finished moving and was open before, it is now closed
    {
      SENSORDataString = createSENSORDataString("D2", "Opened", SENSORDataString); //maximum logging
      door2_open = 1;
    }
    else
    {
      SENSORDataString = createSENSORDataString("D2", "Closed", SENSORDataString); //generate datastring
      door2_open = 0;
      
      //----- transition management
      transition_to_tc = 0; //phase 5/0 target door closed
      if(transition_to_hc == 1) //phase 2 only if already in phase 1
      {
        transition_to_hc = 2; //phase 2 origin door closed
        transition_time = millis();
      }
    }
  }
  
  //FAILSAFE opens door towards homecage ---------------------------------------
  //if a mouse is detected 8/10, open door towards home cage
  if(!door1_open && !door1_moving && !door2_open && !door2_moving && //all closed and not moving
    (transition_to_hc == 0) && (transition_to_tc == 0) && (IR_middle_buffer_sum >= 8)) //not transitioning and IR middle triggered
  {
    //create log entry, failsafe triggered and emulate transition: mouse has to leave towards homecage
    SENSORDataString = createSENSORDataString("FS", "failsafe", SENSORDataString); //generate datastring
    transition_to_hc = 3;
  }
  
    
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //do other stuff (every now and then) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  //do stuff every 10 seconds (that needs rtc) ---------------------------------
  if((millis() - rtccheck_time) > 10000)
  {
    rtccheck_time = millis();
    uint32_t rtc_now = rtc.getEpoch(); //~5.8ms
    
    //check if a mouse didn't visit the testcage -------------------------------
    for(uint8_t i = 1; i < 13; i++)
    {
      uint32_t time_since_R2 = rtc_now - mouse_last_seen[i];    //seen at R2
      
      //remove warning flag when participating again
      if((time_since_R2 < warn_time) && (mouse_participation[i] == 2))
      {
        mouse_participation[i] = 1; //reset mouse warning flag
        SENSORDataString = createSENSORDataString("Mr", String(i), SENSORDataString); //Mr = mouse re-participation
      }
      
      //add warning to log on low participation
      if((time_since_R2 >= warn_time) && (mouse_participation[i] == 1))
      {
        mouse_participation[i] = 2; //mouse warning, low participation
        SENSORDataString = createSENSORDataString("MwR", String(i), SENSORDataString); //Mw = mouse warning
      }
    }
    
  }
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//write Data to log files ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
	//log sensor/motor events
  if(SENSORDataString.length() != 0)
  {
    //append Datastring to file
    dataFile.println(SENSORDataString);
    dataFile.flush();
    if(is_testing == 1)
    {
      Serial.println(SENSORDataString);
    }
  }
  //log RFID events
  if(RFIDdataString.length() != 0) //13.7ms
	{
		//append Datastring to file
		dataFile.println(RFIDdataString);
		dataFile.flush();
		if(is_testing == 1)
		{
			Serial.println(RFIDdataString);
		}
	}

} //end of loop

//##############################################################################
//#####   F U N C T I O N S   ##################################################
//##############################################################################

//Return time as string in HH:MM:SS format
String nicetime()
{
	String ntime = "";
	int h = rtc.getHours() + GMT;
	int m = rtc.getMinutes();
	int s = rtc.getSeconds();

	if (h < 10)
	{
		ntime += "0";
		ntime += h;
	}
	else
	{
		ntime += h;
	}
	ntime += ":";

	if (m < 10)
	{
		ntime += "0";
		ntime += m;
	}
	else
	{
		ntime += m;
	}
	ntime += ":";

	if (s < 10)
	{
		ntime += "0";
		ntime += s;
	}
	else
	{
		ntime += s;
	}
	return ntime;
}

//Sensors related functions
String createSENSORDataString(String identifier, String event, String dataString)
{
  //if datastring is not empty, add newline (for pretty formatting)
  if(dataString != 0)
  {
    dataString += "\n";
  }
  
  dataString += identifier;
  dataString += ",";
  dataString += rtc.getEpoch();
  dataString += ",";
  dataString += ""; //here would be the tag
  dataString += ",";
  dataString += millis();
  dataString += ",";
  dataString += event; //T for triggered (can be enter/exit as well --> ISR CHANGE)

  return dataString;
}

//Interrupt service routines ---------------------------------------------------
void sensor1_ISR()
{
  sensor1_triggered = 1;
}
void sensor2_ISR()
{
  sensor2_triggered = 1;
}
void sensor3_ISR()
{
  sensor3_triggered = 1;
}

//RFID tag realted functions ---------------------------------------------------
//get ID in string format
String getID(uint8_t in[6])
{
  uint64_t in64 = 0;
  in64 |= (in[4] & 0b111111);
  in64 <<= 8;
  in64 |= in[3];
  in64 <<= 8;
  in64 |= in[2];
  in64 <<= 8;
  in64 |= in[1];
  in64 <<= 8;
  in64 |= in[0];

  String result = "";
  while(in64)
  {
    char c = in64 % 10;
    in64 /= 10;
    c += '0'; //add to character zero
    result = c + result; //concatenate
  }
  return result;
}

//convert byte array to char
uint16_t getCountryCode(uint8_t in[6])
{
  uint16_t countrycode = 0;
  countrycode = ((countrycode | in[5]) << 2) | ((in[4] >> 6) & 0b11);
  return countrycode;
}

//enable one reader, wait for cofirmation from reader
void enableReader(byte reader)
{
  uint8_t send_status = 1;
  while(send_status != 0)
  {
    //enable reader
    Wire.beginTransmission(reader);
    Wire.write(1);
    send_status = Wire.endTransmission();
    
    digitalWrite(LED1,HIGH);
    delay(200);
    digitalWrite(LED1,LOW);
    delay(200);
  }
}

//disable one reader, wait for cofirmation from reader
void disableReader(byte reader)
{
  uint8_t send_status = 1;
  while(send_status != 0)
  {
    //disable reader
    Wire.beginTransmission(reader);
    Wire.write(0);
    send_status = Wire.endTransmission();
    
    digitalWrite(LED1,HIGH);
    delay(50);
    digitalWrite(LED1,LOW);
    delay(50);
  }
}

//switch between two readers, optimized timing for minimum downtime
void switchReaders(byte readerON, byte readerOFF)
{
  //turn on one reader
  Wire.beginTransmission(readerON);
  Wire.write(1);
  Wire.endTransmission();
  delayMicroseconds(1200); //reduces down-time of antennas since startup isn't instant
  //turn off the other
  Wire.beginTransmission(readerOFF);
  Wire.write(0);
  Wire.endTransmission();
}

//fetch tag data from reader
byte fetchtag(byte reader, byte busrelease)
{
  //request tag-data from reader
  Wire.requestFrom(reader,6,busrelease); //address, quantity ~574uS, bus release
  int n = 0;
  while(Wire.available())
  {
    tag[n] = Wire.read();
    n++;
  }
  
  //sum received values
  int tag_sum = 0;
  for(uint8_t i = 0; i < sizeof(tag); i++)
  {
    tag_sum = tag_sum + tag[i];
  }
    
  //if tag is empty, no tag was detected
  if(tag_sum > 0)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

//compare current and last tag
byte compareTags(byte currenttag[], byte lasttag[])
{
  //compare lasttag and currenttag if the tag changes, skipped if no tag was present before
  int tagchange = 0; //0 = no change, 1 = new tag entered, 2 = switch (2 present), 3 = tag left
  int lasttag_sum = 0;
  int currenttag_sum = 0;
  for(uint8_t i = 0; i < sizeof(lasttag); i++)
  {
    if(currenttag[i] != lasttag[i]) //if diff between current and last tag, something changed
    {
      //check if arrays are empty by summing all values
      for(uint8_t j = 0; j < sizeof(lasttag); j++)
      {
        lasttag_sum = lasttag_sum + lasttag[j];
        currenttag_sum = currenttag_sum + currenttag[j];
      }
      
      //if lasttag is empty but not currenttag: 1 = new tag entered
      if(lasttag_sum == 0)
      {
        tagchange = 1;
      }
      //if lasttag wasn't empty nad currenttaf isn't either, tags switched (two present, one left)
      if((lasttag_sum != 0) && (currenttag_sum != 0))
      {
        tagchange = 2;
      }
      //if currenttag is empty, but not last tag, 3 = tag left
      if(currenttag_sum == 0)
      {
        tagchange = 3;
      }
      break;
    }
  }

  //return how (if) the tag changed
  return(tagchange);
}

//create string that is later saved to uSD
String createRFIDDataString(byte currenttag[], byte lasttag[], byte currenttag_present, int tagchange, String identifier)
{
  String dataString;
  
  //get country code and tag ID
  int ctCC = getCountryCode(currenttag);
  int ltCC = getCountryCode(lasttag);

  String ctID = getID(currenttag);
  String ltID = getID(lasttag);

  //save tag data to dataString which is written to SD
  //tag left (3) or switch (2)
  if((tagchange == 2) || (tagchange == 3))
  {
    dataString += identifier;
    dataString += ",";
    dataString += rtc.getEpoch();
    dataString += ",";
    dataString += ltCC;
    dataString += "_";
    dataString += ltID;
    dataString += ",";
    dataString += millis();
    dataString += ",X";
  }
  
  //insert newline when a switch happens
  if(tagchange == 2)
  {
    dataString += "\n";
  }
  
  //new tag entered (1) or switch (2)
  if((tagchange == 1) || (tagchange == 2))
  {
    dataString += identifier;
    dataString += ",";
    dataString += rtc.getEpoch();
    dataString += ",";
    dataString += ctCC;
    dataString += "_";
    dataString += ctID;
    dataString += ",";
    dataString += millis();
    dataString += ",E";
  }
  return dataString;
}

//rotate steppers --------------------------------------------------------------
//returns duration until rotation is finished in milliseconds
uint16_t rotate(uint8_t address, uint32_t steps, uint8_t direction, uint8_t speed)
{
  uint8_t sendbuffer[6] = {0,0,0,0,0,0};
  
  //steps: 3200 steps == 1 revolution
  //create 4 bytes from 32bit variable
  sendbuffer[0] = steps & 0xff;         //first 8 bits
  sendbuffer[1] = (steps >>  8) & 0xff; //next 8 bits
  sendbuffer[2] = (steps >> 16) & 0xff; //next 8 bits
  sendbuffer[3] = (steps >> 24) & 0xff; //last 8 bits

  //direction: 0/1
  //speed: 0-255 (<= 230 recommended)
  sendbuffer[4] = direction;
  sendbuffer[5] = 255 - speed;
  
  int send_status = 1;
  while(send_status != 0)
  {
    Wire.beginTransmission(address); //address
    for(uint8_t i = 0; i < 6; i++)
    {
      Wire.write(sendbuffer[i]);
    }
    send_status = Wire.endTransmission();
  }
  
  //(pin_toggle_time) + (delay_between_steps) * 2 * steps / convert_to_ms + rounding
  return (((0.4 + (255-speed)) * 2 * steps) / 1000) + 0.5;
}

//critical error, flash LED SOS ------------------------------------------------
void criticalerror()
{
  while(1)
  {
    digitalWrite(LED1,HIGH);
    delay(100);
    digitalWrite(LED1,LOW);
    delay(100);
    digitalWrite(LED1,HIGH);
    delay(100);
    digitalWrite(LED1,LOW);
    delay(100);
    digitalWrite(LED1,HIGH);
    delay(100);
    digitalWrite(LED1,LOW);
    delay(100);
    
    digitalWrite(LED1,HIGH);
    delay(300);
    digitalWrite(LED1,LOW);
    delay(100);
    digitalWrite(LED1,HIGH);
    delay(300);
    digitalWrite(LED1,LOW);
    delay(100);
    digitalWrite(LED1,HIGH);
    delay(300);
    digitalWrite(LED1,LOW);
    delay(100);
    
    digitalWrite(LED1,HIGH);
    delay(100);
    digitalWrite(LED1,LOW);
    delay(100);
    digitalWrite(LED1,HIGH);
    delay(100);
    digitalWrite(LED1,LOW);
    delay(100);
    digitalWrite(LED1,HIGH);
    delay(100);
    digitalWrite(LED1,LOW);
    delay(500);
    
  }
}

//--- STUFF
/*
//check current tag against mouse_library
for(uint8_t h = 1; h < 13; h++) //iterate through all tags
{
  uint8_t tc = 0;
  for(uint8_t i = 0; i < sizeof(currenttag1); i++) //compare byte by byte
  {
    if(currenttag1[i] == mouse_library[h][i])
    {
      tc++;
    }
    else
    {
      break; //stop comparing current tag at first mismatch
    }
  }
  if(tc == 6) //if all 6 bytes are identical, matching tag found
  {
    current_mouse1 = h; //assign detected mouse to variable, mouse 0 is no detection
    break;  //stop looking after first match
  }
}

if(current_mouse1) //log time of last attempted poke on timeout, if mouse was detected (could also log last time a mouse was not detected...)
{
  mouse_last_seen[current_mouse1][0] = rtc.getEpoch(); //TODO: think and remove
}
*/
