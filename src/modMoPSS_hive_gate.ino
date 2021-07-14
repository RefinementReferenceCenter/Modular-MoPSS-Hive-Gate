//------------------------------------------------------------------------------
/*
--- Adafruit ItsyBitsy M0 pin mapping - Hardware Revision v6.1 ---

 A0- (J10 upper 2-pin)
 A1- Button 1,2,3
 A2- uSD ChipSelect
 A3- (J3 lower 3-pin)
 A4- (J2 lower 3-pin)
 A5- (J1 lower 3-pin)

 D0- (J8 upper debounce 2-pin)
 D1- (J7 upper 3-pin)
 D2- RTC INTerrupt/SQuareWave
 D3- (J9 upper debounce 2-pin)
 D4- (J6 upper 3-pin)
 D5- DEBUG for timing (5V out!)
 D7- (J4 lower 3-pin)
 D9- (J5 lower 3-pin)
D10- WiFi GPIO 0
D11- WiFi Busy
D12- WiFi Reset
D13- WiFi CS

D22- I2C SDA
D23- I2C SCL

--- Experimental Setup ---

^^^^^^^\                                                         /^^^^^^^^
       |                                                         |
h c    |     |R|    |I| | D |   |I|   |I|   | D | |I|    |R|     |    t  c
o a  ––|–––––|F|––––|R|–| O |–––|R|–-–|R|–-–| O |–|R|––––|F|–––––|––  e  a
m g    |     |I|    | | | O |   | |   | |   | O | | |    |I|     |    s  g
e e  ––|–––––|D|––––| |–| R |–––| |–-–| |––-| R |–| |––––|D|–––––|––  t  e
       |     |1|    |1| | 1 |   |3|   |4|   | 2 | |2|    |2|     |
       |                                                         |
______/                   |-----  13 cm  ----|                   \________

*/

//------------------------------------------------------------------------------
#include <Wire.h>             //I2C communication
#include <SD.h>               //Access to SD card
#include <RTClib.h>           //(Adafruit) Provides softRTC as a workaround
#include <WiFiNINA.h>         //(Adafruit) Wifi Chipset modified version from Adafruit
#include <Adafruit_DotStar.h> //(Adafruit) controlling the onboard dotstar RGB LED
#include <U8g2lib.h>          //for SSD1306 OLED Display

//----- declaring variables ----------------------------------------------------
//Current Version of the program
//##############################################################################
//##############################################################################
const char HARDWARE_REV[] = "v6.1";
//##############################################################################
//##############################################################################

//Buttons
const uint8_t buttons = A1; //Bottom: ~6, Middle: ~347, Top: ~688, None: 1023

//Sensors
const int sensor1 = A5; //IR 1 door 1
const int sensor2 = A4; //IR 2 door 2
const int sensor3 = A3; //IR 3 middle left
const int sensor4 = 7; //IR 4 middle right

//use volatile if interrupt based
volatile uint8_t sensor1_triggered = 0;     //IR 1 door 1
volatile uint8_t sensor2_triggered = 0;     //IR 2 door 2
/*volatile*/ uint8_t sensor3_triggered = 0; //IR 3 middle left
/*volatile*/ uint8_t sensor4_triggered = 0; //IR 4 middle right

uint8_t IR_door1_buffer[10] = {};   //10 reads, every 50ms = 0.5s buffer length
uint8_t IR_door2_buffer[10] = {};
uint8_t IR_middleL_buffer[10] = {};
uint8_t IR_middleR_buffer[10] = {};
uint8_t sb = 0;                    //sensor buffer counter
uint8_t IR_door1_buffer_sum = 0;    //holds count of IR triggers in the last 0.5s
uint8_t IR_door2_buffer_sum = 0;
uint8_t IR_middleL_buffer_sum = 0;
uint8_t IR_middleR_buffer_sum = 0;
unsigned long IRsensor_time;       //time when IR sensors 1,2,3 were last checked

//LEDs
volatile uint8_t ledstate = LOW;
Adafruit_DotStar strip(1, 41, 40, DOTSTAR_BRG); //create dotstar object

//SD
const uint8_t SDcs = A2; //ChipSelect pin for SD (SPI)
File dataFile;           //create file object

//WiFi
char ssid[] = "Buchhaim";           //network name
char pass[] = "2416AdZk3881QPnh+";  //WPA key
int status = WL_IDLE_STATUS;        //initialize for first check if wifi up
#define SPIWIFI SPI                  //the SPI port

//RTC - Real Time Clock
const uint8_t GMT = 2;  //current timezone (Sommerzeit)
RTC_DS3231 rtc;         //create rtc object

//Display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0,U8X8_PIN_NONE,23,22); //def,reset,SCL,SDA
uint32_t displaytime = 0;         //stores millis to time display refresh
uint8_t displayon = 1;            //flag to en/disable display

//Door Modules
const uint8_t door1 = 0x10;       //I2C address door module 1
const uint8_t door2 = 0x11;       //I2C address door module 2

uint8_t door1_open = 0;           //flag if door is open
uint8_t door2_open = 0;
uint8_t door1_moving = 0;         //flag if door is moving
uint8_t door2_moving = 0;
unsigned long door1_time;         //stores time when door starts moving
unsigned long door2_time;
uint16_t rotate1_duration;        //duration until door has finished movement
uint16_t rotate2_duration;
uint16_t door1_counter = 0;       //counter how often door was opened (used to reset door position)
uint16_t door2_counter = 0;
uint8_t door1_reset = 0;          //flag is set if door needs to be "re-set"
uint8_t door2_reset = 0;
uint8_t door1_just_closed = 0;    //relic from old code
uint8_t door2_just_closed = 0;

unsigned long door1_poll_time;     //time when we should query the door module for a status update
unsigned long door2_poll_time;

//names for door module control
const uint8_t top = 0;
const uint8_t upper = 1;
const uint8_t middle = 2;
const uint8_t lower = 3;
const uint8_t bottom = 4;

const uint8_t up = 0;
const uint8_t down = 1;

uint8_t door1_IR_state[7]; //0 IR_top, IR_upper, IR_middle, IR_lower, IR_bottom, IR_barrier_rx, 6 IR_barrier_tx
uint8_t door2_IR_state[7];

//RFID
const uint8_t reader1 = 0x08;     //I2C address RFID module 1
const uint8_t reader2 = 0x09;     //I2C address RFID module 2

unsigned long RFIDtime;           //used to measure time before switching to next antenna
uint8_t RFIDtoggle = 0;           //flag used to switch to next antenna

uint8_t tag[6] = {};              //global variable to store returned tag data (limitation of C to return arrays)
uint8_t tag1_present = 0;         //flag that indicates if tag was present during read cycle
uint8_t tag2_present = 0;
uint8_t reader1_cycle = 0;        //toggles flag if a read cycle has just happened (not automatically cleared)
uint8_t reader2_cycle = 0;
uint8_t currenttag1[6] = {};      //saves id of the tag that was read during the current read cycle
uint8_t currenttag2[6] = {};
uint8_t lasttag1[6] = {};         //saves id of the tag that was read during the previous read cycle
uint8_t lasttag2[6] = {};

//TODO: test switching modes
uint8_t RFIDmode = 1;           //select mode to operate in: 1-alternate, 2-reader1, 3-reader2
uint8_t RFIDmode_firstrun = 1;   //to make sure the correct reader is turned on/off

//Experiment variables
//0 = no transition, 1 = origin door opened, 2 = origin door closed, 3 = checks mice present, 4 = target door closing
uint8_t transition_to_tc = 0;         //track transition phase to target cage
uint8_t transition_to_hc = 0;         //track transition phase to home cage
unsigned long transition_time = 0;    //time when both doors are closed, when mouse is in the middle
uint8_t tc_occupied = 0;              //flag that tracks occupation of test cage
uint8_t failsafe_triggered = 0;       //only needed for phase 2

unsigned long starttime;        //start of programm
unsigned long rtccheck_time;    //time the rtc was checked last

uint8_t d2_timeout = 0;         //flag stores if d2 closed from timeout if transitioning to tc
unsigned long d2_timeout_time;  //stores time when timeout happened

//Mice tags
const uint8_t mice = 15;           //number of mice in experiment (add 1 for mouse 0, add 2 for test-mice)
const uint8_t mouse_library[mice][6] = {
  {0x00,0x00,0x00,0x00,0x00,0x00}, //mouse 0
  {0x8A,0x68,0xF7,0x90,0x2E,0xE1}, //mouse 1  we_ge 900 200000628874 e12e90f7 68 8a #funktioniert
  {0xFE,0x68,0xF7,0x90,0x2E,0xE1}, //mouse 2  we_ro 900 200000628990 e12e90f7 68 fe
  {0xE5,0x66,0xF7,0x90,0x2E,0xE1}, //mouse 3  we_sw 900 200000628453 e12e90f7 66 e5
  {0xB8,0x63,0xF7,0x90,0x2E,0xE1}, //mouse 4  we_si 900 200000627640 e12e90f7 63 b8
  {0x92,0x61,0xF7,0x90,0x2E,0xE1}, //mouse 5  sw_ge 900 200000627090 e12e90f7 61 92
  {0x41,0x74,0xF7,0x90,0x2E,0xE1}, //mouse 6  sw_ro 900 200000631873 e12e90f7 74 41
  {0x75,0x69,0xF7,0x90,0x2E,0xE1}, //mouse 7  sw_we 900 200000629109 e12e90f7 69 75
  {0xCA,0x63,0xF7,0x90,0x2E,0xE1}, //mouse 8  sw_si 900 200000627658 e12e90f7 63 ca
  {0x26,0x6E,0xF7,0x90,0x2E,0xE1}, //mouse 9  ro_ge 900 200000630310 e12e90f7 6e 26
  {0xF4,0x72,0xF7,0x90,0x2E,0xE1}, //mouse 10 ro_we 900 200000631540 e12e90f7 72 f4
  {0x6B,0x74,0xF7,0x90,0x2E,0xE1}, //mouse 11 ro_sw 900 200000631915 e12e90f7 74 6b
  {0x0C,0x67,0xF7,0x90,0x2E,0xE1}, //mouse 12 ro_si 900 200000628492 e12e90f7 67 0c
  {0xA1,0x82,0x42,0xDD,0x3E,0xF3}, //mouse 13 polymorphmaus
  {0x66,0x6D,0xF7,0x90,0x2E,0xE1}  //mouse 14 bleistiftmaus2 900_200000630118 e12e90f76d66
};
uint8_t mice_visits[mice][2];      //contains the number of tag reads at reader 1 and 2 since last reset

uint8_t update_current_mouse1 = 1; //flag for updating current mouse at reader 1
uint8_t update_current_mouse2 = 1; //flag for updating current mouse at reader 2
uint8_t current_mouse1 = 0;        //placeholder simple number for tag at reader 1
uint8_t current_mouse2 = 0;        //placeholder simple number for tag at reader 2

uint32_t oledt1 = 0;
uint32_t oledt2 = 0;

//TESTING
uint8_t manual_trigger = 0; //manually trigger gate-events for testing
uint32_t test_timer = 0;

//##############################################################################
//#####   U S E R   C O N F I G  ###############################################
//##############################################################################

//if set to 1, the MoPSS will wait for a wifi connection and synchronization with network time before continuing
const uint8_t enable_wifi = 1;

//if set to 1, the MoPSS will wait until a serial connection via USB is established
//before continuing. Also prints what is written to uSD to Serial as well.
const uint8_t is_testing = 1;

//a high amount of sensor data will be written to log
//debug == 0: logs nothing
//debug <= 1: logs interrupts of IR1 and IR2
//debug <= 3: as above and logs transition management variables
//debug <= 4: as above and logs buffer_sum of all IR sensors continuously, every 500ms (all buffer events!)
const uint8_t debug = 3;

//------------------------------------------------------------------------------
//Habituation phase
//1: Both doors always open
//2: Synchronous movement of both doors, Triggered by IR1/IR2/Failsafe(middle)
//3: Transition management enabled, mulitmice, mouse_limit and transition delay options available
uint8_t habituation_phase = 2;

//enable or disable multimice detection
uint8_t multimice = 0;

//limit the number of mice allowed in the test cage (1 or no limit)
uint8_t mouse_limit = 0;

//time mouse is kept in transition with both doors closed in phase 4 (ms)
uint16_t transition_delay = 0;


//door and transition management
const uint16_t door1_speed = 250;             //speed is the us delay between steps. 0 is fastest possible, and slower the higher the value
const uint16_t door2_speed = 250;
const uint16_t door1_stays_open_min = 3000;   //minimum open time ms
const uint16_t door2_stays_open_min = 3000;
const uint16_t door1_stays_open_max = 10000;  //maximum open time ms, time door will stay open if mice dwell between doors, not if triggering IR1/2
const uint16_t door2_stays_open_max = 10000;

//------------------------------------------------------------------------------
//For easier data evaluation or feedback, mouse participation and warnings can be set here
//0 = does not participate; 1 = regular participation; 2 = warning; 3 = excluded from experiment
//warning label 2 is handled automatically, DON'T SET!
uint8_t mouse_participation[mice] = {
  1, //mouse 0
  1, //mouse 1  sw_si 1923
  1, //mouse 2  ro_ge 8095
  1, //mouse 3  sw_ro 1616
  1, //mouse 4  we_sw 1858
  1, //mouse 5  ro_we 1992
  1, //mouse 6  sw_ge 0296
  1, //mouse 7  we_si 1210
  1, //mouse 8  ro_si 0379
  1, //mouse 9  sw_we 1055
  1, //mouse 10 we_ge 0647
  1, //mouse 11 ro_sw 7857
  1, //mouse 12 we_ro 1617
  1, //mouse 13 polymorphmaus
  1}; //mouse 14 bleistiftmaus

//manually enter time mouse was last seen. accepted input is time in unixtime format
uint32_t mouse_last_seen[mice] = {
  0, //mouse 0
  0, //mouse 1  sw_si 1923
  0, //mouse 2  ro_ge 8095
  0, //mouse 3  sw_ro 1616
  0, //mouse 4  we_sw 1858
  0, //mouse 5  ro_we 1992
  0, //mouse 6  sw_ge 0296
  0, //mouse 7  we_si 1210
  0, //mouse 8  ro_si 0379
  0, //mouse 9  sw_we 1055
  0, //mouse 10 we_ge 0647
  0, //mouse 11 ro_sw 7857
  0, //mouse 12 we_ro 1617
  0, //mouse 13 polymorphmaus
  0}; //mouse 14 bleistiftmaus

//time after which a warning is logged (ms)
const uint32_t warn_time = 60*60*24*1000;


//##############################################################################
//#####   S E T U P   ##########################################################
//##############################################################################
void setup()
{
  //----- DEBUGGING ------------------------------------------------------------
  // pinMode(5,OUTPUT); //to allow port toggle for timing purposes D5 PA15
  // pinMode(A0,OUTPUT);
  // pinMode(4,OUTPUT);
  
  //Set up RGB LED on board, and turn it off ------------------------------------
  strip.begin(); //Initialize pins for output
  strip.show();  //Turn all LEDs off ASAP
  
  //----- communication --------------------------------------------------------
  //start Serial communication
  Serial.begin(115200);
  if(is_testing == 1)
  {
    //while(!Serial); //wait for serial connection
    Serial.println("alive");
  }
  
  //start I2C
  Wire.begin(); //atsamd can't multimaster
  
  //----- Display --------------------------------------------------------------
  oled.setI2CAddress(0x3C*2);
  oled.begin();
  oled.setFont(u8g2_font_6x10_mf); //set font w5 h10
  OLEDprint(0,0,1,1,">>> Module Setup <<<");

  //----- RFID readers ---------------------------------------------------------
  OLEDprint(1,0,0,1,"Setup RFID readers...");
  //disable RFID readers and wait until they acknowledge
  disableReader(reader1);
  disableReader(reader2);
  OLEDprint(2,0,0,1,"-Done");
  
  //----- Buttons --------------------------------------------------------------
  pinMode(buttons,INPUT);
  
  //----- Sensors --------------------------------------------------------------
  //setup input mode for pins (redundant, for clarity)
  pinMode(sensor1,INPUT); //IR 1 door 1
  pinMode(sensor2,INPUT); //IR 2 door 2
  pinMode(sensor3,INPUT); //IR 3 middle left
  pinMode(sensor4,INPUT); //IR 3 middle right
  
  //attach interrupt to all pins (must not analogRead interruptpins or else interrupt stops working!)
  attachInterrupt(digitalPinToInterrupt(sensor1), sensor1_ISR, RISING); //IR 1 door 1
  attachInterrupt(digitalPinToInterrupt(sensor2), sensor2_ISR, RISING); //IR 2 door 2
  //attachInterrupt(digitalPinToInterrupt(sensor3), sensor3_ISR, RISING); //IR 3 middle left
  //attachInterrupt(digitalPinToInterrupt(sensor4), sensor4_ISR, RISING); //IR 4 middle right
  
  //----- Doors ----------------------------------------------------------------
  OLEDprint(3,0,0,1,"Setup Doors...");
  //complete movement up and down
  //moveDoor(door1,up,top,door1_speed);
  //moveDoor(door2,up,top,door2_speed);
  while(getDoorStatus(door1) || getDoorStatus(door2)) //while busy wait for move to finish
  {
    delay(250);
  }
  //moveDoor(door1,down,bottom,door1_speed);
  //moveDoor(door2,down,bottom,door2_speed);
//  while(getDoorStatus(door1) || getDoorStatus(door2)) //while busy wait for move to finish
//  {
//    delay(250);
//  }
  
  OLEDprint(4,0,0,1,"-Done");
  delay(1000); //small delay before clearing display in next step
  
  //----- WiFi -----------------------------------------------------------------
  OLEDprint(0,0,1,1,">>>  WiFi Setup  <<<");
  
  if(enable_wifi == 1)
	{
		Serial.println("----- WiFi Setup -----"); //check if the WiFi module works
    OLEDprint(1,0,0,1,"Connect to WiFi...");
    OLEDprint(2,0,0,1,"SSID:Buchhaim");
    OLEDprint(3,0,0,1,"Key:2416AdZk3881QPnh+");
    
    WiFi.setPins(13, 11, 12, -1, &SPIWIFI); //function only available in adafruit WiFi library
    
		if(WiFi.status() == WL_NO_MODULE)
    {
			Serial.println("WiFi chip not working/disconnected, program stopped!");
      OLEDprint(4,0,0,1,"---WiFi not working");
      OLEDprint(5,0,0,1,"---program stopped!");
			criticalerror(); //don't continue
		}
    
    //attempt to connect to WiFi network:
    Serial.print("Connecting to SSID: ");
    Serial.println(ssid);
    OLEDprint(4,0,0,1,"-Connecting: Try:");
    uint8_t numberOfTries = 0;
    while(status != WL_CONNECTED)
    {
      numberOfTries++;
      Serial.print("Waiting to connect, attempt No.: ");
      Serial.println(numberOfTries);
      OLEDprint(4,17,0,1,numberOfTries);
      
      //Connect to WPA/WPA2 network
      status = WiFi.begin(ssid, pass);
      
      //wait 10 seconds between each connection attempt
      delay(10000);
    }
    Serial.println("Successfully connected!");
    OLEDprint(4,0,0,1,"-Connecting: Success! ");
    
    //----- Real Time Clock ------------------------------------------------------
    Serial.println("----- RTC Setup -----");
    
    unsigned long epoch = 0; //stores the time in seconds since beginning
    
    //repeatedly contact NTP server
    Serial.println("Attempting to reach NTP server");
    OLEDprint(5,0,0,1,"-Sync RTC: Try:");
    numberOfTries = 0;
    while(epoch == 0)
    {
      numberOfTries++;
      Serial.print("Attempt No. ");
      Serial.println(numberOfTries);
      OLEDprint(5,16,0,1,numberOfTries);
      
      delay(1000);
      epoch = WiFi.getTime();
    }
    
    rtc.adjust(DateTime(epoch));
    
    Serial.print("Success! Time received: ");
    Serial.println(nicetime());
    OLEDprint(5,0,0,1,"-Sync RTC: Success!   ");
    
    //disable wifi module after fetching time to conserve power (~83mA)
    WiFi.end();
  } //enablewifibracket
  
  //Set time to zero if WiFi isn't enabled
  if(enable_wifi == 0)
  {
    OLEDprint(1,0,0,1,"WiFi disabled");
    OLEDprint(2,0,0,1,"Setting time to:");
    OLEDprint(3,0,0,1,"01.01.2000 00:00:00");
    WiFi.setPins(13, 11, 12, -1, &SPIWIFI); //function only available in adafruit WiFi library
    rtc.adjust(DateTime(946684800));
    Serial.println("WiFi disabled, setting time to 0");
    WiFi.end(); //disable wifi module TODO: not working
  }
  delay(1000); //short delay to allow reading of scrren
  
  //----- Setup SD Card --------------------------------------------------------
  Serial.println("----- SD-Card Setup -----");
  OLEDprint(0,0,1,1,">>>  uSD  Setup  <<<");
  //see if the card is present and can be initialized:
  if (!SD.begin(SDcs))
  {
    Serial.println("Card failed, or not present, program stopped!");
    //Stop program if uSD was not detected/faulty (needs to be FAT32 format):
    OLEDprint(1,0,0,1,"uSD Card failed!");
    OLEDprint(2,0,0,1,"program stopped");
    criticalerror();
  }
  else
  {
    Serial.println("SD card initialized successfully!");
    OLEDprint(1,0,0,1,"-setup uSD: Success!");
  }
  
  //----- Setup log file, and write initial configuration ----------------------
  //open file, or create if empty
  dataFile = SD.open("RFIDLOG.TXT", FILE_WRITE); //TODO: add date to filename
  DateTime now = rtc.now();
  
  //get experiment start time
  starttime = now.unixtime();
  
  //TODO: add option to query RFID reader for version and resonant frequency
  //write current version to SD and some other startup/system related information
  dataFile.println("");
  dataFile.print("# Modular MoPSS Hive version: ");
  dataFile.println("not yet implemented");

  dataFile.print("# Door Module 1 version: ");
  dataFile.println("not yet implemented");
  dataFile.print("# Door Module 2 version: ");
  dataFile.println("not yet implemented");

  dataFile.print("# RFID Module 1 version: ");
  dataFile.println("not yet implemented");
  dataFile.print("# RFID Module 2 version: ");
  dataFile.println("not yet implemented");

  dataFile.print("# Habituation phase: ");
  dataFile.println(habituation_phase);

  dataFile.print("# debug level: ");
  dataFile.println(debug);

  dataFile.print("# System start @ ");
  dataFile.print(nicetime());
  dataFile.print(" ");
  dataFile.print(now.day());
  dataFile.print("-");
  dataFile.print(now.month());
  dataFile.print("-");
  dataFile.println(now.year());
  dataFile.print("# Unixtime: ");
  dataFile.println(starttime);
  dataFile.println();

  dataFile.flush();
  
  //---- setup experiment variables --------------------------------------------
  
  //initialize last seen time with starttime unless provided in setup section
  for(uint8_t i = 1; i < mice; i++) //exclude mouse 0 from time
  {
    if(mouse_last_seen[i] == 0)
    {
      mouse_last_seen[i] = starttime;
    }
  }
  
  //Move both doors up for habituation phase 1
  if(habituation_phase == 1)
  {
    moveDoor(door1,up,top,door1_speed);
    moveDoor(door2,up,top,door2_speed);
    while(getDoorStatus(door1) || getDoorStatus(door2)) //while busy wait for move to finish
    {
      delay(250);
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
  // REG_PORT_OUTSET0 = PORT_PA08; //J6/D3
  // REG_PORT_OUTCLR0 = PORT_PA08; //J6/D3
  // REG_PORT_OUTSET0 = PORT_PA02; //J10/A0
  // REG_PORT_OUTCLR0 = PORT_PA02; //J10/A0
  //REG_PORT_OUTSET0 = PORT_PA15; //D5
  //REG_PORT_OUTCLR0 = PORT_PA15; //D5
  
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
  //do stuff (continuously) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  String SENSORDataString = ""; //create/clear data string
  
  //----------------------------------------------------------------------------
  //check tags at RFID readers -------------------------------------------------
  //----------------------------------------------------------------------------
  //check current tag against mouse_library, every read cycle if tag present
  //Reader 1
  if(reader1_cycle && tag1_present)
  {
    reader1_cycle = 0;
    
    //check current tag against library
    current_mouse1 = 0;
    for(uint8_t h = 1; h < mice; h++) //iterate through all tags
    {
      uint8_t tc = 0;
      for(uint8_t i = 0; i < sizeof(currenttag1); i++) //compare byte by byte
      {
        if(currenttag1[i] == mouse_library[h][i]) tc++;
        else break; //stop comparing current tag at first mismatch
      }
      if(tc == 6) //if all 6 bytes are identical, matching tag found
      {
        current_mouse1 = h; //assign detected mouse to variable, mouse 0 is no detection
        break;  //stop looking after first match
      }
    }
    DateTime now = rtc.now();
    mouse_last_seen[current_mouse1] = now.unixtime();
    if(current_mouse1 > 0) mice_visits[current_mouse1][1]++;
  }
  
  //Reader 2
  if(reader2_cycle && tag2_present)
  {
    reader2_cycle = 0;
    
    //check current tag against library
    current_mouse2 = 0;
    for(uint8_t h = 1; h < mice; h++) //iterate through all tags
    {
      uint8_t tc = 0;
      for(uint8_t i = 0; i < sizeof(currenttag2); i++) //compare byte by byte
      {
        if(currenttag2[i] == mouse_library[h][i]) tc++;
        else break; //stop comparing current tag at first mismatch
      }
      if(tc == 6) //if all 6 bytes are identical, matching tag found
      {
        current_mouse2 = h; //assign detected mouse to variable, mouse 0 is no detection
        break;  //stop looking after first match
      }
    }
    DateTime now = rtc.now();
    mouse_last_seen[current_mouse2] = now.unixtime();
    if(current_mouse2 > 0) mice_visits[current_mouse2][2]++;
  }
  
  //----------------------------------------------------------------------------
  //read IR sensors 1+2+3 periodically -----------------------------------------
  //----------------------------------------------------------------------------
  if((millis() - IRsensor_time) >= 50)
  {
    IRsensor_time = millis();
    
    //write sensor Value to buffer
    sensor3_triggered = digitalRead(sensor3); //not triggered by interrupt, set flag here
    sensor4_triggered = digitalRead(sensor4); //not triggered by interrupt, set flag here
    
    IR_door1_buffer[sb] = digitalRead(sensor1); //triggered by interrupt, read here as well
    IR_door2_buffer[sb] = digitalRead(sensor2); //triggered by interrupt, read here as well
    IR_middleL_buffer[sb] = sensor3_triggered;
    IR_middleR_buffer[sb] = sensor4_triggered;
    
    //count up buffer position, start again at 0 if end of array is reached
    sb++;
    if(sb > 9) sb = 0;
    
    //calculate sum i.e. times IR sensor was interrupted
    IR_door1_buffer_sum = 0;  //reset
    IR_door2_buffer_sum = 0;
    IR_middleL_buffer_sum = 0;
    IR_middleR_buffer_sum = 0;
    for(uint8_t i = 0; i < 10; i++)
    {
      IR_door1_buffer_sum += IR_door1_buffer[i];
      IR_door2_buffer_sum += IR_door2_buffer[i];
      IR_middleL_buffer_sum += IR_middleL_buffer[i];
      IR_middleR_buffer_sum += IR_middleR_buffer[i];
    }
    
    //debug only, print buffer (all)
    if((debug>=4)&&(sb==9)){SENSORDataString=createSENSORDataString("IRB",String(IR_door1_buffer_sum)+":"+String(IR_door2_buffer_sum)+":"+String(IR_middleL_buffer_sum)+":"+String(IR_middleR_buffer_sum),SENSORDataString);}
  }
  
  //----------------------------------------------------------------------------
  //read door status on demand -------------------------------------------------
  //----------------------------------------------------------------------------
  if(door1_moving && ((millis() - door1_poll_time) >= 200))
  {
    door1_poll_time = millis();
    door1_moving = getDoorStatus(door1);
    
    if(!door1_moving)
    {
      door1_open = !door1_open;
      if(door1_open)
      {
        door1_time = door1_poll_time;
        SENSORDataString = createSENSORDataString("D1", "opened", SENSORDataString); //maximum logging
      }
      else
      {
        SENSORDataString = createSENSORDataString("D1", "closed", SENSORDataString); //maximum logging
        //----- transition management
        if(habituation_phase == 3)
        {
          if(transition_to_tc == 1)
          {
            if(debug>=3){SENSORDataString=createSENSORDataString("TM","to_tc2",SENSORDataString);}
            transition_to_tc = 2; //phase 2 origin door closed
            transition_time = millis();
          }
          if(debug>=3){SENSORDataString=createSENSORDataString("TM","to_hc0",SENSORDataString);}
          transition_to_hc = 0; //phase 5/0 target door closed
        }
      }
    }
  }
  
  if(door2_moving && ((millis() - door2_poll_time) >= 200))
  {
    door2_poll_time = millis();
    door2_moving = getDoorStatus(door2);
    
    if(!door2_moving)
    {
      door2_open = !door2_open;
      if(door2_open)
      {
        door2_time = door2_poll_time;
        SENSORDataString = createSENSORDataString("D2", "opened", SENSORDataString); //maximum logging
      }
      else
      {
        SENSORDataString = createSENSORDataString("D2", "closed", SENSORDataString); //maximum logging
        if(habituation_phase == 3)
        {
          //----- transition management
          if(debug>=3){SENSORDataString=createSENSORDataString("TM","to_tc0",SENSORDataString);}
          transition_to_tc = 0; //phase 5/0 target door closed
          if(transition_to_hc == 1) //phase 2 only if already in phase 1
          {
            if(debug>=3){SENSORDataString=createSENSORDataString("TM","to_hc2",SENSORDataString);}
            transition_to_hc = 2; //phase 2 origin door closed
            transition_time = millis();
          }
        }
      }
    }
  }
  
  //----------------------------------------------------------------------------
  //door management for phase 2 ------------------------------------------------
  //----------------------------------------------------------------------------
  if(habituation_phase == 2) //both doors open simultaniously
  {
  
  //DOOR 1/2 OPENING
  if(sensor1_triggered || sensor2_triggered || failsafe_triggered)
  {
    if((debug>=1)&&sensor1_triggered){SENSORDataString=createSENSORDataString("IR1","IR1_interrupt",SENSORDataString);}
    if((debug>=1)&&sensor2_triggered){SENSORDataString=createSENSORDataString("IR2","IR2_interrupt",SENSORDataString);}
    
    sensor1_triggered = 0; //clearing flags
    sensor2_triggered = 0;
    failsafe_triggered = 0;
    
    if(!door1_open && !door1_moving && !door2_open && !door2_moving)  //all closed and not moving
    {
      moveDoor(door1,up,top,door1_speed);
      SENSORDataString = createSENSORDataString("D1", "Opening", SENSORDataString); //generate datastring
      moveDoor(door2,up,top,door2_speed);
      SENSORDataString = createSENSORDataString("D2", "Opening", SENSORDataString); //generate datastring
      
      door1_moving = 1;             //set flag, door is moving
      door2_moving = 1;             //set flag, door is moving
      door1_poll_time = millis();
      door2_poll_time = door1_poll_time;
    }
  }
  
  //DOOR 1/2 CLOSING
  if(door1_open && !door1_moving && door2_open && !door2_moving && //open and not moving
    ((millis() - door1_time) >= door1_stays_open_min) && //minimum open time passed
      ((!IR_middleL_buffer_sum && !IR_middleR_buffer_sum) || ((millis() - door1_time) >= door1_stays_open_max))) //doors and middle is not blocked OR maximum open time passed: takes precedence above all conditions
  {
    //----- door management
    moveDoor(door1,down,bottom,door1_speed);
    SENSORDataString = createSENSORDataString("D1", "Closing", SENSORDataString); //maximum logging
    moveDoor(door2,down,bottom,door2_speed);
    SENSORDataString = createSENSORDataString("D2", "Closing", SENSORDataString); //maximum logging
    
    door1_moving = 1;             //set flag, door is moving
    door2_moving = 1;             //set flag, door is moving
    door1_poll_time = millis();
    door2_poll_time = door1_poll_time;
  }
  
  //FAILSAFE -------------------------------------------------------------------
  //case: mouse is in middle and both doors are closed
  if(!door1_open && !door1_moving && !door2_open && !door2_moving && ((IR_middleL_buffer_sum >= 8) || (IR_middleR_buffer_sum >= 8)))
  {
    SENSORDataString = createSENSORDataString("FS", "failsafe1", SENSORDataString); //generate datastring
    failsafe_triggered = 1;
  }
  } //end habituation phase 2
  
  //----------------------------------------------------------------------------
  //door management for phase 3 ------------------------------------------------
  //----------------------------------------------------------------------------
  if(habituation_phase == 3)
  {
  
  //----- DOOR 1 MANAGEMENT ----------------------------------------------------
  //DOOR 1 OPENING, IR sensor 1 or The Way Forward
  if(sensor1_triggered || (transition_to_hc == 3))
  {
    if((debug>=1)&&sensor1_triggered){SENSORDataString=createSENSORDataString("IR1","IR1_interrupt",SENSORDataString);}
    
    sensor1_triggered = 0; //clear flag
    
    if(!door1_open && !door1_moving && !door2_open && !door2_moving && (transition_to_tc == 0) && //all closed and not moving and not in transition towards testcage
      (((transition_to_hc == 0) && !IR_middleL_buffer_sum && !IR_middleR_buffer_sum && !tc_occupied) || (transition_to_hc == 3))) //not transitioning to hc and middle must be empty OR already transitioning to hc
    {
      //----- door management
      moveDoor(door1,up,top,door1_speed);
      SENSORDataString = createSENSORDataString("D1", "Opening", SENSORDataString); //generate datastring
      
      door1_poll_time = millis();
      door1_moving = 1; //set flag, door is moving
      
      //----- transition management
      if(transition_to_hc == 0) //only start transition to tc if not on its way back (transition to hc)
      {
        if(debug>=3){SENSORDataString=createSENSORDataString("TM","to_tc1",SENSORDataString);}
        transition_to_tc = 1; //phase 1 origin door opening
      }
      if(transition_to_hc == 3)
      {
        if(debug>=3){SENSORDataString=createSENSORDataString("TM","to_hc4",SENSORDataString);}
        transition_to_hc = 4; //phase 4 target door opening/open
      }
    }
  }
  
  //DOOR 1 CLOSING
  if(door1_open && !door1_moving && ((millis() - door1_time) >= door1_stays_open_min) && //open, not moving and minimum open time passed
      ( ((transition_to_tc == 1) || (!IR_middleL_buffer_sum && !IR_middleR_buffer_sum && (transition_to_hc == 4))) || //starting a transition i.e. has yet to enter middle OR has to leave middle from transition
        ((millis() - door1_time) >= door1_stays_open_max) )) //OR maximum time passed: takes precedence above all conditions
  {
    //----- door management
    moveDoor(door1,down,bottom,door1_speed);
    SENSORDataString = createSENSORDataString("D1", "Closing", SENSORDataString); //maximum logging
    
    door1_poll_time = millis();
    door1_moving = 1;
  }
    
  //----- TRANSITION MANAGEMENT ------------------------------------------------
  //check mice in transition (both doors closed)
  //transitioning to TestCage
  if(transition_to_tc == 2)
  {
    //for x sec. we will read the IR, and afterwards decide if a mouse was present or not.
    if((millis() - transition_time) >= transition_delay)
    {
      //if no mouse is/was present, abort
      if(!IR_middleL_buffer_sum && !IR_middleR_buffer_sum)
      {
        if(debug>=3){SENSORDataString=createSENSORDataString("TM","to_tc0",SENSORDataString);}
        transition_to_tc = 0; //reset
      }
      //if a mouse is(was) present, continue
      if((IR_middleL_buffer_sum > 0) || (IR_middleR_buffer_sum > 0))
      {
        if(debug>=3){SENSORDataString=createSENSORDataString("TM","to_tc3",SENSORDataString);}
        
        //multimice detection
        if((IR_middleL_buffer_sum > 0) && (IR_middleR_buffer_sum > 0) && multimice) //both IR sensors are triggered
        {
          transition_to_tc = 0; //abort transition, hc door opens and waits for mice to exit
        }
        else
        {
          transition_to_tc = 3; //phase 3, open door of target cage
          
          if(mouse_limit)
          {
            if(debug>=3){SENSORDataString=createSENSORDataString("TM","tc_occ=1",SENSORDataString);}
            tc_occupied = 1; //set flag, test cage is no longer empty
          }
        }
      }
    }
  }
  //transitioning to HomeCage (here we don't check for multiple mice, as the way back is always free)
  if(transition_to_hc == 2)
  {
    //for x ms we will wait and read the IR of R2, and afterwards decide if a mouse was present or not.
    if((millis() - transition_time) >= transition_delay)
    {
      //if no mouse is/was present, abort
      if(!IR_middleL_buffer_sum && !IR_middleR_buffer_sum)
      {
        if(debug>=3){SENSORDataString=createSENSORDataString("TM","to_hc0",SENSORDataString);}
        transition_to_hc = 0; //abort transition
      }
      //if a mouse is/was present, continue
      if((IR_middleL_buffer_sum > 0) || (IR_middleR_buffer_sum > 0))
      {
        if(debug>=3){SENSORDataString=createSENSORDataString("TM","to_hc3",SENSORDataString);}
        transition_to_hc = 3; //phase 3, open door towards target cage
        if(mouse_limit)
        {
          if(debug>=3){SENSORDataString=createSENSORDataString("TM","tc_occ=0",SENSORDataString);}
          tc_occupied = 0; //reset flag tc is now empty (mouse returned)
        }
      }
    }
  }
  
  //----- DOOR 2 MANAGEMENT ----------------------------------------------------
  //DOOR 2 OPENING, IR sensor 2 or The Way Back
  if(sensor2_triggered || (transition_to_tc == 3))
  {
    if((debug>=1)&&sensor2_triggered){SENSORDataString=createSENSORDataString("IR2","IR2_interrupt",SENSORDataString);}
    sensor2_triggered = 0; //clear flag
    
    if(!door2_open && !door2_moving && !door1_moving && !door1_open && (transition_to_hc == 0) && //all closed and not moving and not transitioning to homecage
      (((transition_to_tc == 0) && !IR_middleL_buffer_sum && !IR_middleR_buffer_sum) || (transition_to_tc == 3))) //not transitioning to tc and middle must be empty OR already transitioning to tc
    {
      //----- transition management
      if(transition_to_tc == 0) //log/transition only if not already transitioning
      {
        if(debug>=3){SENSORDataString=createSENSORDataString("TM","to_hc1",SENSORDataString);}
        transition_to_hc = 1; //phase 1 origin door opens
      }
      if(transition_to_tc == 3)
      {
        if(debug>=3){SENSORDataString=createSENSORDataString("TM","to_tc4",SENSORDataString);}
        transition_to_tc = 4; //phase 4 target door opens
      }
      //----- door management
      moveDoor(door2,up,top,door2_speed);
      SENSORDataString = createSENSORDataString("D2", "Opening", SENSORDataString); //generate datastring
      
      door2_poll_time = millis();
      door2_moving = 1; //set flag, door is moving
    }
  }
  
  //DOOR 2 CLOSING
  if(door2_open && !door2_moving && ((millis() - door2_time) >= door2_stays_open_min) && //open and not moving and minimum open time passed
      (((transition_to_hc == 1) || (!IR_middleL_buffer_sum && !IR_middleR_buffer_sum && (transition_to_tc == 4))) || //starting a transition i.e. has yet to enter middle OR has to leave middle from transition
        ((millis() - door2_time) >= door2_stays_open_max))) //OR maximum time passed: takes precedence above all conditions
  {
    //----- door management
    moveDoor(door2,down,bottom,door2_speed);
    SENSORDataString = createSENSORDataString("D2", "Closing", SENSORDataString); //maximum logging
    
    //manage failsafe for mouse that doesn't leave middle
    if(((millis() - door2_time) >= door2_stays_open_max) && tc_occupied)
    {
      //sets flag door2 closing after timeout when mouse should transition to tc
      d2_timeout = 1;
      //save time to evaluate failsafe
      d2_timeout_time = millis();
    }
    
    door2_poll_time = millis();
    door2_moving = 1;
  }
  
  //----------------------------------------------------------------------------
  //FAILSAFE 1 opens door towards homecage -------------------------------------
  //cases: mouse manually opens door 1 or 2 and gets into middle
  //if a mouse is detected 8/10, open door towards home cage
  if(!door1_open && !door1_moving && !door2_open && !door2_moving && //all closed and not moving
    (transition_to_hc == 0) && (transition_to_tc == 0) && ((IR_middleL_buffer_sum >= 8) || (IR_middleR_buffer_sum >= 8) )) //not transitioning and IR middle triggered
  {
    //create log entry, failsafe triggered and emulate transition: mouse has to leave towards homecage
    SENSORDataString = createSENSORDataString("FS", "failsafe1", SENSORDataString); //generate datastring
    if(debug>=3){SENSORDataString=createSENSORDataString("TM","to_hc3",SENSORDataString);}
    transition_to_hc = 3;
  }
  
  //if FS1 occured within 3sec. after door 2 has closed, reset tc occupation (mouse never left towards tc, stayed in middle)
  if(mouse_limit)
  {
    if(d2_timeout && ((millis() - d2_timeout_time) <= 3000))
    {
      //reset flag
      d2_timeout = 0;
      //reset tc occupied
      if(debug>=3){SENSORDataString=createSENSORDataString("TM","tc_occ=0",SENSORDataString);}
      tc_occupied = 0;
    }
  }
  
  //FAILSAFE 2 -----------------------------------------------------------------
  //case: even though testcage should be empty, a mouse is detected
  //if a mouse is detected at IR 2, assume testcage not empty
  if(mouse_limit)
  {
    if(!tc_occupied && (transition_to_hc == 1))
    {
      if(debug>=3){SENSORDataString=createSENSORDataString("TM","tc_occ=1",SENSORDataString);}
      tc_occupied = 1; //set tc not empty, needs to be reset by completing transition to hc
      SENSORDataString = createSENSORDataString("FS", "failsafe2", SENSORDataString); //generate datastring
    }
  }
  } //end habituation phase 3
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //do other stuff (every now and then) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  if((millis() - displaytime) > 1000) //~59ms display on, off 0.8us
  {
    displaytime = millis();
    
    //switch display on/off if button pressed
    if(analogRead(buttons) < 900)
    {
      displayon = !displayon;
      if(!displayon)
      {
        oled.clearBuffer();   //clear display
        oled.sendBuffer();    //~33ms
      }
    }
    
    if(displayon)
    {
      oled.clearBuffer();             //clear display
      //create nice date string
      DateTime now = rtc.now();
      uint8_t D = now.day();
      uint8_t M = now.month();
      String nDate = "";

      if(D < 10) nDate += "0";
      nDate += D;
      nDate += "-";
      if(M < 10) nDate += "0";
      nDate += M;
      nDate += "-";
      nDate += now.year();
      
      //display current time from RTC and date
      OLEDprint(0,0,0,0,nicetime());
      OLEDprint(0,11,0,0,nDate);
      
      //display info on door status and TC status
      OLEDprint(1,0,0,0,"D1:");
      if(door1_open && !door1_moving) OLEDprint(1,3,0,0,"Open");
      if(door1_open && door1_moving) OLEDprint(1,3,0,0,"Closing");
      if(!door1_open && !door1_moving) OLEDprint(1,3,0,0,"Closed");
      if(!door1_open && door1_moving) OLEDprint(1,3,0,0,"Opening");
      
      OLEDprint(1,10,0,0,"D2:");
      if(door2_open && !door2_moving) OLEDprint(1,13,0,0,"Open");
      if(door2_open && door2_moving) OLEDprint(1,13,0,0,"Closing");
      if(!door2_open && !door2_moving) OLEDprint(1,13,0,0,"Closed");
      if(!door2_open && door2_moving) OLEDprint(1,13,0,0,"Opening");
      
      OLEDprint(2,0,0,0,"TC occupied:");
      if(mouse_limit)
      {
        if(tc_occupied) OLEDprint(2,13,0,0,"Yes");
        else OLEDprint(2,13,0,0,"No");
      }
      else OLEDprint(2,13,0,0,"N/A");
      
      //Print a letter for each mouse and activity histogram for last 24h
      String letters = "ABCDEFGHIJKL";
      for(uint8_t i = 0;i < 12;i++)
      {
        oled.setCursor(i*10,63);
        oled.print(letters[i]);
      }
      
      //clip values
      for(uint8_t i = 0;i < 12;i++)
      {
        if(mice_visits[i+1][1] > 100) mice_visits[i+1][1] = 100;
        if(mice_visits[i+1][2] > 100) mice_visits[i+1][2] = 100;
      }
      
      for(uint8_t i = 0;i < 12;i++)
      {
        oled.drawLine(i*10,52,i*10,52-(mice_visits[i+1][1]/10));     //x y x y Reader 1
        oled.drawLine(i*10+1,52,i*10+1,52-(mice_visits[i+1][1]/10)); //x y x y 2 pixel wide
        
        oled.drawLine(i*10+3,52,i*10+3,52-(mice_visits[i+1][2]/10)); //x y x y Reader 2
        oled.drawLine(i*10+4,52,i*10+4,52-(mice_visits[i+1][2]/10)); //x y x y 2 pixel wide
      }
      //update display
      oled.sendBuffer();
    }
  }
  
  //do stuff every 10 minutes (that needs rtc) ----------------------------------
  if((millis() - rtccheck_time) > 600000)
  {
    rtccheck_time = millis();
    DateTime now = rtc.now();
    uint32_t rtc_now = now.unixtime();
  
    //check if a mouse didn't visit the testcage -------------------------------
    for(uint8_t i = 1; i < mice; i++)
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
	//write Data to log files ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
  DateTime now = rtc.now();
  
	String ntime = "";
  uint8_t h = now.hour() + GMT;
  uint8_t m = now.minute();
  uint8_t s = now.second();

	if (h < 10) ntime += "0";
	ntime += h;
	ntime += ":";
	if (m < 10) ntime += "0";
	ntime += m;
	ntime += ":";
	if (s < 10) ntime += "0";
	ntime += s;
	return ntime;
}

//Sensors related functions
String createSENSORDataString(String identifier, String event, String dataString)
{
  DateTime now = rtc.now();
  //if datastring is not empty, add newline (for pretty formatting)
  if(dataString != 0)
  {
    dataString += "\n";
  }
  
  dataString += identifier;
  dataString += ",";
  dataString += now.unixtime();
  dataString += ",";
  dataString += ""; //here would be the tag
  dataString += ",";
  dataString += millis();
  dataString += ",";
  dataString += event; //T for triggered (can be enter/exit as well --> ISR CHANGE)

  return dataString;
}

//Helper for printing to OLED Display ------------------------------------------
void OLEDprint(uint8_t row, uint8_t column, uint8_t clear, uint8_t update, String text)
{
  if(clear)
  {
    oled.clearBuffer();   //clear screen
  }
  
  oled.setCursor(column * 6,(row * 10) + 10); //max row 0-5, max col 0-20
  oled.print(text);
  
  if(update)
  {
    oled.sendBuffer();
  }
}

void OLEDprint(uint8_t row, uint8_t column, uint8_t clear, uint8_t update, int32_t number)
{
  if(clear)
  {
    oled.clearBuffer();   //clear screen
  }
  
  oled.setCursor(column * 6,(row * 10) + 10); //max row 0-5, max col 0-20
  oled.print(number);
  
  if(update)
  {
    oled.sendBuffer();
  }
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

//RFID tag related functions ---------------------------------------------------
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
  DateTime now = rtc.now();
  
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
    //dataString += rtc.getEpoch();
    dataString += now.unixtime();
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
    //dataString += rtc.getEpoch();
    dataString += now.unixtime();
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

//Doors ------------------------------------------------------------------------
uint8_t getDoorStatus(uint8_t address)
{
  //request tag-data from reader
  uint8_t doorStatus[2];
  Wire.requestFrom(address,2,1); //address, quantity ~574uS 6 bytes, bus release
  uint8_t n = 0;
  while(Wire.available())
  {
    doorStatus[n] = Wire.read();
    n++;
  }
  
  //0 IR_top, IR_upper, IR_middle, IR_lower, IR_bottom, IR_barrier_rx, 6 IR_barrier_tx
  door1_IR_state[0] = (doorStatus[0] >> 0) & 0x01;
  door1_IR_state[1] = (doorStatus[0] >> 1) & 0x01;
  door1_IR_state[2] = (doorStatus[0] >> 2) & 0x01;
  door1_IR_state[3] = (doorStatus[0] >> 3) & 0x01;
  door1_IR_state[4] = (doorStatus[0] >> 4) & 0x01;
  door1_IR_state[5] = (doorStatus[0] >> 5) & 0x01;
  door1_IR_state[6] = (doorStatus[0] >> 6) & 0x01;
//  door1_IR_state[7] = (doorStatus[0] >> 7) & 0x01; //unused at the moment
  
  return doorStatus[1] & 0x01; //busy flag
}

//------------------------------------------------------------------------------
void calibrateDoor(uint8_t address)
{
  uint8_t sendbuffer[7] = {0,0,0,0,0,0,0};
  sendbuffer[0] = 1;
  
  int send_status = 1;
  while(send_status != 0)
  {
    Wire.beginTransmission(address); //address
    for(uint8_t i = 0; i < 7; i++)
    {
      Wire.write(sendbuffer[i]);
    }
    send_status = Wire.endTransmission();
  }
}

//------------------------------------------------------------------------------
//move command is sent to door, which can be queried if it is finished.
void moveDoor(uint8_t address, uint8_t direction, uint8_t target, uint16_t pulsetime)
{
  uint8_t sendbuffer[7] = {0,0,0,0,0,0,0};
  sendbuffer[0] = 2;
  
  sendbuffer[1] = direction;
  sendbuffer[2] = target;
  
  sendbuffer[3] = pulsetime & 0xff;
  sendbuffer[4] = (pulsetime >> 8) & 0xff;
  
  uint8_t send_status = 1;
  while(send_status != 0)
  {
    Wire.beginTransmission(address); //address
    for(uint8_t i = 0; i < 7; i++)
    {
      Wire.write(sendbuffer[i]);
    }
    send_status = Wire.endTransmission();
  }
}

//------------------------------------------------------------------------------
void closeDoorFB(uint8_t address, uint8_t start, uint8_t stop)
{
  uint8_t sendbuffer[7] = {0,0,0,0,0,0,0};
  
  //option sendbuffer[0]
  //0 = setup, 1 = calibrate, 2 = move simple, 3 = movefancy
  sendbuffer[0] = 3;
  
  //IR 0 = top, 1 = upper, 2 = middle, 3 = lower, 4 = bottom
  //start sendbuffer[1]
  sendbuffer[1] = start;
  //stop sendbuffer[2]
  sendbuffer[2] = stop;
  
  int send_status = 1;
  while(send_status != 0)
  {
    Wire.beginTransmission(address); //address
    for(uint8_t i = 0; i < 7; i++)
    {
      Wire.write(sendbuffer[i]);
    }
    send_status = Wire.endTransmission();
  }
}

//------------------------------------------------------------------------------
//returns duration until rotation is finished in milliseconds
//not in use
// uint16_t rotate(uint8_t address, uint8_t stepper, uint32_t steps, uint8_t direction, uint8_t speed)
// {
//   uint8_t sendbuffer[7] = {0,0,0,0,0,0,0};
//
//   //steps: 3200 steps == 1 revolution
//   //create 4 bytes from 32bit variable
//   sendbuffer[0] = steps & 0xff;         //first 8 bits
//   sendbuffer[1] = (steps >>  8) & 0xff; //next 8 bits
//   sendbuffer[2] = (steps >> 16) & 0xff; //next 8 bits
//   sendbuffer[3] = (steps >> 24) & 0xff; //last 8 bits
//
//   //direction: 0/1
//   //speed: 0-255 (<= 230 recommended)
//   sendbuffer[4] = direction;
//   sendbuffer[5] = 255 - speed;
//   sendbuffer[6] = stepper;
//
//   int send_status = 1;
//   while(send_status != 0)
//   {
//     Wire.beginTransmission(address); //address
//     for(uint8_t i = 0; i < 7; i++)
//     {
//       Wire.write(sendbuffer[i]);
//     }
//     send_status = Wire.endTransmission();
//   }
//
//   //(pin_toggle_time) + (delay_between_steps) * 2 * steps / convert_to_ms + rounding
//   return (((0.4 + (255-speed)) * 2 * steps) / 1000) + 0.5;
// }

//critical error, flash LED SOS ------------------------------------------------
void criticalerror()
{
  while(1)
  {
    strip.setPixelColor(0,255,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,0,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,255,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,0,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,255,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,0,0,0);
    strip.show();
    delay(100);
    
    strip.setPixelColor(0,255,0,0);
    strip.show();
    delay(300);
    strip.setPixelColor(0,0,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,255,0,0);
    strip.show();
    delay(300);
    strip.setPixelColor(0,0,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,255,0,0);
    strip.show();
    delay(300);
    strip.setPixelColor(0,0,0,0);
    strip.show();
    delay(100);
    
    strip.setPixelColor(0,255,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,0,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,255,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,0,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,255,0,0);
    strip.show();
    delay(100);
    strip.setPixelColor(0,0,0,0);
    strip.show();
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
