/*------------------------------------------------------------------------------
- PJRC Teensy 4.1 (with ethernet) pin mapping - Hardware Revision v7.1

- dual-infrared lightbarrier connectors (S|S|GND|+12V)
D36,D37 - X1 infrared barrier 1
A14,A15 - X2 infrared barrier 2
A11,A10 - X3 infrared barrier 3
A16,A17 - X4 infrared barrier 4
A1,A0   - X5
A6,A7   - X6
A8,A9   - X7
D3,D2   - X8

- multi-purpose 3-pin connectors (Signal|GND|+12V)
D9  - J4
D8  - J5
D7  - J6
D6  - J7

- fan-connectors 2-pin, 12V transistor-switched (GND|+12V)
D29 - F1 Fan 1
D28 - F2 Fan 2

D32 - ERR LED   Error LED, used for various error states
D31 - STAT LED  Status LED, can be used to signal stuff
A13 - B1,B2,B3  Input from the three buttons on the board

D11 - MOSI
D12 - MISO
D13 - SCK
D18 - SDA
D19 - SCL

--- Experimental Setup ---

^^^^^^^\                                                         /^^^^^^^^
       |                  H                   T                  |
h  c   |     |R|    |I| | C |   |I|   |I|   | C | |I|    |R|     |    t  c
o  a ––|–––––|F|––––|R|–| D |–––|R|---|R|---| D |–|R|––––|F|–––––|––  e  a
m  g   |     |I|    | | | O |   | |   | |   | O | | |    |I|     |    s  g
e  e ––|–––––|D|––––| |-| O |–––| |---| |---| O |–| |––––|D|–––––|––  t  e
       |     |1|    |1| | R |   |3|   |4|   | R | |2|    |2|     |
       |                                                         |
_______/                  |-----   X cm  -----|                  \________

*///----------------------------------------------------------------------------
#include <TimeLib.h>  //Manage Real Time CLock
#include <Wire.h>     //I2C communication
#include <SdFat.h>    //Access SD Cards
#include <U8g2lib.h>  //for SSD1306 OLED Display

void callback()
{
    SNVS_HPSR |= 0b11;              // reset interrupt flag
    digitalToggleFast(31); // toggle a pin to generate the pps signal
    asm("dsb");                     // wait until flag is synced over the busses to prevent double calls of the isr
}

void setupRTCInterrupt()
{
    // disable periodic interrupt
    SNVS_HPCR &= ~SNVS_HPCR_PI_EN;
    while ((SNVS_HPCR & SNVS_HPCR_PI_EN)) {} //The RTC is a slow peripheral, need to spin until PI_EN is reset...

    // set interrupt frequency to 2Hz / 0.5s
    SNVS_HPCR = SNVS_HPCR_PI_FREQ(0b1110);  // change to 0b1111 if you need a 1s interrupt

    // enable periodic interrupt
    SNVS_HPCR |= SNVS_HPCR_PI_EN;
    while (!(SNVS_HPCR & SNVS_HPCR_PI_EN)) {} // spin until PI_EN is set...

    // attach a callback
    attachInterruptVector(IRQ_SNVS_IRQ, callback);
    NVIC_SET_PRIORITY(IRQ_SNVS_IRQ, 16); // some priority 0-255 (low number = high priority)
    NVIC_ENABLE_IRQ(IRQ_SNVS_IRQ);
}



//----- declaring variables ----------------------------------------------------
//Current Version of the program
const char SOFTWARE_REV[] = "v1.0.0";

//I2C addresses
const uint8_t reader1 = 0x08;     //I2C address RFID module 1
const uint8_t reader2 = 0x09;     //I2C address RFID module 2
const uint8_t doorMod1 = 0x10;
//const uint8_t turntable1 = 0x11;  //for future use
const uint8_t oledDisplay = 0x78; //I2C address oled display

//Buttons
const int buttons = A13;  //~1022 not pressed, ~1 left, ~323 middle, ~711 right

//Fans
const int fan1 = 29;        //fan1
const int fan2 = 28;        //fan2
uint8_t fan1on = 0;         //fan1 state
uint8_t fan2on = 0;         //fan2 state
uint8_t cleardoorblock = 0; //to separate between different conditions that require a fan

//IR Sensors
const int IR1[2] = {36,37};
const int IR2[2] = {A14,A15};
const int IR3[2] = {A11,A10};
const int IR4[2] = {A16,A17};

//use volatile if interrupt based
uint8_t IR1_trigd[2] = {0,0};
uint8_t IR2_trigd[2] = {0,0};
uint8_t IR3_trigd[2] = {0,0};
uint8_t IR4_trigd[2] = {0,0};

const uint8_t buffer_reads = 50;
uint8_t IR1_buffer[2][buffer_reads] = {};
uint8_t IR2_buffer[2][buffer_reads] = {};
uint8_t IR3_buffer[2][buffer_reads] = {};
uint8_t IR4_buffer[2][buffer_reads] = {};

//coincidence buffer sum, only counts if both IR barriers are triggered simultaniously
uint8_t IR1_cbuffer_sum = 0;
uint8_t IR2_cbuffer_sum = 0;
uint8_t IR3_cbuffer_sum = 0;
uint8_t IR4_cbuffer_sum = 0;
uint8_t IR34_cbuffer_sum = 0;
uint8_t IR_middle_csum = 0; //contains the sum of the coincidence sums of IR3 IR4
uint8_t sb = 0;             //sensor buffer counter
uint32_t IRsensor_time;     //time when IR sensors 1,2,3,4 were last checked

//LEDs
const int errorLED = 32;
const int statusLED = 31;

//SD cardsModules
#define SD_FAT_TYPE 3
#define SPI_CLOCK SD_SCK_MHZ(16)
const uint8_t SDcs = 10;    //Chip Select External SD
SdFs SD;
FsFile dataFile;
const uint8_t SDBcs = 44;   //Chip Select Internal SD
SdFs SDb;
FsFile dataFileBackup;

//Display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0,U8X8_PIN_NONE,23,22); //def,reset,SCL,SDA
uint32_t displaytime = 0;         //stores millis to time display refresh
uint8_t displayon = 1;            //flag to en/disable display

//Stepper Modules
uint8_t door_moving[2];           //moving
uint8_t door_open[2];             //open or close
uint32_t door_poll_time[2];       //time door module was last polled for status
uint16_t door_speed[2];           //speed in us delay between steps
uint16_t door_stays_open_min;     //minimum time a door will stay open for mouse to exit towards HC/TC
uint32_t door_stop_time[2];       //time when a change in the tm state last happened. Used to track delays that should happen between different stages
uint32_t door_move_time[2];       //time the door has started moving
uint8_t tm_state;                 //transition management state
uint8_t tm_state_restart = 0x1A;  //transition management restart state after failsafe
uint8_t doublemouseflag;          //flag if a mouse is detected in the testcage, when it should be empty
//uint8_t turntable_moving;  //placeholder for turntable

//human readable door names
const uint8_t HCdoor = 0;
const uint8_t TCdoor = 1;
//human readable names for door control
const uint8_t top = 0;
const uint8_t bottom = 1;
const uint8_t up = 0;
const uint8_t down = 1;
//busy if door is still moving and the state of all attached IR barriers
uint8_t door1_state[7];        //busy, tx1, tx2, rx1, rx2, top, bottom
uint8_t door2_state[7];        //busy, tx1, tx2, rx1, rx2, top, bottom

//RFID
uint32_t RFIDtime;           //used to measure time before switching to next antenna
uint8_t RFIDtoggle = 0;      //flag used to switch to next antenna

uint8_t tag[6] = {};         //global variable to store returned tag data (limitation of C to return arrays)
uint8_t tag1_present = 0;    //flag that indicates if tag was present during read cycle
uint8_t tag2_present = 0;
uint8_t reader1_cycle = 0;   //toggles flag if a read cycle has just happened (not automatically cleared)
uint8_t reader2_cycle = 0;
uint8_t currenttag1[6] = {}; //saves id of the tag that was read during the current read cycle
uint8_t currenttag2[6] = {};
uint8_t lasttag1[6] = {};    //saves id of the tag that was read during the previous read cycle
uint8_t lasttag2[6] = {};

uint8_t RFIDmode = 1;           //select mode to operate in: 1-alternate, 2-reader1, 3-reader2
uint8_t RFIDmode_firstrun = 1;  //to make sure the correct reader is turned on/off

//Experiment variables
uint8_t tc_occupied = 0;   //flag that tracks occupation of test cage
uint32_t starttime;        //start of programm
uint32_t rtccheck_time;    //time the rtc was checked last

//Mice tags
const uint8_t mice = 15;           //number of mice in experiment (add 1 for mouse 0, add 2 for test-mice)
const uint8_t mouse_library[mice][6] = {
  {0x00,0x00,0x00,0x00,0x00,0x00}, //mouse 0
  {0x73,0x74,0xF7,0x90,0x2E,0xE1}, //mouse 1  sw_si 1923
  {0x7F,0x65,0x7F,0x90,0x2E,0xE1}, //mouse 2  ro_ge 8095
  {0x40,0x73,0x7F,0x90,0x2E,0xE1}, //mouse 3  sw_ro 1616
  {0x32,0x74,0x7F,0x90,0x2E,0xE1}, //mouse 4  we_sw 1858
  {0xB8,0x74,0x7F,0x90,0x2E,0xE1}, //mouse 5  ro_we 1992
  {0x18,0x6E,0x7F,0x90,0x2E,0xE1}, //mouse 6  sw_ge 0296
  {0xAA,0x71,0x7F,0x90,0x2E,0xE1}, //mouse 7  we_si 1210
  {0x6B,0x6E,0x7F,0x90,0x2E,0xE1}, //mouse 8  ro_si 0379
  {0x0F,0x71,0x7F,0x90,0x2E,0xE1}, //mouse 9  sw_we 1055
  {0x77,0x6F,0x7F,0x90,0x2E,0xE1}, //mouse 10 we_ge 0647
  {0x91,0x64,0x7F,0x90,0x2E,0xE1}, //mouse 11 ro_sw 7857
  {0x41,0x73,0x7F,0x90,0x2E,0xE1}, //mouse 12 we_ro 1617
  {0xA1,0x82,0x42,0xDD,0x3E,0xF3}, //mouse 13 polymorphmaus
  {0x0E,0x67,0xF7,0x90,0x2E,0xE1}};//mouse 14 bleistiftmaus

uint8_t mice_visits[mice][2];      //contains the number of tag reads at reader 1 and 2 during the last 24 hours
uint8_t current_mouse1 = 0;        //placeholder simple number for tag at reader 1
uint8_t current_mouse2 = 0;        //placeholder simple number for tag at reader 2

//##############################################################################
//#####   U S E R   C O N F I G  ###############################################
//##############################################################################

//if set to 1, the MoPSS prints what is written to uSD to Serial as well.
const uint8_t is_testing = 1;

//write additional information to SD card
//debug = 0 includes door movements, RFID tags
//debug = 1 include regular status of the IR barriers
const uint8_t debug = 1;

//Habituation phase
//1: Both doors always open
//3: Transition management enabled, transition delay option
uint8_t habituation_phase = 3;

//time mouse is kept in transition (inside gate) with both doors closed (ms)
uint16_t transition_delay = 3000; //ms

//time until fan1 is turned on
const uint16_t fan1delay = 30000; //ms door 1 open and until fan1 is turned on

int ms;

//##############################################################################
//#####   S E T U P   ##########################################################
//##############################################################################
void setup(){
  //----- USER CONFIG (CONTINUED) ----------------------------------------------
  door_speed[HCdoor] = 125;     //speed is the us delay between steps. 0 is fastest possible, and slower the higher the value (250 starting value)
  door_speed[TCdoor] = 125;
  door_stays_open_min = 8000;   //minimum time a door stays open

  //----- Extra Variable setup -------------------------------------------------
  door_open[HCdoor] = 0;
  door_open[TCdoor] = 0;

  pinMode(31,OUTPUT);

  setupRTCInterrupt();

  //----- communication --------------------------------------------------------
  //start Serial communication
  Serial.begin(115200);
  if(is_testing == 1){
    //while(!Serial); //wait for serial connection
    Serial.println("alive");
  }
  
  //start I2C
  Wire.begin();

  //----- Buttons & Fans -------------------------------------------------------
  pinMode(buttons,INPUT);
  pinMode(fan1,OUTPUT);
  pinMode(fan2,OUTPUT);

  //----- Sensors --------------------------------------------------------------
  pinMode(IR1[0],INPUT);
  pinMode(IR1[1],INPUT);
  pinMode(IR2[0],INPUT);
  pinMode(IR2[1],INPUT);
  pinMode(IR3[0],INPUT);
  pinMode(IR3[1],INPUT);
  pinMode(IR4[0],INPUT);
  pinMode(IR4[1],INPUT);

  //----- Real Time Clock ------------------------------------------------------
  setSyncProvider(getTeensy3Time);

  //----- Display --------------------------------------------------------------
  oled.setI2CAddress(oledDisplay);
  oled.begin();
  oled.setFont(u8g2_font_6x10_mf); //set font w5 h10

  //----- Setup RFID readers ---------------------------------------------------
  //measure resonant frequency and confirm/repeat on detune
  uint8_t RFIDmodulestate = 0;
  int32_t reader1_freq = 0;
  int32_t reader2_freq = 0;
  
  //----- Setup SD Card --------------------------------------------------------
  //Stop program if uSDs are not detected/faulty (needs to be FAT/FAT32/exFAT format)
  OLEDprint(0,0,1,1,">>>  uSD  Setup  <<<");
  OLEDprint(1,0,0,1,"SD EXternal:"); //see if the cards are present and can be initialized
  OLEDprint(2,0,0,1,"SD INternal:");
  
  //SD card external (main, for data collection)
  if(!SD.begin(SDcs)){
    OLEDprint(1,13,0,1,"FAIL!");
    OLEDprint(5,0,0,1,"PROGRAM STOPPED");
    criticalerror();
  }
  else{
    Serial.println("External SD card initialized successfully!");
    OLEDprint(1,13,0,1,"OK!");
  }
  //SD card internal (Backup)
  if(!SDb.begin(SdioConfig(FIFO_SDIO))){ //internal SD Card
    OLEDprint(2,13,0,1,"FAIL!");
    OLEDprint(5,0,0,1,"PROGRAM STOPPED");
    criticalerror();
  }
  else{
    Serial.println("Internal SD card initialized successfully!");
    OLEDprint(2,13,0,1,"OK!");
  }
  delay(1000);

  //----- Setup log file, and write initial configuration ----------------------
  dataFile = SD.open("RFIDLOG.TXT", FILE_WRITE); //open file, or create if empty
  dataFileBackup = SDb.open("RFIDLOG_BACKUP.TXT", FILE_WRITE);
  
  starttime = now(); //get experiment start time

  //write current version to SD and some other startup/system related information
  dataFile.println("");
  dataFile.print("# Modular MoPSS Hive version: ");
  dataFile.println(SOFTWARE_REV);
  dataFile.print("# RFID Module 1 resonant frequency: ");
  dataFile.print(reader1_freq);
  dataFile.println(" Hz");
  dataFile.print("# RFID Module 2 resonant frequency: ");
  dataFile.print(reader2_freq);
  dataFile.println(" Hz");
  dataFile.print("# Habituation phase: ");
  dataFile.println(habituation_phase);
  dataFile.print("# debug level: ");
  dataFile.println(debug);
  dataFile.print("# System start @ ");
  dataFile.print(nicetime(starttime));
  dataFile.print(" ");
  dataFile.print(day(starttime));
  dataFile.print("-");
  dataFile.print(month(starttime));
  dataFile.print("-");
  dataFile.println(year(starttime));
  dataFile.print("# Unixtime: ");
  dataFile.println(starttime);
  dataFile.println();

  dataFile.flush();

  //and same for backup SD
  dataFileBackup.println("");
  dataFileBackup.print("# Modular MoPSS Hive version: ");
  dataFileBackup.println(SOFTWARE_REV);
  dataFileBackup.print("# RFID Module 1 resonant frequency: ");
  dataFileBackup.print(reader1_freq);
  dataFileBackup.println(" Hz");
  dataFileBackup.print("# RFID Module 2 resonant frequency: ");
  dataFileBackup.print(reader2_freq);
  dataFileBackup.println(" Hz");
  dataFileBackup.print("# Habituation phase: ");
  dataFileBackup.println(habituation_phase);
  dataFileBackup.print("# debug level: ");
  dataFileBackup.println(debug);
  dataFileBackup.print("# System start @ ");
  dataFileBackup.print(nicetime(starttime));
  dataFileBackup.print(" ");
  dataFileBackup.print(day(starttime));
  dataFileBackup.print("-");
  dataFileBackup.print(month(starttime));
  dataFileBackup.print("-");
  dataFileBackup.println(year(starttime));
  dataFileBackup.print("# Unixtime: ");
  dataFileBackup.println(starttime);
  dataFileBackup.println();

  dataFileBackup.flush();
  
} //end of setup

//##############################################################################
//#####   L O O P   ############################################################
//##############################################################################
void loop(){

  //create/clear strings that get written to uSD card
  String RFIDdataString = ""; //holds tag and date
  String SENSORDataString = ""; //holds various sensor and diagnostics data
  
  //----------------------------------------------------------------------------
  //update display -------------------------------------------------------------
  //----------------------------------------------------------------------------
  if((millis() - displaytime) > 1000){ //once every second
    displaytime = millis();

    //switch display on/off if button pressed
    if(analogRead(buttons) < 850){
      displayon = !displayon;
      if(!displayon){
        oled.clearBuffer();   //clear display
        oled.sendBuffer();
      }
    }

    if(displayon){
      oled.clearBuffer(); //clear display
      time_t rtctime = now(); //create nice date string
      uint8_t D = day(rtctime);
      uint8_t M = month(rtctime);
      String nDate = "";

      if(D < 10) nDate += "0";
      nDate += D;
      nDate += "-";
      if(M < 10) nDate += "0";
      nDate += M;
      nDate += "-";
      nDate += year(rtctime);

      //display current time from RTC and date
      OLEDprint(0,0,0,0,nicetime(rtctime));
      OLEDprint(0,11,0,0,nDate);

      //update display
      oled.sendBuffer();
    }
  }

  //----------------------------------------------------------------------------
	//write Data to log files ----------------------------------------------------
	//----------------------------------------------------------------------------
	//log sensor/motor events
  if(SENSORDataString.length() != 0){
    dataFile.println(SENSORDataString); //append Datastring to file
    dataFile.flush();
    dataFileBackup.println(SENSORDataString);
    dataFileBackup.flush();
    if(is_testing == 1) Serial.println(SENSORDataString); //print to serial if in testmode
  }
  //log RFID events
  if(RFIDdataString.length() != 0){
		dataFile.println(RFIDdataString);	//append Datastring to file
		dataFile.flush();
		dataFileBackup.println(RFIDdataString);
		dataFileBackup.flush();
		if(is_testing == 1) Serial.println(RFIDdataString);
	}

} //end of loop

//##############################################################################
//#####   F U N C T I O N S   ##################################################
//##############################################################################

//Get Time from internal RTC (updated on program upload) -----------------------
time_t getTeensy3Time(){
  return Teensy3Clock.get();
}

//Return time as string in HH:MM:SS format -------------------------------------
String nicetime(time_t nowtime){
	String ntime = "";
  uint8_t h = hour(nowtime);// + GMT;
  uint8_t m = minute(nowtime);
  uint8_t s = second(nowtime);

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

//Sensors related functions ----------------------------------------------------
String createSENSORDataString(String identifier, String event, String dataString){
  time_t nowtime = now();

  if(dataString != 0) dataString += "\n"; //if datastring is not empty, add newline
  dataString += identifier;
  dataString += ",";
  dataString += nowtime;
  dataString += ",";
  dataString += "";
  dataString += ",";
  dataString += millis();
  dataString += ",";
  dataString += event;

  return dataString;
}

//Helper for printing to OLED Display (text) -----------------------------------
void OLEDprint(uint8_t row, uint8_t column, uint8_t clear, uint8_t update, String text){
  if(clear) oled.clearBuffer(); //clear screen 
  oled.setCursor(column * 6,(row * 10) + 10); //max row 0-5, max col 0-20
  oled.print(text);
  if(update) oled.sendBuffer();
}

//Helper for printing to OLED Display (number) ---------------------------------
void OLEDprint(uint8_t row, uint8_t column, uint8_t clear, uint8_t update, int32_t number){
  if(clear) oled.clearBuffer(); //clear screen  
  oled.setCursor(column * 6,(row * 10) + 10); //max row 0-5, max col 0-20
  oled.print(number);
  if(update) oled.sendBuffer();
}

//Helper for printing to OLED Display (number with n decimals) -----------------
void OLEDprintFraction(uint8_t row, uint8_t column, uint8_t clear, uint8_t update, float number, uint8_t decimals){
  oled.setCursor(column * 6,(row * 10) + 10); //max row 0-5, max col 0-20
  oled.print(number,decimals);
  if(update) oled.sendBuffer();
}

//get RFID ID in string format -------------------------------------------------
String getID(uint8_t in[6]){
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
  while(in64){
    char c = in64 % 10;
    in64 /= 10;
    c += '0'; //add to character zero
    result = c + result; //concatenate
  }
  return result;
}

//convert byte array to char (RFID countrycode) --------------------------------
uint16_t getCountryCode(uint8_t in[6]){
  uint16_t countrycode = 0;
  countrycode = ((countrycode | in[5]) << 2) | ((in[4] >> 6) & 0b11);
  return countrycode;
}

//enable one reader, wait for confirmation from reader -------------------------
void enableReader(uint8_t reader){
  uint8_t send_status = 1;
  while(send_status != 0){
    Wire.beginTransmission(reader);
    Wire.write(1); //enable reader
    send_status = Wire.endTransmission();
  }
}

//disable one reader, wait for confirmation from reader ------------------------
void disableReader(uint8_t reader){
  uint8_t send_status = 1;
  while(send_status != 0){
    Wire.beginTransmission(reader);
    Wire.write(0); //disable reader
    send_status = Wire.endTransmission();
  }
}

//switch between two readers, optimized timing for minimum downtime ------------
void switchReaders(byte readerON, byte readerOFF){
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

//sets mode of the RFID reader 2 = RFID mode (retun tags), 3 = measure mode (return resonant frequency)
void setReaderMode(uint8_t reader,uint8_t mode){
  uint8_t send_status = 1;
  while(send_status != 0){
    Wire.beginTransmission(reader);
    Wire.write(mode);
    send_status = Wire.endTransmission();
  }
}

//Query reader for additional information --------------------------------------
uint32_t fetchResFreq(uint8_t reader){
  setReaderMode(reader,3); //set to frequency measure mode and perform measurement
  delay(1200);             //frequency measurement takes about >=1.1 seconds

  //fetch measured frequency
  uint32_t resfreq = 0;
  uint8_t rcv[4];
  Wire.requestFrom(reader,4,1); //request frequency
  uint8_t n = 0;
  while(Wire.available()){
    rcv[n] = Wire.read();
    n++;
  }
  //assemble from array to 32bit variable
  resfreq |= (rcv[3] << 24);
  resfreq |= (rcv[2] << 16);
  resfreq |= (rcv[1] <<  8);
  resfreq |= (rcv[0] <<  0);

  //leave measure mode
  setReaderMode(reader,2);  //set to tag-transmitting mode

  return resfreq;
}

//fetch tag data from reader ---------------------------------------------------
uint8_t fetchtag(byte reader, byte busrelease){
  Wire.requestFrom(reader,6,busrelease); //address, quantity ~574uS, bus release
  uint8_t n = 0;
  while(Wire.available()){
    tag[n] = Wire.read();
    n++;
  }
  //sum received values
  int16_t tag_sum = 0;
  for(uint8_t i = 0; i < sizeof(tag); i++){
    tag_sum = tag_sum + tag[i];
  }
  //if tag is empty, no tag was detected
  if(tag_sum > 0) return 1;
  else return 0;
}

//compare current and last tag, no change 0, new tag entered 1, switch 2 (2 present), tag left 3
uint8_t compareTags(byte currenttag[], byte lasttag[]){
  uint8_t tagchange = 0; //0 = no change, 1 = new tag entered, 2 = switch (2 present), 3 = tag left
  int16_t lasttag_sum = 0;
  int16_t currenttag_sum = 0;

  for(uint8_t i = 0; i < sizeof(lasttag); i++){
    if(currenttag[i] != lasttag[i]){ //if diff between current and last tag, something changed
      for(uint8_t j = 0; j < sizeof(lasttag); j++){  //check if arrays are empty by summing all values
        lasttag_sum = lasttag_sum + lasttag[j];
        currenttag_sum = currenttag_sum + currenttag[j];
      }
      if(lasttag_sum == 0) tagchange = 1;                            //if lasttag is empty but not currenttag: 1 = new tag entered
      if((lasttag_sum != 0) && (currenttag_sum != 0)) tagchange = 2; //if lasttag wasn't empty and currenttag isn't either, tags switched (two present, one left)
      if(currenttag_sum == 0) tagchange = 3;                         //if currenttag is empty, but not last tag, 3 = tag left
      break;
    }
  }
  return(tagchange); //return how (if) the tag changed
}

//create string that is later saved to uSD -------------------------------------
String createRFIDDataString(byte currenttag[], byte lasttag[], byte currenttag_present, int tagchange, String identifier){
  String dataString;
  time_t nowtime = now();
  
  //get country code and tag ID for currenttag (ct) and lasttag (lt)
  int16_t ctCC = getCountryCode(currenttag);
  int16_t ltCC = getCountryCode(lasttag);
  String ctID = getID(currenttag);
  String ltID = getID(lasttag);

  //save tag data to dataString which is written to SD
  if((tagchange == 2) || (tagchange == 3)){ //tag left (3) or switch (2)
    dataString += identifier;
    dataString += ",";
    dataString += nowtime;
    dataString += ",";
    dataString += ltCC;
    dataString += "_";
    dataString += ltID;
    dataString += ",";
    dataString += millis();
    dataString += ",X";
  }
  //insert newline when a switch happens
  if(tagchange == 2){
    dataString += "\n";
  }
  //new tag entered (1) or switch (2)
  if((tagchange == 1) || (tagchange == 2)){
    dataString += identifier;
    dataString += ",";
    dataString += nowtime;
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

//get status of door module, busy and IR barrier status ------------------------
uint8_t getDoorModuleStatus(uint8_t address){
  uint8_t rcv[2];
  Wire.requestFrom(address,2,1); //address, quantity ~574uS 6 bytes, bus release
  uint8_t n = 0;
  while(Wire.available()){
    rcv[n] = Wire.read();
    n++;
  }

  door1_state[0] = (rcv[0] >> 0) & 0x01;  //busy flag
  door1_state[1] = (rcv[0] >> 1) & 0x01;  //tx1
  door1_state[2] = (rcv[0] >> 2) & 0x01;  //tx2
  door1_state[3] = (rcv[0] >> 3) & 0x01;  //rx1
  door1_state[4] = (rcv[0] >> 4) & 0x01;  //rx2
  door1_state[5] = (rcv[0] >> 5) & 0x01;  //top
  door1_state[6] = (rcv[0] >> 6) & 0x01;  //bottom
//  door1_state[7] = (rcv[0] >> 7) & 0x01; //unused at the moment

  door2_state[0] = (rcv[1] >> 0) & 0x01;  //busy flag
  door2_state[1] = (rcv[1] >> 1) & 0x01;  //tx1
  door2_state[2] = (rcv[1] >> 2) & 0x01;  //tx2
  door2_state[3] = (rcv[1] >> 3) & 0x01;  //rx1
  door2_state[4] = (rcv[1] >> 4) & 0x01;  //rx2
  door2_state[5] = (rcv[1] >> 5) & 0x01;  //top
  door2_state[6] = (rcv[1] >> 6) & 0x01;  //bottom
//  door2_state[7] = (rcv[1] >> 7) & 0x01; //unused at the moment

  //update time door has stopped moving
  if(!door1_state[0]) door_stop_time[HCdoor] = millis();
  if(!door2_state[0]) door_stop_time[TCdoor] = millis();

  //returns busy flags of both doors, 0b00 none busy, 0b01 door1 busy, 0b10 door2 busy, 0b11 both busy
  return (door2_state[0] << 1) | door1_state[0];  //busy flag
}

//move door up/down ------------------------------------------------------------
void moveDoor(uint8_t address,uint8_t door,uint8_t direction){
  uint16_t pulsetime = door_speed[door];

  //check if door is not already at target or moving towards target. up =  0, down = 1
  if((!door_open[door] && !door_moving[door] && !direction) ||
     (!door_open[door] &&  door_moving[door] &&  direction) ||
     ( door_open[door] && !door_moving[door] &&  direction) ||
     ( door_open[door] &&  door_moving[door] && !direction)){

    //if door is moving and we issue a new command, door state must be changed as the
    //getDoorstatus command will not be triggered before door has reached new (opposite) target
    if(door_moving[door]) door_open[door] = !door_open[door];

    //send movement instructions to door
    uint8_t sendbuffer[7] = {0,0,0,0,0,0,0};
    sendbuffer[0] = 2;  //option for different move commands
    sendbuffer[1] = direction;
    sendbuffer[2] = pulsetime & 0xff;
    sendbuffer[3] = (pulsetime >> 8) & 0xff;
    sendbuffer[4] = door;

    uint8_t send_status = 1;
    while(send_status != 0){
      Wire.beginTransmission(address); //address
      for(uint8_t i = 0; i < 7; i++) Wire.write(sendbuffer[i]);
      send_status = Wire.endTransmission();
    }

    //set flag for moving and times for polling and checking movement duration
    door_moving[door] = 1; //set door status to moving
    door_poll_time[HCdoor] = millis();
    door_poll_time[TCdoor] = millis();
    door_move_time[door] = millis();
  }
}

//critical error, flash LED SOS, stop everything -------------------------------
void criticalerror(){
  while(1){
    digitalWrite(errorLED,HIGH);
    delay(200);
    digitalWrite(errorLED,LOW);
    delay(200);
  }
}

//confirm with any button ------------------------------------------------------
void confirm(){
  while(analogRead(buttons) > 850){
    delay(50);
  }
}

//waits and returns which button (1,2,3) was pressed ---------------------------
uint8_t getButton(){
  uint16_t input = 1023;
  while(input > 850){
    input = analogRead(buttons);
    delay(50);
  }
  if(input <= 150) return 1;
  if(input > 150 && input <= 450) return 2;
  if(input > 450 && input <= 850) return 3;
}