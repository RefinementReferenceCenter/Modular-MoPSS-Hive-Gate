/*------------------------------------------------------------------------------
- PJRC Teensy 4.1 (with ethernet) pin mapping - Hardware Revision v7.0

- dual-infrared lightbarrier connectors (S|S|GND|+12V)
D36,D37 - X1
A14,A15 - X2
A11,A10 - X3
A16,A17 - X4
A1,A0   - X5
A6,A7   - X6
A8,A9   - X7
D3,D2   - X8

- multi-purpose 3-pin connectors (+12V|GND|Signal)
D29 - J1
D28 - J2
D33 - J3
D9  - J4
D8  - J5
D7  - J6
D6  - J7

D32 - ERR LED   Error LED, used for various error states
D31 - STAT LED  Status LED, can be used to signal stuff
A13 - B1,B2,B3  Input from the three buttons on the board

D11 - MOSI
D12 - MISO
D13 - SCK
D18 - SDA
D19 - SCL

*///----------------------------------------------------------------------------
#include <TimeLib.h>  //Manage Real Time CLock
#include <Wire.h>     //I2C communication
#include <SdFat.h>    //Access SD Cards
#include <U8g2lib.h>  //for SSD1306 OLED Display
#include <QNEthernet.h>

//Current Version of the program
const char SOFTWARE_REV[] = "v1.0.0";

//----- declaring variables ----------------------------------------------------

//Ethernet
using namespace qindesign::network;

constexpr uint32_t DHCPTimeout = 15'000;      //15 seconds timeout to get DHCP IP address
constexpr uint16_t NTPPort = 123;             //port for ntp requests
constexpr uint32_t EpochDiff = 2'208'988'800; //01-Jan-1900 00:00:00 -> 01-Jan-1970 00:00:00
constexpr uint32_t EBreakTime = 2'085'978'496; //Epoch -> 07-Feb-2036 06:28:16

EthernetUDP udp; //UDP port

//NTP stuff
uint8_t ntpbuf[48];        //ntp packet buffer
double RTC_drift = 100;    //difference between RTC and NTP time
elapsedMillis NTPsynctime; //time since last ntp sync in ms
//fritz.box on local network is very fast and recommended
//de.pool.ntp.org took in tests about ~200ms to respond to the ntp request vs fritz.box ~3ms
const char NTPserver[] = "fritz.box";

//NTP printing stuff
tmElements_t slt;
float slt_ms;
tmElements_t rst;
float rst_ms;
tmElements_t sst;
float sst_ms;
tmElements_t rlt;
float rlt_ms;
tmElements_t alt;
float alt_ms;
float loop_t_ms;
float server_res_t;


//I2C addresses
const uint8_t oledDisplay = 0x78; //I2C address oled display

//Buttons
const int buttons = A13;  //~1022 not pressed, ~1 left, ~323 middle, ~711 right

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
uint32_t starttime;        //start of programm

//Display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0,U8X8_PIN_NONE,23,22); //def,reset,SCL,SDA
uint32_t displaytime = 0;         //stores millis to time display refresh
uint8_t displayon = 1;            //flag to en/disable display

int once = 1;
const uint8_t is_testing = 0;

uint32_t benchy;

//##############################################################################
//#####   S E T U P   ##########################################################
//##############################################################################
void setup(){

  //----- Buttons & Fans -------------------------------------------------------
  pinMode(buttons,INPUT);
  pinMode(34,OUTPUT);
  pinMode(statusLED,OUTPUT);
  pinMode(errorLED,OUTPUT);
  
  //start I2C
  Wire.begin();
  
  //----- Display --------------------------------------------------------------
  oled.setI2CAddress(oledDisplay);
  oled.begin();
  oled.setFont(u8g2_font_6x10_mf); //set font w5 h10

  //----- communication --------------------------------------------------------
  //start Serial communication

  digitalWrite(errorLED,HIGH);
  Serial.begin(115200);
  //while(!Serial); //wait for serial connection
  digitalWrite(errorLED,LOW);
  Serial.println("alive");

  //----- Real Time Clock ------------------------------------------------------
  setSyncProvider(getTeensy3Time);
  
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
  
  //output start log to console
  if(is_testing){
    Serial.print("# Modular MoPSS Hive version: ");
    Serial.println(SOFTWARE_REV);
    
    Serial.print("# System start @ ");
    Serial.print(nicetime(starttime));
    Serial.print(" ");
    Serial.print(day(starttime));
    Serial.print("-");
    Serial.print(month(starttime));
    Serial.print("-");
    Serial.println(year(starttime));
    Serial.print("# Unixtime: ");
    Serial.println(starttime);
    Serial.println();
  }
  //----- Ethernet -------------------------------------------------------------
  //fetch mac address
  Serial.println("Fetching mac address...");
  OLEDprint(0,0,1,1,">>> Ethernet <<<");
  uint8_t mac[6];
  Ethernet.macAddress(mac);  //This is informative; it retrieves, not sets
  char dataString[18];
  sprintf(dataString,"%02X:%02X:%02X:%02X:%02X:%02X",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  //print to serial
  Serial.print("MAC: "); 
  Serial.println(dataString);
  //print to oled
  OLEDprint(1,0,0,1,"MAC");
  OLEDprint(1,4,0,1,dataString);

  //Start ethernet with DHCP
  Serial.println("Starting Ethernet with DHCP...");
  OLEDprint(2,0,0,1,"get IP via DHCP...");
  uint8_t Ethstate = 0;
  while(Ethstate == 0){
    if(!Ethernet.begin()){  //Starting Ethernet
      Serial.printf("Failed to start Ethernet\r\n");
      
      OLEDprint(4,0,0,0,"Failed to start Eth.");
      OLEDprint(5,0,0,0,"CONFIRM");
      OLEDprint(5,14,0,1,"REPEAT");
      uint8_t buttonpress = getButton();
      if(buttonpress == 1) Ethstate = 1;
      OLEDprint(4,0,0,0,"                    "); //clear line
      OLEDprint(5,0,0,1,"                    ");
    }
    if(!Ethernet.waitForLocalIP(DHCPTimeout)){ //15 second timeout to get IP address via DHCP
      Serial.printf("Failed to get IP address from DHCP\r\n");
      OLEDprint(4,0,0,1,"Failed to get IP");
      OLEDprint(5,0,0,1,"CONFIRM");
      OLEDprint(5,14,0,1,"REPEAT");
      uint8_t buttonpress = getButton();
      if(buttonpress == 1) Ethstate = 1;
      OLEDprint(4,0,0,0,"                    "); //clear line
      OLEDprint(5,0,0,1,"                    ");
    }
    else{
      Ethstate = 1;
    }
  }

  //print to display
  IPAddress ip = Ethernet.localIP();
  sprintf(dataString,"%u.%u.%u.%u",ip[0],ip[1],ip[2],ip[3]);
  Serial.print("IP:          ");
  Serial.println(dataString);
  OLEDprint(2,0,0,0,"                    ");
  OLEDprint(2,0,0,0,"IP");
  OLEDprint(2,4,0,0,dataString);

  ip = Ethernet.subnetMask();
  sprintf(dataString,"%u.%u.%u.%u",ip[0],ip[1],ip[2],ip[3]);
  Serial.print("Subnet Mask: ");
  Serial.println(dataString);
  OLEDprint(3,0,0,0,"Sub");
  OLEDprint(3,4,0,0,dataString);

  ip = Ethernet.gatewayIP();
  sprintf(dataString,"%u.%u.%u.%u",ip[0],ip[1],ip[2],ip[3]);
  Serial.print("Gateway:     ");
  Serial.println(dataString);
  OLEDprint(4,0,0,0,"Gat");
  OLEDprint(4,4,0,0,dataString);

  ip = Ethernet.dnsServerIP();
  sprintf(dataString,"%u.%u.%u.%u",ip[0],ip[1],ip[2],ip[3]);
  Serial.print("DNS:         ");
  Serial.println(dataString);
  OLEDprint(5,0,0,0,"DNS");
  OLEDprint(5,4,0,1,dataString);

  // Start UDP listening on the NTP port
  udp.begin(NTPPort);

  //create NTP request package
  memset(ntpbuf, 0, 48);
  ntpbuf[0] = 0b00'100'011;  // LI leap indicator warns of leap seconds, Version number (current 3), Mode 3 = client
  ntpbuf[1] = 0; //stratum, 0 = unspecified
  ntpbuf[2] = 6; //maximum time between successive messages 2^x
  ntpbuf[3] = 0xF1;  // Peer Clock Precision -15

  ntpbuf[12] = 90; //reference identifier "x" for experimental
  ntpbuf[13] = 90;
  ntpbuf[14] = 90;
  ntpbuf[15] = 90;
  
} //end of setup


//##############################################################################
//#####   L O O P   ############################################################
//##############################################################################
void loop(){
  
  String MISCdataString = ""; //holds info of time sync events
  
  if(once){
    NTPsync(1);
    // delay(5000);
    // NTPsync();
    // delay(5000);
    once = 0;
  }
  
  //NTP sync at the full hour, or if we missed it after one hour
  if((((Teensy3Clock.get() % 10) == 0) && (NTPsynctime > 2000)) || (NTPsynctime > 1000 * 60 + 2)){  //sync at the full minute
  //if((((Teensy3Clock.get() % 3600) == 0) && (NTPsynctime > 2000)) || (NTPsynctime > 1000 * 60 * 60 + 30)){ //sync at full hour or when 1h + 30 sec. has passed
    MISCdataString = createMISCDataString("NTP","last sync ms ago",String(NTPsynctime),MISCdataString);
    
    uint8_t NTPstate = NTPsync(1);
    
    if(NTPstate){
      NTPsynctime = 10002; //check again in 10 seconds if we didn't get a response
      MISCdataString = createMISCDataString("NTP","Error return code",NTPstate,MISCdataString);
    }
    else{
      NTPsynctime = 0;
      
      char timeinfo[37];
      sprintf(timeinfo, "loc send: %04u-%02u-%2u %02u:%02u:%02u-%06.2f",slt.Year + 1970,slt.Month,slt.Day,slt.Hour,slt.Minute,slt.Second,slt_ms);
      MISCdataString = createMISCDataString("NTP",timeinfo,"",MISCdataString);
      sprintf(timeinfo, "NTP rec.: %04u-%02u-%2u %02u:%02u:%02u-%06.2f",rst.Year + 1970,rst.Month,rst.Day,rst.Hour,rst.Minute,rst.Second,rst_ms);
      MISCdataString = createMISCDataString("NTP",timeinfo,"",MISCdataString);
      sprintf(timeinfo, "NTP send: %04u-%02u-%2u %02u:%02u:%02u-%06.2f",sst.Year + 1970,sst.Month,sst.Day,sst.Hour,sst.Minute,sst.Second,sst_ms);
      MISCdataString = createMISCDataString("NTP",timeinfo,"",MISCdataString);
      sprintf(timeinfo, "loc rec.: %04u-%02u-%2u %02u:%02u:%02u-%06.2f",rlt.Year + 1970,rlt.Month,rlt.Day,rlt.Hour,rlt.Minute,rlt.Second,rlt_ms);
      MISCdataString = createMISCDataString("NTP",timeinfo,"",MISCdataString);
      sprintf(timeinfo, "loc adj.: %04u-%02u-%2u %02u:%02u:%02u-%06.2f",alt.Year + 1970,alt.Month,alt.Day,alt.Hour,alt.Minute,alt.Second,alt_ms);
      MISCdataString = createMISCDataString("NTP",timeinfo,"",MISCdataString);
      
      MISCdataString = createMISCDataString("NTP","RTC diff from NTP ms",RTC_drift,MISCdataString);
      MISCdataString = createMISCDataString("NTP","Roundtrip time RTC ms",loop_t_ms,MISCdataString);
      MISCdataString = createMISCDataString("NTP","time to server response ms",server_res_t,MISCdataString);
    }
  }
  
  digitalWriteFast(statusLED,LOW);

  //##### Display #####
  if(((millis() - displaytime) >= 100)  && (readRTCfrac() <= 328)){ //1638 = 50ms, 328 10ms
    displaytime = millis();
    digitalWriteFast(statusLED,HIGH);
    
    oled.clearBuffer(); //clear display
    
    //display current time from RTC and date
    OLEDprint(0,0,0,0,nicetime(now()));
    
    float timefrac = ((float)readRTCfrac()/32768) * 1000;
    OLEDprintFraction(0,11,0,0,timefrac,3);
    OLEDprint(1,0,0,0,"RTC drift ms");
    OLEDprintFraction(1,13,0,0,RTC_drift,2);

    oled.sendBuffer();
    digitalWriteFast(statusLED,LOW);
  }
  
  //log MISC events
  if(MISCdataString.length() != 0){
		dataFile.println(MISCdataString);	//append Datastring to file
		dataFile.flush();
		dataFileBackup.println(MISCdataString);
		dataFileBackup.flush();
		Serial.println(MISCdataString);
	}

} //end of loop

//##############################################################################
//#####   F U N C T I O N S   ##################################################
//##############################################################################

uint64_t rtc_get_64(void)
{
	uint32_t hi1 = SNVS_HPRTCMR;
  uint64_t tmp64;
	uint32_t lo1 = SNVS_HPRTCLR;
	while (1) {
		uint32_t hi2 = SNVS_HPRTCMR;
		uint32_t lo2 = SNVS_HPRTCLR;
		if (lo1 == lo2 && hi1 == hi2) {
      tmp64 = hi1;
			return (tmp64 << 32) | lo1;
		}
		hi1 = hi2;
		lo1 = lo2;
	}
}

int16_t readRTCfrac() //read fractional seconds from RTC 1/32768 sec.
{
  uint32_t hi1, lo1, hi2, lo2;
  hi1 = SNVS_HPRTCMR;
  lo1 = SNVS_HPRTCLR;
  while(1) {
    hi2 = SNVS_HPRTCMR;
    lo2 = SNVS_HPRTCLR;
    if (lo1 == lo2 && hi1 == hi2) {
      return (int16_t)(lo2 & 0x7fff); //return last 15 bits
    }
    hi1 = hi2;
    lo1 = lo2;
  }
}

void rtc_set_64(uint64_t t){
	// stop the RTC
	SNVS_HPCR &= ~(SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS);
	while (SNVS_HPCR & SNVS_HPCR_RTC_EN); // wait
	// stop the SRTC
	SNVS_LPCR &= ~SNVS_LPCR_SRTC_ENV;
	while (SNVS_LPCR & SNVS_LPCR_SRTC_ENV); // wait
	// set the SRTC
	SNVS_LPSRTCLR = t & 0xffffffff;
  SNVS_LPSRTCMR = t >> 32;
	// start the SRTC
	SNVS_LPCR |= SNVS_LPCR_SRTC_ENV;
	while (!(SNVS_LPCR & SNVS_LPCR_SRTC_ENV)); // wait
	// start the RTC and sync it to the SRTC
	SNVS_HPCR |= SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS;
}

void rtc_set_secs_and_frac(uint32_t secs, uint32_t frac)
{
	// stop the RTC
	SNVS_HPCR &= ~(SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS);
	while (SNVS_HPCR & SNVS_HPCR_RTC_EN); // wait
	// stop the SRTC
	SNVS_LPCR &= ~SNVS_LPCR_SRTC_ENV;
	while (SNVS_LPCR & SNVS_LPCR_SRTC_ENV); // wait
	// set the SRTC
  SNVS_LPSRTCLR = ((secs & 0x1ffffUL) << 15) | (frac & 0x7fff);
	SNVS_LPSRTCMR = secs >> 17;
	// start the SRTC
	SNVS_LPCR |= SNVS_LPCR_SRTC_ENV;
	while (!(SNVS_LPCR & SNVS_LPCR_SRTC_ENV)); // wait
	// start the RTC and sync it to the SRTC
	SNVS_HPCR |= SNVS_HPCR_RTC_EN | SNVS_HPCR_HP_TS;
}

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

//Fetch NTP time and update RTC ------------------------------------------------
uint8_t NTPsync(uint8_t update_time){
  
  //Set the Transmit Timestamp < 0.001 ms
  while(udp.parsePacket() > 0); //clear any udp data left in the buffer
  if(is_testing == 1) Serial.println("--- Sending NTP request to the gateway..."); //nothing slow from here until time is written
  uint32_t send_lt = Teensy3Clock.get();  //get local time to send
  if(send_lt >= EBreakTime) send_lt -= EBreakTime;  //see epochs etc.
  else send_lt += EpochDiff;
  uint32_t send_lt_frac15 = readRTCfrac(); //fractions of seconds 0 - 2^15
  
  //--- Send the packet, dependent on server response time, with local network fritzbox this can take about ~8 ms (avg ~3 ms)
  if(!udp.send(NTPserver,NTPPort,ntpbuf,48)) Serial.println("ERROR sending ntp package"); //server address, port, data, length
  
  elapsedMicros timeout;  //micros for benchmarking vs millis
  while((udp.parsePacket() < 0) && (timeout < 1000000));   //returns size of packet or <= 0 if no packet
  if(timeout >= 1000000){
    if(is_testing == 1) Serial.println("WARNING: no NTP package received"); //check if the receiving timed out
    return 1;
  }
  else{
    uint32_t timeout_buf = timeout; //mesure how long it took for the server to answer
    
    const uint8_t *ntpbuf = udp.data(); //returns pointer to received package data
    
    //check if the data we received is according to spec < 0.001 ms
    int mode = ntpbuf[0] & 0x07;
    if(((ntpbuf[0] & 0xc0) == 0xc0) || //LI == 3 (Alarm condition)
      (ntpbuf[1] == 0) ||              //Stratum == 0 (Kiss-o'-Death)
      !(mode == 4 || mode == 5)) {     //Must be Server or Broadcast mode
      if(is_testing == 1) Serial.println("Discarding reply, faulty");
      return 2;
    }
    
    //seconds and fractions when NTP received request < 0.002 ms until adjusting time
    uint32_t receive_st        = (uint32_t{ntpbuf[32]} << 24) | (uint32_t{ntpbuf[33]} << 16) | (uint32_t{ntpbuf[34]} << 8) | uint32_t{ntpbuf[35]}; //receive server time
    uint32_t receive_st_frac32 = (uint32_t{ntpbuf[36]} << 24) | (uint32_t{ntpbuf[37]} << 16) | (uint32_t{ntpbuf[38]} << 8) | uint32_t{ntpbuf[39]};
    //seconds and fractions when NTP sent
    uint32_t send_st        = (uint32_t{ntpbuf[40]} << 24) | (uint32_t{ntpbuf[41]} << 16) | (uint32_t{ntpbuf[42]} << 8) | uint32_t{ntpbuf[43]};
    uint32_t send_st_frac32 = (uint32_t{ntpbuf[44]} << 24) | (uint32_t{ntpbuf[45]} << 16) | (uint32_t{ntpbuf[46]} << 8) | uint32_t{ntpbuf[47]};
    //seconds and fractions of local clock when NTP packet is received 
    uint32_t receive_lt = Teensy3Clock.get();
    uint32_t receive_lt_frac15 = readRTCfrac();
    
    //Discard if reply empty, also discard when the Transmit Timestamp is zero
    if (send_st == 0) {
      if(is_testing == 1) Serial.println("Discarding reply, time 0");
      return 3;
    }

    //See: Section 3, "NTP Timestamp Format"
    if((send_lt & 0x80000000U) == 0) send_lt += EBreakTime;
    else send_lt -= EpochDiff;
    if((send_st & 0x80000000U) == 0) send_st += EBreakTime;
    else send_st -= EpochDiff;
    if((receive_st & 0x80000000U) == 0) receive_st += EBreakTime;
    else receive_st -= EpochDiff;

    //calculate time between sending and receiving NTP packet to adjust for network delay
    uint16_t rollover = 0; //account for overflow
    if(receive_lt_frac15 < send_lt_frac15) rollover = 32768;
    uint32_t loop_t_frac15 = receive_lt_frac15 - send_lt_frac15 + rollover;
    
    if(receive_st_frac32 > send_st_frac32){
      receive_st_frac32 = receive_st_frac32 + UINT32_MAX;
      send_st = send_st + 1;
    }
    
    //time between send and receive, assume send/receive takes same time, factor in server time
    //uint32_t adjust_frac15 = loop_t_frac15 >> 1; //calculate difference and divide by 2 simple loop time/2
    uint32_t adjust_frac15 = ((receive_lt_frac15 - send_lt_frac15 + rollover) - ((send_st_frac32 - receive_st_frac32) >> 17)) >> 1; //subtract server time
    
    // Set the RTC and time ~0.98 ms ??
    if(((send_st_frac32 >> 17) + adjust_frac15) > 32767) send_st += 1; //account for frac overflow
    if(update_time) rtc_set_secs_and_frac(send_st,(send_st_frac32 >> 17) + adjust_frac15); //set time and adjust for transmit delay, frac will only use lower 15bits
    
    //read time from adjusted RTC <0.007 ms until end
    uint32_t adjust_lt = Teensy3Clock.get();
    uint32_t adjust_lt_frac15 = readRTCfrac();
    
    //Format time for printing
    //tmElements_t slt; //send local time split
    breakTime(send_lt, slt);
    slt_ms = ((float)send_lt_frac15/32768) * 1000; //convert fractions to milliseconds
    
    //tmElements_t rst;
    breakTime(receive_st, rst);
    rst_ms = ((float)receive_st_frac32/UINT32_MAX) * 1000;
    
    //tmElements_t sst;
    breakTime(send_st, sst);
    sst_ms  = ((float)send_st_frac32/UINT32_MAX) * 1000;
    
    //tmElements_t rlt;
    breakTime(receive_lt, rlt);
    rlt_ms = ((float)receive_lt_frac15/32768) * 1000;
    
    //tmElements_t alt;
    breakTime(adjust_lt, alt);
    alt_ms = ((float)adjust_lt_frac15/32768) * 1000;
    
    RTC_drift = -((((float)receive_lt_frac15/32768) * 1000) - (((float)((send_st_frac32 >> 17) + adjust_frac15)/32768) * 1000) + ((receive_lt - adjust_lt) * 1000));
    
    loop_t_ms = (float)loop_t_frac15/32768 * 1000;
    
    if(is_testing == 1){
      Serial.printf("loc send: %04u-%02u-%02u %02u:%02u:%02u-%02f\r\n", slt.Year + 1970, slt.Month, slt.Day, slt.Hour, slt.Minute, slt.Second, slt_ms);
      Serial.printf("NTP rec.: %04u-%02u-%02u %02u:%02u:%02u-%02f\r\n", rst.Year + 1970, rst.Month, rst.Day, rst.Hour, rst.Minute, rst.Second, rst_ms);
      Serial.printf("NTP send: %04u-%02u-%02u %02u:%02u:%02u-%02f\r\n", sst.Year + 1970, sst.Month, sst.Day, sst.Hour, sst.Minute, sst.Second, sst_ms);
      Serial.printf("loc rec.: %04u-%02u-%02u %02u:%02u:%02u-%02f\r\n", rlt.Year + 1970, rlt.Month, rlt.Day, rlt.Hour, rlt.Minute, rlt.Second, rlt_ms);
      Serial.printf("loc adj.: %04u-%02u-%02u %02u:%02u:%02u-%02f\r\n", alt.Year + 1970, alt.Month, alt.Day, alt.Hour, alt.Minute, alt.Second, alt_ms);
      
      Serial.print("RTC diff from NTP (ms): ");
      Serial.println(RTC_drift);
      
      Serial.print("Roundtrip time RTC ms:  ");
      Serial.println(loop_t_ms);

      Serial.print("Add roundtrip delay ms: ");
      Serial.println(((float)adjust_frac15)/32768 * 1000);
      
      Serial.print("time to server response ms ");
      server_res_t = (float)timeout_buf/1000;
      Serial.println(server_res_t);
      
      Serial.println();
    }
    
    return 0;
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

//Create misc data String ------------------------------------------------------
String createMISCDataString(String identifier, String event1,String event2,String dataString){
  time_t unixtime = now();

  if(dataString != 0) dataString += "\n"; //if datastring is not empty, add newline
  dataString += identifier;
  dataString += ",";
  dataString += unixtime;
  dataString += ",";
  dataString += millis();
  dataString += ",";
  dataString += event1;
  dataString += ",";
  dataString += event2;
  
  return dataString;
}