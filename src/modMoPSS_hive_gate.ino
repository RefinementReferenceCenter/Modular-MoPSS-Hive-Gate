/*------------------------------------------------------------------------------
- PJRC Teensy 4.1 (with ethernet) pin mapping - Hardware Revision v7.0

- dual-infrared lightbarrier connectors (S|S|GND|+12V)
D36,D37 - X1 infrared barrier 1
A14,A15 - X2 infrared barrier 2
A11,A10 - X3 infrared barrier 3
A16,A17 - X4 infrared barrier 4
A1,A0   - X5
A6,A7   - X6
A8,A9   - X7
D3,D2   - X8

- multi-purpose 3-pin connectors (+12V|GND|Signal)
D29 - J1  fan 1 
D28 - J2  fan 2
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
#include <QNEthernet.h>

using namespace qindesign::network;

constexpr uint32_t kDHCPTimeout = 15'000;  // 15 seconds
constexpr uint16_t kNTPPort = 123;
// 01-Jan-1900 00:00:00 -> 01-Jan-1970 00:00:00
constexpr uint32_t kEpochDiff = 2'208'988'800;
// Epoch -> 07-Feb-2036 06:28:16
constexpr uint32_t kBreakTime = 2'085'978'496;

// UDP port.
EthernetUDP udp;
// Buffer.
uint8_t buf[48];
float RTC_drift = 100; //difference between RTC and NTP time

uint32_t ntpfetchtime = 0;

//----- declaring variables ----------------------------------------------------
//Current Version of the program
const char SOFTWARE_REV[] = "v1.0.0";

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

//Display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0,U8X8_PIN_NONE,23,22); //def,reset,SCL,SDA
uint32_t displaytime = 0;         //stores millis to time display refresh
uint8_t displayon = 1;            //flag to en/disable display

int once = 0;

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

  //----- Ethernet -------------------------------------------------------------
  //fetch mac address
  Serial.println("Fetching mac address...");
  OLEDprint(0,0,1,1,">>> Ethernet <<<");
  uint8_t mac[6];
  Ethernet.macAddress(mac);  // This is informative; it retrieves, not sets
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
    if(!Ethernet.begin()){
      Serial.printf("Failed to start Ethernet\r\n");
      
      OLEDprint(4,0,0,0,"Failed to start Eth.");
      OLEDprint(5,0,0,0,"CONFIRM");
      OLEDprint(5,14,0,1,"REPEAT");
      uint8_t buttonpress = getButton();
      if(buttonpress == 1) Ethstate = 1;
      OLEDprint(4,0,0,0,"                    "); //clear line
      OLEDprint(5,0,0,1,"                    ");
    }
    if(!Ethernet.waitForLocalIP(15000)){ //15 second timeout to get IP address via DHCP
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
  udp.begin(123);

  //create NTP request package
  memset(buf, 0, 48);
  buf[0] = 0b00'100'011;  // LI leap indicator warns of leap seconds, Version number (current 3), Mode 3 = client
  buf[1] = 0; //stratum, 0 = unspecified
  buf[2] = 6; //maximum time between successive messages 2^x
  buf[3] = 0xF1;  // Peer Clock Precision -15

  buf[12] = 90; //reference identifier "x" for experimental
  buf[13] = 90;
  buf[14] = 90;
  buf[15] = 90;

} //end of setup


//##############################################################################
//#####   L O O P   ############################################################
//##############################################################################
void loop(){

  digitalWriteFast(statusLED,HIGH); // server ~2.45ms

  if((millis() - ntpfetchtime) >= 10000){
    ntpfetchtime = millis();

    //Fetch NTP time and update RTC
    // Set the Transmit Timestamp
    uint32_t send_lt = Teensy3Clock.get();  //send local time
    if (send_lt >= kBreakTime) {
      send_lt -= kBreakTime;
    } else {
      send_lt += kEpochDiff;
    }
    uint32_t send_lt_frac15 = readRTCfrac(); //fractions of seconds 0 - 2^15
    
    // Send the packet
    Serial.println("--- Sending NTP request to the gateway...");
    elapsedMicros looptimemc;
    if (!udp.send("de.pool.ntp.org", 123, buf, 48)) { //server address, port, data, length
      Serial.println("ERROR.");
    }
    elapsedMillis timeout;
    while((udp.parsePacket() < 0) && (timeout < 200)); //returns size of packet or <= 0 if no packet

    const uint8_t *buf = udp.data(); //returns pointer to received package data

    //check if the data we received is according to spec
    int mode = buf[0] & 0x07;
    if (((buf[0] & 0xc0) == 0xc0) ||  // LI == 3 (Alarm condition)
        (buf[1] == 0) ||              // Stratum == 0 (Kiss-o'-Death)
        !(mode == 4 || mode == 5)) {  // Must be Server or Broadcast mode
      Serial.println("Discarding reply, other stuff\r\n");
      return;
    }

    //seconds and fractions when NTP received request
    uint32_t receive_st        = (uint32_t{buf[32]} << 24) | (uint32_t{buf[33]} << 16) | (uint32_t{buf[34]} << 8) | uint32_t{buf[35]};
    uint32_t receive_st_frac32 = (uint32_t{buf[36]} << 24) | (uint32_t{buf[37]} << 16) | (uint32_t{buf[38]} << 8) | uint32_t{buf[39]};
    //seconds and fractions when NTP sent
    uint32_t send_st        = (uint32_t{buf[40]} << 24) | (uint32_t{buf[41]} << 16) | (uint32_t{buf[42]} << 8) | uint32_t{buf[43]};
    uint32_t send_st_frac32 = (uint32_t{buf[44]} << 24) | (uint32_t{buf[45]} << 16) | (uint32_t{buf[46]} << 8) | uint32_t{buf[47]};
    //seconds and fractions of local clock when NTP packet is received 
    uint32_t receive_lt = Teensy3Clock.get();
    uint32_t receive_lt_frac15 = readRTCfrac();
    uint32_t looptimebuffer = looptimemc;


    if (send_st == 0) {
      Serial.println("Discarding reply, time 0\r\n");
      return;  // Also discard when the Transmit Timestamp is zero
    }

    // See: Section 3, "NTP Timestamp Format"
    if ((send_lt & 0x80000000U) == 0) {
      send_lt += kBreakTime;
    } else {
      send_lt -= kEpochDiff;
    }
    if ((send_st & 0x80000000U) == 0) {
      send_st += kBreakTime;
    } else {
      send_st -= kEpochDiff;
    }
    if ((receive_st & 0x80000000U) == 0) {
      receive_st += kBreakTime;
    } else {
      receive_st -= kEpochDiff;
    }

    //calculate time between sending and receiving NTP packet to adjust for network delay
    float loop_t_ms = (float)(receive_lt_frac15 - send_lt_frac15)/32768 * 1000;

    //time between send and receive, assume send/receive takes same time, factor in server time
    uint32_t adjust = ((receive_lt_frac15 - send_lt_frac15) + ((send_st_frac32 - receive_st_frac32) >> 17)) >> 1; 

    // Set the RTC and time ~0.95 ms
    if((once % 6) == 0){
      rtc_set_secs_and_frac(send_st,(send_st_frac32 >> 17) + adjust); //set time and adjust for transmit delay
      Serial.println("--- time adjusted ---");
    }
    once ++;
    //read time from adjusted RTC
    uint32_t adjust_lt = Teensy3Clock.get();
    uint32_t adjust_lt_frac15 = readRTCfrac();

    //Format time for printing
    tmElements_t slt; //send local time split
    breakTime(send_lt, slt);
    float slt_ms = ((float)send_lt_frac15/32768) * 1000; //convert fractions to milliseconds

    tmElements_t rst;
    breakTime(receive_st, rst);
    float rst_ms = ((float)receive_st_frac32/UINT32_MAX) * 1000;

    tmElements_t sst;
    breakTime(send_st, sst);
    float sst_ms  = ((float)send_st_frac32/UINT32_MAX) * 1000;

    tmElements_t rlt;
    breakTime(receive_lt, rlt);
    float rlt_ms = ((float)receive_lt_frac15/32768) * 1000;
    
    tmElements_t alt;
    breakTime(adjust_lt, alt);
    float alt_ms = ((float)adjust_lt_frac15/32768) * 1000;

    RTC_drift = (((float)receive_lt_frac15/32768) * 1000) - (((float)((send_st_frac32 >> 17) + adjust)/32768) * 1000);


    Serial.printf("loc send: %04u-%02u-%02u %02u:%02u:%02u.%02f\r\n", slt.Year + 1970, slt.Month, slt.Day, slt.Hour, slt.Minute, slt.Second, slt_ms);
    Serial.printf("NTP rec.: %04u-%02u-%02u %02u:%02u:%02u.%02f\r\n", rst.Year + 1970, rst.Month, rst.Day, rst.Hour, rst.Minute, rst.Second, rst_ms);
    Serial.printf("NTP send: %04u-%02u-%02u %02u:%02u:%02u.%02f\r\n", sst.Year + 1970, sst.Month, sst.Day, sst.Hour, sst.Minute, sst.Second, sst_ms);
    Serial.printf("loc rec.: %04u-%02u-%02u %02u:%02u:%02u.%02f\r\n", rlt.Year + 1970, rlt.Month, rlt.Day, rlt.Hour, rlt.Minute, rlt.Second, rlt_ms);
    Serial.printf("loc adj.: %04u-%02u-%02u %02u:%02u:%02u.%02f\r\n", alt.Year + 1970, alt.Month, alt.Day, alt.Hour, alt.Minute, alt.Second, alt_ms);
    
    Serial.print("RTC diff from NTP (ms): ");
    Serial.println(RTC_drift);
    
    Serial.print("Loop time RTC:    ");
    Serial.println(loop_t_ms);
    Serial.print("Loop time micros: ");
    Serial.println((float)looptimebuffer/1000);

    Serial.print("Adjust time: ");
    Serial.println(((float)adjust)/32768 * 1000);
    Serial.println();
  }
  digitalWriteFast(statusLED,LOW);

  //##### Display #####
  if(((millis() - displaytime) >= 1000) && (readRTCfrac() <= 1638)){ //1638 = 50ms
    displaytime = millis();
    digitalWriteFast(errorLED,HIGH);

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
    //OLEDprint(0,11,0,0,nDate);
    float timefrac = ((float)readRTCfrac()/32768) * 1000;
    OLEDprintFraction(0,11,0,0,timefrac,2);
    OLEDprint(1,0,0,0,"RTC drift");
    OLEDprintFraction(1,10,0,0,RTC_drift,2);

    // Serial.print("- ");
    // Serial.print(timefrac);
    // Serial.print(" - ");
    // Serial.print(RTC_drift);
    // Serial.println(" -");
    //update display
    oled.sendBuffer();
    digitalWriteFast(errorLED,LOW);
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

void rtc_set_64(uint64_t t)
{
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