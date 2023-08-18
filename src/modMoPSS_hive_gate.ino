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

int once = 2;

//##############################################################################
//#####   S E T U P   ##########################################################
//##############################################################################
void setup(){
  pinMode(statusLED,OUTPUT);
  pinMode(errorLED,OUTPUT);
  //----- Buttons & Fans -------------------------------------------------------
  pinMode(buttons,INPUT);
  pinMode(34,OUTPUT);
 
  //start I2C
  Wire.begin();

  //----- communication --------------------------------------------------------
  //start Serial communication

  digitalWrite(errorLED,HIGH);

  Serial.begin(115200);
  while(!Serial); //wait for serial connection

  digitalWrite(errorLED,LOW);

  Serial.println("alive");

  //----- Real Time Clock ------------------------------------------------------
  //setSyncProvider(getTeensy3Time);

  //----- Ethernet -------------------------------------------------------------
  // start the Ethernet
  

  Serial.println("Starting...\r\n");
  Serial.println("start");

  uint8_t mac[6];
  Ethernet.macAddress(mac);  // This is informative; it retrieves, not sets
  Serial.printf("MAC = %02x:%02x:%02x:%02x:%02x:%02x\r\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Serial.printf("Starting Ethernet with DHCP...\r\n");
  if (!Ethernet.begin()) {
    Serial.printf("Failed to start Ethernet\r\n");
    return;
  }
  if (!Ethernet.waitForLocalIP(kDHCPTimeout)) {
    Serial.printf("Failed to get IP address from DHCP\r\n");
    return;
  }

  IPAddress ip = Ethernet.localIP();
  Serial.printf("    Local IP    = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.subnetMask();
  Serial.printf("    Subnet mask = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.gatewayIP();
  Serial.printf("    Gateway     = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);
  ip = Ethernet.dnsServerIP();
  Serial.printf("    DNS         = %u.%u.%u.%u\r\n", ip[0], ip[1], ip[2], ip[3]);

  // Start UDP listening on the NTP port
  udp.begin(123);

  //create NTP request package
  memset(buf, 0, 48);
  buf[0] = 0b11'100'011;  // LI leap indicator, Version number (current 3), Mode 3 = client
  buf[1] = 0; //stratum, 0 = unspecified
  buf[2] = 6; //maximum time between successive messages 2^x
  buf[3] = 0xEC;  // Peer Clock Precision -13??

//  buf[12] = 49; //reference identifier "x"
//  buf[13] = 0x4E;
//  buf[14] = 49;
//  buf[15] = 52;

 buf[12] = 90; //reference identifier "x"
 buf[13] = 90;
 buf[14] = 90;
 buf[15] = 90;



} //end of setup

time_t prevDisplay = 0; // when the digital clock was displayed

//##############################################################################
//#####   L O O P   ############################################################
//##############################################################################
void loop(){

  digitalWriteFast(statusLED,HIGH); // server ~2.45ms

  // Set the Transmit Timestamp
  uint32_t t = Teensy3Clock.get();
  if (t >= kBreakTime) {
    t -= kBreakTime;
  } else {
    t += kEpochDiff;
  }
  
  uint32_t tfrac = readRTCfrac();
  
  //transmit timestamps will not be sent
  // buf[24] = t >> 24; 
  // buf[25] = t >> 16;
  // buf[26] = t >> 8;
  // buf[27] = t;
  
  // uint64_t tsend = (tfrac/32768) * 4294967296;
  // buf[28] = tsend >> 24;
  // buf[29] = tsend >> 16;
  // buf[30] = tsend >> 8;
  // buf[31] = tsend;

  //16 17 18 19 20 s|f 21 22 23 local last rtc set Reference timestamp
  //24 25 26 27 28 s|f 29 30 31 local send Originate timestamp
  //32 33 34 35 36 s|f 37 38 39 NTP server Receive timestamp
  //40 41 42 43 44 s|f 45 46 47 NTP server Transmit timestamp


  // Send the packet
  Serial.println("--- Sending NTP request to the gateway...");
  if (!udp.send("de.pool.ntp.org", 123, buf, 48)) { //server address, port, data, length
  //if (!udp.send("0.pool.ntp.org", 123, buf, 48)) {
    Serial.println("ERROR.");
  }
  elapsedMillis timeout;
  while((udp.parsePacket() < 0) && (timeout < 200)); //returns size of packet or <= 0 if no packet


  const uint8_t *buf = udp.data(); //returns pointer to received package data

  // See: Section 5, "SNTP Client Operations"
  int mode = buf[0] & 0x07;
  if (((buf[0] & 0xc0) == 0xc0) ||  // LI == 3 (Alarm condition)
      (buf[1] == 0) ||              // Stratum == 0 (Kiss-o'-Death)
      !(mode == 4 || mode == 5)) {  // Must be Server or Broadcast mode
    Serial.println("Discarding reply, other stuff\r\n");
    return;
  }

  //seconds server receive
  uint32_t lt   = t;
  uint32_t ltms = tfrac;

  //seconds server receive
  uint32_t st   = (uint32_t{buf[32]} << 24) | (uint32_t{buf[33]} << 16) | (uint32_t{buf[34]} << 8) | uint32_t{buf[35]};
  //fractions server receive
  uint32_t stms = (uint32_t{buf[36]} << 24) | (uint32_t{buf[37]} << 16) | (uint32_t{buf[38]} << 8) | uint32_t{buf[39]};

  //seconds server send
  t   = (uint32_t{buf[40]} << 24) | (uint32_t{buf[41]} << 16) | (uint32_t{buf[42]} << 8) | uint32_t{buf[43]};
  //fractions server send
  uint32_t tms = (uint32_t{buf[44]} << 24) | (uint32_t{buf[45]} << 16) | (uint32_t{buf[46]} << 8) | uint32_t{buf[47]};

  if (t == 0) {
    Serial.println("Discarding reply, time 0\r\n");
    return;  // Also discard when the Transmit Timestamp is zero
  }

  uint32_t rt = Teensy3Clock.get();
  uint32_t rtfrac = readRTCfrac();

  // See: Section 3, "NTP Timestamp Format"
  if ((lt & 0x80000000U) == 0) {
    lt += kBreakTime;
  } else {
    lt -= kEpochDiff;
  }
  if ((t & 0x80000000U) == 0) {
    t += kBreakTime;
  } else {
    t -= kEpochDiff;
  }
  if ((st & 0x80000000U) == 0) {
    st += kBreakTime;
  } else {
    st -= kEpochDiff;
  }

  //caluclate shift through transit times
   
  // loc send: 2023-08-16 14:00:29.637.573242
  // NTP rec.: 2023-08-16 14:00:29.669.647034
  // NTP send: 2023-08-16 14:00:29.669.718506
  // loc rec.: 2023-08-16 14:00:29.668.487549
  // loc adj.: 2023-08-16 14:00:29.669.799805



  //float diff = abs(((float)tms/UINT32_MAX) - ((float)tfrac/32768))*1000;
  float diff = (float)(rtfrac - tfrac)/32768 * 1000;
  uint32_t adjust = abs(rtfrac - tfrac)/2; //time between send and receive
  //adjust = adjust << 17; //convert to ntp frac

  // Serial.println( ((float)adjust/32768) * 1000);
  // Serial.println( ((float)tms/UINT32_MAX) * 1000);
  uint32_t newdiff = (tms >> 17) + adjust;
  // Serial.println( ((float)newdiff/32768) * 1000);



  // Set the RTC and time ~0.95 ms
  if(once){
    once -= 1;
    rtc_set_secs_and_frac(t,(tms >> 17) + adjust);
  }
  //read RTC and time
  uint32_t atfrac = readRTCfrac();
  uint32_t at = Teensy3Clock.get();
  tmElements_t atm;
  breakTime(at, atm);
  float atimems = ((float)atfrac/32768) * 1000;

  
  // Print the time
  tmElements_t ltm;
  breakTime(lt, ltm);
  tmElements_t tm;
  breakTime(t, tm);
  tmElements_t stm;
  breakTime(st, stm);
  tmElements_t rtm;
  breakTime(rt, rtm);

  float rtimems = ((float)rtfrac/32768) * 1000;
  float ltimems = ((float)ltms/32768) * 1000;
  float stimems = ((float)stms/UINT32_MAX) * 1000;
  float timems  = ((float)tms/UINT32_MAX) * 1000;


  Serial.printf("loc send: %04u-%02u-%02u %02u:%02u:%02u.%02f\r\n", ltm.Year + 1970, ltm.Month, ltm.Day, ltm.Hour, ltm.Minute, ltm.Second, ltimems);
  Serial.printf("NTP rec.: %04u-%02u-%02u %02u:%02u:%02u.%02f\r\n", stm.Year + 1970, stm.Month, stm.Day, stm.Hour, stm.Minute, stm.Second, stimems);
  Serial.printf("NTP send: %04u-%02u-%02u %02u:%02u:%02u.%02f\r\n", tm.Year + 1970, tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second, timems);
  Serial.printf("loc rec.: %04u-%02u-%02u %02u:%02u:%02u.%02f\r\n", rtm.Year + 1970, rtm.Month, rtm.Day, rtm.Hour, rtm.Minute, rtm.Second, rtimems);
  Serial.printf("loc adj.: %04u-%02u-%02u %02u:%02u:%02u.%02f\r\n", atm.Year + 1970, atm.Month, atm.Day, atm.Hour, atm.Minute, atm.Second, atimems);
  Serial.print("difference: ");
  Serial.println( (((float)newdiff/32768) * 1000) - (((float)rtfrac/32768) * 1000) );


  digitalWriteFast(statusLED,LOW);
  delay(60'000); //being nice to ntp server

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


//0b1'11111111'11111111




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