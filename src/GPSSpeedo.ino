/*
GPSSpeedo by Denis French 20181027
Hacked together from:
https://github.com/SlashDevin/NeoGPS/blob/master/examples/NMEAtimezone/NMEAtimezone.ino
https://github.com/olikraus/u8g2/blob/master/sys/arduino/u8g2_full_buffer/FontUsage/FontUsage.ino
http://masteringarduino.blogspot.com/2013/10/fastest-and-smallest-digitalread-and.html
*/

//#define DEBUG // Comment out to disable serial debug output

#define BAUD 115200 // Used for both GPS and serial debug output
#define LEFT_PIN 2 // Interrupt enabled pin on Arduino Uno/Nano/Duemilanove
#define RIGHT_PIN 3 // Interrupt enabled pin on Arduino Uno/Nano/Duemilanove
#define RESET_PIN 8 // Arduino pin connected to SSD1306 RST (not Arduino reset)
#define DC_PIN 9 // Arduino pin connected to SSD1306 DC
#define CS_PIN U8X8_PIN_NONE // Not connected
// SDA must be 11 (MOSI) on Arduino Uno/Nano/Duemilanove for HW SPI
// SCL must be 13 (SCK) on Arduino Uno/Nano/Duemilanove for HW SPI

#define portOfPin(P)\
  (((P)>=0&&(P)<8)?&PORTD:(((P)>7&&(P)<14)?&PORTB:&PORTC))
#define ddrOfPin(P)\
  (((P)>=0&&(P)<8)?&DDRD:(((P)>7&&(P)<14)?&DDRB:&DDRC))
#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))

#define pinAsInput(P) *(ddrOfPin(P))&=~pinMask(P)
#define pinAsInputPullUp(P) *(ddrOfPin(P))&=~pinMask(P);digitalHigh(P)
#define pinAsOutput(P) *(ddrOfPin(P))|=pinMask(P)
#define digitalLow(P) *(portOfPin(P))&=~pinMask(P)
#define digitalHigh(P) *(portOfPin(P))|=pinMask(P)
#define isHigh(P)((*(pinOfPin(P))& pinMask(P))>0)
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)
#define digitalState(P)((uint8_t)isHigh(P))

#include <Arduino.h>
#include <U8g2lib.h>
#include <NMEAGPS.h>
#include <GPSport.h>

U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R2, /* cs=*/ CS_PIN, /* dc=*/ DC_PIN, /* reset=*/ RESET_PIN);

static NMEAGPS  gps; // This parses received characters
static gps_fix  fix; // This contains all the parsed pieces
// ****** Set timezone details below ******
static const int32_t          zone_hours   = +10L; // AEST timezone
static const int32_t          zone_minutes =  0L; // usually zero
static const NeoGPS::clock_t  zone_offset  =
                                zone_hours   * NeoGPS::SECONDS_PER_HOUR +
                                zone_minutes * NeoGPS::SECONDS_PER_MINUTE;
static const uint8_t springMonth =  10;
static const uint8_t springDate  = 7; // latest 1st Sunday
static const uint8_t springHour  =  2;
static const uint8_t fallMonth   = 4;
static const uint8_t fallDate    =  7; // latest 1st Sunday
static const uint8_t fallHour    =  2;
uint16_t search_time = 0;
uint16_t speed_kph = 0;
//volatile bool leftOn = false; // Interrupt functions disabled
//volatile bool rightOn = false; // Interrupt functions disabled

/* Interrupt functions disabled
void leftISR()
{
  leftOn = digitalRead(LEFT_PIN);
}  // leftISR

void rightISR()
{
  rightOn = digitalRead(RIGHT_PIN);
}  // leftISR
*/

void adjustTime( NeoGPS::time_t & dt )
{
  NeoGPS::clock_t seconds = dt; // convert date/time structure to seconds
  //  Calculate DST changeover times once per reset and year!
  static NeoGPS::time_t  changeover;
  static NeoGPS::clock_t springForward, fallBack;
  if ((springForward == 0) || (changeover.year != dt.year)) {
    //  Calculate the spring changeover time (seconds)
    changeover.year    = dt.year;
    changeover.month   = springMonth;
    changeover.date    = springDate;
    changeover.hours   = springHour;
    changeover.minutes = 0;
    changeover.seconds = 0;
    changeover.set_day();
    // Step back to a Sunday, if day != SUNDAY
    changeover.date -= (changeover.day - NeoGPS::time_t::SUNDAY);
    springForward = (NeoGPS::clock_t) changeover;
    //  Calculate the fall changeover time (seconds)
    changeover.month   = fallMonth;
    changeover.date    = fallDate;
    changeover.hours   = fallHour - 1; // to account for the "apparent" DST +1
    changeover.set_day();
    // Step back to a Sunday, if day != SUNDAY
    changeover.date -= (changeover.day - NeoGPS::time_t::SUNDAY);
    fallBack = (NeoGPS::clock_t) changeover;
  }
  //  First, offset from UTC to the local timezone
  seconds += zone_offset;
  //  Then add an hour if DST is in effect
  if ((springForward <= seconds) || (seconds < fallBack)) // && for N hemisphere, || for S
    seconds += NeoGPS::SECONDS_PER_HOUR;
  dt = seconds; // convert seconds back to a date/time structure
} // adjustTime

static void doSomeWork()
{
  // Set local time
  if (fix.valid.time && fix.valid.date) {
    #ifdef DEBUG
      DEBUG_PORT << fix.dateTime;
      DEBUG_PORT.print(F(" UTC "));
      DEBUG_PORT.flush();
    //DEBUG_PORT.println();
    #endif

    adjustTime( fix.dateTime );
  } // Set local time

  // Set speed
  if (fix.valid.speed) {
    #ifdef DEBUG
      DEBUG_PORT.print(u8x8_u8toa(fix.spd.whole, 3));
      DEBUG_PORT.print(F("."));
      DEBUG_PORT.print(u8x8_u16toa(fix.spd.frac, 3));
      DEBUG_PORT.println(F("kts"));
      DEBUG_PORT.flush();
    #endif

    if (fix.speed_mkn() < 1000) { // Too slow, zero out the speed
      fix.spd.whole = 0;
      fix.spd.frac  = 0;
    }
    speed_kph = (fix.spd.whole * 185) / 100;
  } // Set speed

  // Print left arrow if leftOn == true
  if (isHigh(LEFT_PIN)) {
    u8g2.clearBuffer(); // Can't clear screen with overtype of spaces?!?
    u8g2.setFont(u8g2_font_open_iconic_arrow_8x_t);
    u8g2.drawGlyph(0,63,73); // Left arrow glyph of above font

    #ifdef DEBUG
      u8g2.setDrawColor(2);
      u8g2.drawHLine(0,31,128); // Draw line halfway up screen
      u8g2.drawHLine(0,32,128); // Draw line halfway up screen
      u8g2.setDrawColor(1);
    #endif
  } // leftOn

  // Print right arrow if righOn == true
  else if (isHigh(RIGHT_PIN)) {
    u8g2.clearBuffer(); // Can't clear screen with overtype of spaces?!?
    u8g2.setFont(u8g2_font_open_iconic_arrow_8x_t);
    u8g2.drawGlyph(64,63,74); // Right arrow glyph of above font

    #ifdef DEBUG
      u8g2.setDrawColor(2);
      u8g2.drawHLine(0,31,128); // Draw line halfway up screen
      u8g2.drawHLine(0,32,128); // Draw line halfway up screen
      u8g2.setDrawColor(1);
    #endif
  } // rightOn

  // Print speed if >= 10
  else if (fix.valid.speed && speed_kph >= 10) {
      u8g2.setFont(u8g2_font_inb63_mn);
      u8g2.setCursor(-8,62); // Left side of "1" on left edge of display
      if (speed_kph >= 100) { // Print "1" if speed >= 100
        u8g2.print(F("1"));
      }
      else { // Blank leftmost digit if speed < 100
        u8g2.print(F(" "));
      }
      u8g2.setCursor(31,63); // Right side of "5" and "9" on right edge of display
      u8g2.print(u8x8_u16toa(speed_kph % 100, 2)); // Print last 2 digits
    } // Print speed
  // Print time if speed < 10
  else if (fix.valid.time && fix.valid.date){
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_7Segments_26x42_mn);
      u8g2.setCursor(0,53); // 10 up from bottom to fit 10 font status row
      u8g2.print(u8x8_u8toa(fix.dateTime.hours, 2));
      u8g2.print(u8x8_u8toa(fix.dateTime.minutes, 2));
    } // Print time
  // Speed and/or time fix broken; print status
  else {
    #ifdef DEBUG
      DEBUG_PORT.print( F("STATUS: ") );
      DEBUG_PORT.println(fix.status);
      DEBUG_PORT.flush();
    #endif

    u8g2.setFont(u8g2_font_7x14B_mr); // 10 high font; is bold suitable?
    u8g2.setCursor(0,10); // Check top row of text
    if (fix.status == 0) { // Searching for satellites
      if (search_time == 0) search_time = millis();
      u8g2.clearBuffer(); // Clear indicator arrows if present
      u8g2.print(F("SEARCHING... "));
      u8g2.print((millis() - search_time) / 1000);
    }
    else { // Something else ???
      u8g2.print(F("STATUS: "));
      u8g2.print(fix.status);
    }
    u8g2.print(F("          ")); // Blank out init message
  } // Print status

  #ifdef DEBUG
    u8g2.setDrawColor(2);
    u8g2.drawFrame(0,0,128,64); // Show 128x64 boundary
    u8g2.setDrawColor(1);
  #endif

  u8g2.sendBuffer();
} // doSomeWork

static void GPSloop()
{
  while (gps.available( gpsPort )) {
    fix = gps.read();
    doSomeWork();
  }
} // GPSloop

void setup()
{
  /* Interrupt functions disabled
  pinMode(LEFT_PIN, INPUT);
  pinMode(RIGHT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LEFT_PIN), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_PIN), rightISR, CHANGE);
  */
  pinAsInput(LEFT_PIN);
  pinAsInput(RIGHT_PIN);

  u8g2.begin();
  gpsPort.begin(BAUD);

  #ifdef DEBUG
    DEBUG_PORT.begin(BAUD);
    while (!DEBUG_PORT);
    DEBUG_PORT.println( F("INITIALISING...") );
    DEBUG_PORT.flush();
  #endif

  u8g2.clearBuffer();
  u8g2.setFontMode(0);
  u8g2.setDrawColor(1);

  u8g2.setFont(u8g2_font_7x14B_mr); // 10 high font; is bold suitable?
  u8g2.setCursor(0,10); // Check top row of text
  //u8g2.print(F("TEST5678901234567890")); // Row length is ~18.25 characters
  //u8g2.setCursor(0,64); // Last row is 63; start at 64 due blank row bottom of font
  u8g2.print(F("INITIALISING..."));

  #ifdef DEBUG
    u8g2.setDrawColor(2);
    u8g2.drawFrame(0,0,128,64); // Show 128x64 boundary
    u8g2.setDrawColor(1);
  #endif

  u8g2.sendBuffer();
} // setup

void loop()
{
  GPSloop();
} // loop
