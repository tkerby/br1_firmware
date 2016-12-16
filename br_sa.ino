/*
   br1_firmware - for http://timhawes.com/br1 PCB (modified to use UART1 pin GPIO2)

   Arduino packages:
     https://github.com/esp8266/Arduino/

   Libraries:
     https://github.com/lp0/NeoPixelBus/tree/UartDriven (forked from https://github.com/Makuna/NeoPixelBus/tree/UartDriven)

*/

#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <WiFiUdp.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <NeoPixelBusSA.h>
#include <DNSServer.h>
#include <ESP8266mDNS.h>

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

#ifdef ESP8266
extern "C" {
#include "user_interface.h"
}
#endif

// #define DEBUG
// #define DEBUG_PACKETS

#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTDEC(x) Serial.print(x, DEC)
#define DEBUG_PRINTHEX(x) Serial.print(x, HEX)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTDEC(x)
#define DEBUG_PRINTHEX(x)
#define DEBUG_PRINTLN(x)
#endif

#define MAX_PIXELS 250

#define HUE_MAX 360
#define HUE_EXP_RANGE 60
#define HUE_EXP_TIMES 2
#define HUE_EXP_MAX ((HUE_MAX - HUE_EXP_RANGE) + (HUE_EXP_TIMES * HUE_EXP_RANGE))

const uint8_t irPin = 14;
const uint8_t buttonPin = 0;
const uint8_t dataPin = 2;
const unsigned int udpPort = 2812;

const char *wifiModeNames[] = { "STA", "AP", NULL };

const char *colourOrderNames[] = { "RGB", "GRB", "BRG", NULL };
const uint16_t colourOrderValues[] = { NEO_RGB, NEO_GRB, NEO_BRG };

const char *modeNames[255] = { "Black", "Red", "Dim red", "Green", "Yellow", "Blue", "Magenta", "Cyan", "White",
                               "HSV Scroll", "HSV Fade", "Christmas (Red and Green)", "Christmas (Red and Green (Twinkle)", "White (Twinkle)",
                               "Red night light", "Christmas (Work)", "Christmas (Work (Twinkle))", "HSV Scroll (Twinkle)", "HSV Static (Twinkle)",
                               "HSV Fade (Twinkle)", "Knight Rider", "Knight Rider (HSV Fade)", "Single Random 1 (slow)", "Single Random 1 (fast)",
                               "Full Random 1 (slow)", "Full Random 1 (fast)", "Single Random 2 (slow)", "Single Random 2 (fast)",
                               "Full Random 2 (slow)", "Full Random 2 (fast)", "Flames", "Fast Scroll", "Random Wipe", "Rave", NULL
                             };

struct EepromData {
  uint8_t configured;
  char ssid[128];
  char passphrase[128];
  uint16_t pixelcount;
  uint16_t colourorder;
  uint8_t scale0red;
  uint8_t scale0green;
  uint8_t scale0blue;
  uint8_t defaultmode;
  uint8_t wifimode;
  uint8_t scale50red;
  uint8_t scale50green;
  uint8_t scale50blue;
  uint8_t scale100red;
  uint8_t scale100green;
  uint8_t scale100blue;
} eepromData;

char myhostname[64];
IPAddress ip;
NeoPixelBus pixels;
ESP8266WebServer server(80);
WiFiUDP Udp;
MDNSResponder mdns;
uint8_t inboundMessage[1500];
uint8_t ledMode = 9;
boolean ledModeChanged = true;
uint8_t buttonState = HIGH;

uint8_t scalered[MAX_PIXELS];
uint8_t scalegreen[MAX_PIXELS];
uint8_t scaleblue[MAX_PIXELS];

void configuration_mode();
void run_mode();
void udpLoop();
void tcpLoop();
void ledLoop();
void singleColour(uint8_t red, uint8_t green, uint8_t blue);
void makeScale();

void setup() {
  boolean configMode = false;

  pinMode(buttonPin, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println();
  Serial.println("Booting - Hold button to enter configuration mode");
  Serial.print("getResetInfo=");
  Serial.println(ESP.getResetInfo());

  snprintf(myhostname, sizeof(myhostname), "ws2812-%08x", ESP.getChipId());
  wifi_station_set_hostname(myhostname);
  delay(1000); //Used a button also used for flashing. Wait a little
  EEPROM.begin(512);
  EEPROM.get(0, eepromData);

  if (eepromData.configured != 1) {
    Serial.println("EEPROM is empty, going into configuration mode");
    configMode = true;
  } else if (digitalRead(buttonPin) == LOW) {
    delay(500);
    if (digitalRead(buttonPin) == LOW) {
      Serial.println("Button pressed, going into configuration mode");
      configMode = true;
    }
  }

  if (configMode) {
    configuration_mode();
  } else {
    Serial.println("Entering run mode");
    Serial.print("Pixels: ");
    Serial.println(eepromData.pixelcount);
    run_mode();
  }
}

void loop() {
  static boolean waitingForWiFi = true;

  if (eepromData.wifimode != 1 && waitingForWiFi) {
    if (WiFi.status() == WL_CONNECTED) {
      ip = WiFi.localIP();
      if ( mdns.begin ( "WS2812", WiFi.localIP() ) ) {
        mdns.addService("http", "tcp", 80);
        Serial.println ( "MDNS responder started" );
      }
      Serial.print("WiFi ready: SSID=");
      Serial.print(eepromData.ssid);
      Serial.print(" IP=");
      Serial.print(ip);
      Serial.print(" HOSTNAME=");
      Serial.println(myhostname);
      waitingForWiFi = false;
    }
  }

  udpLoop();
  tcpLoop();

  // give time to the ESP8266 WiFi stack
  yield();

  ledLoop();

  uint8_t newButtonState = digitalRead(buttonPin);
  if ((newButtonState == LOW) && (buttonState == HIGH)) {
    ++ledMode;
    ledModeChanged = true;
    Serial.print("Button pressed, mode=");
    Serial.print(ledMode, DEC);
    if (modeNames[ledMode]) {
      Serial.print(" ");
      Serial.println(modeNames[ledMode]);
    } else {
      Serial.println();
    }
    buttonState = LOW;
  } else if ((newButtonState == HIGH) && (buttonState == LOW)) {
    buttonState = HIGH;
  }

  // give time to the ESP8266 WiFi stack
  yield();
}

void udpMessageHandler(int len) {
  switch (inboundMessage[0]) {
    case 0x01: {
        // static colour: r, g, b
        singleColour(inboundMessage[1], inboundMessage[2], inboundMessage[3]);
        ledMode = 255;
        break;
      }
    case 0x02:
      // preset mode: n
      ledMode = inboundMessage[1];
      ledModeChanged = true;
      break;
    case 0x03: {
        // full sequence: r, g, b, r, g, b, ...
        uint8_t red, green, blue;
        uint16_t pos = 1;
        uint16_t pixel = 0;
        while ((pos + 2) < len) {
          red = inboundMessage[pos];
          green = inboundMessage[pos + 1];
          blue = inboundMessage[pos + 2];
          pixels.SetPixelColor(pixel, RgbColor(red, green, blue));
          pixel++;
          pos += 3;
        }
        pixels.Show();
        ledMode = 255;
        break;
      }
  }
}

void udpLoop() {
  unsigned int packetSize;
  IPAddress remoteIp;
  int remotePort;
  int len = 0;

  packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    remoteIp = Udp.remoteIP();
    remotePort = Udp.remotePort();

#ifdef DEBUG_PACKETS
    Serial.print("Packet received size=");
    Serial.print(packetSize);
    Serial.print(" from=");
    Serial.print(remoteIp);
    Serial.print(":");
    Serial.print(remotePort);
#endif

    len = Udp.read(inboundMessage, sizeof(inboundMessage));

#ifdef DEBUG_PACKETS
    if (len > 0) {
      Serial.print(" data=");
      for (int i = 0; i < len; i++) {
        if (inboundMessage[i] < 16) {
          Serial.print('0');
        }
        Serial.print(inboundMessage[i], HEX);
        Serial.print(' ');
      }
    }
    Serial.println();
#endif
    udpMessageHandler(len);
  }
}

void tcpLoop() {
  server.handleClient();
}

void makeScale() {
  for (int i = 0; i < 50; i++) {
    scalered[i] = eepromData.scale0red;
    scalegreen[i] = eepromData.scale0green;
    scaleblue[i] = eepromData.scale0blue;
  }

  for (int i = 50; i < 100; i++) {
    scalered[i] = eepromData.scale50red;
    scalegreen[i] = eepromData.scale50green;
    scaleblue[i] = eepromData.scale50blue;
  }

  for (int i = 100; i < MAX_PIXELS; i++) {
    scalered[i] = eepromData.scale100red;
    scalegreen[i] = eepromData.scale100green;
    scaleblue[i] = eepromData.scale100blue;
  }
}

void singleColour(uint8_t red, uint8_t green, uint8_t blue) {

  for (int i = 0; i < eepromData.pixelcount; i++) {
    pixels.SetPixelColor(i, RgbColor(red * scalered[i] / 255,
                                     green * scalegreen[i] / 255,
                                     blue * scaleblue[i] / 255));
  }
  pixels.Show();
}

void redNightLight() {
  static uint8_t level = 255;
  static unsigned long lastChange = 0;
  const uint8_t minimum = 63;
  const unsigned long runtime = 1800000;
  unsigned long interval = runtime / (255 - minimum);

  if (ledModeChanged) {
    level = 255;
    singleColour(level, 0, 0);
    ledModeChanged = false;
    lastChange = millis();
  } else {
    if (millis() - lastChange > interval) {
      if (level > minimum) {
        level = level - 1;
      }
      singleColour(level, 0, 0);
      lastChange = millis();
    }
  }
}

// Convert a given HSV (Hue Saturation Value) to RGB(Red Green Blue) and set
// the led to the color
//  h is hue value, integer between 0 and 360
//  s is saturation value, double between 0 and 1
//  v is value, double between 0 and 1
// http://splinter.com.au/blog/?p=29
RgbColor ledHSV(int h, double s, double v, int pos) {
  double r = 0;
  double g = 0;
  double b = 0;

  double hf = h / 60.0;

  int i = floor(hf);
  double f = hf - i;
  double pv = v * (1 - s);
  double qv = v * (1 - s * f);
  double tv = v * (1 - s * (1 - f));

  switch (i) {
    case 0:
      r = v;
      g = tv;
      b = pv;
      break;
    case 1:
      r = qv;
      g = v;
      b = pv;
      break;
    case 2:
      r = pv;
      g = v;
      b = tv;
      break;
    case 3:
      r = pv;
      g = qv;
      b = v;
      break;
    case 4:
      r = tv;
      g = pv;
      b = v;
      break;
    case 5:
      r = v;
      g = pv;
      b = qv;
      break;
  }

  // set each component to a integer value between 0 and 255
  int red = constrain((int)255 * r, 0, 255);
  int green = constrain((int)255 * g, 0, 255);
  int blue = constrain((int)255 * b, 0, 255);

  return RgbColor(red * scalered[pos] / 255,
                  green * scalegreen[pos] / 255,
                  blue * scaleblue[pos] / 255);
}

RgbColor ledExpHSV(int h, double s, double v, int pos) {
  if (h < (HUE_EXP_TIMES * HUE_EXP_RANGE))
    return ledHSV(h / HUE_EXP_TIMES, s, v, pos);
  else
    return ledHSV(h - ((HUE_EXP_TIMES - 1) * HUE_EXP_RANGE), s, v, pos);
}

void hsvFade() {
  static int hue = 0;
  static unsigned long lastChange = 0;
  unsigned long interval = 50;

  if (millis() - lastChange > interval) {
    for (int i = 0; i < eepromData.pixelcount; i++) {
      pixels.SetPixelColor(i, ledExpHSV(hue, 1.0, 1.0, i));
    }
    pixels.Show();
    hue++;
    hue %= HUE_EXP_MAX;
    lastChange = millis();
  }
}

void randomWipe() {
  static int hue=0;
  static unsigned long lastChange = 0;
  unsigned long interval = 1000;
  unsigned long transition = 10;
  static unsigned long i = 0;

  transition = interval / eepromData.pixelcount;
  if (millis() - lastChange > transition) {
    if (i >= eepromData.pixelcount) {
      i = 0;
    }
    if (i == 0) {
      hue = random(0, HUE_MAX);
    }
    pixels.SetPixelColor(i, ledHSV(hue, 1.0, 1.0, i));
    pixels.Show();
    i++;
    lastChange = millis();
  }
  
}

boolean hsvFadeNoShow() {
  static int hue = 0;
  static unsigned long lastChange = 0;
  unsigned long interval = 50;

  for (int i = 0; i < eepromData.pixelcount; i++) {
    pixels.SetPixelColor(i, ledExpHSV(hue, 1.0, 1.0, i));
  }

  if (millis() - lastChange > interval) {
    hue++;
    hue %= HUE_EXP_MAX;
    lastChange = millis();
    return true;
  }
  return false;
}

boolean hsvStaticNoShow() {
  for (int i = 0; i < eepromData.pixelcount; i++) {
    pixels.SetPixelColor(i, ledExpHSV(i * HUE_EXP_MAX / eepromData.pixelcount, 1.0, 1.0, i));
  }
  return false;
}

void hsvStatic() {
  hsvStaticNoShow();
  pixels.Show();
}

void rave() {
  static int hue = 0;
  static unsigned long lastChange = 0;
  unsigned long interval = 100;

  if (millis() - lastChange > interval) {
    hue = random(0, HUE_MAX);
    for (int i = 0; i < eepromData.pixelcount; i++) {
      pixels.SetPixelColor(i, ledHSV(hue, 1.0, 1.0, i));
    }
    pixels.Show();
    lastChange = millis();
  }
}

void hsvScroll() {
  static int hue = 0;
  static unsigned long lastChange = 0;
  unsigned long interval = 50;

  if (millis() - lastChange > interval) {
    for (int i = 0; i < eepromData.pixelcount; i++) {
      pixels.SetPixelColor(i, ledExpHSV(((i * HUE_EXP_MAX / eepromData.pixelcount) + hue) % HUE_EXP_MAX, 1.0, 1.0, i));
    }
    pixels.Show();
    hue++;
    hue %= HUE_EXP_MAX;
    lastChange = millis();
  }
}

void hsvFastScroll() {
  static int hue = 0;
  static unsigned long lastChange = 0;
  unsigned long interval = 1;

  if (millis() - lastChange > interval) {
    for (int i = 0; i < eepromData.pixelcount; i++) {
      pixels.SetPixelColor(i, ledExpHSV(((i * HUE_EXP_MAX / eepromData.pixelcount) + hue) % HUE_EXP_MAX, 1.0, 1.0, i));
    }
    pixels.Show();
    hue++;
    hue %= HUE_EXP_MAX;
    lastChange = millis();
  }
}

boolean hsvScrollNoShow() {
  static int hue = 0;
  static unsigned long lastChange = 0;
  unsigned long interval = 50;

  for (int i = 0; i < eepromData.pixelcount; i++) {
    pixels.SetPixelColor(i, ledExpHSV(((i * HUE_EXP_MAX / eepromData.pixelcount) + hue) % HUE_EXP_MAX, 1.0, 1.0, i));
  }

  if (millis() - lastChange > interval) {
    hue++;
    hue %= HUE_EXP_MAX;
    lastChange = millis();
    return true;
  }
  return false;
}

// Flames based on FastLED example code Fire2012 by Mark Kriegsman, July 2012 from
// https://github.com/FastLED/FastLED/blob/master/examples/Fire2012/Fire2012.ino
void flames() {
  // COOLING: How much does the air cool as it rises?
  // Less cooling = taller flames.  More cooling = shorter flames.
  // Default 50, suggested range 20-100
#define COOLING  55

  // SPARKING: What chance (out of 255) is there that a new spark will be lit?
  // Higher chance = more roaring fire.  Lower chance = more flickery fire.
  // Default 120, suggested range 50-200.
#define SPARKING 120

  static byte heat[255];
  static unsigned long lastChange = 0;
  unsigned long interval = 16;
  int randomnum;


  if (millis() - lastChange > interval) {
    // Step 1.  Cool down every cell a little
    for (int i = 0; i < eepromData.pixelcount; i++) {
      randomnum = heat[i] - random(0, ((COOLING * 10) / eepromData.pixelcount) + 2);
      heat[i] = max(randomnum, 0 );
    }
    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for ( int k = eepromData.pixelcount - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
    }
    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
    if ( random(255) < SPARKING ) {
      int y = random(7);
      heat[y] = min((heat[y] + random(160, 255)), 0xFF);
    }
    for ( int j = 0; j < eepromData.pixelcount; j++) {
      pixels.SetPixelColor(j, ledTemp(heat[j],j));
    }
    pixels.Show();
    lastChange = millis();
  }
}

RgbColor ledTemp(int temperature, int i) {
  double r = 0;
  double g = 0;
  double b = 0;

  // Scale 'heat' down from 0-255 to 0-191,
  // which can then be easily divided into three
  // equal 'thirds' of 64 units each.
  uint8_t t192 = map(temperature, 0, 255, 0, 192);

  // calculate a value that ramps up from
  // zero to 255 in each 'third' of the scale.
  uint8_t heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252

  // now figure out which third of the spectrum we're in:
  if ( t192 & 0x80) {
    // we're in the hottest third
    r = 255; // full red
    g = 255; // full green
    b = heatramp; // ramp up blue

  } else if ( t192 & 0x40 ) {
    // we're in the middle third
    r = 255; // full red
    g = heatramp; // ramp up green
    b = 0; // no blue

  } else {
    // we're in the coolest third
    r = heatramp; // ramp up red
    g = 0; // no green
    b = 0; // no blue
  }

  // set each component to a integer value between 0 and 255
  int red = constrain((int)255 * r, 0, 255);
  int green = constrain((int)255 * g, 0, 255);
  int blue = constrain((int)255 * b, 0, 255);

  return RgbColor(red * scalered[i] / 255,
                      green * scalegreen[i] / 255,
                      blue * scaleblue[i] / 255);
}

static RgbColor christmasRGValues[MAX_PIXELS];

boolean christmasRedAndGreen(boolean show) {
  static unsigned long lastChange = 0;
  unsigned long interval = show ? 200 : 1000;

  if (ledModeChanged) {
    // randomise all of the pixels
    for (int i = 0; i < eepromData.pixelcount; i++) {
      const RgbColor red = RgbColor(scalered[i], 0, 0);
      const RgbColor green = RgbColor(0, scalegreen[i], 0);

      if (random(0, 2) == 0) {
        christmasRGValues[i] = red;
        pixels.SetPixelColor(i, red);
      } else {
        christmasRGValues[i] = green;
        pixels.SetPixelColor(i, green);
      }
    }
    if (show)
      pixels.Show();
    ledModeChanged = false;
    lastChange = millis();
    return true;
  }

  if (millis() - lastChange > interval) {
    int i = random(0, eepromData.pixelcount);
    const RgbColor red = RgbColor(scalered[i], 0, 0);
    const RgbColor green = RgbColor(0, scalegreen[i], 0);

    if (christmasRGValues[i] == green) {
      christmasRGValues[i] = red;
      if (show)
        pixels.SetPixelColor(i, red);
    } else {
      christmasRGValues[i] = green;
      if (show)
        pixels.SetPixelColor(i, green);
    }
    if (show) {
      pixels.Show();
    } else {
      for (int i = 0; i < eepromData.pixelcount; i++)
        pixels.SetPixelColor(i, christmasRGValues[i]);
    }
    lastChange = millis();
    return true;
  }

  if (!show)
    for (int i = 0; i < eepromData.pixelcount; i++)
      pixels.SetPixelColor(i, christmasRGValues[i]);
  return false;
}

void christmasRedAndGreen() {
  christmasRedAndGreen(true);
}

boolean christmasRedAndGreenNoShow() {
  return christmasRedAndGreen(false);
}

uint8_t applyTwinkleLevelC(uint8_t value, uint8_t level, uint8_t scale) {
  level = (255 - level);
  value = (value * 255 / scale);
  if (value >= level) {
    value = (value - level) * scale / 255;
  } else {
    value = 0;
  }
  return value;
}

void applyTwinkleLevel(int i, uint8_t level) {
  RgbColor tmp;
  HsbColor hsb;

  tmp = pixels.GetPixelColor(i);

#if 0
  tmp.R = applyTwinkleLevelC(tmp.R, level, scalered[i]);
  tmp.G = applyTwinkleLevelC(tmp.G, level, scalegreen[i]);
  tmp.B = applyTwinkleLevelC(tmp.B, level, scaleblue[i]);
#endif

  hsb = HsbColor(tmp);
  hsb.B *= level / 255.0f;
  tmp = RgbColor(hsb);

  pixels.SetPixelColor(i, tmp);
}

void twinkle(boolean (*func)(void), uint8_t low, uint8_t high, uint8_t step, boolean invert) {
  static unsigned long lastChange = 0;
  static unsigned long lastPulse = 0;
  static uint8_t levels[MAX_PIXELS];
  static uint8_t current[MAX_PIXELS];
  unsigned long pulseInterval = 250;
  unsigned long changeInterval = 10;
  boolean refresh = false;

  if (ledModeChanged) {
    for (int i = 0; i < eepromData.pixelcount; i++) {
      levels[i] = invert ? high : low;
      current[i] = 0;
    }
    func();
    refresh = true;
    lastChange = millis();
    lastPulse = millis();
    ledModeChanged = false;
  }

  if (millis() - lastPulse > pulseInterval) {
    int target = random(0, eepromData.pixelcount);
    if (current[target] == 0 && levels[target] == (invert ? high : low)) {
      current[target] = 1;
    }
    lastPulse = millis();
  }

  if (millis() - lastChange > changeInterval) {
    for (int i = 0; i < eepromData.pixelcount; i++) {
      if (current[i] == 1) {
        // this pixel is rising
        if (invert ? (levels[i] - step < low) : (levels[i] + step > high)) {
          // the pixel has reached maximum
          levels[i] = invert ? low : high;
          current[i] = 0;
        } else {
          levels[i] = invert ? (levels[i] - step) : (levels[i] + step);
        }
      } else if (invert ? (levels[i] < high) : (levels[i] > low)) {
        // this pixel is falling
        if (invert ? (levels[i] + step > high) : (levels[i] - step < low)) {
          levels[i] = invert ? high : low;
        } else {
          levels[i] = invert ? (levels[i] + step) : (levels[i] - step);
        }
      }
    }
    lastChange = millis();
    refresh = true;
  }

  if (func() || refresh) {
    for (int i = 0; i < eepromData.pixelcount; i++)
      applyTwinkleLevel(i, levels[i]);
    pixels.Show();
  }
}

void christmasRedAndGreenTwinkle() {
  twinkle(christmasRedAndGreenNoShow, 50, 255, 10, true);
}

boolean whiteNoShow() {
  for (int i = 0; i < eepromData.pixelcount; i++) {
    pixels.SetPixelColor(i, RgbColor(scalered[i],
                                     scalegreen[i],
                                     scaleblue[i]));
  }
  return false;
}

void whiteTwinkle() {
  twinkle(whiteNoShow, 50, 255, 10, false);
}

void hsvScrollTwinkle() {
  twinkle(hsvScrollNoShow, 50, 255, 10, true);
}

void hsvStaticTwinkle() {
  twinkle(hsvStaticNoShow, 50, 255, 10, true);
}

void hsvFadeTwinkle() {
  twinkle(hsvFadeNoShow, 50, 255, 10, true);
}

static RgbColor christmasWorkValues[MAX_PIXELS];

boolean christmasWork(boolean show) {
  static unsigned long lastChange = 0;
  unsigned long interval = show ? 200 : 1000;

  if (ledModeChanged) {
    // randomise all of the pixels
    for (int i = 0; i < eepromData.pixelcount; i++) {
      const RgbColor grey = RgbColor(74 * scalered[i] / 255, 79 * scalegreen[i] / 255, 85 * scaleblue[i] / 255);
      const RgbColor red = RgbColor(194 * scalered[i] / 255, 4 * scalegreen[i] / 255, 24 * scaleblue[i] / 255);
      const RgbColor lightBlue = RgbColor(0, 195 * scalegreen[i] / 255, 215 * scaleblue[i] / 255);
      const RgbColor darkBlue = RgbColor(0, 51 * scalegreen[i] / 255, 161 * scaleblue[i] / 255);
      const RgbColor colours[4] = { grey, red, lightBlue, darkBlue };
      const int num_colours = sizeof(colours) / sizeof(RgbColor);

      christmasWorkValues[i] = colours[random(0, num_colours)];
      pixels.SetPixelColor(i, christmasWorkValues[i]);
    }
    if (show)
      pixels.Show();
    ledModeChanged = false;
    lastChange = millis();
    return true;
  }

  if (millis() - lastChange > interval) {
    int i = random(0, eepromData.pixelcount);
    const RgbColor grey = RgbColor(74 * scalered[i] / 255, 79 * scalegreen[i] / 255, 85 * scaleblue[i] / 255);
    const RgbColor red = RgbColor(194 * scalered[i] / 255, 4 * scalegreen[i] / 255, 24 * scaleblue[i] / 255);
    const RgbColor lightBlue = RgbColor(0, 195 * scalegreen[i] / 255, 215 * scaleblue[i] / 255);
    const RgbColor darkBlue = RgbColor(0, 51 * scalegreen[i] / 255, 161 * scaleblue[i] / 255);
    const RgbColor colours[4] = { grey, red, lightBlue, darkBlue };
    const int num_colours = sizeof(colours) / sizeof(RgbColor);
    int n = random(1, num_colours);

    for (int j = 0; j < num_colours; j++) {
      if (christmasWorkValues[i] == colours[j]) {
        christmasWorkValues[i] = colours[(j + n) % num_colours];
        if (show)
          pixels.SetPixelColor(i, christmasWorkValues[i]);
        break;
      }
    }
    if (show) {
      pixels.Show();
    } else {
      for (int i = 0; i < eepromData.pixelcount; i++)
        pixels.SetPixelColor(i, christmasWorkValues[i]);
    }
    lastChange = millis();
    return true;
  }

  if (!show)
    for (int i = 0; i < eepromData.pixelcount; i++)
      pixels.SetPixelColor(i, christmasWorkValues[i]);
  return false;
}

void christmasWork() {
  christmasWork(true);
}

boolean christmasWorkNoShow() {
  return christmasWork(false);
}

void christmasWorkTwinkle() {
  twinkle(christmasWorkNoShow, 50, 255, 10, true);
}

void knightRider(boolean hsvFade) {
  static unsigned long lastChange = 0;
  static int pos = 0;
  static int dir = 1;
  unsigned long interval = 600 / eepromData.pixelcount;
  const uint8_t active = 12; // percentage fully bright
  const uint8_t fade = 16; // percentage to fade off at each end
  const float fadeRate = 0.75f;
  int pActive = eepromData.pixelcount * active / 100 / 2;
  int pFade = eepromData.pixelcount * fade / 100;

  static int hue = 0;
  static unsigned long lastHueChange = 0;
  unsigned long hueInterval = 50;

  boolean change = false;
  boolean refresh = false;

  if (pActive < 1)
    pActive = 1;
  if (pFade < 1)
    pFade = 1;

  if (ledModeChanged) {
    pos = pActive;
    dir = 1;
    lastChange = millis() - interval - 1;

    hue = 0;
    lastHueChange = millis();

    ledModeChanged = false;
  }

  if (hsvFade) {
    if (millis() - lastHueChange > hueInterval) {
      hue++;
      hue %= HUE_EXP_MAX;
      lastHueChange = millis();
      refresh = true;
    }
  }

  if (millis() - lastChange > interval) {
    change = true;
    refresh = true;
  }

  if (refresh) {
    for (int i = 0; i < eepromData.pixelcount; i++) {
      RgbColor base;
      HsbColor tmp;

      if (hsvFade) {
        base = ledExpHSV(hue, 1.0, 1.0, i);
      } else {
        base = RgbColor(scalered[i], 16 * scalegreen[i] / 255, 0);
      }

      tmp = HsbColor(base);

      if (i >= pos - pActive && i <= pos + pActive) {
        // full brightness
      } else if (i >= pos - pActive - pFade && i <= pos + pActive + pFade) {
        if (i < pos) {
          //tmp.B *= (float)(pFade - ((pos - pActive) - i)) / (float)pFade * 0.5f;
          tmp.B *= 0.5f;
          for (int j = ((pos - pActive) - i); j > 0; j--)
            tmp.B *= fadeRate;
        } else {
          //tmp.B *= (float)(pFade - (i - (pos + pActive))) / (float)pFade * 0.5f;
          tmp.B *= 0.5f;
          for (int j = (i - (pos + pActive)); j > 0; j--)
            tmp.B *= fadeRate;
        }
      } else {
        tmp.B = 0;
      }

      pixels.SetPixelColor(i, tmp);
    }

    pixels.Show();
  }

  if (change) {
    pos += dir;
    if (pos > eepromData.pixelcount - pActive) {
      dir = -1;
      pos--;
    } else if (pos < pActive) {
      dir = 1;
      pos++;
    }
    lastChange = millis();
  }
}

RgbColor makeRandom1(int pos) {
  RgbColor tmp = RgbColor(HslColor(random(0, 256) / 255.0f, random(128, 256) / 255.0f, random(64, 128) / 255.0f));
  tmp.R = tmp.R * scalered[pos] / 255;
  tmp.G = tmp.G * scalegreen[pos] / 255;
  tmp.B = tmp.B * scaleblue[pos] / 255;
  return tmp;
}

RgbColor makeRandom2(int pos) {
  return ledExpHSV(random(0, HUE_EXP_MAX), 1.0, 1.0, pos);
}

void random_single(RgbColor (*makeRandom)(int), unsigned long interval) {
  static unsigned long lastChange = 0;

  if (ledModeChanged) {
    // randomise all of the pixels
    for (int i = 0; i < eepromData.pixelcount; i++)
      pixels.SetPixelColor(i, makeRandom(i));
    pixels.Show();
    ledModeChanged = false;
  }

  if (millis() - lastChange > interval) {
    int i = random(0, eepromData.pixelcount);
    pixels.SetPixelColor(i, makeRandom(i));
    pixels.Show();
    lastChange = millis();
  }
}

void random_full(RgbColor (*makeRandom)(int), unsigned long interval) {
  static unsigned long lastChange = 0;

  if (ledModeChanged) {
    // randomise all of the pixels
    for (int i = 0; i < eepromData.pixelcount; i++)
      pixels.SetPixelColor(i, makeRandom(i));
    pixels.Show();
    ledModeChanged = false;
  }

  if (millis() - lastChange > interval) {
    for (int i = 0; i < eepromData.pixelcount; i++) {
      pixels.SetPixelColor(i, makeRandom(i));
    }
    pixels.Show();
    lastChange = millis();
  }
}


void ledLoop() {

  switch (ledMode) {
    case 0:
      // black
      singleColour(0, 0, 0);
      break;
    case 1:
      // red
      singleColour(255, 0, 0);
      break;
    case 2:
      // dim red
      singleColour(63, 0, 0);
      break;
    case 3:
      // green
      singleColour(0, 255, 0);
      break;
    case 4:
      // yellow
      singleColour(255, 255, 0);
      break;
    case 5:
      // blue
      singleColour(0, 0, 255);
      break;
    case 6:
      // magenta
      singleColour(255, 0, 255);
      break;
    case 7:
      // cyan
      singleColour(0, 255, 255);
      break;
    case 8:
      // white
      singleColour(255, 255, 255);
      break;
    case 9:
      hsvScroll();
      break;
    case 10:
      hsvFade();
      break;
    case 11:
      christmasRedAndGreen();
      break;
    case 12:
      christmasRedAndGreenTwinkle();
      break;
    case 13:
      whiteTwinkle();
      break;
    case 14:
      redNightLight();
      break;
    case 15:
      christmasWork();
      break;
    case 16:
      christmasWorkTwinkle();
      break;
    case 17:
      hsvScrollTwinkle();
      break;
    case 18:
      hsvStaticTwinkle();
      break;
    case 19:
      hsvFadeTwinkle();
      break;
    case 20:
      knightRider(false);
      break;
    case 21:
      knightRider(true);
      break;
    case 22:
      random_single(makeRandom1, 200);
      break;
    case 23:
      random_single(makeRandom1, 50);
      break;
    case 24:
      random_full(makeRandom1, 500);
      break;
    case 25:
      random_full(makeRandom1, 200);
      break;
    case 26:
      random_single(makeRandom2, 200);
      break;
    case 27:
      random_single(makeRandom2, 50);
      break;
    case 28:
      random_full(makeRandom2, 500);
      break;
    case 29:
      random_full(makeRandom2, 200);
      break;
    case 30:
      flames();
      break;
    case 31:
      hsvFastScroll();
      break;
    case 32:
      randomWipe();
      break;
    case 33:
      rave();
      break;
    case 255:
      // network mode - no action
      break;
    default:
      // default to black
      singleColour(0, 0, 0);
      ledMode = 0;
  }
}

void runRootHandler() {
  int i;

  String form;

  form.reserve(2560);
  form += "<!DOCTYPE html>"
          "<html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
          "<style type=\"text/css\">"
          "html, body, form { padding: 0; margin: 0; }"
          "input {"
          "border: none; background-color: #99f; padding: 1em 0 1em 0;"
          "border-bottom: 2px solid #fff;"
          "margin: 0;"
          "width: 100%;"
          "font-weight: bold;"
          "}"
          "input:active {"
          "background-color: #f99;"
          "}"
          "#selected {"
          "background-color: #9f9;"
          "}"
          "</style>"
          "<script>"
          "Element.prototype.documentOffsetTop = function () {"
          "return this.offsetTop + (this.offsetParent ? this.offsetParent.documentOffsetTop() : 0);"
          "};"
          "window.onload = function() {"
          "var selected = document.getElementById('selected');"
          "if (selected) {"
          "var top = selected.documentOffsetTop() - (window.innerHeight / 2);"
          "window.scrollTo(0, top);"
          "}"
          "};"
          "</script>"
          "</head><body>"
          "<form method=\"POST\" action=\"apply\">";

  for (uint8_t i = 0; i < server.args(); i++) {
    if (server.argName(i) == "default") {
      form += "<input type=\"hidden\" name=\"default\" value=\"1\"/>";
    }
  }

  for (i = 0; modeNames[i] != NULL; i++) {
    //if (i == 2 || i == 13) continue;
    form += "<input type=\"submit\" name=\"ledmode_";
    form += i;
    form += "\"";
    if (ledMode == i)
      form += " id=\"selected\"";
    form += " value=\"";
    form += modeNames[i];
    form += "\"><br>";
  }

  form += "</form>";

  server.send(200, "text/html", form);
}

void runUpdateHandler() {
  int setDefault = 0;

  for (uint8_t i = 0; i < server.args(); i++) {
    if (server.argName(i) == "default") {
      setDefault = 1;
    }

    if (server.argName(i).startsWith("ledmode_")) {
      ledMode = server.argName(i).substring(8).toInt();
      ledModeChanged = true;
      Serial.print("Updated mode=");
      Serial.println(ledMode, DEC);
    }
  }

  if (setDefault) {
    eepromData.defaultmode = ledMode;
    Serial.print("Updated default mode=");
    Serial.println(ledMode, DEC);
    EEPROM.put(0, eepromData);
    EEPROM.commit();

    server.send(200, "text/html", "<!DOCTYPE html><head><meta http-equiv=\"refresh\" content=\"2;URL=/\"></head><p>Updated default</p>");
  } else {
    String redirect;
    redirect.reserve(128);
    redirect += "HTTP/1.1 302 Updated\r\nLocation: http://";
    if (eepromData.wifimode == 1) {
      redirect += WiFi.softAPIP().toString();
    } else {
      redirect += WiFi.localIP().toString();
    }
    redirect += "/\r\nConnection: close\r\nContent-Length: 0\r\n\r\n";
    server.sendContent(redirect);
  }
}

void runConfigHandler() {
  String form;

  form.reserve(1024);
  form += "<!DOCTYPE html>"
          "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"></head>"
          "<form method=\"POST\" action=\"apply2\">";

  form += "Scaling (0-49): "
          "R<input type=\"number\" name=\"scale0red\" min=\"0\" max=\"255\" value=\"";
  form += eepromData.scale0red;
  form += "\"/>"
          "G<input type=\"number\" name=\"scale0green\" min=\"0\" max=\"255\" value=\"";
  form += eepromData.scale0green;
  form += "\"/>"
          "B<input type=\"number\" name=\"scale0blue\" min=\"0\" max=\"255\" value=\"";
  form += eepromData.scale0blue;
  form += "\"/><br/>";

  form += "Scaling (50-99): "
          "R<input type=\"number\" name=\"scale50red\" min=\"0\" max=\"255\" value=\"";
  form += eepromData.scale50red;
  form += "\"/>"
          "G<input type=\"number\" name=\"scale50green\" min=\"0\" max=\"255\" value=\"";
  form += eepromData.scale50green;
  form += "\"/>"
          "B<input type=\"number\" name=\"scale50blue\" min=\"0\" max=\"255\" value=\"";
  form += eepromData.scale50blue;
  form += "\"/><br/>";

  form += "Scaling (100-149): "
          "R<input type=\"number\" name=\"scale100red\" min=\"0\" max=\"255\" value=\"";
  form += eepromData.scale100red;
  form += "\"/>"
          "G<input type=\"number\" name=\"scale100green\" min=\"0\" max=\"255\" value=\"";
  form += eepromData.scale100green;
  form += "\"/>"
          "B<input type=\"number\" name=\"scale100blue\" min=\"0\" max=\"255\" value=\"";
  form += eepromData.scale100blue;
  form += "\"/><br/>";

  form += "Default Mode: <input type=\"number\" name=\"defaultmode\" min=\"0\" max=\"255\" value=\"";
  form += eepromData.defaultmode;
  form += "\"/><br/>";
  form += "<input type=\"submit\" /></form>";

  server.send(200, "text/html", form);
}

void runConfigUpdateHandler() {
  for (uint8_t i = 0; i < server.args(); i++) {
    if (server.argName(i) == "scale0red") {
      eepromData.scale0red = server.arg(i).toInt();
    }
    if (server.argName(i) == "scale0green") {
      eepromData.scale0green = server.arg(i).toInt();
    }
    if (server.argName(i) == "scale0blue") {
      eepromData.scale0blue = server.arg(i).toInt();
    }
    if (server.argName(i) == "scale50red") {
      eepromData.scale50red = server.arg(i).toInt();
    }
    if (server.argName(i) == "scale50green") {
      eepromData.scale50green = server.arg(i).toInt();
    }
    if (server.argName(i) == "scale50blue") {
      eepromData.scale50blue = server.arg(i).toInt();
    }
    if (server.argName(i) == "scale100red") {
      eepromData.scale100red = server.arg(i).toInt();
    }
    if (server.argName(i) == "scale100green") {
      eepromData.scale100green = server.arg(i).toInt();
    }
    if (server.argName(i) == "scale100blue") {
      eepromData.scale100blue = server.arg(i).toInt();
    }
    if (server.argName(i) == "defaultmode") {
      eepromData.defaultmode = server.arg(i).toInt();
    }
  }
  makeScale();
  ledModeChanged = true;
  Serial.println("Updated config:");
  Serial.print("scale0red=");
  Serial.println(eepromData.scale0red, DEC);
  Serial.print("scale0green=");
  Serial.println(eepromData.scale0green, DEC);
  Serial.print("scale0blue=");
  Serial.println(eepromData.scale0blue, DEC);
  Serial.print("scale50red=");
  Serial.println(eepromData.scale50red, DEC);
  Serial.print("scale50green=");
  Serial.println(eepromData.scale50green, DEC);
  Serial.print("scale50blue=");
  Serial.println(eepromData.scale50blue, DEC);
  Serial.print("scale100red=");
  Serial.println(eepromData.scale100red, DEC);
  Serial.print("scale100green=");
  Serial.println(eepromData.scale100green, DEC);
  Serial.print("scale100blue=");
  Serial.println(eepromData.scale100blue, DEC);
  Serial.print("defaultmode=");
  Serial.println(eepromData.defaultmode, DEC);
  EEPROM.put(0, eepromData);
  EEPROM.commit();

  server.send(200, "text/html", "<p>Settings updated</p>");
}

void runUptimeHandler() {
  unsigned long uptime = millis();
  unsigned long days, hours, minutes, seconds, ms;
  char response[64];

  days = uptime / 86400000;
  uptime %= 86400000;

  hours = uptime / 3600000;
  uptime %= 3600000;

  minutes = uptime / 60000;
  uptime %= 60000;

  seconds = uptime / 1000;
  uptime %= 1000;

  ms = uptime;

  snprintf(response, sizeof(response), "%03ld+%02ld:%02ld:%02ld.%03ld\n", days, hours, minutes, seconds, ms);
  server.send(200, "text/plain", response);
}

void run_mode() {
  //eepromData.wifimode = 0;
  switch (eepromData.wifimode) {
    case 1:
      // go into access point mode
      WiFi.mode(WIFI_AP);
      if (strlen(eepromData.passphrase) > 0)
        WiFi.softAP(eepromData.ssid, eepromData.passphrase);
      else
        WiFi.softAP(eepromData.ssid);
      {
        uint8 mode = 0;
        wifi_softap_set_dhcps_offer_option(OFFER_ROUTER, &mode);
      }
      delay(100);
      ip = WiFi.softAPIP();

      // display access details
      Serial.print("WiFi AP: SSID=");
      Serial.print(eepromData.ssid);
      Serial.print(" URL=http://");
      Serial.print(ip);
      Serial.println("/");
      break;

    case 0:
    default:
      WiFi.mode(WIFI_STA);
      WiFi.begin(eepromData.ssid, eepromData.passphrase);
      break;
  }

  makeScale();
  pixels.setPixelCount(eepromData.pixelcount);
  pixels.setType(eepromData.colourorder | NEO_KHZ800 | NEO_IRQLOCK);
  pixels.Begin();

  Udp.begin(udpPort);
  server.on("/", runRootHandler);
  server.on("/apply", runUpdateHandler);
  server.on("/config", runConfigHandler);
  server.on("/apply2", runConfigUpdateHandler);
  server.on("/uptime", runUptimeHandler);
  server.begin();
  ledMode = eepromData.defaultmode;
  ledModeChanged = true;
  Serial.print("Ready, mode=");
  Serial.println(ledMode, DEC);
}

void configRootHandler() {
  int i;

  String form = "<!DOCTYPE html>"
                "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"></head>"
                "<form method=\"POST\" action=\"update\">SSID: <input type=\"text\" "
                "name=\"ssid\"";
  if (eepromData.configured == 1)
    form += " value=\"*\"";
  form += "/><br/>"
          "Passphrase: <input type=\"text\" name=\"passphrase\" ";
  if (eepromData.configured == 1)
    form += " value=\"*\"";
  form += "/><br/>";

  form += "WiFi Mode: <select name=\"wifimode\">";
  for (i = 0; wifiModeNames[i] != NULL; i++) {
    form += "<option value=\"";
    form += i;
    form += "\"";
    if (eepromData.configured == 1 && eepromData.wifimode == i)
      form += " selected";
    form += ">";
    form += wifiModeNames[i];
    form += "</option>";
  }
  form += "</select><br/>";

  form += "Pixel Count: <input type=\"number\" name=\"pixelcount\" min=\"1\"";
  if (eepromData.configured == 1) {
    form += " value=\"";
    form += eepromData.pixelcount;
    form += "\"";
  }
  form += "/><br/>";

  form += "Colour Order: <select name=\"colourorder\">";
  for (i = 0; colourOrderNames[i] != NULL; i++) {
    form += "<option value=\"";
    form += colourOrderValues[i];
    form += "\"";
    if (eepromData.configured == 1 && eepromData.colourorder == colourOrderValues[i])
      form += " selected";
    form += ">";
    form += colourOrderNames[i];
    form += "</option>";
  }
  form += "</select><br/>";

  form += "Scaling (0-49): "
          "R<input type=\"number\" name=\"scale0red\" min=\"0\" max=\"255\" value=\"";
  form += (eepromData.configured == 1 ? eepromData.scale0red : 255);
  form += "\"/>"
          "G<input type=\"number\" name=\"scale0green\" min=\"0\" max=\"255\" value=\"";
  form += (eepromData.configured == 1 ? eepromData.scale0green : 255);
  form += "\"/>"
          "B<input type=\"number\" name=\"scale0blue\" min=\"0\" max=\"255\" value=\"";
  form += (eepromData.configured == 1 ? eepromData.scale0blue : 255);
  form += "\"/><br/>";

  form += "Scaling (50-99): "
          "R<input type=\"number\" name=\"scale50red\" min=\"0\" max=\"255\" value=\"";
  form += (eepromData.configured == 1 ? eepromData.scale50red : 255);
  form += "\"/>"
          "G<input type=\"number\" name=\"scale50green\" min=\"0\" max=\"255\" value=\"";
  form += (eepromData.configured == 1 ? eepromData.scale50green : 255);
  form += "\"/>"
          "B<input type=\"number\" name=\"scale50blue\" min=\"0\" max=\"255\" value=\"";
  form += (eepromData.configured == 1 ? eepromData.scale50blue : 255);
  form += "\"/><br/>";

  form += "Scaling (100-149): "
          "R<input type=\"number\" name=\"scale100red\" min=\"0\" max=\"255\" value=\"";
  form += (eepromData.configured == 1 ? eepromData.scale100red : 255);
  form += "\"/>"
          "G<input type=\"number\" name=\"scalegreen\" min=\"0\" max=\"255\" value=\"";
  form += (eepromData.configured == 1 ? eepromData.scale100green : 255);
  form += "\"/>"
          "B<input type=\"number\" name=\"scaleblue\" min=\"0\" max=\"255\" value=\"";
  form += (eepromData.configured == 1 ? eepromData.scale100blue : 255);
  form += "\"/><br/>";

  form += "Default Mode: <input type=\"number\" name=\"defaultmode\" min=\"0\" max=\"255\" value=\"";
  if (eepromData.configured == 1) {
    form += eepromData.defaultmode;
  } else {
    form += ledMode;
  }
  form += "\"/><br/>";
  form += "<input type=\"submit\" /></form>";

  server.send(200, "text/html", form);
}

void configUpdateHandler() {

  for (uint8_t i = 0; i < server.args(); i++) {
    if (server.argName(i) == "ssid" && server.arg(i) != "*") {
      server.arg(i).toCharArray(eepromData.ssid, sizeof(eepromData.ssid));
    }
    if (server.argName(i) == "passphrase" && server.arg(i) != "*") {
      server.arg(i).toCharArray(eepromData.passphrase, sizeof(eepromData.passphrase));
    }
    if (server.argName(i) == "pixelcount") {
      eepromData.pixelcount = server.arg(i).toInt();
    }
    if (server.argName(i) == "colourorder") {
      eepromData.colourorder = server.arg(i).toInt();
    }
    if (server.argName(i) == "scale0red") {
      eepromData.scale0red = server.arg(i).toInt();
    }
    if (server.argName(i) == "scale0green") {
      eepromData.scale0green = server.arg(i).toInt();
    }
    if (server.argName(i) == "scale0blue") {
      eepromData.scale0blue = server.arg(i).toInt();
    }
    if (server.argName(i) == "scale50red") {
      eepromData.scale50red = server.arg(i).toInt();
    }
    if (server.argName(i) == "scale50green") {
      eepromData.scale50green = server.arg(i).toInt();
    }
    if (server.argName(i) == "scale50blue") {
      eepromData.scale50blue = server.arg(i).toInt();
    }
    if (server.argName(i) == "scale100red") {
      eepromData.scale100red = server.arg(i).toInt();
    }
    if (server.argName(i) == "scale100green") {
      eepromData.scale100green = server.arg(i).toInt();
    }
    if (server.argName(i) == "scale100blue") {
      eepromData.scale100blue = server.arg(i).toInt();
    }
    if (server.argName(i) == "defaultmode") {
      eepromData.defaultmode = server.arg(i).toInt();
    }
    if (server.argName(i) == "wifimode") {
      eepromData.wifimode = server.arg(i).toInt();
    }
  }
  eepromData.configured = 1;
  EEPROM.put(0, eepromData);
  EEPROM.commit();
  server.send(200, "text/html", "<p>Settings updated</p>");

  // set first three pixels to Red-Green-Blue using the updated configuration
  // set the last pixel to White
  makeScale();
  pixels.setPixelCount(eepromData.pixelcount);
  pixels.setType(eepromData.colourorder | NEO_KHZ800);
  pixels.Begin();
  singleColour(0, 0, 0);
  pixels.SetPixelColor(0, RgbColor(eepromData.scale0red, 0, 0));
  pixels.SetPixelColor(1, RgbColor(0, eepromData.scale0green, 0));
  pixels.SetPixelColor(2, RgbColor(0, 0, eepromData.scale0blue));
  pixels.SetPixelColor(eepromData.pixelcount - 1,
                       RgbColor(eepromData.scale0red, eepromData.scale0green, eepromData.scale0blue));
  pixels.Show();
}

void configuration_mode() {
  Serial.print("freeHeap=");
  Serial.println(ESP.getFreeHeap());
  Serial.print("chipId=");
  Serial.println(ESP.getChipId(), HEX);
  Serial.print("sdkVersion=");
  Serial.println(ESP.getSdkVersion());
  Serial.print("bootVersion=");
  Serial.println(ESP.getBootVersion());
  Serial.print("bootMode=");
  Serial.println(ESP.getBootMode());
  Serial.print("cpuFreqMHz=");
  Serial.println(ESP.getCpuFreqMHz());
  Serial.print("flashChipId=");
  Serial.println(ESP.getFlashChipId(), HEX);
  Serial.print("flashChipRealSize=");
  Serial.println(ESP.getFlashChipRealSize());
  Serial.print("flashChipSpeed=");
  Serial.println(ESP.getFlashChipSpeed());
  Serial.print("sketchSize=");
  Serial.println(ESP.getSketchSize());
  Serial.print("freeSketchSpace=");
  Serial.println(ESP.getFreeSketchSpace());
  Serial.println();

  Serial.print("ssid=");
  Serial.println(eepromData.ssid);
  Serial.print("passphrase=");
  Serial.println(eepromData.passphrase);
  Serial.print("pixelcount=");
  Serial.println(eepromData.pixelcount);
  Serial.print("colourorder=");
  Serial.println(eepromData.colourorder);
  Serial.print("scale0red=");
  Serial.println(eepromData.scale0red);
  Serial.print("scale0green=");
  Serial.println(eepromData.scale0green);
  Serial.print("scale0blue=");
  Serial.println(eepromData.scale0blue);
  Serial.print("scale50red=");
  Serial.println(eepromData.scale50red);
  Serial.print("scale50green=");
  Serial.println(eepromData.scale50green);
  Serial.print("scale50blue=");
  Serial.println(eepromData.scale50blue);
  Serial.print("scale100red=");
  Serial.println(eepromData.scale100red);
  Serial.print("scale100green=");
  Serial.println(eepromData.scale100green);
  Serial.print("scale100blue=");
  Serial.println(eepromData.scale100blue);
  Serial.print("defaultmode=");
  Serial.println(eepromData.defaultmode);
  Serial.print("wifimode=");
  Serial.println(eepromData.wifimode);
  Serial.println();

  // set first three pixels to dim white to acknowledge configuration mode
  pixels.setPixelCount(3);
  pixels.setType(NEO_RGB | NEO_KHZ800);
  pixels.Begin();
  pixels.SetPixelColor(0, RgbColor(63, 63, 63));
  pixels.SetPixelColor(1, RgbColor(63, 63, 63));
  pixels.SetPixelColor(2, RgbColor(63, 63, 63));
  pixels.Show();

  // go into access point mode
  WiFi.mode(WIFI_AP);
  WiFi.softAP(myhostname);
  {
    uint8 mode = 0;
    wifi_softap_set_dhcps_offer_option(OFFER_ROUTER, &mode);
  }
  delay(100);
  ip = WiFi.softAPIP();

  // display access details
  Serial.print("WiFi AP: SSID=");
  Serial.print(myhostname);
  Serial.print(" URL=http://");
  Serial.print(ip);
  Serial.println("/");

  // set first three pixels to Red-Green-Blue to indicate that configuration
  // mode AP is ready, and to help the user identify the correct colour order
  pixels.SetPixelColor(0, RgbColor(255, 0, 0));
  pixels.SetPixelColor(1, RgbColor(0, 255, 0));
  pixels.SetPixelColor(2, RgbColor(0, 0, 255));
  pixels.Show();

  server.on("/", configRootHandler);
  server.on("/update", configUpdateHandler);
  server.begin();

  while (1) {
    server.handleClient();
  }
}
