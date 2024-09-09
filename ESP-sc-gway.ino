
#include <sys/time.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <TimeLib.h>  // https://github.com/PaulStoffregen/Time)

extern "C" {
#include "user_interface.h"
#include "lwip/err.h"
#include "lwip/dns.h"
}

#include "gBase64.h"      // https://github.com/adamvr/arduino-base64 (I changed the name)
#include "ESP-sc-gway.h"  // This file contains configuration of GWay
// #include "RGBLed.h"				// Thid file is for onboard RGBLED of WeMos Lora Shield

int debug = 0;  // Debug level! 0 is no msgs, 1 normal, 2 is extensive

using namespace std;

uint8_t currentMode = 0x81;
char message[256];
char b64[256];
bool sx1272 = true;  // Actually we use sx1276/RFM95
uint8_t receivedbytes;

uint32_t cp_nb_rx_rcv;
uint32_t cp_nb_rx_ok;
uint32_t cp_nb_rx_bad;
uint32_t cp_nb_rx_nocrc;
uint32_t cp_up_pkt_fwd;

enum sf_t { SF7 = 7,
            SF8,
            SF9,
            SF10,
            SF11,
            SF12 };

uint8_t MAC_array[6];
char MAC_char[18];

// SX1276 - ESP8266 connections
int ssPin = DEFAULT_PIN_SS;
int dio0 = DEFAULT_PIN_DIO0;
int RST = DEFAULT_PIN_RST;

// Set spreading factor (SF7 - SF12)
sf_t sf = SF7;

// Set center frequency. If in doubt, choose the first one, comment all others
// Each "real" gateway should support the first 3 frequencies according to LoRa spec.
uint32_t freq = 868100000;  // Channel 0, 868.1 MHz
//uint32_t  freq = 868300000; 					// Channel 1, 868.3 MHz
//uint32_t  freq = 868500000; 					// in Mhz! (868.5)
//uint32_t  freq = 867100000; 					// in Mhz! (867.1)
//uint32_t  freq = 867300000; 					// in Mhz! (867.3)
//uint32_t  freq = 867500000; 					// in Mhz! (867.5)
//uint32_t  freq = 867700000; 					// in Mhz! (867.7)
//uint32_t  freq = 867900000; 					// in Mhz! (867.9)
//uint32_t  freq = 868800000; 					// in Mhz! (868.8)
//uint32_t  freq = 869525000; 					// in Mhz! (869.525)
// TTN defines an additional channel at 869.525Mhz using SF9 for class B. Not used

float lat = _LAT;  // Configuration specific info...
float lon = _LON;
int alt = _ALT;
char platform[24] = _PLATFORM;        // platform definition
char email[40] = _EMAIL;              // used for contact email
char description[64] = _DESCRIPTION;  // used for free form description

IPAddress ntpServer;  // IP address of NTP_TIMESERVER
IPAddress ttnServer;  // IP Address of thethingsnetwork server

WiFiUDP Udp;
uint32_t lasttime;

#if A_SERVER == 1
//#include <Streaming.h>          				// http://arduiniana.org/libraries/streaming/
ESP8266WebServer server(SERVERPORT);
#endif


void die(const char *s) {
  Serial.println(s);
  // delay(50);
  yield();
  // system_restart();						// SDK function
  // ESP.reset();
  abort();  // Within a second
}

void printTime() {
  const char *Days[] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
  Serial.printf("%s %02d:%02d:%02d", Days[weekday() - 1], hour(), minute(), second());
}


void ftoa(float f, char *val, int p) {
  int j = 1;
  int ival, fval;
  char b[6];

  for (int i = 0; i < p; i++) { j = j * 10; }

  ival = (int)f;                 // Make integer part
  fval = (int)((f - ival) * j);  // Make fraction. Has same sign as integer part
  if (fval < 0) fval = -fval;    // So if it is negative make fraction positive again.
                                 // sprintf does NOT fit in memory
  strcat(val, itoa(ival, b, 10));
  strcat(val, ".");  // decimal point

  itoa(fval, b, 10);
  for (int i = 0; i < (p - strlen(b)); i++) strcat(val, "0");
  // Fraction can be anything from 0 to 10^p , so can have less digits
  strcat(val, b);
}


const int NTP_PACKET_SIZE = 48;  // Fixed size of NTP record
uint8_t packetBuffer[NTP_PACKET_SIZE];

void sendNTPpacket(IPAddress &timeServerIP) {
  // Zeroise the buffer.
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;  // LI, Version, Mode
  packetBuffer[1] = 0;           // Stratum, or type of clock
  packetBuffer[2] = 6;           // Polling Interval
  packetBuffer[3] = 0xEC;        // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  Udp.beginPacket(timeServerIP, (int)123);  // NTP Server and Port

  if ((Udp.write((char *)packetBuffer, NTP_PACKET_SIZE)) != NTP_PACKET_SIZE) {
    die("sendNtpPacket:: Error write");
  } else {
    // Success
  }
  Udp.endPacket();
}


time_t getNtpTime() {
  WiFi.hostByName(NTP_TIMESERVER, ntpServer);
  //while (Udp.parsePacket() > 0) ; 			// discard any previously received packets
  for (int i = 0; i < 4; i++) {  // 5 retries.
    sendNTPpacket(ntpServer);
    uint32_t beginWait = millis();
    while (millis() - beginWait < 5000) {
      if (Udp.parsePacket()) {
        Udp.read(packetBuffer, NTP_PACKET_SIZE);
        // Extract seconds portion.
        unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
        unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
        unsigned long secSince1900 = highWord << 16 | lowWord;
        Udp.flush();
        return secSince1900 - 2208988800UL + NTP_TIMEZONES * SECS_PER_HOUR;
      }
    }
  }
  return 0;  // return 0 if unable to get the time
}

void setupTime() {
  setSyncProvider(getNtpTime);
  setSyncInterval(NTP_INTERVAL);
}


IPAddress getDnsIP() {
  const ip_addr_t *dns_ip = dns_getserver(0);  // Get the pointer
  if (dns_ip != nullptr) {
    // Extract the IP address as a 32-bit integer
    uint32_t address = dns_ip->addr;
    // Construct IPAddress from the 32-bit integer
    IPAddress dns(address);
    return dns;
  }
  return IPAddress(0, 0, 0, 0);  // Return an empty IP if there's an issue
}

// IPAddress getDnsIP() {
// 	ip_addr_t dns_ip = dns_getserver(0);
// 	IPAddress dns = IPAddress(dns_ip.addr);
// 	return((IPAddress) dns);
// }
// IPAddress getDnsIP() {
//     ip_addr_t dns_ip = dns_getserver(0);
//     // Extract the IP address as a 32-bit integer
//     uint32_t address = dns_ip.u_addr.ip4.addr;
//     // Construct IPAddress from the 32-bit integer
//     IPAddress dns(address);
//     return dns;
// }



int readUdp(int packetSize) {
  char receiveBuffer[64];  //buffer to hold incoming packet
  Udp.read(receiveBuffer, packetSize);
  receiveBuffer[packetSize] = 0;
  IPAddress remoteIpNo = Udp.remoteIP();
  unsigned int remotePortNo = Udp.remotePort();
  if (debug >= 1) {
    Serial.print(F(" Received packet of size "));
    Serial.print(packetSize);
    Serial.print(F(" From "));
    Serial.print(remoteIpNo);
    Serial.print(F(", port "));
    Serial.print(remotePortNo);
    Serial.print(F(", Contents: 0x"));
    for (int i = 0; i < packetSize; i++) {
      Serial.printf("%02X:", receiveBuffer[i]);
    }
    Serial.println();
  }
  return packetSize;
}


int WlanConnect(char *wifi_ssid, char *wifi_pass) {
  char thishost[17];
  WiFiMode_t con_type;
  int ret = WiFi.status();

  WiFi.mode(WIFI_AP_STA);

  // Set Hostname for OTA and network (add only 2 last bytes of last MAC Address)
  sprintf_P(thishost, PSTR("ESP-TTN-GW-%04X"), ESP.getChipId() & 0xFFFF);

  if (debug >= 1) {
    Serial.print(F("========== SDK Saved parameters Start"));
    WiFi.printDiag(Serial);
    Serial.println(F("========== SDK Saved parameters End"));
  }

  if (strncmp(wifi_ssid, "**", 2) && strncmp(wifi_pass, "**", 2)) {
    Serial.println(F("Sketch contain SSID/PSK will set them"));
  }

  // No empty sketch SSID, try connect
  if (*wifi_ssid != '*' && *wifi_pass != '*') {
    Serial.printf("connecting to %s with psk %s\r\n", wifi_ssid, wifi_pass);
    WiFi.begin(wifi_ssid, wifi_pass);
  } else {
    // empty sketch SSID, try autoconnect with SDK saved credentials
    Serial.println(F("No SSID/PSK defined in sketch\r\nConnecting with SDK ones if any"));
  }

  // Loop until connected or 20 sec time out
  unsigned long this_start = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - this_start < 20000) {
    // delay(1);
    yield();
  }

  ret = WiFi.status();

  if (ret == WL_CONNECTED) {
    WiFi.mode(WIFI_STA);
  } else {

    Serial.printf("Starting AP  : %s", thishost);

    if (ret != WL_CONNECTED) {
      // Disable auto retry search channel
      WiFi.disconnect();
    }

    // protected network
    Serial.printf(" with key %s\r\n", _AP_PASS);
    WiFi.softAP(thishost, _AP_PASS);

    Serial.print(F("IP address   : "));
    Serial.println(WiFi.softAPIP());
    Serial.print(F("MAC address  : "));
    Serial.println(WiFi.softAPmacAddress());
  }

#ifdef WEMOS_LORA_GW
  con_type = WiFi.getMode();
  Serial.print(F("WIFI="));
  if (con_type == WIFI_STA) {
    // OK breathing cyan
    // wifi_led_color=COLOR_CYAN;
    Serial.print(F("STA"));
  } else if (con_type == WIFI_AP || con_type == WIFI_AP_STA) {
    // Owe're also in AP ? breathing Yellow
    // wifi_led_color=COLOR_ORANGE;
    if (con_type == WIFI_AP) {
      Serial.print(F("AP"));
    } else {
      Serial.print(F("AP_STA"));
    }
  } else {
    // Error breathing red
    // wifi_led_color=COLOR_RED;
    Serial.print(F("???"));
  }

#endif

  ArduinoOTA.setHostname(thishost);
  ArduinoOTA.begin();

  // OTA callbacks
  ArduinoOTA.onStart([]() {
    Serial.println(F("\r\nOTA Starting"));
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    uint8_t percent = progress / (total / 100);

    if (percent % 10 == 0) {
      Serial.print('.');
    }
  });

  ArduinoOTA.onEnd([]() {
    Serial.println(F("Done Rebooting"));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.println(F("Error"));
    ESP.restart();
  });

  return ret;
}

void sendUdp(char *msg, int length) {
  int l;
  bool err = true;
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("sendUdp: ERROR not connected to WLAN"));
    Udp.flush();
  } else {
    //send the update
    Udp.beginPacket(ttnServer, (int)PORT1);

#ifdef SERVER2
    // delay(1);
    yield();
    Udp.beginPacket((char *)SERVER2, (int)PORT2);
#endif

    if ((l = Udp.write((char *)msg, length)) != length) {
      Serial.println(F("sendUdp:: Error write"));
    } else {
      err = false;
      if (debug >= 2) {
        Serial.printf("sendUdp: sent %d bytes", l);
      }
    }

    yield();
    Udp.endPacket();
  }
}


bool UDPconnect() {

  bool ret = false;
  if (debug >= 1) Serial.println(F("Connecting to UDP"));
  unsigned int localPort = 1701;  // XXX Do not listen to return messages from WiFi
  if (Udp.begin(localPort) == 1) {
    if (debug >= 1) Serial.println(F("Connection successful"));
    ret = true;
  } else {
    //Serial.println("Connection failed");
  }
  return (ret);
}

inline void selectreceiver() {
  digitalWrite(ssPin, LOW);
}

inline void unselectreceiver() {
  digitalWrite(ssPin, HIGH);
}


uint8_t readRegister(uint8_t addr) {
  selectreceiver();
  SPI.beginTransaction(SPISettings(50000, MSBFIRST, SPI_MODE0));
  SPI.transfer(addr & 0x7F);
  uint8_t res = SPI.transfer(0x00);
  SPI.endTransaction();
  unselectreceiver();
  return res;
}


void writeRegister(uint8_t addr, uint8_t value) {
  unsigned char spibuf[2];

  spibuf[0] = addr | 0x80;
  spibuf[1] = value;
  selectreceiver();
  SPI.beginTransaction(SPISettings(50000, MSBFIRST, SPI_MODE0));
  SPI.transfer(spibuf[0]);
  SPI.transfer(spibuf[1]);
  SPI.endTransaction();
  unselectreceiver();
}

bool receivePkt(char *payload) {
  // clear rxDone
  writeRegister(REG_IRQ_FLAGS, 0x40);

  int irqflags = readRegister(REG_IRQ_FLAGS);

  cp_nb_rx_rcv++;  // Receive statistics counter

  //  payload crc: 0x20
  if ((irqflags & 0x20) == 0x20) {
    Serial.println(F("CRC error"));
    writeRegister(REG_IRQ_FLAGS, 0x20);
    return false;
  } else {

    cp_nb_rx_ok++;  // Receive OK statistics counter

    uint8_t currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
    uint8_t receivedCount = readRegister(REG_RX_NB_BYTES);
    receivedbytes = receivedCount;

    writeRegister(REG_FIFO_ADDR_PTR, currentAddr);

    for (int i = 0; i < receivedCount; i++) {
      payload[i] = (char)readRegister(REG_FIFO);
    }
  }
  return true;
}

void SetupLoRa() {
  Serial.println(F("Trying to Setup LoRa Module with"));
  Serial.printf("CS=GPIO%d  DIO0=GPIO%d  Reset=", ssPin, dio0);

  if (RST == NOT_A_PIN) {
    Serial.println(F("Unused"));
  } else {
    Serial.printf("GPIO%d\r\n", RST);
    digitalWrite(RST, HIGH);
    // delay(100);
    yield();
    digitalWrite(RST, LOW);
    // delay(100);
    yield();
  }

  uint8_t version = readRegister(REG_VERSION);  // Read the LoRa chip version id
  if (version == 0x22) {
    // sx1272
    Serial.println(F("SX1272 detected, starting."));
    sx1272 = true;
  } else {
    // sx1276?
    if (RST != NOT_A_PIN) {
      digitalWrite(RST, LOW);
      // delay(100);
      yield();
      digitalWrite(RST, HIGH);
      // delay(100);
      yield();
    }

    version = readRegister(REG_VERSION);
    if (version == 0x12) {
      // sx1276
      Serial.println(F("SX1276 detected, starting."));
      sx1272 = false;
    } else {
      Serial.print(F("Unrecognized transceiver, version: 0x"));
      Serial.printf("%02X", version);
      die("");
    }
  }

  writeRegister(REG_OPMODE, SX72_MODE_SLEEP);

  // set frequency
  uint64_t frf = ((uint64_t)freq << 19) / 32000000;
  writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));

  writeRegister(REG_SYNC_WORD, 0x34);  // LoRaWAN public sync word

  // Set spreading Factor
  if (sx1272) {
    if (sf == SF11 || sf == SF12) {
      writeRegister(REG_MODEM_CONFIG, 0x0B);
    } else {
      writeRegister(REG_MODEM_CONFIG, 0x0A);
    }
    writeRegister(REG_MODEM_CONFIG2, (sf << 4) | 0x04);
  } else {
    if (sf == SF11 || sf == SF12) {
      writeRegister(REG_MODEM_CONFIG3, 0x0C);
    } else {
      writeRegister(REG_MODEM_CONFIG3, 0x04);
    }
    writeRegister(REG_MODEM_CONFIG, 0x72);
    writeRegister(REG_MODEM_CONFIG2, (sf << 4) | 0x04);
  }

  if (sf == SF10 || sf == SF11 || sf == SF12) {
    writeRegister(REG_SYMB_TIMEOUT_LSB, 0x05);
  } else {
    writeRegister(REG_SYMB_TIMEOUT_LSB, 0x08);
  }
  writeRegister(REG_MAX_PAYLOAD_LENGTH, 0x80);
  writeRegister(REG_PAYLOAD_LENGTH, PAYLOAD_LENGTH);
  writeRegister(REG_HOP_PERIOD, 0xFF);
  writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_BASE_AD));

  // Set Continous Receive Mode
  writeRegister(REG_LNA, LNA_MAX_GAIN);  // max lna gain
  writeRegister(REG_OPMODE, SX72_MODE_RX_CONTINUOS);
}


void sendstat() {
  // XXX removed static
  char status_report[STATUS_SIZE];  // status report as a JSON object
  char stat_timestamp[32];          // XXX was 24
  time_t t;
  char clat[8] = { 0 };
  char clon[8] = { 0 };

  int stat_index = 0;

  // pre-fill the data buffer with fixed fields
  status_report[0] = PROTOCOL_VERSION;
  status_report[3] = PKT_PUSH_DATA;

  // READ MAC ADDRESS OF ESP8266
  status_report[4] = MAC_array[0];
  status_report[5] = MAC_array[1];
  status_report[6] = MAC_array[2];
  status_report[7] = 0xFF;
  status_report[8] = 0xFF;
  status_report[9] = MAC_array[3];
  status_report[10] = MAC_array[4];
  status_report[11] = MAC_array[5];

  uint8_t token_h = (uint8_t)rand();  // random token
  uint8_t token_l = (uint8_t)rand();  // random token
  status_report[1] = token_h;
  status_report[2] = token_l;
  stat_index = 12;  // 12-byte header

  t = now();  // get timestamp for statistics


  sprintf(stat_timestamp, "%d-%d-%d %d:%d:%d CET", year(), month(), day(), hour(), minute(), second());

  ftoa(lat, clat, 4);  // Convert lat to char array with 4 decimals
  ftoa(lon, clon, 4);  // As Arduino CANNOT prints floats


  int j = snprintf((char *)(status_report + stat_index), STATUS_SIZE - stat_index,
                   "{\"stat\":{\"time\":\"%s\",\"lati\":%s,\"long\":%s,\"alti\":%i,\"rxnb\":%u,\"rxok\":%u,\"rxfw\":%u,\"ackr\":%u.0,\"dwnb\":%u,\"txnb\":%u,\"pfrm\":\"%s\",\"mail\":\"%s\",\"desc\":\"%s\"}}",
                   stat_timestamp, clat, clon, (int)alt, cp_nb_rx_rcv, cp_nb_rx_ok, cp_up_pkt_fwd, 0, 0, 0, platform, email, description);
  stat_index += j;
  status_report[stat_index] = 0;  // add string terminator, for safety

  if (debug >= 1) {
    Serial.print(F("stat update: <"));
    Serial.print(stat_index);
    Serial.print(F("> "));
    Serial.println((char *)(status_report + 12));  // DEBUG: display JSON stat
  }

  //send the update
  sendUdp(status_report, stat_index);
}

int receivepacket(char *buff_up) {

  long int SNR;
  int rssicorr;
  char cfreq[12] = { 0 };  // Character array to hold freq in MHz

  if (digitalRead(dio0) == 1)  // READY?
  {
    if (receivePkt(message)) {
      uint8_t value = readRegister(REG_PKT_SNR_VALUE);

      if (value & 0x80) {  // The SNR sign bit is 1
        // Invert and divide by 4
        value = ((~value + 1) & 0xFF) >> 2;
        SNR = -value;
      } else {
        // Divide by 4
        SNR = (value & 0xFF) >> 2;
      }

      if (sx1272) {
        rssicorr = 139;
      } else {  // Probably SX1276 or RFM95
        rssicorr = 157;
      }

      if (debug >= 1) {
        Serial.print(F("Packet RSSI: "));
        Serial.print(readRegister(0x1A) - rssicorr);
        Serial.print(F(" RSSI: "));
        Serial.print(readRegister(0x1B) - rssicorr);
        Serial.print(F(" SNR: "));
        Serial.print(SNR);
        Serial.print(F(" Length: "));
        Serial.print((int)receivedbytes);
        Serial.println();
        yield();
      }

      int j;

      int encodedLen = base64_enc_len(receivedbytes);  // max 341
      base64_encode(b64, message, receivedbytes);      // max 341


      int buff_index = 0;

      // pre-fill the data buffer with fixed fields
      buff_up[0] = PROTOCOL_VERSION;
      buff_up[3] = PKT_PUSH_DATA;

      // XXX READ MAC ADDRESS OF ESP8266
      buff_up[4] = MAC_array[0];
      buff_up[5] = MAC_array[1];
      buff_up[6] = MAC_array[2];
      buff_up[7] = 0xFF;
      buff_up[8] = 0xFF;
      buff_up[9] = MAC_array[3];
      buff_up[10] = MAC_array[4];
      buff_up[11] = MAC_array[5];

      // start composing datagram with the header
      uint8_t token_h = (uint8_t)rand();  // random token
      uint8_t token_l = (uint8_t)rand();  // random token
      buff_up[1] = token_h;
      buff_up[2] = token_l;
      buff_index = 12; /* 12-byte header */

      // TODO: tmst can jump if time is (re)set, not good.
      struct timeval now;
      gettimeofday(&now, NULL);
      uint32_t tmst = (uint32_t)(now.tv_sec * 1000000 + now.tv_usec);

      // start of JSON structure that will make payload
      memcpy((void *)(buff_up + buff_index), (void *)"{\"rxpk\":[", 9);
      buff_index += 9;
      buff_up[buff_index] = '{';
      ++buff_index;
      j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE - buff_index, "\"tmst\":%u", tmst);
      buff_index += j;
      ftoa((double)freq / 1000000, cfreq, 6);  // XXX This can be done better
      j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE - buff_index, ",\"chan\":%1u,\"rfch\":%1u,\"freq\":%s", 0, 0, cfreq);
      buff_index += j;
      memcpy((void *)(buff_up + buff_index), (void *)",\"stat\":1", 9);
      buff_index += 9;
      memcpy((void *)(buff_up + buff_index), (void *)",\"modu\":\"LORA\"", 14);
      buff_index += 14;
      /* Lora datarate & bandwidth, 16-19 useful chars */
      switch (sf) {
        case SF7:
          memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF7", 12);
          buff_index += 12;
          break;
        case SF8:
          memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF8", 12);
          buff_index += 12;
          break;
        case SF9:
          memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF9", 12);
          buff_index += 12;
          break;
        case SF10:
          memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF10", 13);
          buff_index += 13;
          break;
        case SF11:
          memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF11", 13);
          buff_index += 13;
          break;
        case SF12:
          memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF12", 13);
          buff_index += 13;
          break;
        default:
          memcpy((void *)(buff_up + buff_index), (void *)",\"datr\":\"SF?", 12);
          buff_index += 12;
      }
      memcpy((void *)(buff_up + buff_index), (void *)"BW125\"", 6);
      buff_index += 6;
      memcpy((void *)(buff_up + buff_index), (void *)",\"codr\":\"4/5\"", 13);
      buff_index += 13;
      j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE - buff_index, ",\"lsnr\":%li", SNR);
      buff_index += j;
      j = snprintf((char *)(buff_up + buff_index), TX_BUFF_SIZE - buff_index, ",\"rssi\":%d,\"size\":%u", readRegister(0x1A) - rssicorr, receivedbytes);
      buff_index += j;
      memcpy((void *)(buff_up + buff_index), (void *)",\"data\":\"", 9);
      buff_index += 9;

      // Use gBase64 library
      encodedLen = base64_enc_len(receivedbytes);  // max 341
      j = base64_encode((char *)(buff_up + buff_index), message, receivedbytes);

      buff_index += j;
      buff_up[buff_index++] = '"';

      // End of packet serialization
      buff_up[buff_index++] = '}';
      buff_up[buff_index++] = ']';
      // end of JSON datagram payload */
      buff_up[buff_index++] = '}';
      buff_up[buff_index] = 0;  // add string terminator, for safety

      if (debug >= 1) {
        Serial.print(F("rxpk update: "));
        Serial.println((char *)(buff_up + 12));  // DEBUG: display JSON payload
      }
      return (buff_index);

    }  // received a message
  }    // dio0=1
  return (-1);
}



#if A_SERVER == 1


// String printIP(IPAddress ipa) {
// 	String response;
// 	response+=(IPAddress)ipa[0]; response+=".";
// 	response+=(IPAddress)ipa[1]; response+=".";
// 	response+=(IPAddress)ipa[2]; response+=".";
// 	response+=(IPAddress)ipa[3];
// 	return (response);
// }
// String printIP(IPAddress ipa) {
//     return ipa.toString();
// }

String printIP(IPAddress ipa) {
  String response = "";
  response += String(ipa[0]);
  response += ".";
  response += String(ipa[1]);
  response += ".";
  response += String(ipa[2]);
  response += ".";
  response += String(ipa[3]);
  return response;
}

// String printIP(IPAddress ipa) {
//     String response;
//     response += String(ipa[0]); response += ".";
//     response += String(ipa[1]); response += ".";
//     response += String(ipa[2]); response += ".";
//     response += String(ipa[3]);
//     return response;
// }



String stringTime(unsigned long t) {
  String response;
  String Days[7] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

  if (t == 0) {
    response = " -none- ";
    return (response);
  }

  // now() gives seconds since 1970
  time_t eventTime = now() - ((millis() - t) / 1000);
  uint8_t _hour = hour(eventTime);
  uint8_t _minute = minute(eventTime);
  uint8_t _second = second(eventTime);

  response += Days[weekday(eventTime) - 1];
  response += " ";
  response += day(eventTime);
  response += "-";
  response += month(eventTime);
  response += "-";
  response += year(eventTime);
  response += " ";
  if (_hour < 10) response += "0";
  response += _hour;
  response += ":";
  if (_minute < 10) response += "0";
  response += _minute;
  response += ":";
  if (_second < 10) response += "0";
  response += _second;
  return (response);
}



void WifiServer(const char *cmd, const char *arg) {

  String response;
  char *dup, *pch;

  yield();
  if (debug >= 2) {
    Serial.print(F("WifiServer new client"));
  }

  // These can be used as a single argument
  if (strcmp(cmd, "DEBUG") == 0) {  // Set debug level 0-2
    debug = atoi(arg);
    response += " debug=";
    response += arg;
  }
  // if (strcmp(cmd, "IP")==0) {										// List local IP address
  // 	response+=" local IP=";
  // 	response+=(IPAddress) WiFi.localIP()[0]; response += ".";
  // 	response+=(IPAddress) WiFi.localIP()[1]; response += ".";
  // 	response+=(IPAddress) WiFi.localIP()[2]; response += ".";
  // 	response+=(IPAddress) WiFi.localIP()[3];
  // }
  if (strcmp(cmd, "IP") == 0) {  // List local IP address
    response += " local IP=";
    IPAddress ip = WiFi.localIP();
    response += String(ip[0]);
    response += ".";
    response += String(ip[1]);
    response += ".";
    response += String(ip[2]);
    response += ".";
    response += String(ip[3]);
  }


  if (strcmp(cmd, "GETTIME") == 0) { response += "gettime tbd"; }  // Get the local time
  if (strcmp(cmd, "SETTIME") == 0) { response += "settime tbd"; }  // Set the local time
  if (strcmp(cmd, "HELP") == 0) { response += "Display Help Topics"; }
  if (strcmp(cmd, "RESET") == 0) {
    response += "Resetting Statistics";
    cp_nb_rx_rcv = 0;
    cp_nb_rx_ok = 0;
    cp_up_pkt_fwd = 0;
  }

  // Do work, fill the webpage
  // delay(15);
  yield();
  response += "<!DOCTYPE HTML>";
  response += "<HTML><HEAD>";
  response += "<TITLE>ESP8266 1ch Gateway</TITLE>";
  response += "</HEAD>";
  response += "<BODY>";
  yield();
  response += "<h1>ESP Gateway Config:</h1>";
  response += "Version: ";
  response += VERSION;
  response += "<br>ESP is alive since ";
  response += stringTime(1);
  response += "<br>Current time is    ";
  response += stringTime(millis());
  response += "<br>";
  yield();
  response += "<h2>WiFi Config</h2>";
  response += "<table style=\"max_width: 100%; min-width: 40%; border: 1px solid black; border-collapse: collapse;\" class=\"config_table\">";
  response += "<tr>";
  response += "<th style=\"background-color: green; color: white;\">Parameter</th>";
  response += "<th style=\"background-color: green; color: white;\">Value</th>";
  response += "</tr>";
  response += "<tr><td style=\"border: 1px solid black;\">IP Address</td><td style=\"border: 1px solid black;\">";
  response += printIP((IPAddress)WiFi.localIP());
  response += "</tr>";
  response += "<tr><td style=\"border: 1px solid black;\">IP Gateway</td><td style=\"border: 1px solid black;\">";
  response += printIP((IPAddress)WiFi.gatewayIP());
  response += "</tr>";
  response += "<tr><td style=\"border: 1px solid black;\">NTP Server</td><td style=\"border: 1px solid black;\">";
  response += NTP_TIMESERVER;
  response += "</tr>";
  response += "<tr><td style=\"border: 1px solid black;\">LoRa Router</td><td style=\"border: 1px solid black;\">";
  response += _TTNSERVER;
  response += "</tr>";
  response += "<tr><td style=\"border: 1px solid black;\">LoRa Router IP</td><td style=\"border: 1px solid black;\">";
  response += printIP((IPAddress)ttnServer);
  response += "</tr>";
  response += "</table>";
  yield();
  response += "<h2>System Status</h2>";
  response += "<table style=\"max_width: 100%; min-width: 40%; border: 1px solid black; border-collapse: collapse;\" class=\"config_table\">";
  response += "<tr>";
  response += "<th style=\"background-color: green; color: white;\">Parameter</th>";
  response += "<th style=\"background-color: green; color: white;\">Value</th>";
  response += "</tr>";
  response += "<tr><td style=\"border: 1px solid black;\">Free heap</td><td style=\"border: 1px solid black;\">";
  response += ESP.getFreeHeap();
  response += "</tr>";
  response += "<tr><td style=\"border: 1px solid black;\">ESP Chip ID</td><td style=\"border: 1px solid black;\">";
  response += ESP.getChipId();
  response += "</tr>";
  response += "</table>";
  yield();
  response += "<h2>LoRa Status</h2>";
  response += "<table style=\"max_width: 100%; min-width: 40%; border: 1px solid black; border-collapse: collapse;\" class=\"config_table\">";
  response += "<tr>";
  response += "<th style=\"background-color: green; color: white;\">Parameter</th>";
  response += "<th style=\"background-color: green; color: white;\">Value</th>";
  response += "</tr>";
  response += "<tr><td style=\"border: 1px solid black;\">Frequency</td><td style=\"border: 1px solid black;\">";
  response += freq;
  response += "</tr>";
  response += "<tr><td style=\"border: 1px solid black;\">Spreading Factor</td><td style=\"border: 1px solid black;\">";
  response += sf;
  response += "</tr>";
  response += "<tr><td style=\"border: 1px solid black;\">Gateway ID</td><td style=\"border: 1px solid black;\">";
  response += String(MAC_array[0], HEX);  // The MAC array is always returned in lowercase
  response += String(MAC_array[1], HEX);
  response += String(MAC_array[2], HEX);
  response += "ffff";
  response += String(MAC_array[3], HEX);
  response += String(MAC_array[4], HEX);
  response += String(MAC_array[5], HEX);
  response += "</tr>";
  response += "</table>";
  yield();
  response += "<h2>Statistics</h2>";
  // delay(1);
  yield();
  response += "<table style=\"max_width: 100%; min-width: 40%; border: 1px solid black; border-collapse: collapse;\" class=\"config_table\">";
  response += "<tr>";
  response += "<th style=\"background-color: green; color: white;\">Counter</th>";
  response += "<th style=\"background-color: green; color: white;\">Value</th>";
  response += "</tr>";
  response += "<tr><td style=\"border: 1px solid black;\">Packages Received</td><td style=\"border: 1px solid black;\">";
  response += cp_nb_rx_rcv;
  response += "</tr>";
  response += "<tr><td style=\"border: 1px solid black;\">Packages OK </td><td style=\"border: 1px solid black;\">";
  response += cp_nb_rx_ok;
  response += "</tr>";
  response += "<tr><td style=\"border: 1px solid black;\">Packages Forwarded</td><td style=\"border: 1px solid black;\">";
  response += cp_up_pkt_fwd;
  response += "</tr>";
  response += "<tr><td>&nbsp</td><td> </tr>";
  yield();
  response += "</table>";

  response += "<br>";
  response += "<h2>Settings</h2>";
  response += "Click <a href=\"/RESET\">here</a> to reset statistics<br>";
  yield();
  response += "Debug level is: ";
  response += debug;
  response += " set to: ";
  response += " <a href=\"DEBUG=0\">0</a>";
  response += " <a href=\"DEBUG=1\">1</a>";
  response += " <a href=\"DEBUG=2\">2</a><br>";

  response += "Click <a href=\"/HELP\">here</a> to explain Help and REST options<br>";
  response += "</BODY></HTML>";
  yield();
  server.send(200, "text/html", response);

  yield();
  free(dup);  // free the memory used, before jumping to other page
}

#endif


void setup() {

  // Watchdog timer
  ESP.wdtFeed();

  WiFiMode_t con_type;
  int ret = WiFi.status();

  Serial.begin(_BAUDRATE);  // As fast as possible for bus

  Serial.print(F("\r\nBooting "));
  Serial.println(ARDUINO_BOARD " " __DATE__ " " __TIME__);

  if (debug >= 1) {
    Serial.print(F("! debug: "));
  }


  if (WlanConnect((char *)_SSID, (char *)_PASS) == WL_CONNECTED) {

    if (!UDPconnect()) {
      Serial.println("Error UDPconnect");
    }
  }

  WiFi.macAddress(MAC_array);
  for (int i = 0; i < sizeof(MAC_array); ++i) {
    sprintf(MAC_char, "%s%02x:", MAC_char, MAC_array[i]);
  }
  Serial.print("MAC: ");
  Serial.println(MAC_char);

  // Configure IO Pin
  pinMode(ssPin, OUTPUT);
  pinMode(dio0, INPUT);
  pinMode(RST, OUTPUT);

  SPI.begin();
  // delay(100);
  yield();
  SetupLoRa();
  // delay(100);
  yield();

  Serial.print("Gateway ID: ");
  Serial.print(MAC_array[0], HEX);
  Serial.print(MAC_array[1], HEX);
  Serial.print(MAC_array[2], HEX);
  Serial.print(0xFF, HEX);
  Serial.print(0xFF, HEX);
  Serial.print(MAC_array[3], HEX);
  Serial.print(MAC_array[4], HEX);
  Serial.print(MAC_array[5], HEX);

  Serial.print(", Listening at SF");
  Serial.print(sf);
  Serial.print(" on ");
  Serial.print((double)freq / 1000000);
  Serial.println(" Mhz.");

  WiFi.hostByName(_TTNSERVER, ttnServer);  // Use DNS to get server IP once
  // delay(100);
  yield();

  setupTime();  // Set NTP time host and interval
  setTime((time_t)getNtpTime());
  Serial.print("time ");
  printTime();
  Serial.println();

#if A_SERVER == 1
  server.on("/", []() {
    WifiServer("", "");
  });
  server.on("/HELP", []() {
    WifiServer("HELP", "");
  });
  server.on("/RESET", []() {
    WifiServer("RESET", "");
  });
  server.on("/DEBUG=0", []() {
    WifiServer("DEBUG", "0");
  });
  server.on("/DEBUG=1", []() {
    WifiServer("DEBUG", "1");
  });
  server.on("/DEBUG=2", []() {
    WifiServer("DEBUG", "2");
  });

  server.begin();  // Start the webserver
  Serial.print(F("Admin Server started on port "));
  Serial.println(SERVERPORT);
#endif
  Serial.println("---------------------------------");
}

void loop() {
  // Trying to reset the watchdog timer
  ESP.wdtFeed();


  static bool led_state;
  bool new_led_state;
  int buff_index;
  char buff_up[TX_BUFF_SIZE];  // buffer to compose the upstream packet

  // Receive Lora messages
  if ((buff_index = receivepacket(buff_up)) >= 0) {  // read is successful
    yield();

    sendUdp(buff_up, buff_index);  // We can send to multiple sockets if necessary
  } else {
    // No message received
  }

  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    yield();

    uint32_t nowseconds = (uint32_t)millis() / 1000;
    if (nowseconds - lasttime >= 30) {  // Send status every 30 seconds
      sendstat();
      lasttime = nowseconds;
    }

    // Handle the WiFi server part of this sketch. Mainly used for administration of the node
#if A_SERVER == 1
    server.handleClient();
#endif

    // On board Led blink management
    if (WiFi.status() == WL_CONNECTED) {
      new_led_state = ((millis() % 1000) < 200) ? LOW : HIGH;  // Connected long blink 200ms on each second
    } else {
      new_led_state = ((millis() % 333) < 111) ? LOW : HIGH;  // AP Mode or client failed quick blink 111ms on each 1/3sec
    }
    // Led management
    if (led_state != new_led_state) {
      led_state = new_led_state;
      digitalWrite(LED_BUILTIN, led_state);
    }

    ArduinoOTA.handle();
  }
}
