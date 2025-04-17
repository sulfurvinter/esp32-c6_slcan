#include <ESP32-TWAI-CAN.hpp>
#include <Adafruit_NeoPixel.h>

// Simple sketch that querries OBD2 over CAN for coolant temperature
// Showcasing simple use of ESP32-TWAI-CAN library driver.

// Default for ESP32
#define CAN_TX		19
#define CAN_RX		20

#define CMD_LEN (sizeof("T12345678811223344556677881234\r")+1)

#define RGB_PIN 8  //built in rgb led

//int g_can_speed = CANSPEED_500; // default: 500k
int g_can_speed = 500;
int g_ts_en = 0;

CanFrame rxFrame;

//set up onboard neopixel
Adafruit_NeoPixel pixels(1, RGB_PIN, NEO_GRB + NEO_KHZ400);


typedef struct
{
	uint16_t id;
	uint16_t ide;
	struct {
		int8_t rtr : 1;
		int8_t ide : 1;
		uint8_t length : 4;
	} header;
	uint8_t data[8];
} tCAN;


//typedef struct {
//    union {
//        struct {
//            //The order of these bits must match deprecated message flags for compatibility reasons
//            uint32_t extd: 1;           /**< Extended Frame Format (29bit ID) */
//            uint32_t rtr: 1;            /**< Message is a Remote Frame */
//            uint32_t ss: 1;             /**< Transmit as a Single Shot Transmission. Unused for received. */
//            uint32_t self: 1;           /**< Transmit as a Self Reception Request. Unused for received. */
//            uint32_t dlc_non_comp: 1;   /**< Message's Data length code is larger than 8. This will break compliance with ISO 11898-1 */
//            uint32_t reserved: 27;      /**< Reserved bits */
//        };
//        //Todo: Deprecate flags
//        uint32_t flags;                 /**< Deprecated: Alternate way to set bits using message flags */
//    };
//    uint32_t identifier;                /**< 11 or 29 bit identifier */
//    uint8_t data_length_code;           /**< Data length code */
//    uint8_t data[TWAI_FRAME_MAX_DLC];    /**< Data bytes (not relevant in RTR frame) */
//} twai_message_t;



//----------------------------------- b2ahex(p, 2, msg.header.length, msg.data);

// conversion from hex to asci-hex, p=output buffer, s=type of conversion (1=1 byte,2=),  n=number of bytes to convert
int b2ahex(char *p, uint8_t s, uint8_t n, void *v)
{
  const char *hex = "0123456789ABCDEF";

  if (s == 1) {
    uint8_t *tmp = (uint8_t *)v;
    for (int i=0; i<n; i++) {
      *p++ = hex[tmp[i] & 0x0f];
    }
  } else if (s == 2) {
    uint8_t *tmp = (uint8_t *)v;
    for (int i=0; i<n; i++) {
      *p++ = hex[(tmp[i] >> 4) & 0x0f];
      *p++ = hex[tmp[i] & 0x0f];
    }
  } else if (s == 3) {
    uint16_t *tmp = (uint16_t *)v;
    for (int i=0; i<n; i++) {
      *p++ = hex[(tmp[i] >> 8) & 0x0f];
      *p++ = hex[(tmp[i] >> 4) & 0x0f];
      *p++ = hex[tmp[i] & 0x0f];
    }
  } else if (s == 4) {
    uint16_t *tmp = (uint16_t *)v;
    for (int i=0; i<n; i++) {
      *p++ = hex[(tmp[i] >> 12) & 0x0f];
      *p++ = hex[(tmp[i] >> 8) & 0x0f];
      *p++ = hex[(tmp[i] >> 4) & 0x0f];
      *p++ = hex[tmp[i] & 0x0f];
    }
  }
  return s*n;
}

int a2bhex_sub(char a)
{
  int val = 0;

  if ('0' <= a && a <= '9') {
    val = a - '0';
  } else if ('A' <= a && a <= 'F') {
    val = a - 'A' + 10;
  } else if ('a' <= a && a <= 'f') {
    val = a - 'a' + 10;
  }
  return val;
}

int a2bhex(char *p, uint8_t s, uint8_t n, void *v)
{
  int i, j;
  char buf[4+1];
  int val;

  if (s == 1 || s == 2)  {
    uint8_t *tmp = (uint8_t *)v;
    for (i=0; i<n; i++) {
      val = 0;
      for (j=0; j<s; j++) {
        val = (val << 4) | a2bhex_sub(*p++);
      }
      *tmp++ = val;
    }
  } else if (s == 3 || s == 4) {
    uint16_t *tmp = (uint16_t *)v;
    for (i=0; i<n; i++) {
      val = 0;
      for (j=0; j<s; j++) {
        val = (val << 4) | a2bhex_sub(*p++);
      }
      *tmp++ = val;
    }
  }
  return n;
}

// transfer messages from CAN bus to host
void xfer_can2tty()
{
  //tCAN msg;
  CanFrame rxframe2;
  char buf[CMD_LEN];
  int i;
  static uint16_t ts = 0;
  char *p;
  uint8_t len;

  while (ESP32Can.readFrame(rxframe2, 1))
  {
    p = buf;
    //if (msg.header.ide) {
    if(rxframe2.extd)  //if extended frame
    {
      //Serial.println("extended");
      //Serial.println(rxframe2.data_length_code);
      //Serial.println(rxframe2.identifier,HEX);
      //Serial.printf("test:%x\n\n",rxframe2.identifier);
      //if (msg.header.rtr)
      if (rxframe2.rtr)
      {
        *p++ = 'R';
      }
      else
      {
        *p++ = 'T';
      }
      //p += b2ahex(p, 4, 1, &msg.id);
      //p += b2ahex(p, 4, 1, &msg.ide);
      
      //p += b2ahex(p, 4, 2, &rxframe2.identifier);
       sprintf(p,"%02X",rxframe2.identifier);
       p+=8;
      
      len = rxframe2.data_length_code % 10;
      //Serial.println(len);
      p += b2ahex(p, 1, 1, &len);
    }
    else  //else non extended frame
    {
      if (rxframe2.rtr)
      {
        *p++ = 'r';
      }
      else
      {
        *p++ = 't';
      }
      p += b2ahex(p, 3, 1, &rxframe2.identifier);
      len = rxframe2.data_length_code;
      p += b2ahex(p, 1, 1, &len);
    }

//    p += b2ahex(p, 2, msg.header.length, msg.data);
    p += b2ahex(p, 2, rxframe2.data_length_code, rxframe2.data);
    // insert timestamp if needed
    if (g_ts_en) {
      p += b2ahex(p, 4, 1, &ts); // up to 60,000ms
      ts++;
    }

    *p++ = '\r';
    *p++ = '\0';
    Serial.print(buf);
  }
}

void slcan_ack()
{
  Serial.write('\r'); // ACK
}

void slcan_nack()
{
  Serial.write('\a'); // NACK
}

void send_canmsg(char *buf)
{
  //tCAN msg;
	CanFrame TxFrame = { 0 };
  int len = strlen(buf) - 1;
  uint16_t id[2];
  uint8_t hlen;
  int is_eff = buf[0] & 0x20 ? 0 : 1;
  int is_rtr = buf[0] & 0x02 ? 1 : 0;

  if (!is_eff && len >= 4) // SFF
  {
    a2bhex(&buf[1], 3, 1, id);
    TxFrame.identifier = id[0];   //msg.id = id[0];
    TxFrame.rtr = is_rtr;          //msg.header.rtr = is_rtr;
    TxFrame.extd = 0;             //msg.header.ide = 0;
    a2bhex(&buf[4], 1, 1, &hlen);
    TxFrame.data_length_code = hlen;     //msg.header.length = hlen;
    if (len - 4 - 1 == TxFrame.data_length_code * 2)
    {
      a2bhex(&buf[5], 2, TxFrame.data_length_code, TxFrame.data);
      ESP32Can.writeFrame(TxFrame,0);
      Serial.print("z");
    }
  }
  else if (is_eff && len >= 9) // EFF
  {
    a2bhex(&buf[1], 4, 2, id);
    TxFrame.identifier = id[0];
    TxFrame.identifier = TxFrame.identifier << 16;
    TxFrame.identifier |= id[1];
    TxFrame.rtr = is_rtr;
    TxFrame.extd = 1;                        //msg.header.ide = 1;
    a2bhex(&buf[9], 1, 1, &hlen);
    TxFrame.data_length_code = hlen;         //msg.header.length = hlen;
    if (len - 9 - 1 == TxFrame.data_length_code * 2)
    {
      a2bhex(&buf[10], 2, TxFrame.data_length_code, TxFrame.data);
      ESP32Can.writeFrame(TxFrame,0);
      Serial.print("Z");
    }
  }


/*
	TxFrame.identifier = 0x7DF; // Default OBD2 address;
	TxFrame.extd = 0;
	TxFrame.data_length_code = 8;
	TxFrame.data[0] = 2;
	TxFrame.data[1] = 1;
	TxFrame.data[2] = obdId;
	TxFrame.data[3] = 0xAA;    // Best to use 0xAA (0b10101010) instead of 0
	TxFrame.data[4] = 0xAA;    // CAN works better this way as it needs
	TxFrame.data[5] = 0xAA;    // to avoid bit-stuffing
	TxFrame.data[6] = 0xAA;
	TxFrame.data[7] = 0xAA;
    // Accepts both pointers and references 
    ESP32Can.writeFrame(TxFrame,0);  // timeout defaults to 1 ms
*/
}

void pars_slcancmd(char *buf)
{
  switch (buf[0]) {
    // common commands
    case 'O': // open channel
      if(ESP32Can.begin())
      {
        pixels.setPixelColor(0, pixels.Color(0, 150, 0)); //green = ok
        pixels.show();   // Send the updated pixel colors to the hardware.
      }
      else
      {
        pixels.setPixelColor(0, pixels.Color(150, 0, 0)); //red = error
        pixels.show();   // Send the updated pixel colors to the hardware.
      }
      slcan_ack();
      break;
    case 'C': // close channel
//      digitalWrite(LED_OPEN, LOW);
//      digitalWrite(LED_ERR, LOW);
      slcan_ack();
      break;
    case 't': // SFF
    case 'T': // EFF
    case 'r': // RTR/SFF
    case 'R': // RTR/EFF
      send_canmsg(buf);
      slcan_ack();
      break;
    case 'Z': // turn timestamp on/off
      if (buf[1] == '0') {
        g_ts_en = 0;
      } else if (buf[1] == '1') {
        g_ts_en = 1;
      } else {
        slcan_nack();
      }
      slcan_ack();
      break;
    case 'M': // acceptance mask
      slcan_ack();
      break;
    case 'm': // acceptance value
      slcan_ack();
      break;

    // non-standard commands
    case 'S': // setup CAN bit-rates
      switch (buf[1])
      {
        case '0': // 10k
        case '1': // 20k
        case '2': // 50k
          slcan_nack();
          break;
        case '3': // 100k
          g_can_speed = 100;
          ESP32Can.setSpeed(ESP32Can.convertSpeed(100));
          slcan_ack();
          break;
        case '4': // 125k
          g_can_speed = 125;
          ESP32Can.setSpeed(ESP32Can.convertSpeed(125));
          slcan_ack();
          break;
        case '5': // 250k
          g_can_speed = 250;
          ESP32Can.setSpeed(ESP32Can.convertSpeed(250));
          slcan_ack();
          break;
        case '6': // 500k
          g_can_speed = 500;
          ESP32Can.setSpeed(ESP32Can.convertSpeed(500));
          slcan_ack();
          break;
        case '7': // 800k
          g_can_speed = 800;
          ESP32Can.setSpeed(ESP32Can.convertSpeed(800));
          slcan_ack();
          break;
        case '8': // 1000k
          g_can_speed = 1000;
          ESP32Can.setSpeed(ESP32Can.convertSpeed(1000));
          slcan_ack();
          break;
        default:
          slcan_nack();
          break;
      }
      break;
    case 's': // directly set bitrate register of mcp2515
      slcan_nack();
      break;
    case 'F': // status flag
      Serial.print("F12");
      slcan_ack();
      break;
    case 'V': // hw/sw version
      Serial.print("V1234");
      slcan_ack();
      break;
    case 'N': // serial number
      Serial.print("N1234");
      slcan_ack();
      break;
    default: // unknown command
      slcan_nack();
      break;
  }
}

// transfer messages from host to CAN bus
void xfer_tty2can()
{
  int length;
  static char cmdbuf[CMD_LEN];
  static int cmdidx = 0;

  if ((length = Serial.available()) > 0) {
    for (int i = 0; i < length; i++) {
      char val = Serial.read();
      cmdbuf[cmdidx++] = val;

      if (cmdidx == CMD_LEN) { // command is too long
        slcan_nack();
        cmdidx = 0;
      } else if (val == '\r') { // end of command
        cmdbuf[cmdidx] = '\0';
        pars_slcancmd(cmdbuf);
        cmdidx = 0;
      }
    }
  }
}


//--------------------------------


void sendObdFrame(uint8_t obdId)
{
	CanFrame obdFrame = { 0 };
	obdFrame.identifier = 0x7DF; // Default OBD2 address;
	obdFrame.extd = 0;
	obdFrame.data_length_code = 8;
	obdFrame.data[0] = 2;
	obdFrame.data[1] = 1;
	obdFrame.data[2] = obdId;
	obdFrame.data[3] = 0xAA;    // Best to use 0xAA (0b10101010) instead of 0
	obdFrame.data[4] = 0xAA;    // CAN works better this way as it needs
	obdFrame.data[5] = 0xAA;    // to avoid bit-stuffing
	obdFrame.data[6] = 0xAA;
	obdFrame.data[7] = 0xAA;
    // Accepts both pointers and references 
    ESP32Can.writeFrame(obdFrame,0);  // timeout defaults to 1 ms
}

void setup()
{

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  
  pixels.setPixelColor(0, pixels.Color(150, 0, 0));
  pixels.show();   // Send the updated pixel colors to the hardware.
  delay(1000);
  pixels.setPixelColor(0, pixels.Color(0, 150, 0));
  pixels.show();   // Send the updated pixel colors to the hardware.
  delay(1000);
  pixels.setPixelColor(0, pixels.Color(0, 0, 150));
  pixels.show();   // Send the updated pixel colors to the hardware.
  delay(1000);
  pixels.clear(); // Set all pixel colors to 'off'
  pixels.show();   // Send the updated pixel colors to the hardware.

  // Setup serial for debbuging.
  Serial.begin(1000000);

  // Set pins
	ESP32Can.setPins(CAN_TX, CAN_RX);
	
  // You can set custom size for the queues - those are default
  ESP32Can.setRxQueueSize(5);
	ESP32Can.setTxQueueSize(5);

  // .setSpeed() and .begin() functions require to use TwaiSpeed enum,
  // but you can easily convert it from numerical value using .convertSpeed()
  ESP32Can.setSpeed(ESP32Can.convertSpeed(500));

  // You can also just use .begin()..
  if(ESP32Can.begin())
  {
    pixels.setPixelColor(0, pixels.Color(0, 150, 0)); //green = ok
    pixels.show();   // Send the updated pixel colors to the hardware.
  }
  else
  {
    pixels.setPixelColor(0, pixels.Color(150, 0, 0)); //red = error
    pixels.show();   // Send the updated pixel colors to the hardware.
  }

  // or override everything in one command;
  // It is also safe to use .begin() without .end() as it calls it internally
  //if(ESP32Can.begin(ESP32Can.convertSpeed(500), CAN_TX, CAN_RX, 10, 10))
  //{
  //    Serial.println("CAN bus started!");
  //}
  //else
  //{
  //  Serial.println("CAN bus failed!");
  // }
}

void loop()
{
    static uint32_t lastStamp = 0;
    uint32_t currentStamp = millis();

  xfer_can2tty();
  xfer_tty2can();



    //if(currentStamp - lastStamp > 1000) {   // sends OBD2 request every second
    //    lastStamp = currentStamp;
    //    sendObdFrame(5); // For coolant temperature
    //}

    // You can set custom timeout, default is 1000
   // if(ESP32Can.readFrame(rxFrame, 1000)) {
   //     // Comment out if too many frames
   //     Serial.printf("Received frame: %03X  \r\n", rxFrame.identifier);
   //     if(rxFrame.identifier == 0x7E8) {   // Standard OBD2 frame responce ID
   //         Serial.printf("Collant temp: %3d°C \r\n", rxFrame.data[3] - 40); // Convert to °C
   //     }
   // }
}