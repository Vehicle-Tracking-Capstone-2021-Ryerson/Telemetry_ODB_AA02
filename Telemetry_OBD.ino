/** OBD-II Data gathering application
 *  Dan Stingaciu - 500765542
 *  Gathers Engine RPM, speed and fuel intake levels
 *
 *  C. 2021
 */
#include <SPI.h>
#include "mcp2515_can.h"
#define CAN2515

// Initialize SPI and CAN pins
const int SPI_CS_PIN = 9;
const int CAN_INIT_PIN = 2;

// Setting MCP 2515 CS PIN
mcp2515_can CAN(SPI_CS_PIN);

// Define PIDs we want to gather
#define PID_ENGINE_RPM 0x0C
#define PID_CAR_SPEED 0x0D
#define PID_THROTTLE_POS 0x45
#define PID_AMBIENT_TEMP 0x46
#define PID_FUEL_LEVEL 0x2F

unsigned char dead_buf[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

// Define CAN system PID
#define CAN_ID_PID 0x7DF

unsigned char getPid = 0;

void set_mask_filt()
{
  /*
      set mask, set both the mask to 0x3ff
  */
  CAN.init_Mask(0, 0, 0x7FC);
  CAN.init_Mask(1, 0, 0x7FC);

  /*
      set filter, we can receive id from 0x04 ~ 0x09

      MIGHT NEED TO UPDATE TO COLLECT ADDITIONAL INFORMATION!
  */
  CAN.init_Filt(0, 0, 0x7E8);
  CAN.init_Filt(1, 0, 0x7E8);

  CAN.init_Filt(2, 0, 0x7E8);
  CAN.init_Filt(3, 0, 0x7E8);
  CAN.init_Filt(4, 0, 0x7E8);
  CAN.init_Filt(5, 0, 0x7E8);
}

void sendPid(unsigned char __pid)
{
  unsigned char tmp[8] = {0x02, 0x01, __pid, 0, 0, 0, 0, 0};
  //SERIAL_PORT_MONITOR.print("SEND PID: 0x");
  //SERIAL_PORT_MONITOR.println(__pid, HEX);
  CAN.sendMsgBuf(CAN_ID_PID, 0, 8, tmp);
}

void setup()
{
  // put your setup code here, to run once:
  SERIAL_PORT_MONITOR.begin(115200);
  while (!Serial)
  {
    SERIAL_PORT_MONITOR.println("No Serial...");
  }

  while (CAN.begin(CAN_500KBPS))
  {
    SERIAL_PORT_MONITOR.println("CAN init fail, retrying...");
    delay(500);
  }

  SERIAL_PORT_MONITOR.println("CAN init successful!");
  set_mask_filt();
}

void loop()
{
  unsigned char buf1[8], buf2[8], buf3[8], buf4[8], buf5[8];
  char rpmStr[4], speedStr[4], throttleStr[4], airTempStr[4], fuelStr[4], printBuf[32];
  float __rpm = 0.0, __speed = 0.0, __throttle = 0.0, __airTemp = 0.0;
  float __fuel_level = 0.0;
  // Send RPM Request
  sendPid(PID_ENGINE_RPM);
  delay(100);        // Wait a bit
  getDataBuf(buf1); // Get data

  if (buf1[0] != 0x0)
  {
    __rpm = extractRPM(buf1);
  }
  
  dtostrf(__rpm, 4, 2, rpmStr);
  strcpy(printBuf, rpmStr);
  strcat(printBuf, ",");

  // SERIAL_PORT_MONITOR.print("ENGINE RPM: ");
  // SERIAL_PORT_MONITOR.println(__rpm);

  // Send Car speed request
  sendPid(PID_CAR_SPEED);
  delay(100);        // Wait a bit
  getDataBuf(buf2); // Get Data

  if (buf2[0] != 0x0)
  {
    __speed = extractVehicleSpeed(buf2);
  }

  dtostrf(__speed, 4, 2, speedStr);
  strcat(printBuf, speedStr);
  strcat(printBuf, ",");

  // SERIAL_PORT_MONITOR.print("VEHICLE SPEED: ");
  // SERIAL_PORT_MONITOR.println(__speed);

  // Send throttle position request
  sendPid(PID_THROTTLE_POS);
  delay(100);        // Wait a bit
  getDataBuf(buf3); // Get data

  if (buf3[0] != 0x0)
  {
    __throttle = extractThrottlePos(buf3);
  }

  dtostrf(__throttle, 4, 2, throttleStr);
  strcat(printBuf, throttleStr);
  strcat(printBuf, ",");


  // SERIAL_PORT_MONITOR.print("THROTTLE POS: ");
  // SERIAL_PORT_MONITOR.println(__throttle);

  // Send air temperature position request
  sendPid(PID_AMBIENT_TEMP);
  delay(100);        // Wait a bit
  getDataBuf(buf4); // Get data

  if (buf4[0] != 0x0)
  {
    __airTemp = extractAmbientTemp(buf4);
  }

  dtostrf(__airTemp, 4, 2, airTempStr);
  strcat(printBuf, airTempStr);
  strcat(printBuf, ",");

  // SERIAL_PORT_MONITOR.print("AIR TEMPERATURE: ");
  // SERIAL_PORT_MONITOR.println(__airTemp);

  // Send throttle position request
  sendPid(PID_FUEL_LEVEL);
  delay(100);      // Wait a bit
  getDataBuf(buf5); // Get data

  if (buf5[0] != 0x0)
  {
    __fuel_level = extractFuelLevel(buf5);
  }

  dtostrf(__fuel_level, 4, 2, fuelStr);
  strcat(printBuf, fuelStr);

  // SERIAL_PORT_MONITOR.print("FUEL LEVEL: ");
  // SERIAL_PORT_MONITOR.println(__fuel_level);
  Serial.println(printBuf);
  delay(1000);
}

float extractRPM(unsigned char *buf)
{
  float rpm;
  if (buf[1] == 0x41)
  {
    float pre = 256 * buf[3] + buf[4];
    rpm = pre / 4;
    return rpm;
  }
  return -1;
}

float extractVehicleSpeed(unsigned char *buf)
{
  float vehicle_speed;
  if (buf[1] == 0x41)
  {
    vehicle_speed = (int) buf[3];
    return vehicle_speed;
  }
  return -1;
}

float extractThrottlePos(unsigned char *buf)
{
  float throttle;
  if (buf[1] == 0x41)
  {
    throttle = 0.39 * buf[3];
    return throttle;
  }
  return -1;
}

float extractAmbientTemp(unsigned char *buf)
{
  float temp;
  if (buf[1] == 0x41)
  {
    temp = buf[3] - 40;
    return temp;
  }
  return -1;
}

float extractFuelLevel(unsigned char *buf)
{
  float fuel;
  if (buf[1] == 0x41)
  {
    double mult = 0.39216;
    fuel = buf[3] * mult;
    return fuel;
  }
  return -1;
}

// Extract data frame from request
void getDataBuf(unsigned char *buf)
{
  unsigned char len = 0;
  unsigned long __timeout = millis();

  // 500 ms time out
  while (millis() - __timeout < 500)
  {
    unsigned long id = 0;

    // Check that data has come in
    if (CAN_MSGAVAIL == CAN.checkReceive())
    {
      CAN.readMsgBuf(&len, buf); // Read data and length of data
    }
  }

  buf = dead_buf;
}
