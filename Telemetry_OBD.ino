/** OBD-II Data gathering application
 *  Dan Stingaciu - 500765542
 *  Gathers Engine RPM, speed and fuel intake levels
 *  
 *  C. 2021
 */
#include <SPI.h>
#include "mcp2515_can.h"
#define CAN2515

//Initialize SPI and CAN pins
const int SPI_CS_PIN = 9;
const int CAN_INIT_PIN = 2;

// Setting MCP 2515 CS PIN
mcp2515_can CAN(SPI_CS_PIN);

// Define PIDs we want to gather
#define PID_ENGINE_RPM    0x0C
#define PID_CAR_SPEED     0x0D
#define PID_THROTTLE_POS  0x11
#define PID_AMBIENT_TEMP  0x46
#define PID_FUEL_LEVEL    0x2F

unsigned char dead_buf[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

// Define CAN system PID
#define CAN_ID_PID        0x7DF

unsigned char getPid = 0;

void set_mask_filt() {
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

void sendPid(unsigned char __pid) {
    unsigned char tmp[8] = {0x02, 0x01, __pid, 0, 0, 0, 0, 0};
    SERIAL_PORT_MONITOR.print("SEND PID: 0x");
    SERIAL_PORT_MONITOR.println(__pid, HEX);
    CAN.sendMsgBuf(CAN_ID_PID, 0, 8, tmp);
}

void setup() {
  // put your setup code here, to run once:
  SERIAL_PORT_MONITOR.begin(115200);
  while(!Serial){
    SERIAL_PORT_MONITOR.println("No Serial..."); 
  }

  while (CAN.begin(CAN_500KBPS)){
    SERIAL_PORT_MONITOR.println("CAN init fail, retrying...");
    delay(500);
  }

  SERIAL_PORT_MONITOR.println("CAN init successful!");
  set_mask_filt();

}

void loop() {
  unsigned char buf[8];
  int __rpm = 0, __speed = 0, __throttle = 0;
  
  // Send RPM Request
  sendPid(PID_ENGINE_RPM);
  delay(500); //Wait a bit
  getDataBuf(buf); //Get data

  if(buf[0] != 0x0) {
    extractRPM(buf, &__rpm);
  }

  SERIAL_PORT_MONITOR.print("ENGINE RPM: ");
  SERIAL_PORT_MONITOR.println(__rpm);
  
  // Send Car speed request
  sendPid(PID_CAR_SPEED);
  delay(500); //Wait a bit
  getDataBuf(buf); //Get Data

  if(buf[0] != 0x0) {
    extractVehicleSpeed(buf, &__speed);
  }

  SERIAL_PORT_MONITOR.print("VEHICLE SPEED: ");
  SERIAL_PORT_MONITOR.println(__speed);

  // Send throttle position request
  sendPid(PID_THROTTLE_POS);
  delay(500); //Wait a bit
  getDataBuf(buf); //Get data

  if(buf[0] != 0x0) {
    extractThrottlePos(buf, &__throttle);
  }

  SERIAL_PORT_MONITOR.print("THROTTLE POS: ");
  SERIAL_PORT_MONITOR.println(__throttle);

  SERIAL_PORT_MONITOR.println(String(__rpm)+","+String(__speed)+","+String(__throttle));
}

bool extractRPM(unsigned char *buf, int *rpm) {
  if(buf[1] == 0x41){
    *rpm = (256 * buf[3] * buf[4]) / 4;
    return 1;
  }
  return 0;
}

bool extractVehicleSpeed(unsigned char *buf, int *vehicle_speed) {
  if(buf[1] == 0x41){
    *vehicle_speed = buf[3];
    return 1;
  }
  return 0;
}

bool extractThrottlePos(unsigned char *buf, int *throttle) {
  if(buf[1] == 0x41){
    *throttle = 100/255 * buf[3];
    return 1;
  }
  return 0;
}

// Extract data frame from request
void getDataBuf(unsigned char *buf) {
  unsigned char len = 0;
  unsigned long __timeout = millis();

  //500 ms time out
  while(millis() - __timeout < 500){
    unsigned long id = 0;
    
    // Check that data has come in
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
      CAN.readMsgBuf(&len, buf); //Read data and length of data
    }
  }

  buf = dead_buf;
}
