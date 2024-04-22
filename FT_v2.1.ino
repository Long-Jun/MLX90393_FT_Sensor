#include "Adafruit_MLX90393.h"
#include <SPI.h>
#include <mcp_can.h>


Adafruit_MLX90393 sensor1 = Adafruit_MLX90393();
Adafruit_MLX90393 sensor2 = Adafruit_MLX90393();
Adafruit_MLX90393 sensor3 = Adafruit_MLX90393();
Adafruit_MLX90393 sensor4 = Adafruit_MLX90393();

#define MLX90393_CS_1 4
#define MLX90393_CS_2 12
#define MLX90393_CS_3 6
#define MLX90393_CS_4 8
#define CAN_CS 9
#define CAN_INT 0
#define LED4 10
#define LED5 5
#define LED6 13

MCP_CAN CAN(CAN_CS);     // Set CS to pin 10


void setup(void)
{
  //Serial.begin(115200);
  pinMode(CAN_INT, INPUT);
  pinMode(CAN_CS, OUTPUT);
  pinMode(MLX90393_CS_1, OUTPUT);
  pinMode(MLX90393_CS_2, OUTPUT);
  pinMode(MLX90393_CS_3, OUTPUT);
  pinMode(MLX90393_CS_4, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);

  digitalWrite(CAN_CS, HIGH);
  digitalWrite(MLX90393_CS_1, HIGH);
  digitalWrite(MLX90393_CS_2, HIGH);
  digitalWrite(MLX90393_CS_3, HIGH);
  digitalWrite(MLX90393_CS_4, HIGH);
  digitalWrite(LED4, HIGH);
  digitalWrite(LED5, HIGH);
  digitalWrite(LED6, HIGH);
  delay(10);

  delay(5000);
  
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN.setMode(MCP_NORMAL);


  Serial.println("Starting MLX90393 Demo");

  if (! sensor1.begin_SPI(MLX90393_CS_1)) {  // hardware SPI mode
    Serial.println("No sensor 1 found ... Please check your wiring?");
    while (1) {
      delay(10);
    }
  }

  if (! sensor2.begin_SPI(MLX90393_CS_2)) {  // hardware SPI mode
    Serial.println("No sensor 2 found ... Please check your wiring?");
    while (1) {
      delay(10);
    }
  }

  if (! sensor3.begin_SPI(MLX90393_CS_3)) {  // hardware SPI mode
    Serial.println("No sensor 3 found ... Please check your wiring?");
    while (1) {
      delay(10);
    }
  }

  if (! sensor4.begin_SPI(MLX90393_CS_4)) {  // hardware SPI mode
    Serial.println("No sensor 4 found ... Please check your wiring?");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Found All MLX90393 sensor");

  sensor1.setGain(MLX90393_GAIN_1X);
  // You can check the gain too
  Serial.print("Gain set to: ");
  switch (sensor1.getGain()) {
    case MLX90393_GAIN_1X: Serial.println("1 x"); break;
    case MLX90393_GAIN_1_33X: Serial.println("1.33 x"); break;
    case MLX90393_GAIN_1_67X: Serial.println("1.67 x"); break;
    case MLX90393_GAIN_2X: Serial.println("2 x"); break;
    case MLX90393_GAIN_2_5X: Serial.println("2.5 x"); break;
    case MLX90393_GAIN_3X: Serial.println("3 x"); break;
    case MLX90393_GAIN_4X: Serial.println("4 x"); break;
    case MLX90393_GAIN_5X: Serial.println("5 x"); break;
  }

  sensor2.setGain(MLX90393_GAIN_1X);
  // You can check the gain too
  Serial.print("Gain set to: ");
  switch (sensor2.getGain()) {
    case MLX90393_GAIN_1X: Serial.println("1 x"); break;
    case MLX90393_GAIN_1_33X: Serial.println("1.33 x"); break;
    case MLX90393_GAIN_1_67X: Serial.println("1.67 x"); break;
    case MLX90393_GAIN_2X: Serial.println("2 x"); break;
    case MLX90393_GAIN_2_5X: Serial.println("2.5 x"); break;
    case MLX90393_GAIN_3X: Serial.println("3 x"); break;
    case MLX90393_GAIN_4X: Serial.println("4 x"); break;
    case MLX90393_GAIN_5X: Serial.println("5 x"); break;
  }

  sensor3.setGain(MLX90393_GAIN_1X);
  // You can check the gain too
  Serial.print("Gain set to: ");
  switch (sensor3.getGain()) {
    case MLX90393_GAIN_1X: Serial.println("1 x"); break;
    case MLX90393_GAIN_1_33X: Serial.println("1.33 x"); break;
    case MLX90393_GAIN_1_67X: Serial.println("1.67 x"); break;
    case MLX90393_GAIN_2X: Serial.println("2 x"); break;
    case MLX90393_GAIN_2_5X: Serial.println("2.5 x"); break;
    case MLX90393_GAIN_3X: Serial.println("3 x"); break;
    case MLX90393_GAIN_4X: Serial.println("4 x"); break;
    case MLX90393_GAIN_5X: Serial.println("5 x"); break;
  }

  sensor4.setGain(MLX90393_GAIN_1X);
  // You can check the gain too
  Serial.print("Gain set to: ");
  switch (sensor4.getGain()) {
    case MLX90393_GAIN_1X: Serial.println("1 x"); break;
    case MLX90393_GAIN_1_33X: Serial.println("1.33 x"); break;
    case MLX90393_GAIN_1_67X: Serial.println("1.67 x"); break;
    case MLX90393_GAIN_2X: Serial.println("2 x"); break;
    case MLX90393_GAIN_2_5X: Serial.println("2.5 x"); break;
    case MLX90393_GAIN_3X: Serial.println("3 x"); break;
    case MLX90393_GAIN_4X: Serial.println("4 x"); break;
    case MLX90393_GAIN_5X: Serial.println("5 x"); break;
  }

  // Set resolution, per axis. Aim for sensitivity of ~0.3 for all axes.
  sensor1.setResolution(MLX90393_X, MLX90393_RES_17);
  sensor1.setResolution(MLX90393_Y, MLX90393_RES_17);
  sensor1.setResolution(MLX90393_Z, MLX90393_RES_16);

  // Set oversampling
  sensor1.setOversampling(MLX90393_OSR_3);

  // Set digital filtering
  sensor1.setFilter(MLX90393_FILTER_0);


  //------------------------
  // Set resolution, per axis. Aim for sensitivity of ~0.3 for all axes.
  sensor2.setResolution(MLX90393_X, MLX90393_RES_17);
  sensor2.setResolution(MLX90393_Y, MLX90393_RES_17);
  sensor2.setResolution(MLX90393_Z, MLX90393_RES_16);

  // Set oversampling
  sensor2.setOversampling(MLX90393_OSR_3);

  // Set digital filtering
  sensor2.setFilter(MLX90393_FILTER_0);


  //------------------------
  // Set resolution, per axis. Aim for sensitivity of ~0.3 for all axes.
  sensor3.setResolution(MLX90393_X, MLX90393_RES_17);
  sensor3.setResolution(MLX90393_Y, MLX90393_RES_17);
  sensor3.setResolution(MLX90393_Z, MLX90393_RES_16);

  // Set oversampling
  sensor3.setOversampling(MLX90393_OSR_3);

  // Set digital filtering
  sensor3.setFilter(MLX90393_FILTER_0);


  //------------------------
  // Set resolution, per axis. Aim for sensitivity of ~0.3 for all axes.
  sensor4.setResolution(MLX90393_X, MLX90393_RES_17);
  sensor4.setResolution(MLX90393_Y, MLX90393_RES_17);
  sensor4.setResolution(MLX90393_Z, MLX90393_RES_16);

  // Set oversampling
  sensor4.setOversampling(MLX90393_OSR_3);

  // Set digital filtering
  sensor4.setFilter(MLX90393_FILTER_0);



}

void loop(void) {
  float x1, y1, z1;
  float x2, y2, z2;
  float x3, y3, z3;
  float x4, y4, z4;


  // get X Y and Z data at once
  if (sensor1.readData(&x1, &y1, &z1)) {
    Serial.print(y1);
    Serial.print("\t");
    comm_can_set_rpm(1, x1);
    //    Serial.print(y1, 4);
    //    Serial.print(z1, 4);
  } else {
    Serial.println("Unable to read XYZ data from the sensor.");
  }

  // get X Y and Z data at once
  if (sensor2.readData(&x2, &y2, &z2)) {
    x2 = 1 - x2;
    Serial.print(x2);
    Serial.print("\t");
    //    Serial.print(y2, 4);
    //    Serial.print(z2, 4);
  } else {
    Serial.println("Unable to read XYZ data from the sensor.");
  }

  // get X Y and Z data at once
  if (sensor3.readData(&x3, &y3, &z3)) {
    y3 = 1 - y3;
    Serial.print(y3);
    Serial.print("\t");
    //    Serial.print(y3, 4);
    //    Serial.print(z3, 4);
  } else {
    Serial.println("Unable to read XYZ data from the sensor.");
  }

  // get X Y and Z data at once
  if (sensor4.readData(&x4, &y4, &z4)) {
    Serial.print(x4);
    //    Serial.print(y4, 4);
    //    Serial.print(z4, 4);
  } else {
    Serial.println("Unable to read XYZ data from the sensor.");
  }
  Serial.println();
  //delay(2);

}





//CAN-BUS 副程式組

void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}


// CAN commands
typedef enum {
  CAN_PACKET_SET_DUTY = 0,
  CAN_PACKET_SET_CURRENT,
  CAN_PACKET_SET_CURRENT_BRAKE,
  CAN_PACKET_SET_RPM,
  CAN_PACKET_SET_POS,
  CAN_PACKET_FILL_RX_BUFFER,
  CAN_PACKET_FILL_RX_BUFFER_LONG,
  CAN_PACKET_PROCESS_RX_BUFFER,
  CAN_PACKET_PROCESS_SHORT_BUFFER,
  CAN_PACKET_STATUS
} CAN_PACKET_ID;


void comm_can_set_duty(uint8_t controller_id, float duty) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)duty, &send_index);

  CAN.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), 1, send_index, buffer);
}

void comm_can_set_current(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)current, &send_index);

  CAN.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), 1, send_index, buffer);
}

void comm_can_set_current_brake(uint8_t controller_id, float current_brake) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)current_brake, &send_index);

  CAN.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), 1, send_index, buffer);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)rpm, &send_index);

  CAN.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), 1, send_index, buffer);
}

void comm_can_set_pos(uint8_t controller_id, float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)pos, &send_index);

  CAN.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), 1, send_index, buffer);
}
