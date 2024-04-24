#include <Adafruit_MLX90393.h>
#include <SPI.h>
#include <mcp_can.h>
#include <SimpleKalmanFilter.h>

// 定義感測器 CS 腳位
#define MLX90393_CS_1 4
#define MLX90393_CS_2 12
#define MLX90393_CS_3 6
#define MLX90393_CS_4 8
#define CAN_CS 9

// 定義其他用途之腳位
#define CAN_INT 0
#define LED4 10
#define LED5 5
#define LED6 13

// 建立 MLX90393 物件
Adafruit_MLX90393 sensor1 = Adafruit_MLX90393();
Adafruit_MLX90393 sensor2 = Adafruit_MLX90393();
Adafruit_MLX90393 sensor3 = Adafruit_MLX90393();
Adafruit_MLX90393 sensor4 = Adafruit_MLX90393();

// 建立 CAN-Bus MCP2515 物件
MCP_CAN CAN(CAN_CS);

//設定卡爾曼參數
const float mea = 20;
const float est = 10;
const float kf_q = 0.1;

// 建立卡爾曼濾波器物件與參數
SimpleKalmanFilter kf_0(mea, est, kf_q);
SimpleKalmanFilter kf_1(mea, est, kf_q);
SimpleKalmanFilter kf_2(mea, est, kf_q);
SimpleKalmanFilter kf_3(mea, est, kf_q);
SimpleKalmanFilter kf_4(mea, est, kf_q);
SimpleKalmanFilter kf_5(mea, est, kf_q);
SimpleKalmanFilter kf_6(mea, est, kf_q);
SimpleKalmanFilter kf_7(mea, est, kf_q);
SimpleKalmanFilter kf_8(mea, est, kf_q);
SimpleKalmanFilter kf_9(mea, est, kf_q);
SimpleKalmanFilter kf_10(mea, est, kf_q);
SimpleKalmanFilter kf_11(mea, est, kf_q);

// MLX90393 Raw DATA 變數
float sensor_raw[12];
float sensor_temp[4];

// 經卡爾曼濾波後變數
float sensor_kf[12];

// 感測器收值狀態，true=正常、false=故障
boolean sensor1_status = false;
boolean sensor2_status = false;
boolean sensor3_status = false;
boolean sensor4_status = false;

// 當前MCU計時器時間
int32_t previous_time = 0;
int16_t acquisition_time = 0;

// 所有 MLX90393 解析度設定 0=16, 1=17, 2=18, 3=19
uint8_t RES = 3;

// 所有 MLX90393 增益設定
uint8_t gain = 0;

// 所有 MLX90393 過採樣設定
uint8_t OSR = 0;

// 所有 MLX90393 濾波器設定
uint8_t Filter = 1;

void setup(void)
{
  //delay(5000);
  // Serial.begin(115200);
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

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN.setMode(MCP_NORMAL);

  Serial.println("Starting MLX90393 Demo");

  if (!sensor1.begin_SPI(MLX90393_CS_1))
  { // hardware SPI mode
    Serial.println("No sensor 1 found ... Please check your wiring?");
    while (1)
    {
      delay(10);
    }
  }

  if (!sensor2.begin_SPI(MLX90393_CS_2))
  { // hardware SPI mode
    Serial.println("No sensor 2 found ... Please check your wiring?");
    while (1)
    {
      delay(10);
    }
  }

  if (!sensor3.begin_SPI(MLX90393_CS_3))
  { // hardware SPI mode
    Serial.println("No sensor 3 found ... Please check your wiring?");
    while (1)
    {
      delay(10);
    }
  }

  if (!sensor4.begin_SPI(MLX90393_CS_4))
  { // hardware SPI mode
    Serial.println("No sensor 4 found ... Please check your wiring?");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("Found All MLX90393 sensor");

  // Reset 所有 MLX90393
  sensor1.reset();
  sensor2.reset();
  sensor3.reset();
  sensor4.reset();
  Serial.println("Reset All MLX90393");
  //delay(1000);

  //設定MLX初始化參數
  Set_MLX_Config();
}

void loop(void)
{

  Read_MLX_Value();
  MLX_Kalman_Filter();

  //Print_MLX_raw_data();

  //Print_MLX_KF_data();
  //Print_raw_Only(2);
  Print_kf_Only(2);
  //Print_Comparison(5);

}

// 讀取MLX90393副程式
void Read_MLX_Value()
{
  previous_time = millis();
  sensor1.readData(&sensor_raw[0], &sensor_raw[1], &sensor_raw[2]);
  for (int i = 0; i <= 2; i++) {
    sensor_raw[i] = sensor_raw[i] * 1.4;
  }
  sensor2.readData(&sensor_raw[3], &sensor_raw[4], &sensor_raw[5]);
  sensor3.readData(&sensor_raw[6], &sensor_raw[7], &sensor_raw[8]);
  sensor4.readData(&sensor_raw[9], &sensor_raw[10], &sensor_raw[11]);
  acquisition_time = previous_time - millis();
}

//卡爾曼濾波器副程式
void MLX_Kalman_Filter() {
  sensor_kf[0] = kf_0.updateEstimate(sensor_raw[0] + 50);
  sensor_kf[1] = kf_1.updateEstimate(sensor_raw[1] +146);
  sensor_kf[2] = kf_2.updateEstimate(sensor_raw[2] - 2945);
  sensor_kf[3] = kf_3.updateEstimate(sensor_raw[3] + 295);
  sensor_kf[4] = kf_4.updateEstimate(sensor_raw[4] + 245);
  sensor_kf[5] = kf_5.updateEstimate(sensor_raw[5] - 2760);
  sensor_kf[6] = kf_6.updateEstimate(sensor_raw[6] + 285);
  sensor_kf[7] = kf_7.updateEstimate(sensor_raw[7] + 130);
  sensor_kf[8] = kf_8.updateEstimate(sensor_raw[8] - 2977);
  sensor_kf[9] = kf_9.updateEstimate(sensor_raw[9] + 96);
  sensor_kf[10] = kf_10.updateEstimate(sensor_raw[10] + 221);
  sensor_kf[11] = kf_11.updateEstimate(sensor_raw[11] - 2917);
}

//印出所有MLX資料副程式
void Print_MLX_raw_data()
{
  Serial.print(acquisition_time);
  for (int i = 0; i <= 11; i++)
  {
    Serial.print(",");
    Serial.print(sensor_raw[i]);
  }
  Serial.println();
}

//印出所有MLX資料副程式
void Print_MLX_KF_data()
{
  for (int i = 0; i <= 11; i++)
  {
    Serial.print(",");
    Serial.print(sensor_kf[i]);
  }
  Serial.println();
}

//印出所有MLX的單軸資料副程式
void Print_raw_Only(int sensor_axis)
{
  if (sensor_axis == 0)
  {
    for (int i = 0; i <= 9; i += 3)
    {
      Serial.print(sensor_raw[i]);
      Serial.print(",");
    }
  }
  if (sensor_axis == 1)
  {
    for (int i = 1; i <= 10; i += 3)
    {
      Serial.print(sensor_raw[i]);
      Serial.print(",");
    }
  }
  if (sensor_axis == 2)
  {
    for (int i = 2; i <= 11; i += 3)
    {
      Serial.print(sensor_raw[i]);
      Serial.print(",");
    }
  }
  Serial.println();
}

//印出所有MLX的單軸卡爾曼資料副程式
void Print_kf_Only(int sensor_axis)
{
  if (sensor_axis == 0)
  {
    for (int i = 0; i <= 9; i += 3)
    {
      Serial.print(sensor_kf[i]);
      Serial.print(",");
    }
  }
  if (sensor_axis == 1)
  {
    for (int i = 1; i <= 10; i += 3)
    {
      Serial.print(sensor_kf[i]);
      Serial.print(",");
    }
  }
  if (sensor_axis == 2)
  {
    for (int i = 2; i <= 11; i += 3)
    {
      Serial.print(sensor_kf[i]);
      Serial.print(",");
    }
  }
  Serial.println();
}

//印出原始資料與卡爾曼比較副程式
void Print_Comparison(int sensor)
{
  Serial.print(sensor_raw[sensor]);
  Serial.print("\t");
  Serial.println(sensor_kf[sensor]);
}

//設定MLX初始化參數副程式
void Set_MLX_Config() {
  // Set resolution, per axis. Aim for sensitivity of ~0.3 for all axes.
  sensor1.setResolution(MLX90393_X, RES);
  sensor1.setResolution(MLX90393_Y, RES);
  sensor1.setResolution(MLX90393_Z, RES);

  // Set oversampling
  sensor1.setOversampling(OSR);

  // Set digital filtering
  sensor1.setFilter(4);

  // Set gain
  sensor1.setGain(gain);

  //------------------------
  // Set resolution, per axis. Aim for sensitivity of ~0.3 for all axes.
  sensor2.setResolution(MLX90393_X, RES);
  sensor2.setResolution(MLX90393_Y, RES);
  sensor2.setResolution(MLX90393_Z, RES);

  // Set oversampling
  sensor2.setOversampling(OSR);

  // Set digital filtering
  sensor2.setFilter(Filter);

  // Set gain
  sensor2.setGain(gain);

  //------------------------
  // Set resolution, per axis. Aim for sensitivity of ~0.3 for all axes.
  sensor3.setResolution(MLX90393_X, RES);
  sensor3.setResolution(MLX90393_Y, RES);
  sensor3.setResolution(MLX90393_Z, RES);

  // Set oversampling
  sensor3.setOversampling(OSR);

  // Set digital filtering
  sensor3.setFilter(Filter);

  // Set gain
  sensor3.setGain(gain);

  //------------------------
  // Set resolution, per axis. Aim for sensitivity of ~0.3 for all axes.
  sensor4.setResolution(MLX90393_X, RES);
  sensor4.setResolution(MLX90393_Y, RES);
  sensor4.setResolution(MLX90393_Z, RES);

  // Set oversampling
  sensor4.setOversampling(OSR);

  // Set digital filtering
  sensor4.setFilter(Filter);

  // Set gain
  sensor4.setGain(gain);
}


// CAN-BUS 副程式組
void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index)
{
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_uint16(uint8_t *buffer, uint16_t number, int32_t *index)
{
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index)
{
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_uint32(uint8_t *buffer, uint32_t number, int32_t *index)
{
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

// CAN commands
typedef enum
{
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

void comm_can_set_duty(uint8_t controller_id, float duty)
{
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)duty, &send_index);

  CAN.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), 1, send_index, buffer);
}

void comm_can_set_current(uint8_t controller_id, float current)
{
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)current, &send_index);

  CAN.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), 1, send_index, buffer);
}

void comm_can_set_current_brake(uint8_t controller_id, float current_brake)
{
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)current_brake, &send_index);

  CAN.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), 1, send_index, buffer);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm)
{
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)rpm, &send_index);

  CAN.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), 1, send_index, buffer);
}

void comm_can_set_pos(uint8_t controller_id, float pos)
{
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)pos, &send_index);

  CAN.sendMsgBuf(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), 1, send_index, buffer);
}
