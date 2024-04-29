#include <SPI.h>
#include <mcp_can.h>
#include "Timer.h"


// 定義感測器 CS 腳位
#define MLX90393_1 4
#define MLX90393_2 12
#define MLX90393_3 6
#define MLX90393_4 8
#define CAN_CS 9

// 定義其他用途之腳位
#define CAN_INT 0
#define LED4 10
#define LED5 5
#define LED6 13

/** Command List. */
#define MLX90393_REG_SB 0x10           /**< Start burst mode. */
#define MLX90393_REG_SW 0x20           /**< Start wakeup on change mode. */
#define MLX90393_REG_SM 0x30           /**> Start single-meas mode. */
#define MLX90393_REG_RM 0x40           /**> Read measurement . */
#define MLX90393_REG_RR 0x50           /**< Read register. */
#define MLX90393_REG_WR 0x60           /**< Write register. */
#define MLX90393_REG_EX 0x80           /**> Exit moode. */
#define MLX90393_REG_HR 0xD0           /**< Memory recall. */
#define MLX90393_REG_HS 0x70           /**< Memory store. */
#define MLX90393_REG_RT 0xF0           /**< Reset. */
#define MLX90393_REG_NOP 0x00          /**< NOP. */

/** Register map. */
#define MLX90393_DEFAULT_ADDR 0x0C     /* Can also be 0x18, depending on IC */
#define MLX90393_Measure_ALL 0x0F      /**< X+Y+Z axis + Temp bits for commands. */
#define MLX90393_CONF1 0x00            /**< Gain */
#define MLX90393_CONF2 0x01            /**< Burst, comm mode */
#define MLX90393_CONF3 0x02            /**< Oversampling, filter, res. */
#define MLX90393_CONF4 0x03            /**< Sensitivty drift. */
#define MLX90393_CONF5 0x04            /**< X axis drift. */
#define MLX90393_CONF6 0x05            /**< Y axis drift. */
#define MLX90393_CONF7 0x06            /**< Z axis drift. */
#define MLX90393_GAIN_SHIFT 4          /**< Left-shift for gain bits. */
#define MLX90393_HALL_CONF 0x0C        /**< Hall plate spinning rate adj. */
#define MLX90393_STATUS_OK 0x00        /**< OK value for status response. */
#define MLX90393_STATUS_SMMODE 0x08    /**< SM Mode status response. */
#define MLX90393_STATUS_RESET 0x01     /**< Reset value for status response. */
#define MLX90393_STATUS_ERROR 0xFF     /**< OK value for status response. */
#define MLX90393_STATUS_MASK 0xFC      /**< Mask for status OK checks. */

//初始化用中位數濾波器參數
#define FILTER_N 100

// 建立 CAN-Bus MCP2515 物件
MCP_CAN CAN(CAN_CS);

// 建立Timer物件
Timer Status_Timer;

// MLX90393 Raw DATA 變數
int32_t sensor_raw[12];
int32_t sensor_temp[4];

// 經卡爾曼濾波後變數
float sensor_kf[12];

// 初始校正平均值變數
int32_t sensor_init[12];


// 感測器收值狀態，true=正常、false=故障
bool sensor1_status = false;
bool sensor2_status = false;
bool sensor3_status = false;
bool sensor4_status = false;

// 當前MCU計時器時間
uint32_t previous_time = 0;
uint32_t acquisition_time = 0;
uint32_t acquisition_freq = 0;

// 讀取register用測試變數
uint16_t MLX_reg;

// SPI傳送封包delay時間參數
uint8_t interdelay = 10;


void setup() {

  // Serial.begin(115200);
  pinMode(CAN_INT, INPUT);
  pinMode(CAN_CS, OUTPUT);
  pinMode(MLX90393_1, OUTPUT);
  pinMode(MLX90393_2, OUTPUT);
  pinMode(MLX90393_3, OUTPUT);
  pinMode(MLX90393_4, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);

  digitalWrite(CAN_CS, HIGH);
  digitalWrite(MLX90393_1, HIGH);
  digitalWrite(MLX90393_2, HIGH);
  digitalWrite(MLX90393_3, HIGH);
  digitalWrite(MLX90393_4, HIGH);
  digitalWrite(LED4, HIGH);
  digitalWrite(LED5, HIGH);
  digitalWrite(LED6, HIGH);
  delay(10);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Succefully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN.setMode(MCP_NORMAL);

  Serial.println("Starting MLX90393 Demo");

  Status_Timer.every(50, Timer_Print);

  SPI.begin();
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
  delay(1000);

  initMLX(MLX90393_1);
  initMLX(MLX90393_2);
  initMLX(MLX90393_3);
  initMLX(MLX90393_4);

  setGain(MLX90393_1, 7);
  setGain(MLX90393_2, 7);
  setGain(MLX90393_3, 7);
  setGain(MLX90393_4, 7);

  setTCMP(MLX90393_1, 1);
  setTCMP(MLX90393_2, 1);
  setTCMP(MLX90393_3, 1);
  setTCMP(MLX90393_4, 1);

  // 解析度選擇 x,y,z 0=16, 1=17, 2=18, 3=19;
  setRes(MLX90393_1, 3, 3, 2);
  setRes(MLX90393_2, 3, 3, 2);
  setRes(MLX90393_3, 3, 3, 2);
  setRes(MLX90393_4, 3, 3, 2);

  setFilter(MLX90393_1, 4);
  setFilter(MLX90393_2, 4);
  setFilter(MLX90393_3, 4);
  setFilter(MLX90393_4, 4);

  setOSR(MLX90393_1, 2, 2);
  setOSR(MLX90393_2, 2, 2);
  setOSR(MLX90393_3, 2, 2);
  setOSR(MLX90393_4, 2, 2);

  read_test(MLX90393_1, MLX90393_CONF1);
  read_test(MLX90393_2, MLX90393_CONF1);
  read_test(MLX90393_3, MLX90393_CONF1);
  read_test(MLX90393_4, MLX90393_CONF1);

  // 模式選擇 1=BM, 2=WOC, 3=SM
  setMODE(MLX90393_1, 1);
  setMODE(MLX90393_2, 1);
  setMODE(MLX90393_3, 1);
  setMODE(MLX90393_4, 1);

  // 感測器數值初始化，歸零
  MLX_avg();
}

void loop() {
  // 讀取感測器開始點，時間戳記
  previous_time = micros();
  readValue();

  // 讀取感測器結束點，時間戳記
  acquisition_time = micros() - previous_time;
  // 計算採集頻率
  acquisition_freq = 1000000 / acquisition_time;

  //  Serial.print(acquisition_time);
  //  Serial.print(",");
  //  Serial.print(acquisition_freq);
  //  Serial.print(",");

  Status_Timer.update();

}


//初始化MLX90393數值，回到原點(歸零)
void MLX_avg() {
  for (int k = 0; k <= 11; k++) {
    int32_t filter_temp, filter_sum = 0;
    int32_t filter_buf[FILTER_N];
    for (int i = 0; i < FILTER_N; i++) {
      readValue();
      filter_buf[i] = sensor_raw[k];
    }
    // 採樣值從小到大排序(泡沫排序)
    for (int j = 0; j < FILTER_N - 1; j++) {
      for (int i = 0; i < FILTER_N - 1 - j; i++) {
        if (filter_buf[i] > filter_buf[i + 1]) {
          filter_temp = filter_buf[i];
          filter_buf[i] = filter_buf[i + 1];
          filter_buf[i + 1] = filter_temp;
        }
      }
    }
    // 去除最大最小極值後求平均值
    for (int i = 1; i < FILTER_N - 1; i++)
      filter_sum += filter_buf[i];

    sensor_init[k] = filter_sum / (FILTER_N - 2);
  }
}

void readValue() {

  //讀取所有感測器 X, Y, Z, T Raw DATA
  readMeasure(MLX90393_1, &sensor_raw[0], &sensor_raw[1], &sensor_raw[2], &sensor_temp[0]);
  readMeasure(MLX90393_2, &sensor_raw[3], &sensor_raw[4], &sensor_raw[5], &sensor_temp[1]);
  readMeasure(MLX90393_3, &sensor_raw[6], &sensor_raw[7], &sensor_raw[8], &sensor_temp[2]);
  readMeasure(MLX90393_4, &sensor_raw[9], &sensor_raw[10], &sensor_raw[11], &sensor_temp[3]);

  //所有感測器歸零、校正
  for (int i; i <= 11; i++)
    sensor_raw[i] = sensor_raw[i] - sensor_init[i];
}

void Timer_Print() {
  Print_MLX_raw_data();
  //Print_raw_Only(2);
}

//印出所有MLX資料副程式
void Print_MLX_raw_data()
{
  //Serial.print(acquisition_time);
  for (int i = 0; i <= 11; i++) {
    Serial.print(sensor_raw[i]);
    if (i < 11)
      Serial.print(",");
  }
  for (int i = 0; i <= 3; i++) {
    Serial.print(",");
    Serial.print(sensor_temp[i]);
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
      if (i < 9)
        Serial.print(",");
    }
  }
  if (sensor_axis == 1)
  {
    for (int i = 1; i <= 10; i += 3)
    {
      Serial.print(sensor_raw[i]);
      if (i < 9)
        Serial.print(",");
    }
  }
  if (sensor_axis == 2)
  {
    for (int i = 2; i <= 11; i += 3)
    {
      Serial.print(sensor_raw[i]);
      if (i < 9)
        Serial.print(",");
    }
  }
  Serial.println();
}

bool initMLX(uint8_t sensor) {
  uint8_t stat;

  //執行Exit Mode
  digitalWrite(sensor, LOW);
  stat = SPI.transfer(MLX90393_REG_EX);
  digitalWrite(sensor, HIGH);
  delay(100);

  //執行Reset Mode
  digitalWrite(sensor, LOW);
  stat &= SPI.transfer(MLX90393_REG_RT);
  digitalWrite(sensor, HIGH);
  delay(100);

  //COMM_MODE + BURST_SEL設定
  writeRegister(sensor, MLX90393_CONF2, 0x43C0);

  if (stat == MLX90393_STATUS_ERROR)
    return true;
  return false;
}

bool readRegister(uint8_t sensor, uint16_t reg, uint16_t *data) {
  uint8_t stat;

  //讀取目前MLX90393暫存器中的資料
  digitalWrite(sensor, LOW);
  SPI.transfer(MLX90393_REG_RR);
  SPI.transfer(reg << 2);
  stat = SPI.transfer(0x00);
  *data = SPI.transfer(0x00);
  *data = (*data << 8) | SPI.transfer(0x00);
  digitalWrite(sensor, HIGH);
  delay(interdelay);

  if (stat == MLX90393_STATUS_OK)
    return true;
  return false;
}

bool writeRegister(uint8_t sensor, uint16_t reg, uint16_t data) {
  uint8_t stat;

  //寫入資料到暫存器
  digitalWrite(sensor, LOW);
  SPI.transfer(MLX90393_REG_WR);
  SPI.transfer(data >> 8);
  SPI.transfer(data);
  stat = SPI.transfer(reg << 2);
  digitalWrite(sensor, HIGH);
  delay(interdelay);

  if (stat == MLX90393_STATUS_OK)
    return true;
  return false;
}

bool setGain(uint8_t sensor, uint16_t gain) {
  uint8_t stat;
  uint16_t data;

  //讀取目前MLX90393暫存器中的資料
  readRegister(sensor, MLX90393_CONF1, &data);

  //處理要傳送的資料
  data &= 0xFF8F;
  data |= (gain << 4);

  //寫入已改寫gain的暫存器資料
  writeRegister(sensor, MLX90393_CONF1, data);

  if (stat == MLX90393_STATUS_OK)
    return true;
  return false;
}

bool setTCMP(uint8_t sensor, bool en) {
  uint8_t stat;
  uint16_t data;

  //讀取目前MLX90393暫存器中的資料
  readRegister(sensor, MLX90393_CONF2, &data);

  //處理要傳送的資料
  data &= 0xFBFF;
  data |= (en << 10);

  //寫入已改寫TCMP_EN的暫存器資料
  writeRegister(sensor, MLX90393_CONF2, data);

  if (stat == MLX90393_STATUS_OK)
    return true;
  return false;
}

bool setRes(uint8_t sensor, uint16_t x_res, uint16_t y_res, uint16_t z_res) {
  uint8_t stat;
  uint16_t data;

  //讀取目前MLX90393暫存器中的資料
  readRegister(sensor, MLX90393_CONF3, &data);

  //處理要傳送的資料
  data &= 0xF81F;
  data |= (x_res << 5 | y_res << 7 | z_res << 9);

  //寫入已改寫Resolutsion的暫存器資料
  writeRegister(sensor, MLX90393_CONF3, data);

  if (stat == MLX90393_STATUS_OK)
    return true;
  return false;

}

bool setFilter(uint8_t sensor, uint16_t filter) {
  uint8_t stat;
  uint16_t data;

  //讀取目前MLX90393暫存器中的資料
  stat = readRegister(sensor, MLX90393_CONF3, &data);

  //處理要傳送的資料
  data &= 0xFFE3;
  data |= (filter << 2);

  //寫入已改寫Filter的暫存器資料
  stat &= writeRegister(sensor, MLX90393_CONF3, data);

  if (stat == MLX90393_STATUS_OK)
    return true;
  return false;

}

bool setOSR(uint8_t sensor, uint16_t osr1, uint16_t osr2) {
  uint8_t stat;
  uint16_t data;

  //讀取目前MLX90393暫存器中的資料
  stat = readRegister(sensor, MLX90393_CONF3, &data);

  //處理要傳送的資料
  data &= 0xE7FC;
  data |= (osr1 | (osr2 << 11));

  //寫入已改寫OSR、OSR2的暫存器資料
  stat &= writeRegister(sensor, MLX90393_CONF3, data);

  if (stat == MLX90393_STATUS_OK)
    return true;
  return false;

}

bool setMODE(uint8_t sensor, uint16_t mode) {
  uint8_t stat;
  uint16_t data;

  //處理要傳送的模式
  data = (mode << 4) | 0x0F;

  //執行Start Mode
  digitalWrite(sensor, LOW);
  stat = SPI.transfer(data);
  digitalWrite(sensor, HIGH);
  delay(100);

  if (stat == MLX90393_STATUS_ERROR)
    return true;
  return false;
}

bool readMeasure(uint8_t sensor, int32_t *x, int32_t *y, int32_t *z, int32_t *t) {
  uint8_t stat;

  //執行感測器資料讀取 x,y,z,t
  digitalWrite(sensor, LOW);
  SPI.transfer(MLX90393_REG_RM | MLX90393_Measure_ALL);
  stat = SPI.transfer(0x00);
  *t = SPI.transfer(0x00);
  *t = (*t << 8) | SPI.transfer(0x00);
  *x = SPI.transfer(0x00);
  *x = (*x << 8) | SPI.transfer(0x00);
  *y = SPI.transfer(0x00);
  *y = (*y << 8) | SPI.transfer(0x00);
  *z = SPI.transfer(0x00);
  *z = (*z << 8) | SPI.transfer(0x00);
  digitalWrite(sensor, HIGH);

  if (stat == MLX90393_STATUS_OK)
    return true;
  return false;
}

bool read_test(uint8_t sensor, uint16_t reg) {
  uint8_t stat;
  uint16_t data;

  //讀取目前MLX90393暫存器中的資料
  digitalWrite(sensor, LOW);
  SPI.transfer(MLX90393_REG_RR);
  SPI.transfer(reg << 2);
  stat = SPI.transfer(0x00);
  data = SPI.transfer(0x00);
  data = (data << 8) | SPI.transfer(0x00);
  digitalWrite(sensor, HIGH);
  delay(interdelay);

  Serial.println(data, BIN);

  if (stat == MLX90393_STATUS_ERROR)
    return true;
  return false;
}
