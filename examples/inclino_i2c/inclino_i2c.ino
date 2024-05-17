//  Wiring: Board->Sensor
//          SDA  -> SDA
//          SCL  -> SCL
//          3.3V -> VDD
//          GND  -> GND
//WARNING: Pull-up resistors needed at the SDA and SCL.

#include <Wire.h>
#include "iis2iclx_reg.h"

#define IIS2ICLX_DEFAULT_ADDRESS 0x6B
#define IIS2ICLX_OK              0
#define IIS2ICLX_ERROR          -1

// Define the device context structure
static stmdev_ctx_t dev_ctx;
static uint8_t whoamI, rst;
const float RAD2DEG = 180.0f / PI;
static float angle;

// Platform-specific write function for I2C communication
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  Wire.beginTransmission(IIS2ICLX_DEFAULT_ADDRESS);
  Wire.write(reg);
  Wire.write(bufp, len);
  if (Wire.endTransmission() != 0) {
    return IIS2ICLX_ERROR;
  }
  return IIS2ICLX_OK;
}

// Platform-specific read function for I2C communication
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  Wire.beginTransmission(IIS2ICLX_DEFAULT_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) {
    return IIS2ICLX_ERROR;
  }
  Wire.requestFrom(IIS2ICLX_DEFAULT_ADDRESS, len);
  for (uint16_t i = 0; i < len; i++) {
    bufp[i] = Wire.read();
  }
  return IIS2ICLX_OK;
}

void setup() {
  Wire.begin();

  Serial.begin(9600);
  while (!Serial);
  
  // Initialize sensor
  if (!initSensor()) {
    Serial.println("Failed to initialize sensor!");
    while (1);
  } else {
      Serial.print("\n\n\nDetected sensor ID: ");
      Serial.println(whoamI,HEX);
  }

}

void loop() {
  int16_t acceleration[2];
  float temperature;
  float angle = atan2(acceleration[1], acceleration[0]) * RAD2DEG;

  // Read acceleration data
  readAcceleration(acceleration);

  // Read temperature data
  temperature = readTemperatureC();

  // Print data
  Serial.print("Acceleration [mg]: ");
  Serial.print(acceleration[0]);
  Serial.print("x   ");
  Serial.print(acceleration[1]);
  Serial.print("y");
  Serial.print("       Degrees: ");
  Serial.print(angle);
  Serial.println("*");

  Serial.print("Temperature [*C]: ");
  Serial.println(temperature, 2);


  delay(500); // Adjust delay according to your requirements
}

bool initSensor() {
  // Initialize device context
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &Wire; // Use Wire for I2C communication


  if (iis2iclx_device_id_get(&dev_ctx, &whoamI) != IIS2ICLX_OK) {
    return false;
  }

  if (whoamI != IIS2ICLX_ID) {
    return false;
  }
 /* Restore default configuration */
  iis2iclx_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    iis2iclx_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  iis2iclx_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  iis2iclx_xl_data_rate_set(&dev_ctx, IIS2ICLX_XL_ODR_12Hz5);
  /* Set full scale */
  iis2iclx_xl_full_scale_set(&dev_ctx, IIS2ICLX_2g);
  /* Configure filtering chain(No aux interface)
   * Accelerometer - LPF1 + LPF2 path
   */
  iis2iclx_xl_hp_path_on_out_set(&dev_ctx, IIS2ICLX_LP_ODR_DIV_100);
  iis2iclx_xl_filter_lp2_set(&dev_ctx, PROPERTY_ENABLE);
  // You can optionally add initialization code here specific to your platform

  return true; // Success
}

void readAcceleration(int16_t* acceleration) {
  uint8_t reg;
  iis2iclx_xl_flag_data_ready_get(&dev_ctx, &reg);

  if (reg) {
    memset(acceleration, 0x00, 2 * sizeof(int16_t));
    iis2iclx_acceleration_raw_get(&dev_ctx, acceleration);
  }
}

float readTemperatureC() {
  uint8_t reg;
  iis2iclx_temp_flag_data_ready_get(&dev_ctx, &reg);

  if (reg) {
    int16_t raw_temp;
    iis2iclx_temperature_raw_get(&dev_ctx, &raw_temp);
    return iis2iclx_from_lsb_to_celsius(raw_temp);
  }

  return 0.0f;
}
