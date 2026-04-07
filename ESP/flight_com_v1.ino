//external libraries
#include "ICM_20948.h"
#include <SPI.h>
#include <SdFat.h>
#include "Adafruit_BMP3XX.h"

//internal libraries
#include "matrix_functions.h"
#include "controlled_LKF.h"
#include "mahony.h" 

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))

#define PAD_PRESSURE 1015
#define GTOM 9.80665

#define SERIAL_PORT Serial
#define HSPI_MISO 17
#define HSPI_MOSI 21
#define HSPI_SCLK 22
#define VSPI_MISO 36
#define VSPI_MOSI 33
#define VSPI_SCLK 32
#define ICM_CS 5
#define BMP_CS 4
#define SD_CS 25
#define AD0_VAL 1

SPIClass hspi(HSPI);
SPIClass vspi(VSPI);
Adafruit_BMP3XX bmp;
ICM_20948_SPI icm;
SdFat sd;

typedef struct ICM_Data{
  float accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY, magZ, temp, alt, kfAlt, kfVel, mfX, mfY, mfZ, quatW, quatX, quatY, quatZ;
  int micros;
}ICM_Data;



QueueHandle_t dataQueue;

garbage_truck* g_man;

//matrix initialization
matrix measurement;
matrix measurement_uncertainty;
matrix input;
matrix observation;
matrix state;
matrix f_state;
matrix s_tran;
matrix cov;
matrix f_cov;
matrix gain;
matrix control;
matrix process_noise;
matrix identity;

ICM_Data currentReadings;

float bmp_var = 1;
float icm_var = 10;
float start_cov = 10;

float deltaT = 0;
int start;
int bmp_track;
int mag_track;
float prevX;
float alt = 0;
float acc_offsets[6] = {-19.38256836, -11.80657959, -5.44467163, 0.55716974, 0.17350864, 0.62226242};
float mag_offsets[6] = {-39.78832626, 42.44295120, 24.76499939, 1.02816880, 0.94117922, 1.03637683};
float angle = 0;

mahony_state ahrs_state;

void SD_Writer(void* params)
{
  sd.begin(SdSpiConfig(SD_CS, DEDICATED_SPI, SD_SCK_MHZ(4), &vspi));
  SdFile file;
  if(!file.open("data.csv", O_RDWR | O_CREAT | O_AT_END))
  {
    SERIAL_PORT.print("file access failed\n");
    while(1)
      delay(1000);
  }
  SERIAL_PORT.print("file access sucessful\n");
  ICM_Data received;
  int i = 0;
  int startTime = esp_timer_get_time();
  while(1)
  {
    if(xQueueReceive(dataQueue, &received, portMAX_DELAY))
    {
      file.print(received.micros - startTime);
      file.print(",");
      file.print(received.accX * GTOM);
      file.print(",");
      file.print(received.accY * GTOM);
      file.print(",");
      file.print(received.accZ * GTOM);
      file.print(",");
      file.print(received.gyrX);
      file.print(",");
      file.print(received.gyrY);
      file.print(",");
      file.print(received.gyrZ);
      file.print(",");
      file.print(received.magX);
      file.print(",");
      file.print(received.magY);
      file.print(",");
      file.print(received.magZ);
      file.print(",");
      file.print(received.alt);
      file.print(",");
      file.print(received.kfAlt);
      file.print(",");
      file.print(received.kfVel);
      file.print(",");
      file.print(received.mfX);
      file.print(",");
      file.print(received.mfY);
      file.print(",");
      file.print(received.mfZ);
      file.print(",");
      file.print(received.quatW);
      file.print(",");
      file.print(received.quatX);
      file.print(",");
      file.print(received.quatY);
      file.print(",");
      file.print(received.quatZ);
      file.print("\n");
      if(i++ % 1125 == 0)
      {
        file.flush();
        delay(1);
      }
    }
  }
}

void setup()
{
  SERIAL_PORT.begin(115200);
  
  dataQueue = xQueueCreate(20, sizeof(ICM_Data));
  hspi.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, -1);
  vspi.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, SD_CS);

  bool initialized = false;
  while (!initialized)
  {
    icm.begin(ICM_CS, hspi, 7000000);
    icm.enableDLPF(ICM_20948_Internal_Acc, true);
    icm.enableDLPF(ICM_20948_Internal_Gyr, true);
    ICM_20948_fss_t icm_fss; // Create a "Full Scale Settings" structure
    icm_fss.a = gpm16;       // Set acc range to 16g. Choices: gpm2, gpm4, gpm8, gpm16
    icm_fss.g = dps2000;     // Set gyro range (optional)
  
    icm.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), icm_fss);
    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(icm.statusString());
    if (icm.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  if (!bmp.begin_SPI(BMP_CS, &hspi, 7000000))
  {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
  }
  else
  {
    Serial.println("bmp initialized correctly");
  }
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
  bmp.setPressureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);

  xTaskCreatePinnedToCore(SD_Writer, "SDTask", 10000, NULL, 1, NULL, 0);

  state = m_new(1, 2);
  f_state = m_new(1, 2);
  cov = m_new(2, 2);
  f_cov = m_new(2, 2);
  process_noise = m_new(2, 2);
  measurement = m_new(1, 2);
  gain = m_new(2, 2);
  s_tran = m_new(2, 2);
  measurement_uncertainty = m_new(1, 1);
  identity = m_identity(2); 
  observation = m_new(2, 1);
  control = m_new(1, 2);
  input = m_new(1, 1);
  
  m_modify(&measurement_uncertainty, 0, 0, bmp_var);
  
  m_modify(&observation, 0, 0, 1);
  
  m_modify(&cov, 0, 0, start_cov);
  m_modify(&cov, 1, 1, start_cov);

  m_modify(&s_tran, 0, 0, 1);
  m_modify(&s_tran, 1, 1, 1);

  state_extrapolation(&f_state, &state, &s_tran, &control);
  covariance_extrapolation(&f_cov, &s_tran, &cov, &process_noise);

  ahrs_state.acc_err[0] = 0;
  ahrs_state.acc_err[1] = 0;
  ahrs_state.acc_err[2] = 0;
  
  ahrs_state.mag_err[0] = 0;
  ahrs_state.mag_err[1] = 0;
  ahrs_state.mag_err[2] = 0;

  ahrs_state.quat[0] = 1;
  ahrs_state.quat[1] = 0;
  ahrs_state.quat[2] = 0;
  ahrs_state.quat[3] = 0;
  
  start = (int)esp_timer_get_time();
  bmp_track = start;
  mag_track = start;
}

void loop()
{ 
  while(!icm.dataReady())
  {}
  icm.getAGMT();         //update icm values
  currentReadings.micros = esp_timer_get_time();
  
  currentReadings.accX = icm.accX() - acc_offsets[2];
  currentReadings.accY = icm.accY() - acc_offsets[0];
  currentReadings.accZ = icm.accZ() - acc_offsets[1];

  currentReadings.gyrX = icm.gyrX() - acc_offsets[5];
  currentReadings.gyrY = icm.gyrY() - acc_offsets[3];
  currentReadings.gyrZ = icm.gyrZ() - acc_offsets[4];

  currentReadings.magX = (icm.magX() - mag_offsets[0]) * mag_offsets[3];
  currentReadings.magY = (icm.magY() - mag_offsets[1]) * mag_offsets[4];
  currentReadings.magZ = (icm.magZ() - mag_offsets[2]) * mag_offsets[5];

  currentReadings.kfAlt = state.elems[0];
  currentReadings.kfVel = state.elems[1];

  float net_acc = sqrtf(currentReadings.accX * currentReadings.accX + currentReadings.accY * currentReadings.accY + currentReadings.accZ * currentReadings.accZ);

  //ahrs_state.acc_kp = -fabsf(2 * net_acc / 1000 - 2) + 1;
  //if(ahrs_state.acc_kp < 0)
    //ahrs_state.acc_kp = 0;
  ahrs_state.acc_kp = 1;
  ahrs_state.acc_ki = 0;
  ahrs_state.mag_kp = 1;
  ahrs_state.mag_ki = 0;

  ahrs_state.acc[0] = currentReadings.accX;
  ahrs_state.acc[2] = -currentReadings.accY;
  ahrs_state.acc[1] = currentReadings.accZ;

  ahrs_state.gyr[0] = currentReadings.gyrX * PI / 180;
  ahrs_state.gyr[2] = -currentReadings.gyrY * PI / 180;
  ahrs_state.gyr[1] = currentReadings.gyrZ * PI / 180;

  ahrs_state.mag[0] = -currentReadings.magX;
  ahrs_state.mag[2] = -currentReadings.magY;
  ahrs_state.mag[1] = currentReadings.magZ;


  m_modify(&s_tran, 1, 0, deltaT);

  m_modify(&control, 0, 0, (powf(deltaT, 2) / 2) * input.elems[0]);
  m_modify(&control, 0, 1, deltaT * input.elems[0]);

  m_modify(&process_noise, 0, 0, (powf(deltaT, 1) / 4) * icm_var);
  m_modify(&process_noise, 0, 1, (powf(deltaT, 1) / 2) * icm_var);
  m_modify(&process_noise, 1, 0, (powf(deltaT, 1) / 2) * icm_var);
  m_modify(&process_noise, 1, 1, powf(deltaT, 1) * icm_var);

  mahony_acc_update(&ahrs_state, deltaT);
  mahony_gyro_update(&ahrs_state, deltaT);

  float acc_out[3];

  currentReadings.mfX = acc_out[0];
  currentReadings.mfY = acc_out[2];
  currentReadings.mfZ = acc_out[1];

  currentReadings.quatW = ahrs_state.quat[0];
  currentReadings.quatX = ahrs_state.quat[1];
  currentReadings.quatY = ahrs_state.quat[2];
  currentReadings.quatZ = ahrs_state.quat[3];

  vector_from_quat(ahrs_state.quat, ahrs_state.acc, acc_out);

  m_modify(&input, 0, 0, ((acc_out[2] - 1000) * GTOM / 1000));

  state_extrapolation(&state, &state, &s_tran, &control);
  covariance_extrapolation(&cov, &s_tran, &cov, &process_noise);
  int current = esp_timer_get_time();

  if((current - mag_track) >= 10000)
  {
    mahony_mag_update(&ahrs_state, (float)(mag_track - current) / 1000000);
    mag_track = current;
  }
  
  if((current - bmp_track) >= 5000)
  {
    bmp_track = current;
    alt = bmp.readAltitude(PAD_PRESSURE);
    m_modify(&measurement, 0, 0, alt);
    update_gain(&gain, &cov, &observation, &measurement_uncertainty);
    if(gain.elems[1] > 0.05)
      gain.elems[1] = 0.05;
    if(gain.elems[0] > 0.5)
      gain.elems[0] = 0.5;
    state_update(&state, &state, &gain, &measurement, &observation);
    covariance_update(&cov, &identity, &gain, &observation, &cov, &measurement_uncertainty);
  }
  
  currentReadings.alt = alt;
  deltaT = (float)((int)esp_timer_get_time() - start) / 1000000.0;
  start = (int)esp_timer_get_time();

  xQueueSend(dataQueue, &currentReadings, 0);
}
