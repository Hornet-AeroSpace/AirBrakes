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

#define PAD_PRESSURE 1024.05
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
  float accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY, magZ, temp, alt, kfAlt, kfVel, mfX, mfY, mfZ;
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

float bmp_stddev = 1;
float icm_stddev = 10;
float start_cov = 500;

float deltaT = 0;
int start;
int track;
float prevX;
float alt = 0;
float* acc_offsets = NULL;
float* mag_offsets = NULL;
float angle = 0;

mahony_state ahrs_state;

void printICM(ICM_20948_SPI* sensor)
{
  SERIAL_PORT.print("ACC [");
  SERIAL_PORT.print(sensor->accX() * GTOM);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(sensor->accY() * GTOM);
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(sensor->accZ() * GTOM);
  SERIAL_PORT.print("] GYR [");
  SERIAL_PORT.print(sensor->gyrX());
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(sensor->gyrY());
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(sensor->gyrZ());
  SERIAL_PORT.print("] MAG [");
  SERIAL_PORT.print(sensor->magX());
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(sensor->magY());
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(sensor->magZ());
  SERIAL_PORT.print("] ICM_TEMP [");
  SERIAL_PORT.print(sensor->temp());
  SERIAL_PORT.print("]\n");
}

void printBMP(Adafruit_BMP3XX* sensor)
{
  SERIAL_PORT.print("BMP_TEMP [");
  SERIAL_PORT.print(bmp.temperature);
  SERIAL_PORT.print("] PRESS [");
  SERIAL_PORT.print(bmp.pressure);
  SERIAL_PORT.print("] BMP_ALT [");
  SERIAL_PORT.print(bmp.readAltitude(PAD_PRESSURE));
  SERIAL_PORT.print("]\n");
}

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
      file.print("\n");
      if(i++ % 1125 == 0)
      {
        SERIAL_PORT.println(i);
        file.flush();
        delay(1);
      }
    }
  }
}

float* cal_acc()
{
  float gyr[3];
  gyr[0] = 0;
  gyr[1] = 0;
  gyr[2] = 0;
  SERIAL_PORT.read();
  float* output = (float*)malloc(6 * sizeof(float));
  for(int i = 0; i < 6; i++)
    output[i] = 0;
  axis_calibration:
  SERIAL_PORT.println("enter a number corresponding to the axis you are calibrating\n1 == bottom, 2 == top, 3 == front, 4 == back, 5 == left, 6 == right, 0 == exit accelerometer calibration");
  while(SERIAL_PORT.available())
    SERIAL_PORT.read();
  SERIAL_PORT.read();
  SERIAL_PORT.read();
  while(SERIAL_PORT.available() < 1)
  {
    delay(1);
  }
  int axis = SERIAL_PORT.parseInt();
  int samples = 0;
  if(axis == 0)
    goto done_calibrating;
  while(SERIAL_PORT.available())
    SERIAL_PORT.read();
  SERIAL_PORT.println("calibrating, enter anything to go back to selection");
  output[axis - 1] = 0;
  while(SERIAL_PORT.available() < 1)
  {
    while(!icm.dataReady())
    {}
    icm.getAGMT(); 
    switch(axis)
    {
      case 1:
        output[axis - 1] += icm.accY();
        gyr[(axis - 1) / 2] += icm.gyrY();
        break;
      case 2:
        output[axis - 1] += icm.accY();
        gyr[(axis - 1) / 2] += icm.gyrY();
        break;
      case 3:
        output[axis - 1] += icm.accZ();
        gyr[(axis - 1) / 2] += icm.gyrZ();
        break;
      case 4:
        output[axis - 1] += icm.accZ();
        gyr[(axis - 1) / 2] += icm.gyrZ();
        break;
      case 5:
        output[axis - 1] += icm.accX();
        gyr[(axis - 1) / 2] += icm.gyrX();
        break;
      case 6:
        output[axis - 1] += icm.accX();
        gyr[(axis - 1) / 2] += icm.gyrX();
        break;
      default:
        break;
    }
    samples++;
  }
  gyr[(axis - 1) / 2] /= samples;
  output[axis - 1] /= samples;
      
  goto axis_calibration;
  done_calibrating:

  output[0] = (output[0] + output[1]) / 2;
  output[1] = (output[2] + output[3]) / 2;
  output[2] = (output[4] + output[5]) / 2;

  output[3] = gyr[0];
  output[4] = gyr[1];
  output[5] = gyr[2];

  SERIAL_PORT.printf("%.8f\n", output[0]);
  SERIAL_PORT.printf("%.8f\n", output[1]);
  SERIAL_PORT.printf("%.8f\n", output[2]);
  SERIAL_PORT.printf("%.8f\n", output[3]);
  SERIAL_PORT.printf("%.8f\n", output[4]);
  SERIAL_PORT.printf("%.8f\n", output[5]);
  delay(10000);
  
  return output;
}

float* cal_magnet()
{
  float* output = (float*)malloc(6 * sizeof(float));
  for(int i = 0; i < 3; i++)
    output[i] = 10000;
  for(int i = 3; i < 6; i++)
    output[i] = -10000;
  SERIAL_PORT.read();
  SERIAL_PORT.println("magnet calibration, move the thing around a bunch, rotations and shit");
  while(SERIAL_PORT.available() < 2)
  {
    int samples = 25;
    
    float xsamples[samples];
    float ysamples[samples];
    float zsamples[samples];

    float xmean = 0;
    float ymean = 0;
    float zmean = 0;

    float xstddev = 0;
    float ystddev = 0;
    float zstddev = 0;
    
    for(int i = 0; i < samples; i++)
    {
      delay(9);
      while(!icm.dataReady())
      {}
      icm.getAGMT(); 
      xsamples[i] = icm.magX();
      ysamples[i] = icm.magY();
      zsamples[i] = icm.magZ();

      xmean += xsamples[i];
      ymean += ysamples[i];
      zmean += zsamples[i];
    }
    
    xmean /= samples;
    ymean /= samples;
    zmean /= samples;

    for(int i = 0; i < samples; i++)
    {
      xstddev += powf(xsamples[i] - xmean, 2);
      ystddev += powf(ysamples[i] - ymean, 2);
      zstddev += powf(zsamples[i] - zmean, 2);
    }

    xstddev = sqrtf(xstddev / samples);
    ystddev = sqrtf(ystddev / samples);
    zstddev = sqrtf(zstddev / samples);

    int xcount = 0;
    int ycount = 0;
    int zcount = 0;

    float cleanxmean = 0;
    float cleanymean = 0;
    float cleanzmean = 0;
    
    for(int i = 0; i < samples; i++)
    {
      if(abs(xsamples[i] - xmean) < xstddev)
      {
        cleanxmean += xsamples[i];
        xcount++;
      }
      if(abs(ysamples[i] - ymean) < ystddev)
      {
        cleanymean += ysamples[i];
        ycount++;
      }
      if(abs(zsamples[i] - zmean) < zstddev)
      {
        cleanzmean += zsamples[i];
        zcount++;
      }
    }

    cleanxmean /= xcount;
    cleanymean /= ycount;
    cleanzmean /= zcount;
    
    output[0] = min(output[0], cleanxmean);
    output[1] = min(output[1], cleanymean);
    output[2] = min(output[2], cleanzmean);
    output[3] = max(output[3], cleanxmean);
    output[4] = max(output[4], cleanymean);
    output[5] = max(output[5], cleanzmean);
  }

  float xbias = (output[0] + output[3]) / 2;
  float ybias = (output[1] + output[4]) / 2;
  float zbias = (output[2] + output[5]) / 2;

  float avg = ((output[3] - output[0]) + (output[4] - output[1]) + (output[5] - output[2])) / 3;

  float xscale = avg / (output[3] - output[0]);
  float yscale = avg / (output[4] - output[1]);
  float zscale = avg / (output[5] - output[2]);

  output[0] = xbias;
  output[1] = ybias;
  output[2] = zbias;

  output[3] = xscale;
  output[4] = yscale;
  output[5] = zscale;

  SERIAL_PORT.printf("%.8f\n", output[0]);
  SERIAL_PORT.printf("%.8f\n", output[1]);
  SERIAL_PORT.printf("%.8f\n", output[2]);
  SERIAL_PORT.printf("%.8f\n", output[3]);
  SERIAL_PORT.printf("%.8f\n", output[4]);
  SERIAL_PORT.printf("%.8f\n", output[5]);
  delay(10000);
  return output;
}

void setup()
{
  SERIAL_PORT.begin(115200);
  
  dataQueue = xQueueCreate(20, sizeof(ICM_Data));
  hspi.begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, -1);
  vspi.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, SD_CS);
  
  //icm.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
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
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
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
  
  m_modify(&measurement_uncertainty, 0, 0, bmp_stddev);
  
  m_modify(&observation, 0, 0, 1);
  
  m_modify(&cov, 0, 0, start_cov);
  m_modify(&cov, 1, 1, start_cov);

  m_modify(&s_tran, 0, 0, 1);
  m_modify(&s_tran, 1, 1, 1);

  state_extrapolation(&f_state, &state, &s_tran, &control);
  covariance_extrapolation(&f_cov, &s_tran, &cov, &process_noise);

  ahrs_state.acc_kp = 1;
  ahrs_state.acc_ki = 0;
  ahrs_state.mag_kp = 0;
  ahrs_state.mag_ki = 0;

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
  track = start;
}

void loop()
{
  static float angle = 0;
  while(acc_offsets == NULL || mag_offsets == NULL)
  {
    SERIAL_PORT.println("enter 1 to calibrate mag, 2 to calibrate acc");
    if(SERIAL_PORT.available())
    {
      int input = SERIAL_PORT.parseInt();
      if(input == 1)
      {
        mag_offsets = cal_magnet();
      }
      if(input == 2)
      {
        acc_offsets = cal_acc();
      }
    }
  }
  while(!icm.dataReady())
  {}
  icm.getAGMT();         // The values are only updated when you call 'getAGMT'
  //printICM(&icm);
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

  ahrs_state.acc[0] = currentReadings.accX;
  ahrs_state.acc[1] = currentReadings.accZ;
  ahrs_state.acc[2] = -currentReadings.accY;

  ahrs_state.gyr[0] = currentReadings.gyrX * PI / 180;
  ahrs_state.gyr[1] = currentReadings.gyrZ * PI / 180;
  ahrs_state.gyr[2] = -currentReadings.gyrY * PI / 180;

  ahrs_state.mag[0] = currentReadings.magX;
  ahrs_state.mag[1] = currentReadings.magZ;
  ahrs_state.mag[2] = -currentReadings.magY;


  m_modify(&s_tran, 1, 0, deltaT);

  m_modify(&control, 0, 0, (powf(deltaT, 2) / 2) * input.elems[0]);
  m_modify(&control, 0, 1, deltaT * input.elems[0]);

  m_modify(&process_noise, 0, 0, (powf(deltaT, 4) / 4) * icm_stddev);
  m_modify(&process_noise, 0, 1, (powf(deltaT, 3) / 2) * icm_stddev);
  m_modify(&process_noise, 1, 0, (powf(deltaT, 3) / 2) * icm_stddev);
  m_modify(&process_noise, 1, 1, powf(deltaT, 2) * icm_stddev);

  mahony_acc_update(&ahrs_state, deltaT);
  mahony_gyro_update(&ahrs_state, deltaT);

  float acc_out[3];

  vector_from_quat(ahrs_state.quat, ahrs_state.acc, acc_out);

  m_modify(&input, 0, 0, ((acc_out[2] - 1000) * GTOM / 1000));
  
  if(((int)esp_timer_get_time() - track) >= 20000)
  {
    track = esp_timer_get_time();
    alt = bmp.readAltitude(PAD_PRESSURE);
    m_modify(&measurement, 0, 0, alt);
    //mahony_mag_update(&ahrs_state, deltaT);
    update_gain(&gain, &f_cov, &observation, &measurement_uncertainty);
    state_update(&state, &f_state, &gain, &measurement, &observation);
    covariance_update(&cov, &identity, &gain, &observation, &f_cov, &measurement_uncertainty);
    state_extrapolation(&f_state, &state, &s_tran, &control);
    covariance_extrapolation(&f_cov, &s_tran, &cov, &process_noise);
    SERIAL_PORT.printf("y acc: %.2f, mahony y acc: %.2f\n", currentReadings.accY, acc_out[2]);
    m_display(state);
    //SERIAL_PORT.println(input.elems[0]);
    //SERIAL_PORT.print("Current total mag ");
    //SERIAL_PORT.println(sqrtf(powf(currentReadings.magX, 2) + powf(currentReadings.magY, 2) + powf(currentReadings.magZ, 2)));
    //SERIAL_PORT.println(currentReadings.magX);
    //SERIAL_PORT.println(currentReadings.magZ);

    angle *= 24.0/25.0;
    angle += atan2f(currentReadings.magX, currentReadings.magZ) * 180 / PI / 25;

    /*
    SERIAL_PORT.println("input gyro rads/s");
    SERIAL_PORT.print(ahrs_state.gyr[0]);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.print(ahrs_state.gyr[1]);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.println(ahrs_state.gyr[2]);

    SERIAL_PORT.println("quaternion");
    SERIAL_PORT.print(ahrs_state.quat[0]);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.print(ahrs_state.quat[1]);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.print(ahrs_state.quat[2]);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.println(ahrs_state.quat[3]);
    
    SERIAL_PORT.println("input acc");
    SERIAL_PORT.print(ahrs_state.acc[0]);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.print(ahrs_state.acc[1]);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.println(ahrs_state.acc[2]);
    
    SERIAL_PORT.println("output acc");
    SERIAL_PORT.print(acc_out[0]);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.print(acc_out[1]);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.println(acc_out[2]);
    //SERIAL_PORT.print(angle);
    //SERIAL_PORT.println("degrees");
    */
  }
  else
  {
  state_extrapolation(&f_state, &f_state, &s_tran, &control);
  covariance_extrapolation(&f_cov, &s_tran, &f_cov, &process_noise);
  }
  
  currentReadings.alt = alt;
  //printBMP(&bmp);
  deltaT = (float)((int)esp_timer_get_time() - start) / 1000000.0;
  start = (int)esp_timer_get_time();

  xQueueSend(dataQueue, &currentReadings, 0);
}
