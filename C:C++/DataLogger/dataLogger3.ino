#include <Wire.h>
#include <SPI.h>
#include <SD.h>

File myFile;

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C
#define    BMP280_ADDRESS             0x76

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

// TODO demander l'accélération maximale de la fusée
// faut sûrement mettre 16g
#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

//select the chip (depend on the SD shield, put 10 for Adafruit)
const int chipSelect = 10;
const int button = 3;
const int led = 8;

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}


// Initializations
void setup()
{
  // Arduino initialization
  Wire.begin();
  Serial.begin(115200);
  
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_16_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
  
  // Request first magnetometer single measurement
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
  I2CwriteByte(BMP280_ADDRESS,0xF5,0x00);  
  I2CwriteByte(BMP280_ADDRESS,0xF4,0b00100111);

  Serial.print("\nInitializing SD card...");
  pinMode(chipSelect, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(button,INPUT_PULLUP);

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    myFile = SD.open("data/test.txt", FILE_WRITE);
   
    // if the file opened okay, write to it:
    if (myFile) {
      Serial.println("File open");
    } else {
      // if the file didn't open, print an error:
      Serial.println("error opening test.txt");
    }

    Serial.println("Press the button to start data logging");
    while(digitalRead(button));
    delay(10);
    while(!digitalRead(button));
    digitalWrite(led,HIGH);
    myFile.println("START");
    String temp ="t[ms],\t ax ,\t ay,\t az,\t gx,\t gy,\t gz,\t mx,\t my,\t mz,\t temperature[C],\t pressure[hPa]";
    myFile.println(temp);
    Serial.println(temp);
}


long int cpt=0;
// Main loop, read and display data
void loop()
{
  // Stop logging and switch off the LED when button is pushed
  if(!digitalRead(button)){
    digitalWrite(led,LOW);
    myFile.println("STOP");
    // close the file:
    myFile.close();
    Serial.println("done.");
   
    while(1);
  }
  
  // ____________________________________
  // :::  accelerometer and gyroscope ::: 

  uint8_t bufIMU[14];
  I2Cread(MPU9250_ADDRESS,0x3B,14,bufIMU);
    
  int16_t ax=bufIMU[0]<<8 | bufIMU[1];
  int16_t ay=bufIMU[2]<<8 | bufIMU[3];
  int16_t az=bufIMU[4]<<8 | bufIMU[5];

  int16_t gx=bufIMU[8]<<8 | bufIMU[9];
  int16_t gy=bufIMU[10]<<8 | bufIMU[11];
  int16_t gz=bufIMU[12]<<8 | bufIMU[13];
  
  // _____________________
  // :::  Magnetometer ::: 

  
  // Read register Status 1 and wait for the DRDY: Data Ready
  I2CwriteByte(MAG_ADDRESS,0x0A,0x01);

  uint8_t ST1;
  do{
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  } while (!(ST1&0x01));

  // Read magnetometer data  
  uint8_t bufMag[7];  
  I2Cread(MAG_ADDRESS,0x03,7,bufMag);
    
  int16_t mx=bufMag[3]<<8 | bufMag[2];
  int16_t my=bufMag[1]<<8 | bufMag[0];
  int16_t mz=bufMag[5]<<8 | bufMag[4];

  unsigned int b1[24];
  unsigned int data[8];
  for (int i = 0; i < 24; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(BMP280_ADDRESS);
    // Select data register
    Wire.write((136 + i));
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 1 byte of data
    Wire.requestFrom(BMP280_ADDRESS, 1);

    // Read 1 byte of data
    if (Wire.available() == 1)
    {
      b1[i] = Wire.read();
    }
  }
  // Convert the data
  // temp coefficients
  unsigned int dig_T1 = (b1[0] & 0xFF) + ((b1[1] & 0xFF) * 256);
  int dig_T2 = b1[2] + (b1[3] * 256);
  int dig_T3 = b1[4] + (b1[5] * 256);

  // pressure coefficients
  unsigned int dig_P1 = (b1[6] & 0xFF) + ((b1[7] & 0xFF) * 256);
  int dig_P2 = b1[8] + (b1[9] * 256);
  int dig_P3 = b1[10] + (b1[11] * 256);
  int dig_P4 = b1[12] + (b1[13] * 256);
  int dig_P5 = b1[14] + (b1[15] * 256);
  int dig_P6 = b1[16] + (b1[17] * 256);
  int dig_P7 = b1[18] + (b1[19] * 256);
  int dig_P8 = b1[20] + (b1[21] * 256);
  int dig_P9 = b1[22] + (b1[23] * 256);
/*
  // Start I2C Transmission
  Wire.beginTransmission(BMP280_ADDRESS);
  // Select control measurement register
  Wire.write(0xF4);
  // Normal mode, temp and pressure over sampling rate = 1
  Wire.write(0x27);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(BMP280_ADDRESS);
  // Select config register
  Wire.write(0xF5);
  // Stand_by time = 1000ms
  Wire.write(0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();
*/
  for (int i = 0; i < 8; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(BMP280_ADDRESS);
    // Select data register
    Wire.write((247 + i));
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 1 byte of data
    Wire.requestFrom(BMP280_ADDRESS, 1);

    // Read 1 byte of data
    if (Wire.available() == 1)
    {
      data[i] = Wire.read();
    }
  }

  // Convert pressure and temperature data to 19-bits
  long adc_p = (((long)(data[0] & 0xFF) * 65536) + ((long)(data[1] & 0xFF) * 256) + (long)(data[2] & 0xF0)) / 16;
  long adc_t = (((long)(data[3] & 0xFF) * 65536) + ((long)(data[4] & 0xFF) * 256) + (long)(data[5] & 0xF0)) / 16;

  // Temperature offset calculations
  double var1 = (((double)adc_t) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
  double var2 = ((((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0) *
                 (((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
  double t_fine = (long)(var1 + var2);
  double temperature = (var1 + var2) / 5120.0;

  // Pressure offset calculations
  var1 = ((double)t_fine / 2.0) - 64000.0;
  var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
  var2 = var2 + var1 * ((double)dig_P5) * 2.0;
  var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
  var1 = (((double) dig_P3) * var1 * var1 / 524288.0 + ((double) dig_P2) * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
  double p = 1048576.0 - (double)adc_p;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  var1 = ((double) dig_P9) * p * p / 2147483648.0;
  var2 = p * ((double) dig_P8) / 32768.0;
  double pressure = (p + (var1 + var2 + ((double)dig_P7)) / 16.0) / 100;

  // on  peut meme mettre le timestamp en microseconds??
  String dataString = String(millis()) +','+'\t'+ String(ax) +','+'\t'+ String(ay) +','+'\t'+ String(az) +','+'\t'+ String(gx) +','+'\t'+ String(gy) +','+'\t'+ String(gz) +','+'\t'+ String(mx) +','+'\t'+ String(my) +','+'\t'+ String(mz) +','+'\t'+ String(temperature) +','+'\t'+ String(pressure) +',' ;

  Serial.println(dataString);
  myFile.println(dataString);
  
  delay(1);    
}
