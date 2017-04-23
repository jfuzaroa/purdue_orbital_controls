#include <MPU9250.h>
#include <quaternionFilters.h>
#include <SD.h>

#include <Wire.h>

MPU9250 myIMU;
File myFile;

// Power Management 1 is adress 0x01
// WHO_AM_I is address 0x71

// FIFO Enable is register 35
// I2C Master Control is register 36
// I2C Slave1 Control is registers 40-42

#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

int I2C_Adress = 0x68;
int intPin = 12;
int myLed = 13;

void setup() {
  Wire.begin();
  Serial.begin(38400);
  Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.
  pinMode(10, OUTPUT);

  if (!SD.begin(10)) {
    Serial.println("Initialization failed!");
    return;
  }
  Serial.println("Initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("imuSensor.txt", FILE_WRITE);

  if (myFile) {
    Serial.print("Writing to imuSensor.txt");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  
  // Set up interrupt pin, active high, push-pull
  pinMode(intPin,INPUT);
  digitalWrite(intPin,LOW);
  pinMode(myLed,OUTPUT);
  digitalWrite(myLed,HIGH);

  // Read WHO_AM_I register
  byte who_I2C = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  myFile.println("who_I2C's address is ");
  myFile.println(who_I2C,HEX);  // WHO_AM_I for MPU should always be 0x68, which is the I2C adress

  // Self test if WHO_AM_I not 0x68
  if (who_I2C == 0x71) {
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    myFile.println("x-axis self test: acceleration trim within : ");
    myFile.println(myIMU.selfTest[0],1); myFile.println("% of factory value");
    myFile.println("y-axis self test: acceleration trim within : ");
    myFile.println(myIMU.selfTest[1],1); myFile.println("% of factory value");
    myFile.println("z-axis self test: acceleration trim within : ");
    myFile.println(myIMU.selfTest[2],1); myFile.println("% of factory value");
    myFile.println("x-axis self test: gyration trim within : ");
    myFile.println(myIMU.selfTest[3],1); myFile.println("% of factory value");
    myFile.println("y-axis self test: gyration trim within : ");
    myFile.println(myIMU.selfTest[4],1); myFile.println("% of factory value");
    myFile.println("z-axis self test: gyration trim within : ");
    myFile.println(myIMU.selfTest[5],1); myFile.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    
  }
 
  // Initialize device for active mode read 
  myIMU.initMPU9250();
  myFile.println("MPU9250 initialized for active data mode....");

  // Read the WHO_AM_I register of magnetometer --> should always be 0x48
  byte who_magnet = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
  myFile.println("The (AK8963) magnetometer's address is: ");
  myFile.println(who_magnet,HEX); 

  // Initialize device for active mode read of magnetometer
  myIMU.initAK8963(myIMU.factoryMagCalibration);
  myFile.println("AK8963 initialized for active data mode....");
  
  if (SerialDebug)
    {
      //  myFile.println("Calibration values: ");
      myFile.println("X-Axis sensitivity adjustment value ");
      myFile.println(myIMU.factoryMagCalibration[0], 2);
      myFile.println("Y-Axis sensitivity adjustment value ");
      myFile.println(myIMU.factoryMagCalibration[1], 2);
      myFile.println("Z-Axis sensitivity adjustment value ");
      myFile.println(myIMU.factoryMagCalibration[2], 2);
    }
   else
  {
    myFile.println("Could not connect to MPU9250: 0x");
    myFile.println(who_magnet, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
}

void loop() {

  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.factoryMagBias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.factoryMagBias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    myIMU.factoryMagBias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.factoryMagCalibration[0] -
               myIMU.factoryMagBias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.factoryMagCalibration[1] -
               myIMU.factoryMagBias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.factoryMagCalibration[2] -
               myIMU.factoryMagBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

    // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                         myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        // Print acceleration values in milligs!
        myFile.println("X-acceleration: "); myFile.println(1000*myIMU.ax);
        myFile.println(" mg ");
        myFile.println("Y-acceleration: "); myFile.println(1000*myIMU.ay);
        myFile.println(" mg ");
        myFile.println("Z-acceleration: "); myFile.println(1000*myIMU.az);
        myFile.println(" mg ");

        // Print gyro values in degree/sec
        myFile.println("X-gyro rate: "); myFile.println(myIMU.gx, 3);
        myFile.println(" degrees/sec ");
        myFile.println("Y-gyro rate: "); myFile.println(myIMU.gy, 3);
        myFile.println(" degrees/sec ");
        myFile.println("Z-gyro rate: "); myFile.println(myIMU.gz, 3);
        myFile.println(" degrees/sec");

        // Print mag values in degree/sec
        myFile.println("X-mag field: "); myFile.println(myIMU.mx);
        myFile.println(" mG ");
        myFile.println("Y-mag field: "); myFile.println(myIMU.my);
        myFile.println(" mG ");
        myFile.println("Z-mag field: "); myFile.println(myIMU.mz);
        myFile.println(" mG");

//        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
//        // Temperature in degrees Centigrade
//        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
//        // Print temperature in degrees Centigrade
//        myFile.println("Temperature is ");  myFile.println(myIMU.temperature, 1);
//        myFile.println(" degrees C");
      }

      myIMU.count = millis();
      digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // update LCD once per half-second independent of read rate
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        myFile.println("ax = "); myFile.println((int)1000*myIMU.ax);
        myFile.println(" ay = "); myFile.println((int)1000*myIMU.ay);
        myFile.println(" az = "); myFile.println((int)1000*myIMU.az);
        myFile.println(" mg");

        myFile.println("gx = "); myFile.println( myIMU.gx, 2);
        myFile.println(" gy = "); myFile.println( myIMU.gy, 2);
        myFile.println(" gz = "); myFile.println( myIMU.gz, 2);
        myFile.println(" deg/s");

        myFile.println("mx = "); myFile.println( (int)myIMU.mx );
        myFile.println(" my = "); myFile.println( (int)myIMU.my );
        myFile.println(" mz = "); myFile.println( (int)myIMU.mz );
        myFile.println(" mG");

        myFile.println("q0 = "); myFile.println(*getQ());
        myFile.println(" qx = "); myFile.println(*(getQ() + 1));
        myFile.println(" qy = "); myFile.println(*(getQ() + 2));
        myFile.println(" qz = "); myFile.println(*(getQ() + 3));
      }

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                    *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                    *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                    *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //    8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // Declination of Armstrong Building (40°25'51.5"N 86°54'52.4"W) is
      //    4.22° W ± 0.36° (or 4.22°) on 2017-04-11
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      //myIMU.yaw   -= 8.5;
      myIMU.yaw   += 4.22;
      myIMU.roll  *= RAD_TO_DEG;

      if(SerialDebug)
      {
        myFile.println("Yaw, Pitch, Roll: ");
        myFile.println(myIMU.yaw, 2);
        myFile.println(", ");
        myFile.println(myIMU.pitch, 2);
        myFile.println(", ");
        myFile.println(myIMU.roll, 2);

        myFile.println("rate = ");
        myFile.println((float)myIMU.sumCount/myIMU.sum, 2);
        myFile.println(" Hz");
      }
      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
  } // if (AHRS)
  myFile.flush();
}


