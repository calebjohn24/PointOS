#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);
int incomingByte = 0; // for incoming serial data

void setup(void)
{
    Serial.begin(115200);

    /* Initialise the sensor */

    if (!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.println("-1,-1,-1,-1,-1,-1");
        while (1)
            ;
    }

    delay(100);

    bno.setExtCrystalUse(true);
}

void loop(void)
{

    /* Get a new sensor event */
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    sensors_event_t event;
    bno.getEvent(&event);
    /* Display the floating point data */
    Serial.print(float((event.orientation.x)));
    Serial.print(",");
    Serial.print(float((gyro.x())));
    Serial.print(",");
    Serial.print(float((gyro.y())));
    Serial.print(",");
    Serial.print(float((accel.x())));
    Serial.print(",");
    Serial.print(float((accel.y())));
    Serial.println(",");

    if (Serial.available() > 0)
    {                                // only send data back if data has been sent
        char inByte = Serial.read(); // read the incoming data
        if(inByte == 'r')
        {
            bno.begin();
        }
        
    }

    //bno = Adafruit_BNO055(55);
}
