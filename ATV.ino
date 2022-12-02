//formatting
#include <Adafruit_Sensor.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20649.h>
#include <stdio.h>
#include <stdlib.h>

//keypad controller
#include <LedKeypad.h>
#include <TM1650.h>
TM1650 m_4DIGDisplay;

//dc motor and servo control
#include <AFMotor.h>
#include <Servo.h> 
AF_DCMotor m_motor(1);
int m_speed = 45;
Servo m_servo;
int m_rudderpos = 74;

//timers
#include <arduino-timer.h>
//auto timer2Hz = timer_create_default();
//int blnSelectPressed = false;

//IMU
#include "ICM_20948.h"
ICM_20948_I2C m_IMU;
double m_yaw;

//magnetometer (duinotech magnetometer)
//#include <i2c_MAG3110.h>

//I2C lib
#include <Wire.h>

//sdcard logging
#include <SD.h>
#include <SPI.h>
File m_SDFile;
int pinCS = 53; // Pin 10 on Arduino Uno Pin 53 for Mega
char m_log_name[] = "ATVNAV.csv"; //no more than 8 chars in name

//GPS
#include <NMEAGPS.h> //this header file has comments that describe how NeoGPS is used
#include <GPSport.h>
#include <Streamers.h>
static NMEAGPS m_GPS;
static gps_fix m_fix;
int32_t m_GPS_lat[10];
int32_t m_GPS_lon[10];
int32_t m_GPS_mid_lat;
int32_t m_GPS_mid_lon;
int32_t m_GPS_mid_lat_stored;
int32_t m_GPS_mid_lon_stored;
int32_t m_GPS_start_lat_stored;
int32_t m_GPS_start_lon_stored;
float m_geof_radius = 0;
float m_travel_dist = 0;
float m_GPS_error_vector;
int m_GPS_index = 0;
int m_GPS_can_read = 0;
float m_GPS_bearing;

//calibrate and control
float m_distFromStart = 0;
float m_Kp_speed = 100; //proportional gain
float m_maxSpeed = 140;
float m_minSpeed = 35;
float m_IMU_bearings[500];
float m_IMU_bearing;
int m_IMU_bearing_index = 0;
float m_GPS_IMU_adj = 0;
float m_Kp_rudder = 0.5; //gain for rudder


//general
int m_test_running = 0;

//FSM states
enum { IDLE, SLOC, CLOC, SHDG, FIND, TST1, TST2} state;

void setup() {

    Wire.begin();
    Serial.begin(9600);
    state = IDLE;
    
    //setup for motor and rudder;
    m_motor.run(FORWARD);
    m_servo.attach(10); //pin 10 for motor driver servo_1 and pin 9 for motor driver servo_2
    m_servo.write(m_rudderpos);

    //timer2Hz.every(2000, timer2Hz_interrupt);
  
    //start LCD screen and play intro
    init_LCD();
    intro();
    setDisplay("LCDY", 2); delay(2000);

    //initialise peripherals/sensors  
    init_Keypad();
    setDisplay("PADY", 2); delay(2000);
    init_IMU();
    setDisplay("INEY", 2); delay(2000);
    init_Logger();
    setDisplay("LOGY", 2); delay(2000);
    init_GPS();
    setDisplay("GPSY", 2); delay(2000);
    //init_Motor();

    setDisplay("IDLE", -1);

}


void loop() {
  
    //timer2Hz.tick();
    GPSloop();
    IMUloop();

    //check keypad for action
    unsigned char keyValue = ledkeypad.getKey();/*Get key value*/

    switch (state) {
    case IDLE:
        if (keyValue == KEY_SELECT) { state = SLOC; setDisplay("SLOC", 0); }
        m_motor.setSpeed(0);
        m_servo.write(74);
        m_test_running = 0;
        break;
    case SLOC:
        //store a GPS location and set geofence radius
        if (keyValue == KEY_SELECT) { state = CLOC; setDisplay("CLOC", 0); }
        if (keyValue == KEY_UP) {
            //store current GPS coord
            setDisplayInt(m_GPS_error_vector);
            m_GPS_mid_lat_stored = m_GPS_mid_lat;
            m_GPS_mid_lon_stored = m_GPS_mid_lon;
        } else if (keyValue == KEY_LEFT) {
            m_geof_radius = m_geof_radius + 2;
            setDisplayInt(m_geof_radius);   
        } else if (keyValue == KEY_RIGHT) {
            m_geof_radius = m_geof_radius - 2;
            setDisplayInt(m_geof_radius);
        }

        break;
    case CLOC:
        if (keyValue == KEY_SELECT) { state = SHDG; setDisplay("SHDG", 0); }
        //not required until multiple waypoints implemented


        break;
    case SHDG:
        //set the heading by orientating boat to north and clicking up button
        if (keyValue == KEY_SELECT) { state = TST1; setDisplay("TST1", -1); }
        if (keyValue == KEY_UP) {
            /*m_heading_cor = m_heading_raw;
            applyHeadingCor();
            setDisplayInt((int)m_heading_cor);*/
        }
        else if (keyValue == KEY_DOWN) {   
            //setDisplayInt((int)m_heading_raw);
        }

        break;
    case TST1:
        //set the motor speed and rudder position
        if (keyValue == KEY_SELECT) { state = TST2; setDisplay("TST2", 0); }
        if (keyValue == KEY_UP) {
            m_speed = m_speed + 5;
            setDisplayInt(m_speed);
            m_motor.setSpeed(m_speed);
        } else if (keyValue == KEY_DOWN) {
            m_speed = m_speed - 5;
            setDisplayInt(m_speed);
            m_motor.setSpeed(m_speed);
        } else if (keyValue == KEY_LEFT) {
            m_rudderpos = m_rudderpos + 2;
            setDisplayInt(m_rudderpos);
            m_servo.write(m_rudderpos);
        } else if (keyValue == KEY_RIGHT) {
            m_rudderpos = m_rudderpos - 2;
            setDisplayInt(m_rudderpos);
            m_servo.write(m_rudderpos);
        }

        break;
    case TST2:
        //start the geofence test
        if (keyValue == KEY_SELECT) { state = IDLE; setDisplay("IDLE", -1); }
        if (m_test_running == 0) {
            
            //store the GPS location of the launch
            m_GPS_start_lat_stored = m_fix.latitudeL();
            m_GPS_start_lon_stored = m_fix.longitudeL();
               
            //start the test but shutdown motor for 5 seconds to make launch easier
            m_motor.setSpeed(0);
            delay(5000);
            m_motor.setSpeed(m_speed);
            m_test_running = 1;

            //start the test 2 log - actual log data will be written within the GPSloop 
            logHeaderT2();
        }

        //if distance reached go into find mode and return to start coord
        setDisplayInt((int)m_distFromStart);
        if (m_distFromStart > m_geof_radius && m_IMU_bearing_index > 10) {
            m_GPS_mid_lat_stored = m_GPS_start_lat_stored;
            m_GPS_mid_lon_stored = m_GPS_start_lon_stored;

            //calc average IMU bearing
            float sumBearings = 0;
            for (int i = 0; i < m_IMU_bearing_index; i++) {
                sumBearings = sumBearings + m_IMU_bearings[i];
            }
            float ave_IMU_bearing = sumBearings / (m_IMU_bearing_index + 1);

            //determine the adjustment factor to apply to IMU bearings to align with GPS bearing system
            m_GPS_IMU_adj = m_GPS_bearing - ave_IMU_bearing;

            state = FIND; setDisplay("FIND", -1);
            delay(3000);

        }
        break;
        
    case FIND:
        //state is not available via the user clicking the select button - only can be entered from other states
        if (keyValue == KEY_SELECT) { state = IDLE; setDisplay("IDLE", -1); }
        //implement control algorithm here
        //navigate to the stored lat and lon to within a set distance of distance of it 
        //then go into idle state
            
        float distError = getDistBetweenCoords(m_fix.latitudeL(), m_fix.longitudeL(), m_GPS_mid_lat_stored, m_GPS_mid_lon_stored);
        setDisplayInt(distError);
        if (distError >= 0 && distError < 0.5) {
            state = IDLE; setDisplay("IDLE", -1);
        } else {
            //proportional control (Kp = proportional gain)
            //we wan't to adjust the speed using the error in distance * by the gain
            if (distError < 0) { distError = -distError;} //just in case but probably don't need
            float newSpeed = (distError * m_Kp_speed) + m_minSpeed;
            if (newSpeed > m_maxSpeed) { newSpeed = m_maxSpeed; } //saturate
            m_motor.setSpeed(newSpeed);

            //get current adjusted IMU bearing
            float IMU_bearing_adj = m_IMU_bearing + m_GPS_IMU_adj;
            if (IMU_bearing_adj < 0){IMU_bearing_adj = 360 + IMU_bearing_adj;}//probably don't need - should always be between 0 and 360

            //determine bearing error
            float GPS_bearing = getBearingBetweenCoords(m_GPS_mid_lat_stored, m_GPS_mid_lon_stored,m_fix.latitudeL(), m_fix.longitudeL());
            float bearingError = GPS_bearing - IMU_bearing_adj;
            if (bearingError > 180) { bearingError = bearingError -360 ; }
            if (bearingError < -180) { bearingError = 360 + bearingError; }

            //RUDDER
            //we wan't to adjust the rudder using the error in bearing * by the gain
            float newRudderAngle = (bearingError * m_Kp_rudder); //min rudder is always 0
            if (newRudderAngle > 45) { newRudderAngle = 45; } //saturate - max rudder is always 45 degrees
            if (newRudderAngle < -45) { newRudderAngle = -45; } //saturate - max rudder is always -45 degrees
            //convert rudder angle to actual rudder pos (0-45 degree rudder angle range)
            //positive rudder angle is starboard turn direction (72-36 servo position range)
            //negative rudder angle is port turn direction (138-72 servo position range)
            float newRudderPos = 72; //0 degrees
            if (newRudderAngle > 0) { 
                newRudderPos = newRudderPos - (36*newRudderAngle/45);
            }
            else if (newRudderAngle < 0) {
                newRudderPos = newRudderPos - (66 * newRudderAngle / 45);
            }
            m_servo.write(newRudderPos);

        }

        
        break;
    }  
}


void intro() {
    setDisplay("HELO", 0); delay(1000);
    setDisplay("    ", -1); delay(750);
    setDisplay("HELO", 1); delay(1000);
    setDisplay("    ", -1); delay(750);
    setDisplay("HELO", 2); delay(1000);
    setDisplay("    ", -1); delay(750);
    setDisplay("HELO", 3); delay(1000);
    setDisplay("    ", -1); delay(750);
}

void setDisplay(char line[], int dotPos) {
    m_4DIGDisplay.displayString(line);
    m_4DIGDisplay.setDot(0, false);
    m_4DIGDisplay.setDot(1, false);
    m_4DIGDisplay.setDot(2, false);
    m_4DIGDisplay.setDot(3, false);
    if (dotPos > -1) {
        m_4DIGDisplay.setDot(dotPos, true);
    }
}

void setDisplayInt(int intToDisplay) {
    int a = (int)(intToDisplay / 1000 - (int)(intToDisplay / 10000.00) * 10.00);
    int b = (int)(intToDisplay / 100.00 - (int)(intToDisplay / 1000.00) * 10.00);
    int c = (int)(intToDisplay / 10.00 - (int)(intToDisplay / 100.00) * 10.00);
    int d = (int)(intToDisplay - (int)(intToDisplay / 10.00) * 10.00);
    char charToDisplay[5] = { '0' + a,'0' + b,'0' + c,'0' + d, '\0' };
    setDisplay(charToDisplay, -1);
}
//called from main loop
static void IMUloop() {
    // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
    icm_20948_DMP_data_t data;
    m_IMU.readDMPdataFromFIFO(&data);

    if ((m_IMU.status == ICM_20948_Stat_Ok) || (m_IMU.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
    {
        //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
        //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
        //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
        //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
        //SERIAL_PORT.println( data.header, HEX );

        if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
        {
            // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
            // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
            // The quaternion data is scaled by 2^30.

            //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

            // Scale to +/- 1
            double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
            double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
            double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

            /*
            SERIAL_PORT.print(F("Q1:"));
            SERIAL_PORT.print(q1, 3);
            SERIAL_PORT.print(F(" Q2:"));
            SERIAL_PORT.print(q2, 3);
            SERIAL_PORT.print(F(" Q3:"));
            SERIAL_PORT.println(q3, 3);
      */

      // Convert the quaternions to Euler angles (roll, pitch, yaw)
      // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

            double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

            double q2sqr = q2 * q2;

            // roll (x-axis rotation)
            double t0 = +2.0 * (q0 * q1 + q2 * q3);
            double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
            double roll = atan2(t0, t1) * 180.0 / PI;

            // pitch (y-axis rotation)
            double t2 = +2.0 * (q0 * q2 - q3 * q1);
            t2 = t2 > 1.0 ? 1.0 : t2;
            t2 = t2 < -1.0 ? -1.0 : t2;
            double pitch = asin(t2) * 180.0 / PI;

            // yaw (z-axis rotation)
            double t3 = +2.0 * (q0 * q3 + q1 * q2);
            double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
            double yaw = atan2(t3, t4) * 180.0 / PI;

            m_yaw = yaw;

            /*Serial.print(F("Yaw:"));
            Serial.println(yaw, 1);*/

           
            /*Serial.print(F("Roll:"));
            Serial.print(roll, 1);
            Serial.print(F(" Pitch:"));
            Serial.print(pitch, 1);
            Serial.print(F(" Heading Raw:"));
            Serial.print(m_heading_raw, 1);
            Serial.print(F(" Heading True:"));
            Serial.println(m_heading_true, 1);*/

            
        }
    }

    if (m_IMU.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
    {
        delay(10);
    }

}


//called from main loop
static void GPSloop() {
    while (m_GPS.available(gpsPort)) {

        //store current lat,lon in array
        m_fix = m_GPS.read();
        m_GPS_lat[m_GPS_index] = m_fix.latitudeL();
        m_GPS_lon[m_GPS_index] = m_fix.longitudeL();
        if (m_GPS_can_read == 1) {
            calcRunningGPS();
            Serial.print("error vector (m): "); Serial.println(m_GPS_error_vector, 10);
        }
        Serial.print(m_fix.latitudeL() / 10000000.00, 10); Serial.print(","); Serial.println(m_fix.longitudeL() / 10000000.00, 10);
        Serial.print(m_yaw); Serial.print((int)m_yaw); Serial.println();
        //trace_all( DEBUG_PORT, m_GPS, m_fix );

        //if (state == SHDG) { setDisplayInt((int)m_yaw); ; }

        //increment
        m_GPS_index++;
        if (m_GPS_index == 10) {
            //reset to 0
            m_GPS_index = 0;
            m_GPS_can_read = 1;
        }

        /*double arrayValues[2];    
        arrayValues[0] = m_fix.latitudeL();
        arrayValues[1] = m_fix.longitudeL();
        int arrayDecimalPlaces[2]= {10, 10 }*/;
        
        //writeToLog(m_fix.dateTime,arrayValues, 2, arrayDecimalPlaces);
        writeToLog();

        if (m_test_running == 1) {

            NeoGPS::time_t t = m_fix.dateTime;

            int32_t GPS_lat = m_fix.latitudeL();
            int32_t GPS_lon = m_fix.longitudeL();

            //check distance and bearing from start point
            m_distFromStart = getDistBetweenCoords(m_GPS_start_lat_stored, m_GPS_start_lon_stored, GPS_lat, GPS_lon);
            m_GPS_bearing = getBearingBetweenCoords(m_GPS_start_lat_stored, m_GPS_start_lon_stored,GPS_lat, GPS_lon);

            //convert IMU yaw to bearing and store in array
            m_IMU_bearing = m_yaw;
            if (m_yaw < 0) { m_IMU_bearing = 180 + (180 + m_yaw); }
            if (m_IMU_bearing_index < 500) {
                m_IMU_bearings[m_IMU_bearing_index] = m_IMU_bearing;
                m_IMU_bearing_index++;
            }
            
            writeToLogT2(t, GPS_lat, GPS_lon); 
        }

    }
}
void calcRunningGPS() {

    //determine the min and max bounding box
    int32_t minLat = m_GPS_lat[0];
    int32_t maxLat = m_GPS_lat[0];
    int32_t minLon = m_GPS_lon[0];
    int32_t maxLon = m_GPS_lon[0];
    for (int i = 1; i < 10; i++) {
        if (m_GPS_lat[i] > minLat) { minLat = m_GPS_lat[i]; }
        if (m_GPS_lat[i] < maxLat) { maxLat = m_GPS_lat[i]; }
        if (m_GPS_lon[i] < minLon) { minLon = m_GPS_lon[i]; }
        if (m_GPS_lon[i] > maxLon) { maxLon = m_GPS_lon[i]; }
        //Serial.print(m_GPS_lat[i],10);Serial.print(",");Serial.println(m_GPS_lon[i],10);
    }
    //determine the midpoint lat and long
    m_GPS_mid_lat = (maxLat + minLat) / 2;
    m_GPS_mid_lon = minLon + ((maxLon - minLon) / 2);
    //Serial.print("maxminlon:");Serial.print(maxLon);Serial.print(",");Serial.print(minLon);Serial.print(",");Serial.println(m_GPS_mid_lon);

    //determine distance between the min and max coords 
    m_GPS_error_vector = getDistBetweenCoords(minLat, minLon, maxLat, maxLon);

}
float getDistBetweenCoords(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
    NeoGPS::Location_t loc1(lat1, lon1);
    NeoGPS::Location_t loc2(lat2, lon2);
    return NeoGPS::Location_t::DistanceKm(loc1, loc2) * 1000;
}
float getBearingBetweenCoords(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
    NeoGPS::Location_t loc1(lat1, lon1);
    NeoGPS::Location_t loc2(lat2, lon2);
    return NeoGPS::Location_t::BearingToDegrees(loc1, loc2);
}
void logHeader() {

    m_SDFile = SD.open(m_log_name, FILE_WRITE);
    m_SDFile.print("datetime,state,speed,rudder,stored_lat,stored_lon,lat,lon,error_vector,geofence_radius,dist_travelled");
    m_SDFile.println("");
    m_SDFile.close();
}
void logHeaderT2() {

    m_SDFile = SD.open("TEST2.CSV", FILE_WRITE);
    m_SDFile.print("start lat: "); m_SDFile.println(m_GPS_start_lat_stored);
    m_SDFile.print("start lon: "); m_SDFile.println(m_GPS_start_lon_stored);
    m_SDFile.print("speed: "); m_SDFile.println(m_speed);
    m_SDFile.print("rudder: "); m_SDFile.println(m_rudderpos);
    m_SDFile.print("geofence_radius: "); m_SDFile.println(m_geof_radius);
    m_SDFile.print("datetime,lat,lon,dist_from_start,gps_bearing,imu_bearing");m_SDFile.println("");
    m_SDFile.close();
}
void writeToLogT2(NeoGPS::time_t t, int32_t GPS_lat, int32_t GPS_lon) {

   
    //create/open file 
    m_SDFile = SD.open("TEST2.CSV", FILE_WRITE);
    if (m_SDFile) {

        m_SDFile.print(t.full_year(t.year));
        m_SDFile.write('-');
        if (t.month < 10) m_SDFile.write('0');
        m_SDFile.print(t.month);
        m_SDFile.write('-');
        if (t.date < 10) m_SDFile.write('0');
        m_SDFile.print(t.date);
        m_SDFile.write(' ');
        if (t.hours < 10) m_SDFile.write('0');
        m_SDFile.print(t.hours);
        m_SDFile.write(':');
        if (t.minutes < 10) m_SDFile.write('0');
        m_SDFile.print(t.minutes);
        m_SDFile.write(':');
        if (t.seconds < 10) m_SDFile.write('0');
        m_SDFile.print(t.seconds);
        m_SDFile.write(',');
        m_SDFile.print(GPS_lat, 10);
        m_SDFile.write(',');
        m_SDFile.print(GPS_lon, 10);
        m_SDFile.write(',');
        m_SDFile.print(m_distFromStart,3);
        m_SDFile.write(',');
        m_SDFile.print(m_GPS_bearing, 3);
        m_SDFile.write(',');
        m_SDFile.print(m_IMU_bearing, 3);
        m_SDFile.println("");
    }
    m_SDFile.close();

}
//NeoGPS::time_t t,double arrayValues[],int numValues,int arrayDecimalPlaces[])
void writeToLog() {

    NeoGPS::time_t t;

    //create/open file 
    m_SDFile = SD.open(m_log_name, FILE_WRITE);
    if (m_SDFile) {

        m_SDFile.print(t.full_year(t.year));
        m_SDFile.write('-');
        if (t.month < 10) m_SDFile.write('0');
        m_SDFile.print(t.month);
        m_SDFile.write('-');
        if (t.date < 10) m_SDFile.write('0');
        m_SDFile.print(t.date);
        m_SDFile.write(' ');
        if (t.hours < 10) m_SDFile.write('0');
        m_SDFile.print(t.hours);
        m_SDFile.write(':');
        if (t.minutes < 10) m_SDFile.write('0');
        m_SDFile.print(t.minutes);
        m_SDFile.write(':');
        if (t.seconds < 10) m_SDFile.write('0');
        m_SDFile.print(t.seconds);
        m_SDFile.write(',');
        m_SDFile.print(state);
        m_SDFile.write(',');
        m_SDFile.print(m_speed);
        m_SDFile.write(',');
        m_SDFile.print(m_rudderpos);
        m_SDFile.write(',');
        m_SDFile.print(m_GPS_mid_lat_stored,10);
        m_SDFile.write(',');
        m_SDFile.print(m_GPS_mid_lon_stored,10);
        m_SDFile.write(',');
        m_SDFile.print(m_fix.latitudeL(), 10);
        m_SDFile.write(',');
        m_SDFile.print(m_fix.longitudeL(), 10);
        m_SDFile.write(',');
        m_SDFile.print(m_GPS_error_vector, 3);
        m_SDFile.write(',');
        m_SDFile.print(m_geof_radius, 3);
        m_SDFile.write(',');
        m_SDFile.print(m_travel_dist, 3);
        //m_SDFile.write(',');
        
        
        /*for (int i = 0; i < numValues; ++i) {    
            m_SDFile.print(arrayValues[i], arrayDecimalPlaces[i]);
            if (i != numValues - 1) { m_SDFile.print(","); }   
        }*/
        m_SDFile.println("");    
    }
    m_SDFile.close();

}
void init_Motor() {
    //  m_motor.setSpeed(200);
    //  m_motor.run(FORWARD);
    //  pinMode(motor, OUTPUT);
      //analogWrite(9, 200);
}
void init_LCD() {
    m_4DIGDisplay.init();
}
void init_IMU() {
    

    Wire.begin();
    Wire.setClock(400000);

    bool initialized = false;
    while (!initialized)
    {

        // Initialize the ICM-20948
        // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.

        m_IMU.begin(Wire, 1);

        Serial.print(F("Initialization of the sensor returned: "));
        Serial.println(m_IMU.statusString());

        if (m_IMU.status != ICM_20948_Stat_Ok)
        {

            Serial.println(F("Trying again..."));
            delay(500);
        }
        else
        {
            initialized = true;
        }
    }

    Serial.println(F("Device connected!"));

    bool success = true; // Use success to show if the DMP configuration was successful

    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    success &= (m_IMU.initializeDMP() == ICM_20948_Stat_Ok);

    // DMP sensor options are defined in ICM_20948_DMP.h
    //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
    //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
    //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
    //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
    //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
    //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

    // Enable the DMP Game Rotation Vector sensor
    success &= (m_IMU.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

    // Enable any additional sensors / features
    //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
    //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
    success &= (m_IMU.enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

    // Configuring DMP to output data at multiple ODRs:
    // DMP is capable of outputting multiple sensor data at different rates to FIFO.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate / ODR ) - 1
    // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    success &= (m_IMU.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

    // Enable the FIFO
    success &= (m_IMU.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (m_IMU.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (m_IMU.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (m_IMU.resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (success)
    {
        Serial.println(F("DMP enabled!"));
    }
    else
    {
        Serial.println(F("Enable DMP failed!"));
        Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
        while (1)
            ; // Do nothing more
    }
    
}
void init_Logger() {

    pinMode(pinCS, OUTPUT);

    // SD Card Initialization
    if (SD.begin())
    {
        Serial.println("SD card is ready to use.");
    }
    else
    {
        Serial.println("SD card initialization failed");
        return;
    }

    //creater the header file in the log
    logHeader();


}
void init_Keypad() {
    ledkeypad.begin();
}

static void init_GPS() {
    DEBUG_PORT.begin(9600);
    while (!DEBUG_PORT)
        ;

    DEBUG_PORT.print(F("NMEA.INO: started\n"));
    DEBUG_PORT.print(F("  fix object size = "));
    DEBUG_PORT.println(sizeof(m_GPS.fix()));
    DEBUG_PORT.print(F("  gps object size = "));
    DEBUG_PORT.println(sizeof(m_GPS));
    DEBUG_PORT.println(F("Looking for GPS device on " GPS_PORT_NAME));



#ifndef NMEAGPS_RECOGNIZE_ALL
#error You must define NMEAGPS_RECOGNIZE_ALL in NMEAGPS_cfg.h!
#endif

#ifdef NMEAGPS_INTERRUPT_PROCESSING
#error You must *NOT* define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif

#if !defined( NMEAGPS_PARSE_GGA ) & !defined( NMEAGPS_PARSE_GLL ) & \
      !defined( NMEAGPS_PARSE_GSA ) & !defined( NMEAGPS_PARSE_GSV ) & \
      !defined( NMEAGPS_PARSE_RMC ) & !defined( NMEAGPS_PARSE_VTG ) & \
      !defined( NMEAGPS_PARSE_ZDA ) & !defined( NMEAGPS_PARSE_GST )

    DEBUG_PORT.println(F("\nWARNING: No NMEA sentences are enabled: no fix data will be displayed."));

#else
    if (m_GPS.merging == NMEAGPS::NO_MERGING) {
        DEBUG_PORT.print(F("\nWARNING: displaying data from "));
        DEBUG_PORT.print(m_GPS.string_for(LAST_SENTENCE_IN_INTERVAL));
        DEBUG_PORT.print(F(" sentences ONLY, and only if "));
        DEBUG_PORT.print(m_GPS.string_for(LAST_SENTENCE_IN_INTERVAL));
        DEBUG_PORT.println(F(" is enabled.\n"
            "  Other sentences may be parsed, but their data will not be displayed."));
    }
#endif

    DEBUG_PORT.print(F("\nGPS quiet time is assumed to begin after a "));
    DEBUG_PORT.print(m_GPS.string_for(LAST_SENTENCE_IN_INTERVAL));
    DEBUG_PORT.println(F(" sentence is received.\n"
        "  You should confirm this with NMEAorder.ino\n"));

    trace_header(DEBUG_PORT);
    DEBUG_PORT.flush();

    m_GPS.send_P(&gpsPort, F("PUBX,40,GST,0,1,0,0,0,0")); // enable GST sentence, one per update interval

    gpsPort.begin(9600);


}



static void timer2Hz_interrupt() {

    //Serial.println("2Hz interrupt state = ");

}
static void Compass(int x, int y, int z) {

    String cardinal;

    //compass.read(&x, &y, &z);

    float headingRadians = atan2(y, x);
    float headingDegrees = headingRadians * 180 / PI;
    float declinationAngle = 0; //11.41666666666667;

    headingDegrees += declinationAngle;

    if (headingDegrees < 0) {
        headingDegrees += 270;
    }

    if (headingDegrees > 348.75 || headingDegrees < 11.25) {
        cardinal = " N";
    }
    else if (headingDegrees > 11.25 && headingDegrees < 33.75) {
        cardinal = " NNE";
    }
    else if (headingDegrees > 33.75 && headingDegrees < 56.25) {
        cardinal = " NE";
    }
    else if (headingDegrees > 56.25 && headingDegrees < 78.75) {
        cardinal = " ENE";
    }
    else if (headingDegrees > 78.75 && headingDegrees < 101.25) {
        cardinal = " E";
    }
    else if (headingDegrees > 101.25 && headingDegrees < 123.75) {
        cardinal = " ESE";
    }
    else if (headingDegrees > 123.75 && headingDegrees < 146.25) {
        cardinal = " SE";
    }
    else if (headingDegrees > 146.25 && headingDegrees < 168.75) {
        cardinal = " SSE";
    }
    else if (headingDegrees > 168.75 && headingDegrees < 191.25) {
        cardinal = " S";
    }
    else if (headingDegrees > 191.25 && headingDegrees < 213.75) {
        cardinal = " SSW";
    }
    else if (headingDegrees > 213.75 && headingDegrees < 236.25) {
        cardinal = " SW";
    }
    else if (headingDegrees > 236.25 && headingDegrees < 258.75) {
        cardinal = " WSW";
    }
    else if (headingDegrees > 258.75 && headingDegrees < 281.25) {
        cardinal = " W";
    }
    else if (headingDegrees > 281.25 && headingDegrees < 303.75) {
        cardinal = " WNW";
    }
    else if (headingDegrees > 303.75 && headingDegrees < 326.25) {
        cardinal = " NW";
    }
    else if (headingDegrees > 326.25 && headingDegrees < 348.75) {
        cardinal = " NNW";
    }

    //  Serial.print("Heading: ");
    //  Serial.print(headingDegrees);
    //  Serial.println(cardinal);

      //delay(250);
}
