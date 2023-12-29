
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <avr/io.h>
#include <avr/interrupt.h>
SoftwareSerial Bluetooth(2, 3);
//SoftwareSerial Bluetooth(3,2);

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define BUTTON_PIN 15
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


unsigned long curr_time; 
int sampling_freq = 20; 
float sampling_time = 1/sampling_freq * 1e6;
char newline = B11111111;

volatile byte count; 
unsigned int reload = 0xF424; 

uint8_t sys, gyro, accel, mag;


ISR(TIMER1_COMPA_vect)
{
count++;
}

void setup(void) {
  Serial.begin(9600);
  Bluetooth.begin(9600); // Default communication rate of the Bluetooth modulet
  while (!Serial) {
    delay(1);
  }
  
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);
  sys = 0;
  gyro = 0; 
  accel = 0; 
  mag = 0; 
  bno.getCalibration(&sys, &gyro, &accel, &mag);


  cli();
  TCCR1A = 0;
  TCCR1B = 0; 
  OCR1A = reload;
  TCCR1B = (1<<WGM12) | (1<<CS10) ; 
  TIMSK1 = (1<<OCIE1A); 
  sei(); 
}


void loop() {
  
  if (count == 10)
  {

    sensors_event_t event;
    bno.getEvent(&event);  
    int x = (int)(event.orientation.x*100);
    int y = (int)((event.orientation.y + 180)*100);
    int z = (int)((event.orientation.z + 180)*100);
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    int sys_int = (int)(sys);
    int gyro_int = (int)(gyro);  
    int accel_int = (int)(accel);  
    int mag_int = (int)(mag);  
    Bluetooth.write((byte *)&newline, 1); 
    Bluetooth.write((byte *)&newline, 1); 
    Bluetooth.write((byte *)&newline, 1); 
    Bluetooth.write((byte *)&x, 2); 
    Bluetooth.write((byte *)&y, 2); 
    Bluetooth.write((byte *)&z, 2); 
    Bluetooth.write((byte *)&sys_int, 2); 
    Bluetooth.write((byte *)&gyro_int, 2); 
    Bluetooth.write((byte *)&accel_int, 2); 
    Bluetooth.write((byte *)&mag_int, 2); 
    curr_time = micros(); 
    Bluetooth.write((byte *)&curr_time, 4); 


    count = 0;
}
}
