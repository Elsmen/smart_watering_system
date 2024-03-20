/* 
 * Project Smart House Plant Watering System
 *Description: Using a relay, water pump, airquality sensor, Dust particle sensor, OLED, and BME280 make the system work
 * Author: Elsmen Aragon
 * Date: 16-March-2024
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Air_Quality_Sensor.h"
#include "math.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_BME280.h"
#include "credentials.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "IoTTimer.h"

//Variables for relay
const int RELAYPIN = D16;
//Variables for MoistureSensor
int moistureRead;
const int MOISTUREPIN =A0;
//Varibles for AirQuality
int current_quality =-1;
int airValue;
//Variable for Dust Sensor
const int DUSTPIN = D3;
unsigned long duration, starttime;
unsigned long sampletime_ms = 30000; //sample 30s
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;
//Variables for OLED
const int ROT = 0;
const int OLED_RESET=-1;
//Variables for BME280
const int HEXADDRESS = 0X76;
float tempC, pressPA, humidRH, tempF, convertedPA;
bool status;
//Time for timer
int timeTimer = 120000;

bool subValue;
unsigned int last, lastTime;


IoTTimer waterTimer;
//object for the BME280
Adafruit_BME280 myReading; //Defining bme object mine is called myReading
//object for OLED called "display"
Adafruit_SSD1306 display(OLED_RESET);
//objects for date and time
String DateTime, TimeOnly;
//object for the airquality sensor
AirQualitySensor airQualitySensor(A2);
/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
//Adafruit_MQTT_Subscribe subFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/feed1");
Adafruit_MQTT_Subscribe pumpOnOff = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/pumpOnOff");
//Adafruit_MQTT_Subscribe brightnessled = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/brightnessled");
Adafruit_MQTT_Publish Humidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Humidity");
Adafruit_MQTT_Publish Temperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temp");
Adafruit_MQTT_Publish Moisture = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Moisture");
Adafruit_MQTT_Publish QualityAir = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Air_Quality");
Adafruit_MQTT_Publish DustConcentration = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/DustConcentration");


//delcare functions
float bmeConverted(float tempF);
void MQTT_connect();
bool MQTT_ping();

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected, 10000);
  Time.zone(-6); // MST = -7, MDT = -6
  Particle.syncTime(); // Sync time with Particle Cloud
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(ROT);
  display.setCursor(0,0);
  display.printf("HELLO\n WORLD!\n");
  display.display();
  display.clearDisplay();
  display.setCursor(0,0);
  status = myReading.begin(HEXADDRESS);
  if ( status == false ) {
    Serial.printf(" BME280 at address 0x%02x X failed to start ", HEXADDRESS );
  }
  pinMode(RELAYPIN, OUTPUT);
  pinMode(MOISTUREPIN, INPUT);
  pinMode(DUSTPIN, INPUT);

  mqtt.subscribe(&pumpOnOff);
  starttime = millis(); //get the current time;
  waterTimer.startTimer(timeTimer);
}

  void loop() {
  MQTT_connect();
  MQTT_ping();

  DateTime = Time.timeStr() ; //Current Date and Time from Particle Time class
  TimeOnly = DateTime.substring (11,19) ; //Extract the Time from the DateTime String
  display.clearDisplay();
  display.printf("Time:\n %s\n\n", TimeOnly.c_str());
  moistureRead = analogRead(MOISTUREPIN);
  //display.printf("Moisture:\n %i\n", moistureRead);
  Serial.printf("moisture%i\n", moistureRead);
  tempC = myReading.readTemperature ();
  tempF = bmeConverted(tempC);
  
  display.printf("TempF      %0.1f\nMoisture  %i\n", tempF, moistureRead);
  display.display();
  display.setCursor(0,0);

 // this is our 'wait for incoming subscription packets' busy subloop 
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) {
    if (subscription == &pumpOnOff) {
      subValue = atoi((char *)pumpOnOff.lastread);
      if(subValue){
        digitalWrite(D16, HIGH);
      }
      else{
        digitalWrite(D16, LOW);
      }
    }
  }
  //if(pubTimer.isTimerReady()){
    if((millis()-lastTime > 120000)) {
    humidRH = myReading.readHumidity ();
    if(mqtt.Update()) {
       Humidity.publish(humidRH);
       Temperature.publish(tempF);
       Moisture.publish(moistureRead);
       QualityAir.publish(airValue);
       DustConcentration.publish(concentration);
    } 
    lastTime = millis();
   
   }
   
  //BLOCK FOR THE RELAY
  if(waterTimer.isTimerReady()){
    if (moistureRead >= 2400) {
    digitalWrite(D16, HIGH);
    delay(50);
    digitalWrite(D16, LOW);
    waterTimer.startTimer(timeTimer);
    } 
  } 
  //}
  //Block for Air quality sensor
  airValue = airQualitySensor.getValue();
  current_quality= airQualitySensor.slope();
  if (current_quality >= 0){
    if (current_quality==0)
      Serial.printf("High pollution! Force signal active %i\n", airValue);
      else if (current_quality==1)
        Serial.printf("High pollution! %i\n", airValue);
      else if (current_quality==2)
        Serial.printf("Low pollution! %i\n", airValue);
      else if (current_quality ==3)
        Serial.printf("Fresh air %i\n", airValue);
    }
  //Block for Dust Sensor
  duration = pulseIn(DUSTPIN, LOW);
  lowpulseoccupancy = lowpulseoccupancy+duration;
  if ((millis()-starttime) > sampletime_ms){      //if the sample time == 30s
    ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
    concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
    Serial.printf("LPO = %i\n", lowpulseoccupancy);
    Serial.printf("Ratio = %0.2f\n", ratio);
    Serial.printf("Conc = %0.2f\n", concentration);
    lowpulseoccupancy = 0;
    starttime = millis();
  }

}
//BME function to convert from tempC to tempF. currently not using pressPA but available
float bmeConverted(float tempF){
  pressPA = myReading.readPressure ();
  convertedPA = pressPA*0.00029530;
  tempF = (tempC*9/5)+32;
  return (tempF);
}
//connect to adafruit
void MQTT_connect() {
  int8_t ret;
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
    Serial.printf("Retrying MQTT connection in 5 seconds...\n");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}
bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;
  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}

