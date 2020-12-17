//#include <ESP8266_Lib.h>
//#include <BlynkSimpleShieldEsp8266.h>


#include <SimpleKalmanFilter.h>
#include "DHT.h"
#include <Wire.h>
#include <SoftwareSerial.h>

//blynk id
char auth[] = "DnXtkJW4APZoddoHc4fwxXMQqOcBdh5s"; 
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "ND";
char pass[] = "nd1994dn";
//BlynkTimer timer;



////// define MQ135 pin ///////////
#define MQ135_pin A0
////// softwareSerial defines /////
#define txPin 2
#define rxPin 3
////// DHT defines /////////
#define DHTPIN 8
#define DHTTYPE DHT11
////// BMP085 defines and variables////////
#define BMP085_ADDRESS 0x77 //I2C address of BMP085
const unsigned char OSS = 0;  // Oversampling Setting
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 
// Use these for altitude conversions
const float p0 = 101325;     // Pressure at sea level (Pa)
//////



unsigned long sending_time = 90000; //millis
unsigned long refresh_time ;
unsigned long notification_timer=0;
unsigned long notification_interval = 300000; //millis, 300000ms = 5 min

int steps = 0;
// data sending time in minutes = maxSteps/30
// 1 minute -> 30 ;    5 minutes->150;   ....
int maxSteps = 1; //1-> 2sec 


int countTrueCommand;
int countTimeCommand; 
boolean found = false;


struct data{
  float tem_dht =0; //temperature from DHT11
  float est_tem_dht = 0; //estimated by kalman filter
  float tem_bmp = 0; // temperature from bmp085
  float est_tem_bmp = 0;
  float hum = 0; //humidity
  float est_hum = 0;
  float hi =0; //heat index in celsius
  float est_hi = 0;
  float alt = 0; //altitude
  float est_alt = 0;
  long p = 0; //pressure
  long est_p = 0;
  float CO2 = 0;
  float est_CO2 = 0;
  
}data;

String str2send;
String str2read;

//String AP = "ND";
//String PASS = "nd1994dn";
String ID = "vB620C86C7546D19"; //pushingbox ID
String HOST = "api.thingspeak.com";
String PORT = "80";
String field_temperature = "field1";
String field_humidity = "field2";
String field_CO2 = "field3";
String field_pressure = "field4";
String field_altitude = "field5";
// String field_hi = "field6";

String ID_notif_unhealty_air = "vB23B01FEC3F78E5";
String ID_notif_veryUnhealty_air = "v9724D0626C9A70C";
String ID_notif_hazardous_air = "v718B912FAE3B00F";

 

//SimpleKalmanFilter(e_mea, e_est, q);
// e_mea: Measurement Uncertainty 
//e_est: Estimation Uncertainty 
// q: Process Noise
SimpleKalmanFilter KF_t(2, 2, 0.01);
SimpleKalmanFilter KF_h(2, 2, 0.01);
SimpleKalmanFilter KF_CO2(5, 2, 0.2);
SimpleKalmanFilter KF_P(2, 2, 0.01);
// Serial output refresh time



//  set up a new serial object
SoftwareSerial wifiSerial (rxPin,txPin);
//ESP8266 wifi(&wifiSerial);
// set up dht sensor
DHT dht(DHTPIN,DHTTYPE);


//void TimerEvent(){
//  Blynk.virtualWrite(V0, data.est_tem_bmp);
//}




void setup() {
  pinMode(rxPin,INPUT);
  pinMode(txPin,OUTPUT);
  
 // Blynk.begin(auth, wifi, ssid, pass);
  // Setup a function to be called every second
  //timer.setInterval(1000L, TimerEvent);
  
  Serial.begin(9600);
  wifiSerial.begin(9600);
  dht.begin();
  Wire.begin();

  
// checking and resetting WiFi module
 if(!WiFi_softRST()){
  Serial.println("WiFi module is not responding");
 }

  bmp085Calibration();
  delay(5000);
}






void loop() {
// command from serial monitor  
  while(Serial.available()){
    char s = Serial.read();
    str2send.concat(s);
  }
  Serial_2_wifiSerial(str2send);
  str2send="";

  read_sensors();
  Serial.print(refresh_time);
  Serial.print("          ");
  Serial.print(sending_time);
  Serial.print("          ");
  Serial.println(millis());
  
  //SerialPlot();
    
 if (millis() > refresh_time){

  
// Data into strings:
    String Data_t =String(data.tem_bmp/10); // temperatre data from bmp
    String Data_h = String(data.hum); // humidity data from DHT
    String Data_HI = String(data.hi); //heat index data
    String Data_CO2 = String(data.CO2); // CO2 data from MQ135
    String Data_p = String(data.p); //pressure data 
    String Data_a = String(data.alt); // Altitude data
// making a url String with data: 
    String url = "GET //pushingbox?devid=";
    url += ID;
    url += "&t=";
    url += Data_t;
    url += "&h=";
    url += Data_h;
    url += "&HI=";
    url += Data_HI;
    url += "&CO2=";
    url += Data_CO2;
    url += "&P=";
    url += Data_p;
    url += "&A=";
    url += Data_a;
    url += " HTTP/1.1\r\n";
    url += "Host: api.pushingbox.com\r\n\r\n";
    int l = url.length();

    sendData(url , l);// sending data to google sheet through pushingbox

    refresh_time = millis() + sending_time;

 }
//send_notif();
}

void sendData(String &url, int l){
 // ESP - configuration and sending data   
    sendCommand("AT+CIPMODE=0\r\n",5,"OK"); 
    sendCommand("AT+CIPMUX=1\r\n", 5,"OK");  
    sendCommand("AT+CIPSTART=4,\"TCP\",\"api.pushingbox.com\",80\r\n",15,"OK");
    sendCommand("AT+CIPSEND=4," + String(l) + "\r\n" , 10,">");
    sendCommand(url, 10,"OK");
    //Serial_2_wifiSerial(url);
    delay(100);
   
}


void send_notif(){
     if(data.CO2 > 0 && (millis()- notification_timer > notification_interval)){

      String url_notif = "/v2/pushes";
      String msgBody = "{\"type\": \"note\", \"title\": \"ESP8266\", \"body\": \"Hello World!\"}\r\n";
      
      String ID_notif = ID_notif_unhealty_air;
      if(data.CO2 > 201){ ID_notif = ID_notif_veryUnhealty_air;}
      else if (data.CO2> 301){ID_notif = ID_notif_hazardous_air;}
      
      
      sendCommand("AT+CIPSTART=4,\"TCP\",\"api.pushbullet.com\",433\r\n",15,"OK");
      sendCommand("AT+CIPSEND=4," + String("POST ") + url_notif + " HTTP/1.1\r\n" +
               "Host: " + "api.pushbullet.com"+ "\r\n" +
               "Authorization: Bearer " + ID_notif + "\r\n" +
               "Content-Type: application/json\r\n" +
               "Content-Length: " +
               String( msgBody.length()) + "\r\n\r\n" , 10,">");
               
      Serial_2_wifiSerial(msgBody);
      delay(100);
      
      notification_timer = millis();
   }
}


void Serial_2_wifiSerial(String sendStr){
  // Serial.println("ok");
  if(sendStr != "" ){
    Serial.print(sendStr);
    wifiSerial.print(sendStr);  
  }
  while(wifiSerial.available() ){
    str2read=wifiSerial.readString();
  }
  if(str2read != ""){
    Serial.println(str2read);
    str2read="";
  }
  sendStr = "";
  delay(100);
  
}

void cmdFromSerialMonitor(){
  // command from serial monitor  
  while(Serial.available()){
    str2send = Serial.readString();    
  }
    Serial_2_wifiSerial(str2send);
    str2send="";
}




bool WiFi_softRST(){
  wifiSerial.write("AT+RST\r\n");
  
  while(wifiSerial.available() ){
    char s = wifiSerial.read();
    str2read.concat(s); 
  }
  
  if(wifiSerial.find("OK")){
    Serial.println(str2read);
    return 1;
  }
  else
    return 0;
}



void read_sensors(){
  
 //******************* Reading DHT11 sensor *****************************
 //collect data in each 2 sec and sum up them, them after 5 minutes gets the average
  delay(2000);
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  data.tem_dht = dht.readTemperature();
  
  data.hum =   dht.readHumidity();
 
  
  
  // Compute heat index in Celsius (isFahreheit = false)
   data.hi =  dht.computeHeatIndex(data.tem_dht, data.hum, false);


//******************* Reading DHT11 sensor *****************************
  data.tem_bmp = bmp085GetTemperature(bmp085ReadUT());
  
  data.p =  bmp085GetPressure(bmp085ReadUP());
  
  data.alt =  (float)44330 * (1 - pow(((float) data.p/p0), 0.190295));
  

//****************** Reading MQ135 sensor ******************************
  data.CO2 =  analogRead(MQ135_pin); 

  // Check if any reads failed and exit early (to try again).
  if (isnan(data.hum) || isnan(data.tem_dht) || isnan(data.tem_bmp) || isnan(data.p) || isnan(data.alt) ||isnan(data.CO2) ) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

 // Kalman filtering : 

 data.est_tem_dht = KalmanFilter(data.tem_dht, "KF_t");
 data.est_tem_bmp =  KalmanFilter(data.tem_bmp, "KF_t");;
 data.est_hum =  KalmanFilter(data.hum, "KF_h");
 data.est_hi = dht.computeHeatIndex(data.est_tem_dht, data.est_hum, false);
 data.est_p =  KalmanFilter(data.p, "KF_P");
 data.est_alt =  (float)44330 * (1 - pow(((float) data.est_p/p0), 0.190295));
 data.est_CO2 =  KalmanFilter(data.CO2, "KF_CO2");
  
 
 
 
}



float KalmanFilter(float realVal,  String filterType ){
// filterType : KF_t, KF_h, KF_CO2, KF_P
  
  // read a reference value from A0 and map it from 0 to 100
  
  float estimated_value = 0;
  // add a noise to the reference value and use as the measured value
  float measured_value = realVal + random(-100,100)/100.0;

  if (filterType == "KF_t"){
    // calculate the estimated value with Kalman Filter
    estimated_value = KF_t.updateEstimate(measured_value);
   
  }
  else if(filterType == "KF_h"){   
  // calculate the estimated value with Kalman Filter
    estimated_value = KF_h.updateEstimate(measured_value);
    
  }
  else if(filterType == "KF_CO2"){    
  // calculate the estimated value with Kalman Filter
    estimated_value = KF_CO2.updateEstimate(measured_value);
  }
  else if(filterType == "KF_P"){    
  // calculate the estimated value with Kalman Filter
    estimated_value = KF_P.updateEstimate(measured_value);
    
  }

    
    return estimated_value;
  
}

void SerialPlot(){
//KF
 
 
//  Serial.print(F("Humidity: "));
//  Serial.print(data.hum);
//  Serial.print(" ");
//  Serial.print(F("Humidity KF: "));
//  Serial.print(data.est_hum);
//  Serial.print(" ");
//  
//  
//  Serial.print(F("DHT_Temperature: "));
//  Serial.print(data.tem_dht);
//  Serial.print(" ");
//  Serial.print(F("DHT_Temperature KF: "));
//  Serial.print(data.est_tem_dht);
//  Serial.print(" ");
//  
//  
//  Serial.print(F("Heat index: "));
//  Serial.print(data.hi);
//  Serial.print(" ");
//  Serial.print(F("Heat index KF: "));
//  //data.tem_dht = KalmanFilter(data.tem_dht,'KF_t');
//  data.hi = dht.computeHeatIndex(data.est_tem_dht, data.est_hum, false);
//  Serial.print(data.hi);
//  Serial.print(" ");
//
//  Serial.print("BMP_Temperature: ");
//  Serial.print(data.tem_bmp/10);
//  Serial.print(" ");
//  Serial.print("BMP_Temperature KF: ");
//  Serial.print(data.est_tem_bmp/10);
//  Serial.print(" ");
//  
//  Serial.print("Altitude: ");
//  Serial.print(data.alt, 2);
//  Serial.print(" ");
// Serial.print("Altitude KF: ");
//  Serial.print(data.est_alt, 2);
//  Serial.print(" ");

  Serial.print("Air Quality : ");
  Serial.print(data.CO2);
  Serial.print(" ");
  Serial.print("Air Quality KF: ");
 
  Serial.print(data.est_CO2);
  Serial.print(" ");
  
  Serial.println(" ");
 
}


void serial_print(){
  Serial.print(F("Humidity: "));
  Serial.print(data.hum);
  Serial.print(F("%   DHT_Temperature: "));
  Serial.print(data.tem_dht);
  Serial.print(F("°C "));
  Serial.print(F("   Heat index: "));
  Serial.print(data.hi);

  Serial.print("   |  BMP_Temperature: ");
  Serial.print(data.tem_bmp/10);
  Serial.print(" °C");
  Serial.print("   Pressure: ");
  Serial.print(data.p,DEC);
  Serial.print(" Pa");
  Serial.print("   Altitude: ");
  Serial.print(data.alt, 2);
  Serial.print(" m");
  Serial.print("   Air Quality : ");
  Serial.print(data.CO2);
  Serial.println(" ppm");
  
  Serial.println();
}

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;
  
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);  
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
  return p;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;
    
  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  
  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;
  
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  
  // Wait at least 4.5ms
  delay(5);
  
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();
  
  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));
  
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);
  
  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();
  
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  
  return up;
}


void sendCommand(String command, int maxTime, char readReplay[]){
  Serial.print(countTrueCommand);
  Serial.print(". at command => ");
  Serial.print(command);
  Serial.print(" ");
  while(countTimeCommand < (maxTime*1))
  {
    wifiSerial.print(command);//at+cipsend
    if(wifiSerial.find(readReplay))//ok
    {
      found = true;
      break;
    }
  
    countTimeCommand++;
  }
  
  if(found == true)
  {
    Serial.println("OYI");
    countTrueCommand++;
    countTimeCommand = 0;
  }
  
  if(found == false)
  {
    Serial.println("Fail");
    countTrueCommand = 0;
    countTimeCommand = 0;
  }
  
  found = false;
 
}
