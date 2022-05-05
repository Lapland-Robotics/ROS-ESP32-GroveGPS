#include "WiFi.h"
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>

//////////////////////
// WiFi Definitions //
//////////////////////
const char* ssid = ""; // Name of the WiFi network
const char* password = ""; // WiFi network password

TinyGPS gps;
SoftwareSerial ss(3, 1); // RX, TX
IPAddress server(172, 16, 200, 98); // IP of ROS server in example 192, 168, 0, 100
IPAddress ip_address;
int status = WL_IDLE_STATUS;
char gpsData[100] = ""; // Read GPS module data to this char array
int separatorVal = 17; // Operates data separation on gpsData, value includes amount of used print functions in loop plus one extra time on print date function


class WiFiHardware {

  public:
  WiFiHardware() {};

  void init() {
    // do your initialization here. this probably includes TCP server/client setup
    client.connect(server, 11411);
  }

  // read a byte from the serial port. -1 = failure
  int read() {
    // implement this method so that it reads a byte from the TCP connection and returns it
    //  you may return -1 is there is an error; for example if the TCP connection is not open
    return client.read();         //will return -1 when it will works
  }

  // write data to the connection to ROS
  void write(uint8_t* data, int length) {
    // implement this so that it takes the arguments and writes or prints them to the TCP connection
    for(int i=0; i<length; i++)
      client.write(data[i]);
  }

  // returns milliseconds since start of program
  unsigned long time() {
     return millis(); // easy; did this one for you
  }

  protected:
    WiFiClient client;
  
};

void setupWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to"); Serial.println(ssid);
    while(1) delay(500);
  }
  Serial.print("Ready! Address is ");
  Serial.print(WiFi.localIP());
}

std_msgs::String str_msg;
ros::NodeHandle_<WiFiHardware> nh;
ros::Publisher chatter("testdata", &str_msg);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("WiFi Setup");
  setupWiFi();
  delay(2000);
  Serial.println("GPS module setup");
  ss.begin(9600);
  delay(2000);
  Serial.println("GPS module setup done");
  delay(2000);
  Serial.println("init node");
  nh.initNode();
  Serial.println("init node done");
  delay(10000);
  nh.advertise(chatter);
  Serial.println("\nSetup done");
}

void loop() {
  float flat, flon;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

/*
  print_int(gps.satellites(), TinyGPS::GPS_INVALID_SATELLITES, 5);
  print_int(gps.hdop(), TinyGPS::GPS_INVALID_HDOP, 5);
  gps.f_get_position(&flat, &flon, &age);
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 2); // 6
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 11, 2); // 6
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  print_date(gps);
  
  print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 7, 2);
  print_float(gps.f_course(), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
  print_float(gps.f_speed_kmph(), TinyGPS::GPS_INVALID_F_SPEED, 6, 2);
  print_str(gps.f_course() == TinyGPS::GPS_INVALID_F_ANGLE ? "****" : TinyGPS::cardinal(gps.f_course()), 6);
  print_int(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0xFFFFFFFF : (unsigned long)TinyGPS::distance_between(flat, flon, LONDON_LAT, LONDON_LON) / 1000, 0xFFFFFFFF, 9);
  print_float(flat == TinyGPS::GPS_INVALID_F_ANGLE ? TinyGPS::GPS_INVALID_F_ANGLE : TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON), TinyGPS::GPS_INVALID_F_ANGLE, 7, 2);
  print_str(flat == TinyGPS::GPS_INVALID_F_ANGLE ? "****" : TinyGPS::cardinal(TinyGPS::course_to(flat, flon, LONDON_LAT, LONDON_LON)), 6);
  
  gps.stats(&chars, &sentences, &failed);
  print_int(chars, 0xFFFFFFFF, 6);
  print_int(sentences, 0xFFFFFFFF, 10);
  print_int(failed, 0xFFFFFFFF, 9);
  */
  strcat(gpsData, "5,168,66.48,25.72,755,04/27/2022 13:15:17,767,118.40,314.80,0.48,NW,2196,233.44,SW,16815,42,2");
  delay(2500);
  str_msg.data = gpsData;
  chatter.publish( &str_msg );
  strcpy(gpsData, ""); // Empty the char array for a new round in this loop
  
  nh.spinOnce();
  smartdelay(1000); // Very important for getting GPS data
  
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void print_int(unsigned long val, unsigned long invalid, int len)
{
  separatorVal--;
  char sz[32];
  
  if (val == invalid)
    strcpy(sz, "****");
  else
    sprintf(sz, "%ld", val);
  
  strcat(gpsData, sz);
  print_separator();
  smartdelay(0);
}

static void print_float(float val, float invalid, int len, int prec)
{
  separatorVal--;
  
  if (val != invalid)
  {
    char temp[32];
    dtostrf(val, 0, prec, temp);
    strcat(gpsData, temp);
    strcpy(temp, "");
  }
  else if (val == invalid)
  {
    strcat(gpsData, "****");
  }
  print_separator();
  smartdelay(0);
}

static void print_date(TinyGPS &gps)
{
  separatorVal--;
  
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    strcat(gpsData, "****");
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d",
        month, day, year, hour, minute, second);
    strcat(gpsData, sz);
  }
  print_separator();
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartdelay(0);
}

static void print_str(const char *str, int len)
{
  separatorVal--;
  
  strcat(gpsData, str);
  print_separator();
  smartdelay(0);
}

void print_separator() {
  if (separatorVal > 0) {
    strcat(gpsData, ",");
  }
  else if (separatorVal <= 0)
  {
    separatorVal = 17;
  }
}
