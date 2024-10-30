
/*************************************************************
  Download latest ERa library here:
    https://github.com/eoh-jsc/era-lib/releases/latest
    https://www.arduino.cc/reference/en/libraries/era
    https://registry.platformio.org/libraries/eoh-ltd/ERa/installation

    ERa website:                https://e-ra.io
    ERa blog:                   https://iotasia.org
    ERa forum:                  https://forum.eoh.io
    Follow us:                  https://www.fb.com/EoHPlatform
 *************************************************************/

// Enable debug console
// #define ERA_DEBUG

/* Define MQTT host */
#define DEFAULT_MQTT_HOST "mqtt1.eoh.io"

// You should get Auth Token in the ERa App or ERa Dashboard
#define ERA_AUTH_TOKEN "539da32f-5785-42e7-bc12-dc0475ef2c78"

/* Define setting button */
// #define BUTTON_PIN              0

#if defined(BUTTON_PIN)
    // Active low (false), Active high (true)
    #define BUTTON_INVERT       false
    #define BUTTON_HOLD_TIMEOUT 5000UL

    // This directive is used to specify whether the configuration should be erased.
    // If it's set to true, the configuration will be erased.
    #define ERA_ERASE_CONFIG    false
#endif

#include <Arduino.h>
#include <ERa.hpp>
#include <DFRobotDFPlayerMini.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>


const char ssid[] = "Smart Backpack";
const char pass[] = "12345678";

WiFiClient mbTcpClient;

#if (defined(ARDUINO_AVR_UNO) || defined(ESP8266))   // Using a soft serial port
#include <SoftwareSerial.h>
SoftwareSerial softSerial(/*rx =*/4, /*tx =*/5);
#define FPSerial softSerial
#else
#define FPSerial Serial1
#endif

DFRobotDFPlayerMini myDFPlayer;
SoftwareSerial Arduino(18, 19);

TinyGPSPlus gps;
SoftwareSerial ss(32, 33);

SoftwareSerial moduleSim(13, 14);

#define SOS 23
#define phoneNumber "+84706905513"

int ID = 0;
int SOSmode = 0;
float Latitude = 10.7766667, Longitude = 106.6792222;
const float Lat = 10.7766667, Lon = 106.6792222;
const float schoolLat = 10.7286320, schoolLon = 106.7266708;
float disfh = 0, disfs = 0, dis = 0;
int Month, Day, Year, Hour, Minute, Second;
uint64_t timer = 0;
String data = "";
char estr[1000];

#if defined(BUTTON_PIN)
    #include <ERa/ERaButton.hpp>

    ERaButton button;

    #if ERA_VERSION_NUMBER >= ERA_VERSION_VAL(1, 2, 0)
        static void eventButton(uint8_t pin, ButtonEventT event) {
            if (event != ButtonEventT::BUTTON_ON_HOLD) {
                return;
            }
            ERa.switchToConfig(ERA_ERASE_CONFIG);
            (void)pin;
        }
    #else
        static void eventButton(ButtonEventT event) {
            if (event != ButtonEventT::BUTTON_ON_HOLD) {
                return;
            }
            ERa.switchToConfig(ERA_ERASE_CONFIG);
        }
    #endif

    #if defined(ESP32)
        #include <pthread.h>

        pthread_t pthreadButton;

        static void* handlerButton(void* args) {
            for (;;) {
                button.run();
                ERaDelay(10);
            }
            pthread_exit(NULL);
        }

        void initButton() {
            pinMode(BUTTON_PIN, INPUT);
            button.setButton(BUTTON_PIN, digitalRead, eventButton,
                            BUTTON_INVERT).onHold(BUTTON_HOLD_TIMEOUT);
            pthread_create(&pthreadButton, NULL, handlerButton, NULL);
        }
    #elif defined(ESP8266)
        #include <Ticker.h>

        Ticker ticker;

        static void handlerButton() {
            button.run();
        }

        void initButton() {
            pinMode(BUTTON_PIN, INPUT);
            button.setButton(BUTTON_PIN, digitalRead, eventButton,
                            BUTTON_INVERT).onHold(BUTTON_HOLD_TIMEOUT);
            ticker.attach_ms(100, handlerButton);
        }
    #endif
#endif

/* This function will run every time ERa is connected */
ERA_CONNECTED() {
    ERA_LOG(ERA_PSTR("ERa"), ERA_PSTR("ERa connected!"));
}

/* This function will run every time ERa is disconnected */
ERA_DISCONNECTED() {
    ERA_LOG(ERA_PSTR("ERa"), ERA_PSTR("ERa disconnected!"));
}

/* This function print uptime every second */
void timerEvent() {
  ERa.virtualWrite(V0, Latitude);
  ERa.virtualWrite(V1, Longitude);
  if (millis() - timer > 60000 || SOSmode == 1) {
    ERa.virtualWrite(V2, estr);
    timer = millis();
  }
  ERa.virtualWrite(V3, disfh);
  ERa.virtualWrite(V4, disfs);
  ERa.virtualWrite(V5, dis);
  ERa.virtualWrite(V10, Month);
  ERa.virtualWrite(V11, Day);
  ERa.virtualWrite(V12, Year);
  ERa.virtualWrite(V13, (Hour + 7) % 24);
  ERa.virtualWrite(V14, Minute);
  ERa.virtualWrite(V15, Second);
  ERA_LOG(ERA_PSTR("Timer"), ERA_PSTR("Uptime: %d"), ERaMillis() / 1000L);
}

void setup() {
  /* Setup debug console */
  Serial.begin(115200);
  moduleSim.begin(115200);
  Arduino.begin(9600);
  ss.begin(9600);
  Serial.print("Testing TinyGPS library v. "); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println("by Mikal Hart");
  Serial.println();
  Serial.println("Sats HDOP Latitude  Longitude  Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum");
  Serial.println("          (deg)     (deg)      Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail");
  Serial.println("-------------------------------------------------------------------------------------------------------------------------------------");

  #if defined(BUTTON_PIN)
    /* Initializing button. */
    initButton();
    /* Enable read/write WiFi credentials */
    ERa.setPersistent(true);
  #endif

  /* Setup Client for Modbus TCP/IP */
  ERa.setModbusClient(mbTcpClient);

  /* Set scan WiFi. If activated, the board will scan
    and connect to the best quality WiFi. */
  ERa.setScanWiFi(true);
          
  /* Initializing the ERa library. */
  ERa.begin(ssid, pass);

  /* Setup timer called function every second */
  ERa.addInterval(1000L, timerEvent);

  #if (defined ESP32)
  FPSerial.begin(9600, SERIAL_8N1, /*rx =*/16, /*tx =*/17);
  #else
  FPSerial.begin(9600);
  #endif

  if (!myDFPlayer.begin(FPSerial, /*isACK = */true, /*doReset = */true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0);
    }
  }
  Serial.println(F("DFPlayer Mini online."));
  myDFPlayer.volume(30);
  setupModuleSim();
}

void loop() {
  smartDelay(500);
  Latitude  = gps.location.lat();
  Longitude = gps.location.lng();
  int dLat = (int) Latitude ;
  int dLon = (int) Longitude;
  int mLat = ((Latitude  - dLat) * 60);
  int mLon = ((Longitude - dLon) * 60);
  float sLat = ((Latitude  - dLat) * 60 - mLat) * 60;
  float sLon = ((Longitude - dLon) * 60 - mLon) * 60;
  Month     = gps.date.month() ;
  Day       = gps.date.day()   ;
  Year      = gps.date.year()  ;
  Hour      = gps.time.hour()  ;
  Minute    = gps.time.minute();
  Second    = gps.time.second();

  String dataa = "https://www.google.com/maps/place/" + String(dLat) + "%%C2%%B0" + String(mLat) + "'" + String(sLat, 1) + "%%22+";
  dataa += String(dLon) + "%%C2%%B0" + String(mLon) + "'" + String(sLon, 1) + "%%22/@";
  dataa += String(Latitude, 6) + "," + String(Longitude, 6) + ",19z/data=!3m1!4b1!4m4!3m3!8m2!3d";
  dataa += String(Latitude, 6) + "!4d" + String(Longitude, 6);
   
  disfs = distance(Latitude, Longitude, schoolLat, schoolLon);
  disfh = distance(Latitude, Longitude, Lat, Lon);
  dis   = distance(Lat, Lon, schoolLat, schoolLon);

  dataa.toCharArray(estr, 999);

  if (Arduino.available()) {
    String data = Arduino.readString();
    ID = data.toInt();

    if (((Hour + 7) % 24) >= 5 && ((Hour + 7) % 24) < 11) {
      if (ID == 4 || ID == 9) {
          goodMorning();
          khoaFather();
          Serial.println(String(Latitude, 6) + ", " + String(Longitude, 6));
          delay(1000);
      }
      else if (ID == 2 || ID == 7) {
          goodMorning();
          khoaMother();
          Serial.println(String(Latitude, 6) + ", " + String(Longitude, 6));
          delay(1000);
      }
      else if (ID == 1 || ID == 6) {
          goodMorning();
          minhKhoa();
          Serial.println(String(Latitude, 6) + ", " + String(Longitude, 6));
          delay(1000);
      }
    }
    else if (((Hour + 7) % 24) >= 11 && ((Hour + 7) % 24) < 18) {
      if (ID == 4  || ID == 9) {
          goodAfternoon();
          khoaFather();
          Serial.println(String(Latitude, 6) + ", " + String(Longitude, 6));
          delay(1000);
      }
      else if (ID == 2 || ID == 7) {
          goodAfternoon();
          khoaMother();
          Serial.println(String(Latitude, 6) + ", " + String(Longitude, 6));
          delay(1000);
      }
      else if (ID == 1 || ID == 6) {
          goodAfternoon();
          minhKhoa();
          Serial.println(String(Latitude, 6) + ", " + String(Longitude, 6));
          delay(1000);
      }
    }
    else {
      if (ID == 4  || ID == 9) {
          goodEvening();
          khoaFather();
          Serial.println(String(Latitude, 6) + ", " + String(Longitude, 6));
          delay(1000);
      }
      else if (ID == 2 || ID == 7) {
          goodEvening();
          khoaMother();
          Serial.println(String(Latitude, 6) + ", " + String(Longitude, 6));
          delay(1000);
      }
      else if (ID == 1 || ID == 6) {
          goodEvening();
          minhKhoa();
          Serial.println(String(Latitude, 6) + ", " + String(Longitude, 6));
          delay(1000);
      }
    }
  }

  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }

  if (digitalRead(SOS) == 1 || ID == -5 || ID == 3 || ID == 8) {
    SOSmode = 1;
    ID = 0;
  }
  else if (digitalRead(SOS) == 0) {
    SOSmode = 0;
  }

  if (ID == -3) {
    warning();
    ID = 0;
  }

  if (SOSmode == 1) {
    sendSMS(phoneNumber, dataa);
    SOSplay();
    callPhone(phoneNumber);
  }

  ERa.run();
}

float distance (float lat1, float lon1, float lat2, float lon2) {

  float dlat = lat2 - lat1;
  float dlon = lon2 - lon1;

  float a = sqrt(dlat * dlat + dlon * dlon);
  return a * ratio;
}

void setupModuleSim(){
  sim_at_cmd("AT");           // Kiểm tra AT Command
    delay(200);
    sim_at_cmd("AT+IPREX?");    // Kiểm tra tốc độ baud rate
    delay(200);
    sim_at_cmd("AT+IPREX=115200");   // Cài tốc độ baudrate
    delay(200);
    sim_at_cmd("ATI");          // Thông tin module SIM
    delay(200);
    sim_at_cmd("AT+CPIN?");     // Kiểm tra trạng thái của thẻ SIM
    delay(200);
    sim_at_cmd("AT+CSQ");       // Kiểm tra tín hiệu mạng
    delay(200);
    sim_at_cmd("AT+CIMI");
    delay(200);
    sim_at_cmd("AT+CMGF=1");    // khởi động chức năng SMS
    delay(100);  
    sim_at_cmd("AT+CNMI=1,2,0,0,0"); 
    delay(100);
    sim_at_cmd("AT+CMGL=\"REC UNREAD\"");          // đọc tin nhắn tới
    delay(100);
}

void sim_at_wait(){
    delay(100);
    while (moduleSim.available()) {
        Serial.write(moduleSim.read());
    }
}

bool sim_at_cmd(String cmd){
    moduleSim.println(cmd);
    sim_at_wait();
    return true;
}

bool sim_at_send(char c){
    moduleSim.write(c);
    return true;
}

void sendSMS(String phone, String data) {             // Hàm thực hiện gửi tin nhắn
    sim_at_cmd("AT+CMGF=1");
    String temp = "AT+CMGS=\"";
    temp += phone;
    temp += "\"";
    sim_at_cmd(temp);
    sim_at_cmd(data);
    sim_at_send(0x1A);    // End charactor for SMS
}

void callPhone(String phone){                   // Hàm thực hiện cuộc gọi
    String temp = "ATD";
    temp += phone;
    temp += ";";
    sim_at_cmd(temp);
    delay(10000);
    sim_at_cmd("ATH");
}

String getValue(String data, char separator, int index){
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
      if (data.charAt(i) == separator || i == maxIndex) {
          found++;
          strIndex[0] = strIndex[1] + 1;
          strIndex[1] = (i == maxIndex) ? i+1 : i;
      }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  } 
  while (millis() - start < ms);
}

void goodMorning() {
  myDFPlayer.play(12);
  delay(1000);
}

void goodAfternoon() {
  myDFPlayer.play(13);
  delay(1000);
}

void goodEvening() {
  myDFPlayer.play(14);
  delay(1000);
}

void khoaFather() {
  myDFPlayer.play(18);
  delay(1000);
}

void khoaMother() {
  myDFPlayer.play(19);
  delay(1000);
}

void minhKhoa() {
  myDFPlayer.play(20);
  delay(1000);
}

void SOSplay() {
  myDFPlayer.play(21);
  delay(5000);
}

void warning() {
  myDFPlayer.play(22);
  delay(3000);
}

void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}
