#include <WiFiNINA.h>     //아두이노 와이파이2 보드용 라이브러리(와이파이를 이용한 서버 접속)
#include <ArduinoJson.h>  //서버-클라이언트 데이터 파싱을 위한 라이브라리.(JASON 형식)

#include <DallasTemperature.h>  //온도 센서를 위한 라이브러리

#include <Arduino_LSM6DS3.h>  //자이로 센서를 위한 라이브러리

#include <NTPClient.h>
#include <WiFiUdp.h>    //서버 시간을 얻어오기 위한 라이브러리(NTPClient로 부터 시간 GET)
//------------------------------------------------------------------------------
#define _WARNING -1
#define _SAFE 0
#define _FIRE 1
#define _GAS 2
#define _QUAKE 3  //SBAS 상태 사전 정의 키워드

#define _PORT 8888  //임의의 값, 조정 가능

volatile int SBAS_status = _SAFE;  //현재 상태 저장 변수
//------------------------------------------------------------------------------
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature temp(&oneWire);  //온도 감지 모듈 핀 지정 및 객체 생성

int flame_sensor = 7;  //불꽃 감지 모듈 핀

int gas_flammable = A1;
int gas_CO = A0;  //MQ센서 모듈 핀 mq7-CO, mq6-인화성가스

int sw = 13;  //SW 모듈 핀(tact-switch, 인터럽트용)

int led_G = 9;
int led_R = 10;
int led_B = 6;  //3-Color LED 모듈 핀

int buzzer = 12;  //수동 부저 모듈 핀
//------------------------------------------------------------------------------

IPAddress serverIP(xxx, xx, x, xx);  //x자리에 접속하고자 하는 IP 입력

char ssid[] = "YOUR_SSID";
char pass[] = "YOUR_PSSWD";

int port = 80;  //임의의 값, 조정 가능
int status = WL_IDLE_STATUS;

WiFiServer server(port);  //port 80으로 와이파이 서버 객체 생성
WiFiClient client;      //클라리언트 객체 생성
//-----------------------------------------------------------------------------
char ntpServer = "pool.ntp.org";    //ntp로 부터 url 가져옴
int timeZone = 9;                  //GMT, 그리니치 기준 시 설정 (한국이므로 +9시간)
int summerTime = 0;                //섬머타임 적용하지 않음

WiFiUDP ntpUDP;                                            //WiFiUDP를 통해
NTPClient timeClient(ntpUDP, ntpServer, timeZone*3600);    //NTP로부터 시간을 가져오는 객체 생성
//-----------------------------------------------------------------------------
const char* token;             //서버로부터 토큰 값을 가져올 변수

unsigned long previousMillis = 0;  //이전 시간 저장
const long interval = 500;         //인터벌 타임 지정

static unsigned long earthquakeStartTime = 0;    // 지진 시작 시간을 기록하는 변수
const unsigned long earthquakeThreshold = 1500;  // 1.5초 이상일 때 감지

bool isPushed;     //스위치 눌림 확인 변수
bool toggleAlert;  //알람 On/Off 변수

bool toggleFire = true;
bool toggleGas = true;
bool toggleQuake = true;      //각 센서별 토글 변수, 외부 인터럽트용
//--------------------------------------------------------------------------------
void setup() {  //초기화 동작
  isPushed = false;        //토글 변수 초기값: False
  //Serial.begin(9600);    //디버깅용
  init_rgbLED();            //LED핀 초기화
  init_sensors();           //센서 초기화
  pinMode(sw, INPUT_PULLUP);  //풀업 저항 스위치 설정

  connectWiFi();          //초기화(장치 동작) 시에 와이파이 연결

  server.begin();        //서버 시작
  //printWifiStatus();    //디버깅용, 와이파이 상태창 표시
  setupTime();            //서버 시간 설정
}
 //---------------------------------------------------------------------------------
void init_rgbLED() {  //LED모듈 초기화 함수
  pinMode(led_G, OUTPUT);
  pinMode(led_R, OUTPUT);
  pinMode(led_B, OUTPUT);
}
void startGyro() {  //내장 자이로 센서 초기화 함수
  if (!IMU.begin()) {
    Serial.println("Failed to init GYRO");  //<- 없어도 되긴 함(디버깅용)
    while (1)
      ;
  }
}
void init_sensors() {  //각종 센서 초기화 함수
  pinMode(flame_sensor, INPUT);
  pinMode(gas_CO, INPUT);
  pinMode(gas_flammable, INPUT);
  temp.begin();
  startGyro();
}
void ledColor(int red, int green, int blue) {  //LED 색 지정 함수
  analogWrite(led_R, red);
  analogWrite(led_G, green);
  analogWrite(led_B, blue);
}
void buzzerLedPattern(int SBAS_status) {  //하드웨어 동작부(LED 및 부저)
  unsigned long currentMillis = millis();
  switch (SBAS_status) {
    case _SAFE:
      ledColor(32, 32, 32);
      noTone(buzzer);
      break;

    case _WARNING:
      ledColor(128, 128, 0);
      if (currentMillis - previousMillis >= interval) {
        tone(buzzer, 300);
        previousMillis = currentMillis;
      } else noTone(buzzer);
      break;

    case _FIRE:
      ledColor(255, 0, 0);
      if (currentMillis - previousMillis >= interval / 2) {
        tone(buzzer, 800);
        previousMillis = currentMillis;
      } else noTone(buzzer);
      break;

    case _GAS:
      ledColor(128, 255, 0);
      if (currentMillis - previousMillis >= interval / 2) {
        tone(buzzer, 550);
        previousMillis = currentMillis;
      } else noTone(buzzer);
      break;

    case _QUAKE:
      ledColor(0, 0, 255);
      if (currentMillis - previousMillis >= interval) {
        tone(buzzer, 1100);
        previousMillis = currentMillis;
      } else noTone(buzzer);
      break;
  }
}
bool flameSensor() {  //불꽃 감지 센서 동작부
  bool flame;         //불꽃 감지 상태 변수
  flame = (digitalRead(flame_sensor));
  return flame;
}

float tempSensor() {  //온도 감지 센서 동작부
  float celcius;      //온도 표시 변수
  temp.requestTemperatures();
  celcius = temp.getTempCByIndex(0);
  return celcius;
}

float gasSensorCO() {  //CO 센서
  float ppmCO;
  ppmCO = (analogRead(gas_CO)) - 80.0;
  return ppmCO;
}

float gasSensorFlammable() {  //인화성 가스 센서
  float ppmFlammable;
  ppmFlammable = (analogRead(gas_flammable)) - 40.0;
  return ppmFlammable;
}

bool gyroSensor() {      //지진 감지 센서 동작부 (n초 이상 감지시 값 리턴)
  static float x, y, z;  //자이로 센서에 이용될 변수

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
  }
  if (abs(x) > 0.3 || abs(y) > 0.3) {
    // 최초 감지 시간을 설정
    if (earthquakeStartTime == 0) {
      earthquakeStartTime = millis();
    }
    return false;
  } else {
    if (earthquakeStartTime != 0) {
      if (millis() - earthquakeStartTime >= earthquakeThreshold) {
        Serial.println("Earthquake threshold reached, alarm ON");
        return true;
      } else return false;
    } else return false;
  }
}
String getTime() {
  String formattedDate;
  formattedDate = timeClient.getFormattedTime();
  return formattedDate;
}
//-------------------------------------------------------------------------------------------------------------
void mainService(bool flame, float temp, float gas_c, float gas_f, bool gyro) {
  SBAS_status = _SAFE;

  printSerial(flameSensor(), tempSensor(), gasSensorCO(), gasSensorFlammable(), gyroSensor());

  if (toggleFire) {
    if (flame == true && temp >= 60.0) {
      SBAS_status = _FIRE;
      Serial.println("    Alert! FIRE Detected!    ");
    } else if (flame == true || temp >= 60.0) {
      SBAS_status = _WARNING;
      Serial.println("    Caution! FIRE Danger!    ");
    }
  }

  if (toggleGas) {
    if (gas_c >= 800 || gas_f >= 2000) {
      SBAS_status = _GAS;
      Serial.println("    Alert! GAS Detected!    ");
    } else if (gas_c >= 500 && gas_c < 750 || gas_f >= 1000 && gas_f < 2000) {
      SBAS_status = _WARNING;
      Serial.println("    Caution! Gas Danger!    ");
    }
  }

  if (toggleQuake) {
    if (gyro) {
      SBAS_status = _QUAKE;
      Serial.println("    Alert! EARTHQUAKE Detected!    ");
      earthquakeStartTime = 0;
    }
  }

  Serial.println(SBAS_status);  //디버깅용
}

void printSerial(bool flame, float temp, float gas_c, float gas_f, bool gyro) {  //디버깅용 상태창 출력 함수
  if (!toggleAlert) {
    Serial.println("----------Current Building's Statuses----------");
    Serial.print("Flame: ");
    Serial.println(flame);
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println(" °C");
    Serial.print("CO ppm: ");
    Serial.print(gas_c);
    Serial.println(" ppm");
    Serial.print("Flammable Gas ppm: ");
    Serial.print(gas_f);
    Serial.println(" ppm");
    Serial.print("Earthquake: ");
    Serial.println(gyro);
    Serial.println("-----------------------------------------------");
    Serial.println(" ");
  }
}
//************************************************************************************************************************************//
void loop() {
  unsigned long actTime = millis() / 1000;

  client = server.available();

  mainService(flameSensor(), tempSensor(), gasSensorCO(), gasSensorFlammable(), gyroSensor());  //내부 동작부

  handleDevice(toggleControl());

  pushData(actTime);

  Serial.println(getTime());
  Serial.println(actTime); 
 
}
//******************************************************************************************************************************//
void pushData(unsigned long actTime){
  if(actTime%600 == 0){
    sendData_fire();
    sendData_gas();
    sendData_EarthQuake();
  }
  else{
    switch(SBAS_status){
      case _FIRE: sendData_fire();  break;
      case _GAS: sendData_gas();  break;
      case _WARNING: sendData_fire(); sendData_gas(); break;
      case _QUAKE: sendData_EarthQuake(); break;
    }
    sendFCM(getToken());
  }
}

bool toggleControl(){
  if(digitalRead(sw) == LOW && !isPushed){
    isPushed = true;
    toggleAlert = !toggleAlert;  //스위치 눌리면 알람 상태 on-off 변환
    toggleFire = !toggleFire;
    toggleGas = !toggleGas;
    toggleQuake = !toggleQuake;  //경보기 일괄 해제/설정(버튼 1개이기 때문)
    delay(200);
  }
  else if (digitalRead(sw) == HIGH) isPushed = false;

  return toggleAlert;
}

void handleDevice(bool control){
    if(!control) buzzerLedPattern(SBAS_status);
    else { ledColor(0, 0, 0); noTone(buzzer);
      Serial.println("*****All Alarms Are Disabled. Check Your Device.***** "); }
}

void printWifiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  delay(5000);
}

void connectWiFi() {
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);

    delay(5000);
  }
}

void setupTime() {
  timeClient.begin();
  timeClient.setTimeOffset(3600 * 9);
  timeClient.forceUpdate();
}

void sendRequest(const char* endpoint, DynamicJsonDocument& doc) {
    if (client.connect(serverIP, _PORT)) {
        Serial.println("Connected to Server");

        String jsonString;
        serializeJson(doc, jsonString);

        client.print("POST /");
        client.print(endpoint);
        client.print(" HTTP/1.1\r\n");
        client.print("Host: ");
        client.println(serverIP);
        client.print(":");
        client.print(_PORT);
        client.println("Connection: close");
        client.println("Content-Type: application/json");
        client.print("Content-Length: ");
        client.println(jsonString.length() + 1);
        client.println();
        client.println(jsonString);
        Serial.println(jsonString);
        Serial.println("HTTP Request Sent");

        delay(500);

        if (client.available()) {
            // 클라이언트 버퍼에 읽을 수 있는 데이터가 있는 경우
            while (client.available()) {
                // 클라이언트에서 한 바이트씩 데이터를 읽어옴
                char c = client.read();
                // 읽어온 데이터를 처리하거나 출력함
                Serial.print(c);
                String response = client.readStringUntil('\r');
                Serial.println("Response Code: " + response);
            }
        } else {
            // 클라이언트 버퍼에 읽을 수 있는 데이터가 없는 경우
            Serial.println("No data available");
        }

        Serial.println();
        Serial.println("Closing connection");
        client.stop();
    } else {
        Serial.println("Connection to server failed");
    }

    delay(100);
}

void sendData_fire(){
    DynamicJsonDocument doc(1024);
    doc["flame"] = String(flameSensor());
    doc["temperature"] = String(tempSensor());
    doc["sensorTime"] = getTime();
    sendRequest("fire/save", doc);
}

void sendData_EarthQuake() {
    DynamicJsonDocument doc(1024);
    doc["vibrationSensor"] = String(gyroSensor());
    doc["sensorTime"] = getTime();
    sendRequest("quake/save", doc);
}

void sendData_gas() {
    DynamicJsonDocument doc(1024);
    doc["flammable"] = String(gasSensorFlammable());
    doc["co"] = String(gasSensorCO());
    doc["sensorTime"] = getTime();
    sendRequest("gas/save", doc);
}

const char* getToken() {
    if (client.connect(serverIP, _PORT)) {
        Serial.println("Connected to Server");

        client.print("GET /");
        client.print("token/list");
        client.print(" HTTP/1.1\r\n");
        client.print("Host: ");
        client.println(serverIP);
        client.print(":");
        client.print(_PORT); 
        client.println("Connection: close");
        client.println();

        delay(500);

        while(client.connected() || client.available()) {
            if(client.available()) {
                String data = client.readStringUntil('\r');
                DynamicJsonDocument doc(1024);
                deserializeJson(doc, data);

                token = doc["token"];
            }
        }
        client.stop();

        delay(1000);

        Serial.println("Communication Ended.");
    }
    return token;
}

void sendFCM(const char* token) {
    DynamicJsonDocument doc(1024);
    doc["message"]["token"] = String(token);
    doc["message"]["data"]["warningId"] = String(SBAS_status);
    sendRequest("fcm/sendData", doc);
}
