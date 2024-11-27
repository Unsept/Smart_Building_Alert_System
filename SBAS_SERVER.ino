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
int led_B = 6;  //3-Color LED 모듈 핀, PWM 제어 가능

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
const long interval = 500;         //인터벌 타임 지정, 0.5s

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
  init_rgbLED();            //LED핀 초기화  code => Line 86~90
  init_sensors();           //센서 초기화   code => Line 97~103
  pinMode(sw, INPUT_PULLUP);  //풀업 저항 스위치 설정

  connectWiFi();          //초기화(장치 동작) 시에 와이파이 연결  code => Line 323~331

  server.begin();        //서버 시작
  //printWifiStatus();    //디버깅용, 와이파이 상태창 표시 code => Line 312~321
  setupTime();            //서버 시간 설정, code => Line 333~337
}
 //---------------------------------------------------------------------------------
void init_rgbLED() {  //LED모듈 초기화 함수
  pinMode(led_G, OUTPUT);
  pinMode(led_R, OUTPUT);
  pinMode(led_B, OUTPUT);
}
void startGyro() {  //내장 자이로 센서 초기화 함수
  if (!IMU.begin()) {
   // Serial.println("Failed to init GYRO");  //초기화 실패시 시리얼 모니터에 출력
    while (1);
  }
}
void init_sensors() {  //각종 센서 초기화 함수
  pinMode(flame_sensor, INPUT);      //불꽃 감지
  pinMode(gas_CO, INPUT);            //일산화 탄소
  pinMode(gas_flammable, INPUT);     //인화성 가스
  temp.begin();                //온도 센서 시작
  startGyro();                //자이로 센서 초기화
}
void ledColor(int red, int green, int blue) {  //LED 색 지정 함수
  analogWrite(led_R, red);
  analogWrite(led_G, green);
  analogWrite(led_B, blue);        //PWM 제어이므로 아날로그 값
}
void buzzerLedPattern(int SBAS_status) {  //H/W 동작부(LED 및 부저)
  unsigned long currentMillis = millis();  //아두이노의 동작 시간 저장. 멀티 쓰레딩 흉내내기
  switch (SBAS_status) {      //SBAS의 상태가
    case _SAFE:               //안전이라면
      ledColor(32, 32, 32);    //옅은 백색
      noTone(buzzer);          //경고음 없음
      break;

    case _WARNING:            //경고라면
      ledColor(128, 128, 0);   //황색 표시
      if (currentMillis - previousMillis >= interval) {  //아도이노 동작시간 - 이전 동작 시간 값 >= 0.5s 라면
        tone(buzzer, 300);     //300Hz의 경고음
        previousMillis = currentMillis;    //이전 동작 시간 값에 현재 동작 시간 값 저장
      } else noTone(buzzer);    //경고음 해제
      break;

    case _FIRE:                //화재라면
      ledColor(255, 0, 0);      //밝은 적색 표시
      if (currentMillis - previousMillis >= interval / 2) {  //WARING의 경우보다 2배 빠른 주기
        tone(buzzer, 800);      //800Hz의 경고음
        previousMillis = currentMillis; 
      } else noTone(buzzer);
      break;

    case _GAS:                  //가스 유출이라면
      ledColor(128, 255, 0);      //녹황색 표시
      if (currentMillis - previousMillis >= interval / 2) {
        tone(buzzer, 550);        //550Hz 경고음
        previousMillis = currentMillis;
      } else noTone(buzzer);
      break;

    case _QUAKE:                  //지진이라면
      ledColor(0, 0, 255);        //청색 표시
      if (currentMillis - previousMillis >= interval) {
        tone(buzzer, 1100);      //1100Hz 경고음
        previousMillis = currentMillis;
      } else noTone(buzzer);
      break;
  }
}
bool flameSensor() {  //불꽃 감지 센서 동작부, bool 값 리턴
  bool flame;         //불꽃 감지 상태 변수, 불꽃 감지 센서를 디지털 센서로 사용. (0 or 1)
  flame = (digitalRead(flame_sensor));  //센서 값 저장
  return flame;
}

float tempSensor() {  //온도 감지 센서 동작부, float 값 리턴
  float celcius;      //온도 표시 변수, 소수점 단위까지 센싱하므로 float 사용
  temp.requestTemperatures();  //기본 내장함수로, 온도 값을 요청함
  celcius = temp.getTempCByIndex(0);  //온도 값 저장
  return celcius;
}

float gasSensorCO() {  //CO 센서, float 값 리턴
  float ppmCO;          //CO 농도 변수
  ppmCO = (analogRead(gas_CO)) - 80.0;    //80의 값은 편차 조정 용도, 아날로그 센싱
  return ppmCO;
}

float gasSensorFlammable() {  //인화성 가스 센서, 위와 동일하므로 생략
  float ppmFlammable;
  ppmFlammable = (analogRead(gas_flammable)) - 40.0;
  return ppmFlammable;
}

bool gyroSensor() {      //지진 감지 센서 동작부 (n초 이상 감지시 값 리턴)
  static float x, y, z;  //자이로 센서에 이용될 변수

  if (IMU.accelerationAvailable()) {    //자이로 센서가 활성화 되어있다면
    IMU.readAcceleration(x, y, z);      //x, y, z 각 축의 값을 저장(z축은 사용x -> 지진이므로 횡축의 진동만 사용)
  }                                      //IMU 센서는 자이로 센서보다는 가속도 센서에 가까우므로 x, y, z(수직축) 중 x, y의 편차 만을 사용
  if (abs(x) > 0.3 || abs(y) > 0.3) {    //x 혹은 y의 값이 0.3보다 크다면
    // 최초 감지 시간을 설정
    if (earthquakeStartTime == 0) {      //최초 지진 감지 시각이 0이라면
      earthquakeStartTime = millis();     //동작 시간으로 시간 기록
    }
    return false;    //0 반환
  } else {
    if (earthquakeStartTime != 0) {        //지진 감지 시간이 0이 아니고
      if (millis() - earthquakeStartTime >= earthquakeThreshold) {    //동작 시간 - 지진 시간 >= 1.5 라면
       // Serial.println("Earthquake threshold reached, alarm ON");    //디버깅용 코드
        return true;        //1 반환
      } else return false;  
    } else return false;    //그 외의 경우는 모두 0 반환
  }
}
String getTime() {      //시간 값을 문자열로 반환
  String formattedDate;    //서버 시간이 저장될 변수
  formattedDate = timeClient.getFormattedTime();      //timeClient 객체를 통해 형식이 지정된 값 저장 (HH:MM:SS)
  return formattedDate;  //반환
}
//-------------------------------------------------------------------------------------------------------------
void mainService(bool flame, float temp, float gas_c, float gas_f, bool gyro) {    //H/W 주동작 함수 구현부, 각각 불꽃, 온도, CO, 인화가스, 지진센서를 의미
  SBAS_status = _SAFE;      //초기 값은 SAFE 상태.

 // printSerial(flameSensor(), tempSensor(), gasSensorCO(), gasSensorFlammable(), gyroSensor()); //디버깅 코드, 센싱 값 표시용

  if (toggleFire) {                //화재 센서가 활성화 되어있을 때,
    if (flame == true && temp >= 60.0) {    //불꽃이 감지되고 60도씨 이상이라면
      SBAS_status = _FIRE;                   //화재 상태
     // Serial.println("    Alert! FIRE Detected!    ");    //디버깅용 코드
    } else if (flame == true || temp >= 60.0) {  //불꽃만 감지되거나 60도씨 이상이라면
      SBAS_status = _WARNING;                    //경고 상태
     // Serial.println("    Caution! FIRE Danger!    ");   //디버깅용 코드
    }
  }

  if (toggleGas) {                //가스 센서가 활성화 되어있을 때,          (센싱 값은 변화할 수 있음)
    if (gas_c >= 800 || gas_f >= 2000) {    //CO의 농도가 800ppm 이상이거나 인화성 가스의 농도가 2000ppm 이상이라면
      SBAS_status = _GAS;                //가스 상태
      //Serial.println("    Alert! GAS Detected!    ");  //디버깅용 코드
    } else if (gas_c >= 500 && gas_c < 750 || gas_f >= 1000 && gas_f < 2000) {  //500 <= CO_ppm < 750 이거나 1000 <= 인화성_ppm < 2000 이라면
      SBAS_status = _WARNING;            //경고 상태
     //Serial.println("    Caution! Gas Danger!    ");    //디버깅용 코드
    }
  }

  if (toggleQuake) {          //지진 센서가 활성화 되어있을 때,
    if (gyro) {              //자이로 센서 값이 1이라면
      SBAS_status = _QUAKE;    //지진 상태
      //Serial.println("    Alert! EARTHQUAKE Detected!    ");  //디버깅용 코드
      earthquakeStartTime = 0;  //지진 시간 값 초기화
    }
  }

  //Serial.println(SBAS_status);  //디버깅용
}

/*void printSerial(bool flame, float temp, float gas_c, float gas_f, bool gyro) {  //디버깅용 상태창 출력 함수
  if (!toggleAlert) {
    Serial.println("----------Current Building's Statuses----------");
    Serial.print("Flame: ");                //화재 센서 값
    Serial.println(flame);
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println(" °C");
    Serial.print("CO ppm: ");                  //가스 센서 값
    Serial.print(gas_c);
    Serial.println(" ppm");
    Serial.print("Flammable Gas ppm: ");
    Serial.print(gas_f);
    Serial.println(" ppm");
    Serial.print("Earthquake: ");              //지진 센서 값
    Serial.println(gyro);
    Serial.println("-----------------------------------------------");
    Serial.println(" ");
  }
}*/
//************************************************************************************************************************************//
void loop() {    //주 동작 함수
  unsigned long actTime = millis() / 1000;    //내부 타이머 변수로 일정 시간마다 서버로 데이터를 보내기 위함. ms -> s로 변환

  client = server.available();        //서버-클라이언트 객체 생성

  mainService(flameSensor(), tempSensor(), gasSensorCO(), gasSensorFlammable(), gyroSensor());  //내부 동작부, 상기 H/W 주 동작 함수의 매개변수들로 센서 이용

  handleDevice(toggleControl());    //버튼 스위치의 토글 값에 종손된 함수 code => Line 306~310.  toggleControl(): 버튼 스위치의 동작 값. code => Line 292~304

  pushData(actTime);  //actTime 값에 따라 데이터를 서버로 전송(push)

 //Serial.println(getTime());
 //Serial.println(actTime);         //디버깅용
 
}
//******************************************************************************************************************************//
void pushData(unsigned long actTime){    //장치 동작 시간을 파라미터로 이용
  if(actTime%600 == 0){      //10분 단위로(자동적으로)
    sendData_fire();    
    sendData_gas();
    sendData_EarthQuake();  //화재, 가스, 지진 센서의 값을 서버로 전달
  }
  else{                 //이벤트 발생 시(경보기 동작 시)
    switch(SBAS_status){  //SBAS 상태 값에 따라 독립적으로 전달
      case _FIRE: sendData_fire();  break;  //화재 경보기 값
      case _GAS: sendData_gas();  break;    //가스 경보기 값
      case _WARNING: sendData_fire(); sendData_gas(); break;    //화재 및 가스 경보기 값
      case _QUAKE: sendData_EarthQuake(); break;    //지진 경보기 값
    }
    sendFCM(getToken());      //사용자 어플을 통해 알람을 보내기 위한 처리 함수.
  }                            //경보기(H/W) -> 서버 -> 사용자 어플| 이때 token 값 이용
}                          //서버 관련 함수는 Line 389부터

bool toggleControl(){    //버튼 값을 bool로 반환하는 함수
  if(digitalRead(sw) == LOW && !isPushed){  //버튼이 눌렸을 때
    isPushed = true;            //버튼 감지 값 1
    toggleAlert = !toggleAlert;  //스위치 눌리면 알람 상태 on-off 변환
    toggleFire = !toggleFire;
    toggleGas = !toggleGas;
    toggleQuake = !toggleQuake;  //경보기 일괄 해제/설정(버튼 1개이기 때문)
    delay(200);    //채터링 방지용이나, H/W의 더딘 동작으로 이어질 수 있음
  }
  else if (digitalRead(sw) == HIGH) isPushed = false;  //버튼이 떼어졌다면 값 0

  return toggleAlert;
}

void handleDevice(bool control){      //매개변수로 위의 함수를 이용할 것임
    if(!control) buzzerLedPattern(SBAS_status);  //버튼이 눌리지 않았다면 장치 동작
    else { ledColor(0, 0, 0); noTone(buzzer);    //버튼이 눌리면 알람 일괄 해제를 의미(데이터는 서버로 전송됨)
      //Serial.println("*****All Alarms Are Disabled. Check Your Device.***** "); }  //디버깅 코드
}

/*void printWifiStatus() {      //와이파이 상태창 출력 함수이나 디버깅 용임
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());    //와이파이 ID와

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);              //해당 IP를 제공

  delay(5000);            //안전한 연결을 위해 5초간 딜레이
}*/

void connectWiFi() {          //와이파이 연결 함수
  while (status != WL_CONNECTED) {      //아직 연결되지 않았다면
    //Serial.print("Attempting to connect to SSID: ");  
    //Serial.println(ssid);        //디버깅, 연결 중이라 표시
    status = WiFi.begin(ssid, pass);    //연결 되고 나면 연결된 와이파이로 상태를 변경

    delay(5000);      //위 함수와 같은 이유
  }
}

void setupTime() {      //서버 시간을 가져오는 함수
  timeClient.begin();    //객체 실행
  //timeClient.setTimeOffset(3600 * 9);    //타임오프셋이나 위에서 GMT9로 조정했으므로 불필요
  timeClient.forceUpdate();  //강제 업데이트
}

void sendRequest(const char* endpoint, DynamicJsonDocument& doc) {  //엔드 포인트 값과 JSON 값을 매개변수로 이용
    if (client.connect(serverIP, _PORT)) {      //클라이언트가 해당 IP와 포트에 연결되어있을 때,
       // Serial.println("Connected to Server");   //디버깅용

        String jsonString;                //전송될 데이터가 담길 변수 생성
        serializeJson(doc, jsonString);    //JSON 파싱

        client.print("POST /");            //요청문 시작, POST 방식
        client.print(endpoint);            //엔드포인트는 후술할 함수에서 등장
        client.print(" HTTP/1.1\r\n");
        client.print("Host: ");
        client.println(serverIP);         //서버의 주소
        client.print(":");
        client.print(_PORT);              //포트
        client.println("Connection: close");
        client.println("Content-Type: application/json");
        client.print("Content-Length: ");
        client.println(jsonString.length() + 1);   //데이터 값 + 1 만큼의 길이
        client.println();
        client.println(jsonString);        //데이터 전송, 요청문 종료
       // Serial.println(jsonString);
       // Serial.println("HTTP Request Sent");  //디버깅용

        delay(500);        //0.5초 간격

      /*  if (client.available()) {
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
        }*/        //데이터 처리가 이루어졌는지 확인하는 코드(디버깅용이므로 주석처리)

       // Serial.println();
       // Serial.println("Closing connection");    //연결 종료됐다 알림
        client.stop();  //접속 해제
    } else {      //클라이언트 접속 실패시 밑 문구 출력
      //  Serial.println("Connection to server failed");
    }

    delay(100);      //0.1초 간격
}

void sendData_fire(){        //화재 경보기의 값을 서버로 전송하는 함수
    DynamicJsonDocument doc(1024);    //데이터 크기 = 2^10
    doc["flame"] = String(flameSensor());  
    doc["temperature"] = String(tempSensor());
    doc["sensorTime"] = getTime();              //각 센서의 값을 서버 형식에 맞게 저장
    sendRequest("fire/save", doc);          //엔드포인트: fire/save
}

void sendData_EarthQuake() {        //지진 경보기
    DynamicJsonDocument doc(1024);
    doc["vibrationSensor"] = String(gyroSensor());
    doc["sensorTime"] = getTime();
    sendRequest("quake/save", doc);
}                                                        //"192.x.x.x:port/fire/save/?flame=value" 형태       client.stop();  //접속 해제
    } else {      //클라이언트 접속 실패시 밑 문구 출력
      //  Serial.println("Connection to server failed");
    }

    delay(100);      //0.1초 간격
}

void sendData_fire(){        //화재 경보기의 값을 서버로 전송하는 함수
    DynamicJsonDocument doc(1024);    //데이터 크기 = 2^10
    doc["flame"] = String(flameSensor());  
    doc["temperature"] = String(tempSensor());
    doc["sensorTime"] = getTime();              //각 센서의 값을 서버 형식에 맞게 저장
    sendRequest("fire/save", doc);          //엔드포인트: fire/save
}

void sendData_EarthQuake() {        //지진 경보기
    DynamicJsonDocument doc(1024);
    doc["vibrationSensor"] = String(gyroSensor());
    doc["sensorTime"] = getTime();
    sendRequest("quake/save", doc);
}                                                        //"x.x.x.x:port/fire/save?flame=value..." 형태가 서버로 전달

void sendData_gas() {                //가스 경보기
    DynamicJsonDocument doc(1024);
    doc["flammable"] = String(gasSensorFlammable());
    doc["co"] = String(gasSensorCO());
    doc["sensorTime"] = getTime();
    sendRequest("gas/save", doc);
}

const char* getToken() {                    //서버에게 token값을 요청하는 함수
    if (client.connect(serverIP, _PORT)) {  
        //Serial.println("Connected to Server");    //디버깅용

        client.print("GET /");          //GET 방식의 요청문
        client.print("token/list");    //엔드포인트: token/list
        client.print(" HTTP/1.1\r\n");
        client.print("Host: ");
        client.println(serverIP);
        client.print(":");
        client.print(_PORT); 
        client.println("Connection: close");
        client.println();      //요청문 종료

        delay(500);

        while(client.connected() || client.available()) {    //클라이언트가 연결되어있거나 유호하다면
            if(client.available()) {      //이중 검사 코드이나 위의 조건을 삭제해도 무방
                String data = client.readStringUntil('\r');    //문자열로 공백이 있을 때까지 읽어들인 후 저장
                DynamicJsonDocument doc(1024); 
                deserializeJson(doc, data);    //디코딩 함수

                token = doc["token"];    //해당 변수에 서버로부터 읽어온 값을 저장
            }
        }
        client.stop();        //연결 해제

        delay(1000);        //1초 간격

       // Serial.println("Communication Ended."); //디버깅용
    }
    return token; 
}

void sendFCM(const char* token) {        //token 값 매개변수 이용                     ┌  Message  ┐
    DynamicJsonDocument doc(1024);                                             // token        data
    doc["message"]["token"] = String(token);                                   //                |
    doc["message"]["data"]["warningId"] = String(SBAS_status);                 //              warningId
    sendRequest("fcm/sendData", doc);                                          //위와 같은 class 구조
}
