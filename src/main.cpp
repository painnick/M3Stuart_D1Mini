#include <Arduino.h>

#include "esp_log.h"

#include <Ps3Controller.h>
#include <ESP32Servo.h>

#include "soc/soc.h"             // disable brownout problems
#include "soc/rtc_cntl_reg.h"    // disable brownout problems

#ifdef USE_SOUND
#include "DFMiniMp3.h"
#endif

#define MAIN_TAG "Main"

// These are all GPIO pins on the ESP32
// Recommended pins include 2,4,12-19,21-23,25-27,32-33
// for the ESP32-S2 the GPIO pins are 1-21,26,33-42

// 13 outputs PWM signal at boot
// 14 outputs PWM signal at boot


#ifdef USE_SOUND
#define PIN_RX 16 // FIXED
#define PIN_TX 17 // FIXED
#endif

#define PIN_TURRET 15

#define PIN_TRACK_A1 21
#define PIN_TRACK_A2 22
#define PIN_TRACK_B1 17
#define PIN_TRACK_B2 16

#define PIN_MISSILE 4

#define CHANNEL_A1 12
#define CHANNEL_A2 13
#define CHANNEL_B1 14
#define CHANNEL_B2 15

#ifdef USE_SOUND
#define MAX_VOLUME 18
#endif

#define STICK_THRESHOLD 20

#define TRACK_MOTOR_RESOLUTION 8

Servo servoTurret;

#ifdef USE_SOUND
class Mp3Notify;
typedef DFMiniMp3<HardwareSerial, Mp3Notify> DfMp3;
HardwareSerial mySerial(2); // 16, 17
DfMp3 dfmp3(mySerial);
int volume = MAX_VOLUME; // 0~30
#endif

int center = 90;

int bodyAngle = center;

int angleStep = 5;
int servoDelay = 15;

bool circlePress = false;
bool triaglePress = false;
bool squarePress = false;
bool crossPress = false;

void init() {
  ESP_LOGI(MAIN_TAG, "Init.(Internal)");

  bodyAngle = center;

  servoTurret.attach(PIN_TURRET, 500, 2400);

  servoTurret.write(center);

  pinMode(PIN_MISSILE, OUTPUT);

#ifdef USE_SOUND
  dfmp3.playMp3FolderTrack(3);
#endif
}

void reset() {
  ESP_LOGI(MAIN_TAG, "Reset");
  bodyAngle = center;

  servoTurret.write(center);

#ifdef USE_SOUND
  dfmp3.playMp3FolderTrack(3);
#endif
}

int battery = 0;
void notify()
{
  // RESET
  if (Ps3.event.button_down.start) {
    ESP_LOGI(MAIN_TAG, "Start(Reset)");
    reset();
  }

  // Missile
  if ((Ps3.event.button_down.square) || (Ps3.event.button_down.triangle)) {
    ESP_LOGI(MAIN_TAG, "Square or Triangle(Missile)");
#ifdef USE_SOUND    
    dfmp3.playMp3FolderTrack(2);
#endif

    digitalWrite(PIN_MISSILE, HIGH);

    // Back
    ledcWrite(CHANNEL_B1, 0);
    ledcWrite(CHANNEL_B2, pow(2, TRACK_MOTOR_RESOLUTION) - 1);

    ledcWrite(CHANNEL_A1, 0);
    ledcWrite(CHANNEL_A2, pow(2, TRACK_MOTOR_RESOLUTION) - 1);

    delay(30); // N30

    ledcWrite(CHANNEL_B1, 0);
    ledcWrite(CHANNEL_B2, 0);

    ledcWrite(CHANNEL_A1, 0);
    ledcWrite(CHANNEL_A2, 0);

    delay(300);
    digitalWrite(PIN_MISSILE, LOW);
  }


  // Turret
  if (Ps3.event.button_down.left) {
    ESP_LOGD(MAIN_TAG, "Left(Turret)");
    bodyAngle = min(bodyAngle + 5, 180);
    servoTurret.write(bodyAngle);
  }
  if (Ps3.event.button_down.right) {
    ESP_LOGD(MAIN_TAG, "Right(Turret)");
    bodyAngle = max(bodyAngle - 5, 0);
    servoTurret.write(bodyAngle);
  }

#ifdef USE_SOUND
  // Volume
  if (Ps3.event.analog_changed.button.up) {
    ESP_LOGD(MAIN_TAG, "Up(Volume)");
    volume = min(volume + 2, MAX_VOLUME);
    dfmp3.setVolume(volume);
  }
  if (Ps3.event.analog_changed.button.down) {
    ESP_LOGD(MAIN_TAG, "Down(Volume)");
    volume = max(volume - 2, 0);
    dfmp3.setVolume(volume);
  }
#endif

  // Track
  int absLy = abs(Ps3.event.analog_changed.stick.ly);
  if (absLy < STICK_THRESHOLD) {
    ledcWrite(CHANNEL_B1, 0);
    ledcWrite(CHANNEL_B2, 0);
  } else {
    if (Ps3.event.analog_changed.stick.ly < -STICK_THRESHOLD) {
      ledcWrite(CHANNEL_B1, 127);
      ledcWrite(CHANNEL_B2, 0);
    }
    else if (Ps3.event.analog_changed.stick.ly > STICK_THRESHOLD) {
      ledcWrite(CHANNEL_B1, 0);
      ledcWrite(CHANNEL_B2, 127);
    }
  }

  int absRy = abs(Ps3.event.analog_changed.stick.ry);
  if (absRy < STICK_THRESHOLD) {
    ledcWrite(CHANNEL_A1, 0);
    ledcWrite(CHANNEL_A2, 0);    
  } else {
    if (Ps3.event.analog_changed.stick.ry < -STICK_THRESHOLD) {
      ledcWrite(CHANNEL_A1, 127);
      ledcWrite(CHANNEL_A2, 0);
    }
    else if (Ps3.event.analog_changed.stick.ry > STICK_THRESHOLD) {
      ledcWrite(CHANNEL_A1, 0);
      ledcWrite(CHANNEL_A2, 127);
    }
  }
}

void onConnect();
void onDisconnect();

void initPs3() {
  ESP_LOGI(MAIN_TAG, "Init. PS3 Wireless Controller");
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisconnect);
  Ps3.begin("44:44:44:44:44:44");
}

void onConnect() {
  ESP_LOGI(MAIN_TAG, "Connected");

  String address = Ps3.getAddress();

  ESP_LOGI(MAIN_TAG, "The ESP32's Bluetooth MAC address is: %s", address.c_str());

  reset();

  servoTurret.attach(PIN_TURRET, 500, 2400);

  ESP_LOGD(MAIN_TAG, "Setup CHANNEL_A1 %d",  CHANNEL_A1);
  ledcSetup(CHANNEL_A1, 1000, 7); // 0~127
  ESP_LOGD(MAIN_TAG, "Setup CHANNEL_A2 %d", CHANNEL_A2);
  ledcSetup(CHANNEL_A2, 1000, 7); // 0~127
  ESP_LOGD(MAIN_TAG, "Setup CHANNEL_B1 %d", CHANNEL_B1);
  ledcSetup(CHANNEL_B1, 1000, 7); // 0~127
  ESP_LOGD(MAIN_TAG, "Setup CHANNEL_B2 %d", CHANNEL_B2);
  ledcSetup(CHANNEL_B2, 1000, 7); // 0~127

  ESP_LOGD(MAIN_TAG, "Attach PIN_TRACK_A1 %d", PIN_TRACK_A1);
  ledcAttachPin(PIN_TRACK_A1, CHANNEL_A1);
  ESP_LOGD(MAIN_TAG, "Attach PIN_TRACK_A2 %d", PIN_TRACK_A2);
  ledcAttachPin(PIN_TRACK_A2, CHANNEL_A2);
  ESP_LOGD(MAIN_TAG, "Attach PIN_TRACK_B1 %d", PIN_TRACK_B1);
  ledcAttachPin(PIN_TRACK_B1, CHANNEL_B1);
  ESP_LOGD(MAIN_TAG, "Attach PIN_TRACK_B2 %d", PIN_TRACK_B2);
  ledcAttachPin(PIN_TRACK_B2, CHANNEL_B2);

#ifdef USE_SOUND
  dfmp3.playMp3FolderTrack(3);
#endif
}

void onDisconnect() {
  ESP_LOGI(MAIN_TAG, "Disconnected");
  Ps3.end();
  initPs3();
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);

  init();
  initPs3();

#ifdef USE_SOUND
  dfmp3.begin(9600, 1000);
  dfmp3.reset();

  while(!dfmp3.isOnline()) {
    delay(10);
  }

  dfmp3.setVolume(volume);
#endif
}

void loop() {
  if(!Ps3.isConnected())
    return;

#ifdef USE_SOUND
  dfmp3.loop();
#endif
  delay(1);
}

#ifdef USE_SOUND
//----------------------------------------------------------------------------------
class Mp3Notify
{
public:
  static void PrintlnSourceAction(DfMp3_PlaySources source, const char *action)
  {
    if (source & DfMp3_PlaySources_Sd) {
      ESP_LOGD(MAIN_TAG, "SD Card, %s", action);
    }
    if (source & DfMp3_PlaySources_Usb) {
      ESP_LOGD(MAIN_TAG, "USB Disk, %s", action);
    }
    if (source & DfMp3_PlaySources_Flash) {
      ESP_LOGD(MAIN_TAG, "Flash, %s", action);
    }
  }
  static void OnError(DfMp3 &mp3, uint16_t errorCode)
  {
    // see DfMp3_Error for code meaning
    Serial.println();
    Serial.print("Com Error ");
    switch (errorCode)
    {
    case DfMp3_Error_Busy:
      ESP_LOGE(MAIN_TAG, "Com Error - Busy");
      break;
    case DfMp3_Error_Sleeping:
      ESP_LOGE(MAIN_TAG, "Com Error - Sleeping");
      break;
    case DfMp3_Error_SerialWrongStack:
      ESP_LOGE(MAIN_TAG, "Com Error - Serial Wrong Stack");
      break;

    case DfMp3_Error_RxTimeout:
      ESP_LOGE(MAIN_TAG, "Com Error - Rx Timeout!!!");
      break;
    case DfMp3_Error_PacketSize:
      ESP_LOGE(MAIN_TAG, "Com Error - Wrong Packet Size!!!");
      break;
    case DfMp3_Error_PacketHeader:
      ESP_LOGE(MAIN_TAG, "Com Error - Wrong Packet Header!!!");
      break;
    case DfMp3_Error_PacketChecksum:
      ESP_LOGE(MAIN_TAG, "Com Error - Wrong Packet Checksum!!!");
      break;

    default:
      ESP_LOGE(MAIN_TAG, "Com Error - %d", errorCode);
      break;
    }
  }
  static void OnPlayFinished(DfMp3 &mp3, DfMp3_PlaySources source, uint16_t track)
  {
    ESP_LOGD(MAIN_TAG, "Play finished for #%d", track);
  }
  static void OnPlaySourceOnline(DfMp3 &mp3, DfMp3_PlaySources source)
  {
    PrintlnSourceAction(source, "online");
  }
  static void OnPlaySourceInserted(DfMp3 &mp3, DfMp3_PlaySources source)
  {
    PrintlnSourceAction(source, "inserted");
  }
  static void OnPlaySourceRemoved(DfMp3 &mp3, DfMp3_PlaySources source)
  {
    PrintlnSourceAction(source, "removed");
  }
};
#endif
