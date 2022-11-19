#include <Arduino.h>

#include "esp_log.h"

#include <Ps3Controller.h>
#include <ESP32Servo.h>

#include "soc/soc.h"             // disable brownout problems
#include "soc/rtc_cntl_reg.h"    // disable brownout problems

#define USE_SOUND

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
#define PIN_RX 16 // RX2
#define PIN_TX 17 // TX2
#endif

#define PIN_TURRET_SERVO 22 // PWM(Servo)

#define PIN_MISSILE_LED 23 // Digital

#define PIN_TRACK_L1_MOTOR 33 // PWM(Analog)
#define PIN_TRACK_L2_MOTOR 32 // PWM(Analog)
#define PIN_TRACK_R1_MOTOR 25 // PWM(Analog)
#define PIN_TRACK_R2_MOTOR 26 // PWM(Analog)

#define CHANNEL_L1 14
#define CHANNEL_L2 15
#define CHANNEL_R1 12
#define CHANNEL_R2 13

#ifdef USE_SOUND
#define MAX_VOLUME 18
#endif

#define STICK_THRESHOLD 20

#define TRACK_MOTOR_RESOLUTION 7

Servo servoTurret;

#ifdef USE_SOUND
class Mp3Notify;
typedef DFMiniMp3<HardwareSerial, Mp3Notify> DfMp3;
HardwareSerial mySerial(2); // 16, 17
DfMp3 dfmp3(mySerial);
int volume = MAX_VOLUME; // 0~30
#endif

const int center = 90;

int bodyAngle = center;

int angleStep = 5;
int servoDelay = 15;

int trackSpeed = pow(2, TRACK_MOTOR_RESOLUTION) - 1; // Default Max
bool isTurbo = false;

bool circlePress = false;
bool triaglePress = false;
bool squarePress = false;
bool crossPress = false;

uint setTrackSpeed(bool isTurbo) {
  #ifdef USE_TURNO
  if (isTurbo) {
    trackSpeed = 255;
  } else {
    trackSpeed = 128;
  }
  ESP_LOGD(MAIN_TAG, "trackSpeed %d", trackSpeed);
  #else
  trackSpeed = 255;
  #endif

  return trackSpeed;
}

void init() {
  ESP_LOGI(MAIN_TAG, "Init.(Internal)");

  bodyAngle = center;

  servoTurret.attach(PIN_TURRET_SERVO, 500, 2400);

  servoTurret.write(bodyAngle);

  pinMode(PIN_MISSILE_LED, OUTPUT);

  setTrackSpeed(isTurbo);

// #ifdef USE_SOUND
//   dfmp3.playMp3FolderTrack(3);
//   delay(1000);
//   dfmp3.playMp3FolderTrack(3);
// #endif
}

void reset() {
  ESP_LOGI(MAIN_TAG, "Reset");
  bodyAngle = center;

  servoTurret.write(bodyAngle);

  isTurbo = false;
  setTrackSpeed(isTurbo);

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

  // TURBO!
  if (Ps3.event.button_down.select) {
    ESP_LOGI(MAIN_TAG, "Select(Turbo)");
    isTurbo = !isTurbo;
    setTrackSpeed(isTurbo);
  }

  // Missile
  if ((Ps3.event.button_down.square) || (Ps3.event.button_down.triangle)) {
    ESP_LOGI(MAIN_TAG, "Square or Triangle(Missile)");
#ifdef USE_SOUND    
    dfmp3.playMp3FolderTrack(2);
#endif

    digitalWrite(PIN_MISSILE_LED, HIGH);

    // Back
    ledcWrite(CHANNEL_L1, 0);
    ledcWrite(CHANNEL_L2, pow(2, TRACK_MOTOR_RESOLUTION) - 1);

    ledcWrite(CHANNEL_R1, 0);
    ledcWrite(CHANNEL_R2, pow(2, TRACK_MOTOR_RESOLUTION) - 1);

    delay(30); // N30

    ledcWrite(CHANNEL_L1, 0);
    ledcWrite(CHANNEL_L2, 0);

    ledcWrite(CHANNEL_R1, 0);
    ledcWrite(CHANNEL_R2, 0);

    delay(300);
    digitalWrite(PIN_MISSILE_LED, LOW);
  }


  // Turret
  if (Ps3.event.button_down.left) {
    ESP_LOGD(MAIN_TAG, "Left(Turret)");
    bodyAngle = max(bodyAngle - 5, 10);
    servoTurret.write(bodyAngle);
  }
  if (Ps3.event.button_down.right) {
    ESP_LOGD(MAIN_TAG, "Right(Turret)");
    bodyAngle = min(bodyAngle + 5, 170);
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
    ledcWrite(CHANNEL_L1, 0);
    ledcWrite(CHANNEL_L2, 0);
  } else {
    if (Ps3.event.analog_changed.stick.ly < -STICK_THRESHOLD) {
      ledcWrite(CHANNEL_L1, trackSpeed);
      ledcWrite(CHANNEL_L2, 0);
    }
    else if (Ps3.event.analog_changed.stick.ly > STICK_THRESHOLD) {
      ledcWrite(CHANNEL_L1, 0);
      ledcWrite(CHANNEL_L2, trackSpeed);
    }
  }

  int absRy = abs(Ps3.event.analog_changed.stick.ry);
  if (absRy < STICK_THRESHOLD) {
    ledcWrite(CHANNEL_R1, 0);
    ledcWrite(CHANNEL_R2, 0);    
  } else {
    if (Ps3.event.analog_changed.stick.ry < -STICK_THRESHOLD) {
      ledcWrite(CHANNEL_R1, trackSpeed);
      ledcWrite(CHANNEL_R2, 0);
    }
    else if (Ps3.event.analog_changed.stick.ry > STICK_THRESHOLD) {
      ledcWrite(CHANNEL_R1, 0);
      ledcWrite(CHANNEL_R2, trackSpeed);
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

  servoTurret.attach(PIN_TURRET_SERVO, 500, 2400);

  ESP_LOGD(MAIN_TAG, "Setup CHANNEL_R1 %d",  CHANNEL_R1);
  ledcSetup(CHANNEL_R1, 1000, TRACK_MOTOR_RESOLUTION);
  ESP_LOGD(MAIN_TAG, "Setup CHANNEL_R2 %d", CHANNEL_R2);
  ledcSetup(CHANNEL_R2, 1000, TRACK_MOTOR_RESOLUTION);
  ESP_LOGD(MAIN_TAG, "Setup CHANNEL_L1 %d", CHANNEL_L1);
  ledcSetup(CHANNEL_L1, 1000, TRACK_MOTOR_RESOLUTION);
  ESP_LOGD(MAIN_TAG, "Setup CHANNEL_L2 %d", CHANNEL_L2);
  ledcSetup(CHANNEL_L2, 1000, TRACK_MOTOR_RESOLUTION);

  ESP_LOGD(MAIN_TAG, "Attach PIN_TRACK_R1_MOTOR %d", PIN_TRACK_R1_MOTOR);
  ledcAttachPin(PIN_TRACK_R1_MOTOR, CHANNEL_R1);
  ESP_LOGD(MAIN_TAG, "Attach PIN_TRACK_R2_MOTOR %d", PIN_TRACK_R2_MOTOR);
  ledcAttachPin(PIN_TRACK_R2_MOTOR, CHANNEL_R2);
  ESP_LOGD(MAIN_TAG, "Attach PIN_TRACK_L1_MOTOR %d", PIN_TRACK_L1_MOTOR);
  ledcAttachPin(PIN_TRACK_L1_MOTOR, CHANNEL_L1);
  ESP_LOGD(MAIN_TAG, "Attach PIN_TRACK_L2_MOTOR %d", PIN_TRACK_L2_MOTOR);
  ledcAttachPin(PIN_TRACK_L2_MOTOR, CHANNEL_L2);

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
      ESP_LOGW(MAIN_TAG, "Com Error - Busy");
      break;
    case DfMp3_Error_Sleeping:
      ESP_LOGW(MAIN_TAG, "Com Error - Sleeping");
      break;
    case DfMp3_Error_SerialWrongStack:
      ESP_LOGW(MAIN_TAG, "Com Error - Serial Wrong Stack");
      break;

    case DfMp3_Error_RxTimeout:
      ESP_LOGW(MAIN_TAG, "Com Error - Rx Timeout!!!");
      break;
    case DfMp3_Error_PacketSize:
      ESP_LOGW(MAIN_TAG, "Com Error - Wrong Packet Size!!!");
      break;
    case DfMp3_Error_PacketHeader:
      ESP_LOGW(MAIN_TAG, "Com Error - Wrong Packet Header!!!");
      break;
    case DfMp3_Error_PacketChecksum:
      ESP_LOGW(MAIN_TAG, "Com Error - Wrong Packet Checksum!!!");
      break;

    default:
      ESP_LOGW(MAIN_TAG, "Com Error - %d", errorCode);
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
