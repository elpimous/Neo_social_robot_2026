
#ifndef ArduBot_h
#define ArduBot_h

#include <inttypes.h>
#include <Arduino.h>
#include <avr/interrupt.h>
#include <math.h>
#include <Wire.h>
#include <VL53L1X.h>
#include "LIS35DE.h"
#include "L3G4200D.h"
#include "comands.h"
#include "motores.h"
#include "LCD03.h"
#include "LCD_Graphics.h"

#define MOTOR_1_CONTROL_1_PIN PE2
#define MOTOR_1_CONTROL_2_PIN PG3
#define MOTOR_2_CONTROL_2_PIN PH2
#define MOTOR_2_CONTROL_1_PIN PG4
#define MOTOR_1_COTROL_1_PORT PORTE
#define MOTOR_1_COTROL_2_PORT PORTG
#define MOTOR_2_COTROL_2_PORT PORTH
#define MOTOR_2_COTROL_1_PORT PORTG
#define MOTOR_1_COTROL_1_REGISTER DDRE
#define MOTOR_1_COTROL_2_REGISTER DDRG
#define MOTOR_2_COTROL_2_REGISTER DDRH
#define MOTOR_2_COTROL_1_REGISTER DDRG
#define MOTOR_1_PWM_ARDUINO_PIN 8
#define MOTOR_2_PWM_ARDUINO_PIN 7
#define MOTOR_1_HALL_A_PIN PJ2
#define MOTOR_1_HALL_B_PIN PJ3
#define MOTOR_2_HALL_A_PIN PJ4
#define MOTOR_2_HALL_B_PIN PJ5
#define MIN_FLOOR_DISTANCE_CM 11
#define MAX_FLOOR_DISTANCE_CM 36
#define MIN_SENSOR_DISTANCE_FRONT_CM 20
#define MIN_SENSOR_DISTANCE_BACK_CM 10

// Capteur distance sol (Sharp IR analogique) — broche analogique A8
// Sur Arduino Mega : A8 = broche physique 62
// ⚠️  Ne pas confondre avec MOTOR_1_PWM_ARDUINO_PIN qui vaut aussi 8
//     mais désigne la broche DIGITALE 8 (PWM Timer4).
//     analogRead(8) == analogRead(A8) sur AVR, mais A8 est explicite.
#define FLOOR_DISTANCE_SENSOR_ARDUINO_PIN A8

#define ENABLE_RIGHT_LASER_PIN PB3
#define AMPLI_ADDR 0x4B
#define LCD_I2C_ADDRESS 0xC6
#define IR1_BIT PE6
#define IR2_BIT PE7
#define IR3_BIT PD2
#define IR_RECEIVE_PIN 19
#define IR_ADRESSE_BASE 0x42B1A

// vince : 500 échantillons × 5ms/cycle = ~2.5s sans blocage
#define CALIB_BUFFERSIZE 500

namespace arduBot
{
  const uint8_t boardId = 0;
  const uint8_t libraryVersion = 3;
  const uint8_t srfCmdByte   = 0x00;
  const uint8_t srfLightByte = 0x01;
  const uint8_t srfRangeByte = 0x02;
  const uint8_t srfGainByte  = 0x01;

  enum CalibrationState {
    CALIB_IDLE    = 0,
    CALIB_RUNNING = 1,
    CALIB_DONE    = 2
  };

  class ArduBot
  {
    private:
      byte alert_stop;
      boolean isSrfUpdateContinuous;
      static long spinLoopPeriodMs;
      VL53L1X LaserL;
      VL53L1X LaserR;
      LIS35DE accelerometer;
      L3G4200D gyro;
      double xCoordinate;
      double yCoordinate;
      double thetaCoordinate;
      void estimatePosition(float loopPeriodSeconds);
      void scanI2C();
      static bool wheelStopAlertFlag;
      static bool robotFallFlag;
      static bool robotCrashFlag;

      // vince : variables machine d'état calibration
      CalibrationState calibState_       = CALIB_IDLE;
      bool             calibResult_      = false;
      int              calibSampleCount_ = 0;
      float            calibMean_[6]     = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
      long             calibRawGzSum_    = 0;

    public:
      static bool robotStallFlag;
      static CMotors leftMotor;
      static CMotors rightMotor;
      static CBaseMovement par_motores;
      LCD03 lcd;
      boolean lcdState;
      boolean energyState;
      boolean gyroState;
      boolean accelerometerState;
      boolean ampliState;
      bool floorSensorPresent;

      byte NUM_SRFs;
      int SRFs_ADRESS[16];
      byte NUM_SRFs_FRONT;
      int SRFs_FRONT[4];
      bool SRFs_FRONT_CRASH_FLAGS[16];
      byte NUM_SRFs_BACK;
      int SRFs_BACK[4];
      float SRFs_FRONT_ALERT_DIST[4];
      float SRFs_BACK_ALERT_DIST[4];
      float FLOOR_MIN_ALERT_DIST;
      int LASERs_ADRESS[16];
      byte NUM_LASERs_FRONT;
      int LASERs_FRONT[4];
      byte NUM_LASERs_BACK;
      int LASERs_BACK[4];
      byte MAX_LASERS;
      byte NUM_LASERS;

      inline void initLasers() {
        Serial.println("Initializing VL53L1X Lasers...");
        PORTB = PORTB | B00000100;
        delay(100);
        LaserL.init();
        LaserL.setAddress(0x70);
        LaserL.setDistanceMode(VL53L1X::Short);
        LaserL.setMeasurementTimingBudget(20000);
        LaserL.startContinuous(20);
        Serial.println("Laser gauche (0x70) initialisé");
        PORTB = PORTB | B00001100;
        delay(100);
        LaserR.init();
        LaserR.setAddress(0x71);
        LaserR.setDistanceMode(VL53L1X::Short);
        LaserR.setMeasurementTimingBudget(20000);
        LaserR.startContinuous(20);
        Serial.println("Laser droit (0x71) initialisé");
        delay(100);
      }

      static bool leftMotorHallA;
      static bool leftMotorHallB;
      static bool rightMotorHallA;
      static bool rightMotorHallB;
      static byte ir1data;
      static byte ir2data;
      static byte ir3data;
      static uint16_t LeftDist;
      static uint16_t RightDist;
      static uint16_t BackLeftDist;
      static uint16_t BackRightDist;
      static uint16_t MiddleDist;

      float gyroX, gyroY, gyroZ;
      float gyroX0, gyroY0, gyroZ0;
      float accelerometerX, accelerometerY, accelerometerZ;
      float accelerometerX0, accelerometerY0, accelerometerZ0;
      const float ACC_SENSITIVITY = 0.0209f;
      float gyroSensitivity = 0.070f;
      int16_t roll;
      int16_t pitch;

      ArduBot(double wheelRadious = 0.102, double wheelDistance = 0.2736, int encoderResolution = 1440); //1560
      void begin(double spinLoopPeriodS = 0.005, double kp = 0.0, double ki = 0.0, double kd = 0.0);

      // -----------------------------------------------------------------------
      // getSpacePosition() — lecture de la position odométrique
      //
      // Coordonnées internes (xCoordinate, yCoordinate, thetaCoordinate) :
      //   → stockées en double pour la précision maximale des calculs
      //      odométriques (intégration cos/sin sur des milliers de cycles)
      //
      // Retour en float :
      //   → la sérialisation série (GET_ODOMETRY) envoie 4 octets par valeur
      //   → double = 8 octets sur ARM/x86, mais sur AVR double = float = 4 octets
      //   → utiliser float ici rend le code portable et explicitement correct
      //   → précision float (~7 chiffres) largement suffisante pour ROS :
      //      position en mètres avec 0.1mm de résolution sur ±100m
      //
      // ⚠️  CORRECTION BUG #11 (Mars 2026) :
      //     L'ancienne version déclarait double xCoordinate dans processCommands()
      //     puis sérialisait 4 octets → silencieusement correct sur AVR (double=float)
      //     mais incorrect sur toute autre architecture (ex. portage Jetson).
      //     Le cast (float) ici rend la troncature explicite et volontaire.
      // -----------------------------------------------------------------------
      inline void getSpacePosition(float& x, float& y, float& angle) {
        x = (float)xCoordinate; y = (float)yCoordinate; angle = (float)thetaCoordinate;
      };

      void filterGyro(float &gx, float &gy, float &gz);

      // =========================================================
      // vince : calibrateRequest() — protocole non-bloquant
      // =========================================================
      inline uint8_t calibrateRequest() {
        if (calibState_ == CALIB_RUNNING) return 0xFF;
        if (calibState_ == CALIB_DONE) {
          calibState_ = CALIB_IDLE;
          return calibResult_ ? 1 : 0;
        }
        // IDLE → démarrer
        calibState_        = CALIB_RUNNING;
        calibSampleCount_  = 0;
        calibRawGzSum_     = 0;
        for (int i = 0; i < 6; i++) calibMean_[i] = 0.0f;
        Serial.println("📊 [IMU CALIBRATION] Démarrage... Ne pas bouger !");
        return 0xFF;
      }

      // =========================================================
      // vince : calibrationStep() — prend les valeurs déjà lues en paramètre
      // CORRECTION goulot 2 : évite la double lecture I2C par cycle
      // Anciennement : calibrationStep() relisait gyro+accel via I2C (~0.2ms)
      // alors que spinOnce() venait de les lire → 4 lectures I2C au lieu de 2
      // Désormais : spinOnce() lit une fois et passe les valeurs brutes
      // =========================================================
      inline void calibrationStep(int raw_gx, int raw_gy, int raw_gz,
                                   int8_t raw_ax, int8_t raw_ay, int8_t raw_az) {
        if (calibState_ != CALIB_RUNNING) return;

        // vince : même transformation capteur → robot que l'ancienne calibrate()
        calibMean_[0] += -raw_ax * ACC_SENSITIVITY;
        calibMean_[1] += -raw_ay * ACC_SENSITIVITY;
        calibMean_[2] +=  raw_az * ACC_SENSITIVITY;
        calibMean_[3] +=  (float)raw_gy;
        calibMean_[4] += -(float)raw_gx;
        calibMean_[5] +=  (float)raw_gz;
        calibRawGzSum_ += raw_gz;
        calibSampleCount_++;

        if (calibSampleCount_ >= CALIB_BUFFERSIZE) {
          for (int i = 0; i < 6; i++) calibMean_[i] /= CALIB_BUFFERSIZE;
          accelerometerX0 = -calibMean_[0];
          accelerometerY0 = -calibMean_[1];
          accelerometerZ0 = -calibMean_[2];
          gyroX0          = -calibMean_[3];
          gyroY0          = -calibMean_[4];
          // vince : GZ calculé sur raw brut pour précision maximale
          gyroZ0          = (float)(calibRawGzSum_ / CALIB_BUFFERSIZE);

          Serial.println("✅ [IMU CALIBRATION] Terminée !");
          Serial.print("Offsets ACC  (X0,Y0,Z0) : ");
          Serial.print(accelerometerX0); Serial.print(", ");
          Serial.print(accelerometerY0); Serial.print(", ");
          Serial.println(accelerometerZ0);
          Serial.print("Offsets GYRO (X0,Y0,Z0) : ");
          Serial.print(gyroX0); Serial.print(", ");
          Serial.print(gyroY0); Serial.print(", ");
          Serial.println(gyroZ0);
          calibResult_ = true;
          calibState_  = CALIB_DONE;
        }
      }

      inline CalibrationState getCalibrationState() const { return calibState_; }

      inline void setPosition(float x, float y, float theta) {
        xCoordinate = x; yCoordinate = y; thetaCoordinate = theta;
      }

      void setSpeeds(double linealSpeed, double angularSpeed);
      double currentLinearSpeed  = 0.0;
      double currentAngularSpeed = 0.0;
      void updatePosition(float loopPeriodSeconds);

      inline float getFloorDistance() {
        int analogValue = analogRead(FLOOR_DISTANCE_SENSOR_ARDUINO_PIN);
        if (analogValue < 5) { ArduBot::wheelStopAlertFlag = true; return -1.0; }
        float distance = 12343.85 * pow((float)analogValue, -1.15);
        if (distance < MIN_FLOOR_DISTANCE_CM || distance > MAX_FLOOR_DISTANCE_CM)
          ArduBot::wheelStopAlertFlag = true;
        else
          ArduBot::wheelStopAlertFlag = false;
        return distance;
      }

      inline unsigned int adcRead(byte pin) { return analogRead(pin); }

      inline bool getBatteryLevel(byte *value, byte *stat) {
        Wire.requestFrom(0x14, 2);
        if (Wire.available() < 2) {
          Serial.println("❌ Erreur : Aucune réponse de la carte d'alimentation !");
          energyState = false;
          return false;
        }
        *stat = Wire.read();
        *value = Wire.read();
        energyState = true;
        return true;
      }

      inline bool initAmpli(uint8_t addr) {
        Wire.beginTransmission(addr);
        Wire.write(0);
        return (Wire.endTransmission() == 0);
      }

      inline void muteAmpli() {
        Wire.beginTransmission(AMPLI_ADDR);
        Wire.write(0);
        ampliState = (Wire.endTransmission() == 0);
      }

      inline void setAmpliVolume(uint8_t volume) {
        if (volume > 63) volume = 63;
        Wire.beginTransmission(AMPLI_ADDR);
        Wire.write(volume);
        ampliState = (Wire.endTransmission() == 0);
      }

      bool detectI2CDevice(byte address) {
        Wire.beginTransmission(address);
        return (Wire.endTransmission() == 0);
      }

      int updateSrfRange(int address);
      unsigned int getSrfRange(int address);
      int getSrfLight(int address);
      int changeSrfAddress(int oldAddress, int newAddress);
      int changeSrfGain(int address, int gain);
      int changeSrfDistance(int address, int range);
      void testSrfs();
      void setSrfsRegisters();
      void testLasers();
      bool devicePresent(uint8_t address);

      inline void setSrfContinuousUpdate(boolean value) { isSrfUpdateContinuous = value; }

      inline void setK(byte k, float value) {
        switch (k) {
          case 0: ArduBot::par_motores.kp = value; break;
          case 1: ArduBot::par_motores.ki = value; break;
          case 2: ArduBot::par_motores.kd = value; break;
        }
      }

      void spinOnce();
  };
}

extern volatile uint8_t ir_score[3];
#endif
