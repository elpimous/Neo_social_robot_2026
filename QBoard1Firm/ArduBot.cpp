
#include "ArduBot.h"
#include <IRremote.h>

using namespace arduBot;

bool ArduBot::leftMotorHallA = LOW;
bool ArduBot::leftMotorHallB = LOW;
bool ArduBot::rightMotorHallA = LOW;
bool ArduBot::rightMotorHallB = LOW;
byte ArduBot::ir1data = 0;
byte ArduBot::ir2data = 0;
byte ArduBot::ir3data = 0;
uint16_t ArduBot::LeftDist = 0;
uint16_t ArduBot::RightDist = 0;
uint16_t ArduBot::BackLeftDist = 0;
uint16_t ArduBot::BackRightDist = 0;
uint16_t ArduBot::MiddleDist = 0;
long ArduBot::spinLoopPeriodMs = 5;
long spinTime = 0;
long srf10time = 0;
bool ArduBot::wheelStopAlertFlag = false;
bool ArduBot::robotFallFlag = false;
bool ArduBot::robotCrashFlag = false;
bool ArduBot::robotStallFlag = false;

CMotors ArduBot::rightMotor = CMotors(&MOTOR_1_COTROL_1_REGISTER, &MOTOR_1_COTROL_2_REGISTER, &MOTOR_1_COTROL_1_PORT, &MOTOR_1_COTROL_2_PORT, MOTOR_1_CONTROL_1_PIN, MOTOR_1_CONTROL_2_PIN, MOTOR_1_PWM_ARDUINO_PIN);
CMotors ArduBot::leftMotor  = CMotors(&MOTOR_2_COTROL_1_REGISTER, &MOTOR_2_COTROL_2_REGISTER, &MOTOR_2_COTROL_1_PORT, &MOTOR_2_COTROL_2_PORT, MOTOR_2_CONTROL_1_PIN, MOTOR_2_CONTROL_2_PIN, MOTOR_2_PWM_ARDUINO_PIN);
CBaseMovement ArduBot::par_motores = CBaseMovement(&ArduBot::leftMotor, &ArduBot::rightMotor);

byte oldPort = 0;
ISR(PCINT1_vect)
{
  byte port = PINJ & ((1 << MOTOR_1_HALL_A_PIN) | (1 << MOTOR_1_HALL_B_PIN) | (1 << MOTOR_2_HALL_A_PIN) | (1 << MOTOR_2_HALL_B_PIN));
  if (port == oldPort) return;
  oldPort = port;
  boolean newLeftMotorHallA  = ((port & (1 << MOTOR_2_HALL_A_PIN)) != 0);
  boolean newLeftMotorHallB  = ((port & (1 << MOTOR_2_HALL_B_PIN)) != 0);
  boolean newRightMotorHallA = ((port & (1 << MOTOR_1_HALL_A_PIN)) != 0);
  boolean newRightMotorHallB = ((port & (1 << MOTOR_1_HALL_B_PIN)) != 0);

  if (newLeftMotorHallA != ArduBot::leftMotorHallA) {
    if (newLeftMotorHallA == newLeftMotorHallB) ArduBot::leftMotor.pulsesDifference--;
    else ArduBot::leftMotor.pulsesDifference++;
    ArduBot::leftMotorHallA = newLeftMotorHallA;
  } else if (newLeftMotorHallB != ArduBot::leftMotorHallB) {
    if (newLeftMotorHallB != newLeftMotorHallA) ArduBot::leftMotor.pulsesDifference--;
    else ArduBot::leftMotor.pulsesDifference++;
    ArduBot::leftMotorHallB = newLeftMotorHallB;
  }
  if (newRightMotorHallA != ArduBot::rightMotorHallA) {
    if (newRightMotorHallA == newRightMotorHallB) ArduBot::rightMotor.pulsesDifference++;
    else ArduBot::rightMotor.pulsesDifference--;
    ArduBot::rightMotorHallA = newRightMotorHallA;
  } else if (newRightMotorHallB != ArduBot::rightMotorHallB) {
    if (newRightMotorHallB != newRightMotorHallA) ArduBot::rightMotor.pulsesDifference++;
    else ArduBot::rightMotor.pulsesDifference--;
    ArduBot::rightMotorHallB = newRightMotorHallB;
  }
}

volatile uint8_t ir_score[3] = {0, 0, 0};
unsigned long lastEvalTime = 0;
ISR(INT6_vect) { if (ir_score[0] < 255) ir_score[0]++; }
ISR(INT7_vect) { if (ir_score[1] < 255) ir_score[1]++; }
ISR(INT2_vect) { if (ir_score[2] < 255) ir_score[2]++; }

ArduBot::ArduBot(double wheelRadius, double wheelDistance, int encoderResolution) :
  lcd(0xc6), isSrfUpdateContinuous(true),
  NUM_SRFs(0), NUM_LASERS(0), NUM_SRFs_FRONT(0),
  NUM_LASERs_FRONT(2), NUM_SRFs_BACK(2), NUM_LASERs_BACK(0), MAX_LASERS(2),
  floorSensorPresent(true), lcdState(false), energyState(false),
  accelerometer(0x1C), gyro(0x69),
  gyroState(false), accelerometerState(false), ampliState(false)
{
  DDRJ &= ~(1 << MOTOR_1_HALL_A_PIN) & ~(1 << MOTOR_1_HALL_B_PIN) & ~(1 << MOTOR_2_HALL_A_PIN) & ~(1 << MOTOR_2_HALL_B_PIN);
  PORTJ |= (1 << MOTOR_1_HALL_A_PIN) | (1 << MOTOR_1_HALL_B_PIN) | (1 << MOTOR_2_HALL_A_PIN) | (1 << MOTOR_2_HALL_B_PIN);
  PCICR |= (1 << PCIE1);

  // ✅ GARDER — initialisation variables
  ArduBot::par_motores.initializePhisicalVariables(encoderResolution, wheelDistance, wheelRadius);
  xCoordinate = 0; yCoordinate = 0; thetaCoordinate = 0; alert_stop = 0;
}

void ArduBot::begin(double spinLoopPeriodS, double kp, double ki, double kd)
{
  DDRB = DDRB | B10001100;
  PORTB = 0;

  // vince : baud rate 500000 — FTDI FT232R : 48MHz/96 = 500000, erreur 0%
  Serial.begin(500000);
  Serial.flush();

  Wire.begin();
  Wire.setClock(400000);
  // vince : timeout Wire 80ms — SRF10 prend ~65ms, 25ms (ancien) trop court
  Wire.setWireTimeout(80000, true);

  if (detectI2CDevice(LCD_I2C_ADDRESS >> 1)) {
    Serial.println("LCD présent...");
    lcdState = true;
    lcd.begin(20, 4);
    lcd.noDisplay();
    lcd.createChar(5, hp);
    lcd.createChar(6, pc);
    lcd.createChar(7, sect);
    lcd.clear();
    lcd.home();
    delay(100);
  } else {
    lcdState = false;
    Serial.println("LCD HS...");
  }

  // On upprime la variable locale car on utlilse directement le membre
  // L3G4200D gyro(0x69);
  gyroState = gyro.begin(2000);
  // LIS35DE accel(0x1E);
  accelerometerState = accelerometer.begin();
  initLasers();
  lcd.home(); lcd.clear(); lcd.print("Waiting for Qbo_PC");

  byte value, stat;
  getBatteryLevel(&value, &stat);
  ampliState = initAmpli(AMPLI_ADDR);

  oldPort = PINJ & ((1 << MOTOR_1_HALL_A_PIN) | (1 << MOTOR_1_HALL_B_PIN) | (1 << MOTOR_2_HALL_A_PIN) | (1 << MOTOR_2_HALL_B_PIN));

  //**Impact :** Au démarrage, la première transition Hall peut générer un pulse fantôme
  // dans le mauvais sens, introduisant une erreur initiale d'odométrie.
  leftMotorHallA  = ((oldPort & (1 << MOTOR_2_HALL_A_PIN)) != 0);  // MOTOR_2 = gauche, cohérent avec l' ISR
  leftMotorHallB  = ((oldPort & (1 << MOTOR_2_HALL_B_PIN)) != 0);
  rightMotorHallA = ((oldPort & (1 << MOTOR_1_HALL_A_PIN)) != 0);  // MOTOR_1 = droite
  rightMotorHallB = ((oldPort & (1 << MOTOR_1_HALL_B_PIN)) != 0);

  ArduBot::spinLoopPeriodMs = long(spinLoopPeriodS * 1000);
  ArduBot::par_motores.initializePIDVariables(kp, kd, ki, spinLoopPeriodS);
  spinTime = srf10time = millis();
  TCCR4B = (TCCR4B & 0b11111000) | 0x01;
  PCMSK1 = 0 | (1 << PCINT11) | (1 << PCINT12) | (1 << PCINT13) | (1 << PCINT14);
  EIMSK |= (1 << INT2) | (1 << INT6) | (1 << INT7);
  Serial.println("Initialisation IR...");

  // -----------------------------------------------------------------------
  // CONFIGURATION INTERRUPTIONS IR — INT2 (IR3/PD2), INT6 (IR1/PE6), INT7 (IR2/PE7)
  //
  // ⚠️  CORRECTION BUG (Mars 2026) : double configuration supprimée.
  //     L'ancienne version configurait ces mêmes registres une première
  //     fois dans le constructeur ArduBot(), puis une seconde fois ici.
  //     → Redondance trompeuse : on ne sait plus quelle version fait foi.
  //     → Risque : si on modifie un bloc sans modifier l'autre, les deux
  //       divergent silencieusement et le comportement devient imprévisible.
  //
  // RÈGLE : le constructeur n'initialise QUE les variables C++.
  //         Toute configuration hardware (DDR, PORT, EICRA, EIMSK...)
  //         appartient exclusivement à begin(), appelé explicitement
  //         par l'utilisateur quand le hardware est prêt.
  //
  // Broches :
  //   PE6 (Arduino pin 5)  → IR1 → INT6
  //   PE7 (Arduino pin 6)  → IR2 → INT7
  //   PD2 (Arduino pin 19) → IR3 → INT2
  //
  // Mode déclenchement : flanc descendant (ISC_1=1, ISC_0=0)
  // -----------------------------------------------------------------------
  DDRE &= ~((1 << IR1_BIT) | (1 << IR2_BIT));
  DDRD &= ~(1 << IR3_BIT);
  PORTE |= (1 << IR1_BIT) | (1 << IR2_BIT);
  PORTD |= (1 << IR3_BIT);
  EICRB &= ~((1 << ISC60) | (1 << ISC70));
  EICRB |= (1 << ISC61) | (1 << ISC71);
  EICRA &= ~((1 << ISC20) | (1 << ISC30));
  EICRA |= (1 << ISC21) | (1 << ISC31);
  EIMSK |= (1 << INT6) | (1 << INT7) | (1 << INT2);
  IrReceiver.begin(IR_RECEIVE_PIN);
}

/* **Sur ATmega2560, le timer0 (base de millis()) n'est configuré qu'au début de `main()`,
APRÈS les initialisations statiques.** `millis()` retourne 0 dans ce contexte.
`old_time` est aussi non utilisé dans le code — la variable est morte.
// long old_time = millis(); // ← appelé AVANT setup(), timers non démarrés
*/
long stopTime = 1000;
long speedComandTime = 0;

void ArduBot::updatePosition(float loopPeriodSeconds)
{
  long now = millis();
  if (now > speedComandTime + stopTime) setSpeeds(0, 0);
  ArduBot::par_motores.doControlLoop();
  estimatePosition(loopPeriodSeconds);
}

void ArduBot::estimatePosition(float loopPeriodSeconds)
{
  float odometryLinearMovement = ArduBot::par_motores.actualLinearMovement;

  // vince : constante précalculée → 1 multiplication au lieu de 2 par cycle
  // Anciennement : 1.042874 * 1.0256 * ... → deux opérations flottantes
  static const float ODO_ANGULAR_CORRECTION = 1.042874f * 1.0256f;
  float odometryAngularMovement = ODO_ANGULAR_CORRECTION * ArduBot::par_motores.actualAngularMovement;

  float gyroAngularMovement = ((float)gyroZ) * 0.0174533f * loopPeriodSeconds;

  const float alpha = 0.95f; // 0.98f 
  float angularMovement = alpha * gyroAngularMovement + (1 - alpha) * odometryAngularMovement;

  // vince : cos/sin précalculés une seule fois → économise ~400µs/cycle (8% budget 5ms)
  // Anciennement : cos() et sin() appelés séparément avec le même argument
  // Sur ATmega2560 à 16MHz, chaque appel flottant coûte ~200µs
  float angle_mid = thetaCoordinate + angularMovement / 2.0f;
  float c = cos(angle_mid);
  float s = sin(angle_mid);
  xCoordinate += odometryLinearMovement * c;
  yCoordinate += odometryLinearMovement * s;

  if (abs(currentLinearSpeed) > 0.001 || abs(currentAngularSpeed) > 0.01 || abs(gyroZ) > 0.8) {
    thetaCoordinate += angularMovement;
  }
}

bool ArduBot::devicePresent(uint8_t address)
{
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

void ArduBot::scanI2C()
{
  byte error, address;
  int nDevices = 0;
  Serial.println("Scanning...");
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX); Serial.println("  !");
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
}

void ArduBot::setSpeeds(double linear_speed, double angular_speed)
{
  if (MiddleDist < MIN_FLOOR_DISTANCE_CM || MiddleDist > MAX_FLOOR_DISTANCE_CM)
    if (linear_speed > 0) linear_speed = 0.0;
  if (abs(roll) > 9 || abs(pitch) > 9) { linear_speed = 0.0; angular_speed = 0.0; }
  if (LeftDist  < 20 && LeftDist  != 0 && linear_speed > 0) linear_speed = 0.0;
  if (RightDist < 20 && RightDist != 0 && linear_speed > 0) linear_speed = 0.0;

  // -----------------------------------------------------------------------
  // SÉCURITÉ ARRIÈRE — SRF10 ultrason arrière gauche (115) et droit (114)
  //
  // Convention de signe : linear_speed > 0 = avance, linear_speed < 0 = recul
  //
  // ⚠️  CORRECTION BUG (Mars 2026) : signe était > 0 → logique inversée
  //     Un obstacle ARRIÈRE doit bloquer le RECUL (< 0), pas l'avance.
  //     L'ancienne version empêchait d'avancer quand un obstacle était
  //     derrière, et laissait reculer dedans sans protection.
  //
  // Seuil : 10 cm (BackDist != 0 exclut les lectures invalides du SRF10)
  // -----------------------------------------------------------------------
  if (BackRightDist < 10 && BackRightDist != 0 && linear_speed < 0) linear_speed = 0.0;
  if (BackLeftDist  < 10 && BackLeftDist  != 0 && linear_speed < 0) linear_speed = 0.0;
  currentLinearSpeed  = linear_speed;
  currentAngularSpeed = angular_speed;
  ArduBot::par_motores.desiredLinearSpeed  = linear_speed;
  ArduBot::par_motores.desiredAngularSpeed = angular_speed;
  // ArduBot::par_motores.transformSpeeds2Pulses();
  speedComandTime = millis();
}

int ArduBot::updateSrfRange(int address)
{
  Wire.beginTransmission(address);
  Wire.write(srfCmdByte);
  Wire.write(0x51);
  Wire.endTransmission();
  return 1;
}

unsigned int ArduBot::getSrfRange(int address)
{
  Wire.beginTransmission(address);
  Wire.write(srfRangeByte);
  Wire.endTransmission();
  Wire.requestFrom(address, 2);
  int highByte = Wire.read();
  int lowByte  = Wire.read();
  unsigned int range = (highByte << 8) + lowByte;
  if (range >= 60) return 9999;
  return range;
}

int ArduBot::getSrfLight(int address)
{
  Wire.beginTransmission(address);
  Wire.write(srfLightByte);
  Wire.endTransmission();
  Wire.requestFrom(address, 1);
  return Wire.read();
}

int ArduBot::changeSrfAddress(int oldAddress, int newAddress)
{
  Wire.beginTransmission(oldAddress); Wire.write(srfCmdByte); Wire.write(0xA0); Wire.endTransmission();
  Wire.beginTransmission(oldAddress); Wire.write(srfCmdByte); Wire.write(0xAA); Wire.endTransmission();
  Wire.beginTransmission(oldAddress); Wire.write(srfCmdByte); Wire.write(0xA5); Wire.endTransmission();
  Wire.beginTransmission(oldAddress); Wire.write(srfCmdByte); Wire.write(newAddress); Wire.endTransmission();
  return 1;
}

int ArduBot::changeSrfGain(int address, int gain)
{
  Wire.beginTransmission(address); Wire.write(srfGainByte); Wire.write(gain); Wire.endTransmission();
  return 1;
}

int ArduBot::changeSrfDistance(int address, int range)
{
  Wire.beginTransmission(address); Wire.write(srfRangeByte); Wire.write(range); Wire.endTransmission();
  return 1;
}

void ArduBot::testSrfs()
{
  Serial.println("Test SRF...");
  int address = 0x70;
  for (byte i = 0; i < 16; i++) {
    if (getSrfLight(address) == 0x80) { SRFs_ADRESS[NUM_SRFs] = address; NUM_SRFs++; }
    address++;
  }
  setSrfsRegisters();
}

void ArduBot::setSrfsRegisters()
{
  for (int i = 0; i < NUM_SRFs; i++) { changeSrfDistance(SRFs_ADRESS[i], 24); changeSrfGain(SRFs_ADRESS[i], 6); }
}

void ArduBot::filterGyro(float &gx, float &gy, float &gz)
{
  static float alpha = 0.7; // 0.92 // τ ≈ 59ms de lag à 5ms/cycle
  // À 0.92, la réponse du gyro est très lente. Si Néo tourne vite, 
  // estimatePosition() va sous-estimer gyroAngularMovement pendant ~60ms
  static float gx_prev = 0, gy_prev = 0, gz_prev = 0;
  gx = alpha * gx_prev + (1 - alpha) * gx;
  gy = alpha * gy_prev + (1 - alpha) * gy;
  gz = alpha * gz_prev + (1 - alpha) * gz;
  gx_prev = gx; gy_prev = gy; gz_prev = gz;
}

void ArduBot::testLasers()
{
  Serial.println("Test VL53L1X Lasers...");
  int addressesToCheck[MAX_LASERS] = { 0x70, 0x71 };
  for (byte i = 0; i < MAX_LASERS; i++) {
    int addr = addressesToCheck[i];
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) { LASERs_ADRESS[NUM_LASERS] = addr; NUM_LASERS++; }
  }
}

byte index_front = 0;
byte index_back  = 0;
long accelerometerTime = 0;
long batteryTime = 0;
byte accelerometer_fall_count = 0;
int heartFrame = 0;

void ArduBot::spinOnce()
{
  long now = millis();

  if (now - spinTime > ArduBot::spinLoopPeriodMs)
  {
    spinTime += ArduBot::spinLoopPeriodMs;
    static unsigned long lastMicros = micros();

    // vince : mesure du temps AVANT les lectures I2C pour ne pas biaiser l'odométrie
    // Anciennement : mesure APRÈS calibrationStep() → loopPeriodSeconds incluait
    // le temps de collecte (~0.2ms) → biais odométrie pendant calibration
    unsigned long nowMicros = micros();
    float loopPeriodSeconds = (nowMicros - lastMicros) / 1e6f;
    lastMicros = nowMicros;

    // Lecture unique IMU — utilisée à la fois pour calibrationStep ET spinOnce
    // vince : évite la double lecture I2C par cycle (goulot 2)
    // Anciennement : calibrationStep() relisait gyro+accel indépendamment
    // → 4 lectures I2C au lieu de 2 pendant la calibration
    int8_t raw_ax, raw_ay, raw_az;
    int raw_gx, raw_gy, raw_gz;
    gyro.getGyroValues(raw_gx, raw_gy, raw_gz);
    accelerometer.getAccelerometerValues(raw_ax, raw_ay, raw_az);

    // vince : passe les valeurs déjà lues à calibrationStep → zéro lecture I2C supplémentaire
    calibrationStep(raw_gx, raw_gy, raw_gz, raw_ax, raw_ay, raw_az);

    // Transformation capteur → robot
    float tmpAccX = -raw_ax;
    float tmpAccY = -raw_ay;
    float tmpAccZ =  raw_az;
    float tmpGyroX =  raw_gy;
    float tmpGyroY = -raw_gx;
    float tmpGyroZ =  raw_gz;

    // Conversion et offsets
    accelerometerX = tmpAccX * ACC_SENSITIVITY - accelerometerX0;
    accelerometerY = tmpAccY * ACC_SENSITIVITY - accelerometerY0;
    accelerometerZ = tmpAccZ * ACC_SENSITIVITY - accelerometerZ0;
    gyroX = (tmpGyroX - gyroX0) * gyroSensitivity;
    gyroY = (tmpGyroY - gyroY0) * gyroSensitivity;
    gyroZ = (tmpGyroZ - gyroZ0) * gyroSensitivity;

    filterGyro(gyroX, gyroY, gyroZ);

    // Roll & Pitch
    if (fabs(accelerometerZ) > 0.1f) {
      roll  = 180.0f * accelerometerY / (accelerometerZ * PI);
      pitch = 180.0f * accelerometerX / (accelerometerZ * PI);
    } else {
      roll = pitch = 90.0f;
    }

    // vince : thetaGyroOnly supprimé — variable jamais utilisée nulle part
    // Anciennement : multiplication flottante inutile à chaque cycle de 5ms
    // static float thetaGyroOnly = 0;
    // thetaGyroOnly += gyroZ * loopPeriodSeconds;

    updatePosition(loopPeriodSeconds);
  }

  if (now - srf10time > 130)
  {
    srf10time += 130;
    LaserL.read(false);
    LaserR.read(false);
    MiddleDist = getFloorDistance();

    if (abs(ArduBot::par_motores.actualLeftWheelPulses) > 0 ||
        abs(ArduBot::par_motores.actualRightWheelPulses) > 0 ||
        isSrfUpdateContinuous)
    {
      if (LaserL.ranging_data.range_status == 0 && LaserL.ranging_data.range_mm > 0)
        LeftDist = LaserL.ranging_data.range_mm / 10;
      else
        LeftDist = 0;

      if (LaserR.ranging_data.range_status == 0 && LaserR.ranging_data.range_mm > 0)
        RightDist = LaserR.ranging_data.range_mm / 10;
      else
        RightDist = 0;

      if (index_back == 0) { BackLeftDist  = getSrfRange(115); updateSrfRange(115); index_back = 1; }
      else                 { BackRightDist = getSrfRange(114); updateSrfRange(114); index_back = 0; }

      if (IrReceiver.decode()) {
        uint32_t code = IrReceiver.decodedIRData.decodedRawData;
        Serial.print("📩 Trame IR reçue : 0x"); Serial.println(code, HEX);
        lcd.setCursor(4, 1); lcd.print("        ");
        if (code == IR_ADRESSE_BASE) {
          Serial.println("✅ Base reconnue !");
          if (millis() - lastEvalTime > 5000) {
            lastEvalTime = millis();
            Serial.print("Score IR1: "); Serial.print(ir_score[0]);
            Serial.print(" | IR2: "); Serial.print(ir_score[1]);
            Serial.print(" | IR3: "); Serial.println(ir_score[2]);
            lcd.setCursor(4, 1);
            if      (ir_score[0] > ir_score[1] && ir_score[0] > ir_score[2]) { lcd.print("IR1: =>"); }
            else if (ir_score[1] > ir_score[0] && ir_score[1] > ir_score[2]) { lcd.print("IR2: ="); }
            else if (ir_score[2] > ir_score[0] && ir_score[2] > ir_score[1]) { lcd.print("IR3: <="); }
            else                                                               { lcd.print(" ? "); }
            ir_score[0] = ir_score[1] = ir_score[2] = 0;
          }
        } else { Serial.println("❌ Code inconnu"); }
        IrReceiver.resume();
      }
    }
  }

  if (now - batteryTime > 1000)
  {
    lcd.setCursor(0, 2); lcd.print("                    "); // vince ajout 1 caractere manquant = 20
    lcd.setCursor(0, 2);
    lcd.print("X"); lcd.print((int)(xCoordinate * 100));
    lcd.setCursor(5, 2);
    lcd.print("Y"); lcd.print((int)(yCoordinate * 100));
    lcd.setCursor(10, 2);
    lcd.print("T"); lcd.print((int)(thetaCoordinate * 180.0 / PI));
    lcd.setCursor(15, 2);
    lcd.print("I"); lcd.print((int)(max(abs(pitch), abs(roll))));
    batteryTime += 1000;

    byte stat = 0, value = 0, charg = 0;
    lcd.setCursor(0, 3); lcd.print("Bat: ");

    if (getBatteryLevel(&value, &stat)) {
      float batV = value / 10.0;
      charg = (stat >> 3) & 0x07;
      lcd.print(batV, 1); lcd.print("V ");
      displayBatteryIcon(lcd, charg, batV);
      bool pcOn = (stat >> 1) & 0x01;
      if (ampliState) {
        if (pcOn) { lcd.setCursor(18, 3); lcd.write(5); lcd.write(6); setAmpliVolume(30); }
        else      { muteAmpli(); lcd.setCursor(18, 3); lcd.print("  "); }
      }
    } else {
      lcd.print("        ");
      lcd.setCursor(0, 3); lcd.print("No com !");
      lcd.setCursor(18, 3); lcd.print("  ");
      Serial.println("⚠️ Impossible de lire le niveau de batterie !");
    }
    lcd.setCursor(16, 3);
    lcd.print((heartFrame == 0) ? "-" : "|");
    heartFrame = !heartFrame;
  }
}
