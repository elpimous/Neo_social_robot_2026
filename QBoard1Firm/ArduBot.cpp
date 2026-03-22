
#include "ArduBot.h"
#include <IRremote.h>

using namespace arduBot;
//---------Definicion de las variables static----------------------//
// states of encoders
bool ArduBot::leftMotorHallA = LOW;
bool ArduBot::leftMotorHallB = LOW;
bool ArduBot::rightMotorHallA = LOW;
bool ArduBot::rightMotorHallB = LOW;

// states of IR sensors
byte ArduBot::ir1data = 0;
byte ArduBot::ir2data = 0;
byte ArduBot::ir3data = 0;

// ranging
uint16_t ArduBot::LeftDist = 0;
uint16_t ArduBot::RightDist = 0;
uint16_t ArduBot::BackLeftDist = 0;
uint16_t ArduBot::BackRightDist = 0;
uint16_t ArduBot::MiddleDist = 0;

// cycle times
long ArduBot::spinLoopPeriodMs = 5;
long spinTime = 0;
long srf10time = 0;
bool ArduBot::wheelStopAlertFlag=false;
bool ArduBot::robotFallFlag=false;
bool ArduBot::robotCrashFlag=false;
bool ArduBot::robotStallFlag=false;

// motors
//velocidad maxima del motor libre: 216rpm -> 22,619467109rad/s.
// nominal: 170-> 17,802358373rad/s
CMotors ArduBot::rightMotor = CMotors(&MOTOR_1_COTROL_1_REGISTER, &MOTOR_1_COTROL_2_REGISTER, &MOTOR_1_COTROL_1_PORT, &MOTOR_1_COTROL_2_PORT, MOTOR_1_CONTROL_1_PIN, MOTOR_1_CONTROL_2_PIN, MOTOR_1_PWM_ARDUINO_PIN);
CMotors ArduBot::leftMotor = CMotors(&MOTOR_2_COTROL_1_REGISTER, &MOTOR_2_COTROL_2_REGISTER, &MOTOR_2_COTROL_1_PORT, &MOTOR_2_COTROL_2_PORT, MOTOR_2_CONTROL_1_PIN, MOTOR_2_CONTROL_2_PIN, MOTOR_2_PWM_ARDUINO_PIN);
CBaseMovement ArduBot::par_motores = CBaseMovement(&ArduBot::leftMotor, &ArduBot::rightMotor);

//------------------------------------------------//
//--------Interrupcion para los motores-----------//
byte oldPort = 0;
ISR ( PCINT1_vect )
{
  byte port = PINJ & ((1 << MOTOR_1_HALL_A_PIN) | (1 << MOTOR_1_HALL_B_PIN) | (1 << MOTOR_2_HALL_A_PIN) | (1 << MOTOR_2_HALL_B_PIN));
  if (port == oldPort)
    return;
  oldPort = port;
  boolean newLeftMotorHallA = ((port & (1 << MOTOR_2_HALL_A_PIN)) != 0);
  boolean newLeftMotorHallB = ((port & (1 << MOTOR_2_HALL_B_PIN)) != 0);
  boolean newRightMotorHallA = ((port & (1 << MOTOR_1_HALL_A_PIN)) != 0);
  boolean newRightMotorHallB = ((port & (1 << MOTOR_1_HALL_B_PIN)) != 0);

  if (newLeftMotorHallA != ArduBot::leftMotorHallA)
  {
    if (newLeftMotorHallA == newLeftMotorHallB)
      ArduBot::leftMotor.pulsesDifference--;
    else
      ArduBot::leftMotor.pulsesDifference++;
    ArduBot::leftMotorHallA = newLeftMotorHallA;
  }
  else if (newLeftMotorHallB != ArduBot::leftMotorHallB)
  {
    if (newLeftMotorHallB != newLeftMotorHallA)
      ArduBot::leftMotor.pulsesDifference--;
    else
      ArduBot::leftMotor.pulsesDifference++;
    ArduBot::leftMotorHallB = newLeftMotorHallB;
  }

  if (newRightMotorHallA != ArduBot::rightMotorHallA)
  {
    if (newRightMotorHallA == newRightMotorHallB)
      ArduBot::rightMotor.pulsesDifference++;
    else
      ArduBot::rightMotor.pulsesDifference--;
    ArduBot::rightMotorHallA = newRightMotorHallA;
  }
  else if (newRightMotorHallB != ArduBot::rightMotorHallB)
  {
    if (newRightMotorHallB != newRightMotorHallA)
      ArduBot::rightMotor.pulsesDifference++;
    else
      ArduBot::rightMotor.pulsesDifference--;
    ArduBot::rightMotorHallB = newRightMotorHallB;
  }
}

// Scores de détection par capteur IR
volatile uint8_t ir_score[3] = {0, 0, 0};
unsigned long lastEvalTime = 0;
// --- Interruptions matérielles IR ---

ISR(INT6_vect) {
  if (ir_score[0] < 255) ir_score[0]++;
}

ISR(INT7_vect) {
  if (ir_score[1] < 255) ir_score[1]++;
}

ISR(INT2_vect) {
  if (ir_score[2] < 255) ir_score[2]++;
}


//----------Inicializacion del sistema----------------------//
ArduBot::ArduBot(double wheelRadius, double wheelDistance, int encoderResolution) : 
  lcd(0xc6), 
  isSrfUpdateContinuous(true),
  NUM_SRFs(0),
  NUM_LASERS(0),
  NUM_SRFs_FRONT(0),
  NUM_LASERs_FRONT(2), 
  NUM_SRFs_BACK(2),
  NUM_LASERs_BACK(0),
  MAX_LASERS(2),
  floorSensorPresent(true),
  lcdState(false), 
  energyState(false), 
  accelerometer(0x1C), 
  gyro(0x69), 
  gyroState(false), 
  accelerometerState(false), 
  ampliState(false)  
{
  //----------Configuracion de los pines---------//
  DDRJ &= ~(1 << MOTOR_1_HALL_A_PIN) & ~(1 << MOTOR_1_HALL_B_PIN) & ~(1 << MOTOR_2_HALL_A_PIN) & ~(1 << MOTOR_2_HALL_B_PIN);  //Pines de los sensores HALL como entradas
  PORTJ |= (1 << MOTOR_1_HALL_A_PIN) | (1 << MOTOR_1_HALL_B_PIN) | (1 << MOTOR_2_HALL_A_PIN) | (1 << MOTOR_2_HALL_B_PIN);  //Resistencias de pull-up para sensores hall
  
  //-----Interrupciones para los motores---------//
  PCICR |= (1 << PCIE1);  //Habilito las interrupciones de los motores
 
  //-----Interrupciones para los IR--------------//
  DDRE &= ~(1 << 6) & ~(1 << 7);  //Pines de los sensores IR como entradas
  DDRD &= ~(1 << 2);  //Pines de los sensores IR como entradas
  PORTE |= (1 << 6) | (1 << 7);  //Resistencias de pull-up para IRs
  PORTD |= (1 << 2);  //Resistencias de pull-up para IRs
  EICRA &= ~(1 << ISC20) & ~(1 << ISC30);  //flanco de bajada (IR3 y Lect_Pulsador)
  EICRA |= (1 << ISC21) | (1 << ISC31);  //flanco de bajada (IR3 y Lect_Pulsador)
  EICRB &= ~(1 << ISC60) & ~(1 << ISC70);  //flanco de bajada (IR1 e IR2)
  EICRB |= (1 << ISC61) | (1 << ISC71);  //flanco de bajada (IR1 e IR2)


  ArduBot::par_motores.initializePhisicalVariables(encoderResolution, wheelDistance, wheelRadius);
  xCoordinate = 0;
  yCoordinate = 0;
  thetaCoordinate = 0;
  alert_stop = 0;
}

void ArduBot::begin(double spinLoopPeriodS, double kp, double ki, double kd)
{
  DDRB = DDRB | B10001100; // Met PB2 et PB3 à LOW
  PORTB = 0;
  
  //-----Inicializacion del puerto serie---------//
  // vince : baud rate 115200 → 500000 pour réduire le temps de transmission de 4.3×
  // Exemple : trame GET_ODOMETRY (20 octets) passe de ~1.7ms à ~0.4ms
  // → jitter mutex QBoard1 divisé par 4 entre base_ctrl et imu_ctrl
  // ⚠️ Mettre à jour baud1 dans qboards_config.yaml en conséquence (500000)
  Serial.begin(500000);
  Serial.flush();
  
  //-------Inicializacion del puerto I2C---------//
  Wire.begin();
  Wire.setClock(400000);
  // vince : timeout Wire augmenté de 25ms → 80ms
  // Raison : un SRF10 prend ~65ms pour une mesure ultrason
  // Avec 25ms, le bus I2C resetait pendant la mesure → données corrompues
  // 80ms = 65ms mesure SRF10 + 15ms de marge de sécurité
  Wire.setWireTimeout(80000, true);
  
  //-------Inicializacion del LCD----------------//
  if (detectI2CDevice(LCD_I2C_ADDRESS >> 1)) 
  { // >>1 si l'adresse est donnée en 8 bits
    Serial.println("LCD présent...");
    lcdState = true;
    lcd.begin(20, 4);
    lcd.noDisplay();
    lcd.createChar(5, hp);      // special character loading
    lcd.createChar(6, pc);      // special character loading
    lcd.createChar(7, sect);    // special character loading
    lcd.clear();
    lcd.home();
    delay(100);
  } else {
    lcdState = false;
    Serial.println("LCD HS...");
  }
  
  // lcd.print("Calibrating...");
  
  //-------Inicializacion del L3G4200D y LIS35DE----------------//
  L3G4200D gyro(0x69);
  gyroState = gyro.begin(2000);

  LIS35DE accel(0x1E);
  accelerometerState = accelerometer.begin();
  
  initLasers();   // Initialisation des lasers VL53L1X
  
  // calibrate();

  lcd.home();
  lcd.clear();
  lcd.print("Waiting for Qbo_PC");

  // scanI2C();
  // delay(3000);

  // testSrfs();
  // testLasers();

  //-------Reading energy status---------------------//
  byte value, stat;
  getBatteryLevel(&value, &stat);
  
  //-------Init audio ampli---------------------//
  // Mise en mute au démarrage
  ampliState = initAmpli(AMPLI_ADDR);

  oldPort = PINJ & ((1 << MOTOR_1_HALL_A_PIN) | (1 << MOTOR_1_HALL_B_PIN) | (1 << MOTOR_2_HALL_A_PIN) | (1 << MOTOR_2_HALL_B_PIN));
  leftMotorHallA = ((oldPort & (1 << MOTOR_1_HALL_A_PIN)) != 0);
  leftMotorHallB = ((oldPort & (1 << MOTOR_1_HALL_B_PIN)) != 0);
  rightMotorHallA = ((oldPort & (1 << MOTOR_2_HALL_A_PIN)) != 0);
  rightMotorHallB = ((oldPort & (1 << MOTOR_2_HALL_B_PIN)) != 0);
  ArduBot::spinLoopPeriodMs = long(spinLoopPeriodS * 1000);
  ArduBot::par_motores.initializePIDVariables(kp, kd, ki, spinLoopPeriodS);
  spinTime = srf10time = millis();
  TCCR4B = (TCCR4B & 0b11111000) | 0x01; //Frecuencia del pwm al máximo para evitar el ruido en los motores (otra )
  PCMSK1 = 0 | (1 << PCINT11) | (1 << PCINT12) | (1 << PCINT13) | (1 << PCINT14);  //Pines que interrumpen para los encoders de los motores
  EIMSK |= (1 << INT2) | (1 << INT6) | (1 << INT7) | (1 << INT3); //Las pongo en marcha
  
  Serial.println("Initialisation IR (interruption + décodage)...");
  // --- Configuration des entrées par registres ---
  DDRE &= ~((1 << IR1_BIT) | (1 << IR2_BIT)); // PE6, PE7 en entrée
  DDRD &= ~(1 << IR3_BIT);                   // PD2 en entrée

  PORTE |= (1 << IR1_BIT) | (1 << IR2_BIT);  // Pull-up internes
  PORTD |= (1 << IR3_BIT);

  // --- Interruptions externes (flanc descendant) ---
  EICRB &= ~((1 << ISC60) | (1 << ISC70));
  EICRB |= (1 << ISC61) | (1 << ISC71);

  EICRA &= ~((1 << ISC20) | (1 << ISC30));
  EICRA |= (1 << ISC21) | (1 << ISC31);

  EIMSK |= (1 << INT6) | (1 << INT7) | (1 << INT2);

  // --- Réception IR via IRremote sur PE7 (INT7) ---
  IrReceiver.begin(IR_RECEIVE_PIN);
}
//----------------------------------------------------------//
//-----------------Funciones para los encoders--------------//

long old_time = millis();
long stopTime = 1000;
long speedComandTime = 0;
void ArduBot::updatePosition(float loopPeriodSeconds)
{
  long now = millis();
  if (now > speedComandTime + stopTime)
  {
    setSpeeds(0, 0);
  }
  ArduBot::par_motores.doControlLoop();
  estimatePosition(loopPeriodSeconds);
}

void ArduBot::estimatePosition(float loopPeriodSeconds)
{
  float odometryLinearMovement = ArduBot::par_motores.actualLinearMovement;

  // ODOMÉTRIE : Δθ en radians
  float odometryAngularMovement = 1.042874 * 1.0256 * ArduBot::par_motores.actualAngularMovement;

  // GYRO : Δθ en radians (gyroZ en °/s → rad)
  float gyroAngularMovement = ((float)gyroZ) * 0.0174533f /*π/180*/ * loopPeriodSeconds;

  // FILTRE COMPLÉMENTAIRE (pondération gyro vs odo)
  const float alpha = 0.98f; // 98% gyro, 2% odométrie
  float angularMovement = alpha * gyroAngularMovement + (1 - alpha) * odometryAngularMovement;

  // POSITION (modèle de déplacement différentiel)
  float x = odometryLinearMovement * cos(thetaCoordinate + angularMovement / 2.0f);
  float y = odometryLinearMovement * sin(thetaCoordinate + angularMovement / 2.0f);

  xCoordinate += x;
  yCoordinate += y;
  if (abs(currentLinearSpeed) > 0.001 || abs(currentAngularSpeed) > 0.01 || abs(gyroZ) > 0.8) {
      thetaCoordinate += angularMovement;
      // Serial.println("🌀 θ intégré (robot en mouvement)");
  // } else {
  //     // Serial.println("⏸️ Robot immobile → pas d’intégration gyro");
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
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
}

// polar control of speed linear_speed in m/s and angular speed in rad/s
void ArduBot::setSpeeds(double linear_speed, double angular_speed)
{
  // lcd.setCursor(0,2);
  // lcd.print("                    ");
  if(MiddleDist<MIN_FLOOR_DISTANCE_CM || MiddleDist>MAX_FLOOR_DISTANCE_CM){
    // lcd.setCursor(3, 2);
    // lcd.print("Alert : Stop");
    if(linear_speed>0){
      linear_speed = 0.0;
    }
  }
  
  if(abs(roll)>9 || abs(pitch)>9){
    // lcd.setCursor(3, 2);
    // lcd.print("Alert : Fall");
    linear_speed = 0.0;
    angular_speed = 0.0;
  }

  if(LeftDist<20 && LeftDist!=0){
    // lcd.setCursor(1, 2);
    // lcd.print("Alert : Crash Left");
    if(linear_speed>0){
      linear_speed = 0.0;
    }
  }

  if(RightDist<20 && RightDist!=0){
    // lcd.setCursor(1, 2);
    // lcd.print("Alert : Crash Right");
    if(linear_speed>0){
      linear_speed = 0.0;
    }
  }

  if(BackRightDist<10 && BackRightDist!=0){
    // lcd.setCursor(3, 2);
    // lcd.print("Alert : Crash");
    if(linear_speed>0){
      linear_speed = 0.0;
    }
  }

  if(BackLeftDist<10 && BackLeftDist!=0){
    // lcd.setCursor(3, 2);
    // lcd.print("Alert : Crash");
    if(linear_speed>0){
      linear_speed = 0.0;
    }
  }
  
  currentLinearSpeed = linear_speed;
  currentAngularSpeed = angular_speed;
  ArduBot::par_motores.desiredLinearSpeed = linear_speed;
  ArduBot::par_motores.desiredAngularSpeed = angular_speed;
  ArduBot::par_motores.transformSpeeds2Pulses();
  speedComandTime = millis();
}

//---------------Funcion para leer los SRF10----------------//
int ArduBot::updateSrfRange(int address)
{
  Wire.beginTransmission(address);                 // Start communticating with SRF10
  Wire.write(srfCmdByte);                          // Send Command Byte
  Wire.write(0x51);                                // Send 0x51 to start a ranging
  Wire.endTransmission();
  return 1;
}

// This function gets a ranging from the SRF10
unsigned int ArduBot::getSrfRange(int address)
{
  Wire.beginTransmission(address);                 // start communicating with SRFmodule
  Wire.write(srfRangeByte);                        // Call the register for start of ranging data
  Wire.endTransmission();
  
  Wire.requestFrom(address, 2);                    // Request 2 bytes from SRF module
  //while(I2c.available() < 2);                    // Wait for data to arrive

  int highByte = Wire.read();                      // Get high byte
  int lowByte = Wire.read();                       // Get low byte

  unsigned int range = (highByte << 8) + lowByte;  // Put them together
  //Serial.println(range,DEC);
  // Filtre : si range dépasse ta portée max + marge, considérer comme invalide
  if (range >= 60) {
    return 9999;
  }
  return(range);                                   // Returns Range
}

// Function to get light reading. Always 0x80 in the SRF10. Used to test the existance of the sensor
int ArduBot::getSrfLight(int address)
{
  
  Wire.beginTransmission(address);
  Wire.write(srfLightByte);                        // Call register to get light reading
  Wire.endTransmission();
  
  Wire.requestFrom(address, 1);                    // Request 1 byte
  //while(I2c.available() < 0);                    // While byte available
  int lightRead = Wire.read();                     // Get light reading
    
  return(lightRead);                               // Returns lightRead
  
}

int ArduBot::changeSrfAddress(int oldAddress, int newAddress)
{

  Wire.beginTransmission(oldAddress);
  Wire.write(srfCmdByte);                           // Send Command Byte
  Wire.write(0xA0);
  Wire.endTransmission();


  Wire.beginTransmission(oldAddress);
  Wire.write(srfCmdByte);
  Wire.write(0xAA);
  Wire.endTransmission();


  Wire.beginTransmission(oldAddress);
  Wire.write(srfCmdByte);
  Wire.write(0xA5);
  Wire.endTransmission();


  Wire.beginTransmission(oldAddress);
  Wire.write(srfCmdByte);
  Wire.write(newAddress);
  Wire.endTransmission();
  return 1;

}

int ArduBot::changeSrfGain(int address, int gain)
{
  // Le registre 0x01 ajuste le gain du récepteur (la sensibilité à l’écho de 0 à 31).
  Wire.beginTransmission(address);
  Wire.write(srfGainByte);                             // Send Command Byte
  Wire.write(gain);
  Wire.endTransmission();
  return 1;
}

int ArduBot::changeSrfDistance(int address, int range)
{
  // Le registre 0x02 du SRF10 détermine la portée maximale d’une mesure, exprimée en unités de 43 mm (24 * 43 mm = 1032 mm = ~1,03 m).
  Wire.beginTransmission(address);
  Wire.write(srfRangeByte);                             // Send Command Byte
  Wire.write(range);
  Wire.endTransmission();
  return 1;
}

void ArduBot::testSrfs()
{
  Serial.println("Test SRF..."); 
  int address=0x70;  //Ojo con las direcciones. El LSB se quita y las direcciones aumentan de 1 en 1
  for(byte i=0;i<16;i++)
  {
    if(getSrfLight(address)==0x80)
    {
      SRFs_ADRESS[NUM_SRFs]=address;
      Serial.println(address);
      NUM_SRFs++;
    }
    address++;
  }
  Serial.println(NUM_SRFs);
  Serial.println(NUM_SRFs_FRONT);
  Serial.println(NUM_SRFs_BACK);
  for(int i=0;i<NUM_SRFs;i++)
  {
    Serial.println(SRFs_ADRESS[i]);
  }
  setSrfsRegisters();
}

void ArduBot::setSrfsRegisters()
{
  for(int i=0;i<NUM_SRFs;i++)
  {
    // PARAMS FOR SRF10  DISTANCE & GAIN
    // distance (range) 0=43mm, 1=0.86m, 24=1m, 94=4m
    changeSrfDistance(SRFs_ADRESS[i],24);
    // Gain : 0 et 1=max analog gain de 40, 2=max analog gain de 50, 3=max analog gain de 60...
    changeSrfGain(SRFs_ADRESS[i],6);  //3
  }
}

void ArduBot::filterGyro(float &gx, float &gy, float &gz)
{
    static float alpha = 0.92;
    static float gx_prev = 0, gy_prev = 0, gz_prev = 0;

    gx = alpha * gx_prev + (1 - alpha) * gx;
    gy = alpha * gy_prev + (1 - alpha) * gy;
    gz = alpha * gz_prev + (1 - alpha) * gz;

    gx_prev = gx;
    gy_prev = gy;
    gz_prev = gz;
}

void ArduBot::testLasers()
{
  Serial.println("Test VL53L1X Lasers...");

  int addressesToCheck[MAX_LASERS] = { 0x70, 0x71 };

  for (byte i = 0; i < MAX_LASERS; i++) {
    int addr = addressesToCheck[i];

    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();

    if (error == 0) {
      LASERs_ADRESS[NUM_LASERS] = addr;
      Serial.print("Laser found at address: ");
      Serial.println(addr, HEX);
      NUM_LASERS++;
    } else {
      Serial.print("Laser NOT found at address: ");
      Serial.println(addr, HEX);
    }
  }

  Serial.print("Number of VL53L1X detected: ");
  Serial.println(NUM_LASERS);
  Serial.println(NUM_LASERs_FRONT);
  Serial.println(NUM_LASERs_BACK);
  for(int i=0;i<NUM_LASERS;i++)
  {
    Serial.println(LASERs_ADRESS[i]);
  }
}
// setAmpliVolume(37); // Volume modéré si PC allumé
//----------------------------------------------------------//

//boolean front=true;
byte index_front = 0;
byte index_back = 0;
long accelerometerTime = 0;
long batteryTime = 0;
byte accelerometer_fall_count = 0;
//long batteryIconTime = 0;
// int batteryIconFrame = 0;
int heartFrame = 0; // Utiliser pour l'affichage si le bus I2C plante

void ArduBot::spinOnce() {
  
  //Calculo de la posicion de l robot y procesamiento del PID
  long now = millis();

  if (now - spinTime > ArduBot::spinLoopPeriodMs)
  {
    spinTime += ArduBot::spinLoopPeriodMs;
    static unsigned long lastMicros = micros();
    
    // vince : appel de la machine d'état de calibration non-bloquante (1 échantillon/cycle)
    // Remplace l'ancienne calibrate() qui bloquait loop() pendant ~4 secondes via 1000×delay(2ms)
    // Désormais : 500 échantillons collectés à raison de 1 par cycle de 5ms = ~2.5s sans blocage
    // processSerial() et le PID moteurs continuent de tourner normalement pendant la calibration
    calibrationStep();
    
    unsigned long nowMicros = micros();
    float loopPeriodSeconds = (nowMicros - lastMicros) / 1e6f;
    lastMicros = nowMicros;
    // testIMU();
    // Lire les valeurs brutes des capteurs
    int8_t raw_ax, raw_ay, raw_az;
    int raw_gx, raw_gy, raw_gz;
    gyro.getGyroValues(raw_gx, raw_gy, raw_gz);
    
    accelerometer.getAccelerometerValues(raw_ax, raw_ay, raw_az);
    // Serial.print("ACC composant (X,Y,Z) : ");
    // Serial.print(raw_ax); Serial.print(", ");
    // Serial.print(raw_ay); Serial.print(", ");
    // Serial.println(raw_az);

    // 1. Convertir en float directement pour éviter les erreurs de type/arrondi
    // float accX_raw = static_cast<float>(-raw_ax);
    // float accY_raw = static_cast<float>(-raw_ay);
    // float accZ_raw = static_cast<float>(raw_az);


    // 2) Transformation capteur → robot (même que dans calibrate)
    float tmpAccX = -raw_ax;
    float tmpAccY = -raw_ay;
    float tmpAccZ =  raw_az;
    // Serial.print("ACC robot (X,Y,Z) : ");
    // Serial.print(tmpAccX); Serial.print(", ");
    // Serial.print(tmpAccY); Serial.print(", ");
    // Serial.println(tmpAccZ);

    float tmpGyroX = raw_gy;
    float tmpGyroY = -raw_gx;
    float tmpGyroZ = raw_gz;

    // Serial.print("SENS = "); Serial.println(ACC_SENSITIVITY, 6);
    // 4) // Conversion en g et application de la correction (transformation et offset)
    // Accéléromètre
    accelerometerX = tmpAccX * ACC_SENSITIVITY - accelerometerX0;
    accelerometerY = tmpAccY * ACC_SENSITIVITY - accelerometerY0;
    accelerometerZ = tmpAccZ * ACC_SENSITIVITY - accelerometerZ0;

    // Gyroscope
    gyroX = (tmpGyroX - gyroX0) * gyroSensitivity;
    gyroY = (tmpGyroY - gyroY0) * gyroSensitivity;
    gyroZ = (tmpGyroZ - gyroZ0) * gyroSensitivity;
    // Serial.print("gyroZ (brut) = ");
    // Serial.println((tmpGyroZ - gyroZ0) * gyroSensitivity, 6);
    // 3) Filtrage du gyroscope
    filterGyro(gyroX, gyroY, gyroZ);

    // Logs (optionnels)
    // Serial.println("📊 [IMU DEBUG] Valeurs transformées + offsets :");
    // Serial.print("ACC (X,Y,Z) : ");
    // Serial.print(accelerometerX, 4); Serial.print(", ");
    // Serial.print(accelerometerY, 4); Serial.print(", ");
    // Serial.println(accelerometerZ, 6);
    // Serial.print("Z en float direct = ");
    // Serial.println(accelerometerZ, 6);

    // Serial.print("GYRO (X,Y,Z) : ");
    // Serial.print(gyroX,3); Serial.print(", ");
    // Serial.print(gyroY,3); Serial.print(", ");
    // Serial.println(gyroZ,3);

    // 5) Calcul Roll & Pitch
    if (fabs(accelerometerZ) > 0.1f) {
      roll  = 180.0f * accelerometerY / (accelerometerZ * PI);
      pitch = 180.0f * accelerometerX / (accelerometerZ * PI);
    } else {
      roll = pitch = 90.0f; // ou NAN
    }

    // Serial.println("📊 Angles calculés :");
    // Serial.print("Roll : "); Serial.print(roll); Serial.print("°  |  ");
    // Serial.print("Pitch : "); Serial.print(pitch); Serial.println("°");
    // Serial.println("------------------------------------------");
    static float thetaGyroOnly = 0;
    thetaGyroOnly += gyroZ * loopPeriodSeconds;

    // Serial.print("θ GYRO ONLY (°) = ");
    // Serial.println(thetaGyroOnly);
    // Serial.print("loopPeriodSeconds = ");
    // Serial.println(loopPeriodSeconds, 6);
    // Serial.print("gyroZ = ");
    // Serial.print(gyroZ, 3);
    // Serial.print(" °/s | Δθ = ");
    // Serial.println(gyroZ * loopPeriodSeconds, 4);
    // ArduBot::loopPeriodSeconds = loopPeriodSeconds;  // si tu veux le mémoriser globalement
    updatePosition(loopPeriodSeconds);
  
  }
  if (now - srf10time > 130) //65ms est le temps de mesure d'un SRF10
  {
    srf10time += 130;
    LaserL.read(false);
    LaserR.read(false);
    MiddleDist = getFloorDistance();
    
    if (abs(ArduBot::par_motores.actualLeftWheelPulses)>0||abs(ArduBot::par_motores.actualRightWheelPulses)>0||isSrfUpdateContinuous){
      if (LaserL.ranging_data.range_status == 0 && LaserL.ranging_data.range_mm > 0) {
        LeftDist = LaserL.ranging_data.range_mm/10;
      } else {
        LeftDist = 0;
      }
      if (LaserR.ranging_data.range_status == 0 && LaserR.ranging_data.range_mm > 0) {
        RightDist = LaserR.ranging_data.range_mm/10;
      } else {
        RightDist = 0;
      }
      if (index_back == 0) {
        BackLeftDist=getSrfRange(115);
        updateSrfRange(115);
        index_back = 1;
      }
      else if (index_back == 1) {
        BackRightDist=getSrfRange(114);
        updateSrfRange(114);
        index_back = 0;
      }
      // Serial.print("Left Laser (mm): ");
      // Serial.print(LaserL.ranging_data.range_mm);
      // Serial.print(" => BackLeftDist (cm): ");
      // Serial.println(BackLeftDist);

      // Serial.print("Right Laser (mm): ");
      // Serial.print(LaserR.ranging_data.range_mm);
      // Serial.print(" => BackRightDist (cm): ");
      // Serial.println(BackRightDist);
      
      // Serial.print("Middle Distance: ");
      // Serial.println(MiddleDist);
      // if (ir1_triggered) {
      //   Serial.println("🟢 IR détecté sur IR1 (INT6)");
      //   ir1_triggered = false;
      // }
      // if (ir2_triggered) {
      //   Serial.println("🟢 IR détecté sur IR2 (INT7)");
      //   ir2_triggered = false;
      // }
      // if (ir3_triggered) {
      //   Serial.println("🟢 IR détecté sur IR3 (INT2)");
      //   ir3_triggered = false;
      // }

      // Décodage via IRremote
      if (IrReceiver.decode()) {
        uint32_t code = IrReceiver.decodedIRData.decodedRawData;

        Serial.print("📩 Trame IR reçue : 0x");
        Serial.println(code, HEX);
        lcd.setCursor(4,1);
        lcd.print("        ");
        if (code == IR_ADRESSE_BASE) {
          Serial.println("✅ Base reconnue !");
          if (millis() - lastEvalTime > 5000) { // toutes les 2 sec
            lastEvalTime = millis();

            Serial.print("Score IR1: ");
            Serial.print(ir_score[0]);
            Serial.print(" | IR2: ");
            Serial.print(ir_score[1]);
            Serial.print(" | IR3: ");
            Serial.println(ir_score[2]);

            
            lcd.setCursor(4,1);
            if (ir_score[0] > ir_score[1] && ir_score[0] > ir_score[2]) {
              Serial.println("➡️ Direction dominante : IR1 (gauche)");
              lcd.print("IR1: =>");
            } else if (ir_score[1] > ir_score[0] && ir_score[1] > ir_score[2]) {
              Serial.println("⬆️ Direction dominante : IR2 (centre)");
              lcd.print("IR2: =");
            } else if (ir_score[2] > ir_score[0] && ir_score[2] > ir_score[1]) {
              Serial.println("⬅️ Direction dominante : IR3 (droite)");
              lcd.print("IR3: <=");
            } else {
              Serial.println("❔ Aucune direction dominante claire");
              lcd.print(" ? ");
            }

            // Reset des scores
            ir_score[0] = ir_score[1] = ir_score[2] = 0;
          }
        } else {
          Serial.println("❌ Code inconnu");
        }

        IrReceiver.resume();
      }
    }
//    if (abs(ArduBot::par_motores.actualLeftWheelPulses)>0||abs(ArduBot::par_motores.actualRightWheelPulses)>0||isSrfUpdateContinuous)
//    {
//      if(NUM_SRFs_FRONT>0)
//      {
//      unsigned int srf10Range=getSrfRange(SRFs_FRONT[index_front]);
//      SRFs_VALUES[SRFs_FRONT[index_front]-112]=srf10Range;
//      }
//      if(NUM_SRFs_BACK>0)
//      {
////        Serial.println("ok");
//        SRFs_VALUES[SRFs_BACK[index_back]-112]=getSrfRange(SRFs_BACK[index_back]);
//        index_back=(index_back+1)%NUM_SRFs_BACK;
//        updateSrfRange(SRFs_BACK[index_back]);
//      }
//    }
  }
  if (now - batteryTime > 1000)
  {
    
    lcd.setCursor(0, 2);
    lcd.print("                   ");
    lcd.setCursor(0, 2);
    lcd.print("X");
    lcd.print((int)(xCoordinate * 100)); // en cm

    lcd.setCursor(5, 2);
    lcd.print("Y");
    lcd.print((int)(yCoordinate * 100));

    lcd.setCursor(10, 2);
    lcd.print("T");
    lcd.print((int)(thetaCoordinate * 180.0 / PI)); // en degrés

    lcd.setCursor(15, 2);
    lcd.print("I");
    lcd.print((int)(max(abs(pitch), abs(roll)))); // inclinaison
    batteryTime += 1000;
    
    byte stat = 0, value = 0, charg = 0;
    lcd.setCursor(0, 3);
    lcd.print("Bat: ");

    if (getBatteryLevel(&value, &stat)) {
        float batV = value / 10.0; // Convertir en Volts
        charg = (stat >> 3) & 0x07;

        lcd.print(batV, 1);
        lcd.print("V ");

        displayBatteryIcon(lcd, charg, batV);

        // Serial.print("🔋 Batterie : ");
        // Serial.print(batV);
        // Serial.println("V");
        // Serial.println(charg);
        // 💡 Gestion de l'ampli en fonction de stat bit 1
        bool pcOn = (stat >> 1) & 0x01; // Bit 1 = état du PC
        if (ampliState) {
          if (pcOn) {
            lcd.setCursor(18,3);
            lcd.write(5);
            lcd.write(6);
            setAmpliVolume(30); // Volume modéré si PC allumé
          } 
          else {
            muteAmpli();        // Sinon on mute
            lcd.setCursor(18,3);
            lcd.print("  ");
          }
        }

    } else {
        lcd.print("        "); // Effacer proprement avant d'afficher "No com !"
        lcd.setCursor(0, 3);
        lcd.print("No com !");
        lcd.setCursor(18,3);
        lcd.print("  ");
        Serial.println("⚠️ Impossible de lire le niveau de batterie !");
    }
    // Heartbeat pour Arduino
    lcd.setCursor(16, 3);
    lcd.print((heartFrame == 0) ? "-" : "|");
    heartFrame = !heartFrame;
  }  
}
