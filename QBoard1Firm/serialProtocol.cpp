/*********************************************************************

   Software License Agreement (BSD License)

    Copyright (c) 2011, TheCorpora.
    All rights reserved.

   \author Miguel Angel Julian
 *********************************************************************/

#include "serialProtocol.h"
#include <inttypes.h>
#include <Arduino.h>

using namespace arduBotSerial;
using namespace arduBot;

uint8_t pearsondata[] = {
  0x00, 0x77, 0xee, 0x99, 0x07, 0x70, 0xe9, 0x9e, 0x0e, 0x79, 0xe0, 0x97,
  0x09, 0x7e, 0xe7, 0x90, 0x1d, 0x6a, 0xf3, 0x84, 0x1a, 0x6d, 0xf4, 0x83,
  0x13, 0x64, 0xfd, 0x8a, 0x14, 0x63, 0xfa, 0x8d, 0x3b, 0x4c, 0xd5, 0xa2,
  0x3c, 0x4b, 0xd2, 0xa5, 0x35, 0x42, 0xdb, 0xac, 0x32, 0x45, 0xdc, 0xab,
  0x26, 0x51, 0xc8, 0xbf, 0x21, 0x56, 0xcf, 0xb8, 0x28, 0x5f, 0xc6, 0xb1,
  0x2f, 0x58, 0xc1, 0xb6, 0x76, 0x01, 0x98, 0xef, 0x71, 0x06, 0x9f, 0xe8,
  0x78, 0x0f, 0x96, 0xe1, 0x7f, 0x08, 0x91, 0xe6, 0x6b, 0x1c, 0x85, 0xf2,
  0x6c, 0x1b, 0x82, 0xf5, 0x65, 0x12, 0x8b, 0xfc, 0x62, 0x15, 0x8c, 0xfb,
  0x4d, 0x3a, 0xa3, 0xd4, 0x4a, 0x3d, 0xa4, 0xd3, 0x43, 0x34, 0xad, 0xda,
  0x44, 0x33, 0xaa, 0xdd, 0x50, 0x27, 0xbe, 0xc9, 0x57, 0x20, 0xb9, 0xce,
  0x5e, 0x29, 0xb0, 0xc7, 0x59, 0x2e, 0xb7, 0xc0, 0xed, 0x9a, 0x03, 0x74,
  0xea, 0x9d, 0x04, 0x73, 0xe3, 0x94, 0x0d, 0x7a, 0xe4, 0x93, 0x0a, 0x7d,
  0xf0, 0x87, 0x1e, 0x69, 0xf7, 0x80, 0x19, 0x6e, 0xfe, 0x89, 0x10, 0x67,
  0xf9, 0x8e, 0x17, 0x60, 0xd6, 0xa1, 0x38, 0x4f, 0xd1, 0xa6, 0x3f, 0x48,
  0xd8, 0xaf, 0x36, 0x41, 0xdf, 0xa8, 0x31, 0x46, 0xcb, 0xbc, 0x25, 0x52,
  0xcc, 0xbb, 0x22, 0x55, 0xc5, 0xb2, 0x2b, 0x5c, 0xc2, 0xb5, 0x2c, 0x5b,
  0x9b, 0xec, 0x75, 0x02, 0x9c, 0xeb, 0x72, 0x05, 0x95, 0xe2, 0x7b, 0x0c,
  0x92, 0xe5, 0x7c, 0x0b, 0x86, 0xf1, 0x68, 0x1f, 0x81, 0xf6, 0x6f, 0x18,
  0x88, 0xff, 0x66, 0x11, 0x8f, 0xf8, 0x61, 0x16, 0xa0, 0xd7, 0x4e, 0x39,
  0xa7, 0xd0, 0x49, 0x3e, 0xae, 0xd9, 0x40, 0x37, 0xa9, 0xde, 0x47, 0x30,
  0xbd, 0xca, 0x53, 0x24, 0xba, 0xcd, 0x54, 0x23, 0xb3, 0xc4, 0x5d, 0x2a,
  0xb4, 0xc3, 0x5a, 0x2d
};

uint8_t pearson(uint8_t *key, uint8_t len)
{
  uint8_t hash = 0;
  for (uint8_t i = 0; i < len; i++)
  {
    hash = pearsondata[hash ^ key[i]];
  }
  return (hash);
}

SerialProtocol::SerialProtocol(arduBot::ArduBot *robot) : robot(robot),
  INPUT_FLAG(0xFF), OUTPUT_FLAG(0xFE), INPUT_ESCAPE(0xFD),
  isInputEscaped_(false), isInputCorrect_(true),
  length_(0)  
{
  // deja appele dans ardubot.cpp
  //Serial.begin(500000);
  //Serial.flush();
}

boolean SerialProtocol::procesaEntrada(byte* buf, byte length)
{
  if (length < 5) return false;
  if (buf[0] != INPUT_FLAG) return false;
  if (buf[length - 1] != OUTPUT_FLAG) return false;

  uint8_t check = pearson(buf + 1, length - 3);
  uint8_t inCheck = buf[length - 2];

  if (check != inCheck) return false;
  if (buf[2] != length - 5) return false;
  if (buf[2] > 50) return false;
  command_.commandNumber = buf[1];
  command_.nInputData = length - 5;
  for (int i = 3; i < length - 2; i++)
    command_.inputData[i - 3] = buf[i];
  return true;
}

void SerialProtocol::processSerial()
{
  if (Serial.available() > 0)
  {
    int incoming = Serial.read();
    byte incomingByte = 0;
    if (incoming != -1)
      incomingByte = byte(incoming);
    if (incomingByte == INPUT_FLAG)
    {
      length_ = 0;
      buf_[length_] = incomingByte;
      length_++;
      return;
    }
    else if (incomingByte == OUTPUT_FLAG)
    {
      buf_[length_] = incomingByte;
      length_++;
      if (procesaEntrada(buf_, length_))
      {
        processCommands();
        sendResponse();
      }
      else
      {
        sendNack();
      }
      length_ = 0;
      return;
    }
    if (isInputEscaped_)
    {
      incomingByte += 2;
      isInputEscaped_ = false;
    }
    else if (incomingByte == INPUT_ESCAPE)
    {
      isInputEscaped_ = true;
      return;
    }
    if (length_ > 128)
    {
      length_ = 0;
      return;
    }
    buf_[length_] = incomingByte;
    length_++;
  }
}

void SerialProtocol::processCommands()
{
  isInputCorrect_ = true;
  switch (command_.commandNumber)
  {
    case GET_VERSION:
      if (command_.nInputData != 0)
      {
        isInputCorrect_ = false;
      }
      else
      {
        command_.nOutputData = 2;
        command_.outputData[0] = arduBot::boardId;
        command_.outputData[1] = arduBot::libraryVersion;
      }
      break;

    case GET_BASE_INFRARED:
      if (command_.nInputData != 0)
      {
        isInputCorrect_ = false;
      }
      else
      {
        command_.nOutputData = 3;
        command_.outputData[0] = robot->ir1data;
        command_.outputData[1] = robot->ir2data;
        command_.outputData[2] = robot->ir3data;
        robot->ir1data = 0;
        robot->ir2data = 0;
        robot->ir3data = 0;
      }
      break;
    
    case GET_I2C_STATE:
      if (command_.nInputData != 0)
      {
        isInputCorrect_ = false;
      }
      else
      {
        command_.nOutputData = 1;
        command_.outputData[0] = robot->energyState;
        command_.outputData[0] = (command_.outputData[0] << 1) + robot->lcdState;
        command_.outputData[0] = (command_.outputData[0] << 1) + robot->accelerometerState;
        command_.outputData[0] = (command_.outputData[0] << 1) + robot->gyroState;
        command_.outputData[0] = (command_.outputData[0] << 1) + robot->ampliState;
      }
      break;

    // ✅ CORRECTION : CALIBRATE_IMU non-bloquant
    //
    // AVANT (problème) :
    //   robot->calibrate() bloquait ~4 secondes dans processCommands()
    //   → loop() suspendu pendant toute la calibration :
    //     - processSerial() non appelé → buffer série plein → commandes perdues
    //     - spinOnce() non appelé → PID moteurs non mis à jour → dérive
    //
    // APRÈS (solution — machine d'état à 3 états) :
    //   Premier appel  → calibrateRequest() démarre la collecte, répond 0xFF (en cours)
    //   Appels suivants → calibrateRequest() répond 0xFF tant que CALIB_RUNNING
    //   Quand terminé  → calibrateRequest() répond 1 (succès) ou 0 (échec), reset état
    //
    //   La collecte des échantillons se fait 1 par 1 dans spinOnce()
    //   via calibrationStep(), sans aucun delay().
    //   loop() continue de tourner normalement pendant toute la calibration.
    case CALIBRATE_IMU:
      if (command_.nInputData != 0)
      {
        isInputCorrect_ = false;
      }
      else
      {
        command_.nOutputData = 1;
        command_.outputData[0] = robot->calibrateRequest();
      }
      break;
    
    case RESET_STALL:
      if (command_.nInputData != 0)
      {
        isInputCorrect_ = false;
      }
      else
      {
        command_.nOutputData = 0;
        ArduBot::robotStallFlag = false;
        ArduBot::par_motores.leftStallDetected  = false;
        ArduBot::par_motores.rightStallDetected = false;
        ArduBot::par_motores.leftStallCount     = 0;
        ArduBot::par_motores.rightStallCount    = 0;
      }
      break;

    case GET_MOTORS_STATE:
      if (command_.nInputData != 0)
      {
        isInputCorrect_ = false;
      }
      else
      {
        command_.nOutputData = 1;
        command_.outputData[0]  = ArduBot::par_motores.leftStallDetected  ? 0 : 1;
        command_.outputData[0] |= (ArduBot::par_motores.rightStallDetected ? 0 : 1) << 1;
      }
      break;

    case SET_SPEED:
      if (command_.nInputData != 8)
      {
        isInputCorrect_ = false;
      }
      else
      {
        command_.nOutputData = 0;
        float* p = (float*)command_.inputData;
        robot->setSpeeds(p[0], p[1]);
      }
      break;

    case SET_LCD:
      if (command_.nInputData < 1)
      {
        isInputCorrect_ = false;
      }
      else
      {
        command_.nOutputData = 0;

        // -----------------------------------------------------------------------
        // CORRECTION BUG (Mars 2026) : VLA (Variable Length Array) supprimé
        //
        // PROBLÈME ORIGINAL :
        //   char msg[command_.nInputData + 1];
        //   → VLA : taille calculée à l'exécution, non standard en C++
        //   → Sur ATmega2560 : seulement ~8KB de RAM totale, pile partagée
        //     avec tous les objets statiques et le heap.
        //     Si nInputData est grand ou imprévisible, allocation pile variable
        //     → stack overflow silencieux → corruption mémoire aléatoire.
        //
        // SOLUTION :
        //   Buffer fixe de 129 octets (128 données + '\0').
        //   inputData[] est déjà borné à 128 octets dans CCommand → taille
        //   cohérente et garantie, zéro risque de débordement.
        // -----------------------------------------------------------------------
        char msg[128 + 1];
        for (int i = 0; i < command_.nInputData; i++)
          msg[i] = (char)command_.inputData[i];
        msg[command_.nInputData] = '\0';
        if (robot->lcdState) {
          robot->lcd.home();
          robot->lcd.print(msg);
        }
      }
      break;

    case GET_ALL_SENSORS:
      if (command_.nInputData != 0)
      {
        isInputCorrect_ = false;
      }
      else
      {
        command_.nOutputData = 12;
        byte* LeftIntToBytes  = (byte*)&robot->LeftDist;
        command_.outputData[0] = byte(112);
        command_.outputData[1] = LeftIntToBytes[0];
        command_.outputData[2] = LeftIntToBytes[1];
        byte* RightIntToBytes  = (byte*)&robot->RightDist;
        command_.outputData[3] = byte(113);
        command_.outputData[4] = RightIntToBytes[0];
        command_.outputData[5] = RightIntToBytes[1];
        
        unsigned int rangeLeft = robot->BackLeftDist;
        byte* LeftRearIntToBytes = (byte*)&rangeLeft;
        command_.outputData[6] = byte(114);
        command_.outputData[7] = LeftRearIntToBytes[0];
        command_.outputData[8] = LeftRearIntToBytes[1];

        unsigned int rangeRight = robot->BackRightDist;
        byte* RightRearIntToBytes = (byte*)&rangeRight;
        command_.outputData[9]  = byte(115);
        command_.outputData[10] = RightRearIntToBytes[0];
        command_.outputData[11] = RightRearIntToBytes[1];
      }
      break;

    case SET_AUTOUPDATE_SRFS:
      if (command_.nInputData > 8 || (command_.nInputData % 2) != 0)
      {
        isInputCorrect_ = false;
      }
      else
      {
        command_.nOutputData = 0;
        byte frontSrfCount = 0;
        byte backSrfCount  = 0;
        for (int j = 0; j < (command_.nInputData / 2); j++)
        {
          int  address  = int(*((byte*)(command_.inputData + 2 * j)));
          byte location = (*((byte*)(command_.inputData + 2 * j + 1)));
          isInputCorrect_ = false;
          for (int k = 0; k < robot->NUM_SRFs; k++)
          {
            if (address == robot->SRFs_ADRESS[k])
              isInputCorrect_ = true;
          }
          if (!isInputCorrect_) break;
          if (location == 0) { robot->SRFs_BACK[backSrfCount++]   = address; }
          if (location == 1) { robot->SRFs_FRONT[frontSrfCount++] = address; }
        }
        robot->NUM_SRFs_FRONT = frontSrfCount;
        robot->NUM_SRFs_BACK  = backSrfCount;
      }
      command_.nOutputData = 0;
      break;

    case GET_BATTERY:
      if (command_.nInputData != 0) {
        isInputCorrect_ = false;
      } else {
        command_.nOutputData = 2;
        byte value = 0, stat = 0;
        if (robot->getBatteryLevel(&value, &stat)) {
          command_.outputData[0] = value;
          command_.outputData[1] = stat;
        } else {
          command_.outputData[0] = 0xFF;
          command_.outputData[1] = 0xFF;
        }
      }
      break;

    case GET_ODOMETRY:
      if (command_.nInputData != 0)
      {
        isInputCorrect_ = false;
      }
      else
      {
        command_.nOutputData = 12;
        float xCoordinate, yCoordinate, angle;
        robot->getSpacePosition(xCoordinate, yCoordinate, angle);
        byte* floatToBytes = (byte*)&xCoordinate;
        command_.outputData[0] = floatToBytes[0];
        command_.outputData[1] = floatToBytes[1];
        command_.outputData[2] = floatToBytes[2];
        command_.outputData[3] = floatToBytes[3];
        floatToBytes = (byte*)&yCoordinate;
        command_.outputData[4] = floatToBytes[0];
        command_.outputData[5] = floatToBytes[1];
        command_.outputData[6] = floatToBytes[2];
        command_.outputData[7] = floatToBytes[3];
        floatToBytes = (byte*)&angle;
        command_.outputData[8]  = floatToBytes[0];
        command_.outputData[9]  = floatToBytes[1];
        command_.outputData[10] = floatToBytes[2];
        command_.outputData[11] = floatToBytes[3];
      }
      break;

    case SET_ODOMETRY:
      if (command_.nInputData != 12)
      {
        isInputCorrect_ = false;
      }
      else
      {
        float* p = (float*)command_.inputData;
        robot->setPosition(p[0], p[1], p[2]);
        command_.nOutputData = 0;
      }
      break;

    case GET_ANALOG_PINS:
      if (command_.nInputData < 1 || command_.nInputData > 16)
      {
        isInputCorrect_ = false;
      }
      else
      {
        command_.nOutputData = 2 * command_.nInputData;
        for (byte j = 0; j < command_.nInputData; j++)   // ← nInputData, pas nOutputData
        {
          unsigned int readedValue = 0;
          if (command_.inputData[j] < 16)               // ← byte toujours >= 0, condition simplifiée
            readedValue = robot->adcRead(command_.inputData[j]);
          byte* intToBytes = (byte*)(&readedValue);
          command_.outputData[2 * j]     = intToBytes[0];
          command_.outputData[2 * j + 1] = intToBytes[1];
        }
      }
      break;

    case GET_IMU:
      if (command_.nInputData != 0)
      {
        isInputCorrect_ = false;
      }
      else
      {
        command_.nOutputData = 12;

        int16_t gx = (int16_t)(robot->gyroX * 1000.0f);
        int16_t gy = (int16_t)(robot->gyroY * 1000.0f);
        int16_t gz = (int16_t)(robot->gyroZ * 1000.0f);

        int16_t ax = (int16_t)(robot->accelerometerX * 1000.0f);
        int16_t ay = (int16_t)(robot->accelerometerY * 1000.0f);
        int16_t az = (int16_t)(robot->accelerometerZ * 1000.0f);

        command_.outputData[0]  = gx & 0xFF;
        command_.outputData[1]  = gx >> 8;
        command_.outputData[2]  = gy & 0xFF;
        command_.outputData[3]  = gy >> 8;
        command_.outputData[4]  = gz & 0xFF;
        command_.outputData[5]  = gz >> 8;
        command_.outputData[6]  = ax & 0xFF;
        command_.outputData[7]  = ax >> 8;
        command_.outputData[8]  = ay & 0xFF;
        command_.outputData[9]  = ay >> 8;
        command_.outputData[10] = az & 0xFF;
        command_.outputData[11] = az >> 8;
      }
      break;
  }
}

void SerialProtocol::sendResponse()
{
  if (isInputCorrect_)
  {
    byte temporalDataArray[128];
    byte outputData[128];
    byte nOutputData = 0;
    outputData[nOutputData] = INPUT_FLAG;
    nOutputData++;

    temporalDataArray[0] = command_.commandNumber;
    temporalDataArray[1] = command_.nOutputData;

    for (int i = 0; i < command_.nOutputData; i++)
      temporalDataArray[i + 2] = command_.outputData[i];

    uint8_t check = pearson(temporalDataArray, command_.nOutputData + 2);
    temporalDataArray[command_.nOutputData + 2] = check;

    for (int i = 0; i < command_.nOutputData + 3; i++)
    {
      if (temporalDataArray[i] == INPUT_FLAG || temporalDataArray[i] == INPUT_ESCAPE || temporalDataArray[i] == OUTPUT_FLAG)
      {
        outputData[nOutputData] = INPUT_ESCAPE;
        nOutputData++;
        outputData[nOutputData] = temporalDataArray[i] - 2;
        nOutputData++;
      }
      else
      {
        outputData[nOutputData] = temporalDataArray[i];
        nOutputData++;
      }
    }

    outputData[nOutputData] = OUTPUT_FLAG;
    nOutputData++;
    // vince : Serial.flush() supprimé — il bloquait ~1.7ms par réponse en attendant
    // que le buffer TX soit physiquement vidé sur le bus série
    // L'UART AVR envoie les données en arrière-plan via interruption : flush() inutile ici
    // Gain : ~10% du CPU Arduino récupéré pour le PID et les capteurs
    {
      Serial.write(outputData, nOutputData);
    }
  }
  else
  {
    sendNack();
    isInputCorrect_ = true;
  }
}

void SerialProtocol::sendNack()
{
  byte outputData[2] = {INPUT_FLAG, OUTPUT_FLAG};
  // vince : Serial.flush() supprimé — même raison que dans sendResponse()
  // un NACK est une trame de 2 octets, flush() était particulièrement inutile ici
  {
    Serial.write(outputData, 2);
  }
}
