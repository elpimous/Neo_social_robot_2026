#ifndef motores_h
#define motores_h

#include <inttypes.h>
#include <avr/io.h>
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, TheCorpora.
 *  All rights reserved.
 *
 * \author Daniel Julian
 *********************************************************************/
 /*
=======================================================================
  MODULE MOTEURS + ODOMETRIE + PID update : Vincent FOUCAULT Mars 2026
  ---------------------------------------------------------------------
  - Gestion PWM moteurs
  - Lecture encodeurs (sécurisée via ATOMIC_BLOCK)
  - Conversion pulses → mouvement réel
  - Boucle PID vitesse roues
  - Détection de blocage (stall)
  - Correction d’odométrie (coefficients ajustables)

  IMPORTANT :
  - Toute correction d’odométrie doit être faite sur la MESURE
    (updateMotorsMovement), PAS sur les consignes
=======================================================================*/

namespace arduBot
{

/* ================================================================
   CLASSE CMotors
================================================================ */
class CMotors {
public:
    int pulsesDifference;
    int pwmValue;

    volatile uint8_t *control1Port;
    volatile uint8_t *control2Port;

    byte control1Pin;
    byte control2Pin;
    byte pwmPin;

    void setDirection()
    {
        if (pwmValue < 0)
        {
            bitClear(*control1Port, control1Pin);
            bitSet(*control2Port, control2Pin);
        }
        else
        {
            bitClear(*control2Port, control2Pin);
            bitSet(*control1Port, control1Pin);
        }
    }

    void pushPwm(int newPwmValue)
    {
        pwmValue = newPwmValue;
        setDirection();
        analogWrite(pwmPin, abs(pwmValue));
    }

    int getPulsesDifference()
    {
        int val;

        /*
         PROBLEME :
         - int = 16 bits sur AVR
         - lecture en 2 cycles CPU
         - ISR encodeur peut interrompre entre les deux
         → valeur corrompue aléatoirement

         SOLUTION :
         - section atomique
        */
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            val = pulsesDifference;
            pulsesDifference = 0;
        }

        return val;
    }

    CMotors(volatile uint8_t *control1Register,
            volatile uint8_t *control2Register,
            volatile uint8_t *control1Port,
            volatile uint8_t *control2Port,
            byte control1Pin,
            byte control2Pin,
            byte pwmPin)
        : control1Port(control1Port),
          control2Port(control2Port),
          control1Pin(control1Pin),
          control2Pin(control2Pin),
          pwmPin(pwmPin)
    {
        pinMode(pwmPin, OUTPUT);

        bitSet(*control1Register, control1Pin);
        bitSet(*control2Register, control2Pin);
    }
};

/* ================================================================
   CLASSE CBaseMovement
================================================================ */
class CBaseMovement {
public:

    /* ===== MOTEURS ===== */
    CMotors *leftMotor;
    CMotors *rightMotor;

    /* ===== CONSIGNES ===== */
    double desiredLinearSpeed;
    double desiredAngularSpeed;

    /* ===== MESURES ===== */
    double actualLinearMovement;
    double actualAngularMovement;

    /* ===== CONVERSIONS ===== */
    double desiredLinearPulses;
    double desiredAngularPulses;

    double referenceLinearPulses;
    double referenceAngularPulses;

    double referenceLeftWheelPulses;
    double referenceRightWheelPulses;

    double actualLeftWheelPulses;
    double actualRightWheelPulses;

    /* ===== PID ===== */
    double linearError;
    double angularError;

    double leftError;
    double rightError;

    double integralLeftError;
    double integralRightError;

    int leftMotorPwm;
    int rightMotorPwm;

    double kp, kd, ki;
    double sampleTime;

    /* ===== PHYSIQUE ===== */
    int encoderResolution;
    double distanceBetweenWheels;
    double wheelRadius;

    double linearPulse2LinearMovementConstant;
    double angularPulse2AngularMovementConstant;

    /* ===== CORRECTION ODOMETRIE ===== */
    double linearOdomCoeff;
    double angularOdomCoeff;

    /* ===== ACCELERATION ===== */
    double maxLinearAcceleration;
    double maxAngularAcceleration;

    /* ===== STALL ===== */
    uint16_t leftStallCount;
    uint16_t rightStallCount;

    bool leftStallDetected;
    bool rightStallDetected;

    double minStallDetectionTime;
    double minStallStopSecurityTime;

    byte variablesInitialized;

    /* ================================================================
    CLASSE CBaseMovement
    ---------------------------------------------------------------
    - Cerveau du robot mobile
    - Convertit vitesse → pulses
    - Applique PID
    - Met à jour position
    ================================================================ */
    CBaseMovement(CMotors *leftMotor,
                  CMotors *rightMotor,
                  double kp = 0,
                  double kd = 0,
                  double ki = 0,
                  double sampleTime = 0.005,
                  int encoderResolution = 1440,
                  double distanceBetweenWheels = 0.2736,
                  double wheelRadius = 0.102)
        : leftMotor(leftMotor),
          rightMotor(rightMotor),
          kp(kp),
          kd(kd),
          ki(ki),
          sampleTime(sampleTime),
          encoderResolution(encoderResolution),
          distanceBetweenWheels(distanceBetweenWheels),
          wheelRadius(wheelRadius)
    {
        desiredLinearSpeed = 0;
        desiredAngularSpeed = 0;

        actualLinearMovement = 0;
        actualAngularMovement = 0;

        /* IMPORTANT : PID */
        linearError = 0.0;
        angularError = 0.0;

        integralLeftError = 0;
        integralRightError = 0;

        leftError = 0;
        rightError = 0;

        /* CORRECTION ODOMETRIE */
        linearOdomCoeff = 1.0;
        angularOdomCoeff = 1.0;

        /* ACCELERATION */
        maxLinearAcceleration = 1;
        maxAngularAcceleration = 3;

        /* STALL */
        leftStallCount = 0;
        rightStallCount = 0;

        leftStallDetected = false;
        rightStallDetected = false;

        minStallDetectionTime = 2.0;
        minStallStopSecurityTime = 5;

        if (encoderResolution == 0)
        {
            variablesInitialized = 0;
            return;
        }

        variablesInitialized = 1;

        linearPulse2LinearMovementConstant =
            PI * wheelRadius / encoderResolution;

        angularPulse2AngularMovementConstant =
            2 * PI * wheelRadius / (distanceBetweenWheels * encoderResolution);
    }

    /* ============================================================
       INIT PHYSIQUE
    ============================================================ */
    void initializePhisicalVariables(int _encoderResolution,
                                     double _distanceBetweenWheels,
                                     double _wheelRadius)
    {
        encoderResolution = _encoderResolution;
        distanceBetweenWheels = _distanceBetweenWheels;
        wheelRadius = _wheelRadius;

        if (encoderResolution == 0)
        {
            variablesInitialized = 0;
            return;
        }

        variablesInitialized = 1;

        linearPulse2LinearMovementConstant =
            PI * wheelRadius / encoderResolution;

        angularPulse2AngularMovementConstant =
            2 * PI * wheelRadius / (distanceBetweenWheels * encoderResolution);
    }

    /* ============================================================
       INIT PID
    ============================================================ */
    void initializePIDVariables(double _kp,
                                double _kd,
                                double _ki,
                                double _sampleTime)
    {
        kp = _kp;
        kd = _kd;
        ki = _ki;
        sampleTime = _sampleTime;
    }

    /* ============================================================
       VITESSE → PULSES
    ============================================================ */
    void transformSpeeds2Pulses()
    {
        if (!variablesInitialized) return;

        desiredLinearPulses =
            desiredLinearSpeed * sampleTime / linearPulse2LinearMovementConstant;

        desiredAngularPulses =
            desiredAngularSpeed * sampleTime / angularPulse2AngularMovementConstant;
    }

    /* ============================================================
       LIMITATION ACCELERATION
    ============================================================ */
    void limitAcceleration()
    {
        double gap = desiredLinearPulses - referenceLinearPulses;

        if (abs(gap) < maxLinearAcceleration)
            referenceLinearPulses = desiredLinearPulses;
        else
            referenceLinearPulses += (gap > 0 ? maxLinearAcceleration : -maxLinearAcceleration);

        gap = desiredAngularPulses - referenceAngularPulses;

        if (abs(gap) < maxAngularAcceleration)
            referenceAngularPulses = desiredAngularPulses;
        else
            referenceAngularPulses += (gap > 0 ? maxAngularAcceleration : -maxAngularAcceleration);
    }

    /* ============================================================
       REPARTITION
    ============================================================ */
    void linearPlusAngular2LeftPlusRight()
    {
        referenceRightWheelPulses =
            (referenceLinearPulses + referenceAngularPulses) / 2;

        referenceLeftWheelPulses =
            (referenceLinearPulses - referenceAngularPulses) / 2;
    }

    /* ============================================================
       ODOMETRIE (AVEC CORRECTION)
    ============================================================ */
    void updateMotorsMovement()
    {
        actualLeftWheelPulses  = leftMotor->getPulsesDifference();
        actualRightWheelPulses = rightMotor->getPulsesDifference();

        actualLinearMovement =
            (actualLeftWheelPulses + actualRightWheelPulses)
            * linearPulse2LinearMovementConstant
            * linearOdomCoeff;

        actualAngularMovement =
            (actualRightWheelPulses - actualLeftWheelPulses)
            * angularPulse2AngularMovementConstant
            * angularOdomCoeff;
    }

    /* ============================================================
       PID
    ============================================================ */
    void doPIDControl()
    {
    // -----------------------------------------------------------------------
    // PID DISCRET — vitesse par roue (gauche et droite indépendants)
    //
    // Convention : error = reference - actual
    //   → error > 0 : roue trop lente → PWM positif → on accélère   ✓
    //   → error < 0 : roue trop rapide → PWM négatif → on freine     ✓
    //
    // ⚠️  CORRECTION BUG #1 (Mars 2026) : signe était actual - reference
    //     → rétroaction POSITIVE : trop lent → PWM négatif → recul → divergence
    //
    // ⚠️  CORRECTION BUG #9 (Mars 2026) : intégrale accumulait l'erreur
    //     du cycle N-1 au lieu du cycle N (lag d'un cycle).
    //     Ordre correct : 1) sauver old, 2) calculer new, 3) intégrer new
    //
    // Termes PID :
    //   P = kp × error                   → réaction proportionnelle à l'écart
    //   I = ki × Σerror                  → corrige l'erreur statique résiduelle
    //   D = kd × (error - oldError)      → amortit les oscillations
    //
    // ⚠️  Sans anti-windup : si le robot est bloqué mécaniquement,
    //     l'intégrale grossit sans limite → overshoots au déblocage.
    //     → Ajouter constrain(integralLeftError, -MAX_I, MAX_I) si nécessaire.
    // -----------------------------------------------------------------------

    // 1. Sauvegarder l'erreur du cycle précédent (pour le terme dérivé)
    double oldLeftError  = leftError;
    double oldRightError = rightError;

    // 2. Calculer la nouvelle erreur (cycle courant)
    leftError  = referenceLeftWheelPulses  - actualLeftWheelPulses;
    rightError = referenceRightWheelPulses - actualRightWheelPulses;

    // 3. Intégrer l'erreur courante (pas l'ancienne)
    integralLeftError  += leftError;
    integralRightError += rightError;

    // 4. Terme dérivé : variation de l'erreur entre deux cycles
    double dLeft  = leftError  - oldLeftError;
    double dRight = rightError - oldRightError;

    // 5. Calcul PWM final
    leftMotorPwm  = kp * leftError  + kd * dLeft  + ki * integralLeftError;
    rightMotorPwm = kp * rightError + kd * dRight + ki * integralRightError;
    }

    /* ============================================================
       LIMITES PWM
    ============================================================ */
    int checkMotorsLimits()
    {
        int limitStatus = 0;

        if ((abs(leftMotorPwm) > 255) || (abs(rightMotorPwm) > 255))
        {
            int maxMotorPwm = max(abs(leftMotorPwm), abs(rightMotorPwm));

            leftMotorPwm  = (leftMotorPwm  * 255) / maxMotorPwm;
            rightMotorPwm = (rightMotorPwm * 255) / maxMotorPwm;

            limitStatus = 1;
        }

        return limitStatus;
    }

    /* ============================================================
       PWM
    ============================================================ */
    void updateMotorsPwm()
    {
        leftMotor->pushPwm(leftMotorPwm);
        rightMotor->pushPwm(rightMotorPwm);
    }

    /* ============================================================
       STOP
    ============================================================ */
    void freeMotors()
    {
        desiredLinearSpeed = 0;
        desiredAngularSpeed = 0;

        integralLeftError = 0;
        integralRightError = 0;

        leftError = 0;
        rightError = 0;

        leftMotor->pushPwm(0);
        rightMotor->pushPwm(0);
    }

    /* ============================================================
       BOUCLE PRINCIPALE
    ============================================================ */
    bool doControlLoop()
    {
        if (!variablesInitialized) return false; 

        updateMotorsMovement();

        if ((desiredLinearSpeed == 0) && (desiredAngularSpeed == 0))
        {
            freeMotors();
            return false; // valeur de retour `limitStatus` ignorée, et `doControlLoop()` retourne toujours `false`
        }

        transformSpeeds2Pulses();
        limitAcceleration();
        linearPlusAngular2LeftPlusRight();
        doPIDControl();
        checkMotorsLimits();
        updateMotorsPwm();

        return false;
    }
};

} // namespace

#endif