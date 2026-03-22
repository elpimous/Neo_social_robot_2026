/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, TheCorpora.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TheCorpora nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.

 *
 * \author Miguel Angel Julian
 *********************************************************************/

#define GET_VERSION 0x40
#define SET_ODOMETRY 0x42
// #define SET_MOUTH_VALUE 0x44       //Qboard2
// #define SET_STATE 0x45             //Qboard2
#define GET_BASE_INFRARED 0x46
// #define SET_MIC 0x4A               //Qboard2
// #define GET_MICS 0x4B              //Qboard2
#define SET_LCD 0x4C
#define SET_SPEED 0x4D
#define GET_ALL_SENSORS 0x4E
// #define SET_MOUTH_ANIMATION  0x53  //Qboard2
// #define TEST_MOUTH  0x54           //Qboard2
// #define TEST_MOUTH  0x55           //Qboard2
#define GET_BATTERY 0x57
#define GET_ODOMETRY 0x59
#define CALIBRATE_IMU 0x63
#define SET_AUTOUPDATE_SRFS 0x72
#define GET_ANALOG_PINS 0x73
#define GET_IMU 0x74
#define RESET_STALL 0x80
#define GET_MOTORS_STATE 0x81
#define GET_I2C_STATE 0x82
