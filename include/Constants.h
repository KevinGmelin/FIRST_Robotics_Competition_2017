/*
 * Constants.h
 *
 *  Created on: Feb 16, 2017
 *      Author: Kevin
 */

#ifndef SRC_CONSTANTS_H_
#define SRC_CONSTANTS_H_
#include <string>

	const int frontLeftChannel	     		 	 = 0;
    const int rearLeftChannel	    		 	 = 1;
    const int frontRightChannel	     		 	 = 2;
    const int rearRightChannel	    		 	 = 3;
    const int servoChannel           		 	 = 4;
    const int hangerMotorChannel     		 	 = 5;
    const int intakeMotorChannel     		 	 = 6;
    const int agitatorMotorChannel   		 	 = 7;


    const int frontLeftEncoderA                  = 8;
    const int frontLeftEncoderB                  = 9;
    const int rearRightEncoderA                  = 2;
    const int rearRightEncoderB                  = 3;
    const double encoderDistancePerPulse 	     = 0.314159;


    const int shooterMotorID         			 = 1;



    const int joystickChannel_1	     		 	 = 0;
    const int joystickChannel_2       			 = 1;
    const int joystickChannel_3       			 = 2;

    const int buttonShooter                      = 1; //Press once to turn on,
    												  //press again to turn off
    const int buttonIntakeTowardShooter			 = 2;
    const int buttonIntakeAwayFromShooter        = 3;
    const int buttonStrafeLeft                   = 4;
    const int buttonHangMotorSlowSpeed           = 4;
    const int buttonServoRotateLeft              = 4;
    const int buttonStrafeRight                  = 5;
    const int buttonHangMotorFastSpeed           = 5; //Must be clicking slow speed button
    const int buttonServoRotateRight             = 5;
    const int buttonAgitator                     = 7; //Press once to turn on,
    												  //press again to turn off
    const int buttonAgitatorReverse              = 8;
    const int buttonHastenShooter                = 10;
    const int buttonSlowShooter                  = 11;


    constexpr double intakeTowardShooterSpeed    = 1;
    constexpr double intakeAwayFromShooterSpeed  = -1;
    constexpr double shooterSpeedInitial   	     = -1;
    constexpr double agitatorSpeed    			 = 1;
    constexpr double hangerLowerSpeed			 = 0.4;
    constexpr double hangerUpperSpeed 			 = 1.0;
    constexpr double frontLeftMotorSpeedStrafeL  = -0.936;
    constexpr double frontRightMotorSpeedStrafeL = 0.923;
    constexpr double rearLeftMotorSpeedStrafeL   = 0.988;
    constexpr double rearRightMotorSpeedStrafeL  = -0.975;
    constexpr double frontLeftMotorSpeedStrafeR  = 0.936;
    constexpr double frontRightMotorSpeedStrafeR = -0.923;
    constexpr double rearLeftMotorSpeedStrafeR   = -0.988;
    constexpr double rearRightMotorSpeedStrafeR  = 0.975;
    constexpr double frontLeftMotorSpeedStraight = 0.47;
    constexpr double rearLeftMotorSpeedStraight  = 0.47;
    constexpr double frontRightMotorSpeedStraight = 0.51;
    constexpr double rearRightMotorSpeedStraight = 0.51;

	const std::string autoNameDefaultStraightGear = "StraightGear";
	const std::string autoNameCustomRightSide = "RightSide";
	const std::string autoNameCustom2Far = "Far";
	const std::string autoNameCustom3StraightCorrected = "StraightCorrectedEncoder";
	const std::string autoNameCustom4StraightCorrectedGyro = "StraightCorrectedGyro";
	const std::string autoNameCustom5StraightCorrectedSlow = "StraightCorrectedEncoderSlow";
	const std::string autoNameCustom6StraightCorrectedGyroSlow = "StraightCorrectedGyroSlow";

	const double lowerGearPegx = 370;
	const double upperGearPegx = 380;
	const double middleGearPegx = 375;


#endif /* SRC_CONSTANTS_H_ */
