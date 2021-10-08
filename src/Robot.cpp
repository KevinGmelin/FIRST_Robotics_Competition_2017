#include "WPILib.h"
#include "CanTalon.h"
#include "Constants.h"
#include "JoystickToggle.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

using namespace cv;
using namespace std;

double centerGear = -1;
bool noCenterGear = false;

void negateBool(bool *b)
{
	*b = !(*b);
	return;
}

void increment(double *n)
{
	*n += 0.01;
	return;
}

void decrement(double *n)
{
	*n -= 0.01;
}

bool enableThread(true);


class Robot: public SampleRobot
{
private:
	//Joysticks
	Joystick stick;
	Joystick stick2;
	Joystick stick3;

	//Motors
    Spark frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor;
	RobotDrive robotDrive;
	CanTalonSRX shooterMotor;
	Victor hangerMotor; // May need to change speed controllers on these, be aware!!!
	Talon intakeMotor;
	Talon agitatorMotor;


	//Sensors
	ADXRS450_Gyro gyro;
	Encoder frontLeftEncoder, rearRightEncoder;



	double speedUpLeft; //Speed up left side to go straight
    bool agitatorOn;
    bool shooterOn;
    bool reverseAgitator;

	JoystickToggle<bool> toggleAgitator, toggleShooter;

	SendableChooser<std::string> chooser;

	void resetSensors()
	{
		frontLeftEncoder.Reset();
		rearRightEncoder.Reset();
		gyro.Reset();
	}

	void resetValues()
	{
		speedUpLeft = 0; //Speed up to go straight
	    agitatorOn = false;
	    shooterOn = false;
	    reverseAgitator = false;
	}

	void zeroMotors()
	{
		frontLeftMotor.Set( 0);
		rearLeftMotor.Set(0 );
		frontRightMotor.Set( 0 );
		rearRightMotor.Set(0);
	}

	void driveStraight(double distance)
	{
		resetSensors();
		while(frontLeftEncoder.GetDistance() < distance)
		{

			frontLeftMotor.Set( 0.5 );
			rearLeftMotor.Set( 0.5 );
			frontRightMotor.Set( 0.5 );
			rearRightMotor.Set(0.5);

		}

		frontLeftMotor.Set(0);
		rearLeftMotor.Set(0);
		frontRightMotor.Set(0);
		rearRightMotor.Set(0);
	}

    static void VisionThread()
    {
   		int iLowH = 60;
    	int iHighH = 95;
    	int iLowS = 0;
    	int iHighS = 255;
    	int iLowV = 30;
    	int iHighV = 100;

    	std::vector<std::vector<Point>> contours;
    	std::vector<Vec4i> hierarchy;


    	cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
    	camera.SetExposureManual(1);
    	camera.SetResolution(640, 480);
    	camera.SetBrightness(50);

        cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
        cv::Mat source, imgHSV, imgThresholded, drawing;
        while(true)
        {
        	if(enableThread)
        	{
                cvSink.GrabFrame(source);
                cvtColor(source, imgHSV, cv::COLOR_BGR2HSV);
                cv::inRange(imgHSV,cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded);



                int erosion_size = 5;
                Mat element = getStructuringElement(cv::MORPH_CROSS,
                		cv::Size(2*erosion_size +1, 2 * erosion_size + 1),
        				cv::Point(erosion_size, erosion_size) );
                dilate(imgThresholded, imgThresholded, element);

                findContours(imgThresholded,contours, hierarchy,
                		CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

                std::vector<std::vector<Point>>hull(contours.size());

                for(int i = 0; i < contours.size(); i++)
                {
                	convexHull(Mat(contours[i]), hull[i], false);
                }

                int biggestContourIdx = -1;
                float biggestContourArea = 0;
                cv::Scalar color = cv::Scalar(0, 100, 0);
                drawing = cv::Mat::zeros( imgThresholded.size(), CV_8UC3 );


                for( int i = 0; i< contours.size(); i++ )
                {
                	drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, cv::Point() );

                	float ctArea= cv::contourArea(contours[i]);
                	if(ctArea > biggestContourArea)
                		{
                            biggestContourArea = ctArea;
                            biggestContourIdx = i;
                		}
                }

                int secondBiggestContourIdx = -1;
                float secondBiggestContourArea = 0;
                for( int i = 0; i< contours.size(); i++ )
                {
                    if(i != biggestContourIdx)
                    {
                    	float ctArea= cv::contourArea(contours[i]);
                    	if(ctArea > secondBiggestContourArea)
                    	{
                    		secondBiggestContourArea = ctArea;
                    		secondBiggestContourIdx = i;
                    	}
                    }
                }

                    	cv::Point2f corners[4];
                    	cv::Point2f corners2[4];

                    // draw the rotated rect

                    if(biggestContourIdx != -1)
                    {
                    	cv::RotatedRect boundingBox = cv::minAreaRect(contours[biggestContourIdx]);
                    	boundingBox.points(corners);
                    }

                    if(secondBiggestContourIdx != -1)
                    {
                    	cv::RotatedRect boundingBox2 = cv::minAreaRect(contours[secondBiggestContourIdx]);

                    	boundingBox2.points(corners2);
                    }

                    if(biggestContourIdx != -1 && secondBiggestContourIdx != -1)
                    {
                        double lowerx = corners[0].x;
                        for(int i = 0; i<4; i++)
                        {
                        	if(corners[i].x<lowerx && corners[i].x>0)
                        	{
                        		lowerx = corners[i].x;
                        	}
                        }
                        for(int i = 0; i<4; i++)
                        {
                        	if(corners2[i].x<lowerx && corners2[i].x >0)
                        	{
                        		lowerx = corners2[i].x;
                        	}
                        }

                        double upperx = 0;
                        for(int i = 0; i<4; i++)
                        {
                        	if(corners[i].x>upperx && corners[i].x>0)
                        	{
                        		upperx = corners[i].x;
                        	}
                        }
                        for(int i = 0; i<4; i++)
                        {
                        	if(corners2[i].x>upperx)
                        	{
                        		upperx = corners2[i].x;
                        	}
                        }
                        SmartDashboard::PutNumber("corner0x", corners[0].x);
                        SmartDashboard::PutNumber("corner1x", corners[1].x);
                        SmartDashboard::PutNumber("corner2x", corners[2].x);
                        SmartDashboard::PutNumber("corner3x", corners[3].x);

                        SmartDashboard::PutNumber("corner0x2", corners2[0].x);
                        SmartDashboard::PutNumber("corner1x2", corners2[1].x);
                        SmartDashboard::PutNumber("corner2x2", corners2[2].x);
                        SmartDashboard::PutNumber("corner3x2", corners2[3].x);

                         SmartDashboard::PutNumber("lowerx", lowerx);
                         SmartDashboard::PutNumber("upperx", upperx);
                         SmartDashboard::PutNumber("centerGear", centerGear);

                         SmartDashboard::PutBoolean("noCenterGear", noCenterGear);
                         if((upperx + lowerx)/2.0 > 200 && (upperx + lowerx)/2.0 < 505)
                         {
                             centerGear = (upperx + lowerx)/2.0;
                             noCenterGear = false;
                         }
                         else
                         {
                        	 noCenterGear = true;
                         }

                    }
                    else
                    {
                    	noCenterGear = true;
                    }
        	}
        }
    }

public:
	Robot() :
			stick(joystickChannel_1),
			stick2(joystickChannel_2),
			stick3(joystickChannel_3),
			frontLeftMotor(frontLeftChannel),
			rearLeftMotor(rearLeftChannel),
			frontRightMotor(frontRightChannel),
			rearRightMotor(rearRightChannel),
			robotDrive(frontLeftMotor, rearLeftMotor,
					   frontRightMotor, rearRightMotor),	// these must be initialized in the same order
			shooterMotor(shooterMotorID),
			hangerMotor(hangerMotorChannel),
			intakeMotor(intakeMotorChannel),
			agitatorMotor(agitatorMotorChannel),
			gyro(),
			frontLeftEncoder(8 , 9, false, Encoder::k4X),
			rearRightEncoder(2, 3, true, Encoder::k4X),
			speedUpLeft(0),
			agitatorOn(false),
			shooterOn(false),
			reverseAgitator(false),
			toggleAgitator(&stick3, buttonAgitator, &agitatorOn, negateBool),
			toggleShooter(&stick3, buttonShooter, &shooterOn, negateBool)
	{
		robotDrive.SetExpiration(0.1);
		robotDrive.SetInvertedMotor(RobotDrive::kRearRightMotor, true);	// invert the left side motors
		robotDrive.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);	// you may need to change or remove this to match robot
		frontRightMotor.SetInverted(true);
		rearRightMotor.SetInverted(true);
		gyro.Calibrate();
		std::thread visionThread(VisionThread);
		visionThread.detach();
		frontLeftEncoder.SetDistancePerPulse(encoderDistancePerPulse);
		rearRightEncoder.SetDistancePerPulse(encoderDistancePerPulse);
		chooser.AddDefault(autoNameDefaultStraightGear, autoNameDefaultStraightGear);
		chooser.AddObject(autoNameCustomRightSide, autoNameCustomRightSide);
		chooser.AddObject(autoNameCustom2Far, autoNameCustom2Far);
		chooser.AddObject(autoNameCustom3StraightCorrected,
				autoNameCustom3StraightCorrected);
		chooser.AddObject(autoNameCustom4StraightCorrectedGyro,
				autoNameCustom4StraightCorrectedGyro);
		SmartDashboard::PutData("Auto Modes", &chooser);
	}

	void Autonomous()
	{
		resetSensors();
		resetValues();
		auto autoSelected = chooser.GetSelected();
		enableThread = true;
		if (autoSelected == autoNameCustomRightSide)
		{
			std::cout << "Running Right Side" << std::endl;
			resetSensors();

			while(frontLeftEncoder.GetDistance() < 87)
			{
				frontLeftMotor.Set( 0.5 );
				rearLeftMotor.Set( 0.5 );
				frontRightMotor.Set( 0.5 );
				rearRightMotor.Set(0.5);
			}

			frontLeftMotor.Set(0);
			rearLeftMotor.Set(0);
			frontRightMotor.Set(0);
			rearRightMotor.Set(0);

			resetSensors();
			while(gyro.GetAngle() > -30)
			{
				frontLeftMotor.Set(- 0.3 );
				rearLeftMotor.Set(- 0.3 );
				frontRightMotor.Set( 0.5 );
				rearRightMotor.Set(0.5);

			}

			resetSensors();
			while(frontLeftEncoder.GetDistance() < 160)
			{
				frontLeftMotor.Set( 0.5 );
				rearLeftMotor.Set( 0.5 );
				frontRightMotor.Set( 0.5 );
				rearRightMotor.Set(0.5);
			}

		}
		else if(autoSelected == autoNameCustom2Far)
		{
			std::cout << "Running Drive Far" << std::endl;
			resetSensors();
			while(frontLeftEncoder.GetDistance() < 160)
			{

				frontLeftMotor.Set( 0.461 );
				rearLeftMotor.Set( 0.461 );
				frontRightMotor.Set( 0.5 );
				rearRightMotor.Set(0.5);

			}

			frontLeftMotor.Set(0);
			rearLeftMotor.Set(0);
			frontRightMotor.Set(0);
			rearRightMotor.Set(0);

		}
		else if(autoSelected == autoNameCustom3StraightCorrected)
		{
			std::cout << "Running straight gear" << std::endl;
			resetSensors();
			resetValues();
			while(frontLeftEncoder.GetDistance() < 110)
			{
				speedUpLeft += 0.000007*(rearRightEncoder.GetDistance()- frontLeftEncoder.GetDistance());

				frontLeftMotor.Set( 0.461 + speedUpLeft );
				rearLeftMotor.Set( 0.461 + speedUpLeft );
				frontRightMotor.Set( 0.5 );
				rearRightMotor.Set(0.5);
				SmartDashboard::PutNumber("FrontLeftEncoder", frontLeftEncoder.GetDistance());
				SmartDashboard::PutNumber("rearRightEncoder", rearRightEncoder.GetDistance());
				SmartDashboard::PutNumber("FrontLeftEncoder - rearleft", frontLeftEncoder.GetDistance()-rearRightEncoder.GetDistance());

				SmartDashboard::PutNumber("Gyro", gyro.GetAngle());
				SmartDashboard::PutNumber("speedUpLeft", speedUpLeft);

			}
			while(frontLeftEncoder.GetDistance() < 115)
			{
				speedUpLeft += 0.000007*(rearRightEncoder.GetDistance()- frontLeftEncoder.GetDistance());

				frontLeftMotor.Set( 0.361 + speedUpLeft );
				rearLeftMotor.Set( 0.361 + speedUpLeft );
				frontRightMotor.Set( 0.4 );
				rearRightMotor.Set(0.4);
				SmartDashboard::PutNumber("FrontLeftEncoder", frontLeftEncoder.GetDistance());
				SmartDashboard::PutNumber("rearRightEncoder", rearRightEncoder.GetDistance());
				SmartDashboard::PutNumber("FrontLeftEncoder - rearleft", frontLeftEncoder.GetDistance()-rearRightEncoder.GetDistance());

				SmartDashboard::PutNumber("Gyro", gyro.GetAngle());
				SmartDashboard::PutNumber("speedUpLeft", speedUpLeft);

			}
			frontLeftMotor.Set(0);
			rearLeftMotor.Set(0);
			frontRightMotor.Set(0);
			rearRightMotor.Set(0);
		}
		else if(autoSelected == autoNameCustom4StraightCorrectedGyro)
		{
			std::cout << "Running straight gear" << std::endl;
			resetSensors();
			resetValues();
			while(frontLeftEncoder.GetDistance() < 30)
			{
				speedUpLeft += 0.000007*(rearRightEncoder.GetDistance()- frontLeftEncoder.GetDistance());

				frontLeftMotor.Set( 0.461 + speedUpLeft );
				rearLeftMotor.Set( 0.461 + speedUpLeft );
				frontRightMotor.Set( 0.5 );
				rearRightMotor.Set(0.5);
				SmartDashboard::PutNumber("FrontLeftEncoder", frontLeftEncoder.GetDistance());
				SmartDashboard::PutNumber("rearRightEncoder", rearRightEncoder.GetDistance());
				SmartDashboard::PutNumber("FrontLeftEncoder - rearright", frontLeftEncoder.GetDistance()-rearRightEncoder.GetDistance());

				SmartDashboard::PutNumber("Gyro", gyro.GetAngle());
				SmartDashboard::PutNumber("speedUpLeft", speedUpLeft);

			}
			zeroMotors();
			Wait(2);
			double error;

			if(centerGear != -1 && !noCenterGear)
			{
				while (centerGear < lowerGearPegx && centerGear > upperGearPegx)
				{

					error = middleGearPegx - centerGear;
					SmartDashboard::PutNumber("CenterGear", centerGear);
					if(error >= 0)
					{
						frontLeftMotor.Set( -0.2 );
						rearLeftMotor.Set( -0.2 );
						frontRightMotor.Set( 0.2 );
						rearRightMotor.Set(0.2);
					}
					else
					{
						frontLeftMotor.Set( 0.2 );
						rearLeftMotor.Set( 0.2 );
						frontRightMotor.Set( -0.2 );
						rearRightMotor.Set(-0.2);
					}
				}
			}
			else
			{
				for(int counter = 0; (noCenterGear) && counter<100; counter++)
				{
					if(centerGear != -1 && !noCenterGear)
					{
						while (centerGear < lowerGearPegx && centerGear > upperGearPegx)
						{

							error = middleGearPegx - centerGear;

							if(error >= 0)
							{
								frontLeftMotor.Set( -0.2 );
								rearLeftMotor.Set( -0.2 );
								frontRightMotor.Set( 0.2 );
								rearRightMotor.Set(0.2);
							}
							else
							{
								frontLeftMotor.Set( 0.2 );
								rearLeftMotor.Set( 0.2 );
								frontRightMotor.Set( -0.2 );
								rearRightMotor.Set(-0.2);
							}
						}
					}
				}
			}
			zeroMotors();
			resetSensors();
			resetValues();
			Wait(2);
			while(frontLeftEncoder.GetDistance() < 100 - 30)
			{
				speedUpLeft += 0.000007*(rearRightEncoder.GetDistance()- frontLeftEncoder.GetDistance());

				frontLeftMotor.Set( 0.461 + speedUpLeft );
				rearLeftMotor.Set( 0.461 + speedUpLeft );
				frontRightMotor.Set( 0.5 );
				rearRightMotor.Set(0.5);
				SmartDashboard::PutNumber("FrontLeftEncoder", frontLeftEncoder.GetDistance());
				SmartDashboard::PutNumber("rearRightEncoder", rearRightEncoder.GetDistance());
				SmartDashboard::PutNumber("FrontLeftEncoder - rearright", frontLeftEncoder.GetDistance()-rearRightEncoder.GetDistance());

				SmartDashboard::PutNumber("Gyro", gyro.GetAngle());
				SmartDashboard::PutNumber("speedUpLeft", speedUpLeft);
			}

			while(frontLeftEncoder.GetDistance() < 110 - 30)
			{
				speedUpLeft += 0.000007*(rearRightEncoder.GetDistance()- frontLeftEncoder.GetDistance());

				frontLeftMotor.Set( 0.361 + speedUpLeft );
				rearLeftMotor.Set( 0.361 + speedUpLeft );
				frontRightMotor.Set( 0.4 );
				rearRightMotor.Set(0.4);
				SmartDashboard::PutNumber("FrontLeftEncoder", frontLeftEncoder.GetDistance());
				SmartDashboard::PutNumber("rearRightEncoder", rearRightEncoder.GetDistance());
				SmartDashboard::PutNumber("FrontLeftEncoder - rearright", frontLeftEncoder.GetDistance()-rearRightEncoder.GetDistance());

				SmartDashboard::PutNumber("Gyro", gyro.GetAngle());
				SmartDashboard::PutNumber("speedUpLeft", speedUpLeft);

			}
			zeroMotors();
		}
		else
		{
			// Default Auto goes here
			std::cout << "Running straight gear" << std::endl;
			resetSensors();
			while(frontLeftEncoder.GetDistance() < 110)
			{

				frontLeftMotor.Set( 0.461 );
				rearLeftMotor.Set( 0.461 );
				frontRightMotor.Set( 0.5 );
				rearRightMotor.Set(0.5);
				SmartDashboard::PutNumber("FrontLeftEncoder", frontLeftEncoder.GetDistance());
				SmartDashboard::PutNumber("rearRightEncoder", rearRightEncoder.GetDistance());
				SmartDashboard::PutNumber("FrontLeftEncoder - rearleft", frontLeftEncoder.GetDistance()-rearRightEncoder.GetDistance());
				SmartDashboard::PutNumber("Gyro", gyro.GetAngle());
				SmartDashboard::PutNumber("speedUpLeft", speedUpLeft);

			}

			frontLeftMotor.Set(0);
			rearLeftMotor.Set(0);
			frontRightMotor.Set(0);
			rearRightMotor.Set(0);
		}

	}

	void OperatorInit()
	{
		agitatorOn = false;
		shooterOn = false;
		robotDrive.SetSafetyEnabled(true);
		resetSensors();
		enableThread = false;
	}

	void OperatorControl()
	{
        if( IsOperatorControl() && IsEnabled())
        {
            while (IsOperatorControl() && IsEnabled())
            {
                SmartDashboard::PutNumber("Gyro", gyro.GetAngle());

                if(stick.GetRawButton(buttonStrafeLeft))
                {
                    frontLeftMotor.Set(frontLeftMotorSpeedStrafeL);
                    rearLeftMotor.Set(rearLeftMotorSpeedStrafeL);
                    frontRightMotor.Set(frontRightMotorSpeedStrafeL);
                    rearRightMotor.Set(rearRightMotorSpeedStrafeL);
                }
                else if(stick.GetRawButton(buttonStrafeRight))
                {
                    frontLeftMotor.Set(frontLeftMotorSpeedStrafeR);
                    rearLeftMotor.Set(rearLeftMotorSpeedStrafeR);
                    frontRightMotor.Set(frontRightMotorSpeedStrafeR);
                    rearRightMotor.Set(rearRightMotorSpeedStrafeR);
                }
                else
                {
                    robotDrive.MecanumDrive_Cartesian(stick.GetX(), stick.GetY(), stick2.GetX());
                }

                if(stick3.GetRawButton(buttonHangMotorSlowSpeed) && stick3.GetRawButton(buttonHangMotorFastSpeed))
                {
                    hangerMotor.Set(hangerUpperSpeed);
                }
                else if(stick3.GetRawButton(buttonHangMotorSlowSpeed))
                {
                    hangerMotor.Set(hangerLowerSpeed);
                }
                else
                {
                    hangerMotor.Set(0);
                }

                if(stick3.GetRawButton(buttonIntakeTowardShooter))
                {
                    intakeMotor.Set(intakeTowardShooterSpeed);
                }
                else if(stick3.GetRawButton(buttonIntakeAwayFromShooter))
                {
                    intakeMotor.Set(intakeAwayFromShooterSpeed);
                }
                else
                {
                    intakeMotor.Set(0);
                }

                toggleAgitator.Run();

                reverseAgitator = stick3.GetRawButton(buttonAgitatorReverse);

                if(agitatorOn & reverseAgitator)
                {
                    agitatorMotor.Set(-agitatorSpeed);
                }
                else if(agitatorOn & !reverseAgitator)
                {
                    agitatorMotor.Set(agitatorSpeed);
                }
                else
                {
                    agitatorMotor.Set(0);
                }

                SmartDashboard::PutBoolean("agitatorOn", agitatorOn);

                toggleShooter.Run();


                if(shooterOn)
                {
                    shooterMotor.Set(shooterSpeedInitial);
                }
                else
                {
                    shooterMotor.Set(0);
                }

                SmartDashboard::PutBoolean("shooterrOn", shooterOn);

            }
        }
	}
};

START_ROBOT_CLASS(Robot)
