#include <iostream>
#include <cmath>
#include <unistd.h>
#include "WPILib.h"
#include "Timer.h"

const double	SMOOTH_DRIVE_P_GAIN		=	0.5;
const double	DRIVE_DEADZONE			=	0.05;
const double	DRIVE_X_TOLERANCE		=	0.05;
const double	ANGLE_TOLERANCE			=	0.1;

const double	TURN_P_GAIN				=	1;
const double	TURN_I_GAIN				=	0.5;
const double	TURN_D_GAIN				=	6;
const double	TURN_K					=	0.001;

const double	PITCH_P_GAIN			=	1;
const double	PITCH_I_GAIN			=	0.5;
const double	PITCH_D_GAIN			=	6;
const double	PITCH_K					=	0.001;

double	ACCEL_CALIBRATION				=	0;

double	speedLeft						=	0;
double	speedRight						=	0;

bool	driveStraight					=	false;

double	turnP							=	0;
double	turnI							=	0;
double	turnD							=	0;
double	turnInterval					=	0;

double	pitch							=	0;
double	pitchSpeed						=	0;
double	pitchP							=	0;
double	pitchI							=	0;
double	pitchD							=	0;

double	drivePower						=	0;
double	drivePowerLeft					=	0;
double	drivePowerRight					=	0;
double	turnPower						=	0;
double	targetAngle						=	0;

int		autoDriveState					=	0;
double	acceleration					=	0;
double	speed							=	0;
double	distance						=	0;
bool	tracking						=	false;

double times;

double accelX;
double accelY;
double accelZ;

class Robot: public IterativeRobot
{
	LiveWindow	*lw = LiveWindow::GetInstance();

	SendableChooser	*autoChooser;
	SendableChooser	*teleChooser;
	const std::string	autoNameDefault = "Default";

	const std::string	autoNameCustom0 = "Auto0";
	const std::string	autoNameCustom1 = "Auto1";
	const std::string	autoNameCustom2 = "Auto2";
	const std::string	autoNameCustom3 = "Auto3";
	const std::string	autoNameCustom4 = "Auto4";

	std::string autoSelected;

	const std::string teleNameDefault = "BothSticks";

	const std::string teleNameCustom0 = "SingleStick";

	std::string teleSelected;

	Timer timer;
//	RobotDrive Robotc;
	Joystick driveStick;
	JoystickButton driveThumb;
	JoystickButton driverB5;
	JoystickButton driverB6;
	JoystickButton driverB3;
	JoystickButton driverB4;

	Joystick operatorStick;
	JoystickButton	operatorThumb;
	JoystickButton operatorB5;
	JoystickButton operatorB6;
	JoystickButton operatorB3;
	JoystickButton operatorB4;

	CANTalon driveLeft;
	CANTalon driveRight;
	CANTalon arm;
	CANTalon throwLow;
	CANTalon throwHigh;
	AnalogGyro gyro;
	ADXL345_I2C accel;

	JoystickButton JoyL;
	JoystickButton JoyR;

	Relay *AR = new Relay(0);
	Relay *AL = new Relay(1);
	DoubleSolenoid *Piston = new DoubleSolenoid(0, 1);

//	const char *JAVA = "/usr/local/frc/JRE/bin/java";
//	char *GRIP_ARGS[5] = {"java", "-jar", "/home/lvuser/grip.jar", "/home/lvuser/project.grip", NULL };

public:
	Robot():
//		Robotc(1, 2),
		driveStick(0),
		driveThumb( &driveStick , 2 ),
		driverB5( &driveStick , 5 ),
		driverB6( &driveStick , 6 ),
		driverB3( &driveStick , 3 ),
		driverB4( &driveStick , 4 ),

		operatorStick(1),
		operatorThumb( &operatorStick , 2 ),
		operatorB5( &operatorStick , 5 ),
		operatorB6( &operatorStick , 6 ),
		operatorB3( &operatorStick , 3 ),
		operatorB4( &operatorStick , 4 ),

		driveLeft(1),
		driveRight(2),
		arm(4),
		throwHigh(3),
		throwLow(5),
		gyro(0),
		JoyL(&driveStick,4),
		JoyR(&driveStick,5),
		autoChooser(),
		teleChooser(),
		accel(I2C::Port::kOnboard)
	{}




	void RobotInit()
	{
		autoChooser = new SendableChooser();
		autoChooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		autoChooser->AddObject(autoNameCustom0, (void*)&autoNameCustom0);
		autoChooser->AddObject(autoNameCustom1, (void*)&autoNameCustom1);
		autoChooser->AddObject(autoNameCustom2, (void*)&autoNameCustom2);
		autoChooser->AddObject(autoNameCustom3, (void*)&autoNameCustom3);
		autoChooser->AddObject(autoNameCustom4, (void*)&autoNameCustom4);

		SmartDashboard::PutData("Auto Modes", autoChooser);

		teleChooser = new SendableChooser();
		teleChooser->AddDefault(teleNameDefault, (void*)&teleNameDefault);
		teleChooser->AddObject(teleNameCustom0, (void*)&teleNameCustom0);

		SmartDashboard::PutData("Tele Modes", teleChooser);

		AR->Set(Relay::Value::kOff);
		AL->Set(Relay::Value::kOff);

//		if (fork() == 0)						//Creating process for grip and testing if it fails
//		{
//			if (execv(JAVA, GRIP_ARGS) == -1)
//			{
//				perror("Error running GRIP");
//		    }
//		}

		timer.Start();

		for ( int n = 0 ; n < 256 ; n++ )
		{
			Wait(0.1);
			ACCEL_CALIBRATION	+=	accel.GetY();
		}
		ACCEL_CALIBRATION	/=	256;

		gyro.Calibrate();
	}


	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the GetString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the if-else structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit()
	{
		Piston->Set(DoubleSolenoid::Value::kOff);
		AR->Set(Relay::Value::kOn);
		AL->Set(Relay::Value::kOff);

		autoSelected = *((std::string*)autoChooser->GetSelected());
		std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;
		if(autoSelected == autoNameCustom0)
		{
			KillAll();
		}
		else if(autoSelected == autoNameCustom1)
		{
			KillAll();
		}
		else if(autoSelected == autoNameCustom2)
		{
			KillAll();
		}
		else if(autoSelected == autoNameCustom3)
		{
			KillAll();
		}
		else if(autoSelected == autoNameCustom4)
		{
			KillAll();
		}
		else
		{
			KillAll();
		}
	}

	void AutonomousPeriodic()
	{
		times = timer.Get();
		if(autoSelected == autoNameCustom0)
		{
			AutoDrive( 200 , -.2 );
		}
		else if(autoSelected == autoNameCustom1)
		{
			KillAll();
		}
		else if(autoSelected == autoNameCustom2)
		{
			KillAll();
		}
		else if(autoSelected == autoNameCustom3)
		{
			KillAll();
		}
		else if(autoSelected == autoNameCustom4)
		{
			KillAll();
		}
		else
		{
				//Default Auto goes here
//			std::cout<<"/n Time =";
//			std::cout<<timer.Get();
//
//			if (times <= 5)
//			{
//				driveLeft.Set(0.2);
//				driveRight.Set(-0.2);
//			}
//			else if ((times <= 10) && (times > 5))
//			{
//				driveLeft.Set(-0.2);
//				driveRight.Set(0.2);
//			}
//			else
//			{
//				timer.Reset();
//				timer.Start();
//			}


			std::cout<<"\n cal = ";
			std::cout<<ACCEL_CALIBRATION;
			std::cout<<"\t accel = ";
			std::cout<<accel.GetY();
			std::cout<<"\t speed = ";
			std::cout<<speed;
			std::cout<<"\t dist = ";
			std::cout<<distance;

			TrackAccel();
		}

//		TankDrive(gyro.GetAngle()/90,0);

		auto grip = NetworkTable::GetTable("grip");

		        /* Get published values from GRIP using NetworkTables */
		auto areas = grip->GetNumberArray("targets/area", llvm::ArrayRef<double>());

		for (auto area : areas)
		{
			std::cout << "Got contour with area=" << area << std::endl;
		}

//		Piston->Set(DoubleSolenoid::Value::kForward);
//		Wait(3);
//		Piston->Set(DoubleSolenoid::Value::kReverse);
//		Wait(3);
//		std::cout << "Piston ";
//
//		std::cout<< "\nangle = ";
//		std::cout<< gyro.GetAngle();
//		std::cout<< "\trate = ";
//		std::cout<< gyro.GetRate();

		TrackAccel();	// must be called at the end of the periodic loop
	}

	void TeleopInit()
	{
		AL->Set(Relay::Value::kOn);
		AR->Set(Relay::Value::kOff);
		gyro.Calibrate();

		teleSelected = *((std::string*)teleChooser->GetSelected());
		std::string TeleSelected = SmartDashboard::GetString("Tele Selector", teleNameDefault);
		std::cout << "Tele selected: " << teleSelected << std::endl;
	}

	void TeleopPeriodic()
	{
//		std::cout<< "\ndist = ";
//		std::cout<< distance;
//		std::cout<< "\tspeed = ";
//		std::cout<< speed;
//		std::cout<< "\taccel = ";
//		std::cout<< accel.GetY();

//		std::cout<< "\nangle = ";
//		std::cout<< gyro.GetAngle();
//		std::cout<< "\tP = ";
//		std::cout<< (int) turnP;
//		std::cout<< "\tI = ";
//		std::cout<< (int) turnI;
//		std::cout<< "\tD = ";
//		std::cout<< (int) turnD;

		// Stop all movement and reset PID controls3

		if (teleSelected == teleNameCustom0) 	//Single Stick Debug Tele
		{
			if ( driveThumb.Get() )

			{
					KillDrive();
			}
			else
			{
				TankDrive( driveStick.GetRawAxis(0) , driveStick.GetRawAxis(1) );
			}
			if ( driveStick.GetPOV() != -1 )
			{
				targetAngle = ModAngle( -driveStick.GetPOV() );
			}

			if (driverB5.Get())
			{
				arm.Set(0.5);
			}
			else if (driverB6.Get())
			{
				arm.Set(-0.5);
			}
			else
			{
				arm.Set(0);
			}
			if (driverB3.Get())
			{
				throwHigh.Set((driveStick.GetRawAxis(3)+1)/2);
				throwLow.Set((driveStick.GetRawAxis(3)+1)/2);
			}
			else if (operatorB4.Get())
			{
				throwHigh.Set(-(driveStick.GetRawAxis(3)+1)/2);
				throwLow.Set(-(driveStick.GetRawAxis(3)+1)/2);
			}
			else
			{
				throwHigh.Set(0);
				throwLow.Set(0);
			}

			TrackAccel();	// must be called at the end of the periodic loop
		}
		else			//Default Tele Code "Both Sticks"
		{
			if ( driveThumb.Get() )

			{
				KillDrive();
			}
			else
			{
				SmoothTankDrive( driveStick.GetRawAxis(0) , driveStick.GetRawAxis(1) );
			}
			if ( driveStick.GetPOV() != -1 )
			{
				targetAngle = ModAngle( -driveStick.GetPOV() );
			}
			if (operatorB5.Get())
			{
				arm.Set(0.5);
			}
			else if (operatorB6.Get())
			{
				arm.Set(-0.5);
			}
			else
			{
				arm.Set(0);
			}
			if (operatorB3.Get())
			{
				throwHigh.Set((operatorStick.GetRawAxis(3)+1)/2);
				throwLow.Set((operatorStick.GetRawAxis(3)+1)/2);
			}
			else if (operatorB4.Get())
			{
				throwHigh.Set(-(operatorStick.GetRawAxis(3)+1)/2);
				throwLow.Set(-(operatorStick.GetRawAxis(3)+1)/2);
			}
			else
			{
				throwHigh.Set(0);
				throwLow.Set(0);
			}

				TrackAccel();	// must be called at the end of the periodic loop
			}
	}

	void TestPeriodic()
	{
		lw->Run();
	}

	/* Kill Functions */

	/*
	 * Unconditionally stop all motors and reset control variables.
	 */
	void	KillAll ()
	{
		driveLeft.Set	(	0	);
		driveRight.Set	(	0	);
		drivePowerLeft	=	0;
		drivePowerRight	=	0;
		drivePower		=	0;
		turnPower		=	0;
		TurnPIDReset();
	}

	/*
	 * Unconditionally stop all drive motors and reset drive control variables.
	 */
	void	KillDrive ()
	{
		driveLeft.Set	(	0	);
		driveRight.Set	(	0	);
		drivePowerLeft	=	0;
		drivePowerRight	=	0;
		drivePower		=	0;
		turnPower		=	0;
		TurnPIDReset();

		turnPower		=	0;
		TurnPIDReset();

	}

	/* DRIVE FUNCTIONS */

	void	Drive	( double _left , double _right )
	{
		// set left motors
		driveLeft.Set	( -_left );
		// set right motors
		driveRight.Set	( _right );
	}

	void	SmoothDrive	( double _left , double _right )
	{
		if( fabs( _left ) <= DRIVE_DEADZONE && fabs( _right ) <= DRIVE_DEADZONE )
		{
			drivePowerLeft	= 0;
			drivePowerRight	= 0;
		}
		else
		{
			drivePowerLeft	+= SMOOTH_DRIVE_P_GAIN * (	_left	- 	drivePowerLeft	);
			drivePowerRight	+= SMOOTH_DRIVE_P_GAIN * (	_right	- 	drivePowerRight	);
		}
		Drive( drivePowerLeft , drivePowerRight );
	}

	void	TankDrive	( double _x , double _y )
	{
		Drive( _y - _x , _y + _x );
	}

	void	SmoothTankDrive	( double _x , double _y )
	{
		SmoothDrive( _y - _x , _y + _x );
	}

	void	KeepAngle	( double _targetAngle , double _drive )
	{
		// calculate angle deviation
		double _currentAngleDeviation = AngularDifference( GetAngle() , _targetAngle );

		// increment interval
		turnInterval++;

		// calculate PID
		turnP	=	_currentAngleDeviation;
		turnI	+=	_currentAngleDeviation;
		turnD	=	-gyro.GetRate();

		// calculate turn power
		turnPower	=	TURN_K * ( TURN_P_GAIN * turnP + TURN_I_GAIN * turnI + TURN_D_GAIN * turnD );

		// limit turnPower to [-1,+1]
		if		( turnPower > +1 )	turnPower	=	+1;
		else if	( turnPower < -1 )	turnPower	=	-1;

		// calculate drive power
		drivePower	+=	SMOOTH_DRIVE_P_GAIN * (	_drive	- 	drivePower	);

		// drive the robot
		TankDrive( turnPower , drivePower );
	}

	void	SpecialTankDrive	( double _x , double _y )
	{
		if ( fabs( _x ) <= DRIVE_X_TOLERANCE || driveStick.GetTrigger() )
		{
			std::cout<<"\nSTRAIGHT DRIVE!";
			if ( !driveStraight )
			{
				gyro.Reset();
				TurnPIDReset();
			}
			if ( fabs( _x ) <= DRIVE_DEADZONE && fabs( _y ) <= DRIVE_DEADZONE )
			{
				driveStraight = false;
				Drive( 0 , 0 );
			}
			else
			{

			driveStraight = true;
			KeepAngle( 0 , _y );

				driveStraight = true;
				KeepAngle( 0 , _y );
			}
		}
		else
		{
			std::cout<<"\nSMOOTH DRIVE!";
			driveStraight = false;
			SmoothDrive( _y - _x , _y + _x );
		}
	}

	/*
	 * Automatically drive straight for a specified distance.
	 */
	bool	AutoDrive	( double _targetDistance , double _speed = 0.5 )
	{
		if ( autoDriveState == 0 )
		{
			gyro.Reset();
			TurnPIDReset();
			DistanceReset();
			autoDriveState = 1;
		}
		if ( autoDriveState == 1 )
		{
			TrackAccel();
			double	distanceRemaining	=	_targetDistance - distance;
			//
			if ( distanceRemaining < 0 )
			{
				// if facing correct angle,
				if ( fabs( GetAngle() ) <= ANGLE_TOLERANCE )
				{
					// drive straight
					KeepAngle( 0 , _speed );
				}
				else
				{
					// stop to correct angle
					KeepAngle( 0 , 0 );
				}
			}
			else if ( distanceRemaining > 0 )
			{
				// if facing correct angle,
				if ( fabs( GetAngle() ) <= ANGLE_TOLERANCE )
				{
					// drive straight
					KeepAngle( 0 , _speed );
				}
				else
				{
					// stop to correct angle
					KeepAngle( 0 , 0 );
				}
			}
			else
			{
				autoDriveState = 2;
			}
		}
		if ( autoDriveState == 2 )
		{
			// if not facing the correct angle
			if ( fabs( GetAngle() ) > ANGLE_TOLERANCE )
			{
				// turn to face the correct angle
				KeepAngle( 0 , 0 );
			}
			else
			{
				autoDriveState = -1;
			}
		}
		if ( autoDriveState == -1 )
		{
			autoDriveState = 0;
			return true;
		}
		else return false;
	}

	/* GYRO FUNCTIONS */

	double	ModAngle	( double angle )
	{
		angle = angle - 360 * floorf( ( angle - 180 ) / 360 ) - 360;
		if ( angle == -180 ) angle = 180;
		return angle;
	}

	double	GetAngle	()
	{
		return ModAngle( gyro.GetAngle() );
	}

	double	AngularDifference	( double left , double right )
	{
		std::cout<<"\n";
		std::cout<<left;
		std::cout<<"-";
		std::cout<<right;
		std::cout<<"=";
		std::cout<<ModAngle( left - right );
		return ModAngle( left - right );
	}

	void	TurnPIDReset	()
	{
		turnP			=	0;
		turnI			=	0;
		turnD			=	0;
		turnInterval	=	0;
	}

	/* ACCELEROMETER FUNCTIONS */
	void	DistanceReset	()
	{
		distance	=	0;
	}

	void	TrackAccel	()
	{
		if ( tracking )
		{
			double _time = timer.Get();
			acceleration	=	accel.GetY() - ACCEL_CALIBRATION;
			speed			+=	_time * acceleration;
			distance		+=	_time * speed;
			timer.Reset();
		}
		tracking = true;
	}
	/* THROWER CONTROL FUNCTIONS */
	void	setPitch	( double _target )
	{
		if ( _target >= 0 && _target <= 80 )
		{
			double _deviation = pitch - _target;
			pitchP	=	_deviation;
			pitchI	+=	_deviation;
			pitchD	=	pitchSpeed;
			arm.Set( PITCH_K * ( PITCH_P_GAIN * pitchP + PITCH_I_GAIN * pitchI + PITCH_D_GAIN * pitchD ) );
		}
		else
		{
			arm.Set( 0 );
			std::cout<<"error in 'setPitch': target out of range";
		}
	}
};

START_ROBOT_CLASS(Robot)
