#include <iostream>
#include <cmath>
#include <unistd.h>
#include "WPILib.h"

const double	SMOOTH_DRIVE_P_GAIN		=	0.5;
const double	SMOOTH_DRIVE_DEADZONE	=	0.01;
const double	ANGLE_TOLERANCE			=	0.1;

const double	TURN_P_GAIN				=	1;
const double	TURN_I_GAIN				=	0.5;
const double	TURN_D_GAIN				=	5;
const double	TURN_K					=	0.001;

double	turnP							=	0;
double	turnI							=	0;
double	turnD							=	0;
double	angleDeviation[64]				=	{0};
int		turnInterval					=	0;

double	drivePower						=	0;
double	drivePowerLeft					=	0;
double	drivePowerRight					=	0;
double	turnPower						=	0;
double	targetAngle						=	0;

class Robot: public IterativeRobot
{
	LiveWindow *lw = LiveWindow::GetInstance();

	SendableChooser *chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "Auto1";

	std::string autoSelected;

	RobotDrive Robotc;
	Joystick driveStick;
	JoystickButton driveThumb;
	JoystickButton driveThumbLU;
	JoystickButton driveThumbRU;
	JoystickButton driveThumbLD;
	JoystickButton driveThumbRD;
	CANTalon driveLeftFront;
	CANTalon driveRightFront;
	CANTalon driveLeftBack;
	CANTalon driveRightBack;
	AnalogGyro gyro;

	JoystickButton JoyR;
	JoystickButton JoyL;
	Relay *AR = new Relay(0);
	Relay *AL = new Relay(1);
	DoubleSolenoid *Piston = new DoubleSolenoid(0, 1);

	const char *JAVA = "/usr/local/frc/JRE/bin/java";
	char *GRIP_ARGS[5] = {"java", "-jar", "/home/lvuser/grip.jar", "/home/lvuser/project.grip", NULL };

public:
	Robot():
		Robotc(1, 2, 3, 4),
		driveStick(0),
		driveThumb( &driveStick , 2 ),
		driveThumbLU( &driveStick , 5 ),
		driveThumbRU( &driveStick , 6 ),
		driveThumbLD( &driveStick , 3 ),
		driveThumbRD( &driveStick , 4 ),
		driveLeftFront(1),
		driveRightFront(2),
		driveLeftBack(3),
		driveRightBack(4),
		gyro(0),
		JoyR(&driveStick,5),
		JoyL(&driveStick,4),
		chooser()
	{}




	void RobotInit()
	{
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		SmartDashboard::PutData("Auto Modes", chooser);

		gyro.Calibrate();
		AR->Set(Relay::Value::kOff);
		AL->Set(Relay::Value::kOff);

		if (fork() == 0)
		{
			if (execv(JAVA, GRIP_ARGS) == -1)
			{
				perror("Error running GRIP");
		    }
		}
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

		autoSelected = *((std::string*)chooser->GetSelected());
		std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;
		if(autoSelected == autoNameCustom){
			//Custom Auto goes here
		} else {
			//Default Auto goes here
		}
	}

	void AutonomousPeriodic()
	{
		if(autoSelected == autoNameCustom){
				//Custom Auto goes here
			} else {
				//Default Auto goes here
			}

		std::cout<<"\ngyro angle =";
		std::cout<<gyro.GetAngle();
		TankDrive(gyro.GetAngle()/90,0);

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

	}

	void TeleopInit()
	{
		AL->Set(Relay::Value::kOn);
		AR->Set(Relay::Value::kOff);


	}

	void TeleopPeriodic()
	{
		std::cout<< "\ngyro angle = ";
		std::cout<< (int) gyro.GetAngle();
		std::cout<< "\tP = ";
		std::cout<< (int) turnP;
		std::cout<< "\tI = ";
		std::cout<< (int) turnI;
		std::cout<< "\tD = ";
		std::cout<< (int) turnD;

		// Stop all movement and reset PID controls
		if ( driveThumb.Get() )
		{
			Drive( 0 , 0 );
			gyro.Reset();

			drivePower		=	0;
			drivePowerLeft	=	0;
			drivePowerRight	=	0;

			turnPower		=	0;
			turnPIDReset();
		}
		// Straight drive
		else if ( driveStick.GetTrigger() )
		{
			KeepAngle( turnPower , driveStick.GetRawAxis(1) );
		}
		// Normal drive
		else
		{
			SmoothTankDrive( driveStick.GetRawAxis(0), driveStick.GetRawAxis(1) );
			turnPIDReset();
		}
//		if ( driveThumbLU.Get() )
//		{
//			targetAngle += 45;
//			while( KeepAngle( targetAngle ) );
//		}
	}

	void TestPeriodic()
	{
		lw->Run();
	}

	void	Drive ( double _left , double _right )
	{
		// set left motors
		driveLeftFront.Set	(_left);
		driveLeftBack.Set	(_left);
		// set right motors
		driveRightFront.Set	(-_right);
		driveRightBack.Set	(-_right);
	}

	void	SmoothDrive ( double _left , double _right )
	{
		if( fabs( _left ) <= SMOOTH_DRIVE_DEADZONE && fabs( _right ) <= SMOOTH_DRIVE_DEADZONE )
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

	void	TankDrive ( double _x , double _y )
	{
		Drive( -_y + _x , -_y - _x );
	}

	void	SmoothTankDrive ( double _x , double _y )
	{
		SmoothDrive( -_y + _x , -_y - _x );
	}

	double	GetAngle ()
	{
		return fmod( gyro.GetAngle() - 180 , 360 ) - 180;
	}

	void	turnPIDReset()
	{
		turnP	=	0;
		turnI	=	0;
		turnD	=	0;
		turnInterval	=	0;
		memset( angleDeviation , 0 , 64 );
	}

	bool	KeepAngle ( double _targetAngle , double _drive )
	{
		// calculate angle deviation
		double _currentAngleDeviation = _targetAngle - GetAngle();

		// reset I
		if ( turnI * _currentAngleDeviation < 0 )
		{
			turnI = 0;
			turnInterval = 0;
			memset( angleDeviation , 0 , 64 );
		}

		// increment interval
		if ( turnInterval < 64 )	turnInterval++;

		// calculate PID
		turnP	=	fmin( _currentAngleDeviation , 90 );
		turnI	+=	( _currentAngleDeviation - angleDeviation[63] );
		turnD	=	-gyro.GetRate();

		// shift angle deviations
		memmove( angleDeviation + 1 , angleDeviation , 63 );

		// store current angle deviation
		angleDeviation[0] = _currentAngleDeviation;

		// calculate turn power
		turnPower	=	TURN_K * ( TURN_P_GAIN * turnP + TURN_I_GAIN * turnI + TURN_D_GAIN * turnD );

		// calculate drive power
		drivePower	+=	SMOOTH_DRIVE_P_GAIN * (	_drive	- 	drivePower	);

		// drive the robot
		TankDrive( turnPower , drivePower );

		// return true if angle is within tolerance
		if ( fabs( _currentAngleDeviation ) <= ANGLE_TOLERANCE ) return true;
		else return false;
	}
};

START_ROBOT_CLASS(Robot)
