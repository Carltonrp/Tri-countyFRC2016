#include <iostream>
#include <cmath>
#include <unistd.h>
#include "WPILib.h"

const double	SMOOTH_DRIVE_P_GAIN		=	0.5;
const double	SMOOTH_DRIVE_DEADZONE	=	0.01;
const double	ANGLE_TOLERANCE			=	0.1;

const double	DRIVE_P_GAIN			=	0.5;
const double	DRIVE_I_GAIN			=	0.1;
const double	DRIVE_D_GAIN			=	0.2;
const double	DRIVE_K					=	1.0;

const double	TURN_P_GAIN				=	1;
const double	TURN_I_GAIN				=	0.5;
const double	TURN_D_GAIN				=	6;
const double	TURN_K					=	0.001;

double speedLeft						=	0;
double speedRight						=	0;

double	driveP							=	0;
double	driveI							=	0;
double	driveD							=	0;

double	turnP							=	0;
double	turnI							=	0;
double	turnD							=	0;
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
	const std::string autoNameCustom = "Auto2";
	const std::string autoNameCustom = "Auto3";
	const std::string autoNameCustom = "Auto4";
	const std::string autoNameCustom = "Auto5";

	std::string autoSelected;

	Timer timer;
	RobotDrive Robotc;
	Joystick driveStick;
	JoystickButton driveThumb;
	JoystickButton driveThumbLU;
	JoystickButton driveThumbRU;
	JoystickButton driveThumbLD;
	JoystickButton driveThumbRD;
	CANTalon driveLeft;
	CANTalon driveRight;
	CANTalon arm;
	AnalogGyro gyro;
	AnalogAccelerometer accelLeft;
	AnalogAccelerometer accelRight;

	JoystickButton JoyR;
	JoystickButton JoyL;
	Relay *AR = new Relay(0);
	Relay *AL = new Relay(1);
	DoubleSolenoid *Piston = new DoubleSolenoid(0, 1);

	const char *JAVA = "/usr/local/frc/JRE/bin/java";
	char *GRIP_ARGS[5] = {"java", "-jar", "/home/lvuser/grip.jar", "/home/lvuser/project.grip", NULL };

public:
	Robot():
		Robotc(1, 2),
		driveStick(0),
		driveThumb( &driveStick , 2 ),
		driveThumbLU( &driveStick , 5 ),
		driveThumbRU( &driveStick , 6 ),
		driveThumbLD( &driveStick , 3 ),
		driveThumbRD( &driveStick , 4 ),
		driveLeft(1),
		driveRight(2),
		arm(3),
		gyro(0),
		accelLeft(1),
		accelRight(2),
		JoyR(&driveStick,5),
		JoyL(&driveStick,4),
		chooser()
	{}


	void RobotInit()
	{
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
		SmartDashboard::PutData("Auto Modes", chooser);

		gyro.Calibrate();						//Setting the gyroscope to zero wherever it is
		AR->Set(Relay::Value::kOff);
		AL->Set(Relay::Value::kOff);

		if (fork() == 0)						//Creating process for grip and testing if it fails
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

		std::cout<< "\nangle = ";
		std::cout<< gyro.GetAngle();

	}

	void TeleopInit()
	{
		AL->Set(Relay::Value::kOn);
		AR->Set(Relay::Value::kOff);

	}

	void TeleopPeriodic()
	{
		std::cout<< "\nangle = ";
		std::cout<< gyro.GetAngle();
//		std::cout<< "\tP = ";
//		std::cout<< (int) turnP;
//		std::cout<< "\tI = ";
//		std::cout<< (int) turnI;
//		std::cout<< "\tD = ";
//		std::cout<< (int) turnD;

		// Stop all movement and reset PID controls
		if ( driveThumb.Get() )
		{
			Drive( 0 , 0 );
			gyro.Reset();

			drivePower		=	0;
			drivePowerLeft	=	0;
			drivePowerRight	=	0;

			turnPower		=	0;
			TurnPIDReset();
		}

		std::cout<< "\nTarget Angle: ";
		std::cout<< (int) targetAngle;
		if ( driveStick.GetPOV() != -1 ) {
			targetAngle = ModAngle( -driveStick.GetPOV() );
		}

		// Straight drive
		else if ( driveStick.GetTrigger() )
		{
			KeepAngle( targetAngle , driveStick.GetRawAxis(1) );
		}
		// Normal drive
		else
		{
			SmoothTankDrive( driveStick.GetRawAxis(0), driveStick.GetRawAxis(1) );
			TurnPIDReset();
		}
		if (driveThumbLU.Get())
		{
			arm.Set(0.2);
		}
		else if (driveThumbLU.Get())
		{
			arm.Set(-0.2);
		}
		else
		{
			arm.Set(0);
		}


	}

	void TestPeriodic()
	{
		lw->Run();
	}

	/* DRIVE FUNCTIONS */

	void	Drive ( double _left , double _right )
		{
			// set left motors
			driveLeft.Set	(_left);
			// set right motors
			driveRight.Set	(-_right);
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
			Drive( _y + _x , _y - _x );
		}

		void	SmoothTankDrive ( double _x , double _y )
		{
			SmoothDrive( _y + _x , _y - _x );
		}

		double	GetSpeed ()
		{
			speedLeft	+=	accelLeft.GetAcceleration() / timer.Get();
			speedRight	+=	accelRight.GetAcceleration() / timer.Get();
			timer.Reset();
			return	( speedLeft + speedRight ) / 2;
		}

		double	GetSpeedLeft ()
		{
			GetSpeed();
			return	speedLeft;
		}

		double	GetSpeedRight ()
		{
			GetSpeed();
			return	speedRight;
		}

		void	PIDTankDrive ( double _x , double _y ) {
			double	_currentY			=	GetSpeed();
			double	_currentX			=	( speedLeft - speedRight ) / _currentY;
			double	_currentDeviation	=	_currentX - _x;
			driveP	=	_currentDeviation;
			driveI	+=	_currentDeviation;
			driveD	=	-_currentX;
			turnPower = DRIVE_P_GAIN * driveP + DRIVE_I_GAIN * driveI + DRIVE_D_GAIN * driveD;
			Drive( turnPower , _y );
		}

		/* GYRO FUNCTIONS */

		double	ModAngle ( double angle )
		{
			angle = angle - 360 * floorf( ( angle - 180 ) / 360 ) - 360;
			if ( angle == -180 ) angle = 180;
			return angle;
		}

		double	GetAngle ()
		{
			return ModAngle( gyro.GetAngle() );
		}

		double	AngularDifference ( double left , double right )
		{
			std::cout<<"\n";
			std::cout<<left;
			std::cout<<"-";
			std::cout<<right;
			std::cout<<"=";
			std::cout<<ModAngle( left - right );
			return ModAngle( left - right );
		}

		void	TurnPIDReset()
		{
			turnP			=	0;
			turnI			=	0;
			turnD			=	0;
			turnInterval	=	0;
		}

		bool	KeepAngle ( double _targetAngle , double _drive )
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

			// return true if angle is within tolerance
			if ( fabs( _currentAngleDeviation ) <= ANGLE_TOLERANCE ) return true;
			else return false;
		}
};

START_ROBOT_CLASS(Robot)
