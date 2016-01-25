#include "WPILib.h"
#include <iostream>

class Robot: public IterativeRobot
{
public:
//	LiveWindow *lw = LiveWindow::GetInstance();
//	SendableChooser *chooser;
//	const std::string autoNameDefault = "Default";
//	const std::string autoNameCustom = "My Auto";
//	std::string autoSelected;
	std::shared_ptr<NetworkTable> table;

	Robot()
	{
		table = NetworkTable::GetTable("GRIP/myContoursReport");

	}

	void RobotInit()
	{
//		chooser = new SendableChooser();
//		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
//		chooser->AddObject(autoNameCustom, (void*)&autoNameCustom);
//		SmartDashboard::PutData("Auto Modes", chooser);


		while(true)
		{
			std::cout << "Areas: ";
			std ::vector<double> arr = table->GetNumberArray("area", llvm::ArrayRef<double>());
			for (unsigned int i = 0 ; i < arr.size() ; i++)
			{
				std::cout << arr[i] << " ";
			}
			std::cout << std::endl;
			Wait(1);
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
//		autoSelected = *((std::string*)chooser->GetSelected());
//		//std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
//		std::cout << "Auto selected: " << autoSelected << std::endl;
//
//		if(autoSelected == autoNameCustom){
//			//Custom Auto goes here
//		} else {
//			//Default Auto goes here
//		}
	}

	void AutonomousPeriodic()
	{
//		if(autoSelected == autoNameCustom){
//			//Custom Auto goes here
//		} else {
//			//Default Auto goes here
//		}
	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{

	}

	void TestPeriodic()
	{
//		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
