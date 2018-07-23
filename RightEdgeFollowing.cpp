 #include "Header.h"
#include <vector>
#include <iostream>
#include  <algorithm>
#include "Aria.h"
#include <thread>

void REFController ::setMembershipValues()
	{
		vector<int> myVec;
		myVec.push_back(0);
		// conditions for leftSenesor
		
		frontSensor.push_back(Memebership(100, 150,200, 400, "close")); //0. close=leftShoulder
		frontSensor.push_back(Memebership(300, 400, 500,700, "average"));//1. average=trapeziod
		frontSensor.push_back(Memebership(530, 1270, 1520, 5030, "far")); //2. far=rightshoulder

		// condition for rightSensor


		backSensor.push_back(Memebership(100, 150, 200, 400, "close")); // 0. close
		backSensor.push_back(Memebership(300, 400,500, 700, "average")); //1. average
		backSensor.push_back(Memebership(500, 1250, 1500, 5050, "far"));// 2.far

		// conditions for wheelVelocity



		wheelVelocity.push_back(Memebership(0, 40, 90, 130, "slow")); // 0 slow=leftShoulder
		wheelVelocity.push_back(Memebership(80, 100, 140, 160, "medium"));//1 medium=trianlge
		wheelVelocity.push_back(Memebership(100, 150, 170, 150, "fast")); // 2 fast=rightShoulder


	}


void REFController:: drawRuleBase()
	{
		this->RIGHTEDGEFOLLOWING.push_back(Rule(&frontSensor[0], &backSensor[0], &wheelVelocity[0], &wheelVelocity[1])); //closeClose slowMedium
		this->RIGHTEDGEFOLLOWING.push_back(Rule(&frontSensor[0], &backSensor[1], &wheelVelocity[0], &wheelVelocity[1])); //closeAverage slowMedium
		this->RIGHTEDGEFOLLOWING.push_back(Rule(&frontSensor[0], &backSensor[2], &wheelVelocity[0], &wheelVelocity[1])); //closeFar slowMedium

		this->RIGHTEDGEFOLLOWING.push_back(Rule(&frontSensor[1], &backSensor[0], &wheelVelocity[0], &wheelVelocity[1])); //averageClose slowMedium
		this->RIGHTEDGEFOLLOWING.push_back(Rule(&frontSensor[1], &backSensor[1], &wheelVelocity[0], &wheelVelocity[1])); //averageAverage slowMedium
		this->RIGHTEDGEFOLLOWING.push_back(Rule(&frontSensor[1], &backSensor[2], &wheelVelocity[2], &wheelVelocity[0])); //averageFar mediumSlow

		this->RIGHTEDGEFOLLOWING.push_back(Rule(&frontSensor[2], &backSensor[0], &wheelVelocity[1], &wheelVelocity[2])); //farClose mediumFast
		this->RIGHTEDGEFOLLOWING.push_back(Rule(&frontSensor[2], &backSensor[1], &wheelVelocity[1], &wheelVelocity[0])); //farAverage mediumSlow
		this->RIGHTEDGEFOLLOWING.push_back(Rule(&frontSensor[2], &backSensor[2], &wheelVelocity[1], &wheelVelocity[0])); //farfar fastSlow 
	}
	
double REFController ::centriodDeFuzzification(vector<Memebership> agg ,double fs[3])
	{
		double fsSum = 0;
		for(int i=0;i<3;i++)
		{
			fsSum += fs[i];
		}
		double fsCenteriodProducts = 0;
		
		for(int i=0;i<3;i++)
		{
			fsCenteriodProducts += agg[i].getCentriod()*fs[i];
		}
		return fsCenteriodProducts / fsSum;
	}

double REFController ::getOutput(double lfs, double lbs, int w)
	{
		double maxFS[3];
		


		
		double out;
		if (w == 1) 
		{
			maxFS[0] = 0, maxFS[1] = 0, maxFS[2] = 0;
			for (int i = 0;i<RIGHTEDGEFOLLOWING.size();i++)
			{
				double fs = RIGHTEDGEFOLLOWING[i].calcFiringStrength(lfs, lbs);
				//cout << "fs = " <<fs;
				if (RIGHTEDGEFOLLOWING[i].leftVelocity->label == "slow") {
					//cout << "LeftVelocity set to slow";
					maxFS[0] = max(maxFS[0], fs);
				}
				else if (RIGHTEDGEFOLLOWING[i].leftVelocity->label == "medium")
				{
					//cout << "LeftVelocity set to medium";
					maxFS[1] = max(maxFS[1], fs);
				}
				else if (RIGHTEDGEFOLLOWING[i].leftVelocity->label == "fast") {
					//cout << "LeftVelocity se to fast";
					maxFS[2] = max(maxFS[2], fs);
				}
			}
			out=centriodDeFuzzification(wheelVelocity, maxFS);
			return  out;
		}
		if (w == 0)

		{
			maxFS[0] = 0, maxFS[1] = 0, maxFS[2] = 0;
			for (int i = 0;i<RIGHTEDGEFOLLOWING.size();i++)
			{
				double fs = RIGHTEDGEFOLLOWING[i].calcFiringStrength(lfs, lbs);
				if (RIGHTEDGEFOLLOWING[i].rightVelocity->label == "slow")
				{
					//cout << "Rightvelocity set to slow";
					maxFS[0] = max(maxFS[0], fs);
				}
				if (RIGHTEDGEFOLLOWING[i].rightVelocity->label == "medium")
				{
					//cout << "RightVelocity set to medium";
					maxFS[1] = max(maxFS[1], fs);
				}
				if (RIGHTEDGEFOLLOWING[i].rightVelocity->label == "fast")
				{
					//cout << "RightVelocity set to fast";
					maxFS[2] = max(maxFS[2], fs);
				}
			}
			out=centriodDeFuzzification(wheelVelocity, maxFS);
			return out;
		}
		

	}

/*void main(int argc, char** argv)
{
	Aria::init();
	ArRobot robot;
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();
	ArRobotConnector robotConnector(&argParser, &robot);
	if (robotConnector.connectRobot())cout << "Robot Connected!" << endl;
	robot.runAsync(false);
	robot.lock();
	robot.enableMotors();
	robot.unlock();
			
	
	cout << "Sonars Connected!" << endl;
	
	ArSensorReading *sonarSensor[8];
	REFController refController;

	refController.setMembershipValues();
	refController.drawRuleBase();
	while (true)
	{
		int  sonarRange[8];
		
		for (int j = 0; j < 8; j++) {
			sonarSensor[j] = robot.getSonarReading(j);
			sonarRange[j] = sonarSensor[j]->getRange();
		}
		double LMS; double RMS;
		LMS = refController.getOutput(sonarRange[6], sonarRange[7], 1);
		RMS = refController.getOutput(sonarRange[6], sonarRange[7], 0);
		cout <<"LEFT SPEED = "<< LMS<<endl;
		cout << "RIGHT SPEED = "<<RMS<<endl;
		robot.setVel2(LMS, RMS);
		this_thread::sleep_for(chrono::milliseconds(10));
	}

	
	robot.lock();
	robot.stop();
	robot.unlock();
	Aria::exit();
}*/


