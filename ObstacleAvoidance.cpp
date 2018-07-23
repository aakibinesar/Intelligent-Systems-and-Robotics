#include "Header.h"
#include <math.h>
#include <algorithm>
#include <Aria.h>

void ObstacleAvoidance::setMemershipValues()
	{
		//LeftFrontSensor MemebershipValues
		LeftFrontSensor.push_back(Memebership(0, 150, 300, 500, "close"));
		LeftFrontSensor.push_back(Memebership(300, 400, 500, 600, "medium"));
		LeftFrontSensor.push_back(Memebership(500, 600, 600, 5010, "far"));

		//RightFrontSensor MemebershipValues
		RightFrontSensor.push_back(Memebership(0, 150, 300, 500, "close"));
		RightFrontSensor.push_back(Memebership(300, 400, 500, 600, "medium"));
		RightFrontSensor.push_back(Memebership(500, 600, 600, 5010, "far"));

		//MotorSpeed MemeberShipValues
		WheelVelocity.push_back(Memebership(-200, -100, -100, -50, "back"));
		WheelVelocity.push_back(Memebership(50, 75, 90, 120, "slow"));
		WheelVelocity.push_back(Memebership(100, 130, 200, 250, "medium"));
		WheelVelocity.push_back(Memebership(220, 240, 300, 350, "fast"));


	}

void ObstacleAvoidance::drawRules()
	{
		obstacleAvoidance.push_back(Rule(&LeftFrontSensor[0], &RightFrontSensor[0], &WheelVelocity[0], &WheelVelocity[1])); //closeClose backSlow
		obstacleAvoidance.push_back(Rule(&LeftFrontSensor[0], &RightFrontSensor[1], &WheelVelocity[0], &WheelVelocity[2])); //closeNormal backAverage
		obstacleAvoidance.push_back(Rule(&LeftFrontSensor[0], &RightFrontSensor[2], &WheelVelocity[0], &WheelVelocity[2])); //closeFar backAverage
		obstacleAvoidance.push_back(Rule(&LeftFrontSensor[1], &RightFrontSensor[0], &WheelVelocity[2], &WheelVelocity[0])); //mediumClose averageBack
		obstacleAvoidance.push_back(Rule(&LeftFrontSensor[1], &RightFrontSensor[1], &WheelVelocity[1], &WheelVelocity[1])); //mediumMedium slowSlow
		obstacleAvoidance.push_back(Rule(&LeftFrontSensor[1], &RightFrontSensor[2], &WheelVelocity[0], &WheelVelocity[1])); //mediumFar backSlow
		obstacleAvoidance.push_back(Rule(&LeftFrontSensor[2], &RightFrontSensor[0], &WheelVelocity[0], &WheelVelocity[1])); //farClose backSlow
		obstacleAvoidance.push_back(Rule(&LeftFrontSensor[2], &RightFrontSensor[1], &WheelVelocity[1], &WheelVelocity[0])); //farMedium slowBack
		obstacleAvoidance.push_back(Rule(&LeftFrontSensor[2], &RightFrontSensor[2], &WheelVelocity[2], &WheelVelocity[2])); //farFar average
	}

double ObstacleAvoidance::centriodDeFuzzification(vector<Memebership> agg, double fs[3])
	{
		double fsSum = 0;
		for (int i = 0; i < 3; i++)
		{
			fsSum += fs[i];
		}
		double fsCenteriodProducts = 0;

		for (int i = 0; i < 3; i++)
		{
			fsCenteriodProducts += agg[i].getCentriod()*fs[i];
		}
		return fsCenteriodProducts / fsSum;
	}

double ObstacleAvoidance::getOutput(double lfs, double lbs, int w)
	{
		double maxFS[3];
		double out;
		if (w == 1)
		{
			maxFS[0] = 0, maxFS[1] = 0, maxFS[2] = 0;
			for (int i = 0; i<obstacleAvoidance.size(); i++)
			{
				double fs = obstacleAvoidance[i].calcFiringStrength(lfs, lbs);
				//cout << "fs = " <<fs;
				if (obstacleAvoidance[i].leftVelocity->label == "slow") {
					//cout << "LeftVelocity set to slow";
					maxFS[0] = max(maxFS[0], fs);
				}
				else if (obstacleAvoidance[i].leftVelocity->label == "medium")
				{
					//cout << "LeftVelocity set to medium";
					maxFS[1] = max(maxFS[1], fs);
				}
				else if (obstacleAvoidance[i].leftVelocity->label == "fast") {
					//cout << "LeftVelocity se to fast";
					maxFS[2] = max(maxFS[2], fs);
				}
			}
			out = centriodDeFuzzification(WheelVelocity, maxFS);
			return  out;
		}

		if (w == 0)
		{
			maxFS[0] = 0, maxFS[1] = 0, maxFS[2] = 0;
			for (int i = 0; i<obstacleAvoidance.size(); i++)
			{
				double fs = obstacleAvoidance[i].calcFiringStrength(lfs, lbs);
				if (obstacleAvoidance[i].rightVelocity->label == "slow")
				{
					//cout << "Rightvelocity set to slow";
					maxFS[0] = max(maxFS[0], fs);
				}
				if (obstacleAvoidance[i].rightVelocity->label == "medium")
				{
					//cout << "RightVelocity set to medium";
					maxFS[1] = max(maxFS[1], fs);
				}
				if (obstacleAvoidance[i].rightVelocity->label == "fast")
				{
					//cout << "RightVelocity set to fast";
					maxFS[2] = max(maxFS[2], fs);
				}
			}
			out = centriodDeFuzzification(WheelVelocity, maxFS);
			return out;
		}
}



void main(int argc, char** argv)
{
		Aria::init();
		ArRobot robot;
		ArArgumentParser argParser(&argc, argv);
		argParser.loadDefaultArguments();
		ArRobotConnector robotConnector(&argParser, &robot);
		if (robotConnector.connectRobot())//cout << "" << endl;
			robot.runAsync(false);
		robot.lock();
		robot.enableMotors();
		robot.unlock();


		//cout << "___Sonar Conneted___" << endl;

		ArSensorReading *sonarSensor[8];
		REFController rightEdgeFollowing;
		ObstacleAvoidance ObstacleAvoidance;
		ObstacleAvoidance.setMemershipValues();
		ObstacleAvoidance.drawRules();
		rightEdgeFollowing.setMembershipValues();
		rightEdgeFollowing.drawRuleBase();
		while (true)
		{
			int  sonarRange[8];

			for (int j = 0; j < 8; j++) {
				sonarSensor[j] = robot.getSonarReading(j);
				sonarRange[j] = sonarSensor[j]->getRange();
			}
			double LMS; double RMS;
			float frontDistance = (sonarRange[3] +sonarRange[4])/ 2;
			;
			if (frontDistance < 800) {
				LMS = ObstacleAvoidance.getOutput(sonarRange[3], sonarRange[4], 1);
				RMS = ObstacleAvoidance.getOutput(sonarRange[3], sonarRange[4], 0);
			}
			
			else 
			{
				LMS = rightEdgeFollowing.getOutput(sonarRange[6], sonarRange[7], 1);
				RMS = rightEdgeFollowing.getOutput(sonarRange[6], sonarRange[7], 0);
			}
	
			//cout << "LEFT SPEED = " << LMS << endl;
			//cout << "RIGHT SPEED = " << RMS << endl;
			robot.setVel2(LMS, RMS);
		}


		robot.lock();
		robot.stop();
		robot.unlock();
		Aria::exit();
};