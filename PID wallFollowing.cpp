#include <Aria.h>;
#include <iostream>;
#include <deque>;

using namespace std;

class PidController
{
private:
	float disiredDistance;
	float kP = 0.6;
	float kI = 0.012;
	float kD = 0.4;


	std::deque<float> previousErrors;
	float minSPEED, maxSPEED;
	float eLast;

	void appendError(float error)
	{
		if(previousErrors.size()>=50)
		{
			previousErrors.pop_front();
			previousErrors.push_back(error);
		}
		else previousErrors.push_back(error);
		
	}
	float sumErrors()
	{
		deque<float>::iterator it = previousErrors.begin();
		float sum = 0.0;
		while(it!=previousErrors.end())
		{
			sum += *it++;
		}
		return sum;
	}
	

public:
	PidController(int speed)
	{
		minSPEED= -speed; maxSPEED = speed;
	}

	void SetTargetRange(float distance)
	{
		disiredDistance = distance;
	}

	

	int GetOutput(float sonarReading[])
	{
		
		float eP = 0; //current error
		float actualDistance = 0.0;
		for (int i = 4; i < 8; i++)
		{
			if (sonarReading[i] > 2 * disiredDistance) {
				sonarReading[i] = 2 * disiredDistance;
			}
			float reading = (sonarReading[i]);
			eP += (disiredDistance-reading);
			actualDistance += reading;
		}
		eP = eP / 4;

		cout << "Actual Distance: " << actualDistance/4<<endl;
		appendError(eP);
		float eI = sumErrors(); // sum of accumulatedError
		float eD = eP - eLast;  // difference between currentError and previousError

		eLast = eP;
		
		int Output = eP*kP + eI*kI + eD*kD;
		Output = Output>maxSPEED? maxSPEED: Output;
		Output = Output < minSPEED ? minSPEED : Output;
		return Output;
	}
	
};

int speed = 200;
float targetRange = 400;

int main(int argc, char** argv)
{
	Aria::init();
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();
	ArRobot robot;
	ArRobotConnector robotConnector(&argParser, &robot);
	if (robotConnector.connectRobot())
		cout << "Robot Connected" << endl;
	robot.runAsync(false);
	robot.lock();
	robot.enableMotors();
	robot.unlock();
		
	ArSensorReading *sonarSensor[8];
	for (int i = 0; i < 8; i++)
	{
		sonarSensor[i] = robot.getSonarReading(i);
	}
	cout << "Sensors Connected" << endl;

	

	PidController controller(speed);
	controller.SetTargetRange(targetRange);
	
	
	float sonarRange[8];
	while (true)
	{
		int output = 0;
		for (int i = 4; i < 8; i++)	sonarRange[i] = sonarSensor[i]->getRange();
		output = controller.GetOutput(sonarRange);

		int lVel = speed - output;
		int rVel = speed + output;
		robot.setVel2(lVel, rVel);
	}

	
	robot.lock();
	robot.stop();
	robot.unlock();
	Aria::exit();
}
