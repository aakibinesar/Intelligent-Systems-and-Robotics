
#include <vector>
using namespace std;
class Memebership
{
public:
	Memebership(double a, double b, double c, double d, string label);
	double getMembershipValue(double x);
	double getCentriod();
	string label;
private:
	double a, b, c, d;
	

};
typedef vector<Memebership> Condition;
typedef vector<Memebership> Consequent;

class Rule
{
public:
	Rule(Memebership *lfs, Memebership *lbs, Memebership *lV, Memebership *rV);
	double calcFiringStrength(double frontReading, double backReading);
	Memebership *leftFrontSensor;
	Memebership *leftBackSensor;
	Memebership *leftVelocity;
	Memebership *rightVelocity;

	
};
typedef vector<Rule> RuleBase;

class REFController 
{
public:
	void setMembershipValues();
	void drawRuleBase();
	double centriodDeFuzzification(vector<Memebership> agg, double fs[3]);
	double getOutput(double lfs, double lbs, int w);

private:
	RuleBase RIGHTEDGEFOLLOWING;
	Condition frontSensor;
	Condition backSensor;
	Consequent wheelVelocity;


};

class ObstacleAvoidance
{

public:
	void setMemershipValues();
	void drawRules();
	double centriodDeFuzzification(vector<Memebership> agg, double fs[3]);
	double getOutput(double lfs, double lbs, int w);
private:

	RuleBase obstacleAvoidance;
	Condition LeftFrontSensor;
	Condition RightFrontSensor;
	Consequent WheelVelocity;
};