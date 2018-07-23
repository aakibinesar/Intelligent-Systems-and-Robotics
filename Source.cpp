#include "Header.h"

Memebership::Memebership(double a, double b, double c, double d, string l)
{
	this->a = a;
	this->b = b;
	this->c = c;
	this->d = d;
	label = l;

}

double Memebership::getMembershipValue(double x)
{
	//leftShoulder
	if (a== b)
	{
		if (x < c)return 1;
		else if (x > c &&x < d)return (d - x) / (d - b);
	}

	//rightSHoudler
	if (b == d)
	{
		if (x > b) return 1;
		
		else if (x <b&&x>a)
		{
			return (d-x)/(d-b);
		}
		
		
	}

	//triangle
	if (b == c)
	{
		if (x > b)return x-a/b-a;
		else if (x > b)return d - x / d - b;
	}
	//Trapeziod
	else {
		if (x > b && x < c) {
			return 1;
		}
		else if (x < b && x > a) {
			return  (x - a) / (b - a);
			;
		}
		else if (x > c && x < d) {
			return  (d - x / (d - c));
			
		}

	}
	return 0;
}

Rule::Rule(Memebership*lfs, Memebership*lbs, Memebership*lv, Memebership *rv)
{
	leftFrontSensor = lfs;
	leftBackSensor = lbs;
	leftVelocity = lv;
	rightVelocity = rv;

}
double Rule::calcFiringStrength(double frontReading, double backReading)
{
	double firingStrength = leftFrontSensor->getMembershipValue(frontReading)*leftBackSensor->getMembershipValue(backReading);
	return firingStrength;

}
double Memebership::getCentriod()
{
	return d+a/ 2;
}

