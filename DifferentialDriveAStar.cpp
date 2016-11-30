//
// Created by Madison on 11/26/2016.
//

#include "DifferentialDriveAStar.hpp"

using namespace std;
using boost::math::constants::pi;

namespace flabs
{
const double
			 DifferentialDriveAStar::neighborLeftSpeed[]  =
	{-1, -1, -1, 0, 0, 1, 1, 1};
const double DifferentialDriveAStar::neighborRightSpeed[] =
				 {-1, 0, 1, -1, 1, -1, 0, 1};

DifferentialDriveAStar::DifferentialDriveAStar(const double timeStep,
	const double topSpeed, const double distanceBetweenWheels) :
	timeStep(timeStep), topSpeed(topSpeed),
	distanceBetweenWheels(distanceBetweenWheels)
{
	topTurnRate = topSpeed / (distanceBetweenWheels / 2);
}

DifferentialDriveAStar::~DifferentialDriveAStar()
{
	reset();
}

void DifferentialDriveAStar::reset()
{
	for (DifferentialDriveAStarNode* node : nodes)
		delete node;
}

DifferentialDriveAStarNode::DifferentialDriveAStarNode(double x, double y,
	double yaw, double leftSpeed, double rightSpeed) :
	x(x), y(y), yaw(yaw), leftSpeed(leftSpeed), rightSpeed(rightSpeed)
{
}

ostream& operator<<(ostream& out, const DifferentialDriveAStarNode* node)
{
	if (node)
		return out << '(' << node->x << ',' << node->y << ',' << node->yaw
			<< ')';
	else
		return out << "(null)";
}
}