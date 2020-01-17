//
// Created by Madison on 11/26/2016.
//

#include <algorithm/DifferentialDriveAStar.hpp>

using namespace std;
using boost::math::constants::pi;
using boost::math::constants::two_pi;

namespace flabs
{
const double
			 DifferentialDriveAStar::neighborLeftSpeed[]  =
	{-1, -1, -1, 0, 0, 1, 1, 1};
const double DifferentialDriveAStar::neighborRightSpeed[] =
				 {-1, 0, 1, -1, 1, -1, 0, 1};

DifferentialDriveAStar::DifferentialDriveAStar(const double topSpeed,
	const double distanceBetweenWheels, const double gridUnit,
	const double angleResolution) :
	topSpeed(topSpeed), distanceBetweenWheels(distanceBetweenWheels),
	AStar(this), gridUnit(gridUnit), angleResolution(angleResolution)
{
	topTurnRate = topSpeed / (distanceBetweenWheels / 2);
	calculateNextStates();
}

DifferentialDriveAStar::~DifferentialDriveAStar()
{
}

void DifferentialDriveAStar::reset()
{
	nodes.clear();
}

int DifferentialDriveAStar::toGrid(double a) const
{
	return (int) floor(a / gridUnit + .5);
}

double DifferentialDriveAStar::fromGrid(int a) const
{
	return a * gridUnit;
}

int DifferentialDriveAStar::toGridAngle(double a) const
{
	return (int) (zeroTo2Pi(a) / angleResolution + .5);
}

double DifferentialDriveAStar::fromGridAngle(int a) const
{
	return a * angleResolution;
}

void DifferentialDriveAStar::calculateNextStates()
{
	for (double yaw = 0; yaw < two_pi<double>(); yaw += angleResolution)
	{
		vector<State>  states;
		vector<double> costs;
		State          current = {0, 0, toGridAngle(yaw)};
		for (size_t    index   = 0; index < neighborCount(nullptr); ++index)
		{
			//Find upper bound
			double upperBound = 1;
			while (current == calculateNextState(yaw, index, upperBound))
				upperBound *= 2;

			//Find lower bound
			double lowerBound = upperBound;
			while (current != calculateNextState(yaw, index, lowerBound))
				lowerBound /= 2;

			//Binary search
			while (abs(upperBound - lowerBound) > .001)
			{
				double timeStep = (lowerBound + upperBound) / 2;
				if (current == calculateNextState(yaw, index, timeStep))
					lowerBound = timeStep;
				else
					upperBound = timeStep;
			}

			State next = calculateNextState(yaw, index, upperBound);
			states.push_back(next);
			costs.push_back(upperBound);
		}
		nextStates.push_back(states);
		nextCosts.push_back(costs);
	}
}

DifferentialDriveAStar::State
DifferentialDriveAStar::calculateNextState(double yaw, int index,
	double timeStep)
{
	double leftDistance  = neighborLeftSpeed[index] * topSpeed * timeStep;
	double rightDistance = neighborRightSpeed[index] * topSpeed * timeStep;
	double deltaTheta    =
			   (rightDistance - leftDistance) / distanceBetweenWheels;
	double deltaX, deltaY;
	if (deltaTheta != 0)
	{
		double
			turnRadius = leftDistance / deltaTheta + distanceBetweenWheels / 2;
		deltaX = turnRadius * (cos(deltaTheta) - 1);
		deltaY = turnRadius * sin(deltaTheta);
	}
	else
	{
		deltaX = 0;
		deltaY = leftDistance;
	}

	double nx   = deltaX * sin(yaw) + deltaY * cos(yaw);
	double ny   = deltaX * cos(yaw) + deltaY * sin(yaw);
	double nYaw = yaw + deltaTheta;

	return {toGrid(nx), toGrid(ny), toGridAngle(nYaw)};
}

DifferentialDriveAStarNode::DifferentialDriveAStarNode(
	const DifferentialDriveAStar& differentialDriveAStar, int x, int y,
	int yaw) :
	differentialDriveAStar(differentialDriveAStar), x(x), y(y), yaw(yaw)
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

DifferentialDriveAStarObstacle::DifferentialDriveAStarObstacle(double x,
	double y, double size) : x(x), y(y), size(size)
{
}

bool DifferentialDriveAStar::State::operator==(const State& state) const
{
	return dx == state.dx && dy == state.dy && yaw == state.yaw;
}

bool DifferentialDriveAStar::State::operator!=(
	const DifferentialDriveAStar::State& state) const
{
	return !(*this == state);
}
}
