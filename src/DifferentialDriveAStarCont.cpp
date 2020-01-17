//
// Created by Madison on 11/26/2016.
//

#include <algorithm/DifferentialDriveAStarCont.hpp>

using namespace std;
using boost::math::constants::pi;
using boost::math::constants::two_pi;

namespace flabs
{
const double
			 DifferentialDriveAStarCont::neighborLeftSpeed[]  =
	{-1, -1, -1, 0, 0, 1, 1, 1};
const double DifferentialDriveAStarCont::neighborRightSpeed[] =
				 {-1, 0, 1, -1, 1, -1, 0, 1};

DifferentialDriveAStarCont::DifferentialDriveAStarCont(const double topSpeed,
	const double distanceBetweenWheels, const double timeStep) :
	topSpeed(topSpeed), distanceBetweenWheels(distanceBetweenWheels),
	AStar(this), timeStep(timeStep)
{
	topTurnRate = topSpeed / (distanceBetweenWheels / 2);
}

DifferentialDriveAStarCont::DifferentialDriveAStarCont(const double topSpeed,
	const double distanceBetweenWheels, const double timeStep, function<void(
	set<DifferentialDriveAStarContNode*, AStar<DifferentialDriveAStarCont,
		DifferentialDriveAStarContNode>::OrderByCost>)> openSetCallback) :
	topSpeed(topSpeed), distanceBetweenWheels(distanceBetweenWheels),
	AStar(this, openSetCallback), timeStep(timeStep)
{
	topTurnRate = topSpeed / (distanceBetweenWheels / 2);
}

DifferentialDriveAStarCont::~DifferentialDriveAStarCont()
{
}

void DifferentialDriveAStarCont::reset()
{
	nodes.clear();
}

DifferentialDriveAStarContNode::DifferentialDriveAStarContNode(
	const DifferentialDriveAStarCont& differentialDriveAStar, double x,
	double y, double yaw) :
	differentialDriveAStar(differentialDriveAStar), x(x), y(y), yaw(yaw)
{
}

ostream& operator<<(ostream& out, const DifferentialDriveAStarContNode* node)
{
	if (node)
		return out << '(' << node->x << ',' << node->y << ',' << node->yaw
			<< ')';
	else
		return out << "(null)";
}

DifferentialDriveAStarContObstacle::DifferentialDriveAStarContObstacle(double x,
	double y, double size) : x(x), y(y), size(size)
{
}

bool DifferentialDriveAStarCont::State::operator==(const State& state) const
{
	return dx == state.dx && dy == state.dy && yaw == state.yaw;
}

bool DifferentialDriveAStarCont::State::operator!=(
	const DifferentialDriveAStarCont::State& state) const
{
	return !(*this == state);
}
}
