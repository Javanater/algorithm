//
// Created by Madison on 11/26/2016.
//

#include "DifferentialDriveAStar.hpp"

using namespace std;

flabs::DifferentialDriveAStarNode::DifferentialDriveAStarNode(int x, int y,
	int t, int leftSpeed, int rightSpeed) :
	state(x, y, t), x(get<0>(state)), y(get<1>(state)), yaw(get<2>(state)),
	leftSpeed(get<3>(state)), rightSpeed(get<4>(state))
{
}

std::ostream& flabs::operator<<(std::ostream& out,
	const flabs::DifferentialDriveAStarNode* node)
{
	if (node)
		return out << '(' << node->x << ',' << node->y << ',' << node->yaw
			<< ')';
	else
		return out << "(null)";
}
