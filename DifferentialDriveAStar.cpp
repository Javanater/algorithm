//
// Created by Madison on 11/26/2016.
//

#include "DifferentialDriveAStar.hpp"

flabs::DifferentialDriveAStarNode::DifferentialDriveAStarNode(int x, int y,
	int t) :
	state(x, y, t), x(get<0>(state)), y(get<1>(state)), t(get<2>(state))
{
}

std::ostream& flabs::operator<<(std::ostream& out,
	const flabs::DifferentialDriveAStarNode* node)
{
	if (node)
		return out << '(' << node->x << ',' << node->y << ',' << node->t << ')';
	else
		return out << "(null)";
}
