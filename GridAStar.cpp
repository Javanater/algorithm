//
// Created by Madison on 11/21/2016.
//

#include "GridAStar.hpp"

using namespace std;

namespace flabs
{
GridAStarNode::GridAStarNode(int x, int y) : x(x), y(y)
{
}

const int    GridAStar::xInc[]                  = {1, 1, 0, -1, -1, -1, 0, 1};
const int    GridAStar::yInc[]                  = {0, 1, 1, 1, 0, -1, -1, -1};
const double GridAStar::costMultiplierLookup[]  = {1, 1000};
//@formatter:off
const double GridAStar::neighborCostLookup[][8] = {
	{
		std::sqrt(std::pow(xInc[0], 2) + std::pow(yInc[0], 2)),
		std::sqrt(std::pow(xInc[1], 2) + std::pow(yInc[1], 2)),
		std::sqrt(std::pow(xInc[2], 2) + std::pow(yInc[2], 2)),
		std::sqrt(std::pow(xInc[3], 2) + std::pow(yInc[3], 2)),
		std::sqrt(std::pow(xInc[4], 2) + std::pow(yInc[4], 2)),
		std::sqrt(std::pow(xInc[5], 2) + std::pow(yInc[5], 2)),
		std::sqrt(std::pow(xInc[6], 2) + std::pow(yInc[6], 2)),
		std::sqrt(std::pow(xInc[7], 2) + std::pow(yInc[7], 2))
	},
	{
		std::sqrt(std::pow(xInc[0], 2) + std::pow(yInc[0], 2)) * 1000,
		std::sqrt(std::pow(xInc[1], 2) + std::pow(yInc[1], 2)) * 1000,
		std::sqrt(std::pow(xInc[2], 2) + std::pow(yInc[2], 2)) * 1000,
		std::sqrt(std::pow(xInc[3], 2) + std::pow(yInc[3], 2)) * 1000,
		std::sqrt(std::pow(xInc[4], 2) + std::pow(yInc[4], 2)) * 1000,
		std::sqrt(std::pow(xInc[5], 2) + std::pow(yInc[5], 2)) * 1000,
		std::sqrt(std::pow(xInc[6], 2) + std::pow(yInc[6], 2)) * 1000,
		std::sqrt(std::pow(xInc[7], 2) + std::pow(yInc[7], 2)) * 1000
	}
};
//@formatter:on

GridAStar::GridAStar() : AStar<GridAStar, GridAStarNode>(this)
{
}

GridAStar::~GridAStar()
{
	for (auto m : nodes)
		for (auto p : m.second)
			delete p.second;
}

ostream& operator<<(ostream& out, const GridAStarNode* node)
{
	if (node)
		return out << '(' << node->x << ',' << node->y << ')';
	else
		return out << "(null)";
}
}