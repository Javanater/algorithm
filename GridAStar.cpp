//
// Created by Madison on 11/21/2016.
//

#include "GridAStar.hpp"
#include <list>

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
		sqrt(pow(xInc[0], 2) + pow(yInc[0], 2)),
		sqrt(pow(xInc[1], 2) + pow(yInc[1], 2)),
		sqrt(pow(xInc[2], 2) + pow(yInc[2], 2)),
		sqrt(pow(xInc[3], 2) + pow(yInc[3], 2)),
		sqrt(pow(xInc[4], 2) + pow(yInc[4], 2)),
		sqrt(pow(xInc[5], 2) + pow(yInc[5], 2)),
		sqrt(pow(xInc[6], 2) + pow(yInc[6], 2)),
		sqrt(pow(xInc[7], 2) + pow(yInc[7], 2))
	},
	{
		sqrt(pow(xInc[0], 2) + pow(yInc[0], 2)) * 1000,
		sqrt(pow(xInc[1], 2) + pow(yInc[1], 2)) * 1000,
		sqrt(pow(xInc[2], 2) + pow(yInc[2], 2)) * 1000,
		sqrt(pow(xInc[3], 2) + pow(yInc[3], 2)) * 1000,
		sqrt(pow(xInc[4], 2) + pow(yInc[4], 2)) * 1000,
		sqrt(pow(xInc[5], 2) + pow(yInc[5], 2)) * 1000,
		sqrt(pow(xInc[6], 2) + pow(yInc[6], 2)) * 1000,
		sqrt(pow(xInc[7], 2) + pow(yInc[7], 2)) * 1000
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

void GridAStar::reset()
{
	std::list<GridAStarNode*> toBeRemoved;
	for (auto                 m : nodes)
		for (auto p : m.second)
			if (obstacles.count(p.second) == 0)
				toBeRemoved.push_back(p.second);

	for (GridAStarNode* node : toBeRemoved)
	{
		std::map<int, GridAStarNode*>& m = nodes[node->x];
		m.erase(node->y);
		if (m.size() == 0)
			nodes.erase(node->x);
		delete node;
	}
}

ostream& operator<<(ostream& out, const GridAStarNode* node)
{
	if (node)
		return out << '(' << node->x << ',' << node->y << ')';
	else
		return out << "(null)";
}
}