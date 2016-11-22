//
// Created by Madison on 11/21/2016.
//

#ifndef PROJECTS_GRIDASTAR_HPP
#define PROJECTS_GRIDASTAR_HPP

#include "AStar.hpp"
#include <cmath>
#include <map>
#include <ostream>

namespace flabs
{
class GridAStar;

struct GridAStarNode : public AStar<GridAStar, GridAStarNode>::SuperNode
{
	int x;
	int y;

	GridAStarNode(int x, int y);

	bool operator<(const GridAStarNode& node) const
	{
		if (x == node.x)
			return y < node.y;
		else
			return x < node.x;
	}

	bool operator==(const GridAStarNode& node) const
	{
		return x == node.x && y == node.y;
	}

	bool isNeighbor(const GridAStarNode* node) const
	{
		return std::abs(x - node->x) <= 1 && std::abs(y - node->y) <= 1;
	}

	friend std::ostream&
	operator<<(std::ostream& out, const GridAStarNode* node);
};

class GridAStar : public AStar<GridAStar, GridAStarNode>
{
public:
	static const int    xInc[];
	static const int    yInc[];
	static const double neighborCostLookup[][8];
	static const double costMultiplierLookup[];

	std::set<GridAStarNode*>                     obstacles;
	std::map<int, std::map<int, GridAStarNode*>> nodes;

	GridAStar();

	~GridAStar();

	double
	cost(GridAStarNode* from, GridAStarNode* to, size_t neighborIndex) const
	{
		return neighborCostLookup[obstacles.count(to)][neighborIndex];
	}

	double cost(GridAStarNode* from, GridAStarNode* to) const
	{
		return std::sqrt(
			std::pow(to->x - from->x, 2) + std::pow(to->y - from->y, 2)) *
			costMultiplierLookup[obstacles.count(to)];
	}

	GridAStarNode* getNeighbor(const GridAStarNode* node, size_t index)
	{
		return getNode(node->x + xInc[index], node->y + yInc[index]);
	}

	size_t neighborCount(const GridAStarNode* node) const
	{
		return 8;
	}

	GridAStarNode*& getNode(int x, int y)
	{
		GridAStarNode*& n = nodes[x][y];
		if (!n)
			n = new GridAStarNode(x, y);
		return n;
	}

	bool isObstacle(GridAStarNode* node) const
	{
		return obstacles.count(node) >= 1;
	}
};
}

#endif //PROJECTS_GRIDASTAR_HPP
