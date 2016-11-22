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

struct Node : public AStar<GridAStar, Node>::SuperNode
{
	int x;
	int y;

	Node(int x, int y);

	bool operator<(const Node& node) const
	{
		if (x == node.x)
			return y < node.y;
		else
			return x < node.x;
	}

	bool operator==(const Node& node) const
	{
		return x == node.x && y == node.y;
	}

	bool isNeighbor(const Node* node) const
	{
		return std::abs(x - node->x) <= 1 && std::abs(y - node->y) <= 1;
	}

	friend std::ostream& operator<<(std::ostream& out, const Node* node);
};

class GridAStar : public AStar<GridAStar, Node>
{
public:
	static const int    xInc[];
	static const int    yInc[];
	static const double neighborCostLookup[][8];
	static const double costMultiplierLookup[];

	std::set<Node*>                     obstacles;
	std::map<int, std::map<int, Node*>> nodes;

	GridAStar();

	~GridAStar();

	double cost(Node* from, Node* to, size_t neighborIndex) const
	{
		return neighborCostLookup[obstacles.count(to)][neighborIndex];
	}

	double cost(Node* from, Node* to) const
	{
		return std::sqrt(
			std::pow(to->x - from->x, 2) + std::pow(to->y - from->y, 2)) *
			costMultiplierLookup[obstacles.count(to)];
	}

	Node* getNeighbor(const Node* node, size_t index)
	{
		return getNode(node->x + xInc[index], node->y + yInc[index]);
	}

	size_t neighborCount(const Node* node) const
	{
		return 8;
	}

	Node*& getNode(int x, int y)
	{
		Node*& n = nodes[x][y];
		if (!n)
			n = new Node(x, y);
		return n;
	}

	bool isObstacle(Node* node) const
	{
		return obstacles.count(node) >= 1;
	}
};
}

#endif //PROJECTS_GRIDASTAR_HPP
