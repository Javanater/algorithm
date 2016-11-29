//
// Created by Madison on 11/26/2016.
//

#ifndef PROJECTS_DIFFERENTIALDRIVEASTAR_HPP
#define PROJECTS_DIFFERENTIALDRIVEASTAR_HPP

#include "AStar.hpp"
#include <cmath>
#include <map>
#include <ostream>
#include <tuple>

namespace flabs
{
class DifferentialDriveAStar;

struct DifferentialDriveAStarNode :
	public AStar<DifferentialDriveAStar, DifferentialDriveAStarNode>::SuperNode
{
	std::tuple<int, int, int, int, int> state;
	int& x;
	int& y;
	int& yaw;
	int& leftSpeed;
	int& rightSpeed;

	DifferentialDriveAStarNode(int x, int y, int t, int leftSpeed,
		int rightSpeed);

	inline bool operator<(const DifferentialDriveAStarNode& node) const
	{
		return state < node.state;
	}

	inline bool operator==(const DifferentialDriveAStarNode& node) const
	{
		return state == node.state;
	}

	inline bool isNeighbor(const DifferentialDriveAStarNode* node) const
	{
		//TODO: Implement for diffy drive
		return std::abs(x - node->x) <= 1 && std::abs(y - node->y) <= 1;
	}

	friend std::ostream&
	operator<<(std::ostream& out, const DifferentialDriveAStarNode* node);
};

class DifferentialDriveAStar
{
public:
	std::set<DifferentialDriveAStarNode*>                            obstacles;
	std::map<std::tuple<int, int, int>, DifferentialDriveAStarNode*> nodes;

	DifferentialDriveAStar();

	~DifferentialDriveAStar();

	inline double
	cost(DifferentialDriveAStarNode* from, DifferentialDriveAStarNode* to,
		size_t neighborIndex) const
	{
		return cost(from, to);
	}

	inline double
	cost(DifferentialDriveAStarNode* from, DifferentialDriveAStarNode* to) const
	{
		return std::sqrt(
			std::pow(to->x - from->x, 2) + std::pow(to->y - from->y, 2)) *
			costMultiplierLookup[obstacles.count(to)];
	}

	inline DifferentialDriveAStarNode*
	getNeighbor(const DifferentialDriveAStarNode* node, size_t index)
	{
		return getNode(node->x + xInc[index], node->y + yInc[index]);
	}

	inline size_t neighborCount(const DifferentialDriveAStarNode* node) const
	{
		return 8;
	}

	inline DifferentialDriveAStarNode*& getNode(int x, int y, int t)
	{
		DifferentialDriveAStarNode*& n = nodes[{x, y, t}];
		if (!n)
			n = new DifferentialDriveAStarNode(x, y, t);
		return n;
	}

	inline bool isObstacle(DifferentialDriveAStarNode* node) const
	{
		return obstacles.count(node) >= 1;
	}

	void reset();
};
}

#endif //PROJECTS_DIFFERENTIALDRIVEASTAR_HPP
