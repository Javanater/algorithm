//
// Created by Madison on 11/26/2016.
//

#ifndef PROJECTS_DIFFERENTIALDRIVEASTAR_HPP
#define PROJECTS_DIFFERENTIALDRIVEASTAR_HPP

#include "AStar.hpp"
#include <cmath>
#include <map>
#include <ostream>

namespace flabs
{
class DifferentialDriveAStar;

struct DifferentialDriveAStarNode :
	public AStar<DifferentialDriveAStar, DifferentialDriveAStarNode>::SuperNode
{
	std::tuple<int, int, int> state;

	DifferentialDriveAStarNode(int x, int y, int t);

	inline bool operator<(const DifferentialDriveAStarNode& node) const
	{
		return state < node.state;
	}

	inline bool operator==(const DifferentialDriveAStarNode& node) const
	{
		return x == node.x && y == node.y;
	}

	inline bool isNeighbor(const DifferentialDriveAStarNode* node) const
	{
		return std::abs(x - node->x) <= 1 && std::abs(y - node->y) <= 1;
	}

	friend std::ostream&
	operator<<(std::ostream& out, const DifferentialDriveAStarNode* node);
};

class DifferentialDriveAStar
{
};
}

#endif //PROJECTS_DIFFERENTIALDRIVEASTAR_HPP
