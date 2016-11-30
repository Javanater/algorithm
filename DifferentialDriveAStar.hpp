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
#include <boost/math/constants/constants.hpp>
#include <Math/Math.hpp>

namespace flabs
{
class DifferentialDriveAStar;

struct DifferentialDriveAStarNode :
	public AStar<DifferentialDriveAStar, DifferentialDriveAStarNode>::SuperNode
{
	double x;
	double y;
	double yaw;
	double leftSpeed;
	double rightSpeed;

	DifferentialDriveAStarNode(double x, double y, double yaw, double leftSpeed,
		double rightSpeed);

	inline bool operator==(const DifferentialDriveAStarNode& node) const
	{
		return std::abs(x - node.x) <= .01 && std::abs(y - node.y) <= .01 &&
			std::abs(angleDifference(yaw, node.yaw)) <= .1;
	}

	inline bool isNeighbor(const DifferentialDriveAStarNode* node) const
	{
		//TODO: Implement for diffy drive
		return true;
	}

	friend std::ostream&
	operator<<(std::ostream& out, const DifferentialDriveAStarNode* node);
};

class DifferentialDriveAStar
{
private:
	static const double neighborLeftSpeed[];
	static const double neighborRightSpeed[];
	const double        timeStep;
	const double        topSpeed;
	const double        distanceBetweenWheels;
	double              topTurnRate;

public:
	std::set<DifferentialDriveAStarNode*>  obstacles;
	std::list<DifferentialDriveAStarNode*> nodes;

	DifferentialDriveAStar(const double timeStep, const double topSpeed,
		const double distanceBetweenWheels);

	~DifferentialDriveAStar();

	inline double
	cost(DifferentialDriveAStarNode* from, DifferentialDriveAStarNode* to,
		size_t neighborIndex) const
	{
		return timeStep;
	}

	inline double
	cost(DifferentialDriveAStarNode* from, DifferentialDriveAStarNode* to) const
	{
		return hypot(to->x - from->x, to->y - from->y) / topSpeed +
			std::abs(angleDifference(to->yaw, from->yaw)) / topTurnRate;
	}

	inline DifferentialDriveAStarNode*
	getNeighbor(const DifferentialDriveAStarNode* node, size_t index)
	{
		double leftDistance  = neighborLeftSpeed[index] * topSpeed * timeStep;
		double rightDistance = neighborRightSpeed[index] * topSpeed * timeStep;
		double deltaTheta    =
				   (rightDistance - leftDistance) / distanceBetweenWheels;
		double deltaX, deltaY;
		if (deltaTheta != 0)
		{
			double turnRadius =
					   leftDistance / deltaTheta + distanceBetweenWheels / 2;
			deltaX = turnRadius * (std::cos(deltaTheta) - 1);
			deltaY = turnRadius * std::sin(deltaTheta);
		}
		else
		{
			deltaX = 0;
			deltaY = leftDistance;
		}

		double nx   = node->x + deltaX * std::sin(node->yaw) +
			deltaY * std::cos(node->yaw);
		double ny   = node->y + deltaX * std::cos(node->yaw) +
			deltaY * std::sin(node->yaw);
		double nYaw = node->yaw + deltaTheta;

		DifferentialDriveAStarNode* neighbor =
									  new DifferentialDriveAStarNode(nx, ny,
										  nYaw, neighborLeftSpeed[index],
										  neighborRightSpeed[index]);
		nodes.push_back(neighbor);
		return neighbor;
	}

	inline size_t neighborCount(const DifferentialDriveAStarNode* node) const
	{
		return 7;
	}

	inline bool isObstacle(DifferentialDriveAStarNode* node) const
	{
		//TODO: Implement for diffy drive
		return false;
	}

	void reset();
};
}

#endif //PROJECTS_DIFFERENTIALDRIVEASTAR_HPP
