//
// Created by Madison on 11/26/2016.
//

#ifndef PROJECTS_DIFFERENTIALDRIVEASTARCONT_HPP
#define PROJECTS_DIFFERENTIALDRIVEASTARCONT_HPP

#include "AStar.hpp"
#include <cmath>
#include <map>
#include <ostream>
#include <vector>
#include <tuple>
#include <boost/math/constants/constants.hpp>
#include <Math/Math.hpp>

namespace flabs
{
class DifferentialDriveAStarCont;

struct DifferentialDriveAStarContObstacle
{
	double x;
	double y;
	double size;

	DifferentialDriveAStarContObstacle(double x, double y, double size);

	inline bool operator<(const DifferentialDriveAStarContObstacle& node) const
	{
		return x < node.x || (x == node.x && y < node.y);
	}
};

struct DifferentialDriveAStarContNode :
	public AStar<DifferentialDriveAStarCont,
		DifferentialDriveAStarContNode>::SuperNode
{
	double x;
	double y;
	double yaw;
	const DifferentialDriveAStarCont& differentialDriveAStar;

	DifferentialDriveAStarContNode(
		const DifferentialDriveAStarCont& differentialDriveAStar, double x,
		double y, double yaw);

	inline bool operator<(const DifferentialDriveAStarContNode& node) const
	{
		return !(*this == node) && x < node.x ||
			(x == node.x && (y < node.y || (y == node.y && yaw < node.yaw)));
	}

	inline bool operator==(const DifferentialDriveAStarContNode& node) const
	{
		return within(x, node.x, .1) && within(y, node.y, .1) &&
			std::abs(angleDifference(yaw, node.yaw)) <= .03;
	}

	friend std::ostream&
	operator<<(std::ostream& out, const DifferentialDriveAStarContNode* node);
};

class DifferentialDriveAStarCont :
	public AStar<DifferentialDriveAStarCont, DifferentialDriveAStarContNode>
{
private:
	static const double neighborLeftSpeed[];
	static const double neighborRightSpeed[];
	const double        timeStep;
	const double        topSpeed;
	const double        distanceBetweenWheels;
	double              topTurnRate;

	struct State
	{
		double dx;
		double dy;
		double yaw;

		bool operator==(const State& state) const;

		bool operator!=(const State& state) const;
	};

	std::vector<std::vector<State>>  nextStates;
	std::vector<std::vector<double>> nextCosts;

public:
	std::set<DifferentialDriveAStarContNode> nodes;

	DifferentialDriveAStarCont(const double topSpeed,
		const double distanceBetweenWheels, const double timeStep);

	DifferentialDriveAStarCont(const double topSpeed,
		const double distanceBetweenWheels, const double timeStep,
		std::function<void(std::set<DifferentialDriveAStarContNode*,
			OrderByCost>)> openSetCallback);

	~DifferentialDriveAStarCont();

	inline double cost(DifferentialDriveAStarContNode* from,
		DifferentialDriveAStarContNode* to, size_t neighborIndex) const
	{
		return timeStep;
	}

	inline double cost(DifferentialDriveAStarContNode* from,
		DifferentialDriveAStarContNode* to) const
	{
		double a = std::atan2(to->y - from->y, to->x - from->x);
		return hypot(to->x - from->x, to->y - from->y) / topSpeed +
			std::abs(angleDifference(a, from->yaw)) / topTurnRate +
			std::abs(angleDifference(to->yaw, a)) / topTurnRate;

//		double a                       =
//				   std::atan2(to->y - from->y, to->x - from->x);
//		double distanceToTarget        =
//				   hypot(to->x - from->x, to->y - from->y);
//		double timeToTranslateToTarget = distanceToTarget / topSpeed;
//		return timeToTranslateToTarget * timeToTranslateToTarget +
//			std::abs(angleDifference(a, from->yaw)) / topTurnRate +
//			std::abs(angleDifference(to->yaw, a)) / topTurnRate;

//		double a                       =
//				   std::atan2(to->y - from->y, to->x - from->x);
//		double distanceToTarget = hypot(to->x - from->x, to->y - from->y);
//		double timeToTranslateToTarget = distanceToTarget / topSpeed;
//		double timeToTurnTowardsTarget =
//				   std::abs(angleDifference(a, from->yaw)) / topTurnRate;
//		double timeToAlignWithTarget   =
//				   std::abs(angleDifference(to->yaw, a)) / topTurnRate;
//		double cost = timeToTranslateToTarget;
//		if (distanceToTarget < 1)
//			cost += timeToTurnTowardsTarget * distanceToTarget + timeToAlignWithTarget * (1-distanceToTarget);
//		else
//			cost += timeToTurnTowardsTarget;
//		return cost;

//		return hypot(to->x - from->x, to->y - from->y) / topSpeed +
//			std::abs(angleDifference(to->yaw, from->yaw)) / topTurnRate;

//		return std::max(hypot(to->x - from->x, to->y - from->y) / topSpeed,
//			std::abs(angleDifference(to->yaw, from->yaw)) / topTurnRate);
	}

	inline DifferentialDriveAStarContNode*
	getNeighbor(const DifferentialDriveAStarContNode* node, size_t index)
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
			deltaX            = turnRadius * (std::cos(deltaTheta) - 1);
			deltaY            = turnRadius * std::sin(deltaTheta);
		}
		else
		{
			deltaX = 0;
			deltaY = leftDistance;
		}

		double nx            = node->x + deltaX * std::sin(node->yaw) +
			deltaY * std::cos(node->yaw);
		double ny            = node->y + deltaX * std::cos(node->yaw) +
			deltaY * std::sin(node->yaw);
		double nYaw          = node->yaw + deltaTheta;

		return getNode(nx, ny, nYaw);
	}

	inline size_t
	neighborCount(const DifferentialDriveAStarContNode* node) const
	{
		return 8;
	}

	inline DifferentialDriveAStarContNode*
	getNode(double x, double y, double yaw)
	{
		std::set<DifferentialDriveAStarContNode>::iterator it;
		std::tie(it, std::ignore) = nodes.emplace(*this, x, y, yaw);
		return &*it;
	}

	void reset();
};
}

#endif //PROJECTS_DIFFERENTIALDRIVEASTARCONT_HPP
