//
// Created by Madison on 11/26/2016.
//

#ifndef PROJECTS_DIFFERENTIALDRIVEASTAR_HPP
#define PROJECTS_DIFFERENTIALDRIVEASTAR_HPP

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
class DifferentialDriveAStar;

struct DifferentialDriveAStarObstacle
{
	double x;
	double y;
	double size;

	DifferentialDriveAStarObstacle(double x, double y, double size);

	inline bool operator<(const DifferentialDriveAStarObstacle& node) const
	{
		return x < node.x || (x == node.x && y < node.y);
	}
};

struct DifferentialDriveAStarNode :
	public AStar<DifferentialDriveAStar, DifferentialDriveAStarNode>::SuperNode
{
	int x;
	int y;
	int yaw;
	const DifferentialDriveAStar& differentialDriveAStar;

	DifferentialDriveAStarNode(
		const DifferentialDriveAStar& differentialDriveAStar, int x, int y,
		int yaw);

	inline bool operator<(const DifferentialDriveAStarNode& node) const
	{
		return x < node.x ||
			(x == node.x && (y < node.y || (y == node.y && yaw < node.yaw)));
	}

	inline bool operator==(const DifferentialDriveAStarNode& node) const
	{
		return x == node.x && y == node.y && yaw == node.yaw;
	}

	friend std::ostream&
	operator<<(std::ostream& out, const DifferentialDriveAStarNode* node);
};

class DifferentialDriveAStar :
	public AStar<DifferentialDriveAStar, DifferentialDriveAStarNode>
{
private:
	static const double neighborLeftSpeed[];
	static const double neighborRightSpeed[];
	const double        topSpeed;
	const double        distanceBetweenWheels;
	const double gridUnit;
	const double angleResolution;
	double              topTurnRate;

	struct State
	{
		int dx;
		int dy;
		int yaw;

		bool operator==(const State& state) const;

		bool operator!=(const State& state) const;
	};

	std::vector<std::vector<State>>  nextStates;
	std::vector<std::vector<double>> nextCosts;

public:
	std::set<DifferentialDriveAStarNode> nodes;

	DifferentialDriveAStar(const double topSpeed,
		const double distanceBetweenWheels, const double gridUnit,
		const double angleResolution);

	~DifferentialDriveAStar();

	inline double
	cost(DifferentialDriveAStarNode* from, DifferentialDriveAStarNode* to,
		size_t neighborIndex) const
	{
		return nextCosts[from->yaw][neighborIndex];
	}

	inline double
	cost(DifferentialDriveAStarNode* from, DifferentialDriveAStarNode* to) const
	{
//		double a = std::atan2(fromGrid(to->y) - fromGrid(from->y),
//			fromGrid(to->x) - fromGrid(from->x));
//		return hypot(fromGrid(to->x) - fromGrid(from->x),
//			fromGrid(to->y) - fromGrid(from->y)) / topSpeed +
//			std::abs(angleDifference(a, fromGridAngle(from->yaw))) /
//				topTurnRate +
//			std::abs(angleDifference(fromGridAngle(to->yaw), a)) / topTurnRate;
//		return hypot(fromGrid(to->x) - fromGrid(from->x),
//			fromGrid(to->y) - fromGrid(from->y)) / topSpeed + std::abs(
//			angleDifference(fromGridAngle(to->yaw), fromGridAngle(from->yaw))) /
//			topTurnRate;
		return std::max(hypot(fromGrid(to->x) - fromGrid(from->x),
			fromGrid(to->y) - fromGrid(from->y)) / topSpeed, std::abs(
			angleDifference(fromGridAngle(to->yaw), fromGridAngle(from->yaw))) /
			topTurnRate);
	}

	inline DifferentialDriveAStarNode*
	getNeighbor(const DifferentialDriveAStarNode* node, size_t index)
	{
		State state = nextStates[node->yaw][index];
		return getNode(node->x + state.dx, node->y + state.dy, state.yaw);
	}

	inline size_t neighborCount(const DifferentialDriveAStarNode* node) const
	{
		return 8;
	}

	inline DifferentialDriveAStarNode* getNode(double x, double y, double yaw)
	{
		std::set<DifferentialDriveAStarNode>::iterator it;
		std::tie(it, std::ignore) =
			nodes.emplace(*this, toGrid(x), toGrid(y), toGridAngle(yaw));
		return &*it;
	}

	inline DifferentialDriveAStarNode* getNode(int x, int y, int yaw)
	{
		std::set<DifferentialDriveAStarNode>::iterator it;
		std::tie(it, std::ignore) = nodes.emplace(*this, x, y, yaw);
		return &*it;
	}

	void reset();

	int toGrid(double a) const;

	double fromGrid(int a) const;

	int toGridAngle(double a) const;

	double fromGridAngle(int a) const;

private:
	void calculateNextStates();

	State calculateNextState(double yaw, int index, double timeStep);
};
}

#endif //PROJECTS_DIFFERENTIALDRIVEASTAR_HPP
