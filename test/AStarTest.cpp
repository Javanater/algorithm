//
// Created by Madison on 8/26/2016.
//

#include "gtest/gtest.h"
#include <algorithm/GridAStar.hpp>

using namespace flabs;
using namespace std;
using std::list;

template<class T>
ostream& operator<<(ostream& out, const list<T>& l)
{
	out << "{";
	if (l.size())
	{
		out << *l.begin();
		for (auto it = ++l.begin(); it != l.end(); ++it)
			out << ", " << *it;
	}
	return out << "}";
}

TEST(AStarTest, cardinal_no_obsticales)
{
	for (int dir = 0; dir < 8; ++dir)
	{
		GridAStar            astar;
		list<GridAStarNode*> path = astar(astar.getNode(0, 0),
			astar.getNode(GridAStar::xInc[dir] * 9, GridAStar::yInc[dir] * 9));
		list<GridAStarNode*> truePath;
		for (int             i    = 0; i <= 9; ++i)
			truePath.push_back(astar
				.getNode(GridAStar::xInc[dir] * i, GridAStar::yInc[dir] * i));
		ASSERT_EQ(truePath, path);
	}
}

TEST(AStarTest, cardinal_no_obsticales_reverse)
{
	for (int dir = 0; dir < 8; ++dir)
	{
		GridAStar            astar;
		list<GridAStarNode*> path = astar(
			astar.getNode(GridAStar::xInc[dir] * 9, GridAStar::yInc[dir] * 9),
			astar.getNode(0, 0));
		list<GridAStarNode*> truePath;
		for (int             i    = 0; i <= 9; ++i)
			truePath.push_front(astar
				.getNode(GridAStar::xInc[dir] * i, GridAStar::yInc[dir] * i));
		ASSERT_EQ(truePath, path);
	}
}

TEST(AStarTest, cardinal_one_obstacle)
{
	for (int dir = 0; dir < 8; ++dir)
	{
		GridAStar astar;
		astar.obstacles.insert(
			astar.getNode(GridAStar::xInc[dir] * 2, GridAStar::yInc[dir] * 2));
		GridAStarNode* start = astar.getNode(0, 0);
		GridAStarNode* goal  = astar
			.getNode(GridAStar::xInc[dir] * 9, GridAStar::yInc[dir] * 9);
		list<GridAStarNode*> path     = astar(start, goal);
		ASSERT_EQ(10 + dir % 2, path.size());
		ASSERT_EQ(start, *path.begin());
		ASSERT_EQ(goal, *--path.end());
		auto                 next     = path.begin();
		auto                 previous = next++;
		for (; next != path.end(); ++next, ++previous)
			ASSERT_EQ(true, (*next)->isNeighbor(*previous));
		for (GridAStarNode* node : path)
			ASSERT_EQ(false, astar.isObstacle(node));
	}
}

void box(GridAStar& astar, GridAStarNode* center, int boxSize)
{
	int x = center->x - boxSize;
	int y = center->y - boxSize;

	for (; x <= center->x + boxSize; ++x)
		astar.obstacles.insert(astar.getNode(x, y));

	for (; y <= center->y + boxSize; ++y)
		astar.obstacles.insert(astar.getNode(x, y));

	for (; x >= center->x - boxSize; --x)
		astar.obstacles.insert(astar.getNode(x, y));

	for (; y >= center->y - boxSize; --y)
		astar.obstacles.insert(astar.getNode(x, y));
}

TEST(AStarTest, cardinal_start_surrounded)
{
	for (int dir = 0; dir < 8; ++dir)
	{
		for (int boxSize = 1; boxSize < 8; ++boxSize)
		{
			GridAStar astar;
			GridAStarNode* start = astar.getNode(0, 0);
			GridAStarNode* goal  = astar
				.getNode(GridAStar::xInc[dir] * 9, GridAStar::yInc[dir] * 9);
			box(astar, start, boxSize);
			list<GridAStarNode*> path = astar(start, goal);
			ASSERT_EQ(10 + dir % 2, path.size());
		}
	}
}

TEST(AStarTest, cardinal_goal_surrounded)
{
	for (int dir = 0; dir < 8; ++dir)
	{
		for (int boxSize = 1; boxSize < 3; ++boxSize)
		{
			GridAStar astar;
			GridAStarNode* start = astar.getNode(0, 0);
			GridAStarNode* goal  = astar
				.getNode(GridAStar::xInc[dir] * 9, GridAStar::yInc[dir] * 9);
			box(astar, goal, boxSize);
			list<GridAStarNode*> path = astar(start, goal);
			for (GridAStarNode* node : path)
				ASSERT_EQ(false, astar.isObstacle(node));
			ASSERT_EQ(0, path.size());
		}
	}
}

void wall(GridAStar& astar, int y, int startX, int endX)
{
	for (; startX <= endX; ++startX)
		astar.obstacles.insert(astar.getNode(startX, y));
}

TEST(AStarTest, large_snake)
{
	GridAStar astar;
	GridAStarNode* start = astar.getNode(0, 0);
	int size = 3600;
	GridAStarNode* goal = astar.getNode(0, size);
	for (int             y        = size / 10; y < size; y += size / 10)
		wall(astar, y, y % size / 5 ? -size : size,
			y % size / 5 ? size / 10 : -size / 10);
	list<GridAStarNode*> path     = astar(start, goal, 10000000);
	ASSERT_EQ(true, path.size() > 0);
	ASSERT_EQ(start, *path.begin());
	ASSERT_EQ(goal, *--path.end());
	auto                 next     = path.begin();
	auto                 previous = next++;
	for (; next != path.end(); ++next, ++previous)
		ASSERT_EQ(true, (*next)->isNeighbor(*previous));
	for (GridAStarNode* node : path)
		ASSERT_EQ(false, astar.isObstacle(node));
}
