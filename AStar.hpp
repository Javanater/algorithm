//
// Created by Madison on 11/14/2016.
//

#ifndef PROJECTS_ASTAR_HPP
#define PROJECTS_ASTAR_HPP

#include <list>
#include <map>
#include <queue>
#include <set>
#include <Utilities/utilities.h>

namespace flabs
{
template<class Node>
class AStar
{
public:

	virtual ~AStar()
	{
	}

	struct OrderByCost
	{
		bool operator()(const Node* node1, const Node* node2) const
		{
			if (node1->fScore == node2->fScore)
				return *node1 < *node2;
			else
				return node1->fScore < node2->fScore;
		}
	};

	struct SuperNode
	{
		double fScore;
		double gScore;
		Node* cameFrom;
		bool closedSet;

		SuperNode(double fScore = std::numeric_limits<double>::infinity(),
			double gScore = std::numeric_limits<double>::infinity(),
			Node* cameFrom = nullptr) :
			fScore(fScore), gScore(gScore), cameFrom(cameFrom), closedSet(false)
		{
		}
	};

	virtual std::list<Node*>
	operator()(Node* start, Node* goal, size_t maxIterations = 1000)
	{
		std::set<Node*, OrderByCost> openSet;
		start->fScore = cost(start, goal);
		start->gScore = 0;
		openSet.insert(start);

		for (; !openSet.empty() && maxIterations; --maxIterations)
		{
			Node* current = *openSet.begin();

			if (!current)
				break;

			if (current == goal)
				return reconstructPath(&*current);

			openSet.erase(current);
			current->closedSet = true;

			for (size_t neighborIndex = 0;
				neighborIndex < neighborCount(current); ++neighborIndex)
			{
				Node* neighbor = getNeighbor(current, neighborIndex);

				if (neighbor->closedSet)
					continue;

				double
					tentativeGScore = current->gScore + cost(current, neighbor);

				if (neighbor->gScore <= tentativeGScore)
					continue;

				auto it = openSet.find(neighbor);

				if (it != openSet.end())
					openSet.erase(it);

				neighbor->cameFrom = current;
				neighbor->gScore   = tentativeGScore;
				neighbor->fScore   = tentativeGScore + cost(neighbor, goal);
				openSet.insert(neighbor);
			}
		}

		std::list<Node*> empty;
		return empty;
	}

	virtual std::list<Node*> reconstructPath(Node* current)
	{
		std::list<Node*> path;
		while (current)
		{
			path.push_front(current);
			current = current->cameFrom;
		}
		return path;
	}

	virtual double cost(Node* from, Node* to) = 0;

	virtual Node* getNeighbor(const Node* node, size_t neighborIndex) = 0;

	virtual size_t neighborCount(const Node* node) = 0;
};
}

#endif //PROJECTS_ASTAR_HPP
