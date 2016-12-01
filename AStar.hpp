//
// Created by Madison on 11/14/2016.
//

#ifndef PROJECTS_ASTAR_HPP
#define PROJECTS_ASTAR_HPP

#include <set>
#include <list>
#include <limits>
#include <cstddef>
#include <cmath>

namespace flabs
{
template<class Base, class Node>
class AStar
{
protected:
	Base* base;

	AStar(Base* base) : base(base)
	{
	}

public:
	struct OrderByCost
	{
		bool operator()(const Node* node1, const Node* node2) const
		{
			return node1->fScore < node2->fScore ||
				(node1->fScore == node2->fScore && *node1 < *node2);
		}
	};

	struct SuperNode
	{
		double fScore;
		double gScore;
		Node* cameFrom;
		bool closedSet;

	protected:
		SuperNode(double fScore = std::numeric_limits<double>::infinity(),
			double gScore = std::numeric_limits<double>::infinity(),
			Node* cameFrom = nullptr) :
			fScore(fScore), gScore(gScore), cameFrom(cameFrom), closedSet(false)
		{
		}

		~SuperNode()
		{
		}
	};

	std::list<Node*>
	operator()(Node* start, Node* goal, size_t maxIterations = 1000)
	{
		std::set<Node*, OrderByCost> openSet;
		start->fScore = base->cost(start, goal);
		start->gScore = 0;
		openSet.insert(start);

		for (; !openSet.empty() && maxIterations; --maxIterations)
		{
			Node* current = *openSet.begin();

			if (!current)
				break;

			if (*current == *goal)
				return reconstructPath(&*current);

			openSet.erase(current);
			current->closedSet = true;

			for (size_t neighborIndex = 0;
				neighborIndex < base->neighborCount(current); ++neighborIndex)
			{
				Node* neighbor = base->getNeighbor(current, neighborIndex);

				if (neighbor->closedSet)
					continue;

				double tentativeGScore = current->gScore +
					base->cost(current, neighbor, neighborIndex);

				if (neighbor->gScore <= tentativeGScore)
					continue;

				openSet.erase(neighbor);
				neighbor->cameFrom = current;
				neighbor->gScore   = tentativeGScore;
				neighbor->fScore   =
					tentativeGScore + base->cost(neighbor, goal);
				openSet.insert(neighbor);
			}
		}

		std::list<Node*> empty;
		return empty;
	}

	std::list<Node*> reconstructPath(Node* current)
	{
		std::list<Node*> path;
		while (current)
		{
			path.push_front(current);
			current = current->cameFrom;
		}
		return path;
	}
};
}

#endif //PROJECTS_ASTAR_HPP
