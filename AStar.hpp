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
			struct CompareNode
			{
				bool operator()(const Node* node1, const Node* node2) const
				{
					return *node1 < *node2;
				}
			};

			virtual ~AStar()
			{
			}

			virtual std::list<Node*>
			reconstructPath(std::map<Node*, Node*, CompareNode>& cameFrom,
				Node* current)
			{
				std::list<Node*> path;
				do
				{
					path.push_front(current);
					current = cameFrom[current];
				}
				while (cameFrom.count(current));
				path.push_front(current);
				return path;
			}

			virtual std::list<Node*>
			operator()(Node* start, Node* goal, size_t maxIterations = 1000)
			{
				std::set<Node*, CompareNode> closedSet;

				std::set<Node*, CompareNode> openSet;
				openSet.insert(start);

				std::map<Node*, Node*, CompareNode> cameFrom;

				std::map<Node*, double, CompareNode> gScoreT;
				auto                                 gScore = wrapMap(gScoreT,
					std::numeric_limits<double>::infinity());
				gScore[start]                               = 0;

				std::map<Node*, double, CompareNode> fScoreT;
				auto                                 fScore = wrapMap(fScoreT,
					std::numeric_limits<double>::infinity());
				fScore[start] = cost(start, goal);

				for (; !openSet.empty() && maxIterations; --maxIterations)
				{
					Node* current = lowest(openSet, fScore);

					if (!current)
						break;

					if (current == goal)
						return reconstructPath(cameFrom, current);

					openSet.erase(current);
					closedSet.insert(current);

					for (size_t neighborIndex = 0;
						neighborIndex < neighborCount(current); ++neighborIndex)
					{
						Node* neighbor = getNeighbor(current, neighborIndex);
						if (closedSet.count(neighbor))
							continue;

						double tentativeScore =
								   gScore[current] + cost(current, neighbor);
						double tentativeCost  =
								   tentativeScore + cost(neighbor, goal);
						if (!openSet.count(neighbor))
							openSet.insert(neighbor);
						else if (tentativeScore >= gScore[neighbor])
							continue;

						cameFrom[neighbor] = current;
						gScore[neighbor]   = tentativeScore;
						fScore[neighbor]   = tentativeCost;
					}
				}

				std::list<Node*> empty;
				return empty;
			}

			Node* lowest(std::set<Node*, CompareNode> openSet, auto& fScore)
			{
				double lowestFScore = std::numeric_limits<double>::infinity();
				Node     * lowest = nullptr;
				for (Node* node : openSet)
				{
					double& score = fScore[node];
					if (score < lowestFScore)
					{
						lowestFScore = score;
						lowest       = node;
					}
				}
				return lowest;
			}

			virtual double cost(Node* from, Node* to) = 0;

			virtual Node*
			getNeighbor(const Node* node, size_t neighborIndex) = 0;

			virtual size_t neighborCount(const Node* node) = 0;
	};
}

#endif //PROJECTS_ASTAR_HPP
