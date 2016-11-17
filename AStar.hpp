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
			typedef lessPointer<Node*> CompareNode;

			virtual ~AStar()
			{
			}

			struct CostNode
			{
				Node* node;
				double cost;

				CostNode(Node* node, double cost) : node(node), cost(cost)
				{
				}

				bool operator<(const CostNode& other) const
				{
					if (cost == other.cost)
						return node < other.node;
					else
						return cost < other.cost;
				}
			};

			virtual std::list<Node*>
			operator()(Node* start, Node* goal, size_t maxIterations = 1000)
			{
				std::set<Node*, CompareNode> closedSet;

				std::map<Node*, Node*, CompareNode> cameFrom;

				std::map<Node*, double, CompareNode> gScoreT;
				auto                                 gScore = wrapMap(gScoreT,
					std::numeric_limits<double>::infinity());
				gScore[start]                               = 0;

				std::map<Node*, double, CompareNode> fScoreT;
				auto                                 fScore = wrapMap(fScoreT,
					std::numeric_limits<double>::infinity());
				fScore[start] = cost(start, goal);

				std::set<CostNode> costSortedOpenSet;
				costSortedOpenSet.emplace(start, fScore[start]);

				for (; !costSortedOpenSet.empty() && maxIterations;
					--maxIterations)
				{
					Node* current = costSortedOpenSet.begin()->node;

					if (!current)
						break;

					if (current == goal)
						return reconstructPath(cameFrom, current);

					costSortedOpenSet.erase(CostNode(current, fScore[current]));
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

						if (!costSortedOpenSet
							.count(CostNode(neighbor, fScore[neighbor])))
							costSortedOpenSet.emplace(neighbor, tentativeCost);
						else if (tentativeScore >= gScore[neighbor])
							continue;
						else
						{
							int c = costSortedOpenSet
								.erase(CostNode(neighbor, fScore[neighbor]));
							costSortedOpenSet.emplace(neighbor, tentativeCost);
						}

						cameFrom[neighbor] = current;
						gScore[neighbor]   = tentativeScore;
						fScore[neighbor]   = tentativeCost;
					}
				}

				std::list<Node*> empty;
				return empty;
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

			virtual double cost(Node* from, Node* to) = 0;

			virtual Node*
			getNeighbor(const Node* node, size_t neighborIndex) = 0;

			virtual size_t neighborCount(const Node* node) = 0;
	};
}

#endif //PROJECTS_ASTAR_HPP
