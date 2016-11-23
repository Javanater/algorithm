//
// Created by Madison on 11/22/2016.
//

#ifndef PROJECTS_LPASTAR_HPP
#define PROJECTS_LPASTAR_HPP

#include <limits>
#include <list>
#include <tuple>
#include <set>

namespace flabs
{
template<class Base, class Node>
class LPAStar
{
protected:
	Base* base;
	Node* start;
	Node* goal;
	std::set<Node*> openList;

public:
	struct OrderByKey
	{
		bool operator()(const Node* node1, const Node* node2) const
		{
			return node1->key < node2->key;
		}
	};

	struct Key
	{
		double first;
		double second;

		Key(double first = std::numeric_limits<double>::infinity(),
			double second = std::numeric_limits<double>::infinity()) :
			first(first), second(second)
		{
		}

		bool operator<(const Key& key) const
		{
			return first < key.first ||
				(first == key.first && second < key.second);
		}
	};

protected:
	LPAStar(Base* base, Node* start, Node* goal) :
		base(base), start(start), goal(goal)
	{
		initialize();
	}

protected:
	struct SuperNode
	{
		double           g;
		double           rhs;
		double           h;
		Key              key;
		std::list<Node*> children;
		std::list<Node*> parents;

	protected:
		SuperNode(double g = std::numeric_limits<double>::infinity(),
			double h = std::numeric_limits<double>::infinity(),
			double rhs = std::numeric_limits<double>::infinity()) :
			g(g), h(h), rhs(rhs)
		{
		}

		~SuperNode()
		{
		}
	};

private:
	inline Key& calculateKey(Node* node) const
	{
		double temp = std::min(node->g, node->rhs);
		node->key = std::make_tuple(temp + base->cost(node, goal), temp);
		return node->key;
	}

	inline void initialize()
	{
		start->rhs = 0;
		calculateKey(start);
		openList.insert(start);
	}

	inline void updateVertex(Node* node)
	{
		if (node != start)
		{
			node->rhs     = std::numeric_limits<double>::infinity();
			for (Node* p : node->parents)
				node->rhs = std::min(node->rhs, p->g + base->cost(p, node));
		}
		openList.erase(node);
		if (node->g != node->rhs)
		{
			calculateKey(node);
			openList.insert(node);
		}
	}

public:
	std::list<Node*> operator()(size_t maxIterations = 1000)
	{
		for (; ((*openList.begin())->key < calculateKey(goal) ||
			goal->rhs != goal->g) && maxIterations; --maxIterations)
		{
			Node* current = *openList.begin();
			openList.erase(openList.begin());

			if (current->g > current->rhs)
				current->g = current->rhs;
			else
				current->g = std::numeric_limits<double>::infinity();

			for (Node* s : current->children)
				updateVertex(s);
		}
	}
};
}

#endif //PROJECTS_LPASTAR_HPP
