//
// Created by Madison on 11/22/2016.
//

#ifndef PROJECTS_LPASTAR_HPP
#define PROJECTS_LPASTAR_HPP

#include <limits>
#include <list>
#include <tuple>

namespace flabs
{
template<class Base, class Node>
class LPAStar
{
protected:
	Base* base;

	LPAStar(Base* base) : base(base)
	{
	}

	struct SuperNode
	{
		double                     g;
		double                     rhs;
		double                     h;
		std::tuple<double, double> key;
		std::list<Node*>           children;
		std::list<Node*>           parents;

	protected:
		SuperNode(double g = std::numeric_limits<double>::infinity(),
			double h = std::numeric_limits<double>::infinity(),
			double rhs = std::numeric_limits<double>::infinity(), double key) :
			g(g), h(h), rhs(rhs), key(key)
		{
		}

		~SuperNode()
		{
		}
	};

	void operator()()
	{
		//For each s in Graph
		//	s.g(x) = rhs(x) = ∞; (locally consistent)
		//end for each

		//startNode.rhs = 0; (overconsistent)
		//Forever
		//While(OpenList.Top().key<goal.key OR
		//		   goal is incosistent)
		//	currentNode=OpenList.Pop();
		//	if(currentNode is overconsistent)
		//		currentNode.g(x) = currentNode.rhs(x); (Consistent)
		//	else
		//		currentNode.g(x)= ∞; (overconsistent OR consistent)
		//	end if
		//	for each s in currentNode.Children[]
		//		update s.rhs(x); (consistent OR inconsistent)
		//	end for each
		//End while
		//Wait for changes in Graph
		//	For each connection (u, v) with changed cost
		//		Update connection(u, v);
		//		Make v locally inconsistent;
		//	end for each
		//End forever
	}
};
}

#endif //PROJECTS_LPASTAR_HPP
