//
// Created by Madison on 11/22/2016.
//

#ifndef PROJECTS_DSTARLITE_HPP
#define PROJECTS_DSTARLITE_HPP

namespace flabs
{
template<class Base, class Node>
class DStarLite
{
protected:
	Base* base;

	DStarLite(Base* base) : base(base)
	{
	}
};
}

#endif //PROJECTS_DSTARLITE_HPP
