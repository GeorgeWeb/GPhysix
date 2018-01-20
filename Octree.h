#ifndef OCTREE_H
#define OCTREE_H

#include "Mesh.h"
#include "Geometry.h"
#include <vector>

namespace GPhysix
{
	typedef struct OctreeNode
	{
		OBB bounds;
		
		OctreeNode* children;
		
		std::vector<Mesh*> models;
		
		inline OctreeNode() : children(0) { }
		inline ~OctreeNode()
		{
			if (children != 0)
				delete[] children;
		}
	} OctreeNode;
}

#endif !OCTREE_H