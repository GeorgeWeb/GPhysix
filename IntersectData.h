#ifndef INTERSECT_DATA_H
#define INTERSECT_DATA_H

#include "Common.h"

namespace GPhysix
{
	typedef struct Interval
	{
		float min;
		float max;
	} Interval;

	class IntersectData
	{
		public:
			IntersectData(const bool hasIntersection, const float distance) :
				m_hasIntersection(hasIntersection),
				m_distance(distance) 
			{}

			IntersectData(const bool hasIntersection, const glm::vec3& axis) :
				m_hasIntersection(hasIntersection),
				m_axis(axis)
			{}

			inline bool getHasIntersection() { return m_hasIntersection; }
			inline float getDistance() { return m_distance; }
			inline const glm::vec3& getAxis() { return m_axis; }

		private:
			bool m_hasIntersection;
			float m_distance;
			glm::vec3 m_axis;
	};
}

#endif