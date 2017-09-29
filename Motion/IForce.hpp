#pragma once
#include "../Common.h"

namespace GPhysix
{
	class IForce
	{
		protected:
			glm::vec3 m_force = glm::vec3(.0f);
		public:
			virtual void plugIn(IForce *newForce) = 0;
			inline virtual const glm::vec3 &getForce() const { return m_force; }
	};
}