#pragma once
#include "../IForce.hpp"
#include <vector>

namespace GPhysix
{
	class DynamicForce : public IForce
	{
		private:
			// dynamic properties
			const glm::vec3 m_grav = glm::vec3(0.0f, -9.8f, 0.0f);
		public:
			DynamicForce();
			~DynamicForce();

			inline const glm::vec3 &getGravity() const { return m_grav; }

			// Interface
			void plugIn(IForce *newForce) override;
			
			void setForce(const glm::vec3 &newForce);
	};
}
