#pragma once
#include "Force.h"

namespace GPhysix
{
	class Gravity : public Force
	{
	private:
		glm::vec3 m_gravity = glm::vec3(0.0f, -9.8f, 0.0f);

	public:
		Gravity();
		Gravity(const glm::vec3 &gravity);
		~Gravity();

		inline const glm::vec3 &getGravity() const { return m_gravity; }
		inline void setGravity(glm::vec3 gravity) { m_gravity = gravity; }

		inline const glm::vec3 &apply(const float &mass, const glm::vec3 &pos, const glm::vec3 &vel) override;
	};
}