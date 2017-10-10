#pragma once
#include "Force.h"

namespace GPhysix
{
	class Drag : public Force
	{
	private:
		glm::vec3 m_drag;

	public:
		Drag();
		Drag(const glm::vec3 &drag);
		~Drag();

		inline const glm::vec3 &getDrag() const { return m_drag; }
		inline void setDrag(const glm::vec3 &drag) { m_drag = drag; }

		inline const glm::vec3 &apply(const float &mass, const glm::vec3 &pos, const glm::vec3 &vel);
	};
}