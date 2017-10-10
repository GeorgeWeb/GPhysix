#pragma once
#include <glm/glm.hpp>
#include <iostream>

namespace GPhysix
{
	class Body;

	class Force
	{
	public:
		Force() {}
		~Force() {}

		virtual const glm::vec3 &apply(const float &mass, const glm::vec3 &pos, const glm::vec3 &vel) = 0;
	};
}