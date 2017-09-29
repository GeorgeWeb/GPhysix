#pragma once
// Math constraints
#define _USE_MATH_DEFINES
#include <cmath>
#include "Body.h"

namespace GPhysix
{
	class Particle : public Body
	{
		public:
			Particle();
			~Particle();
			
			void move(glm::vec3 newVel, glm::vec3 newPos);
	};
}