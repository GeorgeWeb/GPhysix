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
			~Particle() { };
	};
}