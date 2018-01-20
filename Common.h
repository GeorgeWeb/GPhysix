#ifndef COMMON_H
#define COMMON_H
#ifdef PRAGMA_ONCE
#pragma once
#endif // !PRAGMA_ONCE

// Math constants
#define _USE_MATH_DEFINES
#include <cmath>  
#include <random>
#include <limits>

// GLM
#define GLM_FORCE_SWIZZLE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_operation.hpp>
#include "glm/ext.hpp"

// GLEW
#define GLEW_STATIC
#include <GL/glew.h>

// GLFW
#include <GLFW/glfw3.h>

/* OLD MACRO ...
** #define CMP(x, y) (fabsf((x)–(y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))
*/

template<typename T>
static bool CMP(const T x, const T y)
{
	return (std::fabs((x) - (y)) <= std::numeric_limits<T>::epsilon() * std::fmaxf(fabs(x), fabs(y)));
}

template<>
static bool CMP<float>(const float x, const float y)
{
	return (std::fabs((x)-(y)) <= std::numeric_limits<float>::epsilon() * std::fmaxf(1.0f, std::fmaxf(fabs(x), fabs(y))));
}

#endif // !COMMON_H