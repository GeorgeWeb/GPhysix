#ifndef COMMON_H
#define COMMON_H
#ifdef PRAGMA_ONCE
#pragma once
#endif // !PRAGMA_ONCE

// Math constants
#define _USE_MATH_DEFINES
#include <cmath>  
#include <random>

// GLM
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

#endif // !COMMON_H