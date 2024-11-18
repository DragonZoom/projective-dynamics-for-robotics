#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "tiny_gltf.h"

namespace pdsim {

void drawLoop(tinygltf::Model &model, float scale = 1.f);

} // namespace pdsim