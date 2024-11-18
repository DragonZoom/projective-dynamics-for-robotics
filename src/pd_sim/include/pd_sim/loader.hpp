#pragma once

#include "rclcpp/rclcpp.hpp"
#include "tiny_gltf.h"

namespace pdsim {

/**
 * @brief Loads a glTF model from a file.
 *
 * @param[out] model The loaded glTF model.
 * @param filename The path to the glTF file.
 * @return True if the model was loaded successfully, false otherwise.
 */
bool loadModel(tinygltf::Model &model, const std::string &filename);

/**
 * @brief Prints detailed information about the glTF model.
 *
 * @param model The glTF model to print information about.
 * @param logger The logger to use for printing the information.
 */
void dumpModelInfo(tinygltf::Model &model, rclcpp::Logger logger);
} // namespace pdsim
