/**
 * @file loader.cpp
 * @brief This file contains functions to load and dump information about glTF
 * models using TinyGLTF.
 */

#include "pd_sim/loader.hpp"
#include <filesystem>

// Include tinygltf (with ugly warning suppression...)
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-parameter"
#pragma clang diagnostic ignored "-Wsign-compare"
#pragma clang diagnostic ignored "-Wmissing-field-initializers"
#pragma clang diagnostic ignored "-Wunused-function"
#pragma clang diagnostic ignored "-Wunused-variable"
#pragma clang diagnostic ignored "-Wmissing-field-initializers"
#elif defined(__GNUC__) || defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif

#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "tiny_gltf.h"

#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__) || defined(__GNUG__)
#pragma GCC diagnostic pop
#endif

namespace pdsim {

void dumpModelInfo(tinygltf::Model &model, rclcpp::Logger logger) {
  // print model info
  RCLCPP_INFO(logger, "Model has %zu nodes", model.nodes.size());
  RCLCPP_INFO(logger, "Model has %zu meshes", model.meshes.size());
  RCLCPP_INFO(logger, "Model has %zu materials", model.materials.size());
  RCLCPP_INFO(logger, "Model has %zu textures", model.textures.size());
  RCLCPP_INFO(logger, "Model has %zu images", model.images.size());
  RCLCPP_INFO(logger, "Model has %zu animations", model.animations.size());
  RCLCPP_INFO(logger, "Model has %zu skins", model.skins.size());
  RCLCPP_INFO(logger, "Model has %zu cameras", model.cameras.size());
  RCLCPP_INFO(logger, "Model has %zu lights", model.lights.size());

  // print list of vertex attributes
  for (const auto &mesh : model.meshes) {
    RCLCPP_INFO(logger, "Mesh %s has %zu primitives", mesh.name.c_str(),
                mesh.primitives.size());

    for (const auto &primitive : mesh.primitives) {
      RCLCPP_INFO(logger, "Primitive has %zu attributes",
                  primitive.attributes.size());
      // primitive type
      RCLCPP_INFO(logger, "Primitive type: %d", primitive.mode);
      for (const auto &attribute : primitive.attributes) {
        RCLCPP_INFO(logger, "Attribute: %s", attribute.first.c_str());
      }

      // print extras
      for (const auto &extra : primitive.extras.Keys()) {
        RCLCPP_INFO(logger, "Extra: %s", extra.c_str());
      }
    }
  }
}

bool loadModel(tinygltf::Model &model, const std::string &filename) {
  tinygltf::TinyGLTF loader;
  std::string err;
  std::string warn;
  bool ret = false;

  // determine .gltf or .glb
  auto extension = std::filesystem::path(filename).extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 ::tolower);
  if (extension == ".gltf") {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("loadModel"),
                       "Loading glTF file: " << filename);
    ret = loader.LoadASCIIFromFile(&model, &err, &warn, filename);
  } else if (extension == ".glb") {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("loadModel"),
                       "Loading glTF binary file: " << filename);
    ret = loader.LoadBinaryFromFile(&model, &err, &warn, filename);
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("loadModel"),
                        "Unknown file extension: " << filename);
    return false;
  }

  if (!warn.empty()) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("loadModel"), warn);
  }

  if (!err.empty()) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("loadModel"), err);
  }

  if (!ret) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("loadModel"),
                        "Failed to load glTF: " << filename);
  }
  return ret;
}

} // namespace pdsim