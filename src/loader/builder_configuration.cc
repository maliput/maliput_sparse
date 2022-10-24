// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#include "maliput_sparse/loader/builder_configuration.h"

namespace maliput_sparse {
namespace loader {

BuilderConfiguration BuilderConfiguration::FromMap(const std::map<std::string, std::string>& config) {
  BuilderConfiguration builder_config;
  auto it = config.find("road_geometry_id");
  if (it != config.end()) {
    builder_config.road_geometry_id = maliput::api::RoadGeometryId(it->second);
  }

  it = config.find("linear_tolerance");
  if (it != config.end()) {
    builder_config.linear_tolerance = std::stod(it->second);
  }

  it = config.find("angular_tolerance");
  if (it != config.end()) {
    builder_config.angular_tolerance = std::stod(it->second);
  }

  it = config.find("scale_length");
  if (it != config.end()) {
    builder_config.scale_length = std::stod(it->second);
  }

  it = config.find("inertial_to_backend_frame_translation");
  if (it != config.end()) {
    builder_config.inertial_to_backend_frame_translation = maliput::math::Vector3::FromStr(it->second);
  }

  return builder_config;
}

std::map<std::string, std::string> BuilderConfiguration::ToStringMap() const {
  std::map<std::string, std::string> config;
  config.emplace("road_geometry_id", road_geometry_id.string());
  config.emplace("linear_tolerance", std::to_string(linear_tolerance));
  config.emplace("angular_tolerance", std::to_string(angular_tolerance));
  config.emplace("scale_length", std::to_string(scale_length));
  config.emplace("inertial_to_backend_frame_translation", inertial_to_backend_frame_translation.to_str());
  return config;
}

}  // namespace loader
}  // namespace maliput_sparse
