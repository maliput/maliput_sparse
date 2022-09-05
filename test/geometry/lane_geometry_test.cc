// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2022, Toyota Research Institute. All rights reserved.
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
#include "geometry/lane_geometry.h"

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput/math/vector.h>

namespace maliput_sparse {
namespace geometry {
namespace test {
namespace {

using maliput::math::Vector2;
using maliput::math::Vector3;

struct OrientationTestCase {
  LineString3d left{};
  LineString3d right{};
  std::vector<double> p{};
  std::vector<maliput::math::RollPitchYaw> expected_rpy{};
};

std::vector<OrientationTestCase> OrientationTestCases() {
  return {
      {
          // No elevation along x
          LineString3d{{0., 2., 0.}, {100., 2., 0.}} /* left*/,
          LineString3d{{0., -2., 0.}, {100., -2., 0.}} /* right*/,
          {0., 50., 100.} /* p */,
          {{0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}} /* expected_rpy */
      },
      {
          // No elevation along -x
          LineString3d{{0., 2., 0.}, {-100., 2., 0.}} /* line string*/,
          LineString3d{{0., -2., 0.}, {-100., -2., 0.}} /* line string*/,
          {0., 50., 100.} /* p */,
          {{0., 0., M_PI}, {0., 0., M_PI}, {0., 0., M_PI}} /* expected_rpy */
      },
      {
          // Linear Elevation
          LineString3d{{0., 2., 0.}, {100., 2., 100.}} /* left*/,
          LineString3d{{0., -2., 0.}, {100., -2., 100.}} /* right*/,
          {0.} /* p */,
          {{0., -M_PI_4, 0.}} /* expected_rpy */
      },
      {
          // Linear Elevation with negative yaw.
          LineString3d{{1., 1., 0.}, {1. + 70.71067811865476, 1. - 70.71067811865476, 100.}} /* left*/,
          LineString3d{{-1., -1., 0.}, {-1. + 70.71067811865476, -1. - 70.71067811865476, 100.}} /* right*/,
          {
              0.,
              141.4213562373095,
          } /* p */,
          {{0., -M_PI_4, -M_PI_4}, {0., -M_PI_4, -M_PI_4}} /* expected_rpy */
      },
      {
          // Increase elevation + plateau + decrease elevation.
          LineString3d{{0., 2., 0.}, {10., 2., 10.}, {20., 2., 10.}, {30., 2., 0.}} /* left*/,
          LineString3d{{0., -2., 0.}, {10., -2., 10.}, {20., -2., 10.}, {30., -2., 0.}} /* right*/,
          {0., 1 + std::sqrt(2.) * 10., 1 + std::sqrt(2.) * 10. + 10.} /* p */,
          {{0., -M_PI_4, 0.}, {0., 0., 0.}, {0., M_PI_4, 0.}} /* expected_rpy */
      },
  };
}

class OrientationTest : public ::testing::TestWithParam<OrientationTestCase> {
 public:
  static constexpr double kTolerance{1e-12};
  OrientationTestCase case_ = GetParam();
};

TEST_P(OrientationTest, Test) {
  ASSERT_EQ(case_.p.size(), case_.expected_rpy.size()) << ">>>>> Test case is ill-formed.";
  const LaneGeometry lane_geometry{case_.left, case_.right, 1., 1e-3};
  for (std::size_t i = 0; i < case_.p.size(); ++i) {
    const auto rpy = lane_geometry.Orientation(case_.p[i]);
    std::cout << rpy.vector() << std::endl;
    EXPECT_EQ(case_.expected_rpy[i].vector(), rpy.vector())
        << "Expected RPY: " << case_.expected_rpy[i].vector() << " vs RPY: " << rpy.vector();
  }
}

INSTANTIATE_TEST_CASE_P(OrientationTestGroup, OrientationTest, ::testing::ValuesIn(OrientationTestCases()));

}  // namespace
}  // namespace test
}  // namespace geometry
}  // namespace maliput_sparse
