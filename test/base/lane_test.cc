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
#include "base/lane.h"

#include <gtest/gtest.h>
#include <maliput/api/lane.h>
#include <maliput/api/lane_data.h>
#include <maliput/common/assertion_error.h>
#include <maliput/math/vector.h>
#include <maliput/test_utilities/maliput_math_compare.h>

#include "maliput_sparse/geometry/line_string.h"

namespace maliput_sparse {
namespace test {
namespace {

using geometry::LineString3d;
using maliput::api::InertialPosition;
using maliput::api::LanePosition;
using maliput::api::Rotation;
using maliput::math::Vector2;
using maliput::math::Vector3;

struct LaneTestCase {
  LineString3d left{};
  LineString3d right{};
  std::vector<LanePosition> srh{};
  std::vector<InertialPosition> expected_backend_pos{};
  std::vector<Rotation> expected_rotation{};
  double expected_length{};
};

std::vector<LaneTestCase> LaneTestCases() {
  return {{
              LineString3d{{0., 2., 0.}, {100., 2., 0.}} /* left*/,
              LineString3d{{0., -2., 0.}, {100., -2., 0.}} /* right*/,
              {{0., 0., 0.}} /* srh */,
              {{0., 0., 0.}} /* expected_backend_pos */,
              {Rotation::FromRpy(0., 0., 0.)} /* expected_rotation */,
              100. /* expected_length */
          },
          {
              // Arc-like lane:
              //    | |  --> no elevation
              //  __/ /  --> no elevation
              //  __ /   --> linear elevation
              LineString3d{{0., 2., 0.}, {100., 2., 100.}, {200., 102., 100.}, {200., 200., 100.}} /* left*/,
              LineString3d{{0., -2., 0.}, {100., -2., 100.}, {204., 102., 100.}, {204., 200., 100.}} /* right*/,
              // Centerline : {0., 0., 0.}, {100., 0., 100.}, {202., 102, 100.}, {202., 200., 100.}
              {
                  {50. * std::sqrt(2.), 0., 0.},
                  {50. * std::sqrt(2.), -2., -2.},
                  {100. * std::sqrt(2.) + 102. * std::sqrt(2.), 0., 0.},
                  {100. * std::sqrt(2.) + 102. * std::sqrt(2.) + 98., -1., 4.},
              } /* srh */,
              {
                  {50., 0., 50.},
                  {50. + 2 * std::sqrt(2.) / 2., -2., 50. - 2 * std::sqrt(2.) / 2.},
                  {202., 102., 100.},
                  {203., 200., 104.},
              } /* expected_backend_pos */,
              {
                  Rotation::FromRpy(0., -M_PI / 4., 0.),
                  Rotation::FromRpy(0., -M_PI / 4., 0.),
                  Rotation::FromRpy(0., 0., M_PI / 4.),
                  Rotation::FromRpy(0., 0., M_PI / 2.),
              } /* expected_rotation */,
              100. * std::sqrt(2.) + 102. * std::sqrt(2.) + 98. /* expected_length */
          }};
}

class LaneTest : public ::testing::TestWithParam<LaneTestCase> {
 public:
  static constexpr double kTolerance{1.e-5};
  static constexpr double kScaleLength{1.};

  const maliput::api::LaneId kLaneId{"dut id"};
  const maliput::api::HBounds kHBounds{0., 5.};
  LaneTestCase case_ = GetParam();
  std::unique_ptr<geometry::LaneGeometry> lane_geometry_ =
      std::make_unique<geometry::LaneGeometry>(case_.left, case_.right, kTolerance, kScaleLength);
};

TEST_P(LaneTest, Test) {
  ASSERT_EQ(case_.srh.size(), case_.expected_backend_pos.size()) << ">>>>> Test case is ill-formed.";
  ASSERT_EQ(case_.srh.size(), case_.expected_rotation.size()) << ">>>>> Test case is ill-formed.";
  const Lane dut{kLaneId, kHBounds, std::move(lane_geometry_)};
  EXPECT_DOUBLE_EQ(case_.expected_length, dut.length());
  for (std::size_t i = 0; i < case_.srh.size(); ++i) {
    const auto backend_pos = dut.ToBackendPosition(case_.srh[i]);
    const auto rpy = dut.GetOrientation(case_.srh[i]);
    EXPECT_TRUE(maliput::math::test::CompareVectors(case_.expected_backend_pos[i].xyz(), backend_pos, kTolerance))
        << "Expected backend_pos: " << case_.expected_backend_pos[i].xyz() << " vs backend_pos: " << backend_pos;
    EXPECT_TRUE(
        maliput::math::test::CompareVectors(case_.expected_rotation[i].rpy().vector(), rpy.rpy().vector(), kTolerance))
        << "Expected RPY: " << case_.expected_rotation[i].rpy().vector() << " vs RPY: " << rpy.rpy().vector();
  }
}

INSTANTIATE_TEST_CASE_P(LaneTestGroup, LaneTest, ::testing::ValuesIn(LaneTestCases()));

}  // namespace
}  // namespace test
}  // namespace maliput_sparse
