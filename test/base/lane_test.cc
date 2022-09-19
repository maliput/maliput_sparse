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
#include <maliput/test_utilities/maliput_types_compare.h>

#include "maliput_sparse/geometry/line_string.h"

#define IsLanePositionResultClose(lpr_a, lpr_b, tolerance)                                           \
  do {                                                                                               \
    EXPECT_TRUE(IsLanePositionClose(lpr_a.lane_position, lpr_b.lane_position, tolerance));           \
    EXPECT_TRUE(IsInertialPositionClose(lpr_a.nearest_position, lpr_b.nearest_position, tolerance)); \
    EXPECT_NEAR(lpr_a.distance, lpr_b.distance, tolerance);                                          \
  } while (0);

namespace maliput_sparse {
namespace test {
namespace {

using geometry::LineString3d;
using maliput::api::InertialPosition;
using maliput::api::LanePosition;
using maliput::api::LanePositionResult;
using maliput::api::Rotation;
using maliput::api::test::IsInertialPositionClose;
using maliput::api::test::IsLanePositionClose;
using maliput::api::test::IsRotationClose;
using maliput::math::Vector2;
using maliput::math::Vector3;

struct LaneTestCase {
  LineString3d left{};
  LineString3d right{};
  std::vector<LanePosition> srh{};
  std::vector<InertialPosition> expected_backend_pos{};
  std::vector<Rotation> expected_rotation{};
  double expected_length{};
  std::vector<LanePositionResult> expected_lane_position_result{};
};

std::vector<LaneTestCase> LaneTestCases() {
  return {{
              LineString3d{{0., 2., 0.}, {100., 2., 0.}} /* left*/,
              LineString3d{{0., -2., 0.}, {100., -2., 0.}} /* right*/,
              {{0., 0., 0.}} /* srh */,
              {{0., 0., 0.}} /* expected_backend_pos */,
              {Rotation::FromRpy(0., 0., 0.)} /* expected_rotation */,
              100. /* expected_length */,
              {{
                  {0., 0., 0.} /* lane_position */, {0., 0., 0.} /* nearest_position */, 0. /* distance */
              }} /* expected_lane_position_result */
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
              100. * std::sqrt(2.) + 102. * std::sqrt(2.) + 98. /* expected_length */,
              {{
                   {50. * std::sqrt(2.), 0., 0.} /* lane_position */,
                   {50., 0., 50.} /* nearest_position */,
                   0. /* distance */
               },
               {
                   {50. * std::sqrt(2.), -2., -2.} /* lane_position */,
                   {50. + 2 * std::sqrt(2.) / 2., -2., 50. - 2 * std::sqrt(2.) / 2.} /* nearest_position */,
                   0. /* distance */
               },
               {
                   {100. * std::sqrt(2.) + 102. * std::sqrt(2.), 0., 0.} /* lane_position */,
                   {202., 102., 100.} /* nearest_position */,
                   0. /* distance */
               },
               {
                   {100. * std::sqrt(2.) + 102. * std::sqrt(2.) + 98., -1., 4.} /* lane_position */,
                   {203., 200., 104.} /* nearest_position */,
                   0. /* distance */
               }}     /* expected_lane_position_result */
          }};
}

class LaneTest : public ::testing::TestWithParam<LaneTestCase> {
 public:
  static constexpr double kTolerance{1.e-5};
  static constexpr double kScaleLength{1.};

  void SetUp() override {
    ASSERT_EQ(case_.srh.size(), case_.expected_backend_pos.size()) << ">>>>> Test case is ill-formed.";
    ASSERT_EQ(case_.srh.size(), case_.expected_rotation.size()) << ">>>>> Test case is ill-formed.";
  }

  const maliput::api::LaneId kLaneId{"dut id"};
  const maliput::api::HBounds kHBounds{-5., 5.};
  LaneTestCase case_ = GetParam();
  std::unique_ptr<geometry::LaneGeometry> lane_geometry_ =
      std::make_unique<geometry::LaneGeometry>(case_.left, case_.right, kTolerance, kScaleLength);
};

TEST_P(LaneTest, Test) {
  std::unique_ptr<Lane> dut = std::make_unique<Lane>(kLaneId, kHBounds, std::move(lane_geometry_));
  EXPECT_DOUBLE_EQ(case_.expected_length, dut->length());
  for (std::size_t i = 0; i < case_.srh.size(); ++i) {
    const auto backend_pos = dut->ToBackendPosition(case_.srh[i]);
    const auto rpy = dut->GetOrientation(case_.srh[i]);
    const auto lane_position_result = dut->ToLanePositionBackend(case_.expected_backend_pos[i]);
    IsRotationClose(case_.expected_rotation[i], rpy, kTolerance);
    IsInertialPositionClose(case_.expected_backend_pos[i], InertialPosition::FromXyz(backend_pos), kTolerance);
    IsLanePositionResultClose(case_.expected_lane_position_result[i], lane_position_result, kTolerance);
  }
}

INSTANTIATE_TEST_CASE_P(LaneTestGroup, LaneTest, ::testing::ValuesIn(LaneTestCases()));

struct ToLaneSegmentPositionTestCase {
  LineString3d left{};
  LineString3d right{};
  double expected_length{};
  std::vector<InertialPosition> backend_pos{};
  std::vector<LanePositionResult> expected_lane_position_result{};
};

std::vector<ToLaneSegmentPositionTestCase> ToLaneSegmentPositionTestCases() {
  return {{
      // Arc-like lane:
      //    | |  --> no elevation
      //  __/ /  --> no elevation
      //  __ /   --> linear elevation
      LineString3d{{0., 2., 0.}, {100., 2., 100.}, {200., 102., 100.}, {200., 200., 100.}} /* left*/,
      LineString3d{{0., -2., 0.}, {100., -2., 100.}, {204., 102., 100.}, {204., 200., 100.}} /* right*/,
      // Centerline : {0., 0., 0.}, {100., 0., 100.}, {202., 102, 100.}, {202., 200., 100.}
      100. * std::sqrt(2.) + 102. * std::sqrt(2.) + 98. /* expected_length */,
      {
          {50., 0., 50.},
          {50., 2., 50.},
          {50., 10., 50.},
      } /* backend_pos */,
      {
          // In the centerline.
          {
              {50. * std::sqrt(2.), 0., 0.} /* lane_position */,
              {50., 0., 50.} /* nearest_position */,
              0. /* distance */
          },
          // A the edge of the lane.
          {
              {50. * std::sqrt(2.), 2., 0.} /* lane_position */,
              {50., 2., 50.} /* nearest_position */,
              0. /* distance */
          },
          // Outside boundary of the lane.
          // Because of the scaling of the boundaries' linestring the r value is slightly different.
          {
              {50. * std::sqrt(2.), 2.066817, 0.} /* lane_position */,
              {50., 2.066817, 50.} /* nearest_position */,
              7.9331829811625667, /* distance */
          },
      } /* expected_lane_position_result */
  }};
}

class ToLaneSegmentPositionTest : public ::testing::TestWithParam<ToLaneSegmentPositionTestCase> {
 public:
  static constexpr double kTolerance{1.e-5};
  static constexpr double kScaleLength{1.};

  void SetUp() override {
    ASSERT_EQ(case_.backend_pos.size(), case_.expected_lane_position_result.size()) << ">>>>> Test case is ill-formed.";
  }

  const maliput::api::LaneId kLaneId{"dut id"};
  const maliput::api::HBounds kHBounds{-5., 5.};
  ToLaneSegmentPositionTestCase case_ = GetParam();
  std::unique_ptr<geometry::LaneGeometry> lane_geometry_ =
      std::make_unique<geometry::LaneGeometry>(case_.left, case_.right, kTolerance, kScaleLength);
};

TEST_P(ToLaneSegmentPositionTest, Test) {
  std::unique_ptr<Lane> dut = std::make_unique<Lane>(kLaneId, kHBounds, std::move(lane_geometry_));
  EXPECT_DOUBLE_EQ(case_.expected_length, dut->length());
  for (std::size_t i = 0; i < case_.backend_pos.size(); ++i) {
    const auto lane_position_result = dut->ToLanePositionBackend(case_.backend_pos[i]);
    IsLanePositionResultClose(case_.expected_lane_position_result[i], lane_position_result, kTolerance);
  }
}

INSTANTIATE_TEST_CASE_P(ToLaneSegmentPositionTestGroup, ToLaneSegmentPositionTest,
                        ::testing::ValuesIn(ToLaneSegmentPositionTestCases()));

}  // namespace
}  // namespace test
}  // namespace maliput_sparse
