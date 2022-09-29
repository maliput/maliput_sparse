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
#include "maliput_sparse/builder/builder.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <maliput/api/junction.h>
#include <maliput/api/lane.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/segment.h>
#include <maliput/common/assertion_error.h>
#include <maliput/math/vector.h>
#include <maliput/test_utilities/maliput_types_compare.h>

#include "maliput_sparse/geometry/lane_geometry.h"
#include "maliput_sparse/geometry/line_string.h"

namespace maliput_sparse {
namespace builder {
namespace test {
namespace {
using maliput::api::test::IsHBoundsClose;
using maliput::math::Vector3;
using maliput_sparse::geometry::LineString3d;

GTEST_TEST(RoadGeometryBuilderTest, LinearToleranceConstraintFails) {
  EXPECT_THROW(RoadGeometryBuilder().LinearTolerance(-1.0), maliput::common::assertion_error);
  EXPECT_THROW(RoadGeometryBuilder().LinearTolerance(0.0), maliput::common::assertion_error);
}

GTEST_TEST(RoadGeometryBuilderTest, AngularToleranceConstraintFails) {
  EXPECT_THROW(RoadGeometryBuilder().AngularTolerance(-1.0), maliput::common::assertion_error);
  EXPECT_THROW(RoadGeometryBuilder().AngularTolerance(0.0), maliput::common::assertion_error);
}

GTEST_TEST(RoadGeometryBuilderTest, ScaleLengthConstraintFails) {
  EXPECT_THROW(RoadGeometryBuilder().ScaleLength(-1.0), maliput::common::assertion_error);
  EXPECT_THROW(RoadGeometryBuilder().ScaleLength(0.0), maliput::common::assertion_error);
}

GTEST_TEST(RoadGeometryBuilderTest, RoadGeometryBuilderWithoutJunctions) {
  EXPECT_THROW(RoadGeometryBuilder().Build(), maliput::common::assertion_error);
}

GTEST_TEST(RoadGeometryBuilderTest, JunctionBuilderWithoutSegments) {
  const maliput::api::JunctionId kJunctionId("junction_a");
  EXPECT_THROW(
      // clang-format off
      RoadGeometryBuilder()
          .StartJunction()
              .Id(kJunctionId)
          .EndJunction()
      // clang-format on
      ,
      maliput::common::assertion_error);
}

GTEST_TEST(RoadGeometryBuilderTest, SegmentBuilderWithoutLane) {
  const maliput::api::JunctionId kJunctionId("junction_a");
  const maliput::api::SegmentId kSegmentId("segment_a");
  EXPECT_THROW(
      // clang-format off
      RoadGeometryBuilder()
          .StartJunction()
              .Id(kJunctionId)
              .StartSegment()
                  .Id(kSegmentId)
              .EndSegment()
      // clang-format on
      ,
      maliput::common::assertion_error);
}

GTEST_TEST(RoadGeometryBuilderTest, LaneBuilderWithoutLaneGeometry) {
  const maliput::api::JunctionId kJunctionId("junction_a");
  const maliput::api::SegmentId kSegmentId("segment_a");
  const maliput::api::LaneId kLaneId("lane_a");

  EXPECT_THROW(
      // clang-format off
      RoadGeometryBuilder()
          .StartJunction()
              .Id(kJunctionId)
              .StartSegment()
                  .Id(kSegmentId)
                  .StartLane()
                    .Id(kLaneId)
                  .EndLane()
              .EndSegment()
      // clang-format on
      ,
      maliput::common::assertion_error);
}

GTEST_TEST(RoadGeometryBuilderTest, LaneGeometryBuilderWithMissingLineStrings) {
  const maliput::api::JunctionId kJunctionId("junction_a");
  const maliput::api::SegmentId kSegmentId("segment_a");
  const maliput::api::LaneId kLaneId("lane_a");
  const LineString3d kLeftLineString{Vector3{0., 0., 0.}, Vector3{10., 0., 0.}};
  const LineString3d kRightLineString{Vector3{0., 5., 0.}, Vector3{10., 5., 0.}};

  // Fails because of missing left and right LineString3d.
  EXPECT_THROW(
      // clang-format off
      RoadGeometryBuilder()
          .StartJunction()
              .Id(kJunctionId)
              .StartSegment()
                  .Id(kSegmentId)
                  .StartLane()
                    .Id(kLaneId)
                    .StartLaneGeometry()
                    .EndLaneGeometry()
                  .EndLane()
              .EndSegment()
      // clang-format on
      ,
      maliput::common::assertion_error);
  // Fails because of missing right LineString3d.
  EXPECT_THROW(
      // clang-format off
      RoadGeometryBuilder()
          .StartJunction()
              .Id(kJunctionId)
              .StartSegment()
                  .Id(kSegmentId)
                  .StartLane()
                    .Id(kLaneId)
                    .StartLaneGeometry()
                        .LeftLineString(kLeftLineString)
                    .EndLaneGeometry()
                  .EndLane()
              .EndSegment()
      // clang-format on
      ,
      maliput::common::assertion_error);
  // Fails because of missing left LineString3d.
  EXPECT_THROW(
      // clang-format off
      RoadGeometryBuilder()
          .StartJunction()
              .Id(kJunctionId)
              .StartSegment()
                  .Id(kSegmentId)
                  .StartLane()
                    .Id(kLaneId)
                    .StartLaneGeometry()
                        .RightLineString(kRightLineString)
                    .EndLaneGeometry()
                  .EndLane()
              .EndSegment()
      // clang-format on
      ,
      maliput::common::assertion_error);
}

// Evaluates a case where all calls are executed at once and none of them fail their invariants.
GTEST_TEST(RoadGeometryBuilderTest, CompleteCase) {
  static constexpr double kEqualityTolerance{0.};

  const maliput::api::RoadGeometryId kRoadGeometryId("custom_rg_id");
  static constexpr double kLinearTolerance{1.};
  static constexpr double kAngularTolerance{2.};
  static constexpr double kScaleLength{3.};
  const maliput::math::Vector3 kInertialToBackendFrameTranslation(4., 5., 6.);
  const maliput::api::JunctionId kJunctionAId("junction_a");
  const maliput::api::SegmentId kSegmentAId("segment_a");
  const maliput::api::LaneId kLaneAId("lane_a");
  const maliput::api::LaneId kLaneBId("lane_b");
  const maliput::api::SegmentId kSegmentBId("segment_b");
  const maliput::api::LaneId kLaneCId("lane_c");
  const maliput::api::JunctionId kJunctionBId("unset_id");
  const maliput::api::SegmentId kSegmentCId("unset_id");
  const maliput::api::LaneId kLaneDId("unset_id");
  const LineString3d kLeftLineStringA{Vector3{0., 0., 0.}, Vector3{10., 0., 0.}};
  const LineString3d kRigthLineStringA{Vector3{0., 5., 0.}, Vector3{10., 5., 0.}};
  const LineString3d kLeftLineStringB{Vector3{0., 5., 0.}, Vector3{10., 5., 0.}};
  const LineString3d kRigthLineStringB{Vector3{0., 10., 0.}, Vector3{10., 10., 0.}};
  const LineString3d kLeftLineStringC{Vector3{0., 0., 0.}, Vector3{0., 10., 0.}};
  const LineString3d kRigthLineStringC{Vector3{5., 0., 0.}, Vector3{5., 10., 0.}};
  const LineString3d kLeftLineStringD{Vector3{20., 0., 0.}, Vector3{30., 5., 0.}};
  const LineString3d kRigthLineStringD{Vector3{20., 0., 0.}, Vector3{30., 5., 0.}};
  const maliput::api::HBounds kHBoundsA{0., 1.};
  const maliput::api::HBounds kHBoundsB{-1., 2.};
  const maliput::api::HBounds kHBoundsC{-2., 3.};
  const maliput::api::HBounds kHBoundsD{0, 5.};

  RoadGeometryBuilder dut;

  // clang-format off
  std::unique_ptr<maliput::api::RoadGeometry> rg =
      dut.Id(kRoadGeometryId)
          .LinearTolerance(kLinearTolerance)
          .AngularTolerance(kAngularTolerance)
          .ScaleLength(kScaleLength)
          .InertialToBackendFrameTranslation(kInertialToBackendFrameTranslation)
          .StartJunction()
              .Id(kJunctionAId)
              .StartSegment()
                      .Id(kSegmentAId)
                  .StartLane()
                      .Id(kLaneAId)
                      .HeightBounds(kHBoundsA)
                      .StartLaneGeometry()
                          .LeftLineString(kLeftLineStringA)  
                          .RightLineString(kRigthLineStringA)
                      .EndLaneGeometry()
                  .EndLane()
                  .StartLane()
                      .Id(kLaneBId)
                      .HeightBounds(kHBoundsB)
                      .StartLaneGeometry()
                          .LeftLineString(kLeftLineStringB)  
                          .RightLineString(kRigthLineStringB)
                      .EndLaneGeometry()
                  .EndLane()
              .EndSegment()
              .StartSegment()
                  .Id(kSegmentBId)
                  .StartLane()
                      .Id(kLaneCId)
                      .HeightBounds(kHBoundsC)
                      .StartLaneGeometry()
                          .LeftLineString(kLeftLineStringC)  
                          .RightLineString(kRigthLineStringC)
                      .EndLaneGeometry()
                  .EndLane()
              .EndSegment()
          .EndJunction()
          .StartJunction()
              .StartSegment()
                  .StartLane()
                      .StartLaneGeometry()
                          .LeftLineString(kLeftLineStringD)  
                          .RightLineString(kRigthLineStringD)
                      .EndLaneGeometry()
                  .EndLane()
              .EndSegment()
          .EndJunction()
          .Build();
  // clang-format on

  ASSERT_NE(nullptr, rg);
  ASSERT_EQ(kRoadGeometryId, rg->id());
  ASSERT_EQ(2, rg->num_junctions());

  auto* junction_a = rg->junction(0);
  ASSERT_NE(nullptr, junction_a);
  ASSERT_EQ(kJunctionAId, junction_a->id());
  ASSERT_EQ(2, junction_a->num_segments());

  auto* segment_a = junction_a->segment(0);
  ASSERT_NE(nullptr, segment_a);
  ASSERT_EQ(kSegmentAId, segment_a->id());
  ASSERT_EQ(2, segment_a->num_lanes());

  auto* lane_a = segment_a->lane(0);
  ASSERT_NE(nullptr, lane_a);
  ASSERT_EQ(kLaneAId, lane_a->id());
  ASSERT_TRUE(IsHBoundsClose(kHBoundsA, lane_a->elevation_bounds(0., 0.), kEqualityTolerance));

  auto* lane_b = segment_a->lane(1);
  ASSERT_NE(nullptr, lane_b);
  ASSERT_EQ(kLaneBId, lane_b->id());
  ASSERT_TRUE(IsHBoundsClose(kHBoundsB, lane_b->elevation_bounds(0., 0.), kEqualityTolerance));

  ASSERT_EQ(lane_a, lane_b->to_right());
  ASSERT_EQ(lane_b, lane_a->to_left());

  auto* segment_b = junction_a->segment(1);
  ASSERT_NE(nullptr, segment_a);
  ASSERT_EQ(kSegmentAId, segment_a->id());
  ASSERT_EQ(2, segment_a->num_lanes());

  auto* lane_c = segment_b->lane(0);
  ASSERT_NE(nullptr, lane_c);
  ASSERT_EQ(kLaneCId, lane_c->id());
  ASSERT_TRUE(IsHBoundsClose(kHBoundsC, lane_c->elevation_bounds(0., 0.), kEqualityTolerance));

  auto* junction_b = rg->junction(1);
  ASSERT_NE(nullptr, junction_b);
  ASSERT_EQ(kJunctionBId, junction_b->id());
  ASSERT_EQ(1, junction_b->num_segments());

  auto* segment_c = junction_b->segment(0);
  ASSERT_NE(nullptr, segment_c);
  ASSERT_EQ(kSegmentCId, segment_c->id());
  ASSERT_EQ(1, segment_c->num_lanes());

  auto* lane_d = segment_c->lane(0);
  ASSERT_NE(nullptr, lane_d);
  ASSERT_EQ(kLaneDId, lane_d->id());
  ASSERT_TRUE(IsHBoundsClose(kHBoundsD, lane_d->elevation_bounds(0., 0.), kEqualityTolerance));
}

}  // namespace
}  // namespace test
}  // namespace builder
}  // namespace maliput_sparse
