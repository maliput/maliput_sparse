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

namespace maliput_sparse {
namespace builder {
namespace {

GTEST_TEST(Builder, Stub) {
  const maliput::api::RoadGeometryId kRoadGeometryId("custom_rg_id");
  constexpr double kLinearTolerance{1.};
  constexpr double kAngularTolerance{2.};
  constexpr double kScaleLength{2.};
  const maliput::math::Vector3 kInertialToBackendFrameTranslation(4., 5., 6.);
  const maliput::api::JunctionId kJunctionId("custom_j_id");
  const maliput::api::SegmentId kSegmentAId("segment_a");
  const maliput::api::LaneId kLaneAId("lane_a");
  const maliput::api::LaneId kLaneBId("lane_b");
  const maliput::api::SegmentId kSegmentBId("segment_b");
  const maliput::api::LaneId kLaneCId("lane_c");

  RoadGeometryBuilder dut;

  // clang-format off
  std::unique_ptr<maliput::api::RoadGeometry> rg =
      dut.Id(kRoadGeometryId)
          .LinearTolerance(kLinearTolerance)
          .AngularTolerance(kAngularTolerance)
          .ScaleLength(kScaleLength)
          .InertialToBackendFrameTranslation(kInertialToBackendFrameTranslation)
          .StartJunction()
              .Id(kJunctionId)
              .StartSegment()
                      .Id(kSegmentAId)
                  .StartLane()
                      .Id(kLaneAId)
                  .EndLane()
                  .StartLane()
                      .Id(kLaneBId)
                  .EndLane()
              .EndSegment()
              .StartSegment()
                  .Id(kSegmentBId)
                  .StartLane()
                      .Id(kLaneCId)
                  .EndLane()
              .EndSegment()
          .EndJunction()
          .Build();
  // clang-format on

  ASSERT_NE(nullptr, rg);
  ASSERT_EQ(kRoadGeometryId, rg->id());
  ASSERT_EQ(1, rg->num_junctions());

  auto* junction = rg->junction(0);
  ASSERT_NE(nullptr, junction);
  ASSERT_EQ(kJunctionId, junction->id());
  ASSERT_EQ(2, junction->num_segments());

  auto* segment_a = junction->segment(0);
  ASSERT_NE(nullptr, segment_a);
  ASSERT_EQ(kSegmentAId, segment_a->id());
  ASSERT_EQ(2, segment_a->num_lanes());

  auto* lane_a = segment_a->lane(0);
  ASSERT_NE(nullptr, lane_a);
  ASSERT_EQ(kLaneAId, lane_a->id());

  auto* lane_b = segment_a->lane(1);
  ASSERT_NE(nullptr, lane_b);
  ASSERT_EQ(kLaneBId, lane_b->id());

  auto* segment_b = junction->segment(1);
  ASSERT_NE(nullptr, segment_a);
  ASSERT_EQ(kSegmentAId, segment_a->id());
  ASSERT_EQ(2, segment_a->num_lanes());

  auto* lane_c = segment_b->lane(0);
  ASSERT_NE(nullptr, lane_c);
  ASSERT_EQ(kLaneCId, lane_c->id());
}

}  // namespace
}  // namespace builder
}  // namespace maliput_sparse
