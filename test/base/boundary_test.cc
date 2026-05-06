// BSD 3-Clause License
//
// Copyright (c) 2022-2026, Woven by Toyota. All rights reserved.
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

#include <memory>

#include <gtest/gtest.h>
#include <maliput/geometry_base/segment.h>
#include <maliput/math/vector.h>

#include "base/lane.h"
#include "base/lane_boundary.h"
#include "maliput_sparse/geometry/lane_geometry.h"
#include "maliput_sparse/geometry/line_string.h"

namespace maliput_sparse {
namespace test {
namespace {

using maliput::math::Vector3;
using maliput_sparse::geometry::LineString3d;

std::unique_ptr<Lane> MakeLane(const std::string& lane_id, const LineString3d& left, const LineString3d& right) {
  static constexpr double kTolerance{1e-3};
  static constexpr double kScaleLength{1.};
  auto lane_geometry = std::make_unique<geometry::LaneGeometry>(left, right, kTolerance, kScaleLength);
  return std::make_unique<Lane>(maliput::api::LaneId{lane_id}, maliput::api::HBounds{-5., 5.},
                                std::move(lane_geometry));
}

GTEST_TEST(LaneBoundaryTest, SegmentBoundaryAdjacencyAndIndexing) {
  maliput::geometry_base::Segment segment{maliput::api::SegmentId{"seg"}};

  const LineString3d rightmost_left{{Vector3{0., 0., 0.}, Vector3{10., 0., 0.}}};
  const LineString3d rightmost_right{{Vector3{0., -4., 0.}, Vector3{10., -4., 0.}}};
  const LineString3d leftmost_left{{Vector3{0., 4., 0.}, Vector3{10., 4., 0.}}};
  const LineString3d leftmost_right{{Vector3{0., 0., 0.}, Vector3{10., 0., 0.}}};

  const maliput::api::Lane* lane_0 = segment.AddLane(MakeLane("lane_0", rightmost_left, rightmost_right));
  const maliput::api::Lane* lane_1 = segment.AddLane(MakeLane("lane_1", leftmost_left, leftmost_right));

  const auto* boundary_0 = segment.AddBoundary(
      std::make_unique<LaneBoundary>(maliput::api::LaneBoundary::Id{"seg_boundary_0"}, lane_0, nullptr));
  const auto* boundary_1 = segment.AddBoundary(
      std::make_unique<LaneBoundary>(maliput::api::LaneBoundary::Id{"seg_boundary_1"}, lane_1, lane_0));
  const auto* boundary_2 = segment.AddBoundary(
      std::make_unique<LaneBoundary>(maliput::api::LaneBoundary::Id{"seg_boundary_2"}, nullptr, lane_1));

  ASSERT_EQ(2, segment.num_lanes());
  ASSERT_EQ(3, segment.num_boundaries());

  EXPECT_EQ(maliput::api::LaneBoundary::Id{"seg_boundary_0"}, boundary_0->id());
  EXPECT_EQ(maliput::api::LaneBoundary::Id{"seg_boundary_1"}, boundary_1->id());
  EXPECT_EQ(maliput::api::LaneBoundary::Id{"seg_boundary_2"}, boundary_2->id());

  EXPECT_EQ(&segment, boundary_0->segment());
  EXPECT_EQ(&segment, boundary_1->segment());
  EXPECT_EQ(&segment, boundary_2->segment());

  EXPECT_EQ(0, boundary_0->index());
  EXPECT_EQ(1, boundary_1->index());
  EXPECT_EQ(2, boundary_2->index());

  EXPECT_EQ(lane_0, boundary_0->lane_to_left());
  EXPECT_EQ(nullptr, boundary_0->lane_to_right());

  EXPECT_EQ(lane_1, boundary_1->lane_to_left());
  EXPECT_EQ(lane_0, boundary_1->lane_to_right());

  EXPECT_EQ(nullptr, boundary_2->lane_to_left());
  EXPECT_EQ(lane_1, boundary_2->lane_to_right());

  EXPECT_EQ(boundary_0, lane_0->right_boundary());
  EXPECT_EQ(boundary_1, lane_0->left_boundary());
  EXPECT_EQ(boundary_1, lane_1->right_boundary());
  EXPECT_EQ(boundary_2, lane_1->left_boundary());

  EXPECT_FALSE(boundary_0->GetMarking(0.).has_value());
  EXPECT_TRUE(boundary_0->GetMarkings().empty());
}

}  // namespace
}  // namespace test
}  // namespace maliput_sparse
