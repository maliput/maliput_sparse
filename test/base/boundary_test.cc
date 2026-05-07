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
#include <maliput/api/lane_marking.h>
#include <maliput/geometry_base/segment.h>
#include <maliput/math/vector.h>

#include "base/lane.h"
#include "base/lane_boundary.h"
#include "maliput_sparse/geometry/lane_geometry.h"
#include "maliput_sparse/geometry/line_string.h"
#include "maliput_sparse/parser/lane_marking.h"

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

GTEST_TEST(LaneBoundaryMarkingTest, GetMarkingAtPosition) {
  maliput::geometry_base::Segment segment{maliput::api::SegmentId{"seg"}};

  const LineString3d left{{Vector3{0., 0., 0.}, Vector3{10., 0., 0.}}};
  const LineString3d right{{Vector3{0., -4., 0.}, Vector3{10., -4., 0.}}};
  const maliput::api::Lane* lane = segment.AddLane(MakeLane("lane_0", left, right));

  const parser::LaneMarking solid_white{maliput::api::LaneMarkingType::kSolid, maliput::api::LaneMarkingColor::kWhite,
                                        maliput::api::LaneMarkingWeight::kStandard};
  const parser::LaneMarking dashed_yellow{maliput::api::LaneMarkingType::kBroken,
                                          maliput::api::LaneMarkingColor::kYellow,
                                          maliput::api::LaneMarkingWeight::kStandard};

  const std::vector<parser::BoundaryMarkings> markings{
      parser::BoundaryMarkings{0., 5., solid_white},
      parser::BoundaryMarkings{5., 10., dashed_yellow},
  };
  const auto* boundary = segment.AddBoundary(
      std::make_unique<LaneBoundary>(maliput::api::LaneBoundary::Id{"seg_boundary_0"}, lane, nullptr, markings));

  // Query inside first marking range.
  const auto result_at_2 = boundary->GetMarking(2.);
  ASSERT_TRUE(result_at_2.has_value());
  EXPECT_EQ(maliput::api::LaneMarkingType::kSolid, result_at_2->marking.type);
  EXPECT_EQ(maliput::api::LaneMarkingColor::kWhite, result_at_2->marking.color);
  EXPECT_DOUBLE_EQ(0., result_at_2->s_start);
  EXPECT_DOUBLE_EQ(5., result_at_2->s_end);

  // Query inside second marking range.
  const auto result_at_7 = boundary->GetMarking(7.);
  ASSERT_TRUE(result_at_7.has_value());
  EXPECT_EQ(maliput::api::LaneMarkingType::kBroken, result_at_7->marking.type);
  EXPECT_EQ(maliput::api::LaneMarkingColor::kYellow, result_at_7->marking.color);
  EXPECT_DOUBLE_EQ(5., result_at_7->s_start);
  EXPECT_DOUBLE_EQ(10., result_at_7->s_end);

  // Query outside all markings.
  EXPECT_FALSE(boundary->GetMarking(15.).has_value());
}

GTEST_TEST(LaneBoundaryMarkingTest, GetAllMarkings) {
  maliput::geometry_base::Segment segment{maliput::api::SegmentId{"seg"}};

  const LineString3d left{{Vector3{0., 0., 0.}, Vector3{10., 0., 0.}}};
  const LineString3d right{{Vector3{0., -4., 0.}, Vector3{10., -4., 0.}}};
  const maliput::api::Lane* lane = segment.AddLane(MakeLane("lane_0", left, right));

  const parser::LaneMarking solid_white{maliput::api::LaneMarkingType::kSolid, maliput::api::LaneMarkingColor::kWhite,
                                        maliput::api::LaneMarkingWeight::kStandard};
  const parser::LaneMarking dashed_yellow{maliput::api::LaneMarkingType::kBroken,
                                          maliput::api::LaneMarkingColor::kYellow,
                                          maliput::api::LaneMarkingWeight::kStandard};

  const std::vector<parser::BoundaryMarkings> markings{
      parser::BoundaryMarkings{0., 5., solid_white},
      parser::BoundaryMarkings{5., 10., dashed_yellow},
  };
  const auto* boundary = segment.AddBoundary(
      std::make_unique<LaneBoundary>(maliput::api::LaneBoundary::Id{"seg_boundary_0"}, lane, nullptr, markings));

  const auto all = boundary->GetMarkings();
  ASSERT_EQ(2u, all.size());
  EXPECT_EQ(maliput::api::LaneMarkingType::kSolid, all[0].marking.type);
  EXPECT_DOUBLE_EQ(0., all[0].s_start);
  EXPECT_DOUBLE_EQ(5., all[0].s_end);
  EXPECT_EQ(maliput::api::LaneMarkingType::kBroken, all[1].marking.type);
  EXPECT_DOUBLE_EQ(5., all[1].s_start);
  EXPECT_DOUBLE_EQ(10., all[1].s_end);
}

GTEST_TEST(LaneBoundaryMarkingTest, GetMarkingsInRange) {
  maliput::geometry_base::Segment segment{maliput::api::SegmentId{"seg"}};

  const LineString3d left{{Vector3{0., 0., 0.}, Vector3{20., 0., 0.}}};
  const LineString3d right{{Vector3{0., -4., 0.}, Vector3{20., -4., 0.}}};
  const maliput::api::Lane* lane = segment.AddLane(MakeLane("lane_0", left, right));

  const parser::LaneMarking m1{maliput::api::LaneMarkingType::kSolid, maliput::api::LaneMarkingColor::kWhite,
                               maliput::api::LaneMarkingWeight::kStandard};
  const parser::LaneMarking m2{maliput::api::LaneMarkingType::kBroken, maliput::api::LaneMarkingColor::kYellow,
                               maliput::api::LaneMarkingWeight::kStandard};
  const parser::LaneMarking m3{maliput::api::LaneMarkingType::kSolidSolid, maliput::api::LaneMarkingColor::kWhite,
                               maliput::api::LaneMarkingWeight::kBold};

  const std::vector<parser::BoundaryMarkings> markings{
      parser::BoundaryMarkings{0., 5., m1},
      parser::BoundaryMarkings{5., 15., m2},
      parser::BoundaryMarkings{15., 20., m3},
  };
  const auto* boundary = segment.AddBoundary(
      std::make_unique<LaneBoundary>(maliput::api::LaneBoundary::Id{"seg_boundary_0"}, lane, nullptr, markings));

  // Range overlapping first two markings.
  const auto range_results = boundary->GetMarkings(3., 10.);
  ASSERT_EQ(2u, range_results.size());
  EXPECT_EQ(maliput::api::LaneMarkingType::kSolid, range_results[0].marking.type);
  EXPECT_EQ(maliput::api::LaneMarkingType::kBroken, range_results[1].marking.type);

  // Range covering only the middle marking.
  const auto mid_results = boundary->GetMarkings(6., 14.);
  ASSERT_EQ(1u, mid_results.size());
  EXPECT_EQ(maliput::api::LaneMarkingType::kBroken, mid_results[0].marking.type);

  // Range outside all markings.
  EXPECT_TRUE(boundary->GetMarkings(20., 30.).empty());
}

}  // namespace
}  // namespace test
}  // namespace maliput_sparse
