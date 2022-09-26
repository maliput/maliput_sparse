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

#include <algorithm>
#include <set>
#include <utility>

#include "base/lane.h"
#include "base/road_geometry.h"

namespace maliput_sparse {
namespace builder {

LaneGeometryBuilder& LaneGeometryBuilder::LeftLineString(
    const maliput_sparse::geometry::LineString3d& left_line_string) {
  left_line_string_.emplace(left_line_string);
  return *this;
}

LaneGeometryBuilder& LaneGeometryBuilder::RightLineString(
    const maliput_sparse::geometry::LineString3d& right_line_string) {
  right_line_string_.emplace(right_line_string);
  return *this;
}

LaneBuilder& LaneGeometryBuilder::EndLaneGeometry() {
  MALIPUT_THROW_UNLESS(left_line_string_.has_value() && right_line_string_.has_value());
  const double linear_tolerance = Parent()->Parent()->Parent()->Parent()->linear_tolerance({});
  const double scale_length = Parent()->Parent()->Parent()->Parent()->scale_length({});
  auto lane_geometry = std::make_unique<maliput_sparse::geometry::LaneGeometry>(
      left_line_string_.value(), right_line_string_.value(), linear_tolerance, scale_length);
  Parent()->SetLaneGeometry({}, std::move(lane_geometry));
  return End();
}

LaneBuilder& LaneBuilder::Id(const maliput::api::LaneId& lane_id) {
  id_ = lane_id;
  return *this;
}

LaneBuilder& LaneBuilder::HeightBounds(const maliput::api::HBounds& hbounds) {
  hbounds_ = hbounds;
  return *this;
}

LaneGeometryBuilder LaneBuilder::StartLaneGeometry() { return LaneGeometryBuilder(this); }

SegmentBuilder& LaneBuilder::EndLane() {
  MALIPUT_THROW_UNLESS(lane_geometry_ != nullptr);
  auto lane = std::make_unique<Lane>(id_, hbounds_, std::move(lane_geometry_));
  Parent()->SetLane({}, std::move(lane));
  return End();
}

void LaneBuilder::SetLaneGeometry(maliput::common::Passkey<LaneGeometryBuilder>,
                                  std::unique_ptr<maliput_sparse::geometry::LaneGeometry> lane_geometry) {
  MALIPUT_THROW_UNLESS(lane_geometry != nullptr);
  lane_geometry_ = std::move(lane_geometry);
}

SegmentBuilder& SegmentBuilder::Id(const maliput::api::SegmentId& segment_id) {
  id_ = segment_id;
  return *this;
}

LaneBuilder SegmentBuilder::StartLane() { return LaneBuilder(this); }

JunctionBuilder& SegmentBuilder::EndSegment() {
  MALIPUT_THROW_UNLESS(!lanes_.empty());
  auto segment = std::make_unique<maliput::geometry_base::Segment>(id_);
  for (std::unique_ptr<maliput::geometry_base::Lane>& lane : lanes_) {
    segment->AddLane(std::move(lane));
  }
  Parent()->SetSegment({}, std::move(segment));
  return End();
}

void SegmentBuilder::SetLane(maliput::common::Passkey<LaneBuilder>,
                             std::unique_ptr<maliput::geometry_base::Lane> lane) {
  MALIPUT_THROW_UNLESS(lane != nullptr);
  lanes_.emplace_back(std::move(lane));
}

JunctionBuilder& JunctionBuilder::Id(const maliput::api::JunctionId& junction_id) {
  id_ = junction_id;
  return *this;
}

SegmentBuilder JunctionBuilder::StartSegment() { return SegmentBuilder(this); }

RoadGeometryBuilder& JunctionBuilder::EndJunction() {
  MALIPUT_THROW_UNLESS(!segments_.empty());
  auto junction = std::make_unique<maliput::geometry_base::Junction>(id_);
  for (std::unique_ptr<maliput::geometry_base::Segment>& segment : segments_) {
    junction->AddSegment(std::move(segment));
  }
  Parent()->SetJunction({}, std::move(junction));
  return End();
}

void JunctionBuilder::SetSegment(maliput::common::Passkey<SegmentBuilder>,
                                 std::unique_ptr<maliput::geometry_base::Segment> segment) {
  MALIPUT_THROW_UNLESS(segment != nullptr);
  segments_.push_back(std::move(segment));
}


BranchPointBuilder& BranchPointBuilder::Connect(
    const maliput::api::LaneId& lane_id_a, const maliput::api::LaneEnd::Which which_a,
    const maliput::api::LaneId& lane_id_b, const maliput::api::LaneEnd::Which which_b) {
  const LaneEnd lane_end_a = std::make_pair(lane_id_a, which_a);
  const LaneEnd lane_end_b = std::make_pair(lane_id_a, which_b);
  lane_ends_.emplace(lane_end_a, lane_end_b);
  lane_ends_.emplace(lane_end_b, lane_end_a);
}

namespace {
// Holds the A and B side sets of LaneEnds for a future maliput::geometry_base::BranchPoint.
struct BranchPointSets {
  std::set<BranchPointBuilder::LaneEnd> a_side{};
  std::set<BranchPointBuilder::LaneEnd> b_side{};
};

// Proivdes maliput::api::BranchPointId with an increasing counter to make each ID
// that an instance provides pragmatically unique.
// This is not true when the number of requests exceeds the int maximum.
class BranchPointIdBuilder {
 public:
  BranchPointIdBuilder() = default;

  maliput::api::BranchPointId operator()() {
    return maliput::api::BranchPointId(std::to_string(branch_point_count_++));
  }

 private:
  int branch_point_count_{0};
};
  
// Returns a set of BranchPointBuilder::LaneEnds for a given @p key by collecting all the connections
// in @p lane_ends.
std::set<BranchPointBuilder::LaneEnd> GetLaneEndSet(
    const BranchPointBuilder::LaneEnd& key
    const std::unordered_multimap<BranchPointBuilder::LaneEnd, BranchPointBuilder::LaneEnd>& lane_ends) {
  auto its = lane_ends.equal_range(key);
  if (its.first == lane_ends.end()) {
    return {};
  }
  return std::set<BranchPointBuilder::LaneEnd>{its.first, its.second};
}

// @brief Updates @p branch_point_sets based on the @p lane_end under evaluation.
// @details When there are no BranchPointSets with @p lane_end in either the A or B side, a new
// BranchPointSets is inserted using @p lane_end as the only element on the A side.
// When @p lane_ends appears on the A side of a BranchPointSets, the B side of such entity is updated.
// When @p lane_ends appears on the B side of a BranchPointSets, the A side of such entity is updated.
// This function uses @p lane_ends to get the set of BranchPointBuilder::LaneEnd a @p lane_end is connected to.
void UpdateBranchPointSetsFor(
    const BranchPointBuilder::LaneEnd& lane_end,
    const std::unordered_multimap<BranchPointBuilder::LaneEnd, BranchPointBuilder::LaneEnd>& lane_ends,
    std::set<BranchPointSets>* branch_point_sets) {
  
  auto it_a_side = std::find_if(
        branch_point_sets->begin(), branch_point_sets->end(),
        [lane_end](const BranchPointSets& branch_point_sets) { return branch_point_sets.a_side.find(lane_end); });
   
  auto it_b_side = std::find_if(
        branch_point_sets->begin(), branch_point_sets->end(),
        [lane_end](const BranchPointSets& branch_point_sets) { return branch_point_sets.b_side.find(lane_end); });
  
  if (it_a_side == branch_point_sets->end() && it_b_side == branch_point_sets->end()) {
    BranchPointSets branch_point_set;
    branch_point_set.a_side.emplace(lane_end);
    branch_point_set.b_side = GetLaneEndSet(lane_end, lane_ends);
    branch_point_sets->emplace(branch_point_set);
  } else if (it_a_side != branch_point_sets->end()) {
    auto b_side_set = GetLaneEndSet(lane_end, lane_ends);
    it_a_side->b_side.insert(b_side_set.begin(), b_side_set.end());
  } else if (it_b_side != branch_point_sets->end()) {
    auto a_side_set = GetLaneEndSet(lane_end, lane_ends);
    it_b_side->b_side.insert(a_side_set.begin(), a_side_set.end());
  }
}

// Functor that makes a maliput::geometry_base::BranchPoint from a BranchPointSets.
// Note: default branch is selected by picking the first value in the opposing side.
struct MakeBranchPointFromSets {
  std::unique_ptr<maliput::geometry_base::BranchPoint> operator()(const BranchPointSets& branch_point_set) {
    auto branch_point = std::make_unique<maliput::geometry_base::BranchPoint>(branch_point_id_builder_());
    // Populate the A and B side.
    std::for_each(branch_point_set.a_side.begin(), branch_point_set.a_side.end(),
                  [&](BranchPointBuilder::LaneEnd lane_end) {
                    maliput::geometry_base::Lane* lane = lanes_->at(lane_end.first);
                    branch_point->AddABranch(lane, lane_end.second);
                  });
    std::for_each(branch_point_set.b_side.begin(), branch_point_set.b_side.end(),
                  [&](BranchPointBuilder::LaneEnd lane_end) {
                    maliput::geometry_base::Lane* lane = lanes_->at(lane_end.first);
                    branch_point->AddBBranch(lane, lane_end.second);
                  });
    // Set the default branches for each LaneEnd.
    const maliput::api::LaneEndSet* a_side_set = branch_point->GetASide();
    const maliput::api::LaneEndSet* b_side_set = branch_point->GetBSide();
    // We expect to have at least one in A side.
    MALIPUT_THROW_UNLESS(a_side_set->size() > 0);
    for (int i = 0; i < b_side_set->size(); ++i) {
      branch_point->SetDefault(b_side_set->get(i), a_side_set->get(0));
    }
    if (b_side_set->size() > 0) {
      for (int i = 0; i < a_side_set->size(); ++i) {
        branch_point->SetDefault(a_side_set->get(i), b_side_set->get(0));
      }
    }

    return std::move(branch_point);
  }

  const std::unordered_map<maliput::api::LaneId, maliput::geometry_base::Lane*> lanes_;
  BranchPointIdBuilder branch_point_id_builder_;
};

} // namespace

RoadGeometryBuilder& BranchPointBuilder::EndBranchPoints() {
  // Creates set of BranchPointSets from the dictionary of Lanes.
  const std::unordered_map<maliput::api::LaneId, maliput::geometry_base::Lane*> lanes = Parent()->GetLanes();
  std::set<BranchPointSets> branch_point_sets;
  for (const auto& it : lanes) {
    UpdateBranchPointSetsFor(
        std::make_pair(it->first, maliput::api::LaneEnd::Which::kStart), lane_ends_, &branch_point_sets);
    UpdateBranchPointSetsFor(
        std::make_pair(it->fist, maliput::api::LaneEnd::Which::kFinish), lane_ends_, &branch_point_sets);
  }

  // Transforms the set of BranchPointSets into a vector of maliput::geometry_base::BranchPoints
  std::vector<std::unique_ptr<maliput::geometry_base::BranchPoint>> branch_points(branch_point_sets.size());
  std::transform(branch_point_sets.begin(), branch_point_sets.end(), std::back_inserter(branch_points),
                 MakeBranchPointFromSets{lanes, BranchPointIdBuilder()});

  // Registers the BranchPoints into the RoadGeometry.
  Parent()->SetBranchPoints({}, std::move(branch_points));
  return End();
}


RoadGeometryBuilder& RoadGeometryBuilder::Id(const maliput::api::RoadGeometryId& road_geometry_id) {
  id_ = road_geometry_id;
  return *this;
}

RoadGeometryBuilder& RoadGeometryBuilder::LinearTolerance(double linear_tolerance) {
  MALIPUT_THROW_UNLESS(linear_tolerance > 0.);
  linear_tolerance_ = linear_tolerance;
  return *this;
}

RoadGeometryBuilder& RoadGeometryBuilder::AngularTolerance(double angular_tolerance) {
  MALIPUT_THROW_UNLESS(angular_tolerance > 0.);
  angular_tolerance_ = angular_tolerance;
  return *this;
}

RoadGeometryBuilder& RoadGeometryBuilder::ScaleLength(double scale_length) {
  MALIPUT_THROW_UNLESS(scale_length > 0.);
  scale_length_ = scale_length;
  return *this;
}

RoadGeometryBuilder& RoadGeometryBuilder::InertialToBackendFrameTranslation(const maliput::math::Vector3& translation) {
  inertial_to_backend_frame_translation_ = translation;
  return *this;
}

JunctionBuilder RoadGeometryBuilder::StartJunction() { return JunctionBuilder(this); }

BranchPointBuilder RoadGeometryBuilder::StartBranchPoints() { return BranchPointBuilder(this); }

std::unique_ptr<maliput::api::RoadGeometry> RoadGeometryBuilder::Build() {
  MALIPUT_THROW_UNLESS(!junctions_.empty());
  MALIPUT_THROW_UNLESS(!branch_points_.empty());
  auto road_geometry = std::make_unique<RoadGeometry>(id_, linear_tolerance_, angular_tolerance_, scale_length_,
                                                      inertial_to_backend_frame_translation_);
  for (std::unique_ptr<maliput::geometry_base::Junction>& junction : junctions_) {
    road_geometry->AddJunction(std::move(junction));
  }
  for (std::unique_ptr<maliput::geometry_base::BranchPoint>& branch_point : branch_points_) {
    road_geometry->AddBranchPoint(std::move(branch_point));
  }
  return road_geometry;
}

void RoadGeometryBuilder::SetJunction(maliput::common::Passkey<JunctionBuilder>,
                                      std::unique_ptr<maliput::geometry_base::Junction> junction) {
  MALIPUT_THROW_UNLESS(junction != nullptr);
  junctions_.push_back(std::move(junction));
}


void RoadGeometryBuilder::SetBranchPoints(
    maliput::common::Passkey<BranchPointBuilder>,
    std::vector<std::unique_ptr<maliput::geometry_base::BranchPoint>>&& branch_points) {
  std::for_each(branch_points.begin(), branch_points.end(), [](const auto& bp) { MALIPUT_THROW_UNLESS(bp != nullptr)});
  branch_points_ = std::move(branch_points);
}

std::unordered_map<maliput::api::LaneId, maliput::geometry_base::Lane*> RoadGeometryBuilder::GetLanes(
    maliput::common::Passkey<BranchPointBuilder>) const {
  std::unordered_map<maliput::api::LaneId, maliput::geometry_base::Lane*> lanes;
  std::for_each(junctions_.begin(), junctions_.end(),
      [lanes](const auto junction) {
        for (int i = 0; i < junction->num_segments(); ++i) {
          const maliput::api::Segment* segment = junction->segment(j);
          for (int j = 0; j < segment->num_lanes(); ++j) {
            maliput::geometry_base::Lane* lane = segment->lane(j);
            lanes[lane->id()] = lane;
          }
        }
      });
  return lanes;
}


}  // namespace builder
}  // namespace maliput_sparse
