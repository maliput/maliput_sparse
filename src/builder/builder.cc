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

std::unique_ptr<maliput::api::RoadGeometry> RoadGeometryBuilder::Build() {
  MALIPUT_THROW_UNLESS(!junctions_.empty());
  auto road_geometry = std::make_unique<RoadGeometry>(id_, linear_tolerance_, angular_tolerance_, scale_length_,
                                                      inertial_to_backend_frame_translation_);
  for (std::unique_ptr<maliput::geometry_base::Junction>& junction : junctions_) {
    road_geometry->AddJunction(std::move(junction));
  }
  return road_geometry;
}

void RoadGeometryBuilder::SetJunction(maliput::common::Passkey<JunctionBuilder>,
                                      std::unique_ptr<maliput::geometry_base::Junction> junction) {
  MALIPUT_THROW_UNLESS(junction != nullptr);
  junctions_.push_back(std::move(junction));
}

}  // namespace builder
}  // namespace maliput_sparse