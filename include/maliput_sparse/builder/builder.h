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
#pragma once
/**
 * @file Builder API to construct a maliput::api::RoadGeometry.
 *
 * @details It allows simple construction of nested nodes in the maliput::api::RoadGeometry graph. The
 * geometry details are expected to be computed / loaded outside this method to allow parallelization
 * of that process. The graph construction is a synchronous operation though and requires
 * that the geometry information is available when building maliput::api::Lanes.
 *
 * A super simple 2-lane dragway could be constructed as follows:
 *
 * @code{cpp}
 * const maliput::math:Vector3 start_left_lane_1{0., 0., 0.};
 * const maliput::math:Vector3 end_left_lane_1{100., 0., 0.};
 * const maliput::math:Vector3 start_right_lane_1{0., 5., 0.};
 * const maliput::math:Vector3 end_right_lane_1{100., 5., 0.};
 * const maliput::math:Vector3 start_left_lane_2 = start_right_lane_1;
 * const maliput::math:Vector3 end_left_lane_2 = end_right_lane_1;
 * const maliput::math:Vector3 start_right_lane_2{0., 10., 0.};
 * const maliput::math:Vector3 end_right_lane_2{100., 10., 0.};
 *
 * LineString3d left_line_string_1{start_left_lane_1, end_left_lane_1};
 * LineString3d right_line_string_1{start_right_lane_1, end_right_lane_1};
 * LineString3d left_line_string_2{start_left_lane_2, end_left_lane_2};
 * LineString3d right_line_string_2{start_right_lane_2, end_right_lane_2};
 *
 * RoadGeometryBuilder rg_builder;
 * std::unique_ptr<RoadGeometry> road_geometry = RoadGeometryBuilder()
 *   .Id("two_lane_dragway")
 *   .StartJunction()
 *     .Id("j0")
 *     .StartSegment()
 *       .Id("j0_s0")
 *       .StartLane()
 *         .Id("j0_s0_l1")
 *         .StartLaneGeometry()
 *           .LeftLineString(left_line_string_1)
 *           .RightLineString(right_line_string_1)
 *         .EndLaneGeometry()
 *       .EndLane()
 *       .StartLane()
 *         .Id("j0_s0_l2")
 *         .StartLaneGeometry()
 *           .LeftLineString(left_line_string_2)
 *           .RightLineString(right_line_string_2)
 *         .EndLaneGeometry()
 *       .EndLane()
 *     .EndSegment()
 *   .EndJunction()
 *   .Build();
 * @endcode{cpp}
 */
#include <memory>
#include <string>
#include <vector>

#include <maliput/api/junction.h>
#include <maliput/api/lane.h>
#include <maliput/api/road_geometry.h>
#include <maliput/api/segment.h>
#include <maliput/common/passkey.h>
#include <maliput/geometry_base/junction.h>
#include <maliput/geometry_base/lane.h>
#include <maliput/geometry_base/road_geometry.h>
#include <maliput/geometry_base/segment.h>

#include "maliput_sparse/geometry/line_string.h"

namespace maliput_sparse {
namespace builder {
namespace details {

class BuilderBase {
 public:
  virtual ~BuilderBase() = default;
};

template <typename ParentT>
class NestedBuilder : public BuilderBase {
 public:
  explicit NestedBuilder(ParentT* parent) : parent_(parent) { MALIPUT_THROW_UNLESS(parent_ != nullptr); }

  ParentT& End() { return *parent_; }

  ParentT* Parent() { return parent_; }

 private:
  ParentT* parent_{};
};

}  // namespace details

class RoadGeometryBuilder;
class JunctionBuilder;
class SegmentBuilder;
class LaneBuilder;

/**
 * @brief Builder class for LaneGeometry.
 */
class LaneGeometryBuilder final : public details::NestedBuilder<LaneBuilder> {
 public:
  explicit LaneGeometryBuilder(LaneBuilder* parent) : details::NestedBuilder<LaneBuilder>(parent) {}

  // TODO(): implement me!
  // LaneGeometryBuilder& LeftLineString(LineString3d&& left_line_string);
  // LaneGeometryBuilder& RightLineString(LineString3d&& right_line_string);

  LaneBuilder& EndLaneGeometry() { return End(); }
};

/**
 * @brief Builder class for maliput::api::Lanes.
 */
class LaneBuilder final : public details::NestedBuilder<SegmentBuilder> {
 public:
  explicit LaneBuilder(SegmentBuilder* parent) : details::NestedBuilder<SegmentBuilder>(parent) {}

  LaneBuilder& Id(const maliput::api::LaneId& lane_id);
  LaneGeometryBuilder StartLaneGeometry();
  SegmentBuilder& EndLane();

  // TODO(): implement me!
  // void SetLaneGeometry(maliput::common::Passkey<LaneGeometryBuilder>, std::unique_ptr<LaneGeometry> lane_geometry);
 private:
  maliput::api::LaneId id_{"l_id"};
};

/**
 * @brief Builder class for maliput::api::Segments.
 */
class SegmentBuilder final : public details::NestedBuilder<JunctionBuilder> {
 public:
  explicit SegmentBuilder(JunctionBuilder* parent) : details::NestedBuilder<JunctionBuilder>(parent) {}

  SegmentBuilder& Id(const maliput::api::SegmentId& segment_id);
  LaneBuilder StartLane();
  JunctionBuilder& EndSegment();

  void SetLane(maliput::common::Passkey<LaneBuilder>, std::unique_ptr<maliput::geometry_base::Lane> lane);

 private:
  maliput::api::SegmentId id_{"s_id"};
  std::vector<std::unique_ptr<maliput::geometry_base::Lane>> lanes_{};
};

/**
 * @brief Builder class for maliput::api::Junctions.
 */
class JunctionBuilder final : public details::NestedBuilder<RoadGeometryBuilder> {
 public:
  explicit JunctionBuilder(RoadGeometryBuilder* parent) : details::NestedBuilder<RoadGeometryBuilder>(parent) {}

  JunctionBuilder& Id(const maliput::api::JunctionId& junction_id);
  SegmentBuilder StartSegment();
  RoadGeometryBuilder& EndJunction();

  void SetSegment(maliput::common::Passkey<SegmentBuilder>, std::unique_ptr<maliput::geometry_base::Segment> segment);

 private:
  maliput::api::JunctionId id_{"j_id"};
  std::vector<std::unique_ptr<maliput::geometry_base::Segment>> segments_{};
};

class RoadGeometryBuilder final : public details::BuilderBase {
 public:
  RoadGeometryBuilder() = default;

  RoadGeometryBuilder& Id(const maliput::api::RoadGeometryId& road_geometry_id);
  RoadGeometryBuilder& LinearTolerance(double linear_tolerance);
  RoadGeometryBuilder& AngularTolerance(double angular_tolerance);
  RoadGeometryBuilder& ScaleLength(double scale_length);
  RoadGeometryBuilder& InertialToBackendFrameTranslation(const maliput::math::Vector3& translation);
  JunctionBuilder StartJunction();
  std::unique_ptr<maliput::api::RoadGeometry> Build();

  void SetJunction(maliput::common::Passkey<JunctionBuilder>,
                   std::unique_ptr<maliput::geometry_base::Junction> junction);

 private:
  maliput::api::RoadGeometryId id_{"rg_id"};
  double linear_tolerance_{1e-6};
  double angular_tolerance_{1e-6};
  double scale_length_{1.};
  maliput::math::Vector3 inertial_to_backend_frame_translation_{0., 0., 0.};
  std::vector<std::unique_ptr<maliput::geometry_base::Junction>> junctions_{};
};

}  // namespace builder
}  // namespace maliput_sparse