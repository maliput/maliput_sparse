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

/**
 * @brief Base class of the builder's hierarchy.
 * @details Its sole purpose is to offer a template
 * hierachy to avoid duplicating code that is related to storing the parent, returning
 * it and making sure each children can call Passkey-protected methods.
 */
class BuilderBase {
 public:
  virtual ~BuilderBase() = default;
};

/**
 * @brief Holds the parent Builder class and offers a small set of convenient methods to manage
 * the Builder lifecycle.
 * @tparam ParentT The parent BuilderBase type.
 */
template <typename ParentT>
class NestedBuilder : public BuilderBase {
 public:
  /**
   * @brief Construct a new nested builder object.
   * @param parent The pointer to the parent builder. Must not be nullptr.
   * @throws maliput::common::assertion_error When @p parent is nulltr.
   */
  explicit NestedBuilder(ParentT* parent) : parent_(parent) { MALIPUT_THROW_UNLESS(parent_ != nullptr); }

  /**
   * @return A reference to the parent builder.
   */
  ParentT& End() { return *parent_; }

  /**
   * @return A pointer to the parent builder.
   */
  ParentT* Parent() { return parent_; }

 private:
  ParentT* parent_{};
};

}  // namespace details

// Forward declaration of the classes in this headerfile to enable their use before their full declaration.
class RoadGeometryBuilder;
class JunctionBuilder;
class SegmentBuilder;
class LaneBuilder;

/**
 * @brief Builder class for maliput_sparse::geometry::LaneGeometry.
 */
class LaneGeometryBuilder final : public details::NestedBuilder<LaneBuilder> {
 public:
  /**
   * @brief Construct a new Lane Geometry Builder object.
   * @param parent The parent LaneBuilder. It must not be nullptr.
   */
  explicit LaneGeometryBuilder(LaneBuilder* parent) : details::NestedBuilder<LaneBuilder>(parent) {}

  // TODO(maliput_sparse#10): implement me!
  // LaneGeometryBuilder& LeftLineString(LineString3d&& left_line_string);
  // LaneGeometryBuilder& RightLineString(LineString3d&& right_line_string);

  /**
   * @brief Finalizes the construction of the LaneGeometry and sets it to the parent LaneBuilder.
   * @pre Left and right LineStrings must be set before calling this method.
   * @throws maliput::common::assertion_error When the left and right LineStrings were not set.
   * @return The reference to the parent LaneBuilder.
   */
  LaneBuilder& EndLaneGeometry() { return End(); }
};

/**
 * @brief Builder class for maliput::api::Lanes.
 */
class LaneBuilder final : public details::NestedBuilder<SegmentBuilder> {
 public:
  /**
   * @brief Construct a new Lane Builder object.
   * @param parent The parent SegmentBuilder. It must not be nullptr.
   */
  explicit LaneBuilder(SegmentBuilder* parent) : details::NestedBuilder<SegmentBuilder>(parent) {}

  /**
   * @brief Sets the maliput::api::LaneId of the maliput::api::Lane.
   * @param lane_id The maliput::api::LaneId.
   * @return A reference to this LaneBuilder.
   */
  LaneBuilder& Id(const maliput::api::LaneId& lane_id);

  /**
   * @brief Starts the LaneGeometry builder for this Lane.
   * @return A LaneGeometryBuilder.
   */
  LaneGeometryBuilder StartLaneGeometry();

  /**
   * @brief Finanlizes the construction process of this Lane by inserting the Lane into the
   * parent SegmentBuilder.
   * @return A reference to the SegmentBuilder.
   */
  SegmentBuilder& EndLane();

  // TODO(maliput_sparse#10): implement me!
  // void SetLaneGeometry(maliput::common::Passkey<LaneGeometryBuilder>, std::unique_ptr<LaneGeometry> lane_geometry);

 private:
  maliput::api::LaneId id_{"l_id"};
};

/**
 * @brief Builder class for maliput::api::Segments.
 */
class SegmentBuilder final : public details::NestedBuilder<JunctionBuilder> {
 public:
  /**
   * @brief Construct a new Segment Builder object.
   * @param parent The parent JunctionBuilder. It must not be nullptr.
   */
  explicit SegmentBuilder(JunctionBuilder* parent) : details::NestedBuilder<JunctionBuilder>(parent) {}

  /**
   * @brief Sets the maliput::api::SegmentId of the maliput::api::Segment.
   * @param segment_id The maliput::api::SegmentId.
   * @return A reference to this SegmentBuilder.
   */
  SegmentBuilder& Id(const maliput::api::SegmentId& segment_id);

  /**
   * @brief Starts the Lane builder for this Segment.
   * @return A LaneBuilder.
   */
  LaneBuilder StartLane();

  /**
   * @brief Finanlizes the construction process of this Segment by inserting the Segment into the
   * parent JunctionBuilder.
   * @throws maliput::common::assertion_error When there is no lane to be set into the Segment.
   * @return A reference to the JunctionBuilder.
   */
  JunctionBuilder& EndSegment();

  /**
   * @brief Sets a maliput::geometry_base::Lane into this builder to fill in the Segment.
   * @details This method is only intended to be called by LaneBuilder instances. Call this method
   * in order to determine a specific left to right ordering for this Segment.
   * @see maliput::common::Passkey class description for further details.
   * @param lane A lane to be stored into the Segment. It must not be nullptr.
   * @throws maliput::common::assertion_error When @p lane is nullptr.
   */
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
  /**
   * @brief Construct a new Junction Builder object.
   * @param parent The parent RoadGeometryBuilder. It must not be nullptr.
   */
  explicit JunctionBuilder(RoadGeometryBuilder* parent) : details::NestedBuilder<RoadGeometryBuilder>(parent) {}

  /**
   * @brief Sets the maliput::api::JunctionId of the maliput::api::Junction.
   * @param junction_id The maliput::api::JunctionId.
   * @return A reference to this JunctionBuilder.
   */
  JunctionBuilder& Id(const maliput::api::JunctionId& junction_id);

  /**
   * @brief Starts the Segment builder for this Junction.
   * @return A JunctionBuilder.
   */
  SegmentBuilder StartSegment();

  /**
   * @brief Finanlizes the construction process of this Junction by inserting the Junction into the
   * parent RoadGeometryBuilder.
   * @throws maliput::common::assertion_error When there is no segment to be set into the Junction.
   * @return A reference to the RoadGeometryBuilder.
   */
  RoadGeometryBuilder& EndJunction();

  /**
   * @brief Sets a maliput::geometry_base::Segment into this builder to fill in the Junction.
   * @details This method is only intended to be called by SegmentBuilder instances.
   * @see maliput::common::Passkey class description for further details.
   * @param segment A segment to be stored into the Junction. It must not be nullptr.
   * @throws maliput::common::assertion_error When @p segment is nullptr.
   */
  void SetSegment(maliput::common::Passkey<SegmentBuilder>, std::unique_ptr<maliput::geometry_base::Segment> segment);

 private:
  maliput::api::JunctionId id_{"j_id"};
  std::vector<std::unique_ptr<maliput::geometry_base::Segment>> segments_{};
};

/**
 * @brief Builder class for maliput::api::RoadGeometry.
 */
class RoadGeometryBuilder final : public details::BuilderBase {
 public:
  /**
   * @brief Construct a new RoadGeometry Builder object.
   */
  RoadGeometryBuilder() = default;

  /**
   * @brief Sets the maliput::api::RoadGeometryId of the maliput::api::RoadGeometry.
   * @param road_geometry_id The maliput::api::RoadGeometryId.
   * @return A reference to this RoadGeometryBuilder.
   */
  RoadGeometryBuilder& Id(const maliput::api::RoadGeometryId& road_geometry_id);

  /**
   * @brief Sets the linear tolerance of the maliput::api::RoadGeometry.
   * @param linear_tolerance The linear tolerance of the maliput::api::RoadGeometry. It must be positive.
   * @return A reference to this RoadGeometryBuilder.
   */
  RoadGeometryBuilder& LinearTolerance(double linear_tolerance);

  /**
   * @brief Sets the angular tolerance of the maliput::api::RoadGeometry.
   * @param angular_tolerance The angular tolerance of the maliput::api::RoadGeometry. It must be positive.
   * @return A reference to this RoadGeometryBuilder.
   */
  RoadGeometryBuilder& AngularTolerance(double angular_tolerance);

  /**
   * @brief Sets the scale length of the maliput::api::RoadGeometry.
   * @param scale_tolerance The scale length of the maliput::api::RoadGeometry. It must be positive.
   * @return A reference to this RoadGeometryBuilder.
   */
  RoadGeometryBuilder& ScaleLength(double scale_length);

  /**
   * @brief Sets the initial to backend frame translation vector of the maliput::api::RoadGeometry.
   * @param translation The nitial to backend frame translation vector of the maliput::api::RoadGeometry.
   * @return A reference to this RoadGeometryBuilder.
   */
  RoadGeometryBuilder& InertialToBackendFrameTranslation(const maliput::math::Vector3& translation);

  /**
   * @brief Starts the Junction builder for this RoadGeometry.
   * @return A JunctionBuilder.
   */
  JunctionBuilder StartJunction();

  // TODO(maliput_sparse#10): Provide a mechanism to build BranchPoints.

  /**
   * @brief Builds a maliput::api::RoadGeometry.
   * @details The underlying type of the RoadGeometry is maliput_sparse::RoadGeometry which is derived from
   * maliput::geometry_base::RoadGeometry.
   * @throws maliput::common::assertion_error  When there is no Junction to add to the RoadGeometry.
   * @return A std::unique_ptr<maliput::api::RoadGeometry>.
   */
  std::unique_ptr<maliput::api::RoadGeometry> Build();

  /**
   * @brief Sets a maliput::geometry_base::Junction into this builder to fill in the RoadGeometry.
   * @details This method is only intended to be called by JunctionBuilder instances.
   * @see maliput::common::Passkey class description for further details.
   * @param junction A junction to be stored into the RoadGeometry. It must not be nullptr.
   * @throws maliput::common::assertion_error When @p junction is nullptr.
   */
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