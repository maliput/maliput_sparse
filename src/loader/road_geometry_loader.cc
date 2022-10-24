// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet.
// All rights reserved.
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
#include "maliput_sparse/loader/road_geometry_loader.h"

#include "maliput_sparse/builder/builder.h"

namespace maliput_sparse {
namespace loader {

RoadGeometryLoader::RoadGeometryLoader(std::unique_ptr<parser::Parser> parser,
                                         const BuilderConfiguration& builder_configuration)
    : parser_(std::move(parser)), builder_configuration_(builder_configuration) {
  MALIPUT_THROW_UNLESS(parser_ != nullptr);
}

std::unique_ptr<const maliput::api::RoadGeometry> RoadGeometryLoader::operator()() {
  const std::unordered_map<parser::Segment::Id, parser::Segment>& segments = parser_->GetSegments();

  maliput_sparse::builder::RoadGeometryBuilder rg_builder{};
  rg_builder.Id(builder_configuration_.road_geometry_id)
      .LinearTolerance(builder_configuration_.linear_tolerance)
      .AngularTolerance(builder_configuration_.angular_tolerance)
      .ScaleLength(builder_configuration_.scale_length)
      .InertialToBackendFrameTranslation(builder_configuration_.inertial_to_backend_frame_translation);

  for (const auto& segment : segments) {
    builder::JunctionBuilder junction_builder = rg_builder.StartJunction();
    junction_builder.Id(maliput::api::JunctionId{segment.first});
    builder::SegmentBuilder segment_builder = junction_builder.StartSegment();
    segment_builder.Id(maliput::api::SegmentId{segment.first});

    for (const parser::Lane& lane : segment.second.lanes) {
      segment_builder.StartLane()
          .Id(maliput::api::LaneId{lane.id})
          .HeightBounds(maliput::api::HBounds{0., 5.})
          .StartLaneGeometry()
          .LeftLineString(lane.left)
          .RightLineString(lane.right)
          .EndLaneGeometry()
          .EndLane();
    }
    segment_builder.EndSegment().EndJunction();
  }
  return rg_builder.StartBranchPoints().EndBranchPoints().Build();
}

}  // namespace loader
}  // namespace maliput_sparse
