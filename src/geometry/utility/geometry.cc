// Code in this file is inspired by:
// https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/src/Lanelet.cpp
//
// Lanelet2's license follows:
//
// Copyright 2018 FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
// following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
// disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
// following disclaimer in the documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
// products derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//
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

#include "maliput_sample/geometry/utility/geometry.h"

namespace maliput_sample {
namespace geometry {
namespace utility {

using OptDistance = std::optional<double>;
using Segment3d = std::pair<maliput::math::Vector3, maliput::math::Vector3>;
using Segment2d = std::pair<maliput::math::Vector2, maliput::math::Vector2>;

namespace {

// Determines whether two line segments intersects.
//
// Based on https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line_segment
bool SegmentsIntersect2d(const Segment2d& lhs, const Segment2d& rhs) {
  const auto& x1 = lhs.first[0];
  const auto& y1 = lhs.first[1];
  const auto& x2 = lhs.second[0];
  const auto& y2 = lhs.second[1];
  const auto& x3 = rhs.first[0];
  const auto& y3 = rhs.first[1];
  const auto& x4 = rhs.second[0];
  const auto& y4 = rhs.second[1];
  const double den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
  if (den == 0) {
    // They are parallel or coincident.
    return false;
  }
  const double t_num = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4);
  const double u_num = (x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2);
  const double t = t_num / den;
  const double u = u_num / den;
  return (t >= 0 && t <= 1 && u >= 0 && u <= 1);
}

// Obtains the distance between @p lhs and @p rhs points projected on the xy plane.
double Distance2d(const maliput::math::Vector3& lhs, const maliput::math::Vector3& rhs) {
  const maliput::math::Vector2 lhs_2d{lhs.x(), lhs.y()};
  const maliput::math::Vector2 rhs_2d{rhs.x(), rhs.y()};
  return (lhs_2d - rhs_2d).norm();
}

// Determines whether point @p p is located left from segment compound by @p seg_first and @p seg_second.
bool PointIsLeftOf(const maliput::math::Vector3& seg_first, const maliput::math::Vector3& seg_second,
                   const maliput::math::Vector3& p) {
  return ((seg_second - seg_first).cross(p - seg_first)).z() > 0;
}

maliput::math::Vector3 MakeCenterpoint(const maliput::math::Vector3& p1, const maliput::math::Vector3& p2) {
  return (p1 + p2) / 2.;
}

// Helper class for evaluating the boundaries of a lane.
class BoundChecker {
 public:
  BoundChecker(const LineString3d& left, const LineString3d& right)
      : left_(left), right_(right), entry_{right.first(), left.first()}, exit_{left.last(), right.last()} {
    left_segments_ = MakeSegments(left_);
    right_segments_ = MakeSegments(right_);
  }

  bool Intersects2d(const Segment3d& seg) const {
    const Segment2d seg2d{{seg.first.x(), seg.first.y()}, {seg.second.x(), seg.second.y()}};
    const bool result =
        IntersectsLeft2d(seg2d) || IntersectsRight2d(seg2d) || CrossesEntry2d(seg2d) || CrossesExit2d(seg2d);
    return result;
  }

  bool IntersectsLeft2d(const Segment2d& seg) const {
    for (const auto left_segment : left_segments_) {
      const Segment2d left_seg{{left_segment.first.x(), left_segment.first.y()},
                               {left_segment.second.x(), left_segment.second.y()}};
      if (SegmentsIntersect2d(seg, left_seg)) {
        if (!(seg.first == left_seg.first)) {
          return true;
        }
      }
    }
    return false;
  }

  bool IntersectsRight2d(const Segment2d& seg) const {
    for (const auto right_segment : right_segments_) {
      const Segment2d right_seg{{right_segment.first.x(), right_segment.first.y()},
                                {right_segment.second.x(), right_segment.second.y()}};
      if (SegmentsIntersect2d(right_seg, seg)) {
        if (!(seg.first == right_seg.first)) {
          return true;
        }
      }
    }
    return false;
  }

  bool CrossesEntry2d(const Segment2d& seg) const {
    const Segment2d entry_2d{{entry_.first.x(), entry_.first.y()}, {entry_.second.x(), entry_.second.y()}};
    if (SegmentsIntersect2d(seg, entry_2d)) {
      return PointIsLeftOf({entry_2d.first.x(), entry_2d.first.y(), 0.}, {entry_2d.second.x(), entry_2d.second.y(), 0.},
                           {seg.second.x(), seg.second.y(), 0.});
    }
    return false;
  }

  bool CrossesExit2d(const Segment2d& seg) const {
    const Segment2d exit_2d{{exit_.first.x(), exit_.first.y()}, {exit_.second.x(), exit_.second.y()}};
    if (SegmentsIntersect2d(seg, exit_2d)) {
      PointIsLeftOf({exit_2d.first.x(), exit_2d.first.y(), 0.}, {exit_2d.second.x(), exit_2d.second.y(), 0.},
                    {seg.second.x(), seg.second.y(), 0.});
    }
    return false;
  }

  template <typename Func>
  void ForEachPointLeftUntil(LineString3d::const_iterator iter, Func&& f) const {
    return ForEachPointUntilImpl(iter, left_, std::forward<Func>(f));
  }
  template <typename Func>
  void ForEachPointRightUntil(LineString3d::const_iterator iter, Func&& f) const {
    return ForEachPointUntilImpl(iter, right_, std::forward<Func>(f));
  }

 private:
  static std::vector<Segment3d> MakeSegments(const LineString3d& line) {
    std::vector<Segment3d> segments;
    segments.reserve(line.size());
    for (auto i = 0u; i < line.size() - 2; ++i) {
      segments.push_back({line[i], line[i + 1]});
    }
    return segments;
  }

  template <typename Func>
  static void ForEachPointUntilImpl(LineString3d::const_iterator iter, const LineString3d& points, Func&& f) {
    for (auto iter = points.begin(); iter != points.end(); ++iter) {
      if (f(iter)) {
        break;
      }
    }
  }

  const LineString3d& left_;
  const LineString3d& right_;
  Segment3d entry_, exit_;
  std::vector<Segment3d> left_segments_;
  std::vector<Segment3d> right_segments_;
};

std::pair<LineString3d::const_iterator, OptDistance> FindClosestNonintersectingPoint(
    LineString3d::const_iterator current_position, LineString3d::const_iterator other_point, const BoundChecker& bounds,
    const maliput::math::Vector3& last_point, bool is_left) {
  OptDistance distance;
  LineString3d::const_iterator closest_position{nullptr};
  double d_last_other = (*other_point - last_point).norm();

  auto non_intersecting_point_loop = [&](LineString3d::const_iterator candidate) {
    // point must be after current_position
    if (candidate < current_position) {
      return false;
    }
    const auto candidate_distance = Distance2d(*candidate, *other_point) / 2;  // candidate distance
    // we use the triangle inequation to find a distance where we can not
    // expect a closer point than the current one.
    if (!!distance && candidate_distance / 2 - d_last_other > *distance) {
      return true;  // stops the loop
    }
    if (!!distance && *distance <= candidate_distance) {
      return false;
    }
    // Candidates are only valid candidates if
    // 1. their distance is minimal (at least for now) -> checked above
    // 2. the new connection does not intersect with the borders
    // 3. connection between point on one bound and point on other bound
    // does not intersect with other parts of the boundary
    const Segment3d bound_connection(*other_point, *candidate);
    const auto centerline_point_candidate = MakeCenterpoint(bound_connection.first, bound_connection.second);
    const Segment3d centerline_candidate{last_point, centerline_point_candidate};
    if (!bounds.Intersects2d(centerline_candidate)) {
      distance = candidate_distance;
      closest_position = candidate;
    }
    return false;
  };
  if (is_left) {
    bounds.ForEachPointLeftUntil(other_point,
                                 non_intersecting_point_loop);  // For each point in the left string to other_point
  } else {
    bounds.ForEachPointRightUntil(other_point, non_intersecting_point_loop);
  }
  return {closest_position, distance};
}

}  // namespace

LineString3d ComputeCenterline3d(const LineString3d& left, const LineString3d& right) {
  LineString3d centerline;
  BoundChecker bounds(left, right);
  // Initial point
  centerline.push_back(MakeCenterpoint(left.first(), right.first()));

  auto left_current = left.begin();
  auto right_current = right.begin();

  while (left_current != left.end() || right_current != right.end()) {
    std::optional<double> left_candidate_distance;
    std::optional<double> right_candidate_distance;

    LineString3d::const_iterator left_candidate;
    LineString3d::const_iterator right_candidate;
    // Determine left candidate
    std::tie(left_candidate, left_candidate_distance) =
        FindClosestNonintersectingPoint(std::next(left_current), right_current, bounds, centerline.last(), true);

    // Determine right candidate
    std::tie(right_candidate, right_candidate_distance) =
        FindClosestNonintersectingPoint(std::next(right_current), left_current, bounds, centerline.last(), false);
    // Choose the better one
    if (left_candidate_distance && (!right_candidate_distance || left_candidate_distance <= right_candidate_distance)) {
      MALIPUT_THROW_UNLESS(left_candidate != left.end());

      const auto& left_point = left[size_t(left_candidate - left.begin())];
      const auto& right_point = right[size_t(right_current - right.begin())];
      const auto centerpoint = MakeCenterpoint(left_point, right_point);
      centerline.push_back(centerpoint);
      left_current = left_candidate;
    } else if (right_candidate_distance &&
               (!left_candidate_distance || left_candidate_distance > right_candidate_distance)) {
      MALIPUT_THROW_UNLESS(right_candidate != right.end());

      const auto& left_point = left[size_t(left_current - left.begin())];
      const auto& right_point = right[size_t(right_candidate - right.begin())];
      const auto centerpoint = MakeCenterpoint(left_point, right_point);
      centerline.push_back(centerpoint);
      right_current = right_candidate;
    } else {
      // no next point found. We are done here
      break;
    }
  }

  // we want the centerpoint defined by the endpoints inside in any case
  if (!(left_current == std::prev(left.end()) && right_current == std::prev(right.end()))) {
    centerline.push_back(MakeCenterpoint(left.last(), right.last()));
  }
  return centerline;
}

maliput::math::Vector3 InterpolatedPointAtP(const LineString3d& line_string, double p) {
  // Implementation inspired on:
  // https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/include/lanelet2_core/geometry/impl/LineString.h#L618
  static constexpr double kEpsilon{1e-12};
  if (p < 0) return line_string.first();
  if (p >= line_string.length()) return line_string.last();

  const auto line_string_points_length = GetBoundPointsAtP(line_string, p);
  const double partial_length{(*line_string_points_length.first - *line_string_points_length.second).norm()};
  const double length_up_to_second{line_string_points_length.length + partial_length};
  const double remaining_distance = p - line_string_points_length.length;
  if (remaining_distance < kEpsilon) {
    return *line_string_points_length.first;
  }
  return *line_string_points_length.first +
         remaining_distance / partial_length * (*line_string_points_length.second - *line_string_points_length.first);
}

double GetSlopeAtP(const LineString3d& line_string, double p) {
  const LineStringPointsAndLength bound_points = GetBoundPointsAtP(line_string, p);
  const double dist{(*bound_points.second - *bound_points.first).norm()};
  const double delta_z{bound_points.second->z() - bound_points.first->z()};
  MALIPUT_THROW_UNLESS(!(dist == 0. && delta_z == 0.));
  return delta_z / dist;
}

LineStringPointsAndLength GetBoundPointsAtP(const LineString3d& line_string, double p) {
  MALIPUT_THROW_UNLESS(p >= 0);
  MALIPUT_THROW_UNLESS(p <= line_string.length());

  LineStringPointsAndLength result;
  double current_cumulative_length = 0.0;
  for (auto first = line_string.begin(), second = std::next(line_string.begin()); second != line_string.end();
       ++first, ++second) {
    const auto p1 = *first;
    const auto p2 = *second;
    const double current_length = (p1 - p2).norm();
    if (current_cumulative_length + current_length >= p) {
      return {first, second, current_cumulative_length};
    }
    current_cumulative_length += current_length;
  }
  return {line_string.end() - 1, line_string.end() - 2, line_string.length()};
}

}  // namespace utility
}  // namespace geometry
}  // namespace maliput_sample
