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

bool SegmentsIntersect2d(const Segment2d& lhs, const Segment2d& rhs) {
  const auto& x1 = lhs.first[0];
  const auto& y1 = lhs.first[1];
  const auto& x2 = lhs.second[0];
  const auto& y2 = lhs.second[1];
  const auto& x3 = rhs.first[0];
  const auto& y3 = rhs.first[1];
  const auto& x4 = rhs.second[0];
  const auto& y4 = rhs.second[1];

  double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
  // If denominator is zero, the segments are parallel or coincident.
  return d != 0;
}

bool PointIsLeftOf(const maliput::math::Vector3& pSeg1, const maliput::math::Vector3& pSeg2,
                   const maliput::math::Vector3& p) {
  return ((pSeg2 - pSeg1).cross(p - pSeg1)).z() > 0;
}

Segment2d To2d(const Segment3d& segment) {
  return {{segment.first.x(), segment.first.y()}, {segment.second.x(), segment.second.y()}};
}

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
      if (SegmentsIntersect2d(seg, right_seg)) {
        if (!(seg.first == right_seg.first)) {
          return true;
        }
      }
    }
    return false;
  }

  bool SecondCrossesBounds(const Segment2d& seg, bool left) const {
    const auto& segments = left ? left_segments_ : right_segments_;
    for (const auto& segment : segments) {
      if (SegmentsIntersect2d(To2d(segment), seg)) {
        return true;
      }
    }
    return false;
  }

  bool CrossesEntry2d(const Segment2d& seg) const {
    const Segment2d entry_2d{{entry_.first.x(), entry_.first.y()}, {entry_.second.x(), entry_.second.y()}};
    if (SegmentsIntersect2d(seg, entry_2d)) {
      PointIsLeftOf(maliput::math::Vector3{entry_2d.first.x(), entry_2d.first.y(), 0.},
                    maliput::math::Vector3{entry_2d.second.x(), entry_2d.second.y(), 0.},
                    maliput::math::Vector3{seg.second.x(), seg.second.y(), 0.});
    }
    return false;
  }

  bool CrossesExit2d(const Segment2d& seg) const {
    const Segment2d exit_2d{{exit_.first.x(), exit_.first.y()}, {exit_.second.x(), exit_.second.y()}};
    if (SegmentsIntersect2d(seg, exit_2d)) {
      PointIsLeftOf(maliput::math::Vector3{exit_2d.first.x(), exit_2d.first.y(), 0.},
                    maliput::math::Vector3{exit_2d.second.x(), exit_2d.second.y(), 0.},
                    maliput::math::Vector3{seg.second.x(), seg.second.y(), 0.});
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

maliput::math::Vector3 MakeCenterpoint(const maliput::math::Vector3& p1, const maliput::math::Vector3& p2) {
  return (p1 + p2) / 2.;
}

std::pair<LineString3d::const_iterator, OptDistance> FindClosestNonintersectingPoint(
    LineString3d::const_iterator current_position, LineString3d::const_iterator other_point, const BoundChecker& bounds,
    const maliput::math::Vector3& last_point, bool is_left) {
  std::cout << "\t\t[FindClosestNonintersectingPoint]: side: " << (is_left ? "left" : "right")
            << " - Current position: " << *current_position << " other_point: " << *other_point
            << " last_center_point: " << last_point << std::endl;
  OptDistance distance;
  LineString3d::const_iterator closest_position{nullptr};
  double d_last_other = (*other_point - last_point).norm();
  std::cout << "\t\td_last_other: " << d_last_other << std::endl;

  // this lambda is called with points of increasing distance to other_point
  auto nonintersectingPointLoop = [&](LineString3d::const_iterator candidate) {
    std::cout << "\t\t\tcandidate: " << *candidate << std::endl;
    // point must be after current_position

    if (candidate < current_position) {
      std::cout << "\t\t\t\tcandidate <= current_position" << std::endl;
      return false;
    }
    // we use the triangle inequation to find a distance where we can not
    // expect a closer point than the current one
    if (!!distance && (*other_point - *candidate).norm() / 2 - d_last_other > *distance) {
      std::cout << "\t\t\t\t!!distance && (*other_point - *candidate).norm() / 2 - d_last_other > *distance"
                << std::endl;
      return true;  // stops the loop
    }
    auto candidate_distance = (*candidate - *other_point).norm() / 2;  // candidate distance
    if (!!distance && *distance <= candidate_distance) {
      std::cout << "\t\t\t\t!!distance && *distance <= candidate_distance" << std::endl;
      return false;
    }
    // Candidates are only valid candidates if
    // 1. their distance is minimal (at least for now) -> checked above
    // 2. the new connection does not intersect with the borders
    // 3. connection between point on one bound and point on other bound
    // does not intersect with other parts of the boundary
    Segment3d bound_connection(*other_point, *candidate);
    Segment3d inv_bound_connection(bound_connection.second, bound_connection.first);
    auto centerline_point_candidate = MakeCenterpoint(bound_connection.first, bound_connection.second);
    std::cout << "\t\t\tcenterline_point_candidate: " << centerline_point_candidate << std::endl;
    Segment3d centerline_candidate{last_point, centerline_point_candidate};
    if (!bounds.Intersects2d(centerline_candidate) /*&& !bounds.SecondCrossesBounds(To2d(bound_connection), is_left)   &&
       !bounds.SecondCrossesBounds(To2d(inv_bound_connection), !is_left)*/) {
      distance = candidate_distance;
      closest_position = candidate;
      std::cout << "\t\t\t\tcandidated taken." << std::endl;
      std::cout << "\t\t\t\t\tdistance: " << *distance << std::endl;
    }
    return false;
  };
  if (is_left) {
    std::cout << "\t\tloop in left: " << std::endl;
    bounds.ForEachPointLeftUntil(other_point,
                                 nonintersectingPointLoop);  // For each point in the left string to other_point
  } else {
    std::cout << "\t\tloop in right: " << std::endl;
    bounds.ForEachPointRightUntil(other_point, nonintersectingPointLoop);
  }
  return {closest_position, distance};
}

}  // namespace

LineString3d ComputeCenterline3d(const LineString3d& left, const LineString3d& right) {
  LineString3d centerline;
  std::cout << "[ComputeCenterline3d]" << std::endl;
  BoundChecker bounds(left, right);
  // Initial point
  centerline.push_back(MakeCenterpoint(left.first(), right.first()));

  auto left_current = left.begin();
  auto right_current = right.begin();

  while (left_current != left.end() || right_current != right.end()) {
    std::cout << "\twhile loop: left_current: " << *left_current << " - right_current: " << *right_current << std::endl;
    std::optional<double> left_candidate_distance;
    std::optional<double> right_candidate_distance;

    LineString3d::const_iterator left_candidate;
    LineString3d::const_iterator right_candidate;
    // Determine left candidate
    std::tie(left_candidate, left_candidate_distance) =
        FindClosestNonintersectingPoint(std::next(left_current), right_current, bounds, centerline.last(), true);
    std::cout << "\tleft_candidate: "
              << (left_candidate == LineString3d::const_iterator{nullptr} ? "None" : (*left_candidate).to_str())
              << std::endl;
    std::cout << "\tleft_candidate_distance: "
              << (left_candidate_distance.has_value() ? std::to_string(*left_candidate_distance) : "std::nullopt")
              << std::endl;

    // Determine right candidate
    std::tie(right_candidate, right_candidate_distance) =
        FindClosestNonintersectingPoint(std::next(right_current), left_current, bounds, centerline.last(), false);
    std::cout << "\tright_candidate: "
              << (right_candidate == LineString3d::const_iterator{nullptr} ? "None" : (*right_candidate).to_str())
              << std::endl;
    std::cout << "\tright_candidate_distance: "
              << (right_candidate_distance.has_value() ? std::to_string(*right_candidate_distance) : "std::nullopt")
              << std::endl;
    // Choose the better one
    if (left_candidate_distance && (!right_candidate_distance || left_candidate_distance <= right_candidate_distance)) {
      MALIPUT_THROW_UNLESS(left_candidate != left.end());

      const auto& left_point = left[size_t(left_candidate - left.begin())];
      std::cout << "\tleft_point: " << left_point << std::endl;
      const auto& right_point = right[size_t(right_current - right.begin())];
      std::cout << "\tright_point: " << right_point << std::endl;
      const auto centerpoint = MakeCenterpoint(left_point, right_point);
      std::cout << "\tcenterpoint: " << centerpoint << std::endl;
      centerline.push_back(centerpoint);
      left_current = left_candidate;
    } else if (right_candidate_distance &&
               (!left_candidate_distance || left_candidate_distance > right_candidate_distance)) {
      MALIPUT_THROW_UNLESS(right_candidate != right.end());

      const auto& left_point = left[size_t(left_current - left.begin())];
      std::cout << "\tleft_point: " << left_point << std::endl;
      const auto& right_point = right[size_t(right_candidate - right.begin())];
      std::cout << "\tright_point: " << right_point << std::endl;
      const auto centerpoint = MakeCenterpoint(left_point, right_point);
      std::cout << "\tcenterpoint: " << centerpoint << std::endl;
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

}  // namespace utility
}  // namespace geometry
}  // namespace maliput_sample
