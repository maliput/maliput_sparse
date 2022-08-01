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

#include <initializer_list>
#include <vector>

#include <maliput/common/maliput_throw.h>

namespace maliput_sample {
namespace geometry {

template <typename CoordinateT, typename DistanceFunction>
class LineString final {
 public:
  explicit LineString(const std::vector<CoordinateT>& coordinates) : coordinates_(coordinates) {
    MALIPUT_THROW_UNLESS(coordinates_.size() > 1);
    length_ = ComputeLength();
  }

  explicit LineString(std::initializer_list<CoordinateT> coordinates) : coordinates_(coordinates) {
    MALIPUT_THROW_UNLESS(coordinates_.size() > 1);
    length_ = ComputeLength();
  }

  const CoordinateT& first() const { return coordinates_.front(); }
  const CoordinateT& last() const { return coordinates_.back(); }
  const CoordinateT& at(size_t i) const { return coordinates_.at(i); }
  size_t size() const { return coordinates_.size(); }
  double length() const { return length_; }

 private:
  const double ComputeLength() const {
    double accumulated_{0.};
    for (size_t i = 0; i < size() - 1; ++i) {
      accumulated_ += DistanceFunction()(at(i), at(i + 1));
    }
    return accumulated_;
  }

  std::vector<CoordinateT> coordinates_{};
  double length_{};
};

// template<typename CoordinateT>
// LineString<CoordinateT> BuildCenterLine(const LineString<CoordinateT>& lhs, const LineString<CoordinateT>& rhs) {
//   return LineString<CoordinateT>(CoordinateT(), Coordinate());
// }

}  // namespace geometry
}  // namespace maliput_sample
