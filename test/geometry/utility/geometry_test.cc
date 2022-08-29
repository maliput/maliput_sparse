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

#include <array>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput/math/vector.h>

namespace maliput_sample {
namespace geometry {
namespace utility {
namespace test {
namespace {

using maliput::math::Vector3;

struct LeftRightCenterlineCase {
  LineString3d left{};
  LineString3d right{};
  LineString3d expected_centerline{};
};

std::vector<LeftRightCenterlineCase> LeftRightCenterlineTestCases() {
  LineString3d{{0., 1, 0.}, {10., 1., 0.}, {20., 1., 0.}};
  return {
      // Straight 2 point lines.
      {
          LineString3d{{0., 1., 0.}, {10., 1., 0.}} /* left */, LineString3d{{0., -1., 0.}, {10., -1., 0.}} /*right*/,
          LineString3d{{0., 0., 0.}, {5., 0., 0.}, {10., 0., 0.}} /*centerline*/
      },
      // Straight 3 point lines.
      {
          LineString3d{{0., 1., 0.}, {10., 1., 0.}, {20., 1., 0.}} /* left */,
          LineString3d{{0., -1., 0.}, {10., -1., 0.}, {20., -1., 0.}} /*right*/,
          LineString3d{{0., 0., 0.}, {5., 0., 0.}, {10., 0., 0.}, {15., 0., 0.}, {20., 0., 0.}} /*centerline*/
      },
      // Straight elevated lines.
      {
          LineString3d{{0., 1., 10.}, {10., 1., 10.}, {20., 1., 10.}} /* left */,
          LineString3d{{0., -1., 10.}, {10., -1., 10.}, {20., -1., 10.}} /*right*/,
          LineString3d{{0., 0., 10.}, {5., 0., 10.}, {10., 0., 10.}, {15., 0., 10.}, {20., 0., 10.}} /*centerline*/
      },
      // Straight different elevation lines.
      {
          LineString3d{{0., 1., 0.}, {10., 1., 0.}, {20., 1., 0.}} /* left */,
          LineString3d{{0., -1., 10.}, {10., -1., 10.}, {20., -1., 10.}} /*right*/,
          LineString3d{{0., 0., 5.}, {5., 0., 5.}, {10., 0., 5.}, {15., 0., 5.}, {20., 0., 5.}} /*centerline*/
      },
      // Straight varying elevation lines in right.
      {
          LineString3d{{0., 1., 0.}, {10., 1., 0.}, {20., 1., 0.}} /* left */,
          LineString3d{{0., -1., 12.}, {10., -1., 14.}, {20., -1., 16.}} /*right*/,
          LineString3d{{0., 0., 6.}, {5., 0., 6.}, {10., 0., 7.}, {15., 0., 7.}, {20., 0., 8.}} /*centerline*/
      },
      // Straight varying elevation lines in both line strings.
      {
          LineString3d{{0., 1., 0.}, {10., 1., -2.}, {20., 1., -4.}} /* left */,
          LineString3d{{0., -1., 12.}, {10., -1., 14.}, {20., -1., 16.}} /*right*/,
          LineString3d{{0., 0., 6.}, {5., 0., 5.}, {10., 0., 6.}, {15., 0., 5.}, {20., 0., 6.}} /*centerline*/
      },
      // Variable width. Right line opens.
      {
          LineString3d{{0., 2., 0.}, {1., 2., 0.}, {2., 2., 0.}, {3., 2., 0.}, {4., 2., 0.}} /* left */,
          LineString3d{{0., -2., 0.}, {1., -2., 0.}, {2., -4., 0.}, {3., -4., 0.}, {4., -4., 0.}} /* right */,
          LineString3d{{0., 0., 0.},
                       {0.5, 0., 0.},
                       {1., 0., 0.},
                       {1.5, 0., 0.},
                       {2., 0., 0.},
                       {2.5, 0., 0.},
                       {4., -1., 0.}} /* centerline */,
      },
      // Variable width. Right line closes.
      {
          LineString3d{{0., 4., 0.}, {10., 4., 0.}, {20., 4., 0.}, {30., 4., 0.}, {40., 4., 0.}, {50., 4., 0.}} /* left
                                                                                                                 */
          ,
          LineString3d{{0., -4., 10.},
                       {10., -4., 10.},
                       {20., -0., 10.},
                       {30., -0., 10.},
                       {40., -4., 10.},
                       {50., -4., 10.}} /* right */,
          LineString3d{{0., 0., 5.},
                       {5., 0., 5.},
                       {10., 0., 5.},
                       {15., 2., 5.},
                       {20., 2., 5.},
                       {25., 2., 5.},
                       {30., 2., 5.},
                       {35., 2., 5.},
                       {40., 0., 5.},
                       {45., 0., 5.},
                       {50., 0., 5.}} /* centerline */,
      },
      // Starting at different point
      {
          LineString3d{{0.5, 2., 0}, {1.5, 2., 0}, {2.5, 2., 0}, {3.5, 2., 0}} /* left */,
          LineString3d{{0., -2., 0}, {1, -2., 0}, {2., -2., 0}, {3, -2., 0}} /* right */,
          LineString3d{{0.25, 0, 0},
                       {0.75, 0, 0},
                       {1.25, 0, 0},
                       {1.75, 0, 0},
                       {2.25, 0, 0},
                       {2.75, 0, 0},
                       {3.25, 0, 0}} /* centerline */,
      },
      // Starting different point elevated lines.
      {
          LineString3d{{0.5, 2., 0}, {1.5, 2., 0}, {2.5, 2., 0}, {3.5, 2., 0}} /* left */,
          LineString3d{{0., -2., 10}, {1, -2., 12}, {2., -2., 14}, {3, -2., 16}} /* right */,
          LineString3d{{0.25, 0, 5},
                       {0.75, 0, 6},
                       {1.25, 0, 6},
                       {1.75, 0, 7},
                       {2.25, 0, 7},
                       {2.75, 0, 8},
                       {3.25, 0, 8}} /* centerline */,
      },
      // Starting different point varying elevation of lines.
      {
          LineString3d{{0.5, 2., -3.}, {1.5, 2., -5}, {2.5, 2., -2}, {3.5, 2., -7}} /* left */,
          LineString3d{{0., -2., 10}, {1, -2., 12}, {2., -2., 14}, {3, -2., 16}} /* right */,
          LineString3d{{0.25, 0, 3.5},
                       {0.75, 0, 4.5},
                       {1.25, 0, 3.5},
                       {1.75, 0, 4.5},
                       {2.25, 0, 6.},
                       {2.75, 0, 7.},
                       {3.25, 0, 4.5}} /* centerline */,
      },
  };
}

class ComputeCenterlineTest : public ::testing::TestWithParam<LeftRightCenterlineCase> {
 public:
  LeftRightCenterlineCase left_right_centerline_ = GetParam();
};

TEST_P(ComputeCenterlineTest, Test) {
  const auto dut = ComputeCenterline3d(left_right_centerline_.left, left_right_centerline_.right);
  EXPECT_EQ(left_right_centerline_.expected_centerline.size(), dut.size());
  EXPECT_EQ(left_right_centerline_.expected_centerline, dut);
}

INSTANTIATE_TEST_CASE_P(ComputeCenterlineTestGroup, ComputeCenterlineTest,
                        ::testing::ValuesIn(LeftRightCenterlineTestCases()));

}  // namespace
}  // namespace test
}  // namespace utility
}  // namespace geometry
}  // namespace maliput_sample
