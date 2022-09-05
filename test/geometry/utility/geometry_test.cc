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
#include "maliput_sparse/geometry/utility/geometry.h"

#include <array>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput/math/vector.h>
#include <maliput/test_utilities/maliput_math_compare.h>

namespace maliput_sparse {
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

struct LineStringPointAtPCase {
  LineString3d line_string{};
  double p{};
  Vector3 expected_point{};
};

std::vector<LineStringPointAtPCase> LineStringPointAtPTestCases() {
  return {
      {
          LineString3d{{0., 0., 0.}, {10., 0., 0.}} /* line string*/,
          double{5.} /* p */,
          {5., 0., 0.} /* expected_point */
      },
      {
          LineString3d{{0., 0., 0.}, {10., 0., 0.}, {10., 10., 0.}, {10., 10., 10.}, {0., 10., 10.}} /* line string*/,
          double{35.} /* p */,
          {5., 10., 10.} /* expected_point */
      },
      {
          LineString3d{{-157., 123., 25.},
                       {5468., -67., -1.},
                       {-385., 15., 25.},
                       {-67., 5468., 85.},
                       {0., 5468., 85.}} /* line string*/,
          double{17000} /* p */,
          {-11.4941296448815, 5468., 85.} /* expected_point */
      },
  };
}

class InterpolatedPointAtPTest : public ::testing::TestWithParam<LineStringPointAtPCase> {
 public:
  static constexpr double kTolerance{1e-12};
  LineStringPointAtPCase line_string_point_at_p_case_ = GetParam();
};

TEST_P(InterpolatedPointAtPTest, Test) {
  const auto dut = InterpolatedPointAtP(line_string_point_at_p_case_.line_string, line_string_point_at_p_case_.p);
  EXPECT_TRUE(maliput::math::test::CompareVectors(line_string_point_at_p_case_.expected_point, dut, kTolerance));
}

TEST_P(InterpolatedPointAtPTest, NegativeP) {
  const double negative_p{-1.};
  const auto dut = InterpolatedPointAtP(line_string_point_at_p_case_.line_string, negative_p);
  EXPECT_EQ(line_string_point_at_p_case_.line_string.first(), dut);
}

TEST_P(InterpolatedPointAtPTest, ExceededP) {
  const double exceeded_p{line_string_point_at_p_case_.line_string.length() + 1};
  const auto dut = InterpolatedPointAtP(line_string_point_at_p_case_.line_string, exceeded_p);
  EXPECT_EQ(line_string_point_at_p_case_.line_string.last(), dut);
}

INSTANTIATE_TEST_CASE_P(InterpolatedPointAtPTestGroup, InterpolatedPointAtPTest,
                        ::testing::ValuesIn(LineStringPointAtPTestCases()));

struct SlopeTestCase {
  LineString3d line_string{};
  std::vector<double> p{};
  std::vector<double> expected_slopes{};
};

std::vector<SlopeTestCase> SlopeTestCases() {
  return {
      {
          // LineString with length of: std::sqrt(2)*10
          LineString3d{{0., 0., 0.}, {10., 0., 10.}} /* line string*/,
          {0., std::sqrt(2) * 10. / 2., std::sqrt(2) * 10} /* ps */,
          {std::sqrt(2.) / 2., std::sqrt(2.) / 2., std::sqrt(2.) / 2.} /* expected_slopes */
      },
      {
          // LineString with different z values along y axis.
          LineString3d{{0., 0., 0.}, {5., 0., 5.}, {10., 0., 5.}, {15., 0., 10.}} /* line string*/,
          {0., std::sqrt(2.) * 5, std::sqrt(2.) * 5 + 5., 2 * std::sqrt(2.) * 5 + 5.} /* ps */,
          {std::sqrt(2.) / 2., std::sqrt(2.) / 2., 0., std::sqrt(2.) / 2.} /* expected_slopes */
      },
      {
          // LineString with different z values along x axis.
          LineString3d{{0., 0., 0.}, {0., 5., 5.}, {0., 10., 5.}, {0., 15., 10.}} /* line string*/,
          {0., std::sqrt(2.) * 5, std::sqrt(2.) * 5 + 5., 2 * std::sqrt(2.) * 5 + 5.} /* ps */,
          {std::sqrt(2.) / 2., std::sqrt(2.) / 2., 0., std::sqrt(2.) / 2.} /* expected_slopes */
      },
      {
          // LineString with different z values.
          LineString3d{{0., 0., 0.}, {6., 3., 2.}, {10., 6., 2.}, {16., 9., 4.}, {10., 6., 2.}} /* line string*/,
          {0., 7., 12., 19., 26.} /* ps */,
          {2. / 7., 2. / 7., 0., 2. / 7., -2. / 7.} /* expected_slopes */
      },
  };
}

class GetSlopeAtPTest : public ::testing::TestWithParam<SlopeTestCase> {
 public:
  static constexpr double kTolerance{1e-12};
  SlopeTestCase case_ = GetParam();
};

TEST_P(GetSlopeAtPTest, Test) {
  ASSERT_EQ(case_.p.size(), case_.expected_slopes.size()) << ">>>>> Test case is ill-formed.";
  for (std::size_t i = 0; i < case_.p.size(); ++i) {
    const double dut = GetSlopeAtP(case_.line_string, case_.p[i]);
    EXPECT_DOUBLE_EQ(case_.expected_slopes[i], dut);
  }
}

INSTANTIATE_TEST_CASE_P(GetSlopeAtPTestGroup, GetSlopeAtPTest, ::testing::ValuesIn(SlopeTestCases()));

class GetSlopeAtPSpecialCasesTest : public testing::Test {};

TEST_F(GetSlopeAtPSpecialCasesTest, Throw) {
  const LineString3d kNoLength{{0., 0., 0.}, {0., 0., 0.}};
  EXPECT_THROW(GetSlopeAtP(kNoLength, 0.), maliput::common::assertion_error);
}

TEST_F(GetSlopeAtPSpecialCasesTest, InfinitySlope) {
  const LineString3d kOnlyZ{{0., 0., 0.}, {0., 0., 100.}, {0., 0., 0.}};
  EXPECT_EQ(1, GetSlopeAtP(kOnlyZ, 50.));
  EXPECT_EQ(-1, GetSlopeAtP(kOnlyZ, 150.));
}

}  // namespace
}  // namespace test
}  // namespace utility
}  // namespace geometry
}  // namespace maliput_sparse
