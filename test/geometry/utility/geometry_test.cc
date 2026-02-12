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
#include <maliput/common/error.h>
#include <maliput/math/compare.h>
#include <maliput/math/vector.h>

#include "assert_compare.h"

namespace maliput_sparse {
namespace geometry {
namespace utility {
namespace test {
namespace {

using maliput::math::CompareVectors;
using maliput::math::Vector3;
using maliput_sparse::test::AssertCompare;

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
          LineString3d{{0., 0., 0.}, {10., 0., 0.}} /* line string*/, 5. /* p */, {5., 0., 0.} /* expected_point */
      },
      {
          LineString3d{{0., 0., 0.}, {10., 0., 0.}, {10., 10., 0.}, {10., 10., 10.}, {0., 10., 10.}} /* line string*/,
          35. /* p */,
          {5., 10., 10.} /* expected_point */
      },
      {
          LineString3d{{-157., 123., 25.},
                       {5468., -67., -1.},
                       {-385., 15., 25.},
                       {-67., 5468., 85.},
                       {0., 5468., 85.}} /* line string*/,
          17000. /* p */,
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
  const auto dut =
      InterpolatedPointAtP(line_string_point_at_p_case_.line_string, line_string_point_at_p_case_.p, kTolerance);
  EXPECT_TRUE(AssertCompare(CompareVectors(line_string_point_at_p_case_.expected_point, dut, kTolerance)));
}

TEST_P(InterpolatedPointAtPTest, NegativeP) {
  const double negative_p{-1.};
  const auto dut = InterpolatedPointAtP(line_string_point_at_p_case_.line_string, negative_p, kTolerance);
  EXPECT_EQ(line_string_point_at_p_case_.line_string.first(), dut);
}

TEST_P(InterpolatedPointAtPTest, ExceededP) {
  const double exceeded_p{line_string_point_at_p_case_.line_string.length() + 1};
  const auto dut = InterpolatedPointAtP(line_string_point_at_p_case_.line_string, exceeded_p, kTolerance);
  EXPECT_EQ(line_string_point_at_p_case_.line_string.last(), dut);
}

INSTANTIATE_TEST_CASE_P(InterpolatedPointAtPTestGroup, InterpolatedPointAtPTest,
                        ::testing::ValuesIn(LineStringPointAtPTestCases()));

class GetBoundPointsAtPTest : public ::testing::Test {
 public:
  static constexpr double kTolerance{1e-12};
  const LineString3d line_string{{0., 0., 0.}, {6., 3., 2.}, {10., 6., 2.}, {16., 9., 4.}, {10., 6., 2.}};
};

TEST_F(GetBoundPointsAtPTest, Test) {
  {
    const double p{0.};
    const BoundPointsResult expected_result{0, 1, 0.};
    const BoundPointsResult dut = GetBoundPointsAtP(line_string, p, kTolerance);
    EXPECT_EQ(expected_result.idx_start, dut.idx_start);
    EXPECT_EQ(expected_result.idx_end, dut.idx_end);
    EXPECT_EQ(expected_result.length, dut.length);
  }
  {
    const double p{7.5};
    const BoundPointsResult expected_result{1, 2, 7.};
    const BoundPointsResult dut = GetBoundPointsAtP(line_string, p, kTolerance);
    EXPECT_EQ(expected_result.idx_start, dut.idx_start);
    EXPECT_EQ(expected_result.idx_end, dut.idx_end);
    EXPECT_EQ(expected_result.length, dut.length);
  }
  {
    const double p{12.5};
    const BoundPointsResult expected_result{2, 3, 12.};
    const BoundPointsResult dut = GetBoundPointsAtP(line_string, p, kTolerance);
    EXPECT_EQ(expected_result.idx_start, dut.idx_start);
    EXPECT_EQ(expected_result.idx_end, dut.idx_end);
    EXPECT_EQ(expected_result.length, dut.length);
  }
  {
    const double p{19.5};
    const BoundPointsResult expected_result{3, 4, 19.};
    const BoundPointsResult dut = GetBoundPointsAtP(line_string, p, kTolerance);
    EXPECT_EQ(expected_result.idx_start, dut.idx_start);
    EXPECT_EQ(expected_result.idx_end, dut.idx_end);
    EXPECT_EQ(expected_result.length, dut.length);
  }
}

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
          {1., 1., 1.} /* expected_slopes */
      },
      {
          // LineString with different z values along y axis.
          LineString3d{{0., 0., 0.}, {5., 0., 5.}, {10., 0., 5.}, {15., 0., 10.}} /* line string*/,
          {0., std::sqrt(2.) * 5. / 2., std::sqrt(2.) * 5. + 5. / 2., 2 * std::sqrt(2.) * 5. + 5.} /* ps */,
          {1., 1., 0., 1.} /* expected_slopes */
      },
      {
          // LineString with different z values along x axis.
          LineString3d{{0., 0., 0.}, {0., 5., 5.}, {0., 10., 5.}, {0., 15., 10.}} /* line string*/,
          {0., std::sqrt(2.) * 5. / 2., std::sqrt(2.) * 5. + 5. / 2., 2 * std::sqrt(2.) * 5. + 5.} /* ps */,
          {1., 1., 0., 1.} /* expected_slopes */
      },
      {
          // LineString with different z values.
          LineString3d{{0., 0., 0.}, {6., 3., 2.}, {10., 6., 2.}, {16., 9., 4.}, {10., 6., 2.}} /* line string*/,
          {0., 3.5, 10.5, 17., 26.} /* ps */,
          {2. / std::sqrt(45), 2. / std::sqrt(45), 0., 2. / std::sqrt(45), -2. / std::sqrt(45)} /* expected_slopes */
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
    const double dut = GetSlopeAtP(case_.line_string, case_.p[i], kTolerance);
    EXPECT_DOUBLE_EQ(case_.expected_slopes[i], dut);
  }
}

INSTANTIATE_TEST_CASE_P(GetSlopeAtPTestGroup, GetSlopeAtPTest, ::testing::ValuesIn(SlopeTestCases()));

class GetSlopeAtPSpecialCasesTest : public testing::Test {
 public:
  static constexpr double kTolerance{1e-12};
};

TEST_F(GetSlopeAtPSpecialCasesTest, InfinitySlope) {
  const double inf{std::numeric_limits<double>::infinity()};
  const LineString3d kOnlyZ{{0., 0., 0.}, {0., 0., 100.}, {0., 0., 0.}};
  EXPECT_EQ(inf, GetSlopeAtP(kOnlyZ, 50., kTolerance));
  EXPECT_EQ(-inf, GetSlopeAtP(kOnlyZ, 150., kTolerance));
}

struct HeadingTestCase {
  LineString3d line_string{};
  std::vector<double> p{};
  std::vector<double> expected_headings{};
};

std::vector<HeadingTestCase> HeadingTestCases() {
  return {
      {
          // LineString along y = 0 for x > 0;
          LineString3d{{0., 0., 0.}, {10., 0., 10.}} /* line string*/,
          {0., std::sqrt(2) * 10. / 2., std::sqrt(2) * 10} /* ps */,
          {0., 0., 0.} /* expected_slopes */
      },
      {
          // LineString along y = 0 for x < 0;
          LineString3d{{0., 0., 0.}, {-10., 0., 10.}} /* line string*/,
          {0., std::sqrt(2) * 10. / 2., std::sqrt(2) * 10} /* ps */,
          {M_PI, M_PI, M_PI} /* expected_slopes */
      },
      {
          // LineString along x = 0 for y > 0;
          LineString3d{{0., 0., 0.}, {0., 10., 10.}} /* line string*/,
          {0., std::sqrt(2) * 10. / 2., std::sqrt(2) * 10} /* ps */,
          {M_PI_2, M_PI_2, M_PI_2} /* expected_slopes */
      },
      {
          // LineString along x = 0 for y < 0;
          LineString3d{{0., 0., 0.}, {0., -10., 10.}} /* line string*/,
          {0., std::sqrt(2) * 10. / 2., std::sqrt(2) * 10} /* ps */,
          {-M_PI_2, -M_PI_2, -M_PI_2} /* expected_slopes */
      },
  };
}

class Get2DHeadingAtPTest : public ::testing::TestWithParam<HeadingTestCase> {
 public:
  static constexpr double kTolerance{1e-12};
  HeadingTestCase case_ = GetParam();
};

TEST_P(Get2DHeadingAtPTest, Test) {
  ASSERT_EQ(case_.p.size(), case_.expected_headings.size()) << ">>>>> Test case is ill-formed.";
  for (std::size_t i = 0; i < case_.p.size(); ++i) {
    const double dut = Get2DHeadingAtP(case_.line_string, case_.p[i], kTolerance);
    EXPECT_DOUBLE_EQ(case_.expected_headings[i], dut);
  }
}

INSTANTIATE_TEST_CASE_P(Get2DHeadingAtPTestGroup, Get2DHeadingAtPTest, ::testing::ValuesIn(HeadingTestCases()));

struct GetClosestPointToSegmentTestCase {
  std::pair<maliput::math::Vector3, maliput::math::Vector3> segment;
  std::vector<maliput::math::Vector3> eval_points;
  std::vector<ClosestPointToSegmentResult3d> expected_closest{};
};

std::vector<GetClosestPointToSegmentTestCase> GetClosestPointToSegmentTestCases() {
  return {
      {{{0., 0., 0.}, {10., 0., 0.}} /* segment */,
       {{5., 5., 0.}, {-5., 5., 0.}, {10., 5., 0.}, {15., 5., 0.}} /* eval points */,
       {
           {5., {5., 0., 0.}, 5.}, /* expected: p, point, distance */
           {0., {0., 0., 0.}, 5. * std::sqrt(2.)},
           {10., {10., 0., 0.}, 5.},
           {10., {10., 0., 0.}, 5. * std::sqrt(2.)},
       }},
      {{{0., -10., 0.}, {0., 10., 0.}} /* segment */,
       {{0., 0., 0.}, {1., 5., 0.}, {100., 5., 0.}, {100., 15., 0.}} /* eval points */,
       {
           {10., {0., 0., 0.}, 0.},
           {15., {0., 5., 0.}, 1.},
           {15., {0., 5., 0.}, 100.},
           {20., {0., 10., 0.}, std::sqrt(5. * 5. + 100. * 100.)},
       }},
      {{{0., 0., -10.}, {0., 0., 10.}} /* segment */,
       {{0., 0., 0.}, {1., 5., 3.}, {100., 15., 50.}, {100., 15., -50.}} /* eval points */,
       {
           {10., {0., 0., 0.}, 0.},
           {13., {0., 0., 3.}, std::sqrt(1. * 1. + 5. * 5.)},
           {20., {0., 0., 10.}, std::sqrt(40. * 40. + 100. * 100. + 15. * 15.)},
           {0., {0., 0., -10.}, std::sqrt(40. * 40. + 100. * 100. + 15. * 15.)},
       }},
      {{{-10., -10., 0.}, {10., 10., 0.}} /* segment */,
       {{-10., 0., 0.}, {0., 0., 0.}, {10., 0., 0.}} /* eval points */,
       {
           {5. * std::sqrt(2.), {-5., -5., 0.}, 5. * std::sqrt(2.)},
           {10 * std::sqrt(2.), {0., 0., 0.}, 0.},
           {15. * std::sqrt(2.), {5., 5., 0.}, 5. * std::sqrt(2.)},
       }},
  };
}

class GetClosestPointToSegmentTest : public ::testing::TestWithParam<GetClosestPointToSegmentTestCase> {
 public:
  static constexpr double kTolerance{1e-12};
  GetClosestPointToSegmentTestCase case_ = GetParam();
};

TEST_P(GetClosestPointToSegmentTest, Test) {
  ASSERT_EQ(case_.eval_points.size(), case_.expected_closest.size()) << ">>>>> Test case is ill-formed.";
  for (std::size_t i = 0; i < case_.eval_points.size(); ++i) {
    const auto dut =
        GetClosestPointToSegment(case_.segment.first, case_.segment.second, case_.eval_points[i], kTolerance);
    EXPECT_NEAR(case_.expected_closest[i].p, dut.p, kTolerance);
    EXPECT_TRUE(AssertCompare(CompareVectors(case_.expected_closest[i].point, dut.point, kTolerance)));
    EXPECT_NEAR(case_.expected_closest[i].distance, dut.distance, kTolerance);
  }
}

INSTANTIATE_TEST_CASE_P(GetClosestPointToSegmentTestGroup, GetClosestPointToSegmentTest,
                        ::testing::ValuesIn(GetClosestPointToSegmentTestCases()));

struct GetClosestPointLineStringTestCase {
  LineString3d line_string;
  std::vector<maliput::math::Vector3> eval_points;
  std::vector<ClosestPointResult3d> expected_closest{};
};

std::vector<GetClosestPointLineStringTestCase> GetClosestPointLineStringTestCases() {
  return {
      {LineString3d{{0., 0., 0.}, {10., 0., 0.}} /* line_string */,
       {{5., 5., 0.}, {-5., 5., 0.}, {10., 5., 0.}, {15., 5., 0.}} /* eval points */,
       {
           {5., {5., 0., 0.}, 5., {0, 1, {0., 10.}}}, /* expected: p, point, distance, segment */
           {0., {0., 0., 0.}, 5. * std::sqrt(2.), {0, 1, {0., 10.}}},
           {10., {10., 0., 0.}, 5., {0, 1, {0., 10.}}},
           {10., {10., 0., 0.}, 5. * std::sqrt(2.), {0, 1, {0., 10.}}},
       }},
      {LineString3d{{0., 0., 0.}, {10., 0., 0.}, {15., 0., 0.}} /* line_string */,
       {{12.5, 0., 0.}} /* eval points */,
       {
           {12.5, {12.5, 0., 0.}, 0., {1, 2, {10., 15.}}}, /* expected: p, point, distance, segment */
       }},
      {LineString3d{{-2., -4., -6.},
                    {0., -2., -4.},
                    {0., 0., 0.},
                    {10., 0., 0.},
                    {20., 10., 5.},
                    {40., 20., 10.}} /* line_string */,
       {{-1., -3., -5.}, {5., 5., 5.}} /* eval points */,
       {
           {std::sqrt(3.), {-1., -3., -5.}, 0., {0, 1, {0., 3.4641016151377544}}}, /* expected: p, point, distance,
                                                                                      segment */
           {std::sqrt(2. * 2. + 2. * 2. + 2. * 2.) + std::sqrt(2. * 2. + 4. * 4.) + 5.,
            {5., 0., 0.},
            5. * std::sqrt(2.),
            {2, 3, {7.936237570137334, 17.936237570137333}}},
       }},
  };
}

class GetClosestPointLineStringTest : public ::testing::TestWithParam<GetClosestPointLineStringTestCase> {
 public:
  static constexpr double kTolerance{1e-12};
  GetClosestPointLineStringTestCase case_ = GetParam();
};

TEST_P(GetClosestPointLineStringTest, Test) {
  ASSERT_EQ(case_.eval_points.size(), case_.expected_closest.size()) << ">>>>> Test case is ill-formed.";
  for (std::size_t i = 0; i < case_.eval_points.size(); ++i) {
    const auto dut = GetClosestPoint(case_.line_string, case_.eval_points[i], kTolerance);
    EXPECT_NEAR(case_.expected_closest[i].p, dut.p, kTolerance);
    EXPECT_TRUE(AssertCompare(CompareVectors(case_.expected_closest[i].point, dut.point, kTolerance)));
    EXPECT_NEAR(case_.expected_closest[i].distance, dut.distance, kTolerance);
    EXPECT_EQ(case_.expected_closest[i].segment.idx_start, dut.segment.idx_start);
    EXPECT_EQ(case_.expected_closest[i].segment.idx_end, dut.segment.idx_end);
    EXPECT_NEAR(case_.expected_closest[i].segment.p_interval.min, dut.segment.p_interval.min, kTolerance);
    EXPECT_NEAR(case_.expected_closest[i].segment.p_interval.max, dut.segment.p_interval.max, kTolerance);
  }
}

INSTANTIATE_TEST_CASE_P(GetClosestPointLineStringTestGroup, GetClosestPointLineStringTest,
                        ::testing::ValuesIn(GetClosestPointLineStringTestCases()));

std::vector<GetClosestPointLineStringTestCase> GetClosestPointIn2dLineStringTestCases() {
  return {
      // Increasing in elevation line string and eval points are below(in Z) the line string.
      //
      // ^ z
      // |         *{10., 0., 10.}
      // |       /
      // |     /
      // |   /
      // | /
      // |-----*-------> x
      //       {5., 0., 0.} (eval point)
      // Because of the 2d projection the closest point is the one in the half of the line segment. Which is the desired
      // value.
      //
      {LineString3d{{0., 0., 0.}, {10., 0., 10.}} /* line_string */,
       {{5., 0., 0.}, {5., -2., 0.}} /* eval points */,
       {
           {10 * std::sqrt(2.) / 2., {5., 0., 5.}, 5., {0, 1, {0., 10. * std::sqrt(2.)}}}, /* expected: p, point,
                                                                                              distance, segment */
           {10 * std::sqrt(2.) / 2.,
            {5., 0., 5.},
            5.385164807134504,
            {0, 1, {0., 10. * std::sqrt(2.)}}}, /* expected: p, point, distance, segment */
       }},
  };
}

class GetClosestPointIn2dLineStringTest : public ::testing::TestWithParam<GetClosestPointLineStringTestCase> {
 public:
  static constexpr double kTolerance{1e-12};
  GetClosestPointLineStringTestCase case_ = GetParam();
};

TEST_P(GetClosestPointIn2dLineStringTest, Test) {
  ASSERT_EQ(case_.eval_points.size(), case_.expected_closest.size()) << ">>>>> Test case is ill-formed.";
  for (std::size_t i = 0; i < case_.eval_points.size(); ++i) {
    const auto dut = GetClosestPointUsing2dProjection(case_.line_string, case_.eval_points[i], kTolerance);
    EXPECT_NEAR(case_.expected_closest[i].p, dut.p, kTolerance);
    EXPECT_TRUE(AssertCompare(CompareVectors(case_.expected_closest[i].point, dut.point, kTolerance)));
    EXPECT_NEAR(case_.expected_closest[i].distance, dut.distance, kTolerance);
    EXPECT_EQ(case_.expected_closest[i].segment.idx_start, dut.segment.idx_start);
    EXPECT_EQ(case_.expected_closest[i].segment.idx_end, dut.segment.idx_end);
    EXPECT_NEAR(case_.expected_closest[i].segment.p_interval.min, dut.segment.p_interval.min, kTolerance);
    EXPECT_NEAR(case_.expected_closest[i].segment.p_interval.max, dut.segment.p_interval.max, kTolerance);
  }
}

INSTANTIATE_TEST_CASE_P(GetClosestPointIn2dLineStringTestGroup, GetClosestPointIn2dLineStringTest,
                        ::testing::ValuesIn(GetClosestPointIn2dLineStringTestCases()));

struct ComputeDistanceTestCase {
  LineString3d lhs;
  LineString3d rhs;
  double expected_distance{0.};
};

std::vector<ComputeDistanceTestCase> ComputeDistanceTestCases() {
  return {
      {
          LineString3d{{0., 0., 0.}, {10., 0., 0.}} /* lhs */, LineString3d{{0., 0., 0.}, {10., 0., 0.}} /* rhs */,
          0. /* expected_distance */
      },
      {
          LineString3d{{0., 5., 0.}, {10., 5., 0.}} /* lhs */, LineString3d{{0., 0., 0.}, {10., 0., 0.}} /* rhs */,
          5. /* expected_distance */
      },
      {
          LineString3d{{0., 5., 0.}, {10., 5., 0.}} /* lhs */,
          LineString3d{{0., 0., 0.}, {2., 0., 0.}, {4., 0., 0.}, {6., 0., 0.}, {8., 0., 0.}, {10., 0., 0.}} /* rhs */,
          5. /* expected_distance */
      },
      {
          LineString3d{{0., 0., 0.}, {2., 0., 0.}, {4., 0., 0.}, {6., 0., 0.}, {8., 0., 0.}, {10., 0., 0.}} /* lhs */,
          LineString3d{{0., 5., 0.}, {10., 5., 0.}} /* rhs */, 5. /* expected_distance */
      },
  };
}

class ComputeDistanceTest : public ::testing::TestWithParam<ComputeDistanceTestCase> {
 public:
  static constexpr double kTolerance{1e-12};
  ComputeDistanceTestCase case_ = GetParam();
};

TEST_P(ComputeDistanceTest, Test) {
  const auto dut = ComputeDistance(case_.lhs, case_.rhs, kTolerance);
  EXPECT_NEAR(case_.expected_distance, dut, kTolerance);
}

INSTANTIATE_TEST_CASE_P(ComputeDistanceTestGroup, ComputeDistanceTest, ::testing::ValuesIn(ComputeDistanceTestCases()));

// Test for arc road centerline computation.
// This test verifies that ComputeCenterline3d correctly handles arc geometries
// where the first centerline point lies on the entry segment.
// Bug fix: The algorithm was incorrectly flagging centerline segments as
// intersecting the entry line when the intersection was at t=0 (start point).
class ComputeCenterlineArcTest : public ::testing::Test {
 public:
  static constexpr double kTolerance{1e-6};

  // Generate arc points for a given radius and number of samples
  static std::vector<Vector3> GenerateArcPoints(double center_x, double center_y, double radius, double start_angle,
                                                double end_angle, int num_points) {
    std::vector<Vector3> points;
    for (int i = 0; i < num_points; ++i) {
      const double t = static_cast<double>(i) / (num_points - 1);
      const double angle = start_angle + t * (end_angle - start_angle);
      const double x = center_x + radius * std::cos(angle);
      const double y = center_y + radius * std::sin(angle);
      points.push_back({x, y, 0.0});
    }
    return points;
  }

  // Calculate total length of a polyline
  static double CalculateLength(const LineString3d& line) { return line.length(); }
};

// Test arc road where the first centerline point is on the entry segment.
// Arc parameters: center at (0, 40), radius 40m for road centerline,
// left boundary at radius 41m (outer), right boundary at radius 39m (inner).
// Arc spans from -π/2 to a small angle for a short test arc.
TEST_F(ComputeCenterlineArcTest, ArcRoadWithFirstPointOnEntry) {
  // Arc parameters for a simple test arc
  const double center_x = 0.0;
  const double center_y = 10.0;  // Center 10m to the left of start
  const double road_radius = 10.0;
  const double lane_half_width = 1.0;
  const double start_angle = -M_PI / 2.0;  // -90 degrees (pointing down from center)
  const double arc_angle = M_PI / 4.0;     // 45 degrees arc
  const double end_angle = start_angle + arc_angle;
  const int num_points = 5;

  // Left boundary (outer) at radius + lane_half_width
  const auto left_points =
      GenerateArcPoints(center_x, center_y, road_radius + lane_half_width, start_angle, end_angle, num_points);
  // Right boundary (inner) at radius - lane_half_width
  const auto right_points =
      GenerateArcPoints(center_x, center_y, road_radius - lane_half_width, start_angle, end_angle, num_points);

  const LineString3d left(left_points);
  const LineString3d right(right_points);

  // Verify boundary setup: first points should be at x ≈ 0 (on the entry line)
  EXPECT_NEAR(left_points[0].x(), 0.0, kTolerance);
  EXPECT_NEAR(right_points[0].x(), 0.0, kTolerance);

  // Compute centerline
  const auto centerline = ComputeCenterline3d(left, right);

  // The centerline should have multiple points, not just start and end
  // If the bug exists, it will only have 2 points (start and end) and wrong length
  EXPECT_GT(centerline.size(), 2u) << "Centerline should have more than just start and end points";

  // Calculate expected centerline length (arc length at road_radius)
  const double expected_centerline_length = road_radius * arc_angle;

  // The computed centerline length should be close to the expected arc length
  const double computed_length = CalculateLength(centerline);
  EXPECT_NEAR(computed_length, expected_centerline_length, 0.5)
      << "Centerline length should be approximately " << expected_centerline_length << "m but got " << computed_length
      << "m";

  // Verify first and last centerline points are midpoints of boundaries
  const Vector3 expected_first = (left_points[0] + right_points[0]) / 2.0;
  const Vector3 expected_last = (left_points.back() + right_points.back()) / 2.0;

  EXPECT_TRUE(AssertCompare(CompareVectors(expected_first, centerline.first(), kTolerance)));
  EXPECT_TRUE(AssertCompare(CompareVectors(expected_last, centerline.last(), kTolerance)));
}

// Test with a larger arc (similar to the arc_lane.gpkg test case)
TEST_F(ComputeCenterlineArcTest, LargeArcRoad) {
  // Arc parameters matching arc_lane.gpkg
  const double center_x = 0.0;
  const double center_y = 40.0;  // Radius = 40m
  const double road_radius = 40.0;
  const double lane_half_width = 1.0;
  const double start_angle = -M_PI / 2.0;
  const double arc_length = 100.0;                    // 100m arc
  const double arc_angle = arc_length / road_radius;  // 2.5 radians
  const double end_angle = start_angle + arc_angle;
  const int num_points = 21;  // Same as arc_lane.gpkg

  // Left boundary (outer)
  const auto left_points =
      GenerateArcPoints(center_x, center_y, road_radius + lane_half_width, start_angle, end_angle, num_points);
  // Right boundary (inner)
  const auto right_points =
      GenerateArcPoints(center_x, center_y, road_radius - lane_half_width, start_angle, end_angle, num_points);

  const LineString3d left(left_points);
  const LineString3d right(right_points);

  // Calculate boundary lengths for reference
  const double left_length = left.length();
  const double right_length = right.length();
  const double expected_left_length = (road_radius + lane_half_width) * arc_angle;   // ~102.5m
  const double expected_right_length = (road_radius - lane_half_width) * arc_angle;  // ~97.5m

  EXPECT_NEAR(left_length, expected_left_length, 1.0);
  EXPECT_NEAR(right_length, expected_right_length, 1.0);

  // Compute centerline
  const auto centerline = ComputeCenterline3d(left, right);

  // Should have many points, not just 2
  EXPECT_GT(centerline.size(), 2u) << "Centerline should have more than just start and end points";

  // The centerline length should be approximately 100m (road_radius * arc_angle)
  const double expected_centerline_length = road_radius * arc_angle;
  const double computed_length = CalculateLength(centerline);

  // Allow some tolerance due to chord approximation
  EXPECT_NEAR(computed_length, expected_centerline_length, 2.0)
      << "Centerline length should be approximately " << expected_centerline_length << "m but got " << computed_length
      << "m";
}

}  // namespace
}  // namespace test
}  // namespace utility
}  // namespace geometry
}  // namespace maliput_sparse
