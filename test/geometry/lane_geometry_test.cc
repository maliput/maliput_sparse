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
#include "maliput_sparse/geometry/lane_geometry.h"

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput/math/vector.h>
#include <maliput/test_utilities/maliput_math_compare.h>

namespace maliput_sparse {
namespace geometry {
namespace test {
namespace {

using maliput::math::Vector2;
using maliput::math::Vector3;

class LaneGeometryBasicTest : public testing::Test {
 public:
  const LineString3d centerline{{0., 0., 0.}, {50., 0., 0.}, {100., 0., 0.}};
  const LineString3d left{{0., 2., 0.}, {100., 2., 0.}};
  const LineString3d right{{0., -2., 0.}, {100., -2., 0.}};
  const double kTolerance{1e-12};
  const double kScaleLength{1.};
};

TEST_F(LaneGeometryBasicTest, ConstructorWithCenter) {
  const LaneGeometry lane(centerline, left, right, kTolerance, kScaleLength);
  EXPECT_EQ(lane.centerline(), centerline);
}

TEST_F(LaneGeometryBasicTest, ConstructorWithoutCenter) {
  const LaneGeometry lane(left, right, kTolerance, kScaleLength);
  EXPECT_EQ(lane.centerline(), centerline);
}

struct OrientationTestCase {
  LineString3d left{};
  LineString3d right{};
  std::vector<double> p{};
  std::vector<maliput::math::RollPitchYaw> expected_rpy{};
};

std::vector<OrientationTestCase> OrientationTestCases() {
  return {
      {
          // No elevation along x
          LineString3d{{0., 2., 0.}, {100., 2., 0.}} /* left*/,
          LineString3d{{0., -2., 0.}, {100., -2., 0.}} /* right*/,
          {0., 50., 100.} /* p */,
          {{0., 0., 0.}, {0., 0., 0.}, {0., 0., 0.}} /* expected_rpy */
      },
      {
          // No elevation along -x
          LineString3d{{0., 2., 0.}, {-100., 2., 0.}} /* line string*/,
          LineString3d{{0., -2., 0.}, {-100., -2., 0.}} /* line string*/,
          {0., 50., 100.} /* p */,
          {{0., 0., M_PI}, {0., 0., M_PI}, {0., 0., M_PI}} /* expected_rpy */
      },
      {
          // Linear Elevation
          LineString3d{{0., 2., 0.}, {100., 2., 100.}} /* left*/,
          LineString3d{{0., -2., 0.}, {100., -2., 100.}} /* right*/,
          {0.} /* p */,
          {{0., -M_PI_4, 0.}} /* expected_rpy */
      },
      {
          // Linear Elevation with negative yaw.
          LineString3d{{1., 1., 0.}, {1. + 70.71067811865476, 1. - 70.71067811865476, 100.}} /* left*/,
          LineString3d{{-1., -1., 0.}, {-1. + 70.71067811865476, -1. - 70.71067811865476, 100.}} /* right*/,
          {
              0.,
              141.4213562373095,
          } /* p */,
          {{0., -M_PI_4, -M_PI_4}, {0., -M_PI_4, -M_PI_4}} /* expected_rpy */
      },
      {
          // Increase elevation + plateau + decrease elevation.
          LineString3d{{0., 2., 0.}, {10., 2., 10.}, {20., 2., 10.}, {30., 2., 0.}} /* left*/,
          LineString3d{{0., -2., 0.}, {10., -2., 10.}, {20., -2., 10.}, {30., -2., 0.}} /* right*/,
          {0., 1 + std::sqrt(2.) * 10., 1 + std::sqrt(2.) * 10. + 10.} /* p */,
          {{0., -M_PI_4, 0.}, {0., 0., 0.}, {0., M_PI_4, 0.}} /* expected_rpy */
      },
  };
}

class OrientationTest : public ::testing::TestWithParam<OrientationTestCase> {
 public:
  static constexpr double kTolerance{1e-12};
  OrientationTestCase case_ = GetParam();
};

TEST_P(OrientationTest, Test) {
  ASSERT_EQ(case_.p.size(), case_.expected_rpy.size()) << ">>>>> Test case is ill-formed.";
  const LaneGeometry lane_geometry{case_.left, case_.right, 1e-3, 1.};
  for (std::size_t i = 0; i < case_.p.size(); ++i) {
    const auto rpy = lane_geometry.Orientation(case_.p[i]);
    EXPECT_EQ(case_.expected_rpy[i].vector(), rpy.vector())
        << "Expected RPY: " << case_.expected_rpy[i].vector() << " vs RPY: " << rpy.vector();
  }
}

INSTANTIATE_TEST_CASE_P(OrientationTestGroup, OrientationTest, ::testing::ValuesIn(OrientationTestCases()));

struct WAndWInverseCase {
  LineString3d left{};
  LineString3d right{};
  std::vector<Vector3> prh{};
  std::vector<Vector3> expected_w{};
};

std::vector<WAndWInverseCase> WAndWInverseCases() {
  return {
      {
          // No elevation along x
          LineString3d{{0., 2., 0.}, {100., 2., 0.}} /* left*/,
          LineString3d{{0., -2., 0.}, {100., -2., 0.}} /* right*/,
          {{0., 0., 0.}, {50., 0., 0.}, {100., 0., 0.}, {0., 4., 2.}, {50., -2., -2.}, {100., 7., -1.}} /* prh */,
          {{0., 0., 0.}, {50., 0., 0.}, {100., 0., 0.}, {0., 4., 2.}, {50., -2., -2.}, {100., 7., -1.}} /* expected_w */
      },
      {
          // No elevation along -x
          LineString3d{{0., 2., 0.}, {-100., 2., 0.}} /* line string*/,
          LineString3d{{0., -2., 0.}, {-100., -2., 0.}} /* line string*/,
          {{0., 0., 0.}, {50., 0., 0.}, {100., 0., 0.}, {0., 5., 1.}, {50., -6., -12.}, {100., -24., 4.}} /* prh */,
          {{0., 0., 0.},
           {-50., 0., 0.},
           {-100., 0., 0.},
           {0., -5., 1.},
           {-50., 6., -12.},
           {-100., 24., 4.}} /* expected_w
                              */
      },
      {
          // Linear Elevation. `h` value implies a movement in x.
          LineString3d{{0., 2., 0.}, {100., 2., 100.}} /* left*/,
          LineString3d{{0., -2., 0.}, {100., -2., 100.}} /* right*/,
          {{0., 0., 0.},
           {std::sqrt(2.) * 50., 0., 0.},
           {std::sqrt(2.) * 100., 0., 0.},
           {0., 5., 0.},
           {std::sqrt(2.) * 50., 0., 5.},
           {std::sqrt(2.) * 100., 5., 5.}} /* prh */,
          {{0., 0., 0.},
           {50., 0., 50.},
           {100., 0., 100.},
           {0., 5., 0.},
           {50. - 5 * std::sqrt(2.) / 2., 0., 50. + 5 * std::sqrt(2.) / 2.},
           {100. - 5 * std::sqrt(2.) / 2., 5., 100. + 5 * std::sqrt(2.) / 2.}} /* expected_w */
      },
      {
          // Linear Elevation with negative yaw. `h` value implies a movement in x and y.
          LineString3d{{1., 1., 0.}, {1. + 70.71067811865476, 1. - 70.71067811865476, 100.}} /* left*/,
          LineString3d{{-1., -1., 0.}, {-1. + 70.71067811865476, -1. - 70.71067811865476, 100.}} /* right*/,
          {{0., 0., 0.},
           {141.4213562373095, 0., 0.},
           {0., 5., 0.},
           {0., 0., -2.},
           {0., 5., -2.},
           {141.4213562373095, 5., -2.}} /* prh */,
          {{0., 0., 0.},
           {70.71067811865476, -70.71067811865476, 100.},
           {0. + 5 * std::sqrt(2.) / 2., 0. + 5 * std::sqrt(2.) / 2., 0.},
           {1., -1., -2 * std::sqrt(2.) / 2.},
           {1. + 5 * std::sqrt(2.) / 2., -1. + 5 * std::sqrt(2.) / 2., -2 * std::sqrt(2.) / 2.},
           {70.71067811865476 + 1. + 5 * std::sqrt(2.) / 2., -70.71067811865476 - 1. + 5 * std::sqrt(2.) / 2.,
            100. - 2 * std::sqrt(2.) / 2.}} /* expected_w
                                             */
      },
  };
}

class WAndWInverseTest : public ::testing::TestWithParam<WAndWInverseCase> {
 public:
  static constexpr double kTolerance{1e-12};
  WAndWInverseCase case_ = GetParam();
};

TEST_P(WAndWInverseTest, Test) {
  ASSERT_EQ(case_.prh.size(), case_.expected_w.size()) << ">>>>> Test case is ill-formed.";
  const LaneGeometry lane_geometry{case_.left, case_.right, 1e-3, 1.};
  for (std::size_t i = 0; i < case_.prh.size(); ++i) {
    const auto dut_w = lane_geometry.W(case_.prh[i]);
    EXPECT_TRUE(maliput::math::test::CompareVectors(case_.expected_w[i], dut_w, kTolerance));
    const auto dut_w_inverse = lane_geometry.WInverse(case_.expected_w[i]);
    EXPECT_TRUE(maliput::math::test::CompareVectors(case_.prh[i], dut_w_inverse, kTolerance));
  }
}

INSTANTIATE_TEST_CASE_P(WAndWInverseTestGroup, WAndWInverseTest, ::testing::ValuesIn(WAndWInverseCases()));

struct WDotCase {
  LineString3d left{};
  LineString3d right{};
  std::vector<Vector3> prh{};
  std::vector<Vector3> expected_w_dot{};
};

std::vector<WDotCase> WDotCases() {
  return {
      {
          // Straight flat lane.
          LineString3d{{0., 2., 0.}, {100., 2., 0.}} /* left*/,
          LineString3d{{0., -2., 0.}, {100., -2., 0.}} /* right*/,
          {{0., 0., 0.}, {50., 0., 0.}, {100., 0., 0.}, {0., 4., 2.}, {50., -2., -2.}, {100., 7., -1.}} /* prh */,
          {{1., 0., 0.}, {1., 0., 0.}, {1., 0., 0.}, {1., 0., 0.}, {1., 0., 0.}, {1., 0., 0.}} /* expected_w_dot */
      },
      {
          // Straight linearly elevated lane.
          LineString3d{{0., 2., 0.}, {100., 2., 100.}} /* left*/,
          LineString3d{{0., -2., 0.}, {100., -2., 100.}} /* right*/,
          {{0., 0., 0.}, {50., -2., -2.}, {100. * std::sqrt(2.), 7., -1.}} /* prh */,
          {{std::sqrt(2.) / 2., 0., std::sqrt(2.) / 2.},
           {std::sqrt(2.) / 2., 0., std::sqrt(2.) / 2.},
           {std::sqrt(2.) / 2., 0., std::sqrt(2.) / 2.}} /* expected_w_dot */
      },
      {
          // Arc-like lane:
          //    | |  --> no elevation
          //  __/ /  --> no elevation
          //  ___/   --> linear elevation
          LineString3d{{0., 2., 0.}, {100., 2., 100.}, {200., 102., 100.}, {200., 200., 100.}} /* left*/,
          LineString3d{{0., -2., 0.}, {100., -2., 100.}, {204., 102., 100.}, {204., 200., 100.}} /* right*/,
          {{50., -2., -2.}, {100. * (1. + std::sqrt(2.)), 0., 0.}, {100. * (2. + std::sqrt(2.)), 0., 0.}} /* prh */,
          {
              {std::sqrt(2.) / 2., 0., std::sqrt(2.) / 2.},
              {std::sqrt(2.) / 2., std::sqrt(2.) / 2., 0.},
              {0., 1., 0.},
          } /* expected_w_dot */
      },
      // TODO(francocipollone): Once r_dot is taking into account in WDot, this test case should be uncommented.
      // {
      // Straight increasing width.
      // LineString3d{{0., 2., 0.}, {100., 102., 0.}} /* left*/,
      // LineString3d{{0., -2., 0.}, {100., -2., 0.}} /* right*/,
      // {{0., 0., 0.}, {50., -2., -2.}, {100. * std::sqrt(2.), 7., -1.}} /* prh */,
      // {{std::sqrt(2.)/2., std::sqrt(2.)/2., 0.}, {std::sqrt(2.)/2., std::sqrt(2.)/2., 0.}, {std::sqrt(2.)/2.,
      // std::sqrt(2.)/2., 0.}} /* expected_w_dot */
      // },
  };
}

class WDotTest : public ::testing::TestWithParam<WDotCase> {
 public:
  static constexpr double kTolerance{1e-12};
  WDotCase case_ = GetParam();
};

TEST_P(WDotTest, Test) {
  ASSERT_EQ(case_.prh.size(), case_.expected_w_dot.size()) << ">>>>> Test case is ill-formed.";
  const LaneGeometry lane_geometry{case_.left, case_.right, 1e-3, 1.};
  for (std::size_t i = 0; i < case_.prh.size(); ++i) {
    const auto dut = lane_geometry.WDot(case_.prh[i]);
    EXPECT_TRUE(maliput::math::test::CompareVectors(case_.expected_w_dot[i], dut, kTolerance));
  }
}

INSTANTIATE_TEST_CASE_P(WDotTestGroup, WDotTest, ::testing::ValuesIn(WDotCases()));

struct RBoundsCase {
  LineString3d left{};
  LineString3d right{};
  double length{};
  std::vector<double> p{};
  std::vector<double> expected_left_p{};
  std::vector<double> expected_right_p{};
  std::vector<maliput::api::RBounds> expected_r_bounds{};
};

std::vector<RBoundsCase> RBoundsCases() {
  return {
      {
          // Straight lane.
          LineString3d{{0., 2., 0.}, {100., 2., 0.}} /* left */,
          LineString3d{{0., -2., 0.}, {100., -2., 0.}} /* right */,
          100. /* length */,
          {0., 50., 100.} /* p */,
          {0., 50., 100.} /* expected_left_p */,
          {0., 50., 100.} /* expected_right_p */,
          {{-2., 2.}, {-2., 2.}, {-2., 2.}} /* expected_r_bounds */
      },
      {
          // Left LineString with linear elevation.
          // Right LineString with zero elevation.
          LineString3d{{0., 2., 0.}, {10., 2., 10.}} /* left */,
          LineString3d{{0., -2., 0.}, {10., -2., 0.}} /* right */,
          // centerline: {0, 0, 0}, {5, 0, 0}, {10, 0, 5}
          // length: 5.*(std::sqrt(2.) + 1.) = 12.071067811865476
          12.071067811865476 /* length */,
          {0., 2.5, 5., 7.5, 12.071067811865476} /* p */,
          {0., 2.928932188134525, 5.85786437626905, 8.786796564403575, 14.142135623730951} /* expected_left_p */,
          {0., 2.071067811865475, 4.14213562373095, 6.213203435596426, 10.} /* expected_right_p */,
          {{-2., 2.},
           {-2.6864458697336402, 2.045478629078747},
           {-4.1070628975017831, 2.1762194944608613},
           {-5.4419740342632519, 2.4671732743644945},
           {-5.3851648071345037, 5.3851648071345037}} /* expected_r_bounds */
      },
      {
          // Arc-like lane:
          /*
                  ______________
                 /              \
                /    ________    \
               /    /        \    \
              /    /          \    \
          */
          LineString3d{
              {0., 0., 0.},
              {5., 5., 0.},
              {15., 5., 0.},
              {20., 0., 0.},
          } /* left */,
          LineString3d{{2., -2., 0.}, {5., 1., 0.}, {15., 1., 0.}, {18., -2., 0.}} /* right */,
          // centerline:
          // {1., -1., 0.}, {2.5, 0.5, 0.}, {5., 3., 0.}, {10., 3., 0.}, {15., 3., 0.}, {17.5, 0.5, 0.}, {19., -1., 0.}
          21.313708498984759 /* length */,
          {0., 5.656854249492381, 10.656854249492381, 15.656854249492381, 21.313708498984759} /* p */,
          {0., 6.407544820340815, 12.071067811865479, 17.73459080339014, 24.14213562373095} /* expected_left_p */,
          {0., 4.906163678643947, 9.242640687119287, 13.579117695594627, 18.48528137423857} /* expected_right_p */,
          {{-1.4142135623730951, 1.4142135623730951},
           {-2.1071930999037169, 1.6011047227338844},
           {-2., 2.},
           {-2.1071930999037161, 1.6011047227338822},
           {-1.4142135623730951, 1.4142135623730951}} /* expected_r_bounds */
      },
  };
}

class RBoundsCaseTest : public ::testing::TestWithParam<RBoundsCase> {
 public:
  static constexpr double kLinearTolerance{1e-3};
  static constexpr double kScaleLength{1.};
  RBoundsCase case_ = GetParam();
};

TEST_P(RBoundsCaseTest, Test) {
  ASSERT_EQ(case_.p.size(), case_.expected_left_p.size()) << ">>>>> Test case is ill-formed.";
  ASSERT_EQ(case_.p.size(), case_.expected_right_p.size()) << ">>>>> Test case is ill-formed.";
  ASSERT_EQ(case_.p.size(), case_.expected_r_bounds.size()) << ">>>>> Test case is ill-formed.";

  const LaneGeometry lane_geometry{case_.left, case_.right, kLinearTolerance, kScaleLength};
  ASSERT_DOUBLE_EQ(case_.length, lane_geometry.ArcLength());
  for (std::size_t i = 0; i < case_.p.size(); ++i) {
    const double p_left = lane_geometry.FromCenterPToLateralP(LaneGeometry::LineStringType::kLeftBoundary, case_.p[i]);
    const double p_right =
        lane_geometry.FromCenterPToLateralP(LaneGeometry::LineStringType::kRightBoundary, case_.p[i]);
    EXPECT_DOUBLE_EQ(case_.expected_left_p[i], p_left);
    EXPECT_DOUBLE_EQ(case_.expected_right_p[i], p_right);
    const maliput::api::RBounds r_bounds = lane_geometry.RBounds(case_.p[i]);
    EXPECT_DOUBLE_EQ(case_.expected_r_bounds[i].min(), r_bounds.min());
    EXPECT_DOUBLE_EQ(case_.expected_r_bounds[i].max(), r_bounds.max());
  }
}

INSTANTIATE_TEST_CASE_P(RBoundsCaseTestGroup, RBoundsCaseTest, ::testing::ValuesIn(RBoundsCases()));

}  // namespace
}  // namespace test
}  // namespace geometry
}  // namespace maliput_sparse
