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
#include "maliput_sparse/parser/lane.h"

#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

#include "maliput_sparse/geometry/line_string.h"

namespace maliput_sparse {
namespace parser {
namespace test {
namespace {

using maliput_sparse::geometry::LineString3d;

// -- SpeedLimit tests --------------------------------------------------------

class SpeedLimitTest : public ::testing::Test {
 protected:
  const SpeedLimit dut{0., 100., 0., 30., "Urban road. [m/s]", 0};
};

TEST_F(SpeedLimitTest, Members) {
  EXPECT_DOUBLE_EQ(0., dut.s_start);
  EXPECT_DOUBLE_EQ(100., dut.s_end);
  EXPECT_DOUBLE_EQ(0., dut.min);
  EXPECT_DOUBLE_EQ(30., dut.max);
  EXPECT_EQ("Urban road. [m/s]", dut.description);
  EXPECT_EQ(0, dut.severity);
}

TEST_F(SpeedLimitTest, DefaultConstruction) {
  const SpeedLimit default_dut{};
  EXPECT_DOUBLE_EQ(0., default_dut.s_start);
  EXPECT_DOUBLE_EQ(0., default_dut.s_end);
  EXPECT_DOUBLE_EQ(0., default_dut.min);
  EXPECT_DOUBLE_EQ(0., default_dut.max);
  EXPECT_TRUE(default_dut.description.empty());
  EXPECT_EQ(0, default_dut.severity);
}

TEST_F(SpeedLimitTest, EqualityOperator) {
  const SpeedLimit dut2{0., 100., 0., 30., "Urban road. [m/s]", 0};
  EXPECT_EQ(dut, dut2);
}

TEST_F(SpeedLimitTest, InequalityOnMax) {
  const SpeedLimit other{0., 100., 0., 60., "Urban road. [m/s]", 0};
  EXPECT_FALSE(dut == other);
}

TEST_F(SpeedLimitTest, InequalityOnSeverity) {
  const SpeedLimit other{0., 100., 0., 30., "Urban road. [m/s]", 1};
  EXPECT_FALSE(dut == other);
}

// -- Lane tests (existing) ---------------------------------------------------

class LaneTest : public ::testing::Test {
 protected:
  const Lane::Id id{"lane_id"};
  const LineString3d left{{1., 1., 1.}, {10., 1., 1.}};
  const LineString3d right{{1., -1., 1.}, {10., -1., 1.}};
  const std::optional<Lane::Id> left_lane_id{"left_lane_id"};
  const std::optional<Lane::Id> right_lane_id{"right_lane_id"};
  const std::unordered_map<Lane::Id, LaneEnd> successors{
      {Lane::Id{"successor_1"}, {Lane::Id{"successor_1"}, LaneEnd::Which::kStart}},
      {Lane::Id{"successor_2"}, {Lane::Id{"successor_2"}, LaneEnd::Which::kFinish}}};
  const std::unordered_map<Lane::Id, LaneEnd> predecessors{
      {Lane::Id{"predecessor_1"}, {Lane::Id{"predecessor_1"}, LaneEnd::Which::kStart}},
      {Lane::Id{"predecessor_2"}, {Lane::Id{"predecessor_2"}, LaneEnd::Which::kFinish}}};
  const Lane dut{id, left, right, left_lane_id, right_lane_id, successors, predecessors};
};

TEST_F(LaneTest, Members) {
  EXPECT_EQ(id, dut.id);
  EXPECT_EQ(left, dut.left);
  EXPECT_EQ(right, dut.right);
  EXPECT_EQ(left_lane_id, dut.left_lane_id);
  EXPECT_EQ(right_lane_id, dut.right_lane_id);
  EXPECT_EQ(successors, dut.successors);
  EXPECT_EQ(predecessors, dut.predecessors);
  EXPECT_TRUE(dut.speed_limits.empty());
}

TEST_F(LaneTest, EqualityOperator) {
  const Lane dut2 = dut;
  EXPECT_EQ(dut, dut2);
}

// -- Lane with speed limits tests -------------------------------------------

class LaneWithSpeedLimitsTest : public ::testing::Test {
 protected:
  const Lane::Id id{"lane_id"};
  const LineString3d left{{0., 1., 0.}, {100., 1., 0.}};
  const LineString3d right{{0., -1., 0.}, {100., -1., 0.}};
  const std::vector<SpeedLimit> speed_limits{
      {0., 50., 0., 30., "Zone A. [m/s]", 0},
      {50., 100., 0., 15., "Zone B. [m/s]", 0},
  };
  const Lane dut{id, left, right, std::nullopt, std::nullopt, {}, {}, speed_limits};
};

TEST_F(LaneWithSpeedLimitsTest, Members) {
  ASSERT_EQ(2u, dut.speed_limits.size());
  EXPECT_EQ(speed_limits[0], dut.speed_limits[0]);
  EXPECT_EQ(speed_limits[1], dut.speed_limits[1]);
}

TEST_F(LaneWithSpeedLimitsTest, EqualityOperator) {
  const Lane dut2 = dut;
  EXPECT_EQ(dut, dut2);
}

TEST_F(LaneWithSpeedLimitsTest, InequalityWhenSpeedLimitsDiffer) {
  const Lane other{id, left, right, std::nullopt, std::nullopt, {}, {}, {{0., 100., 0., 60., "Highway. [m/s]", 0}}};
  EXPECT_FALSE(dut == other);
}

}  // namespace
}  // namespace test
}  // namespace parser
}  // namespace maliput_sparse
