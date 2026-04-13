// BSD 3-Clause License
//
// Copyright (c) 2022-2026, Woven by Toyota.
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
#include "maliput_sparse/loader/road_network_loader.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/api/lane_data.h>
#include <maliput/api/regions.h>
#include <maliput/api/rules/range_value_rule.h>
#include <maliput/api/rules/road_rulebook.h>
#include <maliput/base/rule_registry.h>

#include "maliput_sparse/geometry/line_string.h"
#include "maliput_sparse/loader/builder_configuration.h"
#include "maliput_sparse/parser/connection.h"
#include "maliput_sparse/parser/junction.h"
#include "maliput_sparse/parser/lane.h"
#include "maliput_sparse/parser/parser.h"
#include "maliput_sparse/parser/segment.h"

namespace maliput_sparse {
namespace loader {
namespace test {
namespace {

using maliput_sparse::geometry::LineString3d;

/// Mock parser for testing the RoadNetworkLoader.
class MockParser : public parser::Parser {
 public:
  MockParser(const std::unordered_map<parser::Junction::Id, parser::Junction>& junctions,
             const std::vector<parser::Connection>& connections)
      : junctions_(junctions), connections_(connections) {}

 private:
  const std::unordered_map<parser::Junction::Id, parser::Junction>& DoGetJunctions() const override {
    return junctions_;
  }
  const std::vector<parser::Connection>& DoGetConnections() const override { return connections_; }
  const std::string& DoGetGeoReferenceInfo() const override {
    static const std::string empty_geo_ref = "";
    return empty_geo_ref;
  }
  const std::unordered_map<parser::Junction::Id, parser::Junction> junctions_;
  const std::vector<parser::Connection> connections_;
};

/// Creates a simple single-lane road for testing.
/// The lane spans from (0,0,0) to (100,0,0) with 2m width.
parser::Lane CreateSimpleLane(const parser::Lane::Id& id, const std::vector<parser::SpeedLimit>& speed_limits = {}) {
  return parser::Lane{id,
                      LineString3d{{0., 1., 0.}, {100., 1., 0.}},    // left
                      LineString3d{{0., -1., 0.}, {100., -1., 0.}},  // right
                      std::nullopt,                                  // left_lane_id
                      std::nullopt,                                  // right_lane_id
                      {},                                            // successors
                      {},                                            // predecessors
                      speed_limits};
}

// -- Test fixture ------------------------------------------------------------

class RoadNetworkLoaderTest : public ::testing::Test {
 protected:
  const BuilderConfiguration builder_config_{maliput::api::RoadGeometryId{"test_rg"}, 5e-2, 1e-3, 1., {0., 0., 0.}};
};

// Tests that a RoadNetwork can be built with no speed limits (backward compatibility).
TEST_F(RoadNetworkLoaderTest, NoSpeedLimits) {
  const auto lane = CreateSimpleLane("lane_0");
  const parser::Segment segment{"segment_0", {lane}};
  const parser::Junction junction{"junction_0", {{"segment_0", segment}}};
  auto parser =
      std::make_unique<MockParser>(std::unordered_map<parser::Junction::Id, parser::Junction>{{"junction_0", junction}},
                                   std::vector<parser::Connection>{});

  RoadNetworkLoader loader(std::move(parser), builder_config_);
  std::unique_ptr<maliput::api::RoadNetwork> road_network;
  ASSERT_NO_THROW(road_network = loader());
  ASSERT_NE(nullptr, road_network);

  // The rulebook should have no range value rules.
  const auto rules = road_network->rulebook()->Rules();
  EXPECT_TRUE(rules.range_value_rules.empty());
}

// Tests that speed limits from the parser are registered as RangeValueRules.
TEST_F(RoadNetworkLoaderTest, WithSpeedLimits) {
  const std::vector<parser::SpeedLimit> speed_limits{
      {0., 100., 0., 30., "Urban road. [m/s]", 0},
  };
  const auto lane = CreateSimpleLane("lane_0", speed_limits);
  const parser::Segment segment{"segment_0", {lane}};
  const parser::Junction junction{"junction_0", {{"segment_0", segment}}};
  auto parser =
      std::make_unique<MockParser>(std::unordered_map<parser::Junction::Id, parser::Junction>{{"junction_0", junction}},
                                   std::vector<parser::Connection>{});

  RoadNetworkLoader loader(std::move(parser), builder_config_);
  std::unique_ptr<maliput::api::RoadNetwork> road_network;
  ASSERT_NO_THROW(road_network = loader());
  ASSERT_NE(nullptr, road_network);

  // Verify the speed limit rule was created.
  const auto rules = road_network->rulebook()->Rules();
  ASSERT_EQ(1u, rules.range_value_rules.size());

  const auto& rule = rules.range_value_rules.begin()->second;
  EXPECT_EQ(maliput::SpeedLimitRuleTypeId(), rule.type_id());
  ASSERT_EQ(1u, rule.states().size());
  EXPECT_DOUBLE_EQ(0., rule.states()[0].min);
  EXPECT_DOUBLE_EQ(30., rule.states()[0].max);
  EXPECT_EQ("Urban road. [m/s]", rule.states()[0].description);

  // Verify the rule zone references the correct lane.
  const auto& zone = rule.zone();
  ASSERT_EQ(1u, zone.ranges().size());
  EXPECT_EQ(maliput::api::LaneId("lane_0"), zone.ranges()[0].lane_id());
  EXPECT_DOUBLE_EQ(0., zone.ranges()[0].s_range().s0());
  EXPECT_DOUBLE_EQ(100., zone.ranges()[0].s_range().s1());

  // Verify that the registry has the speed limit type registered.
  const auto query_result = road_network->rule_registry()->GetPossibleStatesOfRuleType(maliput::SpeedLimitRuleTypeId());
  ASSERT_TRUE(query_result.has_value());
}

// Tests that multiple speed limits on the same lane produce multiple rules.
TEST_F(RoadNetworkLoaderTest, MultipleSpeedLimitsOnSameLane) {
  const std::vector<parser::SpeedLimit> speed_limits{
      {0., 50., 0., 30., "Zone A. [m/s]", 0},
      {50., 100., 0., 15., "Zone B. [m/s]", 0},
  };
  const auto lane = CreateSimpleLane("lane_0", speed_limits);
  const parser::Segment segment{"segment_0", {lane}};
  const parser::Junction junction{"junction_0", {{"segment_0", segment}}};
  auto parser =
      std::make_unique<MockParser>(std::unordered_map<parser::Junction::Id, parser::Junction>{{"junction_0", junction}},
                                   std::vector<parser::Connection>{});

  RoadNetworkLoader loader(std::move(parser), builder_config_);
  std::unique_ptr<maliput::api::RoadNetwork> road_network;
  ASSERT_NO_THROW(road_network = loader());
  ASSERT_NE(nullptr, road_network);

  const auto rules = road_network->rulebook()->Rules();
  EXPECT_EQ(2u, rules.range_value_rules.size());

  // Verify both speed limit ranges are registered.
  const auto query_result = road_network->rule_registry()->GetPossibleStatesOfRuleType(maliput::SpeedLimitRuleTypeId());
  ASSERT_TRUE(query_result.has_value());
  const auto* ranges = std::get_if<std::vector<maliput::api::rules::RangeValueRule::Range>>(&query_result->rule_values);
  ASSERT_NE(nullptr, ranges);
  EXPECT_EQ(2u, ranges->size());
}

// Tests that speed limits across multiple lanes produce rules for each.
TEST_F(RoadNetworkLoaderTest, SpeedLimitsOnMultipleLanes) {
  const auto lane_0 = CreateSimpleLane("lane_0", {{0., 100., 0., 30., "Lane 0 limit. [m/s]", 0}});
  const auto lane_1 = CreateSimpleLane("lane_1", {{0., 100., 0., 60., "Lane 1 limit. [m/s]", 0}});
  // Place lanes in separate junctions to avoid adjacency validation issues.
  const parser::Segment segment_0{"segment_0", {lane_0}};
  const parser::Segment segment_1{"segment_1", {lane_1}};
  const parser::Junction junction_0{"junction_0", {{"segment_0", segment_0}}};
  const parser::Junction junction_1{"junction_1", {{"segment_1", segment_1}}};
  auto parser = std::make_unique<MockParser>(
      std::unordered_map<parser::Junction::Id, parser::Junction>{{"junction_0", junction_0},
                                                                 {"junction_1", junction_1}},
      std::vector<parser::Connection>{});

  RoadNetworkLoader loader(std::move(parser), builder_config_);
  std::unique_ptr<maliput::api::RoadNetwork> road_network;
  ASSERT_NO_THROW(road_network = loader());
  ASSERT_NE(nullptr, road_network);

  const auto rules = road_network->rulebook()->Rules();
  EXPECT_EQ(2u, rules.range_value_rules.size());
}

// Tests that identical speed limits on different lanes only register one range.
TEST_F(RoadNetworkLoaderTest, DeduplicatesRangesInRegistry) {
  const auto lane_0 = CreateSimpleLane("lane_0", {{0., 100., 0., 30., "Same limit. [m/s]", 0}});
  const auto lane_1 = CreateSimpleLane("lane_1", {{0., 100., 0., 30., "Same limit. [m/s]", 0}});
  // Place lanes in separate junctions to avoid adjacency validation issues.
  const parser::Segment segment_0{"segment_0", {lane_0}};
  const parser::Segment segment_1{"segment_1", {lane_1}};
  const parser::Junction junction_0{"junction_0", {{"segment_0", segment_0}}};
  const parser::Junction junction_1{"junction_1", {{"segment_1", segment_1}}};
  auto parser = std::make_unique<MockParser>(
      std::unordered_map<parser::Junction::Id, parser::Junction>{{"junction_0", junction_0},
                                                                 {"junction_1", junction_1}},
      std::vector<parser::Connection>{});

  RoadNetworkLoader loader(std::move(parser), builder_config_);
  std::unique_ptr<maliput::api::RoadNetwork> road_network;
  ASSERT_NO_THROW(road_network = loader());
  ASSERT_NE(nullptr, road_network);

  // Two rules (one per lane), but only one unique range registered.
  const auto rules = road_network->rulebook()->Rules();
  EXPECT_EQ(2u, rules.range_value_rules.size());

  const auto query_result = road_network->rule_registry()->GetPossibleStatesOfRuleType(maliput::SpeedLimitRuleTypeId());
  ASSERT_TRUE(query_result.has_value());
  const auto* ranges = std::get_if<std::vector<maliput::api::rules::RangeValueRule::Range>>(&query_result->rule_values);
  ASSERT_NE(nullptr, ranges);
  EXPECT_EQ(1u, ranges->size());
}

}  // namespace
}  // namespace test
}  // namespace loader
}  // namespace maliput_sparse
