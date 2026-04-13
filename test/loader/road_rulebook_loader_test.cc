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
#include "maliput_sparse/loader/road_rulebook_loader.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>
#include <maliput/api/rules/range_value_rule.h>
#include <maliput/api/rules/road_rulebook.h>
#include <maliput/api/rules/rule_registry.h>
#include <maliput/base/manual_rulebook.h>
#include <maliput/base/rule_registry.h>

#include "maliput_sparse/parser/connection.h"
#include "maliput_sparse/parser/junction.h"
#include "maliput_sparse/parser/lane.h"
#include "maliput_sparse/parser/parser.h"
#include "maliput_sparse/parser/segment.h"

namespace maliput_sparse {
namespace loader {
namespace test {
namespace {

/// Minimal mock parser for unit testing rule loading in isolation.
class MockParser : public parser::Parser {
 public:
  MockParser(const std::unordered_map<parser::Junction::Id, parser::Junction>& junctions) : junctions_(junctions) {}

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

/// Creates a lane stub with the given speed limits. Geometry is irrelevant for
/// rule loading tests; only the lane id and speed_limits fields are used.
parser::Lane MakeLane(const parser::Lane::Id& id, const std::vector<parser::SpeedLimit>& speed_limits = {}) {
  return parser::Lane{id,
                      geometry::LineString3d{{0., 1., 0.}, {100., 1., 0.}},
                      geometry::LineString3d{{0., -1., 0.}, {100., -1., 0.}},
                      std::nullopt,
                      std::nullopt,
                      {},
                      {},
                      speed_limits};
}

// -- Test fixture ------------------------------------------------------------

class RoadRulebookLoaderTest : public ::testing::Test {
 protected:
  std::unique_ptr<maliput::api::rules::RuleRegistry> rule_registry_ =
      std::make_unique<maliput::api::rules::RuleRegistry>();
  std::unique_ptr<maliput::ManualRulebook> rule_book_ = std::make_unique<maliput::ManualRulebook>();
};

// Tests that a null parser is rejected.
TEST_F(RoadRulebookLoaderTest, NullParserThrows) { EXPECT_THROW(RoadRulebookLoader(nullptr), std::exception); }

// Tests that a null rule_registry is rejected.
TEST_F(RoadRulebookLoaderTest, NullRuleRegistryThrows) {
  const auto lane = MakeLane("lane_0");
  const parser::Segment segment{"segment_0", {lane}};
  const parser::Junction junction{"junction_0", {{"segment_0", segment}}};
  MockParser parser({{"junction_0", junction}});
  RoadRulebookLoader loader(&parser);
  EXPECT_THROW(loader(nullptr, rule_book_.get()), std::exception);
}

// Tests that a null rule_book is rejected.
TEST_F(RoadRulebookLoaderTest, NullRuleBookThrows) {
  const auto lane = MakeLane("lane_0");
  const parser::Segment segment{"segment_0", {lane}};
  const parser::Junction junction{"junction_0", {{"segment_0", segment}}};
  MockParser parser({{"junction_0", junction}});
  RoadRulebookLoader loader(&parser);
  EXPECT_THROW(loader(rule_registry_.get(), nullptr), std::exception);
}

// Tests that loader does nothing when no speed limits are present.
TEST_F(RoadRulebookLoaderTest, NoSpeedLimits) {
  const auto lane = MakeLane("lane_0");
  const parser::Segment segment{"segment_0", {lane}};
  const parser::Junction junction{"junction_0", {{"segment_0", segment}}};
  MockParser parser({{"junction_0", junction}});

  RoadRulebookLoader loader(&parser);
  ASSERT_NO_THROW(loader(rule_registry_.get(), rule_book_.get()));

  const auto rules = rule_book_->Rules();
  EXPECT_TRUE(rules.range_value_rules.empty());
}

// Tests that a single speed limit produces one RangeValueRule.
TEST_F(RoadRulebookLoaderTest, SingleSpeedLimit) {
  const auto lane = MakeLane("lane_0", {{0., 100., 0., 30., "Urban road. [m/s]", 0}});
  const parser::Segment segment{"segment_0", {lane}};
  const parser::Junction junction{"junction_0", {{"segment_0", segment}}};
  MockParser parser({{"junction_0", junction}});

  RoadRulebookLoader loader(&parser);
  loader(rule_registry_.get(), rule_book_.get());

  const auto rules = rule_book_->Rules();
  ASSERT_EQ(1u, rules.range_value_rules.size());

  const auto& rule = rules.range_value_rules.begin()->second;
  EXPECT_EQ(maliput::SpeedLimitRuleTypeId(), rule.type_id());
  ASSERT_EQ(1u, rule.states().size());
  EXPECT_DOUBLE_EQ(0., rule.states()[0].min);
  EXPECT_DOUBLE_EQ(30., rule.states()[0].max);
  EXPECT_EQ("Urban road. [m/s]", rule.states()[0].description);

  // Verify the zone references the correct lane.
  const auto& zone = rule.zone();
  ASSERT_EQ(1u, zone.ranges().size());
  EXPECT_EQ(maliput::api::LaneId("lane_0"), zone.ranges()[0].lane_id());
  EXPECT_DOUBLE_EQ(0., zone.ranges()[0].s_range().s0());
  EXPECT_DOUBLE_EQ(100., zone.ranges()[0].s_range().s1());
}

// Tests that multiple speed limits on one lane produce multiple rules.
TEST_F(RoadRulebookLoaderTest, MultipleSpeedLimitsOnSameLane) {
  const auto lane = MakeLane("lane_0", {
                                           {0., 50., 0., 30., "Zone A. [m/s]", 0},
                                           {50., 100., 0., 15., "Zone B. [m/s]", 0},
                                       });
  const parser::Segment segment{"segment_0", {lane}};
  const parser::Junction junction{"junction_0", {{"segment_0", segment}}};
  MockParser parser({{"junction_0", junction}});

  RoadRulebookLoader loader(&parser);
  loader(rule_registry_.get(), rule_book_.get());

  const auto rules = rule_book_->Rules();
  EXPECT_EQ(2u, rules.range_value_rules.size());

  // Both ranges should be registered.
  const auto query_result = rule_registry_->GetPossibleStatesOfRuleType(maliput::SpeedLimitRuleTypeId());
  ASSERT_TRUE(query_result.has_value());
  const auto* ranges = std::get_if<std::vector<maliput::api::rules::RangeValueRule::Range>>(&query_result->rule_values);
  ASSERT_NE(nullptr, ranges);
  EXPECT_EQ(2u, ranges->size());
}

// Tests that identical speed limits across lanes are deduplicated in the registry.
TEST_F(RoadRulebookLoaderTest, DeduplicatesIdenticalRanges) {
  const auto lane_0 = MakeLane("lane_0", {{0., 100., 0., 30., "Same limit. [m/s]", 0}});
  const auto lane_1 = MakeLane("lane_1", {{0., 100., 0., 30., "Same limit. [m/s]", 0}});
  const parser::Segment segment_0{"segment_0", {lane_0}};
  const parser::Segment segment_1{"segment_1", {lane_1}};
  const parser::Junction junction_0{"junction_0", {{"segment_0", segment_0}}};
  const parser::Junction junction_1{"junction_1", {{"segment_1", segment_1}}};
  MockParser parser({{"junction_0", junction_0}, {"junction_1", junction_1}});

  RoadRulebookLoader loader(&parser);
  loader(rule_registry_.get(), rule_book_.get());

  // Two rules (one per lane).
  const auto rules = rule_book_->Rules();
  EXPECT_EQ(2u, rules.range_value_rules.size());

  // But only one unique range registered.
  const auto query_result = rule_registry_->GetPossibleStatesOfRuleType(maliput::SpeedLimitRuleTypeId());
  ASSERT_TRUE(query_result.has_value());
  const auto* ranges = std::get_if<std::vector<maliput::api::rules::RangeValueRule::Range>>(&query_result->rule_values);
  ASSERT_NE(nullptr, ranges);
  EXPECT_EQ(1u, ranges->size());
}

// Tests that the registry reflects distinct ranges from different lanes.
TEST_F(RoadRulebookLoaderTest, DistinctRangesAcrossLanes) {
  const auto lane_0 = MakeLane("lane_0", {{0., 100., 0., 30., "Lane 0 limit. [m/s]", 0}});
  const auto lane_1 = MakeLane("lane_1", {{0., 100., 0., 60., "Lane 1 limit. [m/s]", 0}});
  const parser::Segment segment_0{"segment_0", {lane_0}};
  const parser::Segment segment_1{"segment_1", {lane_1}};
  const parser::Junction junction_0{"junction_0", {{"segment_0", segment_0}}};
  const parser::Junction junction_1{"junction_1", {{"segment_1", segment_1}}};
  MockParser parser({{"junction_0", junction_0}, {"junction_1", junction_1}});

  RoadRulebookLoader loader(&parser);
  loader(rule_registry_.get(), rule_book_.get());

  const auto rules = rule_book_->Rules();
  EXPECT_EQ(2u, rules.range_value_rules.size());

  // Two distinct ranges registered.
  const auto query_result = rule_registry_->GetPossibleStatesOfRuleType(maliput::SpeedLimitRuleTypeId());
  ASSERT_TRUE(query_result.has_value());
  const auto* ranges = std::get_if<std::vector<maliput::api::rules::RangeValueRule::Range>>(&query_result->rule_values);
  ASSERT_NE(nullptr, ranges);
  EXPECT_EQ(2u, ranges->size());
}

}  // namespace
}  // namespace test
}  // namespace loader
}  // namespace maliput_sparse
