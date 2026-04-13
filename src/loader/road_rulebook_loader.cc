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

#include <set>
#include <string>
#include <vector>

#include <maliput/api/lane_data.h>
#include <maliput/api/regions.h>
#include <maliput/api/rules/range_value_rule.h>
#include <maliput/api/rules/rule.h>
#include <maliput/base/manual_rulebook.h>
#include <maliput/base/rule_registry.h>
#include <maliput/common/logger.h>
#include <maliput/common/maliput_throw.h>

namespace maliput_sparse {
namespace loader {

RoadRulebookLoader::RoadRulebookLoader(const parser::Parser* parser) : parser_(parser) {
  MALIPUT_THROW_UNLESS(parser_ != nullptr);
}

void RoadRulebookLoader::operator()(maliput::api::rules::RuleRegistry* rule_registry,
                                    maliput::api::rules::RoadRulebook* rule_book) const {
  MALIPUT_THROW_UNLESS(rule_registry != nullptr);
  MALIPUT_THROW_UNLESS(rule_book != nullptr);
  AddSpeedLimitRules(rule_registry, rule_book);
}

void RoadRulebookLoader::AddSpeedLimitRules(maliput::api::rules::RuleRegistry* rule_registry,
                                            maliput::api::rules::RoadRulebook* rule_book) const {
  // Collect all unique speed limit ranges from the parser's lane data.
  const auto& junctions = parser_->GetJunctions();
  std::set<maliput::api::rules::RangeValueRule::Range> speed_limit_ranges;
  for (const auto& [junction_id, junction] : junctions) {
    for (const auto& [segment_id, segment] : junction.segments) {
      for (const auto& lane : segment.lanes) {
        for (const auto& speed_limit : lane.speed_limits) {
          speed_limit_ranges.insert(maliput::api::rules::RangeValueRule::Range{speed_limit.severity,
                                                                               {} /* related_rules */,
                                                                               {} /* related_unique_ids */,
                                                                               speed_limit.description,
                                                                               speed_limit.min,
                                                                               speed_limit.max});
        }
      }
    }
  }

  if (speed_limit_ranges.empty()) {
    maliput::log()->trace("No speed limits provided by parser.");
    return;
  }

  // Register the speed limit rule type with all discovered ranges.
  maliput::log()->trace("Registering speed limit rule type with ", speed_limit_ranges.size(), " range(s).");
  rule_registry->RegisterRangeValueRule(
      maliput::SpeedLimitRuleTypeId(),
      std::vector<maliput::api::rules::RangeValueRule::Range>(speed_limit_ranges.begin(), speed_limit_ranges.end()));

  // Create a rule for each lane's speed limit zone.
  auto* manual_rulebook = dynamic_cast<maliput::ManualRulebook*>(rule_book);
  MALIPUT_THROW_UNLESS(manual_rulebook != nullptr);

  int rule_index = 0;
  for (const auto& [junction_id, junction] : junctions) {
    for (const auto& [segment_id, segment] : junction.segments) {
      for (const auto& lane : segment.lanes) {
        for (const auto& speed_limit : lane.speed_limits) {
          const maliput::api::rules::Rule::Id rule_id{maliput::SpeedLimitRuleTypeId().string() + "/" + lane.id + "_" +
                                                      std::to_string(rule_index++)};
          const maliput::api::LaneSRoute zone({maliput::api::LaneSRange(
              maliput::api::LaneId(lane.id), maliput::api::SRange(speed_limit.s_start, speed_limit.s_end))});
          const maliput::api::rules::RangeValueRule::Range range{
              speed_limit.severity,    {} /* related_rules */, {} /* related_unique_ids */,
              speed_limit.description, speed_limit.min,        speed_limit.max};

          manual_rulebook->AddRule(
              rule_registry->BuildRangeValueRule(rule_id, maliput::SpeedLimitRuleTypeId(), zone, {range}));
          maliput::log()->trace("Added speed limit rule: ", rule_id.string(), " for lane ", lane.id);
        }
      }
    }
  }
}

}  // namespace loader
}  // namespace maliput_sparse
