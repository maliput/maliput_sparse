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
#pragma once

#include <maliput/api/rules/road_rulebook.h>
#include <maliput/api/rules/rule_registry.h>

#include "maliput_sparse/parser/parser.h"

namespace maliput_sparse {
namespace loader {

/// Loads road rules from a maliput_sparse::parser::Parser implementation into
/// an existing RuleRegistry and RoadRulebook.
///
/// This functor extracts rule information (e.g., speed limits) from parsed
/// lane data and registers the corresponding maliput rule objects. It is
/// designed to be extensible: new rule types can be added as private methods
/// and invoked from operator().
///
/// This class is expected to be used by the maliput backends that rely on
/// maliput_sparse to populate rules from parsed road data, as an alternative
/// to loading rules from YAML configuration files.
class RoadRulebookLoader {
 public:
  /// Constructs a RoadRulebookLoader.
  /// @param parser The parser whose lane data provides rule information.
  ///        Must not be nullptr and must outlive this loader.
  explicit RoadRulebookLoader(const parser::Parser* parser);

  /// Populates the given @p rule_registry and @p rule_book with rules
  /// extracted from parser data.
  /// @param rule_registry The RuleRegistry to register rule types into.
  ///        Must not be nullptr.
  /// @param rule_book The RoadRulebook to add rules to. Must point to a
  ///        maliput::ManualRulebook instance. Must not be nullptr.
  void operator()(maliput::api::rules::RuleRegistry* rule_registry, maliput::api::rules::RoadRulebook* rule_book) const;

 private:
  /// Adds speed limit rules from parser data to the given registry and rulebook.
  /// Does nothing if no lanes have speed limits.
  void AddSpeedLimitRules(maliput::api::rules::RuleRegistry* rule_registry,
                          maliput::api::rules::RoadRulebook* rule_book) const;

  const parser::Parser* parser_;
};

}  // namespace loader
}  // namespace maliput_sparse
