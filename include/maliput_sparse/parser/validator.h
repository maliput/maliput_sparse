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
#pragma once

#include <string>
#include <vector>

#include "maliput_sparse/parser/parser.h"

namespace maliput_sparse {
namespace parser {

/// ValidatorOptions struct that contains the options for the Validator.
struct ValidatorOptions {
  /// Verifies adjacency for lanes in a segment.
  bool lane_adjacency{true};
};

/// ValidatorConfig struct that contains the configuration for the Validator.
struct ValidatorConfig {
  double linear_tolerance{1e-12};
};

/// After parsing a road network, the Validator can be used to check for errors before
/// creating a maliput::api::RoadNetwork via the maliput_sparse::loader::RoadNetworkLoader.
/// The Validator can be configured to check for different types of errors and provide an interface to retrieve
/// the errors found.
/// The errors are stored in a vector of Error structs. The Error struct contains a message, the type of error, and the
/// severity. It's on the user to decide how to handle the errors.
class Validator {
 public:
  /// Error struct that contains the error message, type, and severity.
  struct Error {
    /// The type of error.
    enum class Type {
      kLaneAdjacency,
    };
    /// The severity of the error.
    enum class Severity {
      kWarning,
      kError,
    };

    /// Equality operator for Error.
    bool operator==(const Error& other) const;
    /// Inequality operator for Error.
    bool operator!=(const Error& other) const;

    /// Message describing the error.
    std::string message;
    /// The type of error.
    Type type;
    /// The severity of the error.
    Severity severity;
  };

  /// Constructor for Validator.
  /// During construction, the Validator will perform the validation checks.
  ////
  /// @param parser The maliput_sparse::parser::Parser instance to validate.
  /// @param options The maliput_sparse::parser::ValidatorOptions to use.
  /// @param config The maliput_sparse::parser::ValidatorConfig to use.
  Validator(const Parser* parser, const ValidatorOptions& options, const ValidatorConfig config);

  /// Returns the errors found during validation.
  const std::vector<Error>& GetErrors() const;

 private:
  // Helper functions for reporting errors.
  // @param message The error message.
  // @Param type The type of error.
  // @Param severity The severity of the error.
  void Report(const std::string& message, const Error::Type& type, const Error::Severity& severity);

  // Method to validate lane adjacency.
  // @param parser The maliput_sparse::parser::Parser instance to validate.
  // @param config The maliput_sparse::parser::ValidatorConfig to use.
  void ValidateLaneAdjacency(const Parser* parser, const ValidatorConfig config);

  // Helper function to geometrically check lane adjacency.
  void CheckAdjacency(const Lane& lane, const Lane& adjacent_lane, bool left, double tolerance);

  // Holds the errors found during validation.
  std::vector<Error> errors_;
};

}  // namespace parser
}  // namespace maliput_sparse
