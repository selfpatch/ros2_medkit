// Copyright 2026 Selfpatch
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_MEDKIT_SERIALIZATION__SERIALIZATION_ERROR_HPP_
#define ROS2_MEDKIT_SERIALIZATION__SERIALIZATION_ERROR_HPP_

#include <stdexcept>
#include <string>

namespace ros2_medkit_serialization {

/// Base class for all serialization errors
class SerializationError : public std::runtime_error {
 public:
  explicit SerializationError(const std::string & message) : std::runtime_error(message) {
  }
};

/// Error when type cannot be found or loaded
class TypeNotFoundError : public SerializationError {
 public:
  explicit TypeNotFoundError(const std::string & type_name)
    : SerializationError("Type not found: " + type_name), type_name_(type_name) {
  }

  const std::string & type_name() const noexcept {
    return type_name_;
  }

 private:
  std::string type_name_;
};

/// Error during JSON parsing or conversion
class JsonConversionError : public SerializationError {
 public:
  explicit JsonConversionError(const std::string & message) : SerializationError("JSON conversion error: " + message) {
  }
};

/// Error during YAML parsing or conversion
class YamlConversionError : public SerializationError {
 public:
  explicit YamlConversionError(const std::string & message) : SerializationError("YAML conversion error: " + message) {
  }
};

/// Error when a required field is missing
class MissingFieldError : public SerializationError {
 public:
  explicit MissingFieldError(const std::string & field_name)
    : SerializationError("Missing required field: " + field_name), field_name_(field_name) {
  }

  const std::string & field_name() const noexcept {
    return field_name_;
  }

 private:
  std::string field_name_;
};

/// Error when field type doesn't match expected type
class TypeMismatchError : public SerializationError {
 public:
  TypeMismatchError(const std::string & field_name, const std::string & expected_type, const std::string & actual_type)
    : SerializationError("Type mismatch for field '" + field_name + "': expected " + expected_type + ", got " +
                         actual_type)
    , field_name_(field_name)
    , expected_type_(expected_type)
    , actual_type_(actual_type) {
  }

  const std::string & field_name() const noexcept {
    return field_name_;
  }
  const std::string & expected_type() const noexcept {
    return expected_type_;
  }
  const std::string & actual_type() const noexcept {
    return actual_type_;
  }

 private:
  std::string field_name_;
  std::string expected_type_;
  std::string actual_type_;
};

}  // namespace ros2_medkit_serialization

#endif  // ROS2_MEDKIT_SERIALIZATION__SERIALIZATION_ERROR_HPP_
