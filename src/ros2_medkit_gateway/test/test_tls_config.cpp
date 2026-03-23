// Copyright 2025 bburda
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

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <random>
#include <stdexcept>
#include <string>

#include "ros2_medkit_gateway/config.hpp"

using namespace ros2_medkit_gateway;

class TlsConfigTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create temporary test files
    cert_file_ = create_temp_file("cert");
    key_file_ = create_temp_file("key");
    ca_file_ = create_temp_file("ca");
  }

  void TearDown() override {
    // Clean up temporary files
    if (!cert_file_.empty()) {
      std::remove(cert_file_.c_str());
    }
    if (!key_file_.empty()) {
      std::remove(key_file_.c_str());
    }
    if (!ca_file_.empty()) {
      std::remove(ca_file_.c_str());
    }
  }

  std::string create_temp_file(const std::string & prefix) {
    // Use std::random_device for proper random number generation
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(0, 999999);
    std::string filename = "/tmp/tls_test_" + prefix + "_" + std::to_string(dis(gen)) + ".pem";
    std::ofstream file(filename);
    file << "test content";
    file.close();
    return filename;
  }

  std::string cert_file_;
  std::string key_file_;
  std::string ca_file_;
};

// Test disabled TLS configuration (default)
TEST_F(TlsConfigTest, disabled_config_is_valid) {
  TlsConfig config;
  EXPECT_FALSE(config.enabled);
  EXPECT_TRUE(config.validate().empty());
}

// Test valid TLS configuration with cert and key
TEST_F(TlsConfigTest, valid_config_with_cert_and_key) {
  auto config = TlsConfigBuilder().with_enabled(true).with_cert_file(cert_file_).with_key_file(key_file_).build();

  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(config.cert_file, cert_file_);
  EXPECT_EQ(config.key_file, key_file_);
  EXPECT_EQ(config.min_version, "1.2");
}

// Test valid TLS configuration with all options
TEST_F(TlsConfigTest, valid_config_with_all_options) {
  auto config = TlsConfigBuilder()
                    .with_enabled(true)
                    .with_cert_file(cert_file_)
                    .with_key_file(key_file_)
                    .with_ca_file(ca_file_)
                    .with_min_version("1.3")
                    .build();

  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(config.cert_file, cert_file_);
  EXPECT_EQ(config.key_file, key_file_);
  EXPECT_EQ(config.ca_file, ca_file_);
  EXPECT_EQ(config.min_version, "1.3");
}

// Test missing certificate file
TEST_F(TlsConfigTest, missing_cert_file_throws) {
  EXPECT_THROW({ TlsConfigBuilder().with_enabled(true).with_key_file(key_file_).build(); }, std::invalid_argument);
}

// Test missing key file
TEST_F(TlsConfigTest, missing_key_file_throws) {
  EXPECT_THROW({ TlsConfigBuilder().with_enabled(true).with_cert_file(cert_file_).build(); }, std::invalid_argument);
}

// Test non-existent certificate file
TEST_F(TlsConfigTest, nonexistent_cert_file_throws) {
  EXPECT_THROW(
      {
        TlsConfigBuilder().with_enabled(true).with_cert_file("/nonexistent/cert.pem").with_key_file(key_file_).build();
      },
      std::invalid_argument);
}

// Test non-existent key file
TEST_F(TlsConfigTest, nonexistent_key_file_throws) {
  EXPECT_THROW(
      {
        TlsConfigBuilder().with_enabled(true).with_cert_file(cert_file_).with_key_file("/nonexistent/key.pem").build();
      },
      std::invalid_argument);
}

// TODO(future): Add mutual TLS tests when implemented
// TEST_F(TlsConfigTest, mutual_tls_without_ca_file_throws) { ... }
// TEST_F(TlsConfigTest, mutual_tls_with_ca_file_succeeds) { ... }

// Test invalid TLS version
TEST_F(TlsConfigTest, invalid_tls_version_throws) {
  EXPECT_THROW(
      {
        TlsConfigBuilder()
            .with_enabled(true)
            .with_cert_file(cert_file_)
            .with_key_file(key_file_)
            .with_min_version("1.0")  // Invalid
            .build();
      },
      std::invalid_argument);
}

// Test valid TLS version 1.2
TEST_F(TlsConfigTest, tls_version_1_2_succeeds) {
  auto config = TlsConfigBuilder()
                    .with_enabled(true)
                    .with_cert_file(cert_file_)
                    .with_key_file(key_file_)
                    .with_min_version("1.2")
                    .build();

  EXPECT_EQ(config.min_version, "1.2");
}

// Test valid TLS version 1.3
TEST_F(TlsConfigTest, tls_version_1_3_succeeds) {
  auto config = TlsConfigBuilder()
                    .with_enabled(true)
                    .with_cert_file(cert_file_)
                    .with_key_file(key_file_)
                    .with_min_version("1.3")
                    .build();

  EXPECT_EQ(config.min_version, "1.3");
}

// Test non-existent CA file
TEST_F(TlsConfigTest, nonexistent_ca_file_throws) {
  EXPECT_THROW(
      {
        TlsConfigBuilder()
            .with_enabled(true)
            .with_cert_file(cert_file_)
            .with_key_file(key_file_)
            .with_ca_file("/nonexistent/ca.pem")
            .build();
      },
      std::invalid_argument);
}

// Test validate() method directly
TEST_F(TlsConfigTest, validate_returns_error_for_missing_cert) {
  TlsConfig config;
  config.enabled = true;
  config.key_file = key_file_;

  std::string error = config.validate();
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(error.find("cert_file") != std::string::npos);
}

// Test validate() method for missing key
TEST_F(TlsConfigTest, validate_returns_error_for_missing_key) {
  TlsConfig config;
  config.enabled = true;
  config.cert_file = cert_file_;

  std::string error = config.validate();
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(error.find("key_file") != std::string::npos);
}

// Test validate() method for invalid version
TEST_F(TlsConfigTest, validate_returns_error_for_invalid_version) {
  TlsConfig config;
  config.enabled = true;
  config.cert_file = cert_file_;
  config.key_file = key_file_;
  config.min_version = "1.1";

  std::string error = config.validate();
  EXPECT_FALSE(error.empty());
  EXPECT_TRUE(error.find("min_version") != std::string::npos);
}

// Test disabled config with invalid paths doesn't throw
TEST_F(TlsConfigTest, disabled_config_ignores_invalid_paths) {
  // When disabled, validation should pass even with invalid paths
  TlsConfig config;
  config.enabled = false;
  config.cert_file = "/nonexistent/cert.pem";
  config.key_file = "/nonexistent/key.pem";

  EXPECT_TRUE(config.validate().empty());
}

// Note: Expired certificate testing
// ================================
// Testing behavior with expired certificates requires:
// 1. Generating a certificate with -days 0 (already expired)
// 2. Starting HttpServerManager with that certificate
// 3. Making HTTPS requests and verifying SSL_ERROR_SSL or similar
//
// This is better suited for integration testing (test_tls.test.py)
// or manual testing, as it requires:
// - Real OpenSSL certificate generation
// - Actually starting an HTTPS server
// - Client-side certificate validation
//
// The current certificate validation (file exists + is readable) happens
// at config time. Certificate validity (expiration, chain, etc.) is
// validated by OpenSSL at connection time.
