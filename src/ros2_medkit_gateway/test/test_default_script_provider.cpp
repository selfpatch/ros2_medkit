// Copyright 2026 Bartlomiej Burda
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

#include "ros2_medkit_gateway/default_script_provider.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

using namespace ros2_medkit_gateway;
using json = nlohmann::json;

namespace {

// Path to the demo scripts in the source tree
std::string get_demo_scripts_dir() {
  // __FILE__ is the path to this test source file at compile time.
  // From test/test_default_script_provider.cpp, scripts are at
  // test/demo_nodes/scripts/
  std::filesystem::path this_file(__FILE__);
  return (this_file.parent_path() / "demo_nodes" / "scripts").string();
}

/// Polls execution status until it leaves "running" or timeout is reached.
/// Returns the final ExecutionInfo.
std::optional<ExecutionInfo> wait_for_completion(DefaultScriptProvider & provider, const std::string & entity_id,
                                                 const std::string & script_id, const std::string & execution_id,
                                                 int timeout_ms = 10000) {
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    auto result = provider.get_execution(entity_id, script_id, execution_id);
    if (!result.has_value()) {
      return std::nullopt;
    }
    if (result->status != "running") {
      return *result;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  // Return last status even if still running
  auto result = provider.get_execution(entity_id, script_id, execution_id);
  if (result.has_value()) {
    return *result;
  }
  return std::nullopt;
}

class DefaultScriptProviderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    test_dir_ = std::filesystem::temp_directory_path() /
                ("test_scripts_" + std::to_string(getpid()) + "_" + std::to_string(test_counter_++));
    std::filesystem::create_directories(test_dir_);
    scripts_dir_ = get_demo_scripts_dir();
  }

  void TearDown() override {
    std::error_code ec;
    std::filesystem::remove_all(test_dir_, ec);
  }

  ScriptsConfig make_config(std::vector<ScriptEntryConfig> entries = {}) {
    ScriptsConfig config;
    config.scripts_dir = test_dir_.string();
    config.max_file_size_mb = 10;
    config.max_concurrent_executions = 5;
    config.default_timeout_sec = 300;
    config.entries = std::move(entries);
    return config;
  }

  ScriptEntryConfig make_manifest_entry(const std::string & id, const std::string & name,
                                        const std::string & description = "",
                                        std::vector<std::string> entity_filter = {}) {
    ScriptEntryConfig entry;
    entry.id = id;
    entry.name = name;
    entry.description = description;
    entry.path = "/usr/local/bin/test_script.py";
    entry.format = "python";
    entry.timeout_sec = 300;
    entry.entity_filter = std::move(entity_filter);
    return entry;
  }

  /// Creates a manifest entry pointing to a real demo script.
  ScriptEntryConfig make_real_script_entry(const std::string & id, const std::string & script_name,
                                           const std::string & format, int timeout_sec = 300) {
    ScriptEntryConfig entry;
    entry.id = id;
    entry.name = id;
    entry.description = "Test script";
    entry.path = scripts_dir_ + "/" + script_name;
    entry.format = format;
    entry.timeout_sec = timeout_sec;
    return entry;
  }

  std::filesystem::path test_dir_;
  std::string scripts_dir_;
  static int test_counter_;
};

int DefaultScriptProviderTest::test_counter_ = 0;

// --- CRUD tests (from Task 6, unchanged) ---

// @verifies REQ_INTEROP_041
TEST_F(DefaultScriptProviderTest, ManifestScriptsLoaded) {
  auto entry1 = make_manifest_entry("diag_check", "Diagnostic Check", "Runs diagnostics");
  auto entry2 = make_manifest_entry("health_scan", "Health Scan", "Scans health");
  auto config = make_config({entry1, entry2});

  DefaultScriptProvider provider(config);

  auto result = provider.list_scripts("any_entity");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->size(), 2u);

  // Verify both scripts are present (order may vary since std::map is sorted by key)
  bool found_diag = false;
  bool found_health = false;
  for (const auto & info : *result) {
    if (info.id == "diag_check") {
      found_diag = true;
      EXPECT_EQ(info.name, "Diagnostic Check");
      EXPECT_EQ(info.description, "Runs diagnostics");
      EXPECT_TRUE(info.managed);
    }
    if (info.id == "health_scan") {
      found_health = true;
      EXPECT_EQ(info.name, "Health Scan");
      EXPECT_TRUE(info.managed);
    }
  }
  EXPECT_TRUE(found_diag);
  EXPECT_TRUE(found_health);
}

// @verifies REQ_INTEROP_040
TEST_F(DefaultScriptProviderTest, UploadToFilesystem) {
  auto config = make_config();
  DefaultScriptProvider provider(config);

  std::string content = "#!/usr/bin/env python3\nprint('hello')";
  json metadata = {{"name", "My Script"}, {"description", "A test script"}};

  auto result = provider.upload_script("comp1", "test.py", content, metadata);
  ASSERT_TRUE(result.has_value()) << result.error().message;
  EXPECT_FALSE(result->id.empty());
  EXPECT_EQ(result->name, "My Script");

  // Verify file was written to filesystem
  auto script_dir = test_dir_ / "comp1" / result->id;
  EXPECT_TRUE(std::filesystem::exists(script_dir / "metadata.json"));
  EXPECT_TRUE(std::filesystem::exists(script_dir / "script.py"));

  // Verify script content
  std::ifstream script_file(script_dir / "script.py");
  std::string file_content((std::istreambuf_iterator<char>(script_file)), std::istreambuf_iterator<char>());
  EXPECT_EQ(file_content, content);

  // Verify metadata
  std::ifstream meta_file(script_dir / "metadata.json");
  auto meta = json::parse(meta_file);
  EXPECT_EQ(meta["name"], "My Script");
  EXPECT_EQ(meta["description"], "A test script");
  EXPECT_EQ(meta["format"], "python");
  EXPECT_EQ(meta["filename"], "test.py");
  EXPECT_TRUE(meta.contains("created_at"));
}

// @verifies REQ_INTEROP_041
TEST_F(DefaultScriptProviderTest, ListMergesManifestAndUploaded) {
  auto entry = make_manifest_entry("manifest_script", "Manifest Script");
  auto config = make_config({entry});

  DefaultScriptProvider provider(config);

  // Upload a script
  std::string content = "#!/bin/bash\necho test";
  auto upload_result = provider.upload_script("comp1", "uploaded.sh", content, std::nullopt);
  ASSERT_TRUE(upload_result.has_value());

  // List should include both manifest and uploaded
  auto list_result = provider.list_scripts("comp1");
  ASSERT_TRUE(list_result.has_value());
  EXPECT_EQ(list_result->size(), 2u);

  bool found_manifest = false;
  bool found_uploaded = false;
  for (const auto & info : *list_result) {
    if (info.id == "manifest_script") {
      found_manifest = true;
      EXPECT_TRUE(info.managed);
    }
    if (info.id == upload_result->id) {
      found_uploaded = true;
      EXPECT_FALSE(info.managed);
    }
  }
  EXPECT_TRUE(found_manifest);
  EXPECT_TRUE(found_uploaded);
}

// @verifies REQ_INTEROP_042
TEST_F(DefaultScriptProviderTest, GetManifestScript) {
  auto entry = make_manifest_entry("diag_check", "Diagnostic Check", "Runs diagnostics");
  auto config = make_config({entry});

  DefaultScriptProvider provider(config);

  auto result = provider.get_script("any_entity", "diag_check");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->id, "diag_check");
  EXPECT_EQ(result->name, "Diagnostic Check");
  EXPECT_EQ(result->description, "Runs diagnostics");
  EXPECT_TRUE(result->managed);
}

// @verifies REQ_INTEROP_042
TEST_F(DefaultScriptProviderTest, GetUploadedScript) {
  auto config = make_config();
  DefaultScriptProvider provider(config);

  std::string content = "#!/usr/bin/env python3\nprint('test')";
  json metadata = {{"name", "Uploaded Script"}, {"description", "An uploaded script"}};

  auto upload_result = provider.upload_script("comp1", "test.py", content, metadata);
  ASSERT_TRUE(upload_result.has_value());

  auto result = provider.get_script("comp1", upload_result->id);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->id, upload_result->id);
  EXPECT_EQ(result->name, "Uploaded Script");
  EXPECT_EQ(result->description, "An uploaded script");
  EXPECT_FALSE(result->managed);
}

// @verifies REQ_INTEROP_043
TEST_F(DefaultScriptProviderTest, DeleteUploadedScript) {
  auto config = make_config();
  DefaultScriptProvider provider(config);

  // Upload first
  std::string content = "#!/bin/bash\necho hello";
  auto upload_result = provider.upload_script("comp1", "test.sh", content, std::nullopt);
  ASSERT_TRUE(upload_result.has_value());

  // Verify it exists
  auto script_dir = test_dir_ / "comp1" / upload_result->id;
  EXPECT_TRUE(std::filesystem::exists(script_dir));

  // Delete
  auto delete_result = provider.delete_script("comp1", upload_result->id);
  EXPECT_TRUE(delete_result.has_value());

  // Verify removed from filesystem
  EXPECT_FALSE(std::filesystem::exists(script_dir));

  // Verify get returns NotFound
  auto get_result = provider.get_script("comp1", upload_result->id);
  ASSERT_FALSE(get_result.has_value());
  EXPECT_EQ(get_result.error().code, ScriptBackendError::NotFound);
}

// @verifies REQ_INTEROP_043
TEST_F(DefaultScriptProviderTest, DeleteManifestScriptFails) {
  auto entry = make_manifest_entry("managed_script", "Managed Script");
  auto config = make_config({entry});

  DefaultScriptProvider provider(config);

  auto result = provider.delete_script("comp1", "managed_script");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ScriptBackendError::ManagedScript);
}

// @verifies REQ_INTEROP_042
TEST_F(DefaultScriptProviderTest, GetNonexistentScript) {
  auto config = make_config();
  DefaultScriptProvider provider(config);

  auto result = provider.get_script("comp1", "does_not_exist");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ScriptBackendError::NotFound);
}

TEST_F(DefaultScriptProviderTest, DetectsFormatFromExtension) {
  auto config = make_config();
  DefaultScriptProvider provider(config);

  // Upload .py -> python
  auto py_result = provider.upload_script("comp1", "check.py", "print('x')", std::nullopt);
  ASSERT_TRUE(py_result.has_value());
  auto py_meta_path = test_dir_ / "comp1" / py_result->id / "metadata.json";
  std::ifstream py_meta(py_meta_path);
  auto py_json = json::parse(py_meta);
  EXPECT_EQ(py_json["format"], "python");
  EXPECT_TRUE(std::filesystem::exists(test_dir_ / "comp1" / py_result->id / "script.py"));

  // Upload .bash -> bash
  auto bash_result = provider.upload_script("comp1", "run.bash", "echo hi", std::nullopt);
  ASSERT_TRUE(bash_result.has_value());
  auto bash_meta_path = test_dir_ / "comp1" / bash_result->id / "metadata.json";
  std::ifstream bash_meta(bash_meta_path);
  auto bash_json = json::parse(bash_meta);
  EXPECT_EQ(bash_json["format"], "bash");
  EXPECT_TRUE(std::filesystem::exists(test_dir_ / "comp1" / bash_result->id / "script.bash"));

  // Upload .sh -> sh
  auto sh_result = provider.upload_script("comp1", "run.sh", "echo hi", std::nullopt);
  ASSERT_TRUE(sh_result.has_value());
  auto sh_meta_path = test_dir_ / "comp1" / sh_result->id / "metadata.json";
  std::ifstream sh_meta(sh_meta_path);
  auto sh_json = json::parse(sh_meta);
  EXPECT_EQ(sh_json["format"], "sh");
  EXPECT_TRUE(std::filesystem::exists(test_dir_ / "comp1" / sh_result->id / "script.sh"));
}

// @verifies REQ_INTEROP_040
TEST_F(DefaultScriptProviderTest, UploadWithParametersSchema) {
  auto config = make_config();
  DefaultScriptProvider provider(config);

  json schema = {{"type", "object"}, {"properties", {{"timeout", {{"type", "integer"}}}}}};
  json metadata = {{"name", "With Schema"}, {"parameters_schema", schema}};

  auto result = provider.upload_script("comp1", "test.py", "pass", metadata);
  ASSERT_TRUE(result.has_value());

  auto get_result = provider.get_script("comp1", result->id);
  ASSERT_TRUE(get_result.has_value());
  ASSERT_TRUE(get_result->parameters_schema.has_value());
  EXPECT_EQ(get_result->parameters_schema.value()["type"], "object");
}

// @verifies REQ_INTEROP_040
TEST_F(DefaultScriptProviderTest, UploadWithoutMetadata) {
  auto config = make_config();
  DefaultScriptProvider provider(config);

  auto result = provider.upload_script("comp1", "diagnostic.sh", "echo check", std::nullopt);
  ASSERT_TRUE(result.has_value());
  // Name defaults to filename when no metadata provided
  EXPECT_EQ(result->name, "diagnostic.sh");
}

TEST_F(DefaultScriptProviderTest, EntityFilterExactMatch) {
  auto entry = make_manifest_entry("filtered_script", "Filtered Script", "Only for comp1", {"comp1"});
  auto config = make_config({entry});

  DefaultScriptProvider provider(config);

  // Should match comp1
  auto result1 = provider.list_scripts("comp1");
  ASSERT_TRUE(result1.has_value());
  EXPECT_EQ(result1->size(), 1u);

  // Should not match comp2
  auto result2 = provider.list_scripts("comp2");
  ASSERT_TRUE(result2.has_value());
  EXPECT_EQ(result2->size(), 0u);
}

TEST_F(DefaultScriptProviderTest, EntityFilterWildcard) {
  auto entry = make_manifest_entry("wildcard_script", "Wildcard Script", "For all entities", {"*"});
  auto config = make_config({entry});

  DefaultScriptProvider provider(config);

  auto result = provider.list_scripts("any_entity_id");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->size(), 1u);
}

TEST_F(DefaultScriptProviderTest, EntityFilterEmpty) {
  // Empty filter matches everything
  auto entry = make_manifest_entry("unfiltered_script", "Unfiltered Script", "Matches all");
  auto config = make_config({entry});

  DefaultScriptProvider provider(config);

  auto result = provider.list_scripts("anything");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->size(), 1u);
}

// @verifies REQ_INTEROP_043
TEST_F(DefaultScriptProviderTest, DeleteNonexistentScriptFails) {
  auto config = make_config();
  DefaultScriptProvider provider(config);

  auto result = provider.delete_script("comp1", "nonexistent");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ScriptBackendError::NotFound);
}

// @verifies REQ_INTEROP_042
TEST_F(DefaultScriptProviderTest, ManifestScriptWithParametersSchema) {
  auto entry = make_manifest_entry("schema_script", "Schema Script");
  entry.parameters_schema = json{{"type", "object"}, {"properties", {{"mode", {{"type", "string"}}}}}};
  auto config = make_config({entry});

  DefaultScriptProvider provider(config);

  auto result = provider.get_script("any_entity", "schema_script");
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result->parameters_schema.has_value());
  EXPECT_EQ(result->parameters_schema.value()["type"], "object");
}

// @verifies REQ_INTEROP_041
TEST_F(DefaultScriptProviderTest, UploadedScriptsIsolatedByEntity) {
  auto config = make_config();
  DefaultScriptProvider provider(config);

  // Upload to comp1
  auto r1 = provider.upload_script("comp1", "script.py", "pass", std::nullopt);
  ASSERT_TRUE(r1.has_value());

  // Upload to comp2
  auto r2 = provider.upload_script("comp2", "script.py", "pass", std::nullopt);
  ASSERT_TRUE(r2.has_value());

  // Each entity should only see its own
  auto list1 = provider.list_scripts("comp1");
  ASSERT_TRUE(list1.has_value());
  EXPECT_EQ(list1->size(), 1u);

  auto list2 = provider.list_scripts("comp2");
  ASSERT_TRUE(list2.has_value());
  EXPECT_EQ(list2->size(), 1u);

  // comp3 has nothing
  auto list3 = provider.list_scripts("comp3");
  ASSERT_TRUE(list3.has_value());
  EXPECT_EQ(list3->size(), 0u);
}

// --- Execution tests (Task 7) ---

// @verifies REQ_INTEROP_044
TEST_F(DefaultScriptProviderTest, SubprocessExecutionPython) {
  auto entry = make_real_script_entry("echo_py", "echo_params.py", "python");
  auto config = make_config({entry});
  DefaultScriptProvider provider(config);

  ExecutionRequest req{"now", std::nullopt, std::nullopt};
  auto start = provider.start_execution("comp1", "echo_py", req);
  ASSERT_TRUE(start.has_value()) << start.error().message;
  EXPECT_EQ(start->status, "running");
  EXPECT_FALSE(start->id.empty());

  auto final_state = wait_for_completion(provider, "comp1", "echo_py", start->id);
  ASSERT_TRUE(final_state.has_value());
  EXPECT_EQ(final_state->status, "completed")
      << "error: " << (final_state->error.has_value() ? final_state->error->dump() : "none");
  ASSERT_TRUE(final_state->output_parameters.has_value());

  // echo_params.py outputs {"args": [...], "env": {...}}
  auto output = final_state->output_parameters.value();
  EXPECT_TRUE(output.contains("args"));
  EXPECT_TRUE(output["args"].is_array());
}

// @verifies REQ_INTEROP_044
TEST_F(DefaultScriptProviderTest, SubprocessExecutionBash) {
  auto entry = make_real_script_entry("echo_bash", "echo_params.bash", "bash");
  auto config = make_config({entry});
  DefaultScriptProvider provider(config);

  ExecutionRequest req{"now", std::nullopt, std::nullopt};
  auto start = provider.start_execution("comp1", "echo_bash", req);
  ASSERT_TRUE(start.has_value()) << start.error().message;

  auto final_state = wait_for_completion(provider, "comp1", "echo_bash", start->id);
  ASSERT_TRUE(final_state.has_value());
  EXPECT_EQ(final_state->status, "completed")
      << "error: " << (final_state->error.has_value() ? final_state->error->dump() : "none");
  ASSERT_TRUE(final_state->output_parameters.has_value());
}

// @verifies REQ_INTEROP_044
TEST_F(DefaultScriptProviderTest, SubprocessExecutionSh) {
  auto entry = make_real_script_entry("echo_sh", "echo_params.sh", "sh");
  auto config = make_config({entry});
  DefaultScriptProvider provider(config);

  ExecutionRequest req{"now", std::nullopt, std::nullopt};
  auto start = provider.start_execution("comp1", "echo_sh", req);
  ASSERT_TRUE(start.has_value()) << start.error().message;

  auto final_state = wait_for_completion(provider, "comp1", "echo_sh", start->id);
  ASSERT_TRUE(final_state.has_value());
  EXPECT_EQ(final_state->status, "completed")
      << "error: " << (final_state->error.has_value() ? final_state->error->dump() : "none");
  ASSERT_TRUE(final_state->output_parameters.has_value());
}

// @verifies REQ_INTEROP_044
TEST_F(DefaultScriptProviderTest, SubprocessFailure) {
  // Create a script that exits with code 1
  auto fail_script = test_dir_ / "fail.sh";
  {
    std::ofstream f(fail_script);
    f << "#!/bin/sh\necho 'something went wrong' >&2\nexit 1\n";
  }
  std::filesystem::permissions(fail_script, std::filesystem::perms::owner_exec, std::filesystem::perm_options::add);

  ScriptEntryConfig entry;
  entry.id = "fail_script";
  entry.name = "Fail Script";
  entry.path = fail_script.string();
  entry.format = "sh";
  entry.timeout_sec = 30;
  auto config = make_config({entry});
  DefaultScriptProvider provider(config);

  ExecutionRequest req{"now", std::nullopt, std::nullopt};
  auto start = provider.start_execution("comp1", "fail_script", req);
  ASSERT_TRUE(start.has_value()) << start.error().message;

  auto final_state = wait_for_completion(provider, "comp1", "fail_script", start->id);
  ASSERT_TRUE(final_state.has_value());
  EXPECT_EQ(final_state->status, "failed");
  ASSERT_TRUE(final_state->error.has_value());
  EXPECT_TRUE(final_state->error->contains("message"));
  EXPECT_TRUE(final_state->error->contains("exit_code"));
  EXPECT_EQ(final_state->error->at("exit_code").get<int>(), 1);
}

// @verifies REQ_INTEROP_044
TEST_F(DefaultScriptProviderTest, SubprocessTimeout) {
  // Create a script that sleeps for a long time
  auto slow_script = test_dir_ / "slow.sh";
  {
    std::ofstream f(slow_script);
    f << "#!/bin/sh\nsleep 60\n";
  }
  std::filesystem::permissions(slow_script, std::filesystem::perms::owner_exec, std::filesystem::perm_options::add);

  ScriptEntryConfig entry;
  entry.id = "slow_script";
  entry.name = "Slow Script";
  entry.path = slow_script.string();
  entry.format = "sh";
  entry.timeout_sec = 1;  // 1 second timeout
  auto config = make_config({entry});
  DefaultScriptProvider provider(config);

  ExecutionRequest req{"now", std::nullopt, std::nullopt};
  auto start = provider.start_execution("comp1", "slow_script", req);
  ASSERT_TRUE(start.has_value()) << start.error().message;

  // Wait for the timeout to trigger (timeout is 1s + 2s grace period + some margin)
  auto final_state = wait_for_completion(provider, "comp1", "slow_script", start->id, 15000);
  ASSERT_TRUE(final_state.has_value());
  EXPECT_EQ(final_state->status, "terminated");
  ASSERT_TRUE(final_state->error.has_value());
  EXPECT_NE(final_state->error->at("message").get<std::string>().find("timed out"), std::string::npos);
}

// @verifies REQ_INTEROP_044
TEST_F(DefaultScriptProviderTest, PositionalArgs) {
  auto entry = make_real_script_entry("echo_py", "echo_params.py", "python");
  entry.args = json::array({
      {{"name", "input"}, {"type", "positional"}},
      {{"name", "output"}, {"type", "positional"}},
  });
  auto config = make_config({entry});
  DefaultScriptProvider provider(config);

  json params = {{"input", "/data/sensor.log"}, {"output", "/tmp/result.json"}};
  ExecutionRequest req{"now", params, std::nullopt};
  auto start = provider.start_execution("comp1", "echo_py", req);
  ASSERT_TRUE(start.has_value()) << start.error().message;

  auto final_state = wait_for_completion(provider, "comp1", "echo_py", start->id);
  ASSERT_TRUE(final_state.has_value());
  EXPECT_EQ(final_state->status, "completed")
      << "error: " << (final_state->error.has_value() ? final_state->error->dump() : "none");
  ASSERT_TRUE(final_state->output_parameters.has_value());

  auto output = final_state->output_parameters.value();
  ASSERT_TRUE(output.contains("args"));
  auto args = output["args"];
  ASSERT_GE(args.size(), 2u);
  EXPECT_EQ(args[0].get<std::string>(), "/data/sensor.log");
  EXPECT_EQ(args[1].get<std::string>(), "/tmp/result.json");
}

// @verifies REQ_INTEROP_044
TEST_F(DefaultScriptProviderTest, NamedArgs) {
  auto entry = make_real_script_entry("echo_py", "echo_params.py", "python");
  entry.args = json::array({
      {{"name", "threshold"}, {"type", "named"}, {"flag", "--threshold"}},
  });
  auto config = make_config({entry});
  DefaultScriptProvider provider(config);

  json params = {{"threshold", "0.1"}};
  ExecutionRequest req{"now", params, std::nullopt};
  auto start = provider.start_execution("comp1", "echo_py", req);
  ASSERT_TRUE(start.has_value()) << start.error().message;

  auto final_state = wait_for_completion(provider, "comp1", "echo_py", start->id);
  ASSERT_TRUE(final_state.has_value());
  EXPECT_EQ(final_state->status, "completed")
      << "error: " << (final_state->error.has_value() ? final_state->error->dump() : "none");
  ASSERT_TRUE(final_state->output_parameters.has_value());

  auto output = final_state->output_parameters.value();
  auto args = output["args"];
  // Should contain "--threshold" followed by "0.1"
  bool found = false;
  for (size_t i = 0; i + 1 < args.size(); ++i) {
    if (args[i].get<std::string>() == "--threshold" && args[i + 1].get<std::string>() == "0.1") {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected --threshold 0.1 in args: " << args.dump();
}

// @verifies REQ_INTEROP_044
TEST_F(DefaultScriptProviderTest, FlagArgs) {
  auto entry = make_real_script_entry("echo_py", "echo_params.py", "python");
  entry.args = json::array({
      {{"name", "verbose"}, {"type", "flag"}, {"flag", "-v"}},
  });
  auto config = make_config({entry});
  DefaultScriptProvider provider(config);

  json params = {{"verbose", true}};
  ExecutionRequest req{"now", params, std::nullopt};
  auto start = provider.start_execution("comp1", "echo_py", req);
  ASSERT_TRUE(start.has_value()) << start.error().message;

  auto final_state = wait_for_completion(provider, "comp1", "echo_py", start->id);
  ASSERT_TRUE(final_state.has_value());
  EXPECT_EQ(final_state->status, "completed")
      << "error: " << (final_state->error.has_value() ? final_state->error->dump() : "none");
  ASSERT_TRUE(final_state->output_parameters.has_value());

  auto output = final_state->output_parameters.value();
  auto args = output["args"];
  // Should contain "-v"
  bool found = false;
  for (const auto & arg : args) {
    if (arg.get<std::string>() == "-v") {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected -v in args: " << args.dump();
}

// @verifies REQ_INTEROP_044
TEST_F(DefaultScriptProviderTest, EnvVars) {
  auto entry = make_real_script_entry("echo_py", "echo_params.py", "python");
  entry.env = {{"ROBOT_ID", "robot_42"}, {"GATEWAY_URL", "http://localhost:8080"}};
  auto config = make_config({entry});
  DefaultScriptProvider provider(config);

  ExecutionRequest req{"now", std::nullopt, std::nullopt};
  auto start = provider.start_execution("comp1", "echo_py", req);
  ASSERT_TRUE(start.has_value()) << start.error().message;

  auto final_state = wait_for_completion(provider, "comp1", "echo_py", start->id);
  ASSERT_TRUE(final_state.has_value());
  EXPECT_EQ(final_state->status, "completed")
      << "error: " << (final_state->error.has_value() ? final_state->error->dump() : "none");
  ASSERT_TRUE(final_state->output_parameters.has_value());

  auto output = final_state->output_parameters.value();
  ASSERT_TRUE(output.contains("env"));
  auto env = output["env"];
  EXPECT_EQ(env.value("ROBOT_ID", ""), "robot_42");
  EXPECT_EQ(env.value("GATEWAY_URL", ""), "http://localhost:8080");
}

// @verifies REQ_INTEROP_044
TEST_F(DefaultScriptProviderTest, StdinJsonFallback) {
  auto entry = make_real_script_entry("echo_py", "echo_params.py", "python");
  // No args config - parameters should be passed via stdin as JSON
  auto config = make_config({entry});
  DefaultScriptProvider provider(config);

  json params = {{"sensor", "lidar"}, {"threshold", 0.5}};
  ExecutionRequest req{"now", params, std::nullopt};
  auto start = provider.start_execution("comp1", "echo_py", req);
  ASSERT_TRUE(start.has_value()) << start.error().message;

  auto final_state = wait_for_completion(provider, "comp1", "echo_py", start->id);
  ASSERT_TRUE(final_state.has_value());
  EXPECT_EQ(final_state->status, "completed")
      << "error: " << (final_state->error.has_value() ? final_state->error->dump() : "none");
  ASSERT_TRUE(final_state->output_parameters.has_value());

  auto output = final_state->output_parameters.value();
  // echo_params.py reads stdin and puts it in output["stdin"]
  ASSERT_TRUE(output.contains("stdin")) << "Expected stdin in output: " << output.dump();
  EXPECT_EQ(output["stdin"]["sensor"].get<std::string>(), "lidar");
  EXPECT_DOUBLE_EQ(output["stdin"]["threshold"].get<double>(), 0.5);
}

// @verifies REQ_INTEROP_044
TEST_F(DefaultScriptProviderTest, ConcurrencyLimit) {
  // Create a script that sleeps briefly
  auto sleep_script = test_dir_ / "sleep.sh";
  {
    std::ofstream f(sleep_script);
    f << "#!/bin/sh\nsleep 10\n";
  }
  std::filesystem::permissions(sleep_script, std::filesystem::perms::owner_exec, std::filesystem::perm_options::add);

  ScriptEntryConfig entry;
  entry.id = "sleep_script";
  entry.name = "Sleep Script";
  entry.path = sleep_script.string();
  entry.format = "sh";
  entry.timeout_sec = 30;

  auto config = make_config({entry});
  config.max_concurrent_executions = 2;  // Low limit for testing
  DefaultScriptProvider provider(config);

  // Start 2 executions (the limit)
  ExecutionRequest req{"now", std::nullopt, std::nullopt};
  auto exec1 = provider.start_execution("comp1", "sleep_script", req);
  ASSERT_TRUE(exec1.has_value()) << exec1.error().message;

  auto exec2 = provider.start_execution("comp1", "sleep_script", req);
  ASSERT_TRUE(exec2.has_value()) << exec2.error().message;

  // Third should be rejected
  auto exec3 = provider.start_execution("comp1", "sleep_script", req);
  ASSERT_FALSE(exec3.has_value());
  EXPECT_EQ(exec3.error().code, ScriptBackendError::ConcurrencyLimit);

  // Clean up: stop the running executions
  (void)provider.control_execution("comp1", "sleep_script", exec1->id, "forced_termination");
  (void)provider.control_execution("comp1", "sleep_script", exec2->id, "forced_termination");

  // Wait for them to finish
  wait_for_completion(provider, "comp1", "sleep_script", exec1->id, 5000);
  wait_for_completion(provider, "comp1", "sleep_script", exec2->id, 5000);
}

// @verifies REQ_INTEROP_044
TEST_F(DefaultScriptProviderTest, UnsupportedExecutionType) {
  auto entry = make_real_script_entry("echo_py", "echo_params.py", "python");
  auto config = make_config({entry});
  DefaultScriptProvider provider(config);

  ExecutionRequest req{"on_restart", std::nullopt, std::nullopt};
  auto result = provider.start_execution("comp1", "echo_py", req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ScriptBackendError::UnsupportedType);
}

// @verifies REQ_INTEROP_046
TEST_F(DefaultScriptProviderTest, ExecutionNotFound) {
  auto config = make_config();
  DefaultScriptProvider provider(config);

  auto result = provider.get_execution("comp1", "script", "nonexistent_exec");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ScriptBackendError::NotFound);
}

TEST_F(DefaultScriptProviderTest, DeleteRunningExecutionFails) {
  auto sleep_script = test_dir_ / "sleep2.sh";
  {
    std::ofstream f(sleep_script);
    f << "#!/bin/sh\nsleep 10\n";
  }
  std::filesystem::permissions(sleep_script, std::filesystem::perms::owner_exec, std::filesystem::perm_options::add);

  ScriptEntryConfig entry;
  entry.id = "sleep2";
  entry.name = "Sleep";
  entry.path = sleep_script.string();
  entry.format = "sh";
  entry.timeout_sec = 30;
  auto config = make_config({entry});
  DefaultScriptProvider provider(config);

  ExecutionRequest req{"now", std::nullopt, std::nullopt};
  auto start = provider.start_execution("comp1", "sleep2", req);
  ASSERT_TRUE(start.has_value());

  // Try to delete while running
  auto del = provider.delete_execution("comp1", "sleep2", start->id);
  ASSERT_FALSE(del.has_value());
  EXPECT_EQ(del.error().code, ScriptBackendError::AlreadyRunning);

  // Clean up
  (void)provider.control_execution("comp1", "sleep2", start->id, "forced_termination");
  wait_for_completion(provider, "comp1", "sleep2", start->id, 5000);
}

TEST_F(DefaultScriptProviderTest, DeleteCompletedExecution) {
  auto entry = make_real_script_entry("echo_py", "echo_params.py", "python");
  auto config = make_config({entry});
  DefaultScriptProvider provider(config);

  ExecutionRequest req{"now", std::nullopt, std::nullopt};
  auto start = provider.start_execution("comp1", "echo_py", req);
  ASSERT_TRUE(start.has_value());

  auto final_state = wait_for_completion(provider, "comp1", "echo_py", start->id);
  ASSERT_TRUE(final_state.has_value());
  EXPECT_EQ(final_state->status, "completed");

  // Delete completed execution
  auto del = provider.delete_execution("comp1", "echo_py", start->id);
  EXPECT_TRUE(del.has_value());

  // Verify it's gone
  auto get = provider.get_execution("comp1", "echo_py", start->id);
  ASSERT_FALSE(get.has_value());
  EXPECT_EQ(get.error().code, ScriptBackendError::NotFound);
}

// @verifies REQ_INTEROP_047
TEST_F(DefaultScriptProviderTest, ControlStopExecution) {
  auto sleep_script = test_dir_ / "sleep3.sh";
  {
    std::ofstream f(sleep_script);
    f << "#!/bin/sh\nsleep 60\n";
  }
  std::filesystem::permissions(sleep_script, std::filesystem::perms::owner_exec, std::filesystem::perm_options::add);

  ScriptEntryConfig entry;
  entry.id = "sleep3";
  entry.name = "Sleep";
  entry.path = sleep_script.string();
  entry.format = "sh";
  entry.timeout_sec = 300;
  auto config = make_config({entry});
  DefaultScriptProvider provider(config);

  ExecutionRequest req{"now", std::nullopt, std::nullopt};
  auto start = provider.start_execution("comp1", "sleep3", req);
  ASSERT_TRUE(start.has_value());

  // Stop execution
  auto control = provider.control_execution("comp1", "sleep3", start->id, "stop");
  ASSERT_TRUE(control.has_value());
  EXPECT_EQ(control->status, "terminated");

  // Wait for monitor thread to reap the child
  wait_for_completion(provider, "comp1", "sleep3", start->id, 5000);
}

// @verifies REQ_INTEROP_044
TEST_F(DefaultScriptProviderTest, ScriptNotFoundExecution) {
  auto config = make_config();
  DefaultScriptProvider provider(config);

  ExecutionRequest req{"now", std::nullopt, std::nullopt};
  auto result = provider.start_execution("comp1", "nonexistent_script", req);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ScriptBackendError::NotFound);
}

// --- Path traversal tests ---

TEST_F(DefaultScriptProviderTest, PathTraversalScriptIdRejected) {
  auto config = make_config();
  DefaultScriptProvider provider(config);

  auto result = provider.get_script("comp1", "..");
  ASSERT_FALSE(result.has_value());
}

TEST_F(DefaultScriptProviderTest, DeletePathTraversalRejected) {
  auto config = make_config();
  DefaultScriptProvider provider(config);

  auto result = provider.delete_script("comp1", "..");
  ASSERT_FALSE(result.has_value());
}

TEST_F(DefaultScriptProviderTest, PathTraversalEntityIdRejected) {
  auto config = make_config();
  DefaultScriptProvider provider(config);

  auto result = provider.get_script("..", "script1");
  ASSERT_FALSE(result.has_value());
}

// @verifies REQ_INTEROP_046
TEST_F(DefaultScriptProviderTest, CrossEntityExecutionAccessRejected) {
  auto entry = make_real_script_entry("echo_py", "echo_params.py", "python");
  auto config = make_config({entry});
  DefaultScriptProvider provider(config);

  // Start execution as entity "comp1"
  ExecutionRequest req{"now", std::nullopt, std::nullopt};
  auto start = provider.start_execution("comp1", "echo_py", req);
  ASSERT_TRUE(start.has_value()) << start.error().message;

  auto exec_id = start->id;

  // get_execution via wrong entity -> NotFound
  auto get_wrong_entity = provider.get_execution("comp2", "echo_py", exec_id);
  ASSERT_FALSE(get_wrong_entity.has_value());
  EXPECT_EQ(get_wrong_entity.error().code, ScriptBackendError::NotFound);

  // get_execution via wrong script -> NotFound
  auto get_wrong_script = provider.get_execution("comp1", "other_script", exec_id);
  ASSERT_FALSE(get_wrong_script.has_value());
  EXPECT_EQ(get_wrong_script.error().code, ScriptBackendError::NotFound);

  // control_execution via wrong entity -> NotFound
  auto ctrl_wrong = provider.control_execution("comp2", "echo_py", exec_id, "stop");
  ASSERT_FALSE(ctrl_wrong.has_value());
  EXPECT_EQ(ctrl_wrong.error().code, ScriptBackendError::NotFound);

  // delete_execution via wrong entity -> NotFound (wait for completion first)
  auto final_state = wait_for_completion(provider, "comp1", "echo_py", exec_id);
  ASSERT_TRUE(final_state.has_value());
  EXPECT_EQ(final_state->status, "completed");

  auto del_wrong = provider.delete_execution("comp2", "echo_py", exec_id);
  ASSERT_FALSE(del_wrong.has_value());
  EXPECT_EQ(del_wrong.error().code, ScriptBackendError::NotFound);

  // Correct entity+script should still work
  auto get_correct = provider.get_execution("comp1", "echo_py", exec_id);
  ASSERT_TRUE(get_correct.has_value());
  EXPECT_EQ(get_correct->id, exec_id);
}

// @verifies REQ_INTEROP_043
TEST_F(DefaultScriptProviderTest, DeleteScriptWithRunningExecutionBlocked) {
  // Create a long-running sleep script on the filesystem (uploaded, not manifest)
  auto config = make_config();
  DefaultScriptProvider provider(config);

  // Upload a script that sleeps
  std::string content = "#!/bin/sh\nsleep 60\n";
  auto upload = provider.upload_script("comp1", "slow.sh", content, std::nullopt);
  ASSERT_TRUE(upload.has_value()) << upload.error().message;
  auto script_id = upload->id;

  // Start execution of the uploaded script
  ExecutionRequest req{"now", std::nullopt, std::nullopt};
  auto start = provider.start_execution("comp1", script_id, req);
  ASSERT_TRUE(start.has_value()) << start.error().message;
  auto exec_id = start->id;

  // Try to delete the script while execution is running -> AlreadyRunning
  auto del = provider.delete_script("comp1", script_id);
  ASSERT_FALSE(del.has_value());
  EXPECT_EQ(del.error().code, ScriptBackendError::AlreadyRunning);

  // Stop the execution
  auto ctrl = provider.control_execution("comp1", script_id, exec_id, "forced_termination");
  ASSERT_TRUE(ctrl.has_value());

  // Wait for it to finish
  wait_for_completion(provider, "comp1", script_id, exec_id, 5000);

  // Now delete should succeed
  auto del2 = provider.delete_script("comp1", script_id);
  EXPECT_TRUE(del2.has_value()) << del2.error().message;
}

// @verifies REQ_INTEROP_040
TEST_F(DefaultScriptProviderTest, UploadExceedingFileSizeRejected) {
  auto config = make_config();
  config.max_file_size_mb = 0;  // Zero MB limit - any content exceeds it
  DefaultScriptProvider provider(config);

  std::string content = "#!/usr/bin/env python3\nprint('hello')";
  auto result = provider.upload_script("comp1", "test.py", content, std::nullopt);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ScriptBackendError::FileTooLarge);
}

// @verifies REQ_INTEROP_047
TEST_F(DefaultScriptProviderTest, ControlCompletedExecutionReturnsNotRunning) {
  auto entry = make_real_script_entry("echo_py", "echo_params.py", "python");
  auto config = make_config({entry});
  DefaultScriptProvider provider(config);

  // Start a fast script
  ExecutionRequest req{"now", std::nullopt, std::nullopt};
  auto start = provider.start_execution("comp1", "echo_py", req);
  ASSERT_TRUE(start.has_value()) << start.error().message;

  // Wait for completion
  auto final_state = wait_for_completion(provider, "comp1", "echo_py", start->id);
  ASSERT_TRUE(final_state.has_value());
  EXPECT_EQ(final_state->status, "completed")
      << "error: " << (final_state->error.has_value() ? final_state->error->dump() : "none");

  // Attempt to stop the completed execution -> NotRunning
  auto ctrl = provider.control_execution("comp1", "echo_py", start->id, "stop");
  ASSERT_FALSE(ctrl.has_value());
  EXPECT_EQ(ctrl.error().code, ScriptBackendError::NotRunning);
}

}  // namespace
