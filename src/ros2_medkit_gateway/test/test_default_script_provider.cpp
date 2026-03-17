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

#include <filesystem>
#include <fstream>
#include <string>

using namespace ros2_medkit_gateway;
using json = nlohmann::json;

namespace {

class DefaultScriptProviderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    test_dir_ = std::filesystem::temp_directory_path() /
                ("test_scripts_" + std::to_string(getpid()) + "_" + std::to_string(test_counter_++));
    std::filesystem::create_directories(test_dir_);
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

  std::filesystem::path test_dir_;
  static int test_counter_;
};

int DefaultScriptProviderTest::test_counter_ = 0;

// @verifies REQ_INTEROP_090
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

// @verifies REQ_INTEROP_090
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

// @verifies REQ_INTEROP_090
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

// @verifies REQ_INTEROP_090
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

// @verifies REQ_INTEROP_090
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

// @verifies REQ_INTEROP_090
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

// @verifies REQ_INTEROP_090
TEST_F(DefaultScriptProviderTest, DeleteManifestScriptFails) {
  auto entry = make_manifest_entry("managed_script", "Managed Script");
  auto config = make_config({entry});

  DefaultScriptProvider provider(config);

  auto result = provider.delete_script("comp1", "managed_script");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ScriptBackendError::ManagedScript);
}

// @verifies REQ_INTEROP_090
TEST_F(DefaultScriptProviderTest, GetNonexistentScript) {
  auto config = make_config();
  DefaultScriptProvider provider(config);

  auto result = provider.get_script("comp1", "does_not_exist");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ScriptBackendError::NotFound);
}

// @verifies REQ_INTEROP_090
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

// @verifies REQ_INTEROP_090
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

// @verifies REQ_INTEROP_090
TEST_F(DefaultScriptProviderTest, UploadWithoutMetadata) {
  auto config = make_config();
  DefaultScriptProvider provider(config);

  auto result = provider.upload_script("comp1", "diagnostic.sh", "echo check", std::nullopt);
  ASSERT_TRUE(result.has_value());
  // Name defaults to filename when no metadata provided
  EXPECT_EQ(result->name, "diagnostic.sh");
}

// @verifies REQ_INTEROP_090
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

// @verifies REQ_INTEROP_090
TEST_F(DefaultScriptProviderTest, EntityFilterWildcard) {
  auto entry = make_manifest_entry("wildcard_script", "Wildcard Script", "For all entities", {"*"});
  auto config = make_config({entry});

  DefaultScriptProvider provider(config);

  auto result = provider.list_scripts("any_entity_id");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->size(), 1u);
}

// @verifies REQ_INTEROP_090
TEST_F(DefaultScriptProviderTest, EntityFilterEmpty) {
  // Empty filter matches everything
  auto entry = make_manifest_entry("unfiltered_script", "Unfiltered Script", "Matches all");
  auto config = make_config({entry});

  DefaultScriptProvider provider(config);

  auto result = provider.list_scripts("anything");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->size(), 1u);
}

// @verifies REQ_INTEROP_090
TEST_F(DefaultScriptProviderTest, DeleteNonexistentScriptFails) {
  auto config = make_config();
  DefaultScriptProvider provider(config);

  auto result = provider.delete_script("comp1", "nonexistent");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ScriptBackendError::NotFound);
}

// @verifies REQ_INTEROP_090
TEST_F(DefaultScriptProviderTest, ExecutionMethodsStubbed) {
  auto config = make_config();
  DefaultScriptProvider provider(config);

  ExecutionRequest req{"now", std::nullopt, std::nullopt};

  auto start = provider.start_execution("comp1", "script_0", req);
  ASSERT_FALSE(start.has_value());
  EXPECT_EQ(start.error().code, ScriptBackendError::Internal);

  auto get = provider.get_execution("comp1", "script_0", "exec_0");
  ASSERT_FALSE(get.has_value());
  EXPECT_EQ(get.error().code, ScriptBackendError::Internal);

  auto control = provider.control_execution("comp1", "script_0", "exec_0", "stop");
  ASSERT_FALSE(control.has_value());
  EXPECT_EQ(control.error().code, ScriptBackendError::Internal);

  auto del = provider.delete_execution("comp1", "script_0", "exec_0");
  ASSERT_FALSE(del.has_value());
  EXPECT_EQ(del.error().code, ScriptBackendError::Internal);
}

// @verifies REQ_INTEROP_090
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

// @verifies REQ_INTEROP_090
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

}  // namespace
