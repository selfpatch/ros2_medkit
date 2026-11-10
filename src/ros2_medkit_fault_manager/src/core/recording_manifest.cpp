// Copyright 2026 mfaferek93
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

#include "ros2_medkit_fault_manager/core/recording_manifest.hpp"

#include <unistd.h>

#include <array>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <vector>

#include <nlohmann/json.hpp>
#include <openssl/evp.h>
#include <yaml-cpp/yaml.h>

#include "ros2_medkit_fault_manager/fault_storage.hpp"

namespace ros2_medkit_fault_manager::core {

namespace {

constexpr int kManifestVersion = 1;

struct MdCtxDeleter {
  void operator()(EVP_MD_CTX * ctx) const {
    EVP_MD_CTX_free(ctx);
  }
};

struct FileCloser {
  void operator()(std::FILE * file) const {
    std::fclose(file);
  }
};

}  // namespace

std::optional<std::string> sha256_file_hex(const std::string & path) {
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    return std::nullopt;
  }
  std::unique_ptr<EVP_MD_CTX, MdCtxDeleter> ctx(EVP_MD_CTX_new());
  if (!ctx || EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) != 1) {
    return std::nullopt;
  }
  std::vector<char> buffer(1 << 20);
  while (file) {
    file.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = static_cast<size_t>(file.gcount());
    if (count > 0 && EVP_DigestUpdate(ctx.get(), buffer.data(), count) != 1) {
      return std::nullopt;
    }
  }
  if (file.bad()) {
    return std::nullopt;
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_len = 0;
  if (EVP_DigestFinal_ex(ctx.get(), digest.data(), &digest_len) != 1) {
    return std::nullopt;
  }
  static constexpr char kHex[] = "0123456789abcdef";
  std::string hex;
  hex.reserve(static_cast<size_t>(digest_len) * 2);
  for (unsigned int i = 0; i < digest_len; ++i) {
    hex.push_back(kHex[digest[i] >> 4]);
    hex.push_back(kHex[digest[i] & 0x0F]);
  }
  return hex;
}

std::optional<std::string> build_recording_manifest(const std::string & bag_path, int64_t created_at_ns,
                                                    std::string & error) {
  const std::filesystem::path bag_dir(bag_path);

  std::string storage_id;
  std::vector<std::string> names;
  try {
    const YAML::Node info = YAML::LoadFile((bag_dir / "metadata.yaml").string())["rosbag2_bagfile_information"];
    storage_id = info["storage_identifier"].as<std::string>();
    // Order comes from metadata.yaml: a directory listing sorts `_10` before `_2`.
    for (const auto & entry : info["relative_file_paths"]) {
      names.push_back(std::filesystem::path(entry.as<std::string>()).filename().string());
    }
  } catch (const std::exception & e) {
    error = std::string("cannot read metadata.yaml: ") + e.what();
    return std::nullopt;
  }

  if (storage_id != "mcap" && storage_id != "sqlite3") {
    error = "unsupported storage format: " + storage_id;
    return std::nullopt;
  }
  if (names.empty()) {
    error = "metadata.yaml names no storage file";
    return std::nullopt;
  }

  nlohmann::ordered_json files = nlohmann::ordered_json::array();
  uint64_t total_bytes = 0;
  std::set<std::string> seen;
  for (const auto & name : names) {
    if (name.empty() || !seen.insert(name).second) {
      error = "metadata.yaml names an empty or repeated storage file";
      return std::nullopt;
    }
    const std::filesystem::path file = bag_dir / name;
    std::error_code ec;
    if (!std::filesystem::is_regular_file(file, ec)) {
      error = "storage file missing: " + name;
      return std::nullopt;
    }
    const auto size = std::filesystem::file_size(file, ec);
    if (ec) {
      error = "cannot size storage file: " + name;
      return std::nullopt;
    }
    const auto sha256 = sha256_file_hex(file.string());
    if (!sha256) {
      error = "cannot hash storage file: " + name;
      return std::nullopt;
    }
    files.push_back({{"name", name}, {"size", static_cast<uint64_t>(size)}, {"sha256", *sha256}});
    total_bytes += static_cast<uint64_t>(size);
  }

  nlohmann::ordered_json manifest;
  manifest["manifest_version"] = kManifestVersion;
  manifest["recording_id"] = rosbag_recording_id(bag_path);
  manifest["format"] = storage_id;
  manifest["created_at_ns"] = created_at_ns;
  manifest["total_bytes"] = total_bytes;
  manifest["files"] = std::move(files);
  return manifest.dump(2) + "\n";
}

std::optional<std::string> write_recording_manifest(const std::string & bag_path, int64_t created_at_ns) {
  const std::filesystem::path bag_dir(bag_path);
  const std::filesystem::path target = bag_dir / kRecordingManifestFile;
  std::error_code ec;
  if (std::filesystem::exists(target, ec)) {
    return std::string("manifest already exists");
  }

  std::string error;
  const auto text = build_recording_manifest(bag_path, created_at_ns, error);
  if (!text) {
    return error;
  }

  const std::filesystem::path temp = bag_dir / (std::string(".") + kRecordingManifestFile + ".tmp");
  bool written = false;
  {
    std::unique_ptr<std::FILE, FileCloser> file(std::fopen(temp.c_str(), "wb"));
    if (file) {
      written = std::fwrite(text->data(), 1, text->size(), file.get()) == text->size() &&
                std::fflush(file.get()) == 0 && ::fsync(::fileno(file.get())) == 0;
    }
  }
  if (written) {
    std::filesystem::rename(temp, target, ec);
    if (!ec) {
      return std::nullopt;
    }
  }
  std::filesystem::remove(temp, ec);
  return std::string("cannot write ") + kRecordingManifestFile;
}

}  // namespace ros2_medkit_fault_manager::core
