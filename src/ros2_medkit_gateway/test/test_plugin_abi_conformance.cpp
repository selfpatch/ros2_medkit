// Copyright 2026 bburda
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

// Plugin ABI conformance test.
//
// Locks in the public plugin ABI surface so future internal refactors that
// would silently break commercial / out-of-tree plugins (UDS, OPC-UA, Uptane
// OTA, Mender OTA) fail loudly here. The contract under test:
//
//   1. Compile-time shape of the four public ABI types:
//        - GatewayPlugin (virtual interface, get_routes return type)
//        - GatewayPlugin::PluginRoute (struct layout, handler signature)
//        - PluginRequest (ctor takes const httplib::Request *, accessor shapes)
//        - PluginResponse (ctor takes void *, send_json / send_error signatures)
//      plus PLUGIN_API_VERSION constant existence and integer type.
//
//   2. Runtime ABI: the in-tree test plugin (`libtest_gateway_plugin.so`,
//      build artefact of test/demo_nodes/test_gateway_plugin.cpp) loads via
//      the same `PluginLoader::load` entry point the production gateway uses,
//      its `get_routes()` returns the routes the plugin author declared, and
//      each route handler can be invoked with a synthetic PluginRequest +
//      PluginResponse pair.
//
//   3. Wire format: when a plugin handler calls PluginResponse::send_json /
//      send_error, the resulting httplib::Response body is the byte-for-byte
//      output of `http::detail::write_json_body` / `write_generic_error`
//      (Content-Type "application/json", 2-space indent, SOVD GenericError
//      shape for errors). This pins the friend-gate path introduced in
//      commits 6 / 30: plugins MUST go through the typed router's primitives,
//      not bypass them.

#include <gtest/gtest.h>
#include <httplib.h>

#include <ament_index_cpp/get_package_prefix.hpp>

#include <functional>
#include <regex>
#include <string>
#include <type_traits>
#include <vector>

#include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_http_types.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_loader.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_types.hpp"

namespace ros2_medkit_gateway {
namespace {

// ─── Compile-time ABI guards ────────────────────────────────────────────────
//
// These static_asserts pin the shape of the plugin ABI types. Any future
// refactor that changes a signature, member type, or constructor parameter
// will fail to compile here long before it breaks a downstream plugin .so.

// PLUGIN_API_VERSION exists, is constexpr, and is an int. Out-of-tree plugin
// builds key off this value via `extern "C" int plugin_api_version()`.
static_assert(std::is_same_v<decltype(PLUGIN_API_VERSION), const int>,
              "PLUGIN_API_VERSION must remain a `constexpr int` to preserve the C ABI");

// GatewayPlugin is polymorphic and has a virtual destructor.
static_assert(std::is_polymorphic_v<GatewayPlugin>, "GatewayPlugin must remain polymorphic");
static_assert(std::has_virtual_destructor_v<GatewayPlugin>,
              "GatewayPlugin must keep its virtual destructor for correct dlclose ordering");

// PluginRoute layout. The handler signature is the load-bearing piece - it is
// what every plugin author writes today. Changing it silently miscompiles
// every existing plugin .so.
using PluginRoute = GatewayPlugin::PluginRoute;
static_assert(std::is_same_v<decltype(std::declval<PluginRoute>().method), std::string>,
              "PluginRoute::method must remain std::string");
static_assert(std::is_same_v<decltype(std::declval<PluginRoute>().pattern), std::string>,
              "PluginRoute::pattern must remain std::string");
static_assert(std::is_same_v<decltype(std::declval<PluginRoute>().handler),
                             std::function<void(const PluginRequest &, PluginResponse &)>>,
              "PluginRoute::handler signature is part of the plugin ABI");

// get_routes() returns std::vector<PluginRoute>. Plugins implement this
// override; changing the return type breaks every existing override.
static_assert(std::is_same_v<decltype(std::declval<GatewayPlugin>().get_routes()), std::vector<PluginRoute>>,
              "GatewayPlugin::get_routes() must return std::vector<PluginRoute>");

// PluginRequest constructor takes `const void *` (opaque pointer to a
// cpp-httplib request). The opaque-pointer indirection is intentional: it
// lets us swap httplib without breaking the plugin headers. Plugins call the
// ctor via the gateway's plugin shim path; tests construct it directly with
// a httplib::Request *.
static_assert(std::is_constructible_v<PluginRequest, const void *>,
              "PluginRequest must be constructible from const void *");
static_assert(std::is_constructible_v<PluginRequest, const httplib::Request *>,
              "PluginRequest must accept a const httplib::Request * (the shim path)");

// PluginResponse constructor takes `void *` (opaque pointer to a cpp-httplib
// response). Same rationale as PluginRequest.
static_assert(std::is_constructible_v<PluginResponse, void *>, "PluginResponse must be constructible from void *");
static_assert(std::is_constructible_v<PluginResponse, httplib::Response *>,
              "PluginResponse must accept a httplib::Response * (the shim path)");

// PluginResponse member function signatures. These are the only emission
// surface plugins have; they must keep their exact shape so commercial
// plugins compile across gateway versions.
using SendJsonSig = void (PluginResponse::*)(const nlohmann::json &);
using SendErrorSig = void (PluginResponse::*)(int, const std::string &, const std::string &, const nlohmann::json &);
static_assert(std::is_same_v<decltype(&PluginResponse::send_json), SendJsonSig>,
              "PluginResponse::send_json signature is part of the plugin ABI");
static_assert(std::is_same_v<decltype(&PluginResponse::send_error), SendErrorSig>,
              "PluginResponse::send_error signature is part of the plugin ABI");

// PluginRequest accessor signatures.
using PathParamSig = std::string (PluginRequest::*)(size_t) const;
using HeaderSig = std::string (PluginRequest::*)(const std::string &) const;
using PathSig = const std::string & (PluginRequest::*)() const;
using BodySig = const std::string & (PluginRequest::*)() const;
using QueryParamSig = std::string (PluginRequest::*)(const std::string &) const;
static_assert(std::is_same_v<decltype(&PluginRequest::path_param), PathParamSig>,
              "PluginRequest::path_param signature is part of the plugin ABI");
static_assert(std::is_same_v<decltype(&PluginRequest::header), HeaderSig>,
              "PluginRequest::header signature is part of the plugin ABI");
static_assert(std::is_same_v<decltype(&PluginRequest::path), PathSig>,
              "PluginRequest::path signature is part of the plugin ABI");
static_assert(std::is_same_v<decltype(&PluginRequest::body), BodySig>,
              "PluginRequest::body signature is part of the plugin ABI");
static_assert(std::is_same_v<decltype(&PluginRequest::query_param), QueryParamSig>,
              "PluginRequest::query_param signature is part of the plugin ABI");

// ─── Runtime helpers ────────────────────────────────────────────────────────

std::string test_plugin_path() {
  return ament_index_cpp::get_package_prefix("ros2_medkit_gateway") +
         "/lib/ros2_medkit_gateway/libtest_gateway_plugin.so";
}

}  // namespace

// ─── Runtime tests ──────────────────────────────────────────────────────────

// The reference test plugin loads via the same PluginLoader::load entry point
// the production gateway uses. This locks in that the dlopen / dlsym path is
// unchanged for plugin authors.
TEST(PluginAbiConformance, TestPluginLoadsViaProductionEntryPoint) {
  auto result = PluginLoader::load(test_plugin_path());
  ASSERT_TRUE(result.has_value()) << result.error();
  ASSERT_NE(result->plugin, nullptr);
  EXPECT_EQ(result->plugin->name(), "test_plugin");
}

// The plugin's `get_routes()` returns the route table the plugin author
// declared. We assert on the routes the in-tree test plugin actually
// registers (an `x-test/ping` extension and a per-component diagnostics
// endpoint), which is enough to demonstrate that PluginRoute survives the
// dlopen boundary.
TEST(PluginAbiConformance, TestPluginRegistersExpectedRoutes) {
  auto result = PluginLoader::load(test_plugin_path());
  ASSERT_TRUE(result.has_value()) << result.error();
  ASSERT_NE(result->plugin, nullptr);

  auto routes = result->plugin->get_routes();
  ASSERT_EQ(routes.size(), 2u) << "test plugin's route table has changed; update this assertion or the plugin";

  // Routes are returned in source order in test_gateway_plugin.cpp.
  EXPECT_EQ(routes[0].method, "GET");
  EXPECT_EQ(routes[0].pattern, "x-test/ping");
  EXPECT_TRUE(static_cast<bool>(routes[0].handler));

  EXPECT_EQ(routes[1].method, "GET");
  EXPECT_EQ(routes[1].pattern, R"(components/([^/]+)/x-medkit-diagnostics)");
  EXPECT_TRUE(static_cast<bool>(routes[1].handler));
}

// Sanity check: the plugin's simplest handler (the ping route) can be
// invoked with synthetic PluginRequest / PluginResponse, exactly as the
// gateway's plugin shim (`PluginManager::register_routes`) does at runtime.
// This pins the handler invocation shape - the plugin shim path constructs
// the PluginRequest from `const httplib::Request *` and PluginResponse from
// `httplib::Response *`, then calls the std::function with both by
// reference.
TEST(PluginAbiConformance, PluginHandlerInvokableViaShimPath) {
  auto result = PluginLoader::load(test_plugin_path());
  ASSERT_TRUE(result.has_value()) << result.error();

  auto routes = result->plugin->get_routes();
  ASSERT_FALSE(routes.empty());

  // Invoke the GET x-test/ping handler. It does not touch the plugin context
  // and so is safe to call without a full gateway stand-up; the per-component
  // diagnostics route requires PluginContext::validate_entity_for_route and is
  // out of scope for an ABI conformance check.
  httplib::Request req;
  httplib::Response res;
  PluginRequest preq(&req);
  PluginResponse pres(&res);

  ASSERT_NO_THROW(routes[0].handler(preq, pres));

  auto body = nlohmann::json::parse(res.body);
  EXPECT_EQ(body["response"], "pong");
}

// Wire format: PluginResponse::send_json must emit through
// http::detail::write_json_body. That primitive sets Content-Type to
// "application/json" and serialises the body with 2-space indent. If a
// future refactor swaps PluginResponse off the friend-gated path, the
// Content-Type / indent / sentinel-status contract will drift and this test
// will fail.
TEST(PluginAbiConformance, SendJsonGoesThroughWriteJsonBodyPrimitive) {
  httplib::Response res;
  PluginResponse pres(&res);

  pres.send_json({{"answer", 42}});

  EXPECT_EQ(res.get_header_value("Content-Type"), "application/json");

  // 2-space indent is part of write_json_body's contract. Asserting on the
  // raw byte sequence catches both "switched to compact dump()" and
  // "switched to 4-space indent" regressions.
  EXPECT_NE(res.body.find("\n  \"answer\": 42"), std::string::npos)
      << "send_json no longer emits 2-space-indented JSON; the primitive path is broken. Body: " << res.body;
}

// Wire format: PluginResponse::send_error must emit through
// http::detail::write_generic_error. That primitive produces the SOVD
// GenericError envelope (`error_code` + `message`, with the status clamped
// into 400-599). Asserting on the envelope shape pins the primitive path.
TEST(PluginAbiConformance, SendErrorGoesThroughWriteGenericErrorPrimitive) {
  httplib::Response res;
  PluginResponse pres(&res);

  pres.send_error(404, "x-medkit-test", "test message", nlohmann::json{{"hint", "details"}});

  EXPECT_EQ(res.status, 404);
  EXPECT_EQ(res.get_header_value("Content-Type"), "application/json");

  auto body = nlohmann::json::parse(res.body);
  // x-medkit-* codes are re-mapped to the SOVD vendor-error envelope by
  // write_generic_error. This re-mapping is the load-bearing signal that
  // the primitive (not a direct set_content) emitted the body.
  EXPECT_EQ(body["error_code"], "vendor-error");
  EXPECT_EQ(body["vendor_code"], "x-medkit-test");
  EXPECT_EQ(body["message"], "test message");
  ASSERT_TRUE(body.contains("parameters"));
  EXPECT_EQ(body["parameters"]["hint"], "details");
}

// Status clamping is part of the wire-format contract. The primitive clamps
// out-of-range statuses into [400, 599]; PluginResponse pre-clamps as well
// (see plugin_http_types.cpp). Both layers cooperating is what the contract
// guarantees, so we assert on the final wire status only.
TEST(PluginAbiConformance, SendErrorClampsStatusIntoSovdRange) {
  httplib::Response res;
  PluginResponse pres(&res);

  pres.send_error(200, "x-medkit-test", "should clamp upward to 400");
  EXPECT_EQ(res.status, 400);

  httplib::Response res2;
  PluginResponse pres2(&res2);
  pres2.send_error(999, "x-medkit-test", "should clamp downward to 599");
  EXPECT_EQ(res2.status, 599);
}

}  // namespace ros2_medkit_gateway
