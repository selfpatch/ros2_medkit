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

// Edge cases of the loopback classifier; the integration test covers the
// behaviour of the gateway on top of it.

#include <gtest/gtest.h>

#include "ros2_medkit_gateway/core/net/bind_address.hpp"

using ros2_medkit_gateway::is_loopback_bind_host;

TEST(BindAddress, LoopbackForms) {
  for (const char * host :
       {"127.0.0.1", "127.0.0.0", "127.255.255.254", "127.255.255.255", "127.1", "::1", "[::1]", "0:0:0:0:0:0:0:1",
        "::ffff:127.0.0.1", "::ffff:127.9.9.9", "localhost", "LOCALHOST", "LocalHost"}) {
    EXPECT_TRUE(is_loopback_bind_host(host)) << host;
  }
}

TEST(BindAddress, NotLoopback) {
  for (const char * host : {"",
                            "0.0.0.0",
                            "::",
                            "[::]",
                            "126.255.255.255",
                            "128.0.0.1",
                            "192.168.1.10",
                            "10.0.0.5",
                            "2001:db8::1",
                            "fe80::1",
                            "::ffff:10.0.0.5",
                            "::ffff:0.0.0.0",
                            "example.test",
                            "localhost.example.test",
                            "my-host",
                            "127.0.0.1 ",
                            " 127.0.0.1",
                            "127.0.0.1.example.test",
                            "[127.0.0.1]x",
                            "[",
                            "[]",
                            "::1%lo",
                            "0"}) {
    EXPECT_FALSE(is_loopback_bind_host(host)) << host;
  }
}
