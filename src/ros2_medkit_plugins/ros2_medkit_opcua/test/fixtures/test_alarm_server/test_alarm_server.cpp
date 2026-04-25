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

// Standalone OPC UA test fixture: emits AlarmConditionType events for
// integration testing of the ros2_medkit_opcua native alarm subscription
// code path. OpenPLC does not implement AlarmConditionType (only
// OffNormalAlarmType-derived demos with no Acknowledge / Confirm methods),
// so we build this companion server to cover the full Part 9 surface.
//
// Reads commands from stdin, one per line; writes "OK <name>" or
// "ERR <reason>" to stdout for the test harness to poll. See
// docs in the header comment of fire(), clear(), ack(), etc. below.

#include <open62541/plugin/log_stdout.h>
#include <open62541/server.h>
#include <open62541/server_config_default.h>
#include <open62541/types_generated.h>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <signal.h>
#include <sstream>
#include <string>
#include <thread>

namespace {

constexpr const char * NS_URI = "urn:test:alarms";

struct Condition {
  std::string name;
  UA_NodeId node{};
  UA_NodeId source{};
  // Local mirror of the OPC-UA state we expose. Kept in lockstep with the
  // condition node by every handler so the test harness can grep one line
  // for the truth instead of running a separate OPC-UA browse round-trip.
  bool active{false};
  bool acked{true};
  bool confirmed{true};
  bool enabled{true};
  bool shelved{false};
  bool retain{false};
};

void log_state(const Condition & c) {
  std::cout << "STATE " << c.name << " active=" << (c.active ? "true" : "false")
            << " acked=" << (c.acked ? "true" : "false") << " confirmed=" << (c.confirmed ? "true" : "false")
            << " enabled=" << (c.enabled ? "true" : "false") << " shelved=" << (c.shelved ? "true" : "false")
            << " retain=" << (c.retain ? "true" : "false") << std::endl;
}

std::map<std::string, Condition> g_conditions;
std::mutex g_mutex;
std::atomic<bool> g_running{true};

void stop_handler(int) {
  g_running = false;
}

UA_StatusCode add_source(UA_Server * server, const std::string & name, UA_UInt16 ns, UA_NodeId * out) {
  UA_ObjectAttributes attr = UA_ObjectAttributes_default;
  attr.eventNotifier = 1;
  std::string display = name + "Source";
  attr.displayName = UA_LOCALIZEDTEXT(const_cast<char *>("en"), const_cast<char *>(display.c_str()));
  UA_QualifiedName qname = UA_QUALIFIEDNAME(ns, const_cast<char *>(display.c_str()));
  // Use a predictable string NodeId ``Alarms.<name>`` in the user namespace so
  // the gateway's ``alarm_source: "ns=2;s=Alarms.Overpressure"`` config can
  // address this exact node. Auto-assigned numeric IDs (the previous form)
  // would not be reproducible across server restarts and would force the test
  // harness to browse-resolve at runtime.
  std::string source_id = "Alarms." + name;
  UA_NodeId requested = UA_NODEID_STRING_ALLOC(ns, source_id.c_str());
  UA_StatusCode rc = UA_Server_addObjectNode(server, requested, UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                                             UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES), qname,
                                             UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE), attr, nullptr, out);
  UA_NodeId_clear(&requested);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  // The condition source must be a notifier of the Server object so that the
  // A&C subsystem can route events through the standard notification path.
  UA_ExpandedNodeId target;
  UA_ExpandedNodeId_init(&target);
  UA_NodeId_copy(out, &target.nodeId);
  rc = UA_Server_addReference(server, UA_NODEID_NUMERIC(0, UA_NS0ID_SERVER), UA_NODEID_NUMERIC(0, UA_NS0ID_HASNOTIFIER),
                              target, UA_TRUE);
  UA_ExpandedNodeId_clear(&target);
  return rc;
}

UA_StatusCode add_condition(UA_Server * server, const std::string & name, UA_UInt16 ns, Condition & out) {
  out.name = name;
  UA_StatusCode rc = add_source(server, name, ns, &out.source);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  UA_QualifiedName cqname = UA_QUALIFIEDNAME(ns, const_cast<char *>(name.c_str()));
  rc = UA_Server_createCondition(server, UA_NODEID_NUMERIC(ns, 0), UA_NODEID_NUMERIC(0, UA_NS0ID_ALARMCONDITIONTYPE),
                                 cqname, out.source, UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT), &out.node);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }

  // Add ShelvingState as an optional field. open62541's createCondition only
  // materializes mandatory fields; ShelvingState is optional in Part 9 but
  // tests assert it is present, so we add it explicitly. Failure here is
  // non-fatal - some tests skip the shelve transition path entirely.
  UA_NodeId shelving;
  UA_Server_addConditionOptionalField(server, out.node, UA_NODEID_NUMERIC(0, UA_NS0ID_ALARMCONDITIONTYPE),
                                      UA_QUALIFIEDNAME(0, const_cast<char *>("ShelvingState")), &shelving);

  // Enable the condition so events are emitted (Part 9: EnabledState=true is
  // a precondition for Retain/Active/Acked transitions to fire events).
  UA_Variant value;
  UA_Boolean enabled = true;
  UA_Variant_setScalar(&value, &enabled, &UA_TYPES[UA_TYPES_BOOLEAN]);
  rc = UA_Server_setConditionVariableFieldProperty(server, out.node, &value,
                                                   UA_QUALIFIEDNAME(0, const_cast<char *>("EnabledState")),
                                                   UA_QUALIFIEDNAME(0, const_cast<char *>("Id")));
  return rc;
}

UA_StatusCode set_two_state(UA_Server * server, const UA_NodeId & cond, const char * field, UA_Boolean value) {
  UA_Variant v;
  UA_Variant_setScalar(&v, &value, &UA_TYPES[UA_TYPES_BOOLEAN]);
  return UA_Server_setConditionVariableFieldProperty(server, cond, &v, UA_QUALIFIEDNAME(0, const_cast<char *>(field)),
                                                     UA_QUALIFIEDNAME(0, const_cast<char *>("Id")));
}

UA_StatusCode set_field_bool(UA_Server * server, const UA_NodeId & cond, const char * field, UA_Boolean value) {
  UA_Variant v;
  UA_Variant_setScalar(&v, &value, &UA_TYPES[UA_TYPES_BOOLEAN]);
  return UA_Server_setConditionField(server, cond, &v, UA_QUALIFIEDNAME(0, const_cast<char *>(field)));
}

UA_StatusCode handle_fire(UA_Server * server, const Condition & c, UA_UInt16 severity) {
  UA_Variant v;
  UA_Variant_setScalar(&v, &severity, &UA_TYPES[UA_TYPES_UINT16]);
  UA_StatusCode rc =
      UA_Server_setConditionField(server, c.node, &v, UA_QUALIFIEDNAME(0, const_cast<char *>("Severity")));
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  UA_LocalizedText msg = UA_LOCALIZEDTEXT(const_cast<char *>("en"), const_cast<char *>("Alarm fired"));
  UA_Variant_setScalar(&v, &msg, &UA_TYPES[UA_TYPES_LOCALIZEDTEXT]);
  rc = UA_Server_setConditionField(server, c.node, &v, UA_QUALIFIEDNAME(0, const_cast<char *>("Message")));
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  rc = set_field_bool(server, c.node, "Retain", true);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  rc = set_two_state(server, c.node, "AckedState", false);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  rc = set_two_state(server, c.node, "ConfirmedState", false);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  // ActiveState=true is the trigger that makes the A&C subsystem regenerate
  // an EventId and broadcast a CONFIRMED notification.
  rc = set_two_state(server, c.node, "ActiveState", true);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  return UA_Server_triggerConditionEvent(server, c.node, c.source, nullptr);
}

UA_StatusCode handle_clear(UA_Server * server, const Condition & c) {
  UA_StatusCode rc = set_two_state(server, c.node, "ActiveState", false);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  rc = set_field_bool(server, c.node, "Retain", false);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  return UA_Server_triggerConditionEvent(server, c.node, c.source, nullptr);
}

UA_StatusCode handle_latch(UA_Server * server, const Condition & c) {
  UA_StatusCode rc = set_two_state(server, c.node, "ActiveState", false);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  rc = set_field_bool(server, c.node, "Retain", true);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  return UA_Server_triggerConditionEvent(server, c.node, c.source, nullptr);
}

UA_StatusCode handle_ack(UA_Server * server, const Condition & c) {
  UA_StatusCode rc = set_two_state(server, c.node, "AckedState", true);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  return UA_Server_triggerConditionEvent(server, c.node, c.source, nullptr);
}

UA_StatusCode handle_confirm(UA_Server * server, const Condition & c) {
  UA_StatusCode rc = set_two_state(server, c.node, "ConfirmedState", true);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  rc = set_field_bool(server, c.node, "Retain", false);
  if (rc != UA_STATUSCODE_GOOD) {
    return rc;
  }
  return UA_Server_triggerConditionEvent(server, c.node, c.source, nullptr);
}

UA_StatusCode set_shelving(UA_Server * server, const Condition & c, bool shelved) {
  // ShelvingState is a ShelvedStateMachineType sub-object on the condition
  // (Part 9). open62541's experimental A&C does not implement the TimedShelve
  // / Unshelve methods, so for the test fixture we resolve the path
  // ShelvingState/CurrentState via translateBrowsePath and write both the
  // LocalizedText and the Id (NodeId) properties directly. The medkit
  // EventFilter reads ``ShelvingState/CurrentState/Id`` (a NodeId pointing
  // at one of i=2929 Unshelved / i=2930 TimedShelved / i=2932 OneShotShelved)
  // because the LocalizedText is locale-dependent. Writing only the text
  // leaves Id at its default and the bridge sees shelved=false.
  UA_RelativePathElement elems[2];
  UA_RelativePathElement_init(&elems[0]);
  UA_RelativePathElement_init(&elems[1]);
  elems[0].targetName = UA_QUALIFIEDNAME(0, const_cast<char *>("ShelvingState"));
  elems[0].referenceTypeId = UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT);
  elems[1].targetName = UA_QUALIFIEDNAME(0, const_cast<char *>("CurrentState"));
  elems[1].referenceTypeId = UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT);
  UA_BrowsePath path;
  UA_BrowsePath_init(&path);
  path.startingNode = c.node;
  path.relativePath.elementsSize = 2;
  path.relativePath.elements = elems;
  UA_BrowsePathResult result = UA_Server_translateBrowsePathToNodeIds(server, &path);
  if (result.statusCode != UA_STATUSCODE_GOOD || result.targetsSize == 0) {
    UA_BrowsePathResult_clear(&result);
    return UA_STATUSCODE_BADNOTFOUND;
  }
  UA_NodeId currentState = result.targets[0].targetId.nodeId;
  UA_LocalizedText state = shelved ? UA_LOCALIZEDTEXT(const_cast<char *>("en"), const_cast<char *>("TimedShelved"))
                                   : UA_LOCALIZEDTEXT(const_cast<char *>("en"), const_cast<char *>("Unshelved"));
  UA_Variant v;
  UA_Variant_setScalar(&v, &state, &UA_TYPES[UA_TYPES_LOCALIZEDTEXT]);
  UA_StatusCode rc = UA_Server_writeValue(server, currentState, v);
  if (rc != UA_STATUSCODE_GOOD) {
    UA_BrowsePathResult_clear(&result);
    return rc;
  }

  // Write the Id property of CurrentState. The medkit alarm bridge keys
  // suppression off ``ShelvingState/CurrentState/Id`` (one of i=2929 /
  // i=2930 / i=2932) - text is informational only.
  UA_RelativePathElement idElems[3];
  UA_RelativePathElement_init(&idElems[0]);
  UA_RelativePathElement_init(&idElems[1]);
  UA_RelativePathElement_init(&idElems[2]);
  idElems[0].targetName = UA_QUALIFIEDNAME(0, const_cast<char *>("ShelvingState"));
  idElems[0].referenceTypeId = UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT);
  idElems[1].targetName = UA_QUALIFIEDNAME(0, const_cast<char *>("CurrentState"));
  idElems[1].referenceTypeId = UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT);
  idElems[2].targetName = UA_QUALIFIEDNAME(0, const_cast<char *>("Id"));
  idElems[2].referenceTypeId = UA_NODEID_NUMERIC(0, UA_NS0ID_HASPROPERTY);
  UA_BrowsePath idPath;
  UA_BrowsePath_init(&idPath);
  idPath.startingNode = c.node;
  idPath.relativePath.elementsSize = 3;
  idPath.relativePath.elements = idElems;
  UA_BrowsePathResult idResult = UA_Server_translateBrowsePathToNodeIds(server, &idPath);
  if (idResult.statusCode == UA_STATUSCODE_GOOD && idResult.targetsSize > 0) {
    UA_NodeId stateIdNode = UA_NODEID_NUMERIC(0, shelved ? 2930u /* TimedShelved */ : 2929u /* Unshelved */);
    UA_Variant idVar;
    UA_Variant_setScalar(&idVar, &stateIdNode, &UA_TYPES[UA_TYPES_NODEID]);
    UA_Server_writeValue(server, idResult.targets[0].targetId.nodeId, idVar);
  }
  UA_BrowsePathResult_clear(&idResult);
  UA_BrowsePathResult_clear(&result);
  return UA_Server_triggerConditionEvent(server, c.node, c.source, nullptr);
}

UA_StatusCode handle_enable(UA_Server * server, const Condition & c, bool enable) {
  return set_two_state(server, c.node, "EnabledState", enable);
}

void cli_loop(UA_Server * server) {
  std::string line;
  while (g_running && std::getline(std::cin, line)) {
    std::istringstream iss(line);
    std::string cmd, name;
    iss >> cmd >> name;
    if (cmd == "quit") {
      g_running = false;
      std::cout << "OK quit" << std::endl;
      break;
    }
    std::lock_guard<std::mutex> guard(g_mutex);
    auto it = g_conditions.find(name);
    if (cmd != "quit" && it == g_conditions.end()) {
      std::cout << "ERR unknown_condition:" << name << std::endl;
      continue;
    }
    Condition & cref = it->second;
    UA_StatusCode rc = UA_STATUSCODE_BADNOTSUPPORTED;
    if (cmd == "fire") {
      UA_UInt16 sev = 500;
      iss >> sev;
      rc = handle_fire(server, cref, sev);
      if (rc == UA_STATUSCODE_GOOD) {
        cref.active = true;
        cref.acked = false;
        cref.confirmed = false;
        cref.retain = true;
      }
    } else if (cmd == "clear") {
      rc = handle_clear(server, cref);
      if (rc == UA_STATUSCODE_GOOD) {
        cref.active = false;
        cref.retain = false;
      }
    } else if (cmd == "latch") {
      rc = handle_latch(server, cref);
      if (rc == UA_STATUSCODE_GOOD) {
        cref.active = false;
        cref.retain = true;
      }
    } else if (cmd == "ack") {
      rc = handle_ack(server, cref);
      if (rc == UA_STATUSCODE_GOOD) {
        cref.acked = true;
      }
    } else if (cmd == "confirm") {
      rc = handle_confirm(server, cref);
      if (rc == UA_STATUSCODE_GOOD) {
        cref.confirmed = true;
        cref.retain = false;
      }
    } else if (cmd == "shelve") {
      rc = set_shelving(server, cref, true);
      if (rc == UA_STATUSCODE_GOOD) {
        cref.shelved = true;
      }
    } else if (cmd == "unshelve") {
      rc = set_shelving(server, cref, false);
      if (rc == UA_STATUSCODE_GOOD) {
        cref.shelved = false;
      }
    } else if (cmd == "disable") {
      rc = handle_enable(server, cref, false);
      if (rc == UA_STATUSCODE_GOOD) {
        cref.enabled = false;
      }
    } else if (cmd == "enable") {
      rc = handle_enable(server, cref, true);
      if (rc == UA_STATUSCODE_GOOD) {
        cref.enabled = true;
      }
    } else {
      std::cout << "ERR unknown_cmd:" << cmd << std::endl;
      continue;
    }
    if (rc == UA_STATUSCODE_GOOD) {
      std::cout << "OK " << name << std::endl;
      log_state(cref);
    } else {
      std::cout << "ERR " << name << ":" << UA_StatusCode_name(rc) << std::endl;
    }
  }
}

}  // namespace

int main(int argc, char ** argv) {
  signal(SIGINT, stop_handler);
  signal(SIGTERM, stop_handler);

  UA_UInt16 port = 4842;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
      port = static_cast<UA_UInt16>(std::atoi(argv[++i]));
    }
  }

  UA_Server * server = UA_Server_new();
  UA_ServerConfig * config = UA_Server_getConfig(server);
  UA_ServerConfig_setMinimal(config, port, nullptr);

  UA_UInt16 ns = UA_Server_addNamespace(server, NS_URI);

  Condition op, oh, sl;
  if (add_condition(server, "Overpressure", ns, op) != UA_STATUSCODE_GOOD ||
      add_condition(server, "Overheat", ns, oh) != UA_STATUSCODE_GOOD ||
      add_condition(server, "SensorLost", ns, sl) != UA_STATUSCODE_GOOD) {
    UA_LOG_ERROR(UA_Log_Stdout, UA_LOGCATEGORY_USERLAND, "Failed to register conditions");
    UA_Server_delete(server);
    return 1;
  }
  g_conditions[op.name] = op;
  g_conditions[oh.name] = oh;
  g_conditions[sl.name] = sl;

  std::cout << "READY port=" << port << " namespace=" << ns << std::endl;
  std::thread cli(cli_loop, server);

  UA_StatusCode rc = UA_Server_run(server, reinterpret_cast<volatile UA_Boolean *>(&g_running));
  g_running = false;
  if (cli.joinable()) {
    cli.join();
  }
  UA_Server_delete(server);
  return rc == UA_STATUSCODE_GOOD ? 0 : 1;
}
