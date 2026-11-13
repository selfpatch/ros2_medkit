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

// Runs the SQL on stdin against the PostgreSQL server of the connection string in argv[1].
// A launch test uses it to write and drop tables, since the test has no PostgreSQL client of its own.

#include <pqxx/pqxx>

#include <iostream>
#include <iterator>
#include <string>

int main(int argc, char ** argv) {
  if (argc != 2) {
    std::cerr << "usage: pg_exec <connection string> < statements.sql\n";
    return 2;
  }
  try {
    const std::string sql((std::istreambuf_iterator<char>(std::cin)), std::istreambuf_iterator<char>());
    pqxx::connection connection(argv[1]);
    pqxx::nontransaction tx(connection);
    tx.exec(sql);
  } catch (const std::exception & e) {
    std::cerr << "pg_exec failed: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
