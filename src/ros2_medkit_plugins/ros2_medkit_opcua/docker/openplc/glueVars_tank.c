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

/* glueVars.c - Generated for tank_demo.st
 * Provides LOCATED variable storage and buffer glue for OpenPLC v4 runtime.
 */

#include "iec_std_lib.h"

#define BUFFER_SIZE 1024

// Buffer declarations (defined in image_tables.c via plcapp_manager)
extern IEC_UINT * int_memory[];
extern IEC_UDINT * dint_memory[];
extern IEC_ULINT * lint_memory[];
extern IEC_BOOL * bool_memory[][8];

// Backing storage for LOCATED variables
// These are the globals that __INIT_LOCATED references via extern
REAL __md0_storage;
REAL __md1_storage;
REAL __md2_storage;
REAL __md3_storage;
REAL __md5_storage;
BOOL __mx16_0_storage;
BOOL __mx24_0_storage;
BOOL __mx24_1_storage;
BOOL __mx24_2_storage;
BOOL __mx24_3_storage;

// Global pointers that POUS.c __INIT_LOCATED expects
REAL * __MD0 = &__md0_storage;
REAL * __MD1 = &__md1_storage;
REAL * __MD2 = &__md2_storage;
REAL * __MD3 = &__md3_storage;
REAL * __MD5 = &__md5_storage;
BOOL * __MX16_0 = &__mx16_0_storage;
BOOL * __MX24_0 = &__mx24_0_storage;
BOOL * __MX24_1 = &__mx24_1_storage;
BOOL * __MX24_2 = &__mx24_2_storage;
BOOL * __MX24_3 = &__mx24_3_storage;

void glueVars() {
  // Map LOCATED variables to runtime memory buffers
  // REAL vars -> dint_memory (32-bit)
  if (dint_memory[0]) {
    __MD0 = (REAL *)dint_memory[0];
  }
  if (dint_memory[1]) {
    __MD1 = (REAL *)dint_memory[1];
  }
  if (dint_memory[2]) {
    __MD2 = (REAL *)dint_memory[2];
  }
  if (dint_memory[3]) {
    __MD3 = (REAL *)dint_memory[3];
  }
  if (dint_memory[5]) {
    __MD5 = (REAL *)dint_memory[5];
  }

  // BOOL vars -> bool_memory
  if (bool_memory[16][0]) {
    __MX16_0 = bool_memory[16][0];
  }
  if (bool_memory[24][0]) {
    __MX24_0 = bool_memory[24][0];
  }
  if (bool_memory[24][1]) {
    __MX24_1 = bool_memory[24][1];
  }
  if (bool_memory[24][2]) {
    __MX24_2 = bool_memory[24][2];
  }
  if (bool_memory[24][3]) {
    __MX24_3 = bool_memory[24][3];
  }
}

void updateTime() {
}
void updateBuffersIn() {
}
void updateBuffersOut() {
}

// --- Required symbols for OpenPLC v4 runtime ---

void setBufferPointers(IEC_BOOL * bi[][8], IEC_BOOL * bo[][8], IEC_BYTE * ib[], IEC_BYTE * ob[], IEC_UINT * ii[],
                       IEC_UINT * io[], IEC_UDINT * di[], IEC_UDINT * do_[], IEC_ULINT * li[], IEC_ULINT * lo[],
                       IEC_UINT * im[], IEC_UDINT * dm[], IEC_ULINT * lm[]) {
  (void)bi;
  (void)bo;
  (void)ib;
  (void)ob;
  (void)ii;
  (void)io;
  (void)di;
  (void)do_;
  (void)li;
  (void)lo;
  (void)im;
  (void)dm;
  (void)lm;
}

void setBufferPointers_v4(IEC_BOOL * bi[][8], IEC_BOOL * bo[][8], IEC_BYTE * ib[], IEC_BYTE * ob[], IEC_UINT * ii[],
                          IEC_UINT * io[], IEC_UDINT * di[], IEC_UDINT * do_[], IEC_ULINT * li[], IEC_ULINT * lo[],
                          IEC_UINT * im[], IEC_UDINT * dm[], IEC_ULINT * lm[], IEC_BOOL * bm[][8]) {
  (void)bi;
  (void)bo;
  (void)ib;
  (void)ob;
  (void)ii;
  (void)io;
  (void)di;
  (void)do_;
  (void)li;
  (void)lo;
  (void)im;
  (void)dm;
  (void)lm;
  (void)bm;
}

const char * plc_program_md5(void) {
  return "tank_demo_v1";
}
void set_endianness(int e) {
  (void)e;
}
int get_var_count(void) {
  return 10;
}
int get_var_size(int idx) {
  return (idx < 6) ? 4 : 1;
}

// Debug/trace symbols required by runtime
void * get_var_addr(int idx) {
  switch (idx) {
    case 0:
      return __MD0;
    case 1:
      return __MD1;
    case 2:
      return __MD2;
    case 3:
      return __MD3;
    case 4:
      return __MD5;
    case 5:
      return __MX16_0;
    case 6:
      return __MX24_0;
    case 7:
      return __MX24_1;
    case 8:
      return __MX24_2;
    case 9:
      return __MX24_3;
    default:
      return (void *)0;
  }
}

void set_trace(int idx, int force, void * val) {
  (void)idx;
  (void)force;
  (void)val;
}
