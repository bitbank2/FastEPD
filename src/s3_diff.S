//
// SPDX-FileCopyrightText: 2026 Larry Bank <bitbank@pobox.com>
// SPDX-License-Identifier: Apache-2.0
//
// ESP32S3 SIMD optimized eink functions
//
// Copyright 2026 BitBank Software, Inc. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//    http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//===========================================================================
//
#if defined (ARDUINO_ARCH_ESP32) && !defined(NO_SIMD)
#if __has_include ("dsps_fft2r_platform.h")
#include "dsps_fft2r_platform.h"
#if (dsps_fft2r_sc16_aes3_enabled == 1)
	.text
	.align 4

    .global s3_prep_diff
    .type   s3_prep_diff,@function
//
// Convert a row of 1-bit pixels (0 = black, 1 = white)
// into the 2-bit push codes for parallel eink panels to push only black pixels
// 00 = skip, 01 = push black, 10 = push white
// The source pointer is assumed to be unaligned and
// the destination pointer is assumed to be 16-byte aligned
// The bit widening algorithm implemented is the following:
//
//  x = (x | (x << 8)) & 0x00FF00FF;
//  x = (x | (x << 4)) & 0x0F0F0F0F;
//  x = (x | (x << 2)) & 0x33333333;
//  x = (x | (x << 1)) & 0x55555555;
//
//                                       A2            A3            A4            A5
// Call as void s3_prep_diff(uint8_t *pCurr, uint8_t *pPrev, uint8_t *pDest, int iWidth);
  s3_prep_diff:
    .align 4
    entry    a1,16
# create bit masks for the widening of the image data
  movi.n a6,0x0f
  s8i a6,a4,0
  movi.n a6,0x33
  s8i a6,a4,1
  movi.n a6,0x55
  s8i a6,a4,2
  ee.vldbc.8.ip q3,a4,1   # load the masks into Q registers
  ee.vldbc.8.ip q4,a4,1
  ee.vldbc.8.ip q5,a4,1
  addi.n a4,a4,-3

  srli a5,a5,6                # number of 8-byte blocks to process (width/64)
  ee.vcmp.eq.s8 q6,q6,q6      # form a register of 0xffff...
  ee.xorq q1,q1,q1            # and a register of 0's
  ee.vzip.8 q6,q1             # create a mask of 0x00ff00ff...

  loopnez a5, .big_chunk_loop
  l32i a6,a2,0                # load 64 current pixels
  l32i a7,a2,4
  addi.n a2,a2,8
  ee.xorq q2,q2,q2
  ee.movi.32.q q0,a6,0        # pack them into q0
  ee.movi.32.q q0,a7,1
  ee.vzip.16 q0,q2            # expand each group of 16 bits
  l32i a8,a3,0                # load 64 previous pixels
  l32i a9,a3,4
  s32i a6,a3,0                # store current->previous
  s32i a7,a3,4
  addi.n a3,a3,8
  ee.xorq q2,q2,q2
  ee.movi.32.q q1,a8,0        # pack previous pixels into q1
  ee.movi.32.q q1,a9,1
  ee.vzip.16 q1,q2            # expand each group of 16 bits
  ee.notq q7,q0               # ~cur
  ee.andq q7,q7,q1            # diffb = prev & ~cur
  ee.notq q1,q1
  ee.andq q0,q0,q1            # diffw = ~prev & cur
  ee.xorq q1,q1,q1            # a register of 0's
  ee.vzip.16 q0,q1            # interleave each word with 0x0000

  movi.n a6,8
  wsr.sar a6                  # set shift amount to 8
  ee.vsl.32 q1,q0
  ee.orq q0,q0,q1
  ee.andq q0,q0,q6            # mask with 0x00ff00ff...

  ee.vsl.32 q1,q7
  ee.orq q7,q7,q1
  ee.andq q7,q7,q6

  movi.n a6,4
  wsr.sar a6                  # set shift amount to 4
  ee.vsl.32 q1,q0             # shift everything left 4 bits
  ee.orq q0,q0,q1             # combine results
  ee.andq q0,q0,q3            # mask with 0x0f0f...

  ee.vsl.32 q1,q7
  ee.orq q7,q7,q1
  ee.andq q7,q7,q3

  movi.n a6,2
  wsr.sar a6                  # set shift amount to 2
  ee.vsl.32 q1,q0             # shift left 2
  ee.orq q0,q0,q1             # combine results
  ee.andq q0,q0,q4            # mask with 0x3333...

  ee.vsl.32 q1,q7
  ee.orq q7,q7,q1
  ee.andq q7,q7,q4

  movi.n a6,1
  wsr.sar a6                  # set shift amount to 1
  ee.vsl.32 q1,q0             # shift left 1
  ee.orq q0,q0,q1             # combine results
  ee.andq q0,q0,q5            # mask with 0x5555...
  ee.vsl.32 q0,q0             # shift white pushes 1 more bit to be 10 10 10

  ee.vsl.32 q1,q7
  ee.orq q7,q7,q1
  ee.andq q7,q7,q5

  ee.orq q0,q0,q7             # combine black and white pushes

# The pixel deltas have now been transformed into data capable of properly
# doing a partial/differential update.

  mv.qr q1,q0                 # duplicate the output
  ee.vunzip.8 q0,q1           # interleave even/odd bytes
  ee.vzip.8 q1,q0             # reverse the byte order
  ee.movi.32.a q0,a6,0        # assume dest pointer is unaligned
  s32i a6,a4,0
  ee.movi.32.a q0,a6,1
  s32i a6,a4,4
  ee.movi.32.a q0,a6,2
  s32i a6,a4,8
  ee.movi.32.a q0,a6,3
  s32i a6,a4,12
  addi.n a4,a4,16 
#  ee.vst.128.ip q0,a4,16
  .big_chunk_loop:
  retw.n                      # length is a multiple of 16
#endif // dsps_fft2r_sc16_aes3_enabled
#endif // __has_include
#endif // ESP32
