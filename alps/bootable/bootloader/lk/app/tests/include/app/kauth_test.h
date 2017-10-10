/*
 * Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __APP_KAUTHTEST_H
#define __APP_KAUTHTEST_H

#include <sys/types.h>
#include <stdint.h>

#define IMAGE_SIZE 64*1024

void kauth_test(const char *arg, void *data, unsigned sz);

#if VERIFIED_BOOT
unsigned char correct_signature_signature[] = {
  0xdb, 0x81, 0x5e, 0x7b, 0x78, 0x66, 0xe5, 0x5b, 0x18, 0x5d, 0x97, 0x4b,
  0x95, 0xfc, 0xf2, 0xe0, 0x0f, 0xa5, 0x13, 0x1b, 0x4d, 0xa9, 0x14, 0x87,
  0x15, 0xa4, 0xfc, 0xf8, 0x53, 0xf5, 0xeb, 0xd9, 0x50, 0x60, 0xa0, 0xc2,
  0x31, 0x09, 0xd3, 0x9a, 0xb5, 0x98, 0x5c, 0x08, 0x1d, 0xce, 0x48, 0x9e,
  0x78, 0xde, 0xff, 0x8f, 0x8d, 0x20, 0x26, 0x58, 0xd9, 0x0f, 0xd2, 0x00,
  0x91, 0x3c, 0x4e, 0xc6, 0x5e, 0x40, 0xeb, 0x09, 0x3f, 0xef, 0xbf, 0xaf,
  0x9d, 0xea, 0xd7, 0x35, 0x59, 0x36, 0x1f, 0xa8, 0xf9, 0x01, 0x1e, 0xdf,
  0x7c, 0xf1, 0xb6, 0xaa, 0x85, 0x37, 0xc3, 0x29, 0x35, 0x31, 0xb2, 0xd3,
  0xba, 0xaf, 0xfe, 0xd8, 0x2d, 0xca, 0xab, 0x62, 0x4c, 0x5c, 0x66, 0x87,
  0x76, 0xc5, 0xaf, 0x79, 0x69, 0xf8, 0xd8, 0xbe, 0x47, 0x38, 0x8b, 0x6a,
  0x44, 0xe1, 0x48, 0x87, 0x5a, 0xc6, 0x0d, 0x47, 0xec, 0x8c, 0x3a, 0x23,
  0x38, 0x77, 0xee, 0x13, 0x80, 0x09, 0xd8, 0xe9, 0xc8, 0xf0, 0xc5, 0x39,
  0xcb, 0xc5, 0xc2, 0x47, 0x43, 0x58, 0xd7, 0x47, 0xf6, 0x9f, 0xb6, 0xae,
  0x25, 0x98, 0x27, 0xa8, 0x4f, 0xf2, 0x75, 0x08, 0xa1, 0xf4, 0xf5, 0xf7,
  0xc0, 0x52, 0xe7, 0xab, 0xba, 0x14, 0x1b, 0x64, 0x2d, 0x00, 0x35, 0x1d,
  0x6d, 0xe3, 0x04, 0x57, 0xec, 0x18, 0x3c, 0xce, 0x1e, 0xeb, 0x6a, 0x16,
  0x08, 0x2a, 0xf0, 0x4a, 0xf6, 0x1c, 0xe7, 0x7f, 0xc6, 0x3f, 0x2d, 0x96,
  0x09, 0x1a, 0x69, 0x3d, 0xd4, 0x7d, 0x3d, 0x77, 0xa5, 0xa0, 0x59, 0x91,
  0x14, 0x98, 0x8f, 0x79, 0x51, 0x32, 0x66, 0x0a, 0x31, 0x23, 0xe4, 0xe6,
  0x41, 0x6e, 0xfd, 0x11, 0xc5, 0x91, 0x1d, 0x18, 0x87, 0x14, 0xf8, 0xf5,
  0x53, 0x8e, 0x62, 0x8f, 0x7a, 0xc7, 0x7c, 0x19, 0xe5, 0x65, 0xef, 0xa0,
  0xbc, 0xa5, 0x28, 0x7c
};

unsigned char missing_first_byte_signature[] = {
  0x5e, 0x7f, 0x55, 0x65, 0x67, 0xce, 0xae, 0xce, 0x9a, 0xf6, 0x5d, 0x87,
  0x4c, 0x46, 0xc0, 0x26, 0xc5, 0x57, 0xa7, 0xeb, 0xe8, 0x5d, 0xc4, 0x5b,
  0xc9, 0xed, 0x94, 0xb7, 0x81, 0xc1, 0xdb, 0xed, 0x5f, 0x12, 0x21, 0x8e,
  0x0a, 0x76, 0x00, 0x2a, 0x11, 0xed, 0x20, 0x64, 0xcb, 0xf3, 0x4e, 0x9a,
  0x5f, 0xf2, 0xde, 0xf9, 0xd9, 0xe5, 0x91, 0x10, 0x33, 0x07, 0x2b, 0x92,
  0xd7, 0x53, 0x75, 0x5c, 0xc6, 0x00, 0x13, 0x07, 0x7c, 0x39, 0x6f, 0xf2,
  0xe3, 0xd8, 0x33, 0x82, 0x97, 0xb5, 0xff, 0x79, 0x4e, 0x92, 0x5d, 0xe7,
  0x18, 0x6c, 0x4c, 0xa0, 0x43, 0x64, 0x80, 0xa1, 0xe5, 0xf2, 0x53, 0x8e,
  0x04, 0xa1, 0x16, 0x68, 0x88, 0xca, 0x00, 0x76, 0x8f, 0xff, 0xc8, 0xc8,
  0xf9, 0x1e, 0x1c, 0x43, 0x52, 0x40, 0xa0, 0x5b, 0x8e, 0x4d, 0x5b, 0xa0,
  0x42, 0xf4, 0x28, 0xbe, 0x07, 0x70, 0x3c, 0xcc, 0x2e, 0x61, 0x77, 0xb6,
  0x65, 0x58, 0xca, 0x2d, 0x5e, 0xcb, 0xbd, 0xfa, 0xdc, 0xbb, 0x38, 0xee,
  0x78, 0x01, 0x6d, 0xcd, 0x88, 0x73, 0xa2, 0x0b, 0x18, 0x0d, 0x6f, 0x9a,
  0xdb, 0xcf, 0x15, 0x02, 0xc0, 0xe7, 0xa3, 0xc8, 0x78, 0x62, 0xf5, 0x70,
  0x92, 0x80, 0x68, 0x88, 0x00, 0x9f, 0xcf, 0xdc, 0x52, 0xfc, 0xc4, 0xa3,
  0xd9, 0xae, 0xbe, 0xbd, 0xcd, 0xf4, 0xa4, 0x04, 0x62, 0x42, 0x6c, 0x14,
  0x83, 0x2b, 0x6a, 0x2a, 0x55, 0x3b, 0xb3, 0xe3, 0x1d, 0xdc, 0x5e, 0x18,
  0xa8, 0x76, 0x98, 0x52, 0x92, 0x64, 0xae, 0xb7, 0x65, 0x2d, 0xc4, 0xd8,
  0x14, 0x01, 0xb2, 0x15, 0xfc, 0xcf, 0x2f, 0xf6, 0xb5, 0xe9, 0xb5, 0x07,
  0x43, 0xca, 0x05, 0x90, 0x9c, 0x84, 0x49, 0x8a, 0xd5, 0x5b, 0xb5, 0xe5,
  0x23, 0x9a, 0x62, 0x33, 0xdf, 0x3f, 0xd5, 0xc0, 0xcb, 0x3f, 0x55, 0x0c,
  0xd2, 0xea, 0xf1, 0x1a
};

unsigned char incorrect_first_byte_signature[] = {
  0x24, 0x72, 0x99, 0x43, 0x76, 0x36, 0xb4, 0x00, 0xfa, 0x08, 0x95, 0xf9,
  0xea, 0x12, 0x17, 0x0e, 0x5a, 0x12, 0x05, 0xd7, 0x5e, 0x53, 0x99, 0xbf,
  0x51, 0xec, 0xaa, 0x21, 0x22, 0xd9, 0x6b, 0x2a, 0xb6, 0x2a, 0x8a, 0xd8,
  0xdf, 0x3c, 0xe9, 0xa2, 0x28, 0xdf, 0xbf, 0x41, 0x53, 0x7e, 0x2a, 0x9b,
  0xff, 0xca, 0x63, 0x0e, 0x43, 0xee, 0xae, 0x18, 0x8b, 0x45, 0xa0, 0x8b,
  0xdb, 0x9a, 0x1c, 0xa9, 0xed, 0xf5, 0x25, 0x15, 0x48, 0xb1, 0xac, 0xe6,
  0x09, 0xe5, 0x19, 0xbd, 0xa6, 0xef, 0x07, 0xc4, 0x87, 0x42, 0x79, 0x9c,
  0x69, 0x51, 0xcd, 0x57, 0x99, 0x7c, 0x2d, 0x3c, 0xe2, 0x84, 0x81, 0xbb,
  0xb1, 0xb9, 0xa5, 0xcc, 0x1f, 0xd3, 0xec, 0x34, 0xd1, 0xc8, 0xda, 0x07,
  0x86, 0x7d, 0x52, 0xdc, 0x76, 0x02, 0xfe, 0x3d, 0xb8, 0x8c, 0x79, 0xe4,
  0x43, 0x0b, 0xd3, 0x93, 0xb4, 0x05, 0xbb, 0xf4, 0x00, 0x54, 0x5a, 0x7d,
  0xef, 0x50, 0xdb, 0x88, 0xc1, 0xf4, 0xb5, 0xa4, 0x5a, 0x3d, 0xe3, 0x37,
  0x37, 0x1c, 0xf1, 0x22, 0x81, 0x5a, 0x08, 0xf5, 0xe9, 0x3c, 0xc5, 0xdd,
  0xef, 0x50, 0x65, 0x47, 0xad, 0xb4, 0xfe, 0xaa, 0x51, 0xd5, 0xdc, 0x6e,
  0x0f, 0x35, 0x57, 0x01, 0x52, 0x41, 0x47, 0x07, 0x20, 0xba, 0x6d, 0x1d,
  0x2d, 0x44, 0xd7, 0xb9, 0xff, 0x4a, 0x62, 0xac, 0x38, 0xaa, 0x2e, 0x00,
  0x86, 0x86, 0x5f, 0x59, 0x47, 0x6c, 0xdd, 0x64, 0x10, 0x47, 0x53, 0x6d,
  0x65, 0x43, 0xde, 0x22, 0x3c, 0xa8, 0x4e, 0x4c, 0xa5, 0xad, 0xc8, 0xc7,
  0x69, 0x34, 0xbf, 0x22, 0x52, 0x86, 0x73, 0x8f, 0xe8, 0x21, 0x4e, 0xb9,
  0x74, 0x53, 0xbc, 0x0c, 0x80, 0xcb, 0xe3, 0x04, 0x3f, 0xe7, 0x09, 0x4e,
  0x10, 0x9f, 0x71, 0x15, 0x98, 0x91, 0xf3, 0x01, 0xfe, 0x55, 0xa3, 0x81,
  0x19, 0xb9, 0x9c, 0x34
};

unsigned char missing_second_byte_signature[] = {
  0xbc, 0x1d, 0xa8, 0x10, 0x50, 0x9f, 0x37, 0x1d, 0x5a, 0xb5, 0x9f, 0x79,
  0x13, 0x85, 0x86, 0x7d, 0x09, 0xdf, 0x6c, 0x12, 0xc4, 0x54, 0xe6, 0x63,
  0x71, 0x0d, 0x8b, 0x6d, 0x6a, 0xa0, 0x94, 0xac, 0x3b, 0xb9, 0xcd, 0xa7,
  0x05, 0x78, 0xff, 0x80, 0x71, 0x11, 0x85, 0xd9, 0x59, 0x79, 0xa3, 0x74,
  0xbe, 0xb7, 0x9e, 0xd1, 0x6c, 0x49, 0x7c, 0xf1, 0x70, 0x8b, 0x27, 0x68,
  0xa4, 0x06, 0xda, 0x4f, 0x3b, 0x63, 0x37, 0xcc, 0xdf, 0xe4, 0xe8, 0xc4,
  0x78, 0x19, 0x1d, 0x6d, 0x92, 0xe7, 0x74, 0x9c, 0x0e, 0x54, 0x29, 0x94,
  0x6c, 0x5e, 0x67, 0xbb, 0xf1, 0x6d, 0x1b, 0xc5, 0xa6, 0x8d, 0x43, 0x6b,
  0xd9, 0x46, 0x97, 0xfb, 0xe0, 0xbd, 0x98, 0xa7, 0x83, 0x50, 0x21, 0x4c,
  0xb0, 0xa3, 0x37, 0x9e, 0x4f, 0x56, 0x12, 0x78, 0x4c, 0x6a, 0x91, 0x7b,
  0x9e, 0x54, 0x0d, 0x48, 0x65, 0x17, 0x62, 0xa3, 0xb4, 0xff, 0x5d, 0xc0,
  0xd5, 0xc9, 0x8e, 0x96, 0x58, 0x05, 0x3b, 0x80, 0x94, 0xe9, 0x42, 0xe1,
  0x28, 0x87, 0xf4, 0x87, 0xbe, 0x39, 0x4e, 0x24, 0xe5, 0x6f, 0x3c, 0x4d,
  0x4c, 0x88, 0x3c, 0x64, 0x8f, 0x8e, 0xc7, 0xe8, 0x63, 0x8a, 0x82, 0x9f,
  0xfe, 0xfd, 0x8f, 0x31, 0x84, 0xcb, 0x9a, 0x22, 0x63, 0xe5, 0x91, 0x7f,
  0xb3, 0x68, 0xed, 0x60, 0x7d, 0x14, 0xf7, 0x64, 0x99, 0x29, 0xdd, 0x9d,
  0x49, 0x5f, 0xa5, 0x4a, 0x7f, 0x8f, 0x5c, 0x3a, 0xba, 0x15, 0xc4, 0x13,
  0x07, 0x90, 0xcc, 0x8b, 0x7f, 0x36, 0x47, 0xa2, 0xcb, 0x15, 0x40, 0x26,
  0xf7, 0x34, 0x26, 0xe2, 0x01, 0x0d, 0x9c, 0x02, 0x48, 0x96, 0xb7, 0xe7,
  0x30, 0x04, 0xfe, 0x83, 0x19, 0xc1, 0xac, 0x19, 0x6a, 0x15, 0xdc, 0xab,
  0x0f, 0x67, 0x1d, 0x77, 0xc2, 0xf0, 0xd8, 0x1b, 0x7e, 0x13, 0x88, 0x92,
  0x1a, 0x6d, 0x13, 0x2a
};

unsigned char incorrect_second_byte_signature[] = {
  0xe6, 0x8a, 0xa2, 0xb4, 0xa5, 0xbb, 0x95, 0xb8, 0x74, 0xb1, 0xa6, 0x65,
  0x50, 0xa2, 0xf9, 0x86, 0x47, 0x12, 0x20, 0xbc, 0x7d, 0xe1, 0x68, 0x87,
  0x47, 0xb8, 0xf9, 0x77, 0x14, 0xb5, 0x5e, 0xe8, 0x40, 0x05, 0xe1, 0xb4,
  0x0f, 0xc4, 0xec, 0x0c, 0xcc, 0xe3, 0xfd, 0x60, 0xcc, 0x8b, 0x87, 0x1f,
  0xcb, 0xd3, 0xdb, 0x49, 0xf0, 0x43, 0x5c, 0x14, 0xed, 0x76, 0xc1, 0x5a,
  0x42, 0x2b, 0x17, 0x30, 0xe4, 0x78, 0xc9, 0xc8, 0x77, 0xb4, 0x9e, 0xee,
  0xa2, 0xd3, 0x49, 0x68, 0xff, 0xe2, 0xa7, 0xeb, 0xef, 0x78, 0x1c, 0x63,
  0x88, 0x38, 0xd6, 0x78, 0x17, 0xb8, 0x97, 0x17, 0x99, 0x9d, 0x59, 0xfd,
  0xfd, 0x4f, 0xff, 0xe9, 0xb8, 0x97, 0xc9, 0xa7, 0xa5, 0x03, 0x32, 0x8d,
  0x8f, 0xdd, 0x2d, 0xf4, 0xcf, 0x0d, 0xd1, 0x17, 0xa3, 0xa6, 0x13, 0x85,
  0x67, 0xd1, 0x9d, 0x84, 0xd5, 0xf2, 0x3d, 0x1b, 0xc3, 0x7c, 0xf2, 0xa7,
  0x55, 0x9c, 0x75, 0xbf, 0xc4, 0x2a, 0x87, 0xa4, 0xf4, 0xf8, 0xfc, 0xba,
  0x9f, 0x6b, 0x6d, 0xc9, 0xc4, 0x50, 0x39, 0x7d, 0x72, 0xb2, 0x35, 0xbf,
  0xee, 0x6e, 0xe8, 0xf8, 0x8b, 0xd8, 0xdf, 0x06, 0x4c, 0xe5, 0xb3, 0x3f,
  0x8e, 0x0b, 0x0b, 0xdb, 0x99, 0x4a, 0x30, 0xfc, 0x63, 0x03, 0xd6, 0xa6,
  0x1d, 0x3e, 0x96, 0x71, 0x90, 0xa1, 0x49, 0xaf, 0x50, 0xf3, 0x95, 0xd7,
  0x37, 0x01, 0xf9, 0xfb, 0xad, 0x21, 0x13, 0xdb, 0xdc, 0x28, 0x73, 0xdc,
  0x1e, 0x0f, 0x81, 0xc7, 0xda, 0xf0, 0x4b, 0x59, 0x4c, 0x5d, 0x04, 0x1b,
  0x79, 0x03, 0x20, 0x91, 0x43, 0x3f, 0xc7, 0x3c, 0xe1, 0xa2, 0xa9, 0x39,
  0x15, 0xc1, 0xbe, 0x82, 0x40, 0xb4, 0xf1, 0x3b, 0x5b, 0x02, 0xae, 0x87,
  0x82, 0x60, 0xf6, 0x8f, 0x16, 0x21, 0x29, 0xac, 0xe2, 0x87, 0x06, 0xf0,
  0xd2, 0x74, 0xbd, 0x5f
};

unsigned char incorrect_ff_padding_signature[] = {
  0x91, 0x78, 0xcc, 0x63, 0xc7, 0x03, 0x63, 0xe7, 0xe9, 0x7e, 0x0f, 0xe5,
  0xe1, 0x75, 0x15, 0x9f, 0x17, 0xff, 0x5f, 0x2b, 0x91, 0xe9, 0x0d, 0x94,
  0x02, 0xe4, 0x57, 0x41, 0x5e, 0x98, 0x81, 0xb5, 0x01, 0x43, 0x40, 0x02,
  0x45, 0xbe, 0xc8, 0x67, 0x51, 0xfd, 0xd8, 0x49, 0xa8, 0xaf, 0x20, 0x93,
  0xc2, 0xae, 0xf6, 0x06, 0xef, 0xb0, 0x8d, 0xc1, 0xca, 0xf2, 0x6f, 0x93,
  0x09, 0xa1, 0xae, 0xf4, 0xeb, 0xc6, 0xa8, 0xc7, 0x37, 0xee, 0xeb, 0xd1,
  0xe8, 0xf9, 0xcf, 0xf1, 0x34, 0x26, 0xf9, 0xc8, 0x50, 0x98, 0x65, 0x97,
  0x75, 0x06, 0xcd, 0xf6, 0x91, 0x18, 0xbd, 0x5d, 0xc9, 0x35, 0xc6, 0xa4,
  0x07, 0xc2, 0x88, 0x03, 0x7a, 0xf9, 0x81, 0x2c, 0x5d, 0xd3, 0x86, 0xb4,
  0xd1, 0x63, 0x78, 0x38, 0x18, 0xdd, 0xb4, 0x93, 0xcd, 0xe6, 0x65, 0x22,
  0x61, 0xd3, 0xdd, 0xf1, 0x74, 0x3c, 0xd6, 0x4f, 0x5e, 0xd9, 0xfc, 0x16,
  0x98, 0xbf, 0x67, 0xbb, 0x77, 0xcb, 0x56, 0xdc, 0x34, 0x31, 0xf9, 0x34,
  0x12, 0xde, 0x69, 0x40, 0x0d, 0x57, 0x10, 0xec, 0x81, 0x91, 0x07, 0xda,
  0x88, 0x9c, 0xb9, 0x32, 0x05, 0x73, 0x9a, 0x4b, 0xa8, 0xaf, 0x61, 0x75,
  0x56, 0x43, 0xfc, 0x17, 0x3a, 0x0e, 0xf3, 0x0f, 0x87, 0x69, 0x0f, 0x86,
  0x40, 0x2c, 0x15, 0x17, 0x0a, 0x21, 0xbc, 0x1d, 0x15, 0x96, 0xe9, 0x35,
  0x3c, 0x9d, 0x0d, 0x27, 0x1a, 0xb0, 0x54, 0x41, 0x2c, 0x79, 0xd6, 0xd8,
  0xc9, 0x55, 0x94, 0x4b, 0xc9, 0x2e, 0x03, 0xca, 0x68, 0x37, 0xb8, 0xcc,
  0x69, 0x33, 0xec, 0x43, 0xfb, 0x3e, 0xbb, 0x7c, 0x00, 0xc2, 0xc1, 0xb3,
  0xd6, 0x6c, 0x09, 0x11, 0xcc, 0xd7, 0x76, 0x90, 0xcd, 0xdc, 0x5a, 0x9e,
  0xed, 0x49, 0xa6, 0xff, 0x98, 0x82, 0xdf, 0xce, 0x91, 0xf1, 0xc5, 0xfd,
  0xae, 0x44, 0xb0, 0x6e
};

unsigned char missing_delimiter_signature[] = {
  0x6b, 0xbe, 0x2d, 0xda, 0xfe, 0xae, 0x7d, 0xa4, 0x98, 0xb8, 0x8a, 0xcc,
  0x5c, 0x6b, 0xc4, 0xa6, 0x43, 0x33, 0x46, 0xfe, 0x84, 0x68, 0xde, 0xa8,
  0x1f, 0x63, 0x7d, 0x98, 0x7e, 0x36, 0x75, 0x33, 0xba, 0x70, 0x26, 0x1b,
  0x7c, 0xbd, 0x12, 0xfa, 0x4d, 0x3c, 0x1b, 0x9c, 0xfc, 0x1b, 0x75, 0xf2,
  0xc5, 0xbf, 0xea, 0xc2, 0xf6, 0x92, 0xe5, 0xd9, 0x8e, 0xec, 0x81, 0x9d,
  0x6b, 0x43, 0x9a, 0x2e, 0x9a, 0x7a, 0xfb, 0x9a, 0x76, 0xd4, 0xcc, 0x94,
  0xd0, 0xf5, 0x14, 0x0e, 0xab, 0xc9, 0xab, 0x5a, 0xf0, 0x49, 0x4d, 0xdb,
  0x82, 0xc1, 0xca, 0x2a, 0xe6, 0x36, 0xa8, 0x4e, 0x34, 0x6d, 0xa9, 0x81,
  0xf9, 0xad, 0x65, 0xbe, 0xd0, 0xad, 0x9a, 0xac, 0x76, 0x25, 0xfc, 0x93,
  0x8c, 0x26, 0x90, 0x23, 0x2b, 0x06, 0xee, 0xde, 0x52, 0x85, 0x63, 0xf2,
  0xee, 0xd0, 0x12, 0x29, 0x6e, 0xce, 0xfb, 0x49, 0x5f, 0x68, 0xd9, 0xa0,
  0xfd, 0x5c, 0x8c, 0x95, 0x6b, 0x06, 0x6c, 0x41, 0x8b, 0xb5, 0x93, 0xca,
  0xbe, 0x1c, 0xc3, 0x42, 0x57, 0xed, 0x11, 0xf9, 0x6c, 0x3e, 0x0f, 0xff,
  0xf2, 0xf7, 0x3c, 0x91, 0x5a, 0x64, 0xcc, 0x48, 0x4a, 0x03, 0x46, 0xb1,
  0x31, 0xe8, 0xdd, 0xcb, 0x84, 0xdb, 0x35, 0x7d, 0xbf, 0x59, 0x53, 0xeb,
  0x58, 0xa7, 0xa0, 0x39, 0x5c, 0xc9, 0x0a, 0x84, 0x42, 0x2b, 0xdd, 0xfc,
  0xcb, 0x39, 0x19, 0xfa, 0xa0, 0x43, 0x8a, 0x85, 0xa4, 0x9f, 0x5d, 0x73,
  0x88, 0xc5, 0xb3, 0xab, 0x03, 0x3e, 0x8b, 0xdb, 0x71, 0x4c, 0x7d, 0xd5,
  0xf4, 0xe5, 0xd7, 0xa4, 0xff, 0x43, 0xb4, 0x88, 0xd2, 0xb7, 0x97, 0xe0,
  0x1f, 0xb5, 0x2a, 0x51, 0x62, 0x4d, 0x70, 0xe1, 0x5e, 0x54, 0x53, 0x38,
  0x48, 0xde, 0xc7, 0xb2, 0x4a, 0x1b, 0xaa, 0x86, 0x9c, 0x48, 0x1a, 0xfa,
  0x41, 0x33, 0x39, 0x5c
};

unsigned char incorrect_delimiter_signature[] = {
  0xd7, 0xe1, 0x8e, 0x27, 0xf9, 0xae, 0xfd, 0xbf, 0xf4, 0x0e, 0x2f, 0xa5,
  0x8e, 0xc8, 0x5d, 0x45, 0x5f, 0xfa, 0x49, 0x7e, 0xab, 0xe8, 0x00, 0xe2,
  0x8a, 0x71, 0xcc, 0xb3, 0x88, 0x20, 0xaa, 0x32, 0x68, 0x5e, 0x70, 0x9f,
  0x78, 0xb4, 0xcc, 0x13, 0x83, 0x11, 0xc6, 0x7f, 0xf1, 0xde, 0x83, 0xb2,
  0x06, 0x3b, 0xe1, 0xfe, 0xa7, 0xde, 0xb1, 0xcd, 0xce, 0x9e, 0xd6, 0x25,
  0xcf, 0x56, 0x26, 0xe7, 0x52, 0xf2, 0x8e, 0x11, 0xe9, 0xec, 0xdd, 0x80,
  0xa7, 0xc2, 0x8d, 0x57, 0x43, 0x26, 0x56, 0xa2, 0x2c, 0x82, 0x12, 0xf7,
  0x4c, 0xb1, 0x30, 0x16, 0x8c, 0xaf, 0x39, 0xfd, 0x98, 0xe6, 0xca, 0x07,
  0x4d, 0xe0, 0x08, 0xe0, 0x67, 0x9b, 0x3b, 0x20, 0x8f, 0x0b, 0x44, 0xd4,
  0x28, 0x1e, 0xb4, 0x08, 0x4f, 0x1c, 0x8e, 0x72, 0x1c, 0x5f, 0x2d, 0xea,
  0x6e, 0xdf, 0x7a, 0x04, 0x68, 0x79, 0x54, 0x60, 0x68, 0xb5, 0x4e, 0xb4,
  0x07, 0x63, 0xa5, 0x5d, 0x57, 0xcc, 0x6b, 0x1d, 0xcd, 0x9c, 0xa6, 0xf0,
  0xc1, 0x09, 0xc7, 0xdf, 0x89, 0xef, 0x47, 0x48, 0x8b, 0x03, 0xc7, 0x34,
  0x79, 0xa7, 0x37, 0x73, 0x35, 0xee, 0x70, 0x9c, 0x53, 0x96, 0xab, 0xb3,
  0x4f, 0xb4, 0x85, 0x50, 0xda, 0x30, 0x97, 0x8d, 0xf6, 0xe0, 0x6c, 0x10,
  0xba, 0x51, 0xa3, 0x62, 0xbf, 0xeb, 0x64, 0x6c, 0x44, 0x6f, 0x98, 0x0a,
  0xf4, 0x6a, 0x07, 0xdc, 0xc5, 0xde, 0x72, 0xe5, 0xb7, 0xb5, 0xbd, 0x53,
  0x80, 0x72, 0xd0, 0x91, 0x22, 0x96, 0x40, 0xc9, 0xc9, 0x62, 0x9a, 0xc3,
  0x2c, 0x81, 0x9c, 0xcc, 0xf6, 0x9b, 0x68, 0xcc, 0x71, 0x28, 0x56, 0xee,
  0xd0, 0x80, 0xa8, 0xbe, 0xdf, 0x5a, 0x31, 0x56, 0x69, 0x73, 0x2f, 0xfc,
  0xe9, 0xeb, 0xa8, 0x48, 0xa0, 0x72, 0x87, 0xbc, 0x74, 0x0c, 0x1e, 0x13,
  0x45, 0x48, 0x7b, 0x24
};

unsigned char bad_oid_signature[] = {
  0xe3, 0x63, 0x2c, 0x68, 0x6b, 0x44, 0x2b, 0x07, 0x07, 0xe2, 0x69, 0x24,
  0x95, 0x7e, 0xb4, 0x86, 0x11, 0x03, 0xd3, 0x2b, 0xec, 0x70, 0xf4, 0xa6,
  0xdc, 0xcf, 0xb6, 0xa1, 0x27, 0xcf, 0x10, 0xd0, 0xa5, 0xae, 0x50, 0xf2,
  0x14, 0x93, 0xba, 0x66, 0x39, 0x81, 0xfd, 0x05, 0xb6, 0xff, 0xd5, 0xcb,
  0x46, 0x15, 0xca, 0x9d, 0x9f, 0x74, 0xeb, 0x0d, 0xe9, 0x63, 0xd1, 0x0a,
  0x6c, 0x71, 0xbd, 0x34, 0x29, 0xea, 0xbb, 0x45, 0x97, 0xe5, 0x53, 0x62,
  0x3a, 0xb6, 0xa5, 0xf2, 0x7f, 0x66, 0xa5, 0x9c, 0x15, 0x18, 0xd1, 0x41,
  0x73, 0xf8, 0xda, 0x13, 0x13, 0xff, 0xf6, 0x72, 0x37, 0x97, 0x32, 0x19,
  0xd6, 0x08, 0xac, 0xf2, 0x4a, 0x79, 0xf3, 0x97, 0x70, 0x6b, 0x05, 0xbb,
  0x6a, 0x7c, 0xc9, 0xf4, 0xd7, 0x03, 0xbb, 0x93, 0x8e, 0xcc, 0xca, 0xda,
  0x38, 0x49, 0x52, 0xb7, 0x11, 0xe9, 0x60, 0xda, 0xf5, 0x25, 0xd1, 0x62,
  0x2a, 0x5e, 0x69, 0xcd, 0x32, 0x41, 0x4f, 0x5b, 0x7e, 0x0b, 0x2e, 0xd0,
  0xd8, 0xf0, 0xd1, 0xd7, 0xf6, 0x3a, 0xee, 0x18, 0x39, 0xb8, 0x97, 0xe6,
  0x5e, 0x07, 0x7f, 0xa8, 0xcb, 0xa0, 0x15, 0xa2, 0xce, 0xce, 0x48, 0xae,
  0x42, 0xdc, 0xb5, 0xf6, 0x43, 0x8c, 0xc7, 0xb6, 0x66, 0xee, 0x45, 0x44,
  0x18, 0xbd, 0x1a, 0x1a, 0x4a, 0x41, 0xfe, 0xff, 0xc5, 0x3f, 0x94, 0x2e,
  0xf3, 0xf7, 0x98, 0xd7, 0x1a, 0x1d, 0xd7, 0xc1, 0xe3, 0xd8, 0xec, 0x64,
  0x57, 0x3b, 0x05, 0x16, 0x99, 0x8c, 0x36, 0x7a, 0x0f, 0xc9, 0x0b, 0x10,
  0xae, 0x89, 0xb7, 0x37, 0xca, 0xf3, 0x3a, 0x54, 0x03, 0xc1, 0x7a, 0x92,
  0x68, 0x1f, 0xe1, 0xf5, 0xcf, 0xc5, 0x7f, 0x44, 0x83, 0xa4, 0xbd, 0x62,
  0x00, 0x67, 0x73, 0x8e, 0xfd, 0xbf, 0xe5, 0xc3, 0xb4, 0x0e, 0xcc, 0xcb,
  0x1b, 0xb4, 0xa2, 0x39
};

unsigned char bad_hash_value_signature[] = {
  0x3e, 0x78, 0x12, 0xac, 0x4b, 0x07, 0xfd, 0xcc, 0x1b, 0x1f, 0x82, 0x76,
  0x49, 0x52, 0xae, 0x5f, 0x48, 0x74, 0xe8, 0x1b, 0xbc, 0xa2, 0x94, 0x87,
  0x9c, 0x06, 0xbb, 0x48, 0x78, 0x63, 0x69, 0x80, 0x7a, 0x48, 0xa0, 0x34,
  0xab, 0x5f, 0x44, 0x2b, 0xd2, 0x3a, 0x55, 0x58, 0x0f, 0x97, 0xbc, 0x18,
  0x74, 0x38, 0x78, 0x51, 0x47, 0x12, 0x45, 0xbe, 0xe0, 0x06, 0xb5, 0xc3,
  0x46, 0xfd, 0xae, 0xbd, 0x66, 0x25, 0xd7, 0x36, 0x94, 0x3f, 0xda, 0x69,
  0x21, 0xd0, 0x94, 0x81, 0x54, 0xb4, 0xf3, 0x45, 0x31, 0x93, 0x3c, 0x69,
  0x02, 0xc0, 0xb4, 0x38, 0xf5, 0xff, 0xa0, 0x63, 0x87, 0xee, 0x32, 0x10,
  0x57, 0xfa, 0xe6, 0x12, 0xad, 0xb8, 0xaa, 0x37, 0xec, 0x88, 0x4a, 0xb3,
  0xfd, 0x52, 0x4d, 0x91, 0xe8, 0x17, 0x19, 0x39, 0xc7, 0xdf, 0x43, 0x81,
  0x62, 0x08, 0x12, 0xaa, 0xa1, 0x08, 0xd9, 0xc4, 0xdf, 0x5b, 0x8d, 0xcb,
  0x60, 0x0d, 0x34, 0x7d, 0x4e, 0xd7, 0xda, 0xc9, 0x84, 0xe8, 0x36, 0xbf,
  0x2f, 0x7a, 0xa6, 0x98, 0xd4, 0x9c, 0x8d, 0x65, 0x34, 0x66, 0xcf, 0x7e,
  0x57, 0x7e, 0x32, 0xb8, 0x70, 0xca, 0x1d, 0xf6, 0x69, 0x0c, 0xb9, 0x49,
  0x33, 0x68, 0xba, 0x29, 0x50, 0x48, 0xc0, 0xe1, 0xf9, 0x6f, 0x1d, 0x48,
  0x48, 0x5c, 0x99, 0xf2, 0xbb, 0xd1, 0x2c, 0x95, 0xcd, 0xf2, 0xe9, 0x0c,
  0x2a, 0x2e, 0x60, 0x01, 0xbb, 0x2e, 0xf2, 0x6d, 0x4e, 0xd6, 0x03, 0x7b,
  0xf5, 0xb1, 0x65, 0x34, 0xb8, 0xdd, 0x60, 0x10, 0xa6, 0x32, 0xe1, 0x50,
  0x5c, 0x27, 0x72, 0x03, 0x74, 0x7b, 0x98, 0x13, 0x86, 0xbc, 0xdc, 0x21,
  0xd1, 0x9d, 0xb1, 0x4f, 0xaf, 0x51, 0xaf, 0xda, 0xc6, 0xfc, 0x24, 0x92,
  0x4e, 0x96, 0xa7, 0xc3, 0x72, 0x77, 0x66, 0x62, 0xd4, 0xf7, 0x3e, 0x9b,
  0x36, 0x5f, 0xcf, 0x32
};

unsigned char Bleichenbacher_at_start_signature[] = {
  0x80, 0xfb, 0xe0, 0xd9, 0xcf, 0x90, 0x9f, 0xd1, 0xc9, 0x3c, 0xd5, 0xad,
  0x7e, 0x79, 0xaa, 0xab, 0x47, 0x35, 0x9c, 0xce, 0x52, 0xac, 0xc1, 0x8f,
  0xfc, 0x0a, 0xd6, 0x61, 0xee, 0x92, 0xf7, 0xcc, 0x25, 0xdc, 0x64, 0x65,
  0x21, 0xab, 0xab, 0x6c, 0x64, 0x18, 0x78, 0x6e, 0xdf, 0xef, 0x5e, 0xf3,
  0xe0, 0xb9, 0x2e, 0xf9, 0x07, 0xb3, 0x68, 0x65, 0x67, 0x1c, 0xca, 0xfd,
  0xb0, 0x71, 0xee, 0x6c, 0xbc, 0xb3, 0x12, 0xa6, 0x32, 0x48, 0x29, 0xb8,
  0xd0, 0xc4, 0x53, 0xf4, 0xc6, 0x5e, 0x4c, 0xa8, 0x66, 0x87, 0x69, 0xce,
  0xcd, 0x7d, 0x21, 0x8b, 0x50, 0x21, 0x22, 0xec, 0x06, 0xfc, 0xe9, 0x78,
  0x6f, 0x7b, 0xc5, 0xbb, 0xd7, 0x7b, 0xe4, 0xf3, 0x3a, 0xd3, 0xd4, 0x68,
  0xa8, 0xb3, 0x5f, 0xea, 0xdb, 0xa9, 0xa2, 0x9b, 0x5c, 0x87, 0xce, 0x4f,
  0x0b, 0x33, 0x16, 0x5a, 0xe3, 0x0e, 0x02, 0xe1, 0xff, 0x79, 0x11, 0xd4,
  0x5c, 0xa0, 0x4c, 0x6b, 0xf6, 0xcf, 0x81, 0x2c, 0x0a, 0xef, 0x80, 0x2f,
  0x96, 0xeb, 0xe2, 0x93, 0xdb, 0x4c, 0x97, 0x55, 0x0f, 0xf9, 0x90, 0x02,
  0x61, 0x60, 0xdb, 0xf9, 0xd4, 0x5a, 0x8c, 0xbb, 0x3f, 0x28, 0x97, 0xda,
  0x29, 0x69, 0xfb, 0xfc, 0x7f, 0xb5, 0x1d, 0xcd, 0xbc, 0x1a, 0xad, 0xbb,
  0xce, 0x7f, 0xcb, 0xd5, 0x4d, 0x2a, 0xd6, 0xb3, 0x2b, 0xbf, 0x1d, 0x0c,
  0x5d, 0xfd, 0x83, 0x07, 0xfe, 0x08, 0x52, 0x13, 0xc1, 0x51, 0xfe, 0xec,
  0x9f, 0x02, 0x4f, 0x96, 0x79, 0xe7, 0x22, 0x9a, 0xef, 0x64, 0x2a, 0x3c,
  0xef, 0xe9, 0xd7, 0x39, 0xad, 0x38, 0xec, 0x0d, 0x82, 0xd7, 0x34, 0xcc,
  0x32, 0x94, 0x8a, 0xf9, 0xfa, 0xc9, 0xb4, 0x05, 0x35, 0xf5, 0xe7, 0x4c,
  0x7f, 0x9c, 0xe3, 0x1c, 0x46, 0x54, 0x91, 0xd0, 0x52, 0xf8, 0xb1, 0x54,
  0xfe, 0xd1, 0xb5, 0xec
};

unsigned char Bleichenbacher_at_end_signature[] = {
  0xe5, 0xbe, 0x82, 0xbd, 0x24, 0x6c, 0xd9, 0x6b, 0xd5, 0xb5, 0xaa, 0x41,
  0x24, 0x70, 0x89, 0xe0, 0x6e, 0xd7, 0x3c, 0xb1, 0x38, 0x98, 0x51, 0x5c,
  0xed, 0xa4, 0x27, 0xd8, 0xe4, 0x2f, 0xd4, 0x0a, 0x3d, 0x14, 0x30, 0x61,
  0x99, 0x6c, 0xc4, 0xf3, 0x6f, 0x94, 0x4c, 0x66, 0xee, 0x60, 0x99, 0x9d,
  0xfa, 0xea, 0xaf, 0x1d, 0xd9, 0xe3, 0xa1, 0xaa, 0x51, 0xfa, 0xde, 0xbc,
  0x35, 0xd1, 0xd3, 0xd2, 0xfc, 0x37, 0x8a, 0xd1, 0x79, 0x80, 0x8d, 0x91,
  0x69, 0xca, 0xc5, 0xf7, 0x85, 0xe3, 0x19, 0xf0, 0x8b, 0xb5, 0xcb, 0x9f,
  0x61, 0xba, 0xee, 0xb7, 0xba, 0x5a, 0xfd, 0xc5, 0x78, 0x7b, 0x47, 0x6a,
  0xe5, 0x8c, 0x8c, 0xa5, 0x1f, 0x8c, 0x5c, 0x63, 0x9a, 0x38, 0x17, 0xfc,
  0x09, 0x0d, 0x87, 0x94, 0x7a, 0xe1, 0xfd, 0x26, 0xd0, 0xee, 0xa5, 0xe2,
  0x34, 0x83, 0xc8, 0xe9, 0xa2, 0x83, 0x98, 0x18, 0x83, 0xab, 0xe7, 0xbf,
  0x52, 0x8d, 0xa9, 0xf7, 0x0d, 0x9a, 0x3a, 0x0f, 0x63, 0x83, 0x29, 0x9c,
  0xa9, 0xfb, 0x52, 0x38, 0x96, 0xa4, 0x1a, 0x50, 0x07, 0x16, 0x68, 0x6a,
  0x84, 0x5b, 0x90, 0x31, 0x65, 0xea, 0xee, 0xc0, 0x9c, 0x57, 0xc3, 0x48,
  0x3a, 0xcd, 0x1d, 0xa9, 0x1f, 0x59, 0x54, 0x4f, 0x85, 0x92, 0x78, 0x44,
  0xb6, 0x93, 0x0a, 0xd0, 0xe2, 0x78, 0xd9, 0x92, 0xe6, 0x3b, 0x69, 0x32,
  0x52, 0x59, 0x8e, 0x1d, 0x6e, 0xd9, 0x0f, 0xa9, 0x81, 0x70, 0x21, 0x29,
  0xce, 0x2e, 0xce, 0xb3, 0xef, 0xe1, 0x12, 0x81, 0xa1, 0x7c, 0x1d, 0x4c,
  0xad, 0x5e, 0x14, 0x20, 0x2c, 0x4b, 0x7f, 0x37, 0xea, 0xff, 0x84, 0x84,
  0xf5, 0x97, 0x16, 0xc1, 0x96, 0xeb, 0xba, 0x84, 0x36, 0x7e, 0xc0, 0xa5,
  0x11, 0x13, 0xa9, 0x88, 0x88, 0xe5, 0xf8, 0x56, 0x88, 0xaa, 0x47, 0x83,
  0xbb, 0x36, 0x5b, 0x62
};

unsigned char *vboot_signatures[] =
{
	correct_signature_signature,
	missing_first_byte_signature,
	incorrect_first_byte_signature,
	missing_second_byte_signature,
	incorrect_second_byte_signature,
	incorrect_ff_padding_signature,
	missing_delimiter_signature,
	incorrect_delimiter_signature,
	bad_oid_signature,
	bad_hash_value_signature,
	Bleichenbacher_at_start_signature,
	Bleichenbacher_at_end_signature,
};

const char *vboot_signatures_str[] =
{
	"correct_signature_signature",
	"missing_first_byte_signature",
	"incorrect_first_byte_signature",
	"missing_second_byte_signature",
	"incorrect_second_byte_signature",
	"incorrect_ff_padding_signature",
	"missing_delimiter_signature",
	"incorrect_delimiter_signature",
	"bad_oid_signature",
	"bad_hash_value_signature",
	"Bleichenbacher_at_start_signature",
	"Bleichenbacher_at_end_signature",
};
#endif

#endif
