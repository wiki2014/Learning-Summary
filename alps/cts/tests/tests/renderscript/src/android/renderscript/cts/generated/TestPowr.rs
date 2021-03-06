/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Don't edit this file!  It is auto-generated by frameworks/rs/api/generate.sh.

#pragma version(1)
#pragma rs java_package_name(android.renderscript.cts)

rs_allocation gAllocInExponent;

float __attribute__((kernel)) testPowrFloatFloatFloat(float inBase, unsigned int x) {
    float inExponent = rsGetElementAt_float(gAllocInExponent, x);
    return powr(inBase, inExponent);
}

float2 __attribute__((kernel)) testPowrFloat2Float2Float2(float2 inBase, unsigned int x) {
    float2 inExponent = rsGetElementAt_float2(gAllocInExponent, x);
    return powr(inBase, inExponent);
}

float3 __attribute__((kernel)) testPowrFloat3Float3Float3(float3 inBase, unsigned int x) {
    float3 inExponent = rsGetElementAt_float3(gAllocInExponent, x);
    return powr(inBase, inExponent);
}

float4 __attribute__((kernel)) testPowrFloat4Float4Float4(float4 inBase, unsigned int x) {
    float4 inExponent = rsGetElementAt_float4(gAllocInExponent, x);
    return powr(inBase, inExponent);
}

half __attribute__((kernel)) testPowrHalfHalfHalf(half inBase, unsigned int x) {
    half inExponent = rsGetElementAt_half(gAllocInExponent, x);
    return powr(inBase, inExponent);
}

half2 __attribute__((kernel)) testPowrHalf2Half2Half2(half2 inBase, unsigned int x) {
    half2 inExponent = rsGetElementAt_half2(gAllocInExponent, x);
    return powr(inBase, inExponent);
}

half3 __attribute__((kernel)) testPowrHalf3Half3Half3(half3 inBase, unsigned int x) {
    half3 inExponent = rsGetElementAt_half3(gAllocInExponent, x);
    return powr(inBase, inExponent);
}

half4 __attribute__((kernel)) testPowrHalf4Half4Half4(half4 inBase, unsigned int x) {
    half4 inExponent = rsGetElementAt_half4(gAllocInExponent, x);
    return powr(inBase, inExponent);
}
