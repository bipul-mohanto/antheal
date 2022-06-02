// ======================================================================== //
// Copyright 2018-2019 Ingo Wald                                            //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#pragma once
 
#include <optix.h>
#include "Probe.cuh"

#include "Material.h"

// for this simple example, we have a single ray type
enum { 
    RAY_TYPE_RADIANCE = 0, 
    RAY_TYPE_OCCLUSION = 1,
    RAY_TYPE_COUNT 
};


struct ParallelogramLight
{
    float3 corner;
    float3 v1, v2;
    float3 normal;
    float3 emission;
};

struct TriangleMeshSBTData {
    float3* vertex;
    float3* normal;
    float2* texcoord;
    uint3* index;
    bool                hasTexture;
    cudaTextureObject_t texture;

	Material material;
};

struct LaunchParams
{
    struct {
        float4* accum_buffer;
		uchar4* frame_buffer;

		float4* color_buffer;
		float4* normal_buffer;
		float4* albedo_buffer;
        
        int2     size;
        unsigned int subframe_index;
    } frame;
    
    struct {
        float3       eye;
        float3       U;
        float3       V;
        float3       W;
    } camera;   

    unsigned int samples_per_launch;

    OptixTraversableHandle traversable;
    ParallelogramLight     light;

    Probe probe;

};

struct RayGenData
{
    void* data;
};

struct MissData
{
    void* data;
};

struct HitGroupData
{
    TriangleMeshSBTData data;
};



