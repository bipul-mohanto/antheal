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

#include <optix_device.h>
#include <random.h>

#include <cuda/helpers.h>

#include <sutil/vec_math.h>

#include "LaunchParams.h"
  
#include "Disney.cuh"

//#define USE_JITTERED_UNIFORM
#define USE_STRATIFIED
#define kProbeSamples 1.f
#define kBsdfSamples 1.f

/*! launch parameters in constant memory, filled in by optix upon
    optixLaunch (this gets filled in from the buffer we pass to
    optixLaunch) */
extern "C" {
     __constant__ LaunchParams params;
}

//------------------------------------------------------------------------------
//
//
//
//------------------------------------------------------------------------------

const int RAY_STATE_FLAGS_DONE               = 1 << 0;
const int RAY_STATE_FLAGS_SECONDARY_RAY      = 1 << 1;
const int RAY_STATE_FLAGS_SHADOW_RAY         = 1 << 2;

struct RadiancePRD
{
    float3       radiance;
    float3       alpha;

    float3       origin;
    float3       direction;

    float3       normal;
    float3       albedo;

    float4       lightSamples;

    float bsdfPdf = 1.0f;
    float3 pathThroughput;
    float rayEta = 1.0f;
    float3 rayAbsorption;
    BSDFType rayType = eReflected;    

    int depth;
    int stateFlags = 0;

    unsigned int seed;
    Random       rand;
};


struct Onb
{
    __forceinline__ __device__ Onb(const float3& normal)
    {
        m_normal = normal;

        if (fabs(m_normal.x) > fabs(m_normal.z))
        {
            m_binormal.x = -m_normal.y;
            m_binormal.y = m_normal.x;
            m_binormal.z = 0;
        }
        else
        {
            m_binormal.x = 0;
            m_binormal.y = -m_normal.z;
            m_binormal.z = m_normal.y;
        }

        m_binormal = normalize(m_binormal);
        m_tangent = cross(m_binormal, m_normal);
    }

    __forceinline__ __device__ void inverse_transform(float3& p) const
    {
        p = p.x * m_tangent + p.y * m_binormal + p.z * m_normal;
    }

    float3 m_tangent;
    float3 m_binormal;
    float3 m_normal;
};

static __forceinline__ __device__ void* unpackPointer(unsigned int i0, unsigned int i1)
{
    const unsigned long long uptr = static_cast<unsigned long long>(i0) << 32 | i1;
    void* ptr = reinterpret_cast<void*>(uptr);
    return ptr;
}


static __forceinline__ __device__ void  packPointer(void* ptr, unsigned int& i0, unsigned int& i1)
{
    const unsigned long long uptr = reinterpret_cast<unsigned long long>(ptr);
    i0 = uptr >> 32;
    i1 = uptr & 0x00000000ffffffff;
}

template<typename T>
static __forceinline__ __device__ T* getPRD()
{
    const unsigned int u0 = optixGetPayload_0();
    const unsigned int u1 = optixGetPayload_1();
    return reinterpret_cast<T*>(unpackPointer(u0, u1));
}


static __forceinline__ __device__ void setPayloadOcclusion(bool occluded)
{
    optixSetPayload_0(static_cast<unsigned int>(occluded));
}


static __forceinline__ __device__ void cosine_sample_hemisphere(const float u1, const float u2, float3& p)
{
    // Uniformly sample disk.
    const float r = sqrtf(u1);
    const float phi = 2.0f * M_PIf * u2;
    p.x = r * cosf(phi);
    p.y = r * sinf(phi);

    // Project up to hemisphere.
    p.z = sqrtf(fmaxf(0.0f, 1.0f - p.x * p.x - p.y * p.y));
}

static __forceinline__ __device__ void traceRadiance(
    OptixTraversableHandle handle,
    float3                 ray_origin,
    float3                 ray_direction,
    float                  tmin,
    float                  tmax,
    RadiancePRD* prd
)
{
    // TODO: deduce stride from num ray-types passed in params

    unsigned int u0, u1;
    packPointer(prd, u0, u1);
    optixTrace(
        handle,
        ray_origin,
        ray_direction,
        tmin,
        tmax,
        0.0f,                // rayTime
        OptixVisibilityMask(1),
        OPTIX_RAY_FLAG_NONE,
        RAY_TYPE_RADIANCE,        // SBT offset
        RAY_TYPE_COUNT,           // SBT stride
        RAY_TYPE_RADIANCE,        // missSBTIndex
        u0, u1);
}


static __forceinline__ __device__ bool traceOcclusion(
    OptixTraversableHandle handle,
    float3                 ray_origin,
    float3                 ray_direction,
    float                  tmin,
    float                  tmax
)
{
    unsigned int occluded = 0u;
    optixTrace(
        handle,
        ray_origin,
        ray_direction,
        tmin,
        tmax,
        0.0f,                    // rayTime
        OptixVisibilityMask(1),
        OPTIX_RAY_FLAG_TERMINATE_ON_FIRST_HIT,
        RAY_TYPE_OCCLUSION,      // SBT offset
        RAY_TYPE_COUNT,          // SBT stride
        RAY_TYPE_OCCLUSION,      // missSBTIndex
        occluded);
    return occluded;
}
  
extern "C" __global__ void __anyhit__radiance()
{ /*! for this simple example, this will remain empty */ }

extern "C" __global__ void __miss__radiance()
{
    RadiancePRD* prd = getPRD<RadiancePRD>();
    const float3 ray_dir = optixGetWorldRayDirection();

    /*float weight = 1.0f;
    // probability that this dir was already sampled by probe sampling
    float skyPdf = ProbePdf(params.probe, ray_dir);

    int N = kProbeSamples + kBsdfSamples;
    float cbsdf = kBsdfSamples / N;
    float csky = float(kProbeSamples) / N;

    weight = cbsdf * prd->bsdfPdf / (cbsdf * prd->bsdfPdf + csky * skyPdf);

    prd->radiance += weight * make_float3(ProbeEval(params.probe, ProbeDirToUV(ray_dir))) * prd->pathThroughput;*/
    prd->albedo = make_float3(0.f);
    prd->normal = make_float3(0.f);

    if ((prd->stateFlags & RAY_STATE_FLAGS_SECONDARY_RAY) != 0) {
        const float3 ray_dir = optixGetWorldRayDirection();
        //prd->alpha = make_float3(1.0f);
        //prd->radiance = make_float3(ProbeEval(params.probe, ProbeDirToUV(ray_dir)));
    }

    prd->stateFlags |= RAY_STATE_FLAGS_DONE;
}

extern "C" __global__ void __anyhit__occlusion()
{ 
    setPayloadOcclusion(true);
}

extern "C" __global__ void __closesthit__occlusion()
{ 
    
}

extern "C" __global__ void __miss__occlusion()
{ 
    setPayloadOcclusion(false);
}

static __device__ __forceinline__ float3 SampleLights(const Material& material, const float etaI, const float etaO, const float3& surfacePos, const float3& surfaceNormal, const float3& wo, Random& rand)
{
    float3 sum = make_float3(0.0f);

    for (int i = 0; i < kProbeSamples; ++i)
    {
        float3 skyColor;
        float skyPdf;
        float3 wi;
        
        ProbeSample(params.probe, wi, skyColor, skyPdf, rand);

        const bool occluded = traceOcclusion(
            params.traversable,
            surfacePos,
            wi,
            0.01f,         // tmin
            1e16f  // tmax
        );

        if (!occluded)
        {
            float bsdfPdf = BSDFPdf(material, etaI, etaO, surfacePos, surfaceNormal, wo, wi);
            float3 f = BSDFEval(material, etaI, etaO, surfacePos, surfaceNormal, wo, wi);

            if (bsdfPdf > 0.0f)
            {
                int N = kProbeSamples + kBsdfSamples;
                float cbsdf = kBsdfSamples / N;
                float csky = float(kProbeSamples) / N;
                float weight = csky * skyPdf / (cbsdf * bsdfPdf + csky * skyPdf);

                if (weight > 0.0f) {
                    float3 val = weight * skyColor * f * abs(dot(wi, surfaceNormal)) / skyPdf * (1.0f / kProbeSamples);
                    sum += val;// make_float3((val.x + val.y + val.z) / 3.0f);
                }
            }
        }
    }
    return sum;
}

static __device__ __forceinline__ float3 SampleShadow(const Material& material, const float etaI, const float etaO, const float3& surfacePos, const float3& surfaceNormal, const float3& wo, Random& rand)
{
    float3 sum = make_float3(0.0f);

    for (int i = 0; i < kProbeSamples; ++i)
    {
        float3 skyColor;
        float skyPdf;
        float3 wi;

        ProbeSample(params.probe, wi, skyColor, skyPdf, rand);

        const bool occluded = traceOcclusion(
            params.traversable,
            surfacePos,
            wi,
            0.01f,         // tmin
            1e16f  // tmax
        );

        if (occluded)
        {
            float bsdfPdf = BSDFPdf(material, etaI, etaO, surfacePos, surfaceNormal, wo, wi);
            float3 f = BSDFEval(material, etaI, etaO, surfacePos, surfaceNormal, wo, wi);

            if (bsdfPdf > 0.0f)
            {
                int N = kProbeSamples + kBsdfSamples;
                float cbsdf = kBsdfSamples / N;
                float csky = float(kProbeSamples) / N;
                float weight = csky * skyPdf / (cbsdf * bsdfPdf + csky * skyPdf);

                if (weight > 0.0f) {
                    float3 val = weight * skyColor * f * abs(dot(wi, surfaceNormal)) / skyPdf * (1.0f / kProbeSamples);
                    sum += val;
                }
            }
        }
    }
    return sum;
}


//------------------------------------------------------------------------------
// ray gen program - the actual rendering happens in here
//------------------------------------------------------------------------------
extern "C" __global__ void __raygen__renderFrame()
{
    const int    w = params.frame.size.x;
    const int    h = params.frame.size.y;
    const float3 eye = params.camera.eye;
    const float3 U = params.camera.U;
    const float3 V = params.camera.V;
    const float3 W = params.camera.W;
    const uint3  idx = optixGetLaunchIndex();
    const unsigned int    subframe_index = params.frame.subframe_index;
      
    float3 result = make_float3(0.0f);

    int samples_per_launch = (subframe_index == 0) ? 4 : params.samples_per_launch;
    int i = samples_per_launch;

    unsigned int seed = tea<4>(idx.y * w + idx.x, subframe_index);

    float3 normal = make_float3(0.f);
    float3 albedo = make_float3(0.f);
    float3 alpha = make_float3(0.f);    

    float3 backplate = make_float3(0.f);   

    do
    {        
        float3 directLight = make_float3(0.0f);
        float3 indirectLight = make_float3(0.0f);

        RadiancePRD prd;
        
        prd.radiance = make_float3(0.f);        
        prd.alpha = make_float3(0.f);

        prd.seed = seed;
        prd.rand = Random(seed);

        prd.rayEta = 1.0f;
        prd.pathThroughput = make_float3(1.f);
        prd.rayAbsorption = make_float3(0.0f);
        prd.bsdfPdf = 1.0f;
        prd.normal = make_float3(0.0f);
        prd.albedo = make_float3(0.0f);
        prd.stateFlags = 0;
        prd.depth = 0;

        // The center of each pixel is at fraction (0.5,0.5)
        const float2 subpixel_jitter = make_float2(rnd(seed), rnd(seed));

        float2 d = 2.0f * make_float2(
            (static_cast<float>(idx.x) + subpixel_jitter.x) / static_cast<float>(w),
            (static_cast<float>(idx.y) + subpixel_jitter.y) / static_cast<float>(h)
        ) - 1.0f;        

        float3 ray_origin;
        ray_origin = eye; /* +U * 0.1f;
        d = d + make_float2(0.25f, 0.0f); 
        */

        /* BM: uncommenting generates disparity and split window together  
        if(idx.x < w/2.0f)
        {
            ray_origin = eye - U * 0.1f;
            d = d + make_float2(0.25f, 0.f);
        } else {
            ray_origin = eye + U * 0.1f;
            d = d - make_float2(0.25f, 0.f);
        }
        */

        float3 ray_direction = normalize(d.x * U + d.y * V + W);
        // float3 ray_origin = eye;         

        

        backplate = make_float3(ProbeEval(params.probe, ProbeDirToUV(ray_direction)));

        for (;; )
        {
            prd.radiance = make_float3(0.f);

            traceRadiance(
                params.traversable,
                ray_origin,
                ray_direction,
                0.001f,  // tmin       // TODO: smarter offset
                1e16f,  // tmax
                &prd);

            if (prd.depth == 0.f) {
                normal += prd.normal;
                albedo += prd.albedo;               
            }           

            if ((prd.stateFlags & RAY_STATE_FLAGS_DONE) || prd.depth >= 2) // TODO RR, variable for depth
                break;

            if (prd.depth == 0) {
                directLight += prd.radiance;
            }
            else {
                indirectLight += prd.radiance;
            }

            ++prd.depth;

            ray_origin = prd.origin;
            ray_direction = prd.direction;                        
        }

        result += directLight + indirectLight;
        alpha += prd.alpha;       

    } while (--i);

    normal /= static_cast<float>(samples_per_launch);
    albedo /= static_cast<float>(samples_per_launch);
    alpha /= static_cast<float>(samples_per_launch);

    float3 color = (backplate * static_cast<float>(params.samples_per_launch)) * (1.0f - alpha) + result;

    const uint3    launch_index = optixGetLaunchIndex();
    const unsigned int image_index = launch_index.y * params.frame.size.x + launch_index.x;
    float3         accum_color = color / static_cast<float>(params.samples_per_launch); // result / static_cast<float>(params.samples_per_launch);

    if (subframe_index > 0)
    {
        accum_color = clamp(accum_color, make_float3(0.0), make_float3(10.0f));
        const float                 a = 1.0f / static_cast<float>(subframe_index + 1);
        const float3 accum_color_prev = make_float3(params.frame.accum_buffer[image_index]);
        accum_color = lerp(accum_color_prev, accum_color, a);
    }
    params.frame.accum_buffer[image_index] = make_float4(accum_color, 1.0f);

    //params.frame.frame_buffer[image_index] = make_color(accum_color);
    params.frame.frame_buffer[image_index] = make_color(accum_color);

    params.frame.normal_buffer[image_index] = make_float4(normal, 1.0f);
    params.frame.color_buffer[image_index] = make_float4(accum_color, 1.0f);
    params.frame.albedo_buffer[image_index] = make_float4(albedo, 1.0f);
}

extern "C" __global__ void __closesthit__radiance()
{
    const TriangleMeshSBTData& sbtData
        = *(const TriangleMeshSBTData*)optixGetSbtDataPointer();

    const int    prim_idx = optixGetPrimitiveIndex();
    const float3 ray_dir = optixGetWorldRayDirection();

    const uint3 index = sbtData.index[prim_idx];

    const float3 v0 = sbtData.vertex[index.x];
    const float3 v1 = sbtData.vertex[index.y];
    const float3 v2 = sbtData.vertex[index.z];
    const float3 N_0 = normalize(cross(v1 - v0, v2 - v0));

    float3 N = faceforward(N_0, -ray_dir, N_0);

    const float t = optixGetRayTmax();
    const float rayTime = 0.f;
    const float3 P = optixGetWorldRayOrigin() + t * ray_dir;    

    float outEta;
    float3 outAbsorption;

    RadiancePRD* prd = getPRD<RadiancePRD>();

    if ((sbtData.material.flags & MATERIAL_FLAG_SHADOW_CATCHER) != 0 && (prd->stateFlags & RAY_STATE_FLAGS_SECONDARY_RAY) != 0) {
        prd->origin = P;
        prd->direction = ray_dir;
        --prd->depth;
        return;
    }

    prd->normal = N;
    prd->albedo = sbtData.material.color;

    float& bsdfPdf = prd->bsdfPdf;    

    if (prd->rayEta == 1.0f)
    {
        outEta = sbtData.material.GetIndexOfRefraction();
        outAbsorption = sbtData.material.absorption;
    }
    else
    {
        // returning to free space
        outEta = 1.0f;
        outAbsorption = make_float3(0.0f);
    }

    // update throughput based on absorption through the medium
    //prd->pathThroughput *= exp(-prd->rayAbsorption * rayTime);    

    //float3 lightValue = SampleLights(sbtData.material, prd->rayEta, outEta, P, N, -ray_dir, prd->rand);
    //prd->radiance += prd->pathThroughput * clamp(lightValue, make_float3(0), make_float3(2));

    if((sbtData.material.flags & MATERIAL_FLAG_SHADOW_CATCHER) == 0){
        float3 lightSample = SampleLights(sbtData.material, prd->rayEta, outEta, P, N, -ray_dir, prd->rand);
        prd->radiance += prd->pathThroughput * lightSample;
        prd->alpha = make_float3(1.0f);
    }
    else {
        float3 shadowSample = SampleShadow(sbtData.material, prd->rayEta, outEta, P, N, -ray_dir, prd->rand);
        prd->alpha += prd->pathThroughput * shadowSample;
    }

    //float3 lightSample = SampleShadow(sbtData.material, prd->rayEta, outEta, P, N, -ray_dir, prd->rand);
    //prd->radiance += prd->pathThroughput * lightSample;

    
    if ((prd->stateFlags & RAY_STATE_FLAGS_SECONDARY_RAY) == 0) {
        prd->radiance += sbtData.material.emission;   
    }    

    float3 u, v;
    BasisFromVector(N, &u, &v);

    float3 bsdfDir;
    BSDFType bsdfType;

    BSDFSample(sbtData.material, prd->rayEta, outEta, P, u, v, N, -ray_dir, bsdfDir, bsdfPdf, bsdfType, prd->rand);

    if (bsdfPdf <= 0.0f){
        prd->stateFlags |= RAY_STATE_FLAGS_DONE;
        return;
    }

    // reflectance
    float3 f = BSDFEval(sbtData.material, prd->rayEta, outEta, P, N, -ray_dir, bsdfDir);

    // update ray medium if we are transmitting through the material
    if (dot(bsdfDir, N) <= 0.0f)
    {
        prd->rayEta = outEta;
        prd->rayAbsorption = outAbsorption;
    }

    // update throughput with primitive reflectance
    prd->pathThroughput *= f * abs(dot(N, bsdfDir)) / bsdfPdf;

    // update ray direction and type
    prd->rayType = bsdfType;
    prd->direction = bsdfDir;
    prd->origin = P;

    prd->stateFlags |= RAY_STATE_FLAGS_SECONDARY_RAY;
}
