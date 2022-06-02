#pragma once

#ifndef OPTIX_DENOISER
#define OPTIX_DENOISER

#include "optix.h"

#include <cuda_runtime.h>

#include <stdint.h>

class OptiXDenoiser
{
public:
    struct DenoiseData
    {
        uint32_t  width = 0;
        uint32_t  height = 0;
        float* color = nullptr;
        float* albedo = nullptr;
        float* normal = nullptr;
        float* output = nullptr;
    };

    // Initialize the API and push all data to the GPU -- normaly done only once per session
    void init(DenoiseData& data);

    // Execute the denoiser. In interactive sessions, this would be done once per frame/subframe
    void exec();

    // Cleanup state, deallocate memory -- normally done only once per render session
    void finish();


private:
    OptixDeviceContext    m_context = nullptr;
    OptixDenoiser         m_denoiser = nullptr;
    OptixDenoiserParams   m_params = {};

    CUdeviceptr           m_intensity = 0;
    CUdeviceptr           m_scratch = 0;
    uint32_t              m_scratch_size = 0;
    CUdeviceptr           m_state = 0;
    uint32_t              m_state_size = 0;

    OptixImage2D          m_inputs[3] = {};
    OptixImage2D          m_output;

    float* m_host_output = nullptr;
};



#endif