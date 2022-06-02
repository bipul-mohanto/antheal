#include "OptixDenoiser.h"

#include <optix_stubs.h>

#include <sutil/Exception.h>
#include <iomanip>

static void context_log_cb_denoiser(uint32_t level, const char* tag, const char* message, void* /*cbdata*/)
{
    if (level < 4)
        std::cerr << "[" << std::setw(2) << level << "][" << std::setw(12) << tag << "]: "
        << message << "\n";
}

void OptiXDenoiser::init(DenoiseData& data)
{
    SUTIL_ASSERT(data.color);
    SUTIL_ASSERT(data.output);
    SUTIL_ASSERT(data.width);
    SUTIL_ASSERT(data.height);
    SUTIL_ASSERT_MSG(!data.normal || data.albedo, "Currently albedo is required if normal input is given");

    m_host_output = data.output;

    //
    // Initialize CUDA and create OptiX context
    //
    {
        // Initialize CUDA
        CUDA_CHECK(cudaFree(0));

        CUcontext cu_ctx = 0;  // zero means take the current context
        OPTIX_CHECK(optixInit());
        OptixDeviceContextOptions options = {};
        options.logCallbackFunction = &context_log_cb_denoiser;
        options.logCallbackLevel = 4;
        OPTIX_CHECK(optixDeviceContextCreate(cu_ctx, &options, &m_context));
    }

    //
    // Create denoiser
    //
    {
        OptixDenoiserOptions options = {};
        options.inputKind =
            data.normal ? OPTIX_DENOISER_INPUT_RGB_ALBEDO_NORMAL :
            data.albedo ? OPTIX_DENOISER_INPUT_RGB_ALBEDO :
            OPTIX_DENOISER_INPUT_RGB;
        OPTIX_CHECK(optixDenoiserCreate(m_context, &options, &m_denoiser));
        OPTIX_CHECK(optixDenoiserSetModel(
            m_denoiser,
            OPTIX_DENOISER_MODEL_KIND_HDR,
            nullptr, // data
            0        // size
        ));
    }


    //
    // Allocate device memory for denoiser
    //
    {
        OptixDenoiserSizes denoiser_sizes;
        OPTIX_CHECK(optixDenoiserComputeMemoryResources(
            m_denoiser,
            data.width,
            data.height,
            &denoiser_sizes
        ));

        // NOTE: if using tiled denoising, we would set scratch-size to 
        //       denoiser_sizes.withOverlapScratchSizeInBytes
        m_scratch_size = static_cast<uint32_t>(denoiser_sizes.withoutOverlapScratchSizeInBytes);

        CUDA_CHECK(cudaMalloc(
            reinterpret_cast<void**>(&m_intensity),
            sizeof(float)
        ));
        CUDA_CHECK(cudaMalloc(
            reinterpret_cast<void**>(&m_scratch),
            m_scratch_size
        ));

        CUDA_CHECK(cudaMalloc(
            reinterpret_cast<void**>(&m_state),
            denoiser_sizes.stateSizeInBytes
        ));
        m_state_size = static_cast<uint32_t>(denoiser_sizes.stateSizeInBytes);

        const uint64_t frame_byte_size = data.width * data.height * sizeof(float4);
        /*CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&m_inputs[0].data), frame_byte_size));
        CUDA_CHECK(cudaMemcpy(
            reinterpret_cast<void*>(m_inputs[0].data),
            data.color,
            frame_byte_size,
            cudaMemcpyHostToDevice
        ));*/
        m_inputs[0].data = (CUdeviceptr)data.color;

        m_inputs[0].width = data.width;
        m_inputs[0].height = data.height;
        m_inputs[0].rowStrideInBytes = data.width * sizeof(float4);
        m_inputs[0].pixelStrideInBytes = sizeof(float4);
        m_inputs[0].format = OPTIX_PIXEL_FORMAT_FLOAT4;

        m_inputs[1].data = 0;
        if (data.albedo)
        {
            /*CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&m_inputs[1].data), frame_byte_size));
            CUDA_CHECK(cudaMemcpy(
                reinterpret_cast<void*>(m_inputs[1].data),
                data.albedo,
                frame_byte_size,
                cudaMemcpyHostToDevice
            ));*/
            m_inputs[1].data = (CUdeviceptr)data.albedo;

            m_inputs[1].width = data.width;
            m_inputs[1].height = data.height;
            m_inputs[1].rowStrideInBytes = data.width * sizeof(float4);
            m_inputs[1].pixelStrideInBytes = sizeof(float4);
            m_inputs[1].format = OPTIX_PIXEL_FORMAT_FLOAT4;
        }

        m_inputs[2].data = 0;
        if (data.normal)
        {
            /*CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&m_inputs[2].data), frame_byte_size));
            CUDA_CHECK(cudaMemcpy(
                reinterpret_cast<void*>(m_inputs[2].data),
                data.normal,
                frame_byte_size,
                cudaMemcpyHostToDevice
            ));*/
            m_inputs[2].data = (CUdeviceptr)data.normal;

            m_inputs[2].width = data.width;
            m_inputs[2].height = data.height;
            m_inputs[2].rowStrideInBytes = data.width * sizeof(float4);
            m_inputs[2].pixelStrideInBytes = sizeof(float4);
            m_inputs[2].format = OPTIX_PIXEL_FORMAT_FLOAT4;
        }

        //CUDA_CHECK(cudaMalloc(reinterpret_cast<void**>(&m_output.data), frame_byte_size));
        m_output.data = (CUdeviceptr)m_host_output;
        m_output.width = data.width;
        m_output.height = data.height;
        m_output.rowStrideInBytes = data.width * sizeof(float4);
        m_output.pixelStrideInBytes = sizeof(float4);
        m_output.format = OPTIX_PIXEL_FORMAT_FLOAT4;
    }

    //
    // Setup denoiser
    //
    {
        OPTIX_CHECK(optixDenoiserSetup(
            m_denoiser,
            0,  // CUDA stream
            data.width,
            data.height,
            m_state,
            m_state_size,
            m_scratch,
            m_scratch_size
        ));


        m_params.denoiseAlpha = 0;
        m_params.hdrIntensity = m_intensity;
        m_params.blendFactor = 0.f;
    }
}


void OptiXDenoiser::exec()
{
    OPTIX_CHECK(optixDenoiserComputeIntensity(
        m_denoiser,
        0, // CUDA stream
        m_inputs,
        m_intensity,
        m_scratch,
        m_scratch_size
    ));

    OPTIX_CHECK(optixDenoiserInvoke(
        m_denoiser,
        0, // CUDA stream
        &m_params,
        m_state,
        m_state_size,
        m_inputs,
        m_inputs[2].data ? 3 : m_inputs[1].data ? 2 : 1, // num input channels
        0, // input offset X
        0, // input offset y
        &m_output,
        m_scratch,
        m_scratch_size
    ));

    CUDA_SYNC_CHECK();

    /*const uint64_t frame_byte_size = m_output.width * m_output.height * sizeof(float4);
    CUDA_CHECK(cudaMemcpy(
        m_host_output,
        reinterpret_cast<void*>(m_output.data),
        frame_byte_size,
        cudaMemcpyDeviceToHost
    ));    */
}


void OptiXDenoiser::finish()
{
    if (!m_denoiser)
        return;

    /*const uint64_t frame_byte_size = m_output.width * m_output.height * sizeof(float4);
    CUDA_CHECK(cudaMemcpy(
        m_host_output,
        reinterpret_cast<void*>(m_output.data),
        frame_byte_size,
        cudaMemcpyDeviceToHost
    ));*/

    // Cleanup resources
    optixDenoiserDestroy(m_denoiser);
    optixDeviceContextDestroy(m_context);

    CUDA_CHECK(cudaFree(reinterpret_cast<void*>(m_intensity)));
    CUDA_CHECK(cudaFree(reinterpret_cast<void*>(m_scratch)));
    CUDA_CHECK(cudaFree(reinterpret_cast<void*>(m_state)));

    m_denoiser = nullptr;
}