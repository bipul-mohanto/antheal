#include "SimplePathtracer.h"
#include <sutil/Exception.h>
#include <optix.h>
#include <optix_function_table_definition.h>
#include <optix_stack_size.h>

#include <iomanip>
#include <iostream>

#include "sampleConfig.h"
#include <sutil/sutil.h>

#include <fstream>

#include <sutil/Matrix.h>
#include <sutil/vec_math.h>

#include "toneMap.h"


#include "Material.h"



template <typename T>
struct SbtRecord
{
    __align__(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
    T data;
};

typedef SbtRecord<RayGenData>     RayGenRecord;
typedef SbtRecord<MissData>       MissRecord;
typedef SbtRecord<HitGroupData>   HitGroupRecord;

static void context_log_cb(unsigned int level, const char* tag, const char* message, void* /*cbdata */)
{
    std::cerr << "[" << std::setw(2) << level << "][" << std::setw(12) << tag << "]: "
        << message << "\n";
}

SampleRenderer::SampleRenderer(const Model* model)
    : model(model)
{
    initOptix();

    std::cout << "creating optix context ..." << std::endl;
    createContext();

    std::cout << "setting up module ..." << std::endl;
    createModule();

    std::cout << "creating raygen programs ..." << std::endl;
    createRaygenPrograms();
    std::cout << "creating miss programs ..." << std::endl;
    createMissPrograms();
    std::cout << "creating hitgroup programs ..." << std::endl;
    createHitgroupPrograms();

    launchParams.traversable = buildAccel();

    std::cout << "setting up optix pipeline ..." << std::endl;
    createPipeline();

    createTextures();

    std::cout << "building SBT ..." << std::endl;
    buildSBT();

    launchParamsBuffer.alloc(sizeof(launchParams));
    std::cout << "context, module, pipeline, etc, all set up ..." << std::endl;

    std::cout << "Optix 7 Sample fully set up" << std::endl;
}

void SampleRenderer::render()
{
    // sanity check: make sure we launch only after first resize is
    // already done:
    if (launchParams.frame.size.x == 0)
        std::cout << "sanity check passed" << std::endl; //return;

    launchParamsBuffer.upload(&launchParams, 1);

    OPTIX_CHECK(optixLaunch(/*! pipeline we're launching launch: */
        pipeline, stream,
        /*! parameters and SBT */
        launchParamsBuffer.d_pointer(),
        launchParamsBuffer.sizeInBytes,
        &sbt,
        /*! dimensions of the launch: */
        launchParams.frame.size.x,
        launchParams.frame.size.y,
        1
    ));
    // sync - make sure the frame is rendered before we download and
    // display (obviously, for a high-performance application you
    // want to use streams and double-buffering, but for this simple
    // example, this will have to do)
    CUDA_SYNC_CHECK();
}

void SampleRenderer::render(sutil::CUDAOutputBuffer<uint32_t> &renderTarget)
{
    uint32_t* result_buffer_data = renderTarget.map();
    launchParams.frame.frame_buffer = (uchar4*)result_buffer_data;
    render();
    //denoiser.exec();
    // 
    //computeFinalPixelColors(launchParams.frame.size, (float4*)denoisedBuffer.d_ptr, result_buffer_data);
    renderTarget.unmap();
}

void SampleRenderer::resize(const int2& newSize)
{
    // if window minimized
    if (newSize.x == 0 || newSize.y == 0) return;

    // BM
    //denoiser.finish();

    // ------------------------------------------------------------------
    // resize our cuda frame buffer
    denoisedBuffer.resize(newSize.x * newSize.y * sizeof(float4));

    // resize our cuda frame buffer
    frame_buffer.resize(newSize.x * newSize.y * sizeof(uint32_t));
    accum_buffer.resize(newSize.x * newSize.y * sizeof(float4));

    normal_buffer.resize(newSize.x * newSize.y * sizeof(float4));
    color_buffer.resize(newSize.x * newSize.y * sizeof(float4));
    albedo_buffer.resize(newSize.x * newSize.y * sizeof(float4));

    // update the launch parameters that we'll pass to the optix
    // launch:
    launchParams.frame.size = newSize;
    launchParams.frame.frame_buffer = (uchar4*)frame_buffer.d_ptr;
    launchParams.frame.accum_buffer = (float4*)accum_buffer.d_ptr;

    launchParams.frame.normal_buffer = (float4*)normal_buffer.d_ptr;
    launchParams.frame.color_buffer = (float4*)color_buffer.d_ptr;
    launchParams.frame.albedo_buffer = (float4*)albedo_buffer.d_ptr;    

    /* BM 
    OptiXDenoiser::DenoiseData denoiseData;
    denoiseData.width = newSize.x;
    denoiseData.height = newSize.y;
    denoiseData.color = (float*)color_buffer.d_ptr;
    denoiseData.albedo = (float*)albedo_buffer.d_ptr;
    */


    //denoiseData.normal = (float*)normal_buffer.d_ptr;

    /* BM
    denoiseData.output = (float*)denoisedBuffer.d_ptr;
    denoiser.init(denoiseData);
    */
}

void SampleRenderer::downloadPixels(uint32_t h_pixels[])
{
    frame_buffer.download(h_pixels,
        launchParams.frame.size.x * launchParams.frame.size.y);
}

void SampleRenderer::setCamera(const sutil::Camera& camera)
{
    lastSetCamera = camera;
        
    lastSetCamera.setAspectRatio(launchParams.frame.size.x / float(launchParams.frame.size.y));
    lastSetCamera.UVWFrame(launchParams.camera.U, launchParams.camera.V, launchParams.camera.W);
    launchParams.camera.eye = lastSetCamera.eye();
}

void SampleRenderer::setProbe(const ProbeData& probe)
{
    probeData.createBuffer(probe);

    launchParams.probe.cdfValuesX = (float*)probeData.cdfValuesX.d_ptr;
    launchParams.probe.cdfValuesY = (float*)probeData.cdfValuesY.d_ptr;

    launchParams.probe.pdfValuesX = (float*)probeData.pdfValuesX.d_ptr;
    launchParams.probe.pdfValuesY = (float*)probeData.pdfValuesY.d_ptr;

    launchParams.probe.data = (float4*)probeData.data.d_ptr;

    launchParams.probe.width = probeData.width;
    launchParams.probe.height = probeData.height;

    launchParams.probe.offset = probeData.offset;
}

void SampleRenderer::initOptix()
{
    std::cout << "#osc: initializing optix..." << std::endl;

    // -------------------------------------------------------
    // check for available optix7 capable devices
    // -------------------------------------------------------
    cudaFree(0);
    int numDevices;
    cudaGetDeviceCount(&numDevices);
    if (numDevices == 0)
        throw std::runtime_error("no CUDA capable devices found!");
    std::cout << "found " << numDevices << " CUDA devices" << std::endl;

    // -------------------------------------------------------
    // initialize optix
    // -------------------------------------------------------
    OPTIX_CHECK(optixInit());
    std::cout << "successfully initialized optix"  << std::endl;
}

void SampleRenderer::createContext()
{
    OptixDeviceContextOptions options = {};
    options.logCallbackFunction = &context_log_cb;
    options.logCallbackLevel = 4;
    cudaContext = 0;  // zero means take the current context
    OPTIX_CHECK(optixDeviceContextCreate(cudaContext, &options, &optixContext));

    CUDA_CHECK(cudaStreamCreate(&stream));
}

void SampleRenderer::createModule()
{
    /*
    OptixModuleCompileOptions module_compile_options = {};
    module_compile_options.maxRegisterCount = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
    module_compile_options.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
    module_compile_options.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_MINIMAL;
    */

    //bm
    pipelineCompileOptions = {};
    moduleCompileOptions = {};
    moduleCompileOptions.maxRegisterCount = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
    moduleCompileOptions.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
    moduleCompileOptions.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_MODERATE;

    pipelineCompileOptions.usesMotionBlur = false;
    pipelineCompileOptions.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_LEVEL_INSTANCING;
    pipelineCompileOptions.numPayloadValues = 2;
    pipelineCompileOptions.numAttributeValues = 2;
    pipelineCompileOptions.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;  // TODO: should be OPTIX_EXCEPTION_FLAG_STACK_OVERFLOW;
    pipelineCompileOptions.pipelineLaunchParamsVariableName = "params";

    size_t      inputSize = 0;
    //BM
    const char* input = sutil::getInputData(OPTIX_SAMPLE_NAME, OPTIX_SAMPLE_DIR, "deviceProgram.cu", inputSize);
    // SV
//    const std::string ptx = sutil::getPtxString(OPTIX_SAMPLE_NAME, OPTIX_SAMPLE_DIR, "deviceProgram.cu");

    char log[2048];
    size_t sizeof_log = sizeof(log);
    OPTIX_CHECK_LOG(optixModuleCreateFromPTX(
        optixContext,
        &moduleCompileOptions,
        &pipelineCompileOptions,
        input, //ptx.c_str(),
        inputSize, // ptx.size()
        log,
        &sizeof_log,
        &module
    ));
}

void SampleRenderer::createRaygenPrograms()
{
    // we do a single ray gen program in this example:
    raygenPGs.resize(1);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc = {};
    pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_RAYGEN;
    pgDesc.raygen.module = module;
    pgDesc.raygen.entryFunctionName = "__raygen__renderFrame";

    // OptixProgramGroup raypg;
    char log[2048];
    size_t sizeof_log = sizeof(log);

    OPTIX_CHECK_LOG(optixProgramGroupCreate(
        optixContext,
        &pgDesc,
        1,   // num program groups
        &pgOptions,
        log,
        &sizeof_log,
        &raygenPGs[0]
    ));
}

void SampleRenderer::createMissPrograms()
{
    // we do a single ray gen program in this example:
    missPGs.resize(RAY_TYPE_COUNT);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc = {};
    pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
    pgDesc.miss.module = module;

    char log[2048];
    size_t sizeof_log = sizeof(log);

    pgDesc.miss.entryFunctionName = "__miss__radiance";
    OPTIX_CHECK_LOG(optixProgramGroupCreate(optixContext,
        &pgDesc,
        1,
        &pgOptions,
        log, &sizeof_log,
        &missPGs[0]
    ));

    pgDesc.miss.entryFunctionName = "__miss__occlusion";
    OPTIX_CHECK_LOG(optixProgramGroupCreate(optixContext,
        &pgDesc,
        1,
        &pgOptions,
        log, &sizeof_log,
        &missPGs[1]
    ));
}

void SampleRenderer::createHitgroupPrograms()
{
    // for this simple example, we set up a single hit group
    hitgroupPGs.resize(RAY_TYPE_COUNT);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc = {};
    pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    pgDesc.hitgroup.moduleCH = module;
    pgDesc.hitgroup.moduleAH = module;

    char log[2048];
    size_t sizeof_log = sizeof(log);

    pgDesc.hitgroup.entryFunctionNameCH = "__closesthit__radiance";    
    pgDesc.hitgroup.entryFunctionNameAH = "__anyhit__radiance";
    OPTIX_CHECK_LOG(optixProgramGroupCreate(optixContext,
        &pgDesc,
        1,
        &pgOptions,
        log, &sizeof_log,
        &hitgroupPGs[0]
    ));

    pgDesc.hitgroup.entryFunctionNameCH = "__closesthit__occlusion";
    pgDesc.hitgroup.entryFunctionNameAH = "__anyhit__occlusion";
    OPTIX_CHECK_LOG(optixProgramGroupCreate(optixContext,
        &pgDesc,
        1,
        &pgOptions,
        log, &sizeof_log,
        &hitgroupPGs[1]
    ));
}

void SampleRenderer::createPipeline()
{
    const uint32_t    max_trace_depth = 1;
    std::vector<OptixProgramGroup> program_Groups;
    for (auto pg : raygenPGs)
        program_Groups.push_back(pg);
    for (auto pg : missPGs)
        program_Groups.push_back(pg);
    for (auto pg : hitgroupPGs)
        program_Groups.push_back(pg);

    pipelineLinkOptions = {};
    pipelineLinkOptions.maxTraceDepth = max_trace_depth;
    pipelineLinkOptions.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_FULL;

    char log[2048];
    size_t sizeof_log = sizeof(log);
    OPTIX_CHECK_LOG(optixPipelineCreate(
        optixContext,
        &pipelineCompileOptions,
        &pipelineLinkOptions,
        program_Groups.data(),
        (int)program_Groups.size(),
        log,
        &sizeof_log,
        &pipeline
    ));

    OptixStackSizes stack_sizes = {};
    for (auto& prog_group : program_Groups)
    {
        OPTIX_CHECK(optixUtilAccumulateStackSizes(prog_group, &stack_sizes));
    }

    uint32_t direct_callable_stack_size_from_traversal;
    uint32_t direct_callable_stack_size_from_state;
    uint32_t continuation_stack_size;
    OPTIX_CHECK(optixUtilComputeStackSizes(&stack_sizes, max_trace_depth,
        0,  // maxCCDepth
        0,  // maxDCDEpth
        &direct_callable_stack_size_from_traversal,
        &direct_callable_stack_size_from_state, &continuation_stack_size));
    OPTIX_CHECK(optixPipelineSetStackSize(pipeline, direct_callable_stack_size_from_traversal,
        direct_callable_stack_size_from_state, continuation_stack_size,
        2  // maxTraversableDepth
    ));
}


void SampleRenderer::buildSBT()
{
    // ------------------------------------------------------------------
    // build raygen records
    // ------------------------------------------------------------------
    std::vector<RayGenRecord> raygenRecords;
    for (int i = 0;i < raygenPGs.size();i++) {
        RayGenRecord rec;
        OPTIX_CHECK(optixSbtRecordPackHeader(raygenPGs[i], &rec));
        rec.data.data = nullptr; /* for now ... */
        raygenRecords.push_back(rec);
    }
    raygenRecordsBuffer.alloc_and_upload(raygenRecords);
    sbt.raygenRecord = raygenRecordsBuffer.d_pointer();

    // ------------------------------------------------------------------
    // build miss records
    // ------------------------------------------------------------------
    std::vector<MissRecord> missRecords;
    for (int i = 0;i < missPGs.size();i++) {
        MissRecord rec;
        OPTIX_CHECK(optixSbtRecordPackHeader(missPGs[i], &rec));
        rec.data.data = nullptr; /* for now ... */
        missRecords.push_back(rec);
    }
    missRecordsBuffer.alloc_and_upload(missRecords);
    sbt.missRecordBase = missRecordsBuffer.d_pointer();
    sbt.missRecordStrideInBytes = sizeof(MissRecord);
    sbt.missRecordCount = (int)missRecords.size();

    // ------------------------------------------------------------------
    // build hitgroup records
    // ------------------------------------------------------------------

    // we don't actually have any objects in this example, but let's
    // create a dummy one so the SBT doesn't have any null pointers
    // (which the sanity checks in compilation would complain about)
    int numObjects = (int)model->meshes.size();
    std::vector<HitGroupRecord> hitgroupRecords;
    for (int meshID = 0;meshID < numObjects;meshID++) {
        for (int rayID = 0;rayID < RAY_TYPE_COUNT;rayID++) {
            auto mesh = model->meshes[meshID];

            HitGroupRecord rec;
            OPTIX_CHECK(optixSbtRecordPackHeader(hitgroupPGs[rayID], &rec));
            
            // BM commented below line 
            //rec.data.data.material = mesh->material;

            if (mesh->diffuseTextureID >= 0) {
                rec.data.data.hasTexture = true;
                rec.data.data.texture = textureObjects[mesh->diffuseTextureID];
            }
            else {
                rec.data.data.hasTexture = false;
            }
            rec.data.data.index = (uint3*)indexBuffer[meshID].d_pointer();
            rec.data.data.vertex = (float3*)vertexBuffer[meshID].d_pointer();
            rec.data.data.normal = (float3*)normalBuffer[meshID].d_pointer();
            rec.data.data.texcoord = (float2*)texcoordBuffer[meshID].d_pointer();
            hitgroupRecords.push_back(rec);
        }
    }
    hitgroupRecordsBuffer.alloc_and_upload(hitgroupRecords);
    sbt.hitgroupRecordBase = hitgroupRecordsBuffer.d_pointer();
    sbt.hitgroupRecordStrideInBytes = sizeof(HitGroupRecord);
    sbt.hitgroupRecordCount = (int)hitgroupRecords.size();
}

OptixTraversableHandle SampleRenderer::buildAccel()
{
    int value;
    optixDeviceContextGetProperty(optixContext, OPTIX_DEVICE_PROPERTY_LIMIT_MAX_PRIMITIVES_PER_GAS, &value, sizeof(int));

    std::cout << "OPTIX_DEVICE_PROPERTY_LIMIT_MAX_PRIMITIVES_PER_GAS " << value << "\n";

    // meshes.resize(1);
    const int numMeshes = (int)model->meshes.size();
    vertexBuffer.resize(numMeshes);
    normalBuffer.resize(numMeshes);
    texcoordBuffer.resize(numMeshes);
    indexBuffer.resize(numMeshes);

    OptixTraversableHandle asHandle{ 0 };

    // ==================================================================
    // triangle inputs
    // ==================================================================
    std::vector<OptixBuildInput> triangleInput(numMeshes);
    std::vector<CUdeviceptr> d_vertices(numMeshes);
    std::vector<CUdeviceptr> d_indices(numMeshes);
    std::vector<uint32_t> triangleInputFlags(numMeshes);

    for (int meshID = 0;meshID < model->meshes.size();meshID++) {
        // upload the model to the device: the builder
        TriangleMesh& mesh = *model->meshes[meshID];
        vertexBuffer[meshID].alloc_and_upload(mesh.vertex);
        indexBuffer[meshID].alloc_and_upload(mesh.index);
        if (!mesh.normal.empty())
            normalBuffer[meshID].alloc_and_upload(mesh.normal);
        if (!mesh.texcoord.empty())
            texcoordBuffer[meshID].alloc_and_upload(mesh.texcoord);

        triangleInput[meshID] = {};
        triangleInput[meshID].type
            = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;

        // create local variables, because we need a *pointer* to the
        // device pointers
        d_vertices[meshID] = vertexBuffer[meshID].d_pointer();
        d_indices[meshID] = indexBuffer[meshID].d_pointer();

        triangleInput[meshID].triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
        triangleInput[meshID].triangleArray.vertexStrideInBytes = sizeof(float3);
        triangleInput[meshID].triangleArray.numVertices = (int)mesh.vertex.size();
        triangleInput[meshID].triangleArray.vertexBuffers = &d_vertices[meshID];

        triangleInput[meshID].triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
        triangleInput[meshID].triangleArray.indexStrideInBytes = sizeof(uint3);
        triangleInput[meshID].triangleArray.numIndexTriplets = (int)mesh.index.size();
        triangleInput[meshID].triangleArray.indexBuffer = d_indices[meshID];

        triangleInputFlags[meshID] = 0;

        // in this example we have one SBT entry, and no per-primitive
        // materials:
        triangleInput[meshID].triangleArray.flags = &triangleInputFlags[meshID];
        triangleInput[meshID].triangleArray.numSbtRecords = 1;
        triangleInput[meshID].triangleArray.sbtIndexOffsetBuffer = 0;
        triangleInput[meshID].triangleArray.sbtIndexOffsetSizeInBytes = 0;
        triangleInput[meshID].triangleArray.sbtIndexOffsetStrideInBytes = 0;
    }
    // ==================================================================
    // BLAS setup
    // ==================================================================

    OptixAccelBuildOptions accelOptions = {};
    accelOptions.buildFlags = OPTIX_BUILD_FLAG_NONE
        | OPTIX_BUILD_FLAG_ALLOW_COMPACTION
        ;
    accelOptions.motionOptions.numKeys = 1;
    accelOptions.operation = OPTIX_BUILD_OPERATION_BUILD;

    OptixAccelBufferSizes blasBufferSizes;
    OPTIX_CHECK(optixAccelComputeMemoryUsage
    (optixContext,
        &accelOptions,
        triangleInput.data(),
        (int)model->meshes.size(),  // num_build_inputs
        &blasBufferSizes
    ));

    // ==================================================================
    // prepare compaction
    // ==================================================================

    CUDABuffer compactedSizeBuffer;
    compactedSizeBuffer.alloc(sizeof(uint64_t));

    OptixAccelEmitDesc emitDesc;
    emitDesc.type = OPTIX_PROPERTY_TYPE_COMPACTED_SIZE;
    emitDesc.result = compactedSizeBuffer.d_pointer();

    // ==================================================================
    // execute build (main stage)
    // ==================================================================

    CUDABuffer tempBuffer;
    tempBuffer.alloc(blasBufferSizes.tempSizeInBytes);

    CUDABuffer outputBuffer;
    outputBuffer.alloc(blasBufferSizes.outputSizeInBytes);

    OPTIX_CHECK(optixAccelBuild(optixContext,
        /*stream*/0,
        &accelOptions,
        triangleInput.data(),
        (int)model->meshes.size(),
        tempBuffer.d_pointer(),
        tempBuffer.sizeInBytes,

        outputBuffer.d_pointer(),
        outputBuffer.sizeInBytes,

        &asHandle,

        &emitDesc, 1
    ));
    CUDA_SYNC_CHECK();

    // ==================================================================
    // perform compaction
    // ==================================================================
    uint64_t compactedSize;
    compactedSizeBuffer.download(&compactedSize, 1);

    asBuffer.alloc(compactedSize);
    OPTIX_CHECK(optixAccelCompact(optixContext,
        /*stream:*/0,
        asHandle,
        asBuffer.d_pointer(),
        asBuffer.sizeInBytes,
        &asHandle));
    CUDA_SYNC_CHECK();

    // ==================================================================
    // aaaaaand .... clean up
    // ==================================================================
    outputBuffer.free(); // << the UNcompacted, temporary output buffer
    tempBuffer.free();
    compactedSizeBuffer.free();

    return asHandle;
}

void SampleRenderer::createTextures()
{
    int numTextures = (int)model->textures.size();

    textureArrays.resize(numTextures);
    textureObjects.resize(numTextures);

    for (int textureID = 0;textureID < numTextures;textureID++) {
        auto texture = model->textures[textureID];

        cudaResourceDesc res_desc = {};

        cudaChannelFormatDesc channel_desc;
        int32_t width = texture->resolution.x;
        int32_t height = texture->resolution.y;
        int32_t numComponents = 4;
        int32_t pitch = width * numComponents * sizeof(uint8_t);
        channel_desc = cudaCreateChannelDesc<uchar4>();

        cudaArray_t& pixelArray = textureArrays[textureID];
        CUDA_CHECK(cudaMallocArray(&pixelArray,
            &channel_desc,
            width, height));

        CUDA_CHECK(cudaMemcpy2DToArray(pixelArray,
            /* offset */0, 0,
            texture->pixel,
            pitch, pitch, height,
            cudaMemcpyHostToDevice));

        res_desc.resType = cudaResourceTypeArray;
        res_desc.res.array.array = pixelArray;

        cudaTextureDesc tex_desc = {};
        tex_desc.addressMode[0] = cudaAddressModeWrap;
        tex_desc.addressMode[1] = cudaAddressModeWrap;
        tex_desc.filterMode = cudaFilterModeLinear;
        tex_desc.readMode = cudaReadModeNormalizedFloat;
        tex_desc.normalizedCoords = 1;
        tex_desc.maxAnisotropy = 1;
        tex_desc.maxMipmapLevelClamp = 99;
        tex_desc.minMipmapLevelClamp = 0;
        tex_desc.mipmapFilterMode = cudaFilterModePoint;
        tex_desc.borderColor[0] = 1.0f;
        tex_desc.sRGB = 0;

        // Create texture object
        cudaTextureObject_t cuda_tex = 0;
        CUDA_CHECK(cudaCreateTextureObject(&cuda_tex, &res_desc, &tex_desc, nullptr));
        textureObjects[textureID] = cuda_tex;
    }
}