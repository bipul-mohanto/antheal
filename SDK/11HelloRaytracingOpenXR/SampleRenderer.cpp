#include "SampleRenderer.h"

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

SampleRenderer::SampleRenderer(const std::vector<TriangleMesh>& meshes)
    : meshes(meshes)
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
    if (launchParams.frame.fbSize.x == 0) return;

    launchParamsBuffer.upload(&launchParams, 1);

    OPTIX_CHECK(optixLaunch(/*! pipeline we're launching launch: */
        pipeline, stream,
        /*! parameters and SBT */
        launchParamsBuffer.d_pointer(),
        launchParamsBuffer.sizeInBytes,
        &sbt,
        /*! dimensions of the launch: */
        launchParams.frame.fbSize.x,
        launchParams.frame.fbSize.y,
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
    launchParams.frame.colorBuffer = result_buffer_data;
    render();
    renderTarget.unmap();
}

void SampleRenderer::resize(const int2& newSize)
{
    // if window minimized
    if (newSize.x == 0 || newSize.y == 0) return;

    // resize our cuda frame buffer
    colorBuffer.resize(newSize.x * newSize.y * sizeof(uint32_t));

    // update the launch parameters that we'll pass to the optix
    // launch:
    launchParams.frame.fbSize = newSize;
    launchParams.frame.colorBuffer = (uint32_t*)colorBuffer.d_ptr;
}

void SampleRenderer::downloadPixels(uint32_t h_pixels[])
{
    colorBuffer.download(h_pixels,
        launchParams.frame.fbSize.x * launchParams.frame.fbSize.y);
}

void SampleRenderer::setCamera(const sutil::Camera& camera)
{
    lastSetCamera = camera;
    launchParams.camera.position = camera.eye();
    launchParams.camera.direction = normalize(camera.lookat() - camera.eye());
    const float cosFovy = 0.66f;
    const float aspect = launchParams.frame.fbSize.x / float(launchParams.frame.fbSize.y);
    launchParams.camera.horizontal
        = cosFovy * aspect * normalize(cross(launchParams.camera.direction,
            camera.up()));
    launchParams.camera.vertical
        = cosFovy * normalize(cross(launchParams.camera.horizontal,
            launchParams.camera.direction));
}

void SampleRenderer::setCamera(
    const float3& position, 
    const float3& direction,
    const float3& up)
{
    launchParams.camera.position = position;
    launchParams.camera.direction = normalize(direction);
    const float cosFovy = 0.66f;
    const float aspect = launchParams.frame.fbSize.x / float(launchParams.frame.fbSize.y);
    launchParams.camera.horizontal
        = cosFovy * aspect * normalize(cross(launchParams.camera.direction, up));
    launchParams.camera.vertical
        = cosFovy * normalize(cross(launchParams.camera.horizontal,
            launchParams.camera.direction));
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
    pipelineCompileOptions = {};

    moduleCompileOptions = {};
    moduleCompileOptions.maxRegisterCount = OPTIX_COMPILE_DEFAULT_MAX_REGISTER_COUNT;
    moduleCompileOptions.optLevel = OPTIX_COMPILE_OPTIMIZATION_DEFAULT;
    moduleCompileOptions.debugLevel = OPTIX_COMPILE_DEBUG_LEVEL_MODERATE;
    
    /* optix 7.2. OPTIX_COMPILE_DEBUG_LEVEL_LINEINFO;*/
    /* OPTIX_COMPILE_DEBUG_LEVEL_MINIMAL generate information that does not impact performance(currently generates line information useful for profiling)*/

    /* OPTIX_COMPILE_DEBUG_LEVEL_MODERATE - generate some debug information
with slight performance costs (currently generates a debug frame necessary to show a
backtrace)*/

    pipelineCompileOptions.usesMotionBlur = false;
    pipelineCompileOptions.traversableGraphFlags = OPTIX_TRAVERSABLE_GRAPH_FLAG_ALLOW_SINGLE_LEVEL_INSTANCING;
    pipelineCompileOptions.numPayloadValues = 2;
    pipelineCompileOptions.numAttributeValues = 2;
    pipelineCompileOptions.exceptionFlags = OPTIX_EXCEPTION_FLAG_NONE;  // TODO: should be OPTIX_EXCEPTION_FLAG_STACK_OVERFLOW;
    pipelineCompileOptions.pipelineLaunchParamsVariableName = "launchParams";

    // BM
    size_t      inputSize = 0;
    const char* input = sutil::getInputData(OPTIX_SAMPLE_NAME, OPTIX_SAMPLE_DIR, "deviceProgram.cu", inputSize);
    //const std::string ptx = sutil::getPtxString(OPTIX_SAMPLE_NAME, OPTIX_SAMPLE_DIR, "deviceProgram.cu");

    char log[2048];
    size_t sizeof_log = sizeof(log);
    //size_t sizeof_log = sizeof(log);
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
    missPGs.resize(1);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc = {};
    pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_MISS;
    pgDesc.miss.module = module;
    pgDesc.miss.entryFunctionName = "__miss__radiance";

    // OptixProgramGroup raypg;
    char log[2048];
    size_t sizeof_log = sizeof(log);
    OPTIX_CHECK_LOG(optixProgramGroupCreate(optixContext,
        &pgDesc,
        1,
        &pgOptions,
        log, &sizeof_log,
        &missPGs[0]
    ));
}

void SampleRenderer::createHitgroupPrograms()
{
    // for this simple example, we set up a single hit group
    hitgroupPGs.resize(1);

    OptixProgramGroupOptions pgOptions = {};
    OptixProgramGroupDesc pgDesc = {};
    pgDesc.kind = OPTIX_PROGRAM_GROUP_KIND_HITGROUP;
    pgDesc.hitgroup.moduleCH = module;
    pgDesc.hitgroup.entryFunctionNameCH = "__closesthit__radiance";
    pgDesc.hitgroup.moduleAH = module;
    pgDesc.hitgroup.entryFunctionNameAH = "__anyhit__radiance";

    char log[2048];
    size_t sizeof_log = sizeof(log);
    OPTIX_CHECK_LOG(optixProgramGroupCreate(optixContext,
        &pgDesc,
        1,
        &pgOptions,
        log, &sizeof_log,
        &hitgroupPGs[0]
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
    int numObjects = meshes.size();
    std::vector<HitGroupRecord> hitgroupRecords;
    for (int i = 0;i < numObjects;i++) {
        int objectType = 0;
        HitGroupRecord rec;
        OPTIX_CHECK(optixSbtRecordPackHeader(hitgroupPGs[objectType], &rec));
        rec.data.data.vertex = (float3*)vertexBuffer[i].d_pointer();
        rec.data.data.index = (int3*)indexBuffer[i].d_pointer();
        rec.data.data.color = meshes[i].color;
        hitgroupRecords.push_back(rec);
    }
    hitgroupRecordsBuffer.alloc_and_upload(hitgroupRecords);
    sbt.hitgroupRecordBase = hitgroupRecordsBuffer.d_pointer();
    sbt.hitgroupRecordStrideInBytes = sizeof(HitGroupRecord);
    sbt.hitgroupRecordCount = (int)hitgroupRecords.size();
}

OptixTraversableHandle SampleRenderer::buildAccel()
{
    // meshes.resize(1);

    vertexBuffer.resize(meshes.size());
    indexBuffer.resize(meshes.size());

    OptixTraversableHandle asHandle{ 0 };

    // ==================================================================
    // triangle inputs
    // ==================================================================
    std::vector<OptixBuildInput> triangleInput(meshes.size());
    std::vector<CUdeviceptr> d_vertices(meshes.size());
    std::vector<CUdeviceptr> d_indices(meshes.size());
    std::vector<uint32_t> triangleInputFlags(meshes.size());

    for (int meshID = 0;meshID < meshes.size();meshID++) {
        // upload the model to the device: the builder
        TriangleMesh& model = meshes[meshID];
        vertexBuffer[meshID].alloc_and_upload(model.vertex);
        indexBuffer[meshID].alloc_and_upload(model.index);

        triangleInput[meshID] = {};
        triangleInput[meshID].type
            = OPTIX_BUILD_INPUT_TYPE_TRIANGLES;

        // create local variables, because we need a *pointer* to the
        // device pointers
        d_vertices[meshID] = vertexBuffer[meshID].d_pointer();
        d_indices[meshID] = indexBuffer[meshID].d_pointer();

        triangleInput[meshID].triangleArray.vertexFormat = OPTIX_VERTEX_FORMAT_FLOAT3;
        triangleInput[meshID].triangleArray.vertexStrideInBytes = sizeof(float3);
        triangleInput[meshID].triangleArray.numVertices = (int)model.vertex.size();
        triangleInput[meshID].triangleArray.vertexBuffers = &d_vertices[meshID];

        triangleInput[meshID].triangleArray.indexFormat = OPTIX_INDICES_FORMAT_UNSIGNED_INT3;
        triangleInput[meshID].triangleArray.indexStrideInBytes = sizeof(int3);
        triangleInput[meshID].triangleArray.numIndexTriplets = (int)model.index.size();
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
        (int)meshes.size(),  // num_build_inputs
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
        /* stream */0,
        &accelOptions,
        triangleInput.data(),
        (int)meshes.size(),
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

void TriangleMesh::addUnitCube(const sutil::Matrix<4, 4>& xfm)
{
    int firstVertexID = (int)vertex.size();
    vertex.push_back(make_float3(xfm * make_float4(0.f, 0.f, 0.f, 1.0f)));
    vertex.push_back(make_float3(xfm * make_float4(1.f, 0.f, 0.f, 1.0f)));
    vertex.push_back(make_float3(xfm * make_float4(0.f, 1.f, 0.f, 1.0f)));
    vertex.push_back(make_float3(xfm * make_float4(1.f, 1.f, 0.f, 1.0f)));
    vertex.push_back(make_float3(xfm * make_float4(0.f, 0.f, 1.f, 1.0f)));
    vertex.push_back(make_float3(xfm * make_float4(1.f, 0.f, 1.f, 1.0f)));
    vertex.push_back(make_float3(xfm * make_float4(0.f, 1.f, 1.f, 1.0f)));
    vertex.push_back(make_float3(xfm * make_float4(1.f, 1.f, 1.f, 1.0f)));

    int indices[] = { 0,1,3, 2,3,0,
                     5,7,6, 5,6,4,
                     0,4,5, 0,5,1,
                     2,3,7, 2,7,6,
                     1,5,7, 1,7,3,
                     4,0,2, 4,2,6
    };
    for (int i = 0;i < 12;i++)
        index.push_back(make_int3(indices[3 * i + 0] + firstVertexID,
            indices[3 * i + 1] + firstVertexID,
            indices[3 * i + 2] + firstVertexID));
}

void TriangleMesh::addCube(const float3& center, const float3& size)
{
    sutil::Matrix<4, 4> matrix = sutil::Matrix<4, 4>::identity();
    matrix = matrix.translate(center - 0.5f * size) * matrix.scale(size);

    addUnitCube(matrix);
}
