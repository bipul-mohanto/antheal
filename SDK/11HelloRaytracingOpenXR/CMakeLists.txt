include_directories(C:/vcpkg/installed/x64-windows/include/openxr)

OPTIX_add_sample_executable(11HelloRaytracingOpenXR target_name
  ${SAMPLES_CUDA_DIR}/helpers.h
  CUDABuffer.h
  LaunchParams.h
  main.cpp
  OptiXRenderer.h
  SampleRenderer.cpp
  SampleRenderer.h
  stb_image_write.h
  deviceProgram.cu    
  OPTIONS -rdc true # BM why?
  )

target_link_libraries( 11HelloRaytracingOpenXR 
                  "${CUDA_LIBRARIES}"
                  "C:/vcpkg/installed/x64-windows/lib/openxr_loader.lib"
    #debug ${CMAKE_SOURCE_DIR}/lib/openxr_loaderd.lib 
    #optimized ${CMAKE_SOURCE_DIR}/lib/openxr_loader.lib
                      )
    
