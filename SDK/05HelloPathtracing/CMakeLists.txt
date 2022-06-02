#[[cuda_add_library(toneMap
  toneMap.cu
  toneMap.h)
]]

OPTIX_add_sample_executable(05HelloPathtracing target_name
  #${SAMPLES_CUDA_DIR}/helpers.h
  LaunchParams.h
  main.cpp
  SimplePathtracer.cpp
  SimplePathtracer.h   
  Model.cpp
  Model.h
  Probe.h  
  maths.h
  sample.h
  #OptixDenoiser.h
  #OptixDenoiser.cpp
  Material.h

  deviceProgram.cu  
  Probe.cuh
  Disney.cuh
  toneMap.cu
  toneMap.h
  
  OPTIONS -rdc true
  )

target_link_libraries( ${target_name}
  ${CUDA_LIBRARIES}
  #toneMap
  #OptixUtils
  )