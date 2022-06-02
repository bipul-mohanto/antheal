#
# Copyright (c) 2021, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

#include_directories("${CMAKE_SOURCE_DIR}/support/glew/include")
include_directories("${CMAKE_SOURCE_DIR}/support/glad")
include_directories("${CMAKE_SOURCE_DIR}/support/GLFW/include")
include_directories("${CMAKE_SOURCE_DIR}/support/glm")
include_directories("${CMAKE_SOURCE_DIR}/support/openxr")
include_directories("${CMAKE_SOURCE_DIR}/support/glfWindow")
                    


OPTIX_add_sample_executable( optixPathTracer target_name
  optixPathTracer.cu
  optixPathTracer.cpp
  optixPathTracer.h
  OptiXRenderer.h
  DeviceSingleGPU.h
  DeviceSingleGPU.cpp
  OPTIONS -rdc true
  )

target_link_libraries( ${target_name}
  ${CUDA_LIBRARIES}
  )

target_link_libraries(${target_name}
                  #"${CMAKE_SOURCE_DIR}/support/glew/lib/Release/x64/glew32s.lib"
                  "${CMAKE_SOURCE_DIR}/lib/OpenGL32.lib"
                  "${CMAKE_SOURCE_DIR}/lib/GlU32.lib"
                  "${CMAKE_SOURCE_DIR}/lib/openxr_loader.lib"
)
