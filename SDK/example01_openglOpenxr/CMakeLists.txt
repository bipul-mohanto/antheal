# include_directories("C:/vcpkg/installed/x64-windows/include")
# include_directories("C:/Users/local-admin/Desktop/openxrOptix/common/3rdParty/glfw/include")
# include_directories("C:/vcpkg/installed/x64-windows/include/glm")

include_directories("${CMAKE_SOURCE_DIR}/support/glew/include")
include_directories("${CMAKE_SOURCE_DIR}/support/GLFW/include")
include_directories("${CMAKE_SOURCE_DIR}/support/glm")
#include_directories("${CMAKE_SOURCE_DIR}/support/openxr")
include_directories("${CMAKE_SOURCE_DIR}/support/glfWindow")
include_directories("C:/vcpkg/installed/x64-windows/include/openxr")                    



add_executable (example01_openglOpenxr
                main.cpp 
                GLSLShader.h
                GLSLShader.cpp
                shaders/simple.fragment.glsl
                shaders/simple.vertex.glsl
                #${embedded_ptx_code}
                #optix7.h
                #CUDABuffer.h
                #LaunchParams.h
                #SampleRenderer.h
                #SampleRenderer.cpp
                #Model.h
                #Model.cpp
                )


target_link_libraries(example01_openglOpenxr
                  "${CMAKE_SOURCE_DIR}/support/glew/lib/Release/x64/glew32s.lib"
                  "${CMAKE_SOURCE_DIR}/lib/OpenGL32.lib"
                  "${CMAKE_SOURCE_DIR}/lib/GlU32.lib"
                  "C:/vcpkg/installed/x64-windows/lib/openxr_loader.lib"
                  #"${CMAKE_SOURCE_DIR}/lib/openxr_loader.lib"
                  #"C:/vcpkg/installed/x64-windows/lib/OpenGL32.lib"
                  #"C:/vcpkg/installed/x64-windows/lib/GlU32.lib"

                  #gdt
# optix dependencies, for rendering
                  # ${optix_LIBRARY}
                  # ${CUDA_LIBRARIES}
                  # ${CUDA_CUDA_LIBRARY}
# glfw and opengl, for display
                  # glfWindow
                  # glfw
                  # ${OPENGL_gl_LIBRARY}
              )   