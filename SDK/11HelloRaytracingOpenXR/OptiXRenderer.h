#include "SampleRenderer.h"

#include <sutil/sutil.h>
#include <sutil/CUDAOutputBuffer.h>

#include <sutil/GLDisplay.h>
#include <GLFW/glfw3.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

class OptiXRenderer {

    sutil::CUDAOutputBuffer<uint32_t>* output_buffer;
    SampleRenderer* sampleRenderer;
    sutil::GLDisplay* gl_display;
public:
    OptiXRenderer()
    {
        std::vector<TriangleMesh> model(2);
        // 100x100 thin ground plane
        model[0].color = make_float3(0.f, 1.f, 0.f);
        model[0].addCube(make_float3(0.f, -1.5f, 0.f), make_float3(10.f, .1f, 10.f));
        // a unit cube centered on top of that
        model[1].color = make_float3(0.f, 1.f, 1.f);
        model[1].addCube(make_float3(0.f, 0.f, 0.f), make_float3(2.f, 2.f, 2.f));

        sampleRenderer = new SampleRenderer(model);
        //sample->setCamera(camera);

        int2 fbSize = make_int2(1200, 1024);
        sampleRenderer->resize(fbSize);
        sampleRenderer->render();

        sutil::CUDAOutputBufferType output_buffer_type = sutil::CUDAOutputBufferType::GL_INTEROP;

        output_buffer = new sutil::CUDAOutputBuffer<uint32_t>(
            output_buffer_type,
            fbSize.x,
            fbSize.y
        );

        output_buffer->setStream(sampleRenderer->stream);
        gl_display = new sutil::GLDisplay();
    }

    void render(GLFWwindow* window, 
        const float3& position, 
        const float3& dir, 
        const float3& up,
        const int w,
        const int h,
        const int view_index,
        const GLuint framebuffer,
        const GLuint image,
        const bool depth_supported,
        const GLuint depthbuffer)
    {
        sampleRenderer->setCamera(position, dir, up);
        sampleRenderer->render(*output_buffer);        
        
        gl_display->display(
            output_buffer->width(),
            output_buffer->height(),
            w,
            h,
            output_buffer->getPBO()//,
            //framebuffer ,
           //image,
         //depth_supported,
        //depthbuffer
        );

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        if (view_index == 0) {

            glBindFramebuffer(GL_READ_FRAMEBUFFER, (GLuint)framebuffer);

            glBlitFramebuffer(
                (GLint)0,                        // srcX0
                (GLint)0,                        // srcY0
                (GLint)w,                        // srcX1
                (GLint)h,                        // srcY1
                (GLint)0,                        // dstX0
                (GLint)0,                        // dstY0
                (GLint)w / 2,                    // dstX1
                (GLint)h / 2,                    // dstY1
                (GLbitfield)GL_COLOR_BUFFER_BIT, // mask
                (GLenum)GL_LINEAR);              // filter

            glfwSwapBuffers(window);
        }
    }
};

