/*!
* this code is based on:
 * @brief A simple, commented, (almost) single file OpenXR example
 * @author Christoph Haag <christoph.haag@collabora.com>
 */


#include "glad/glad.h"

#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WGL
#include "GLFW/glfw3.h"
#include "GLFW/glfw3native.h"

#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_OPENGL

#include "Unknwn.h"

#include "openxr/openxr.h"
#include "openxr/openxr_platform.h"

#include "glm/glm.hpp"
#include "glm/gtx/quaternion.hpp"
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/matrix_clip_space.hpp>
#include "glm/gtc/type_ptr.hpp"

#include "OptiXRenderer.h"

// we need an identity pose for creating spaces without offsets
static XrPosef identity_pose = { {0, 0, 0, 1.0},
                                {0,  0, 0 } };

#define HAND_LEFT_INDEX 0
#define HAND_RIGHT_INDEX 1
#define HAND_COUNT 2

bool init_glfw_window(GLFWwindow** window, HDC* hdc, HGLRC* hglrc, int w, int h);

int init_gl(uint32_t view_count,
    uint32_t* swapchain_lengths,
    GLuint*** framebuffers,
    GLuint* shader_program_id,
    GLuint* VAO);

void
render_frame(GLFWwindow*,
    int w,
    int h,
    GLuint shader_program_id,
    GLuint VAO,
    XrTime predictedDisplayTime,
    int view_index,
    glm::mat4 projectionmatrix,
    glm::mat4 viewmatrix,
    GLuint framebuffer,
    GLuint image,
    bool depth_supported,
    GLuint depthbuffer);

inline static void
Mat4_CreateProjectionFov(glm::mat4 &result,
    const XrFovf fov,
    const float nearZ,
    const float farZ)
{
    const float tanAngleLeft = tanf(fov.angleLeft);
    const float tanAngleRight = tanf(fov.angleRight);

    const float tanAngleDown = tanf(fov.angleDown);
    const float tanAngleUp = tanf(fov.angleUp);

    const float tanAngleWidth = tanAngleRight - tanAngleLeft;

    // Set to tanAngleDown - tanAngleUp for a clip space with positive Y
    // down (Vulkan). Set to tanAngleUp - tanAngleDown for a clip space with
    // positive Y up (OpenGL / D3D / Metal).
    const float tanAngleHeight = (tanAngleUp - tanAngleDown);

    // Set to nearZ for a [-1,1] Z clip space (OpenGL / OpenGL ES).
    // Set to zero for a [0,1] Z clip space (Vulkan / D3D / Metal).
    const float offsetZ = nearZ;

    if (farZ <= nearZ) {
        // place the far plane at infinity
        result[0][0] = 2 / tanAngleWidth;
        result[0][1] = 0;
        result[0][2] = (tanAngleRight + tanAngleLeft) / tanAngleWidth;
        result[0][3] = 0;

        result[1][0] = 0;
        result[1][1] = 2 / tanAngleHeight;
        result[1][2] = (tanAngleUp + tanAngleDown) / tanAngleHeight;
        result[1][3] = 0;

        result[2][0] = 0;
        result[2][1] = 0;
        result[2][2] = -1;
        result[2][3] = -(nearZ + offsetZ);

        result[3][0] = 0;
        result[3][1] = 0;
        result[3][2] = -1;
        result[3][3] = 0;
    }
    else {
        // normal projection
        result[0][0] = 2 / tanAngleWidth;
        result[0][1] = 0;
        result[0][2] = (tanAngleRight + tanAngleLeft) / tanAngleWidth;
        result[0][3] = 0;

        result[1][0] = 0;
        result[1][1] = 2 / tanAngleHeight;
        result[1][2] = (tanAngleUp + tanAngleDown) / tanAngleHeight;
        result[1][3] = 0;

        result[2][0] = 0;
        result[2][1] = 0;
        result[2][2] = -(farZ + offsetZ) / (farZ - nearZ);
        result[2][3] = -(farZ * (nearZ + offsetZ)) / (farZ - nearZ);

        result[3][0] = 0;
        result[3][1] = 0;
        result[3][2] = -1;
        result[3][3] = 0;
    }
}


// true if XrResult is a success code, else print error message and return false
bool
xr_check(XrInstance instance, XrResult result, const char* format, ...)
{
    if (XR_SUCCEEDED(result))
        return true;

    char resultString[XR_MAX_RESULT_STRING_SIZE];
    xrResultToString(instance, result, resultString);

    char formatRes[XR_MAX_RESULT_STRING_SIZE + 1024];
    snprintf(formatRes, XR_MAX_RESULT_STRING_SIZE + 1023, "%s [%s] (%d)\n", format, resultString,
        result);

    va_list args;
    va_start(args, format);
    vprintf(formatRes, args);
    va_end(args);

    return false;
}

static void
print_instance_properties(XrInstance instance)
{
    XrResult result;
    XrInstanceProperties instance_props = {
        XR_TYPE_INSTANCE_PROPERTIES,
        NULL,
    };

    result = xrGetInstanceProperties(instance, &instance_props);
    if (!xr_check(NULL, result, "Failed to get instance info"))
        return;

    printf("Runtime Name: %s\n", instance_props.runtimeName);
    printf("Runtime Version: %d.%d.%d\n", XR_VERSION_MAJOR(instance_props.runtimeVersion),
        XR_VERSION_MINOR(instance_props.runtimeVersion),
        XR_VERSION_PATCH(instance_props.runtimeVersion));
}


static void
print_system_properties(XrSystemProperties* system_properties)
{
    printf("System properties for system %lu: \"%s\", vendor ID %d\n", system_properties->systemId,
        system_properties->systemName, system_properties->vendorId);
    printf("\tMax layers          : %d\n", system_properties->graphicsProperties.maxLayerCount);
    printf("\tMax swapchain height: %d\n",
        system_properties->graphicsProperties.maxSwapchainImageHeight);
    printf("\tMax swapchain width : %d\n",
        system_properties->graphicsProperties.maxSwapchainImageWidth);
    printf("\tOrientation Tracking: %d\n", system_properties->trackingProperties.orientationTracking);
    printf("\tPosition Tracking   : %d\n", system_properties->trackingProperties.positionTracking);
}

static void
print_viewconfig_view_info(uint32_t view_count, XrViewConfigurationView* viewconfig_views)
{
    for (uint32_t i = 0; i < view_count; i++) {
        printf("View Configuration View %d:\n", i);
        printf("\tResolution       : Recommended %dx%d, Max: %dx%d\n",
            viewconfig_views[0].recommendedImageRectWidth,
            viewconfig_views[0].recommendedImageRectHeight, viewconfig_views[0].maxImageRectWidth,
            viewconfig_views[0].maxImageRectHeight);
        printf("\tSwapchain Samples: Recommended: %d, Max: %d)\n",
            viewconfig_views[0].recommendedSwapchainSampleCount,
            viewconfig_views[0].maxSwapchainSampleCount);
    }
}

// returns the preferred swapchain format if it is supported
// else:
// - if fallback is true, return the first supported format
// - if fallback is false, return -1
static int64_t
get_swapchain_format(XrInstance instance,
    XrSession session,
    int64_t preferred_format,
    bool fallback)
{
    XrResult result;

    uint32_t swapchain_format_count;
    result = xrEnumerateSwapchainFormats(session, 0, &swapchain_format_count, NULL);
    if (!xr_check(instance, result, "Failed to get number of supported swapchain formats"))
        return -1;

    printf("Runtime supports %d swapchain formats\n", swapchain_format_count);
    int64_t* swapchain_formats = (int64_t*)malloc(sizeof(int64_t) * swapchain_format_count);
    result = xrEnumerateSwapchainFormats(session, swapchain_format_count, &swapchain_format_count,
        swapchain_formats);
    if (!xr_check(instance, result, "Failed to enumerate swapchain formats"))
        return -1;

    int64_t chosen_format = fallback ? swapchain_formats[0] : -1;

    for (uint32_t i = 0; i < swapchain_format_count; i++) {
        printf("Supported GL format: %#lx\n", swapchain_formats[i]);
        if (swapchain_formats[i] == preferred_format) {
            chosen_format = swapchain_formats[i];
            printf("Using preferred swapchain format %#lx\n", chosen_format);
            break;
        }
    }
    if (fallback && chosen_format != preferred_format) {
        printf("Falling back to non preferred swapchain format %#lx\n", chosen_format);
    }

    free(swapchain_formats);

    return chosen_format;
}

static void
print_api_layers()
{
    uint32_t count = 0;
    XrResult result = xrEnumerateApiLayerProperties(0, &count, NULL);
    if (!xr_check(NULL, result, "Failed to enumerate api layer count"))
        return;

    if (count == 0)
        return;

    XrApiLayerProperties* props = (XrApiLayerProperties * )malloc(count * sizeof(XrApiLayerProperties));
    for (uint32_t i = 0; i < count; i++) {
        props[i].type = XR_TYPE_API_LAYER_PROPERTIES;
        props[i].next = NULL;
    }

    result = xrEnumerateApiLayerProperties(count, &count, props);
    if (!xr_check(NULL, result, "Failed to enumerate api layers"))
        return;

    printf("API layers:\n");
    for (uint32_t i = 0; i < count; i++) {
        printf("\t%s v%d: %s\n", props[i].layerName, props[i].layerVersion, props[i].description);
    }

    free(props);
}

// functions belonging to extensions must be loaded with xrGetInstanceProcAddr before use
static PFN_xrGetOpenGLGraphicsRequirementsKHR pfnGetOpenGLGraphicsRequirementsKHR = NULL;
static bool
load_extension_function_pointers(XrInstance instance)
{
    XrResult result =
        xrGetInstanceProcAddr(instance, "xrGetOpenGLGraphicsRequirementsKHR",
            (PFN_xrVoidFunction*)&pfnGetOpenGLGraphicsRequirementsKHR);
    if (!xr_check(instance, result, "Failed to get OpenGL graphics requirements function!"))
        return false;

    return true;
}

int
main(int argc, char** argv)
{
    GLFWwindow* window;

    // Changing to HANDHELD_DISPLAY or a future form factor may work, but has not been tested.
    XrFormFactor form_factor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;

    // Changing the form_factor may require changing the view_type too.
    XrViewConfigurationType view_type = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;

    // Typically STAGE for room scale/standing, LOCAL for seated
    XrReferenceSpaceType play_space_type = XR_REFERENCE_SPACE_TYPE_LOCAL;
    XrSpace play_space = XR_NULL_HANDLE;

    // the instance handle can be thought of as the basic connection to the OpenXR runtime
    XrInstance instance = XR_NULL_HANDLE;
    // the system represents an (opaque) set of XR devices in use, managed by the runtime
    XrSystemId system_id = XR_NULL_SYSTEM_ID;
    // the session deals with the renderloop submitting frames to the runtime
    XrSession session = XR_NULL_HANDLE;

    // each graphics API requires the use of a specialized struct
    XrGraphicsBindingOpenGLWin32KHR graphics_binding_gl;

    // each physical Display/Eye is described by a view.
    // view_count usually depends on the form_factor / view_type.
    // dynamically allocating all view related structs instead of assuming 2
    // hopefully allows this app to scale easily to different view_counts.
    uint32_t view_count = 0;
    // the viewconfiguration views contain information like resolution about each view
    XrViewConfigurationView* viewconfig_views = NULL;

    // array of view_count containers for submitting swapchains with rendered VR frames
    XrCompositionLayerProjectionView* projection_views = NULL;
    // array of view_count views, filled by the runtime with current HMD display pose
    XrView* views = NULL;

    // array of view_count handles for swapchains.
    // it is possible to use imageRect to render all views to different areas of the
    // same texture, but in this example we use one swapchain per view
    XrSwapchain* swapchains = NULL;
    // array of view_count ints, storing the length of swapchains
    uint32_t* swapchain_lengths = NULL;
    // array of view_count array of swapchain_length containers holding an OpenGL texture
    // that is allocated by the runtime
    XrSwapchainImageOpenGLKHR** images = NULL;

    // depth swapchain equivalent to the VR color swapchains
    XrSwapchain* depth_swapchains = NULL;
    uint32_t* depth_swapchain_lengths = NULL;
    XrSwapchainImageOpenGLKHR** depth_images = NULL;

    XrPath hand_paths[HAND_COUNT];

    struct
    {
        // supporting depth layers is *optional* for runtimes
        bool supported;
        XrCompositionLayerDepthInfoKHR* infos;
    } depth;

    struct
    {
        // To render into a texture we need a framebuffer (one per texture to make it easy)
        GLuint** framebuffers;

        float near_z;
        float far_z;

        GLuint shader_program_id;
        GLuint VAO;
    } gl_rendering;
    gl_rendering.near_z = 0.01f;
    gl_rendering.far_z = 100.0f;

    // reuse this variable for all our OpenXR return codes
    XrResult result = XR_SUCCESS;

    print_api_layers();

    // xrEnumerate*() functions are usually called once with CapacityInput = 0.
    // The function will write the required amount into CountOutput. We then have
    // to allocate an array to hold CountOutput elements and call the function
    // with CountOutput as CapacityInput.
    uint32_t ext_count = 0;
    result = xrEnumerateInstanceExtensionProperties(NULL, 0, &ext_count, NULL);

    /* TODO: instance null will not be able to convert XrResult to string */
    if (!xr_check(NULL, result, "Failed to enumerate number of extension properties"))
        return 1;

    XrExtensionProperties* ext_props = (XrExtensionProperties * )malloc(sizeof(XrExtensionProperties) * ext_count);
    for (uint16_t i = 0; i < ext_count; i++) {
        // we usually have to fill in the type (for validation) and set
        // next to NULL (or a pointer to an extension specific struct)
        ext_props[i].type = XR_TYPE_EXTENSION_PROPERTIES;
        ext_props[i].next = NULL;
    }

    result = xrEnumerateInstanceExtensionProperties(NULL, ext_count, &ext_count, ext_props);
    if (!xr_check(NULL, result, "Failed to enumerate extension properties"))
        return 1;

    bool opengl_supported = false;

    printf("Runtime supports %d extensions\n", ext_count);
    for (uint32_t i = 0; i < ext_count; i++) {
        printf("\t%s v%d\n", ext_props[i].extensionName, ext_props[i].extensionVersion);
        if (strcmp(XR_KHR_OPENGL_ENABLE_EXTENSION_NAME, ext_props[i].extensionName) == 0) {
            opengl_supported = true;
        }

        if (strcmp(XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME, ext_props[i].extensionName) == 0) {
            depth.supported = true;
        }
    }
    free(ext_props);

    // A graphics extension like OpenGL is required to draw anything in VR
    if (!opengl_supported) {
        printf("Runtime does not support OpenGL extension!\n");
        return 1;
    }


    // --- Create XrInstance
    int enabled_ext_count = 1;
    const char* enabled_exts[1] = { XR_KHR_OPENGL_ENABLE_EXTENSION_NAME };
    // same can be done for API layers, but API layers can also be enabled by env var

    XrInstanceCreateInfo instance_create_info;// = {
    instance_create_info.type = XR_TYPE_INSTANCE_CREATE_INFO;
    instance_create_info.next = NULL;
    instance_create_info.createFlags = 0;
    instance_create_info.enabledExtensionCount = enabled_ext_count;
    instance_create_info.enabledExtensionNames = enabled_exts;
    instance_create_info.enabledApiLayerCount = 0;
    instance_create_info.enabledApiLayerNames = NULL;
    instance_create_info.applicationInfo;
            
    // some compilers have trouble with char* initialization
    instance_create_info.applicationInfo.applicationVersion = 1;
    instance_create_info.applicationInfo.engineVersion = 0;
    instance_create_info.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;

    strncpy(instance_create_info.applicationInfo.applicationName, "OpenXR OpenGL Example", XR_MAX_APPLICATION_NAME_SIZE);
    strncpy(instance_create_info.applicationInfo.engineName, "Custom", XR_MAX_ENGINE_NAME_SIZE);

    result = xrCreateInstance(&instance_create_info, &instance);
    if (!xr_check(NULL, result, "Failed to create XR instance."))
        return 1;

    if (!load_extension_function_pointers(instance))
        return 1;

    // Optionally get runtime name and version
    print_instance_properties(instance);

    // --- Get XrSystemId
    XrSystemGetInfo system_get_info = {
        XR_TYPE_SYSTEM_GET_INFO,  NULL, form_factor };

    result = xrGetSystem(instance, &system_get_info, &system_id);
    if (!xr_check(instance, result, "Failed to get system for HMD form factor."))
        return 1;

    printf("Successfully got XrSystem with id %lu for HMD form factor\n", system_id);

    {
        XrSystemProperties system_props = {
            XR_TYPE_SYSTEM_PROPERTIES,
            NULL,
        };

        result = xrGetSystemProperties(instance, system_id, &system_props);
        if (!xr_check(instance, result, "Failed to get System properties"))
            return 1;

        print_system_properties(&system_props);
    }

    result = xrEnumerateViewConfigurationViews(instance, system_id, view_type, 0, &view_count, NULL);
    if (!xr_check(instance, result, "Failed to get view configuration view count!"))
        return 1;

    viewconfig_views = (XrViewConfigurationView*)malloc(sizeof(XrViewConfigurationView) * view_count);
    for (uint32_t i = 0; i < view_count; i++) {
        viewconfig_views[i].type = XR_TYPE_VIEW_CONFIGURATION_VIEW;
        viewconfig_views[i].next = NULL;
    }

    result = xrEnumerateViewConfigurationViews(instance, system_id, view_type, view_count, &view_count, viewconfig_views);
    if (!xr_check(instance, result, "Failed to enumerate view configuration views!"))
        return 1;
    print_viewconfig_view_info(view_count, viewconfig_views);

    // OpenXR requires checking graphics requirements before creating a session.
    XrGraphicsRequirementsOpenGLKHR opengl_reqs = { XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR, NULL };

    // this function pointer was loaded with xrGetInstanceProcAddr
    result = pfnGetOpenGLGraphicsRequirementsKHR(instance, system_id, &opengl_reqs);
    if (!xr_check(instance, result, "Failed to get OpenGL graphics requirements!"))
        return 1;

    /* Checking opengl_reqs.minApiVersionSupported and opengl_reqs.maxApiVersionSupported
     * is not very useful, compatibility will depend on the OpenGL implementation and the
     * OpenXR runtime much more than the OpenGL version.
     * Other APIs have more useful verifiable requirements. */


     // --- Create session
    graphics_binding_gl.type = XR_TYPE_GRAPHICS_BINDING_OPENGL_WIN32_KHR;
    graphics_binding_gl.next = 0;

    if (!init_glfw_window(&window, &graphics_binding_gl.hDC, &graphics_binding_gl.hGLRC,
        viewconfig_views[0].recommendedImageRectWidth,
        viewconfig_views[0].recommendedImageRectHeight))
    {
        printf("GLFW init failed!\n");
        return 1;
    }

    printf("Using OpenGL version: %s\n", glGetString(GL_VERSION));
    printf("Using OpenGL Renderer: %s\n", glGetString(GL_RENDERER));

    XrSessionCreateInfo session_create_info = {
        XR_TYPE_SESSION_CREATE_INFO, &graphics_binding_gl, 0, system_id };

    result = xrCreateSession(instance, &session_create_info, &session);
    if (!xr_check(instance, result, "Failed to create session"))
        return 1;

    printf("Successfully created a session with OpenGL!\n");

    /* Many runtimes support at least STAGE and LOCAL but not all do.
     * Sophisticated apps might check with xrEnumerateReferenceSpaces() if the
     * chosen one is supported and try another one if not.
     * Here we will get an error from xrCreateReferenceSpace() and exit. */
    XrReferenceSpaceCreateInfo play_space_create_info = { XR_TYPE_REFERENCE_SPACE_CREATE_INFO,
                                                         NULL,
                                                         play_space_type,
                                                         identity_pose };

    result = xrCreateReferenceSpace(session, &play_space_create_info, &play_space);
    if (!xr_check(instance, result, "Failed to create play space!"))
        return 1;

    // --- Create Swapchains
    uint32_t swapchain_format_count;
    result = xrEnumerateSwapchainFormats(session, 0, &swapchain_format_count, NULL);
    if (!xr_check(instance, result, "Failed to get number of supported swapchain formats"))
        return 1;

    printf("Runtime supports %d swapchain formats\n", swapchain_format_count);
    int64_t* swapchain_formats = (int64_t*)malloc(sizeof(int64_t) * swapchain_format_count);
    result = xrEnumerateSwapchainFormats(session, swapchain_format_count, &swapchain_format_count,
        swapchain_formats);
    if (!xr_check(instance, result, "Failed to enumerate swapchain formats"))
        return 1;

    // SRGB is usually a better choice than linear
    // a more sophisticated approach would iterate supported swapchain formats and choose from them
    int64_t color_format = get_swapchain_format(instance, session, GL_SRGB8_ALPHA8, true);

    // GL_DEPTH_COMPONENT16 is a good bet
    // SteamVR 1.16.4 supports GL_DEPTH_COMPONENT16, GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT32
    // but NOT GL_DEPTH_COMPONENT32F
    int64_t depth_format = get_swapchain_format(instance, session, GL_DEPTH_COMPONENT16, false);
    if (depth_format < 0) {
        printf("Preferred depth format GL_DEPTH_COMPONENT16 not supported, disabling depth\n");
        depth.supported = false;
    }

    // --- Create swapchain for main VR rendering
    {
        // In the frame loop we render into OpenGL textures we receive from the runtime here.
        swapchains = (XrSwapchain*)malloc(sizeof(XrSwapchain) * view_count);
        swapchain_lengths = (uint32_t*)malloc(sizeof(uint32_t) * view_count);
        images = (XrSwapchainImageOpenGLKHR**)malloc(sizeof(XrSwapchainImageOpenGLKHR*) * view_count);
        for (uint32_t i = 0; i < view_count; i++) {
            XrSwapchainCreateInfo swapchain_create_info;
            swapchain_create_info.type = XR_TYPE_SWAPCHAIN_CREATE_INFO,
            swapchain_create_info.usageFlags = XR_SWAPCHAIN_USAGE_SAMPLED_BIT | XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
            swapchain_create_info.createFlags = 0;
            swapchain_create_info.format = color_format;
            swapchain_create_info.sampleCount = viewconfig_views[i].recommendedSwapchainSampleCount;
            swapchain_create_info.width = viewconfig_views[i].recommendedImageRectWidth;
            swapchain_create_info.height = viewconfig_views[i].recommendedImageRectHeight;
            swapchain_create_info.faceCount = 1;
            swapchain_create_info.arraySize = 1;
            swapchain_create_info.mipCount = 1;
            swapchain_create_info.next = NULL;
            

            result = xrCreateSwapchain(session, &swapchain_create_info, &swapchains[i]);
            if (!xr_check(instance, result, "Failed to create swapchain %d!", i))
                return 1;

            // The runtime controls how many textures we have to be able to render to
            // (e.g. "triple buffering")
            result = xrEnumerateSwapchainImages(swapchains[i], 0, &swapchain_lengths[i], NULL);
            if (!xr_check(instance, result, "Failed to enumerate swapchains"))
                return 1;

            images[i] = (XrSwapchainImageOpenGLKHR*)malloc(sizeof(XrSwapchainImageOpenGLKHR) * swapchain_lengths[i]);
            for (uint32_t j = 0; j < swapchain_lengths[i]; j++) {
                images[i][j].type = XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR;
                images[i][j].next = NULL;
            }
            result =
                xrEnumerateSwapchainImages(swapchains[i], swapchain_lengths[i], &swapchain_lengths[i],
                    (XrSwapchainImageBaseHeader*)images[i]);
            if (!xr_check(instance, result, "Failed to enumerate swapchain images"))
                return 1;
        }
    }

    // --- Create swapchain for depth buffers if supported
    {
        if (depth.supported) {
            depth_swapchains = (XrSwapchain*)malloc(sizeof(XrSwapchain) * view_count);
            depth_swapchain_lengths = (uint32_t*)malloc(sizeof(uint32_t) * view_count);
            depth_images = (XrSwapchainImageOpenGLKHR**)malloc(sizeof(XrSwapchainImageOpenGLKHR*) * view_count);
            for (uint32_t i = 0; i < view_count; i++) {
                XrSwapchainCreateInfo swapchain_create_info;
                swapchain_create_info.type = XR_TYPE_SWAPCHAIN_CREATE_INFO;
                swapchain_create_info.usageFlags = XR_SWAPCHAIN_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
                swapchain_create_info.createFlags = 0;
                swapchain_create_info.format = depth_format;
                swapchain_create_info.sampleCount = viewconfig_views[i].recommendedSwapchainSampleCount;
                swapchain_create_info.width = viewconfig_views[i].recommendedImageRectWidth;
                swapchain_create_info.height = viewconfig_views[i].recommendedImageRectHeight;
                swapchain_create_info.faceCount = 1;
                swapchain_create_info.arraySize = 1;
                swapchain_create_info.mipCount = 1;
                swapchain_create_info.next = NULL;
              

                result = xrCreateSwapchain(session, &swapchain_create_info, &depth_swapchains[i]);
                if (!xr_check(instance, result, "Failed to create swapchain %d!", i))
                    return 1;

                result =
                    xrEnumerateSwapchainImages(depth_swapchains[i], 0, &depth_swapchain_lengths[i], NULL);
                if (!xr_check(instance, result, "Failed to enumerate swapchains"))
                    return 1;

                // these are wrappers for the actual OpenGL texture id
                depth_images[i] = (XrSwapchainImageOpenGLKHR*)malloc(sizeof(XrSwapchainImageOpenGLKHR) * depth_swapchain_lengths[i]);
                for (uint32_t j = 0; j < depth_swapchain_lengths[i]; j++) {
                    depth_images[i][j].type = XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR;
                    depth_images[i][j].next = NULL;
                }
                result = xrEnumerateSwapchainImages(depth_swapchains[i], depth_swapchain_lengths[i],
                    &depth_swapchain_lengths[i],
                    (XrSwapchainImageBaseHeader*)depth_images[i]);
                if (!xr_check(instance, result, "Failed to enumerate swapchain images"))
                    return 1;
            }            
        }
    }

    // Do not allocate these every frame to save some resources
    views = (XrView*)malloc(sizeof(XrView) * view_count);
    for (uint32_t i = 0; i < view_count; i++) {
        views[i].type = XR_TYPE_VIEW;
        views[i].next = NULL;
    }

    projection_views = (XrCompositionLayerProjectionView*)malloc(
        sizeof(XrCompositionLayerProjectionView) * view_count);
    for (uint32_t i = 0; i < view_count; i++) {
        projection_views[i].type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW;
        projection_views[i].next = NULL;

        projection_views[i].subImage.swapchain = swapchains[i];
        projection_views[i].subImage.imageArrayIndex = 0;
        projection_views[i].subImage.imageRect.offset.x = 0;
        projection_views[i].subImage.imageRect.offset.y = 0;
        projection_views[i].subImage.imageRect.extent.width =
            viewconfig_views[i].recommendedImageRectWidth;
        projection_views[i].subImage.imageRect.extent.height =
            viewconfig_views[i].recommendedImageRectHeight;

        // projection_views[i].{pose, fov} have to be filled every frame in frame loop
    };

    if (depth.supported) {
        depth.infos = (XrCompositionLayerDepthInfoKHR*)malloc(sizeof(XrCompositionLayerDepthInfoKHR) *
            view_count);
        for (uint32_t i = 0; i < view_count; i++) {
            depth.infos[i].type = XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR;
            depth.infos[i].next = NULL;
            depth.infos[i].minDepth = 0.f;
            depth.infos[i].maxDepth = 1.f;
            depth.infos[i].nearZ = gl_rendering.near_z;
            depth.infos[i].farZ = gl_rendering.far_z;

            depth.infos[i].subImage.swapchain = depth_swapchains[i];
            depth.infos[i].subImage.imageArrayIndex = 0;
            depth.infos[i].subImage.imageRect.offset.x = 0;
            depth.infos[i].subImage.imageRect.offset.y = 0;
            depth.infos[i].subImage.imageRect.extent.width =
                viewconfig_views[i].recommendedImageRectWidth;
            depth.infos[i].subImage.imageRect.extent.height =
                viewconfig_views[i].recommendedImageRectHeight;

            // depth is chained to projection, not submitted as separate layer
            projection_views[i].next = &depth.infos[i];
        };
    }

    // Set up rendering (compile shaders, ...) before starting the session
    if (init_gl(view_count, swapchain_lengths, &gl_rendering.framebuffers,
        &gl_rendering.shader_program_id, &gl_rendering.VAO) != 0) {
        printf("OpenGl setup failed!\n");
        return 1;
    }

    XrSessionState state = XR_SESSION_STATE_UNKNOWN;

    OptiXRenderer optixrenderer;

    bool session_running = false; // to avoid beginning an already running session
    bool run_framecycle = false;  // for some session states skip the frame cycle
    while (!glfwWindowShouldClose(window)) {

        // --- Handle runtime Events
        // we do this before xrWaitFrame() so we can go idle or
        // break out of the main render loop as early as possible and don't have to
        // uselessly render or submit one. Calling xrWaitFrame commits you to
        // calling xrBeginFrame eventually.
        XrEventDataBuffer runtime_event = { XR_TYPE_EVENT_DATA_BUFFER, NULL };
        XrResult poll_result = xrPollEvent(instance, &runtime_event);
        while (poll_result == XR_SUCCESS) {
            switch (runtime_event.type) {
            case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING: {
                XrEventDataInstanceLossPending* event = (XrEventDataInstanceLossPending*)&runtime_event;
                printf("EVENT: instance loss pending at %lu! Destroying instance.\n", event->lossTime);
                glfwSetWindowShouldClose(window, TRUE);
                continue;
            }
            case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
                XrEventDataSessionStateChanged* event = (XrEventDataSessionStateChanged*)&runtime_event;
                printf("EVENT: session state changed from %d to %d\n", state, event->state);
                state = event->state;

                /*
                 * react to session state changes, see OpenXR spec 9.3 diagram. What we need to react to:
                 *
                 * * READY -> xrBeginSession STOPPING -> xrEndSession (note that the same session can be restarted)
                 * * EXITING -> xrDestroySession (EXITING only happens after we went through STOPPING and called xrEndSession)
                 *
                 * After exiting it is still possible to create a new session but we don't do that here.
                 *
                 * * IDLE -> don't run render loop, but keep polling for events
                 * * SYNCHRONIZED, VISIBLE, FOCUSED -> run render loop
                 */
                switch (state) {
                    // skip render loop, keep polling
                case XR_SESSION_STATE_MAX_ENUM: // must be a bug
                case XR_SESSION_STATE_IDLE:
                case XR_SESSION_STATE_UNKNOWN: {
                    run_framecycle = false;

                    break; // state handling switch
                }

                                             // do nothing, run render loop normally
                case XR_SESSION_STATE_FOCUSED:
                case XR_SESSION_STATE_SYNCHRONIZED:
                case XR_SESSION_STATE_VISIBLE: {
                    run_framecycle = true;

                    break; // state handling switch
                }

                                             // begin session and then run render loop
                case XR_SESSION_STATE_READY: {
                    // start session only if it is not running, i.e. not when we already called xrBeginSession
                    // but the runtime did not switch to the next state yet
                    if (!session_running) {
                        XrSessionBeginInfo session_begin_info = { XR_TYPE_SESSION_BEGIN_INFO,
                                                                 NULL,
                                                                 view_type };
                        result = xrBeginSession(session, &session_begin_info);
                        if (!xr_check(instance, result, "Failed to begin session!"))
                            return 1;
                        printf("Session started!\n");
                        session_running = true;
                    }
                    // after beginning the session, run render loop
                    run_framecycle = true;

                    break; // state handling switch
                }

                                           // end session, skip render loop, keep polling for next state change
                case XR_SESSION_STATE_STOPPING: {
                    // end session only if it is running, i.e. not when we already called xrEndSession but the
                    // runtime did not switch to the next state yet
                    if (session_running) {
                        result = xrEndSession(session);
                        if (!xr_check(instance, result, "Failed to end session!"))
                            return 1;
                        session_running = false;
                    }
                    // after ending the session, don't run render loop
                    run_framecycle = false;

                    break; // state handling switch
                }

                                              // destroy session, skip render loop, exit render loop and quit
                case XR_SESSION_STATE_LOSS_PENDING:
                case XR_SESSION_STATE_EXITING:
                    result = xrDestroySession(session);
                    if (!xr_check(instance, result, "Failed to destroy session!"))
                        return 1;
                    glfwSetWindowShouldClose(window, true);
                    run_framecycle = false;

                    break; // state handling switch
                }
                break; // session event handling switch
            }
            case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED: {
                printf("EVENT: interaction profile changed!\n");
                XrEventDataInteractionProfileChanged* event =
                    (XrEventDataInteractionProfileChanged*)&runtime_event;
                (void)event;

                XrInteractionProfileState state = { XR_TYPE_INTERACTION_PROFILE_STATE, NULL };

                for (int i = 0; i < HAND_COUNT; i++) {
                    XrResult res = xrGetCurrentInteractionProfile(session, hand_paths[i], &state);
                    if (!xr_check(instance, res, "Failed to get interaction profile for %d", i))
                        continue;

                    XrPath prof = state.interactionProfile;

                    uint32_t strl;
                    char profile_str[XR_MAX_PATH_LENGTH];
                    res = xrPathToString(instance, prof, XR_MAX_PATH_LENGTH, &strl, profile_str);
                    if (!xr_check(instance, res, "Failed to get interaction profile path str for %d", i))
                        continue;

                    printf("Event: Interaction profile changed for %d: %s\n", i, profile_str);
                }
                break;
            }
            default: printf("Unhandled event (type %d)\n", runtime_event.type);
            }

            runtime_event.type = XR_TYPE_EVENT_DATA_BUFFER;
            poll_result = xrPollEvent(instance, &runtime_event);
        }
        if (poll_result == XR_EVENT_UNAVAILABLE) {
            // processed all events in the queue
        }
        else {
            printf("Failed to poll events!\n");
            break;
        }

        if (!run_framecycle) {
            continue;
        }

        // --- Wait for our turn to do head-pose dependent computation and render a frame
        XrFrameState frame_state = {XR_TYPE_FRAME_STATE, NULL };
        XrFrameWaitInfo frame_wait_info = { XR_TYPE_FRAME_WAIT_INFO, NULL };
        result = xrWaitFrame(session, &frame_wait_info, &frame_state);
        if (!xr_check(instance, result, "xrWaitFrame() was not successful, exiting..."))
            break;

        // --- Create projection matrices and view matrices for each eye
        XrViewLocateInfo view_locate_info = { XR_TYPE_VIEW_LOCATE_INFO,
                                             NULL,                                             
                                             XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
                                             frame_state.predictedDisplayTime,
                                             play_space };

        XrViewState view_state = { XR_TYPE_VIEW_STATE, NULL };
        result = xrLocateViews(session, &view_locate_info, &view_state, view_count, &view_count, views);
        if (!xr_check(instance, result, "Could not locate views"))
            break;

        // --- Begin frame
        XrFrameBeginInfo frame_begin_info = { XR_TYPE_FRAME_BEGIN_INFO, NULL };

        result = xrBeginFrame(session, &frame_begin_info);
        if (!xr_check(instance, result, "failed to begin frame!"))
            break;

        // render each eye and fill projection_views with the result
        for (uint32_t i = 0; i < view_count; i++) {

            if (!frame_state.shouldRender) {
                printf("shouldRender = false, Skipping rendering work\n");
                continue;
            }
            const float tanAngleLeft = tanf(views[i].fov.angleLeft);
            const float tanAngleRight = tanf(views[i].fov.angleRight);

            const float tanAngleDown = tanf(views[i].fov.angleDown);
            const float tanAngleUp = tanf(views[i].fov.angleUp);

            const float tanAngleWidth = tanAngleRight - tanAngleLeft;

            float aspect = (float)viewconfig_views[i].recommendedImageRectWidth / (float)viewconfig_views[i].recommendedImageRectHeight;
            glm::mat4 projection_matrix = glm::perspective(tanAngleWidth, aspect, gl_rendering.near_z, gl_rendering.far_z);           
            glm::mat4 projectionFOV = glm::mat4(1.0);
            Mat4_CreateProjectionFov(projectionFOV, views[i].fov, gl_rendering.near_z, gl_rendering.far_z);

            glm::mat4 rotationMatrix = glm::toMat4(glm::quat(views[i].pose.orientation.w, views[i].pose.orientation.x, views[i].pose.orientation.y, views[i].pose.orientation.z));
            glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(views[i].pose.position.x, views[i].pose.position.y, views[i].pose.position.z));
            glm::mat4 view_matrix = translationMatrix * rotationMatrix;
            

            XrSwapchainImageAcquireInfo acquire_info = { XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO,
                                                        NULL };
            uint32_t acquired_index;
            result = xrAcquireSwapchainImage(swapchains[i], &acquire_info, &acquired_index);
            if (!xr_check(instance, result, "failed to acquire swapchain image!"))
                break;

            XrSwapchainImageWaitInfo wait_info = {
                XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO,  NULL, 1000 };
            result = xrWaitSwapchainImage(swapchains[i], &wait_info);
            if (!xr_check(instance, result, "failed to wait for swapchain image!"))
                break;

            uint32_t depth_acquired_index = UINT32_MAX;
            if (depth.supported) {
                XrSwapchainImageAcquireInfo depth_acquire_info = {
                    XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO, NULL };
                result = xrAcquireSwapchainImage(depth_swapchains[i], &depth_acquire_info,
                    &depth_acquired_index);
                if (!xr_check(instance, result, "failed to acquire swapchain image!"))
                    break;

                XrSwapchainImageWaitInfo depth_wait_info = {
                    XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO, NULL, 1000 };
                result = xrWaitSwapchainImage(depth_swapchains[i], &depth_wait_info);
                if (!xr_check(instance, result, "failed to wait for swapchain image!"))
                    break;
            }

            projection_views[i].pose = views[i].pose;
            projection_views[i].fov = views[i].fov;

            GLuint depth_image = depth.supported ? depth_images[i][depth_acquired_index].image : 0;

            int w = viewconfig_views[i].recommendedImageRectWidth;
            int h = viewconfig_views[i].recommendedImageRectHeight;

            // TODO: should not be necessary, but is for SteamVR 1.16.4 (but not 1.15.x)
            glfwMakeContextCurrent(window);
            /*
            render_frame(window, w, h, gl_rendering.shader_program_id, gl_rendering.VAO,
                frame_state.predictedDisplayTime, i, projection_matrix,
                view_matrix, gl_rendering.framebuffers[i][acquired_index],
                images[i][acquired_index].image, depth.supported, depth_image);
            */
            optixrenderer.render(window, make_float3(10,10,10), make_float3(-1,-1,-1), make_float3(0,1,0), w, h, i, 
                gl_rendering.framebuffers[i][acquired_index], images[i][acquired_index].image, depth.supported, depth_image);

            XrSwapchainImageReleaseInfo release_info = { XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO,
                                                        NULL };
            result = xrReleaseSwapchainImage(swapchains[i], &release_info);
            if (!xr_check(instance, result, "failed to release swapchain image!"))
                break;

            if (depth.supported) {
                XrSwapchainImageReleaseInfo depth_release_info = {
                    XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO, NULL };
                result = xrReleaseSwapchainImage(depth_swapchains[i], &depth_release_info);
                if (!xr_check(instance, result, "failed to release swapchain image!"))
                    break;
            }
        }

        XrCompositionLayerProjection projection_layer = {
            XR_TYPE_COMPOSITION_LAYER_PROJECTION,
            NULL,
            0,
            play_space,
            view_count,
            projection_views,
        };

        int submitted_layer_count = 1;
        const XrCompositionLayerBaseHeader* submitted_layers[1] = {
            (const XrCompositionLayerBaseHeader* const)&projection_layer };

        if ((view_state.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT) == 0) {
            printf("submitting 0 layers because orientation is invalid\n");
            submitted_layer_count = 0;
        }

        if (!frame_state.shouldRender) {
            printf("submitting 0 layers because shouldRender = false\n");
            submitted_layer_count = 0;
        }

        XrFrameEndInfo frameEndInfo = { XR_TYPE_FRAME_END_INFO, NULL,
                                       frame_state.predictedDisplayTime,
                                       XR_ENVIRONMENT_BLEND_MODE_OPAQUE,
                                       submitted_layer_count,
                                       submitted_layers,                                       
                                       };
        result = xrEndFrame(session, &frameEndInfo);
        if (!xr_check(instance, result, "failed to end frame!"))
            break;

        glfwPollEvents();
    }
}

static const char* vertexshader =
"#version 330 core\n"
"#extension GL_ARB_explicit_uniform_location : require\n"
"layout(location = 0) in vec3 aPos;\n"
"layout(location = 2) uniform mat4 model;\n"
"layout(location = 3) uniform mat4 view;\n"
"layout(location = 4) uniform mat4 proj;\n"
"layout(location = 5) in vec2 aColor;\n"
"out vec2 vertexColor;\n"
"void main() {\n"
"	gl_Position = proj * view * model * vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
"	vertexColor = aColor;\n"
"}\n";

static const char* fragmentshader =
"#version 330 core\n"
"#extension GL_ARB_explicit_uniform_location : require\n"
"layout(location = 0) out vec4 FragColor;\n"
"layout(location = 1) uniform vec3 uniformColor;\n"
"in vec2 vertexColor;\n"
"void main() {\n"
"	FragColor = (uniformColor.x < 0.01 && uniformColor.y < 0.01 && "
"uniformColor.z < 0.01) ? vec4(vertexColor, 1.0, 1.0) : vec4(uniformColor, "
"1.0);\n"
"}\n";


int
init_gl(uint32_t view_count,
    uint32_t* swapchain_lengths,
    GLuint*** framebuffers,
    GLuint* shader_program_id,
    GLuint* VAO)
{

    /* Allocate resources that we use for our own rendering.
     * We will bind framebuffers to the runtime provided textures for rendering.
     * For this, we create one framebuffer per OpenGL texture.
     * This is not mandated by OpenXR, other ways to render to textures will work too.
     */
    *framebuffers = (GLuint**)malloc(sizeof(GLuint*) * view_count);
    for (uint32_t i = 0; i < view_count; i++) {
        (*framebuffers)[i] = (GLuint*)malloc(sizeof(GLuint) * swapchain_lengths[i]);
        glGenFramebuffers(swapchain_lengths[i], (*framebuffers)[i]);
    }

    GLuint vertex_shader_id = glCreateShader(GL_VERTEX_SHADER);
    const GLchar* vertex_shader_source[1];
    vertex_shader_source[0] = vertexshader;
    // printf("Vertex Shader:\n%s\n", vertexShaderSource);
    glShaderSource(vertex_shader_id, 1, vertex_shader_source, NULL);
    glCompileShader(vertex_shader_id);
    int vertex_compile_res;
    glGetShaderiv(vertex_shader_id, GL_COMPILE_STATUS, &vertex_compile_res);
    if (!vertex_compile_res) {
        char info_log[512];
        glGetShaderInfoLog(vertex_shader_id, 512, NULL, info_log);
        printf("Vertex Shader failed to compile: %s\n", info_log);
        return 1;
    }
    else {
        printf("Successfully compiled vertex shader!\n");
    }

    GLuint fragment_shader_id = glCreateShader(GL_FRAGMENT_SHADER);
    const GLchar* fragment_shader_source[1];
    fragment_shader_source[0] = fragmentshader;
    glShaderSource(fragment_shader_id, 1, fragment_shader_source, NULL);
    glCompileShader(fragment_shader_id);
    int fragment_compile_res;
    glGetShaderiv(fragment_shader_id, GL_COMPILE_STATUS, &fragment_compile_res);
    if (!fragment_compile_res) {
        char info_log[512];
        glGetShaderInfoLog(fragment_shader_id, 512, NULL, info_log);
        printf("Fragment Shader failed to compile: %s\n", info_log);
        return 1;
    }
    else {
        printf("Successfully compiled fragment shader!\n");
    }

    *shader_program_id = glCreateProgram();
    glAttachShader(*shader_program_id, vertex_shader_id);
    glAttachShader(*shader_program_id, fragment_shader_id);
    glLinkProgram(*shader_program_id);
    GLint shader_program_res;
    glGetProgramiv(*shader_program_id, GL_LINK_STATUS, &shader_program_res);
    if (!shader_program_res) {
        char info_log[512];
        glGetProgramInfoLog(*shader_program_id, 512, NULL, info_log);
        printf("Shader Program failed to link: %s\n", info_log);
        return 1;
    }
    else {
        printf("Successfully linked shader program!\n");
    }

    glDeleteShader(vertex_shader_id);
    glDeleteShader(fragment_shader_id);

    float vertices[] = { -0.5f, -0.5f, -0.5f, 0.0f, 0.0f, 0.5f,  -0.5f, -0.5f, 1.0f, 0.0f,
                        0.5f,  0.5f,  -0.5f, 1.0f, 1.0f, 0.5f,  0.5f,  -0.5f, 1.0f, 1.0f,
                        -0.5f, 0.5f,  -0.5f, 0.0f, 1.0f, -0.5f, -0.5f, -0.5f, 0.0f, 0.0f,

                        -0.5f, -0.5f, 0.5f,  0.0f, 0.0f, 0.5f,  -0.5f, 0.5f,  1.0f, 0.0f,
                        0.5f,  0.5f,  0.5f,  1.0f, 1.0f, 0.5f,  0.5f,  0.5f,  1.0f, 1.0f,
                        -0.5f, 0.5f,  0.5f,  0.0f, 1.0f, -0.5f, -0.5f, 0.5f,  0.0f, 0.0f,

                        -0.5f, 0.5f,  0.5f,  1.0f, 0.0f, -0.5f, 0.5f,  -0.5f, 1.0f, 1.0f,
                        -0.5f, -0.5f, -0.5f, 0.0f, 1.0f, -0.5f, -0.5f, -0.5f, 0.0f, 1.0f,
                        -0.5f, -0.5f, 0.5f,  0.0f, 0.0f, -0.5f, 0.5f,  0.5f,  1.0f, 0.0f,

                        0.5f,  0.5f,  0.5f,  1.0f, 0.0f, 0.5f,  0.5f,  -0.5f, 1.0f, 1.0f,
                        0.5f,  -0.5f, -0.5f, 0.0f, 1.0f, 0.5f,  -0.5f, -0.5f, 0.0f, 1.0f,
                        0.5f,  -0.5f, 0.5f,  0.0f, 0.0f, 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,

                        -0.5f, -0.5f, -0.5f, 0.0f, 1.0f, 0.5f,  -0.5f, -0.5f, 1.0f, 1.0f,
                        0.5f,  -0.5f, 0.5f,  1.0f, 0.0f, 0.5f,  -0.5f, 0.5f,  1.0f, 0.0f,
                        -0.5f, -0.5f, 0.5f,  0.0f, 0.0f, -0.5f, -0.5f, -0.5f, 0.0f, 1.0f,

                        -0.5f, 0.5f,  -0.5f, 0.0f, 1.0f, 0.5f,  0.5f,  -0.5f, 1.0f, 1.0f,
                        0.5f,  0.5f,  0.5f,  1.0f, 0.0f, 0.5f,  0.5f,  0.5f,  1.0f, 0.0f,
                        -0.5f, 0.5f,  0.5f,  0.0f, 0.0f, -0.5f, 0.5f,  -0.5f, 0.0f, 1.0f };

    GLuint VBOs[1];
    glGenBuffers(1, VBOs);

    glGenVertexArrays(1, VAO);

    glBindVertexArray(*VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBOs[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(5, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(5);

    glEnable(GL_DEPTH_TEST);

    return 0;
}


void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error: %s\n", description);
}

bool init_glfw_window(GLFWwindow** window, HDC* hdc, HGLRC* hglrc, int w, int h) {
    if (!glfwInit())
    {
        return false;
    }

    glfwSetErrorCallback(error_callback);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    *window = glfwCreateWindow(w / 2, h / 2, "My Title", NULL, NULL);
    if (!*window)
    {
        glfwTerminate();
        return false;
    }             

    glfwMakeContextCurrent(*window);
    if (!gladLoadGL())
    {
        return false;
    }
    glfwSwapInterval(1);

    *hglrc = glfwGetWGLContext(*window);    
    *hdc = GetDC(glfwGetWin32Window(*window));

    return true;
}

void
render_rotated_cube(
    glm::vec3 position, float cube_size, float rotation, float* projection_matrix, int modelLoc)
{
    glm::mat4 rotationmatrix = glm::rotate(glm::mat4(1.0), glm::radians(rotation), glm::vec3(0,1,0));
    glm::mat4 modelmatrix = glm::mat4(1.0);
    modelmatrix = glm::translate(modelmatrix, position);
    modelmatrix = glm::scale(modelmatrix, glm::vec3(cube_size / 2));        
    modelmatrix = modelmatrix * rotationmatrix;

    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(modelmatrix));
    glDrawArrays(GL_TRIANGLES, 0, 36);
}


void
render_frame(GLFWwindow* window,
    int w,
    int h,
    GLuint shader_program_id,
    GLuint VAO,
    XrTime predictedDisplayTime,
    int view_index,
    glm::mat4 projectionmatrix,
    glm::mat4 viewmatrix,
    GLuint framebuffer,
    GLuint image,
    bool depth_supported,
    GLuint depthbuffer)
{
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

    glViewport(0, 0, w, h);
    glScissor(0, 0, w, h);

    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, image, 0);
    if (depth_supported) {
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthbuffer, 0);
    }
    else {
        // TODO: need a depth attachment for depth test when rendering to fbo
    }

    glClearColor(.0f, 0.0f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


    glUseProgram(shader_program_id);
    glBindVertexArray(VAO);

    int modelLoc = glGetUniformLocation(shader_program_id, "model");
    int colorLoc = glGetUniformLocation(shader_program_id, "uniformColor");
    int viewLoc = glGetUniformLocation(shader_program_id, "view");
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(viewmatrix));
    int projLoc = glGetUniformLocation(shader_program_id, "proj");
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projectionmatrix));


    // render scene with 4 colorful cubes
    {
        // the special color value (0, 0, 0) will get replaced by some UV color in the shader
        glUniform3f(colorLoc, 0.0, 0.0, 0.0);

        double display_time_seconds = ((double)predictedDisplayTime) / (1000. * 1000. * 1000.);
        const float rotations_per_sec = .25;
        float angle = ((long)(display_time_seconds * 360. * rotations_per_sec)) % 360;

        float dist = 1.5f;
        float height = 0.5f;
        render_rotated_cube(glm::vec3(0, height, -dist), .33f, angle, glm::value_ptr(projectionmatrix), modelLoc);
        render_rotated_cube(glm::vec3(0, height, dist), .33f, angle, glm::value_ptr(projectionmatrix), modelLoc);
        render_rotated_cube(glm::vec3(dist, height, 0), .33f, angle, glm::value_ptr(projectionmatrix), modelLoc);
        render_rotated_cube(glm::vec3(-dist, height, 0), .33f, angle, glm::value_ptr(projectionmatrix), modelLoc);

        render_rotated_cube(glm::vec3(1, 1, 2), .33f, angle, glm::value_ptr(projectionmatrix), modelLoc);
    }

    // blit left eye to desktop window
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
