typedef unsigned int uint32_t;


extern "C" __declspec(dllexport) void computeFinalPixelColors(const int2& fbSize, float4* denoisedBuffer, uint32_t* finalColorBuffer);
