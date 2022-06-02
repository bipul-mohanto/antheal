#pragma once

#include <sutil/vec_math.h>

#if defined(__CUDACC__) || defined(__CUDABE__)
#    define SUTIL_HOSTDEVICE __host__ __device__
#    define SUTIL_INLINE __forceinline__
#    define CONST_STATIC_INIT( ... )
#else
#    define SUTIL_HOSTDEVICE
#    define SUTIL_INLINE inline
#    define CONST_STATIC_INIT( ... ) = __VA_ARGS__
#endif

#ifdef __CUDACC__
#define CUDA_CALLABLE __host__ __device__
#define CUDA_INLINE __forceinline__
#else
#define CUDA_CALLABLE
#define CUDA_INLINE inline
#endif

#if __CUDA_ARCH__
#define RESTRICT __restrict__
#else
#define RESTRICT 
#endif

#define kPi (3.141592653589793f)
#define k2Pi (3.141592653589793f*2.0f)
#define kInvPi (1.0f/kPi)
#define kInv2Pi (1.0f/k2Pi)

typedef float4 Color;

CUDA_CALLABLE CUDA_INLINE float fetchFloat(const float* RESTRICT ptr, int index)
{
#if __CUDA_ARCH__ && USE_TEXTURES
	return tex1Dfetch<float>((cudaTextureObject_t)ptr, index);
#else
	return ptr[index];
#endif

}

CUDA_CALLABLE CUDA_INLINE float3 fetchVec3(const float3* RESTRICT ptr, int index)
{
#if __CUDA_ARCH__ && USE_TEXTURES

	//float x = tex1Dfetch<float>((cudaTextureObject_t)ptr, index*3+0);
	//float y = tex1Dfetch<float>((cudaTextureObject_t)ptr, index*3+1);
	//float z = tex1Dfetch<float>((cudaTextureObject_t)ptr, index*3+2);

	float4 x = tex1Dfetch<float4>((cudaTextureObject_t)ptr, index);
	return Vec3(x.x, x.y, x.z);
#else
	return ptr[index];
#endif

}

CUDA_CALLABLE CUDA_INLINE float4 fetchVec4(const float4* RESTRICT ptr, int index)
{
#if __CUDA_ARCH__ && USE_TEXTURES
	float4 x = tex1Dfetch<float4>((cudaTextureObject_t)ptr, index);
	return Vec4(x.x, x.y, x.z, x.w);
#else
	return ptr[index];
#endif

}

CUDA_CALLABLE CUDA_INLINE float maxf(float a, float b)
{
	return a > b ? a : b;
}

CUDA_CALLABLE CUDA_INLINE float sqr(float a) 
{
	return a * a;
}


CUDA_CALLABLE CUDA_INLINE float3 exp(float3 a)
{
	return make_float3(exp(a.x), exp(a.y), exp(a.z));
}

CUDA_CALLABLE CUDA_INLINE float area(float3 a, float3 b, float3 c)
{
	return abs(a.x*(b.y - c.y) + b.x*(c.y - a.y) + c.x*(a.y - b.y)) / 2.0f;
}

CUDA_CALLABLE inline void BasisFromVector(const float3& w, float3* u, float3* v)
{
	if (fabs(w.x) > fabs(w.y))
	{
		float invLen = 1.0 / sqrt(w.x * w.x + w.z * w.z);
		*u = make_float3(-w.z * invLen, 0.0f, w.x * invLen);
	}
	else
	{
		float invLen = 1.0 / sqrt(w.y * w.y + w.z * w.z);
		*u = make_float3(0.0f, w.z * invLen, -w.y * invLen);
	}

	*v = cross(w, *u);
}

#ifdef __CUDACC__
CUDA_CALLABLE CUDA_INLINE float lerp(float a, float b, float t)
{
	return a + t * (b - a);
}
#endif


#ifndef __CUDACC__

#include <stdio.h>

CUDA_CALLABLE CUDA_INLINE void ValidateImpl(float x, const char* file, int line)
{
	if (!isfinite(x))
		printf("Fail: %s, %d (%f)\n", file, line, x);
}

CUDA_CALLABLE CUDA_INLINE void ValidateImpl(const float3& x, const char* file, int line)
{
	if (!isfinite(x.x) || !isfinite(x.y) || !isfinite(x.z))
		printf("Fail: %s, %d (%f, %f, %f)\n", file, line, x.x, x.y, x.z);
}

CUDA_CALLABLE CUDA_INLINE void ValidateImpl(const Color& c, const char* file, int line)
{
	if (!isfinite(c.x) || !isfinite(c.y) || !isfinite(c.z))
		printf("Fail: %s, %d\n", file, line);
}
#endif


CUDA_CALLABLE CUDA_INLINE float LengthSq(const float3& a) { return dot(a, a); }

CUDA_CALLABLE CUDA_INLINE float3 SafeNormalize(const float3& a, const float3& fallback = make_float3(0.0f))
{
	float m = LengthSq(a);

	if (m > 0.0)
	{
		return a * (1.0 / sqrt(m));
	}
	else
	{
		return fallback;
	}
}


#ifndef NDEBUG
#define Validate(x) ValidateImpl(x, __FILE__, __LINE__)
#else
#define Validate(x)
#endif

CUDA_CALLABLE CUDA_INLINE float Luminance(const Color& c)
{
	return c.x * 0.3f + c.y * 0.6f + c.z * 0.1f;
}

class Random
{
public:

	CUDA_CALLABLE inline Random(int seed = 0)
	{
		seed1 = 315645664 + seed;
		seed2 = seed1 ^ 0x13ab45fe;
	}

	CUDA_CALLABLE inline unsigned int Rand()
	{
		seed1 = (seed2 ^ ((seed1 << 5) | (seed1 >> 27))) ^ (seed1 * seed2);
		seed2 = seed1 ^ ((seed2 << 12) | (seed2 >> 20));

		return seed1;
	}

	// returns a random number in the range [min, max)
	CUDA_CALLABLE inline unsigned int Rand(unsigned int min, unsigned int max)
	{
		return min + Rand() % (max - min);
	}

	//std::default_random_engine generator;

	// returns random number between 0-1
	CUDA_CALLABLE inline float Randf()
	{
		//std::uniform_real_distribution<float> distr(0.0f,1.0f);

		//return distr(generator);

		unsigned int value = Rand();
		unsigned int limit = 0xffffffff;

		return (float)value * (1.0f / (float)limit);

	}

	// returns random number between min and max
	CUDA_CALLABLE inline float Randf(float min, float max)
	{
		float t = Randf();
		return (1.0f - t) * min + t * max;
	}

	// returns random number between 0-max
	CUDA_CALLABLE inline float Randf(float max)
	{
		return Randf() * max;
	}

	unsigned int seed1;
	unsigned int seed2;
};


CUDA_CALLABLE CUDA_INLINE float3 UniformSampleSphere(float u1, float u2)
{
	float z = 1.f - 2.f * u1;
	float r = sqrtf(maxf(0.f, 1.f - z * z));
	float phi = 2.f * kPi * u2;
	float x = r * cosf(phi);
	float y = r * sinf(phi);

	return make_float3(x, y, z);
}



CUDA_CALLABLE CUDA_INLINE float3 UniformSampleHemisphere(Random& rand)
{
	// generate a random z value
	float z = rand.Randf(0.0f, 1.0f);
	float w = sqrt(1.0f - z * z);

	float phi = k2Pi * rand.Randf(0.0f, 1.0f);
	float x = cosf(phi) * w;
	float y = sinf(phi) * w;

	return make_float3(x, y, z);
}

CUDA_CALLABLE CUDA_INLINE float2 UniformSampleDisc(float u1, float u2)
{
	float r = sqrt(u1);
	float theta = k2Pi * u2;

	return make_float2(r * cos(theta), r * sin(theta));
}

CUDA_CALLABLE CUDA_INLINE void UniformSampleTriangle(Random& rand, float& u, float& v)
{
	float r = sqrt(rand.Randf());
	u = 1.0f - r;
	v = rand.Randf() * r;
}

CUDA_CALLABLE CUDA_INLINE float3 CosineSampleHemisphere(float u1, float u2)
{
	float2 s = UniformSampleDisc(u1, u2);
	float z = sqrt(maxf(0.0f, 1.0f - s.x * s.x - s.y * s.y));

	return make_float3(s.x, s.y, z);
}