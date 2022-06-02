#pragma once

#include "maths.h"
#include "sample.h"

struct Probe
{
	int width;
	int height;

	Color* data;

	// world space offset to effectively warp the skybox
	float3 offset;
	
	float* pdfValuesX;
	float* cdfValuesX;

	float* pdfValuesY;
	float* cdfValuesY;
};

CUDA_CALLABLE __forceinline__ float4 SampleProbeSphere(const Probe& image, const float3& dir)
{
	// convert world space dir to probe space
	float c = kInvPi * acosf(dir.z) / sqrt(dir.x * dir.x + dir.y * dir.y);

	int px = (0.5f + 0.5f * (dir.x * c)) * image.width;
	int py = (0.5f + 0.5f * (-dir.y * c)) * image.height;

	px = min(max(0, px), image.width - 1);
	py = min(max(0, py), image.height - 1);

	return image.data[py * image.width + px];

}

CUDA_CALLABLE __forceinline__ float2 ProbeDirToUV(const float3& dir)
{
	float theta = acosf(clamp(dir.y, -1.0f, 1.0f));
	float phi = (dir.x == 0.0f && dir.z == 0.0f) ? 0.0f : atan2(dir.z, dir.x);
	float u = (kPi + phi) * kInvPi * 0.5f;
	float v = theta * kInvPi;

	return make_float2(u, v);
}

CUDA_CALLABLE __forceinline__ float3 ProbeUVToDir(const float2& uv)
{
	float theta = uv.y * kPi;
	float phi = uv.x * 2.0f * kPi;

	float x = -sinf(theta) * cosf(phi);
	float y = cosf(theta);
	float z = -sinf(theta) * sinf(phi);

	return make_float3(x, y, z);
}


CUDA_CALLABLE __forceinline__ float4 ProbeEval(const Probe& image, const float2& uv)
{
	int px = clamp(int(uv.x * image.width), 0, image.width - 1);
	int py = clamp(int(uv.y * image.height), 0, image.height - 1);

	return fetchVec4(image.data, py * image.width + px);
}

CUDA_CALLABLE __forceinline__ float ProbePdf(const Probe& image, const float3& d)
{

	float2 uv = ProbeDirToUV(d);

	int col = clamp(int(uv.x * image.width), 0, image.width - 1);
	int row = clamp(int(uv.y * image.height), 0, image.height - 1);

	float pdf = fetchFloat(image.pdfValuesX, row * image.width + col) * fetchFloat(image.pdfValuesY, row);

	/*Validate(pdf);
	Validate(uv.y);
	Validate(uv.x);*/

	float sinTheta = sinf(uv.y * kPi);
	//Validate(sinTheta);
	if (fabsf(sinTheta) < 0.0001f)
		pdf = 0.0f;
	else
		pdf *= float(image.width) * float(image.height) / (2.0f * kPi * kPi * sinTheta);

	//Validate(pdf);

	return pdf;
}

template <typename T>
CUDA_CALLABLE __forceinline__ const T* LowerBound(const T* begin, const T* end, const T& value)
{
	const T* lower = begin;
	const T* upper = end;

	while (lower < upper)
	{
		const T* mid = lower + (upper - lower) / 2;

		if (*mid < value)
		{
			lower = mid + 1;
		}
		else
		{
			upper = mid;
		}
	}

	return lower;
}


CUDA_CALLABLE __forceinline__ int LowerBound(const float* array, int lower, int upper, const float value)
{
	while (lower < upper)
	{
		int mid = lower + (upper - lower) / 2;

		if (fetchFloat(array, mid) < value)
		{
			lower = mid + 1;
		}
		else
		{
			upper = mid;
		}
	}

	return lower;
}

CUDA_CALLABLE __forceinline__ void ProbeSample(const Probe& image, float3& dir, float3& color, float& pdf, Random& rand)
{
	float r1, r2;
	Sample2D(rand, r1, r2);

	// sample rows
	//float* rowPtr = std::lower_bound(image.cdfValuesY, image.cdfValuesY+image.height, r1);
	//const float* rowPtr = LowerBound(image.cdfValuesY, image.cdfValuesY+image.height, r1);
	//int row = rowPtr - image.cdfValuesY;
	int row = LowerBound(image.cdfValuesY, 0, image.height, r1);

	// sample cols of row
	//float* colPtr = std::lower_bound(&image.cdfValuesX[row*image.width], &image.cdfValuesX[(row+1)*image.width], r2);
	//const float* colPtr = LowerBound(&image.cdfValuesX[row*image.width], &image.cdfValuesX[(row+1)*image.width], r2);
	//int col = colPtr - &image.cdfValuesX[row*image.width];
	int col = LowerBound(image.cdfValuesX, row * image.width, (row + 1) * image.width, r2) - row * image.width;

	color = make_float3(fetchVec4(image.data, row * image.width + col));
	pdf = fetchFloat(image.pdfValuesX, row * image.width + col) * fetchFloat(image.pdfValuesY, row);

	float u = col / float(image.width);
	float v = row / float(image.height);

	float sinTheta = sinf(v * kPi);
	if (sinTheta == 0.0f)
		pdf = 0.0f;
	else
		pdf *= image.width * image.height / (2.0f * kPi * kPi * sinTheta);

	dir = ProbeUVToDir(make_float2(u, v));

}

/*inline Probe ProbeLoadFromFile(const char* path)
{
	double start = GetSeconds();

	PfmImage image;
	//PfmLoad(path, image);
	if (HdrLoad(path, image))
	{
		Probe probe;
		probe.width = image.width;
		probe.height = image.height;

		int numPixels = image.width*image.height;

		// convert image data to color data, apply pre-exposure etc
		probe.data = new Color[numPixels];

		for (int i=0; i < numPixels; ++i)
			probe.data[i] = Color(image.data[i*3+0], image.data[i*3+1], image.data[i*3+2]);

		probe.BuildCDF();

		delete[] image.data;

		double end = GetSeconds();

		printf("Imported probe %s in %fms\n", path, (end-start)*1000.0f);

		return probe;
	}
	else
	{
		return Probe();
	}
}

inline Probe ProbeCreateTest()
{
	Probe p;
	p.width = 100;
	p.height = 50;
	p.data = new Color[p.width*p.height];

	float3 axis = Normalize(float3(.0f, 1.0f, 0.0f));

	for (int i=0; i < p.width; i++)
	{
		for (int j=0; j < p.height; j++)
		{
			// add a circular disc based on a view dir
			float u = i/float(p.width);
			float v = j/float(p.height);

			float3 dir = ProbeUVToDir(Vec2(u, v));

			if (Dot(dir, axis) >= 0.95f)
			{
				p.data[j*p.width+i] = Color(10.0f);

			}
			else
			{
				p.data[j*p.width+i] = 0.0f;
			}
		}
	}

	p.BuildCDF();


	return p;
}


inline void ProbeMark(Probe& probe)
{
	Random rand;

	// sample probe a number of times
	for (int i=0; i < 500; ++i)
	{
		float3 dir;
		float3 color;
		float pdf;

		ProbeSample(probe, dir, color, pdf, rand);

		Vec2 uv = ProbeDirToUV(dir);

		int px = Clamp(int(uv.x*probe.width), 0, probe.width-1);
		int py = Clamp(int(uv.y*probe.height), 0, probe.height-1);

		//printf("%d %d\n", px, py);

		probe.data[py*probe.width+px] = Color(1.0f, 0.0f, 0.0f);


	}
}*/