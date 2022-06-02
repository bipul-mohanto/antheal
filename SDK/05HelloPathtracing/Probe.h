#pragma once

#include "math.h"

#include "CUDABuffer.h"

struct ProbeData
{
	int width;
	int height;

	Color* data;

	// world space offset to effectively warp the skybox
	float3 offset;

	ProbeData() : valid(false) {}

	// cdf

		struct Entry
		{
			float weight;
			int index;

			inline bool operator < (const Entry& e) const { return weight < e.weight; }
		};

	inline void BuildCDF()
	{
		pdfValuesX = new float[width*height];
		pdfValuesY = new float[height];

		cdfValuesX = new float[width*height];
		cdfValuesY = new float[height];

		float totalWeightY = 0.0f;

		for (int j=0; j < height; ++j)
		{
			float totalWeightX = 0.0f;

			for (int i=0; i < width; ++i)
			{
				float weight = Luminance(data[j*width + i]);

				totalWeightX += weight;
				
				pdfValuesX[j*width + i] = weight;
				cdfValuesX[j*width + i] = totalWeightX;
			}			

			float invTotalWeightX = 1.0f/totalWeightX;

			// convert to pdf and cdf
			for (int i=0; i < width; ++i)
			{
				pdfValuesX[j*width + i] *= invTotalWeightX;
				cdfValuesX[j*width + i] *= invTotalWeightX;
			}

			// total weight y 
			totalWeightY += totalWeightX;

			pdfValuesY[j] = totalWeightX;
			cdfValuesY[j] = totalWeightY;
		}

		// convert Y to cdf
		for (int j=0; j < height; ++j)
		{
			cdfValuesY[j] /= float(totalWeightY);
			pdfValuesY[j] /= float(totalWeightY);
		}

		valid = true;
	}

	bool valid;
	
	float* pdfValuesX;
	float* cdfValuesX;

	float* pdfValuesY;
	float* cdfValuesY;
};

struct CUDAProbeData {
	int width;
	int height;

	float3 offset;

	CUDABuffer pdfValuesX;
	CUDABuffer pdfValuesY;

	CUDABuffer cdfValuesX;
	CUDABuffer cdfValuesY;

	CUDABuffer data;

	void createBuffer(const ProbeData& probeData) {

		if (!probeData.valid)
			throw std::runtime_error("Probe Data is not valid");

		pdfValuesX.alloc(probeData.width * probeData.height * sizeof(float));
		pdfValuesX.upload(probeData.pdfValuesX, probeData.width * probeData.height);
		pdfValuesY.alloc(probeData.height * sizeof(float));
		pdfValuesY.upload(probeData.pdfValuesY, probeData.height);

		cdfValuesX.alloc(probeData.width * probeData.height * sizeof(float));
		cdfValuesX.upload(probeData.cdfValuesX, probeData.width * probeData.height);
		cdfValuesY.alloc(probeData.height * sizeof(float));
		cdfValuesY.upload(probeData.cdfValuesY, probeData.height);

		data.alloc((size_t)probeData.width * (size_t)probeData.height * sizeof(float4));
		data.upload(probeData.data, (size_t)probeData.width * (size_t)probeData.height);

		width = probeData.width;
		height = probeData.height;

		offset = probeData.offset;
	}
};