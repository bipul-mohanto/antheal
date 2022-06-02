#pragma once

#include "Maths.h"

#ifndef MATERIAL_HEADER
#define MATERIAL_HEADER

static const int MATERIAL_FLAG_NONE = 0;
static const int MATERIAL_FLAG_SHADOW_CATCHER = 1 << 0;

struct Material
{
	Material()
	{
		color = make_float3(0.6f, 0.6f, 0.6f);
		emission = make_float3(0.0f);
		absorption = make_float3(0.0);

		// when eta is zero the index of refraction will be inferred from the specular component
		eta = 0.0f;

		metallic = 0.0f;
		subsurface = 0.0f;
		specular = 0.5f;
		roughness = 1.0f;
		specularTint = 0.0f;
		anisotropic = 0.0f;
		sheen = 0.0f;
		sheenTint = 0.0f;
		clearcoat = 0.0f;
		clearcoatGloss = 1.0f;
		transmission = 0.0f;
		bump = 0.0f;
		bumpTile = make_float3(10.0f);		

		flags = 0;
	}

	CUDA_CALLABLE __forceinline__ float GetIndexOfRefraction() const
	{
		if (eta == 0.0f)
			return 2.0f / (1.0f - sqrt(0.08f * specular)) - 1.0f;
		else
			return eta;
	}

	float3 emission;
	float3 color;
	float3 absorption;

	float eta;
	float metallic;
	float subsurface;
	float specular;
	float roughness;
	float specularTint;
	float anisotropic;
	float sheen;
	float sheenTint;
	float clearcoat;
	float clearcoatGloss;
	float transmission;

	//Texture bumpMap;
	float bump;
	float3 bumpTile;

	int flags;
};

#endif // !MATERIAL_HEADER