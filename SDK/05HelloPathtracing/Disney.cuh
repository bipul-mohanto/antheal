/*
# Copyright Disney Enterprises, Inc.  All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License
# and the following modification to it: Section 6 Trademarks.
# deleted and replaced with:
#
# 6. Trademarks. This License does not grant permission to use the
# trade names, trademarks, service marks, or product names of the
# Licensor and its affiliates, except as required for reproducing
# the content of the NOTICE file.
#
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0

# Adapted to C++ by Miles Macklin 2016

*/
#define USE_UNIFORM_SAMPLING 0
#define USE_SIMPLE_BSDF 0

#include "maths.h"
#include "sample.h"

#include "LaunchParams.h"

enum BSDFType
{
    eReflected,
    eTransmitted,
    eSpecular
};

CUDA_CALLABLE inline bool Refract(const float3& wi, const float3& n, float eta, float3& wt)
{
    float cosThetaI = dot(n, wi);
    float sin2ThetaI = max(0.0f, float(1.0f - cosThetaI * cosThetaI));
    float sin2ThetaT = eta * eta * sin2ThetaI;

    // total internal reflection
    if (sin2ThetaT >= 1)
        return false;

    float cosThetaT = sqrtf(1.0f - sin2ThetaT);
    wt = eta * -wi + (eta * cosThetaI - cosThetaT) * float3(n);
    return true;
}

CUDA_CALLABLE inline float SchlickFresnel(float u)
{
    float m = clamp(1 - u, 0.0f, 1.0f);
    float m2 = m * m;
    return m2 * m2 * m; // pow(m,5)
}

CUDA_CALLABLE inline float GTR1(float NDotH, float a)
{
    if (a >= 1) return kInvPi;
    float a2 = a * a;
    float t = 1 + (a2 - 1) * NDotH * NDotH;
    return (a2 - 1) / (kPi * logf(a2) * t);
}

CUDA_CALLABLE inline float GTR2(float NDotH, float a)
{
    float a2 = a * a;
    float t = 1.0f + (a2 - 1.0f) * NDotH * NDotH;
    return a2 / (kPi * t * t);
}

CUDA_CALLABLE inline float SmithGGX(float NDotv, float alphaG)
{
    float a = alphaG * alphaG;
    float b = NDotv * NDotv;
    return 1 / (NDotv + sqrtf(a + b - a * b));
}


CUDA_CALLABLE inline float Fr(float VDotN, float etaI, float etaT)
{
    float SinThetaT2 = sqr(etaI / etaT) * (1.0f - VDotN * VDotN);

    // total internal reflection
    if (SinThetaT2 > 1.0f)
        return 1.0f;

    float LDotN = sqrt(1.0f - SinThetaT2);

    // todo: reformulate to remove this division
    float eta = etaT / etaI;

    float r1 = (VDotN - eta * LDotN) / (VDotN + eta * LDotN);
    float r2 = (LDotN - eta * VDotN) / (LDotN + eta * VDotN);

    return 0.5f * (sqr(r1) + sqr(r2));
}

/*float Fr(float cosThetaI, float etaI, float etaT) {
    cosThetaI = clamp(cosThetaI, -1.f, 1.f);
    // Potentially swap indices of refraction
    bool entering = cosThetaI > 0.f;
    if (!entering) {
        float temp = etaI;
        etaI = etaT;
        etaT = temp;
        cosThetaI = abs(cosThetaI);
    }

    // Compute _cosThetaT_ using Snell's law
    float sinThetaI = sqrt(max((float)0, 1 - cosThetaI * cosThetaI));
    float sinThetaT = etaI / etaT * sinThetaI;

    // Handle total internal reflection
    if (sinThetaT >= 1) return 1;
    float cosThetaT = sqrt(max((float)0, 1 - sinThetaT * sinThetaT));
    float Rparl = ((etaT * cosThetaI) - (etaI * cosThetaT)) /
        ((etaT * cosThetaI) + (etaI * cosThetaT));
    float Rperp = ((etaI * cosThetaI) - (etaT * cosThetaT)) /
        ((etaI * cosThetaI) + (etaT * cosThetaT));
    return (Rparl * Rparl + Rperp * Rperp) / 2;
}*/

// lambert
#if USE_SIMPLE_BSDF

CUDA_CALLABLE inline float BSDFPdf(const Material& mat, float etaI, float etaO, const float3& P, const float3& n, const float3& V, const float3& L)
{
    if (Dot(L, n) <= 0.0f)
        return 0.0f;
    else
        return kInv2Pi;
}

CUDA_CALLABLE inline void BSDFSample(const Material& mat, float etaI, float etaO, const float3& P, const float3& U, const float3& V, const float3& N, const float3& view, float3& light, float& pdf, BSDFType& type, Random& rand)
{
    float3 d = UniformSampleHemisphere(rand);

    light = U * d.x + V * d.y + N * d.z;
    pdf = kInv2Pi;
    type = eReflected;
}

CUDA_CALLABLE inline float3 BSDFEval(const Material& mat, float etaI, float etaO, const float3& P, const float3& N, const float3& V, const float3& L)
{
    return kInvPi * mat.color;
}

#else

CUDA_CALLABLE inline float BSDFPdf(const Material& mat, float etaI, float etaO, const float3& P, const float3& n, const float3& V, const float3& L)
{
#if USE_UNIFORM_SAMPLING

    return kInv2Pi * 0.5f;

#endif

    if (dot(L, n) <= 0.0f)
    {

        float bsdfPdf = 0.0f;
        float brdfPdf = kInv2Pi * mat.subsurface * 0.5f;

        return lerp(brdfPdf, bsdfPdf, mat.transmission);
    }
    else
    {
        float F = Fr(dot(n, V), etaI, etaO);

        const float a = max(0.001f, mat.roughness);

        const float3 half = SafeNormalize(L + V);

        const float cosThetaHalf = abs(dot(half, n));
        const float pdfHalf = GTR2(cosThetaHalf, a) * cosThetaHalf;

        // calculate pdf for each method given outgoing light vector
        float pdfSpec = 0.25f * pdfHalf / max(1.e-6f, dot(L, half));
        //assert(isfinite(pdfSpec));

        float pdfDiff = abs(dot(L, n)) * kInvPi * (1.0f - mat.subsurface);
        //assert(isfinite(pdfDiff));

        float bsdfPdf = pdfSpec * F;
        float brdfPdf = lerp(pdfDiff, pdfSpec, 0.5f);

        // weight pdfs equally
        return lerp(brdfPdf, bsdfPdf, mat.transmission);

    }
}


// generate an importance sampled BSDF direction
CUDA_CALLABLE inline void BSDFSample(const Material& mat, float etaI, float etaO, const float3& P, const float3& U, const float3& V, const float3& N, const float3& view, float3& light, float& pdf, BSDFType& type, Random& rand)
{
    if (rand.Randf() < mat.transmission)
    {
        // sample BSDF
        float F = Fr(dot(N, view), etaI, etaO);

        // sample reflectance or transmission based on Fresnel term
        if (rand.Randf() < F)
        {
            // sample specular
            float r1, r2;
            Sample2D(rand, r1, r2);

            const float a = max(0.001f, mat.roughness);
            const float phiHalf = r1 * k2Pi;

            const float cosThetaHalf = sqrtf((1.0f - r2) / (1.0f + (sqr(a) - 1.0f) * r2));
            const float sinThetaHalf = sqrtf(max(0.0f, 1.0f - sqr(cosThetaHalf)));
            const float sinPhiHalf = sinf(phiHalf);
            const float cosPhiHalf = cosf(phiHalf);

            float3 half = U * (sinThetaHalf * cosPhiHalf) + V * (sinThetaHalf * sinPhiHalf) + N * cosThetaHalf;

            // ensure half angle in same hemisphere as incoming light vector
            if (dot(half, view) <= 0.0f)
                half *= -1.0f;

            type = eReflected;
            light = 2.0f * dot(view, half) * half - view;

        }
        else
        {
            // sample transmission
            float eta = etaI / etaO;

            //float3 h = Normalize(V+light);

            if (Refract(view, N, eta, light))
            {
                type = eSpecular;
                pdf = (1.0f - F) * mat.transmission;
                return;
            }
            else
            {
                //assert(0);
                pdf = 0.0f;
                return;
            }
        }
    }
    else
    {

#if USE_UNIFORM_SAMPLING

        light = UniformSampleSphere(rand.Randf(), rand.Randf());
        pdf = kInv2Pi * 0.5f;

        return;
#else

        // sample brdf
        float r1, r2;
        Sample2D(rand, r1, r2);

        if (rand.Randf() < 0.5f)
        {
            // sample diffuse	
            if (rand.Randf() < mat.subsurface)
            {
                const float3 d = UniformSampleHemisphere(rand);

                // negate z coordinate to sample inside the surface
                light = U * d.x + V * d.y - N * d.z;
                type = eTransmitted;
            }
            else
            {
                const float3 d = CosineSampleHemisphere(r1, r2);

                light = U * d.x + V * d.y + N * d.z;
                type = eReflected;
            }
        }
        else
        {
            // sample specular
            const float a = max(0.001f, mat.roughness);

            const float phiHalf = r1 * k2Pi;

            const float cosThetaHalf = sqrtf((1.0f - r2) / (1.0f + (sqr(a) - 1.0f) * r2));
            const float sinThetaHalf = sqrtf(max(0.0f, 1.0f - sqr(cosThetaHalf)));
            const float sinPhiHalf = sinf(phiHalf);
            const float cosPhiHalf = cosf(phiHalf);

            /*Validate(cosThetaHalf);
            Validate(sinThetaHalf);
            Validate(sinPhiHalf);
            Validate(cosPhiHalf);*/

            float3 half = U * (sinThetaHalf * cosPhiHalf) + V * (sinThetaHalf * sinPhiHalf) + N * cosThetaHalf;

            // ensure half angle in same hemisphere as incoming light vector
            if (dot(half, view) <= 0.0f)
                half *= -1.0f;

            light = 2.0f * dot(view, half) * half - view;
            type = eReflected;
        }
#endif
    }

    pdf = BSDFPdf(mat, etaI, etaO, P, N, view, light);

}


CUDA_CALLABLE inline float3 BSDFEval(const Material& mat, float etaI, float etaO, const float3& P, const float3& N, const float3& V, const float3& L)
{
    float NDotL = dot(N, L);
    float NDotV = dot(N, V);

    float3 H = normalize(L + V);

    float NDotH = dot(N, H);
    float LDotH = dot(L, H);

    float3 Cdlin = float3(mat.color);
    float Cdlum = .3 * Cdlin.x + .6 * Cdlin.y + .1 * Cdlin.z; // luminance approx.

    float3 Ctint = Cdlum > 0.0f ? Cdlin / Cdlum : make_float3(1.0f); // normalize lum. to isolate hue+sat
    float3 Cspec0 = lerp(mat.specular * .08 * lerp(make_float3(1.0f), Ctint, mat.specularTint), Cdlin, mat.metallic);
    // float3 Csheen = Lerp(float3(1), Ctint, mat.sheenTint);

    float3 bsdf = make_float3(0.0f);
    float3 brdf = make_float3(0.0f);

    if (mat.transmission > 0.0f)
    {
        // evaluate BSDF
        if (NDotL <= 0)
        {
            // transmission Fresnel
            float F = Fr(NDotV, etaI, etaO);

            bsdf = make_float3(mat.transmission * (1.0f - F) / abs(NDotL) * (1.0f - mat.metallic));
        }
        else
        {
            // specular lobe
            float a = max(0.001f, mat.roughness);
            float Ds = GTR2(NDotH, a);

            // Fresnel term with the microfacet normal
            float FH = Fr(LDotH, etaI, etaO);

            float3 Fs = lerp(Cspec0, make_float3(1.0f), FH);
            float roughg = a;
            float Gs = SmithGGX(NDotV, roughg) * SmithGGX(NDotL, roughg);

            bsdf = Gs * Fs * Ds;
        }
    }

    if (mat.transmission < 1.0f)
    {
        // evaluate BRDF
        if (NDotL <= 0)
        {
            if (mat.subsurface > 0.0f)
            {
                // take sqrt to account for entry/exit of the ray through the medium
                // this ensures transmitted light corresponds to the diffuse model
                float3 s = make_float3(sqrt(mat.color.x), sqrt(mat.color.y), sqrt(mat.color.z));

                float FL = SchlickFresnel(abs(NDotL)), FV = SchlickFresnel(NDotV);
                float Fd = (1.0f - 0.5f * FL) * (1.0f - 0.5f * FV);

                brdf = kInvPi * s * mat.subsurface * Fd * (1.0f - mat.metallic);
            }
        }
        else
        {
            // specular
            float a = max(0.001f, mat.roughness);
            float Ds = GTR2(NDotH, a);

            // Fresnel term with the microfacet normal
            float FH = SchlickFresnel(LDotH);

            float3 Fs = lerp(Cspec0, make_float3(1.f), FH);
            float roughg = a;
            float Gs = SmithGGX(NDotV, roughg) * SmithGGX(NDotL, roughg);

            // Diffuse fresnel - go from 1 at normal incidence to .5 at grazing
            // and mix in diffuse retro-reflection based on roughness
            float FL = SchlickFresnel(NDotL), FV = SchlickFresnel(NDotV);
            float Fd90 = 0.5 + 2.0f * LDotH * LDotH * mat.roughness;
            float Fd = lerp(1.0f, Fd90, FL) * lerp(1.0f, Fd90, FV);

            // Based on Hanrahan-Krueger BSDF approximation of isotrokPic bssrdf
            // 1.25 scale is used to (roughly) preserve albedo
            // Fss90 used to "flatten" retroreflection based on roughness
            //float Fss90 = LDotH*LDotH*mat.roughness;
            //float Fss = Lerp(1.0f, Fss90, FL) * Lerp(1.0f, Fss90, FV);
            //float ss = 1.25 * (Fss * (1.0f / (NDotL + NDotV) - .5) + .5);

            // clearcoat (ior = 1.5 -> F0 = 0.04)
            float Dr = GTR1(NDotH, lerp(.1f, .001f, mat.clearcoatGloss));
            float Fc = lerp(.04f, 1.0f, FH);
            float Gr = SmithGGX(NDotL, .25f) * SmithGGX(NDotV, .25f);

            /*
            // sheen
            float3 Fsheen = FH * mat.sheen * Csheen;

            float3 out = ((1/kPi) * Lerp(Fd, ss, mat.subsurface)*Cdlin + Fsheen)
                * (1-mat.metallic)*(1.0f-mat.transmission)
                + Gs*Fs*Ds + .25*mat.clearcoat*Gr*Fr*Dr;
            */

            brdf = kInvPi * Fd * Cdlin * (1.0f - mat.metallic) * (1.0f - mat.subsurface) + Gs * Fs * Ds + mat.clearcoat * Gr * Fc * Dr;
        }
    }

    return lerp(brdf, bsdf, mat.transmission);
}
#endif


//inline void BSDFTest(Material mat, Mat33 frame, float woTheta, const char* filename)
//{
    /* example code to visualize a BSDF, its PDF, and sampling

    Material mat;
    mat.color = Vec(0.95, 0.9, 0.9);
    mat.specular = 1.0;
    mat.roughness = 0.025;
    mat.metallic = 0.0;

    float3 n = Normalize(float3(1.0f, 0.0f, 0.0f));
    float3 u, v;
    BasisFromVector(n, &u, &v);

    BSDFTest(mat, Mat33(u, v, n), kPi/2.05f, "BSDFtest.pfm");
    */
    /*
    int width = 512;
    int height = 256;

    PfmImage image;
    image.width = width;
    image.height = height;
    image.depth = 1;

    image.data = new float[width * height * 3];

    float3* pixels = (float3*)image.data;

    float3 wo = frame * float3(0.0f, -sinf(woTheta), cosf(woTheta));

    Random rand;

    for (int j = 0; j < height; ++j)
    {
        for (int i = 0; i < width; ++i)
        {
            float u = float(i) / width;
            float v = float(j) / height;

            float3 wi = ProbeUVToDir(Vec2(u, v));

            float3 f = BSDFEval(mat, 1.0f, 1.0f, float3(0.0f), frame.GetCol(2), wo, wi);
            float pdf = BSDFPdf(mat, 1.0f, 1.0f, float3(0.0f), frame.GetCol(2), wo, wi);

            //  f.x = u;
              //f.y = v;
              //f.z = 1.0;
          //    printf("%f %f %f\n", f.x, f.y, f.z);

            pixels[j * width + i] = float3(f.x, pdf, 0.5f);
        }
    }

    int numSamples = 1000;

    for (int i = 0; i < numSamples; ++i)
    {
        float3 wi;
        float pdf;
        BSDFType type;

        BSDFSample(mat, 1.0f, 1.0f, float3(0.0f), frame.GetCol(0), frame.GetCol(1), frame.GetCol(2), wo, wi, pdf, type, rand);

        Vec2 uv = ProbeDirToUV(wi);

        int px = Clamp(int(uv.x * width), 0, width - 1);
        int py = Clamp(int(uv.y * height), 0, height - 1);

        pixels[py * width + px] = float3(1.0f, 0.0f, 0.0f);
    }

    PfmSave(filename, image);
}*/
