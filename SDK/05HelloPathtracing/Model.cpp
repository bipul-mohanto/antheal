// ======================================================================== //
// Copyright 2018-2019 Ingo Wald                                            //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "Model.h"
#define TINYOBJLOADER_IMPLEMENTATION
#include "support/tiny_obj_loader.h"

#include "support/stb/stb_image.h"

//std
#include <set>
#include <sutil/vec_math.h>

#include <iostream>
#include "random.h"

namespace std {
    inline bool operator<(const tinyobj::index_t& a,
        const tinyobj::index_t& b)
    {
        if (a.vertex_index < b.vertex_index) return true;
        if (a.vertex_index > b.vertex_index) return false;

        if (a.normal_index < b.normal_index) return true;
        if (a.normal_index > b.normal_index) return false;

        if (a.texcoord_index < b.texcoord_index) return true;
        if (a.texcoord_index > b.texcoord_index) return false;

        return false;
    }
}


/*! find vertex with given position, normal, texcoord, and return
    its vertex ID, or, if it doesn't exit, add it to the mesh, and
    its just-created index */
int addVertex(TriangleMesh* mesh,
    tinyobj::attrib_t& attributes,
    const tinyobj::index_t& idx,
    std::map<tinyobj::index_t, int>& knownVertices)
{
    if (knownVertices.find(idx) != knownVertices.end())
        return knownVertices[idx];

    const float3* vertex_array = (const float3*)attributes.vertices.data();
    const float3* normal_array = (const float3*)attributes.normals.data();
    const float2* texcoord_array = (const float2*)attributes.texcoords.data();

    int newID = mesh->vertex.size();
    knownVertices[idx] = newID;

    mesh->vertex.push_back(vertex_array[idx.vertex_index]);
    if (idx.normal_index >= 0) {
        while (mesh->normal.size() < mesh->vertex.size())
            mesh->normal.push_back(normal_array[idx.normal_index]);
    }
    if (idx.texcoord_index >= 0) {
        while (mesh->texcoord.size() < mesh->vertex.size())
            mesh->texcoord.push_back(texcoord_array[idx.texcoord_index]);
    }

    // just for sanity's sake:
    if (mesh->texcoord.size() > 0)
        mesh->texcoord.resize(mesh->vertex.size());
    // just for sanity's sake:
    if (mesh->normal.size() > 0)
        mesh->normal.resize(mesh->vertex.size());

    return newID;
}
/*! load a texture (if not already loaded), and return its ID in the
  model's textures[] vector. Textures that could not get loaded
  return -1 */
int loadTexture(Model* model,
    std::map<std::string, int>& knownTextures,
    const std::string& inFileName,
    const std::string& modelPath)
{
    if (inFileName == "")
        return -1;

    if (knownTextures.find(inFileName) != knownTextures.end())
        return knownTextures[inFileName];

    std::string fileName = inFileName;
    // first, fix backspaces:
    for (auto& c : fileName)
        if (c == '\\') c = '/';
    fileName = modelPath + "/" + fileName;

    int2 res;
    int   comp;
    unsigned char* image = stbi_load(fileName.c_str(),
        &res.x, &res.y, &comp, STBI_rgb_alpha);
    int textureID = -1;
    if (image) {
        textureID = (int)model->textures.size();
        Texture* texture = new Texture;
        texture->resolution = res;
        texture->pixel = (uint32_t*)image;

        /* iw - actually, it seems that stbi loads the pictures
           mirrored along the y axis - mirror them here */
        for (int y = 0; y < res.y / 2; y++) {
            uint32_t* line_y = texture->pixel + y * res.x;
            uint32_t* mirrored_y = texture->pixel + (res.y - 1 - y) * res.x;
            int mirror_y = res.y - 1 - y;
            for (int x = 0; x < res.x; x++) {
                std::swap(line_y[x], mirrored_y[x]);
            }
        }

        model->textures.push_back(texture);
    }
    else {
        std::cout << "Could not load texture from " << fileName << "!" << std::endl;
    }

    knownTextures[inFileName] = textureID;
    return textureID;
}

Model* loadOBJ(const std::string& objFile)
{
    Model* model = new Model;

    const std::string modelDir
        = objFile.substr(0, objFile.rfind('/') + 1);

    tinyobj::attrib_t attributes;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err = "";

    bool readOK
        = tinyobj::LoadObj(&attributes,
            &shapes,
            &materials,
            &err,
            &err,
            objFile.c_str(),
            modelDir.c_str(),
            /* triangulate */true);
    if (!readOK) {
        throw std::runtime_error("Could not read OBJ model from " + objFile + " : " + err);
    }

    /*if (materials.empty())
        throw std::runtime_error("could not parse materials ...");*/

    std::cout << "Done loading obj file - found " << shapes.size() << " shapes with " << materials.size() << " materials" << std::endl;
    for (int shapeID = 0; shapeID < (int)shapes.size(); shapeID++) {
        tinyobj::shape_t& shape = shapes[shapeID];

        std::set<int> materialIDs;
        for (auto faceMatID : shape.mesh.material_ids)
            materialIDs.insert(faceMatID);

        std::map<tinyobj::index_t, int> knownVertices;
        std::map<std::string, int>      knownTextures;

        for (int materialID : materialIDs) {
            TriangleMesh* mesh = new TriangleMesh;

            for (int faceID = 0; faceID < shape.mesh.material_ids.size(); faceID++) {
                if (shape.mesh.material_ids[faceID] != materialID) continue;
                tinyobj::index_t idx0 = shape.mesh.indices[3 * faceID + 0];
                tinyobj::index_t idx1 = shape.mesh.indices[3 * faceID + 1];
                tinyobj::index_t idx2 = shape.mesh.indices[3 * faceID + 2];

                uint3 idx = make_uint3(addVertex(mesh, attributes, idx0, knownVertices),
                    addVertex(mesh, attributes, idx1, knownVertices),
                    addVertex(mesh, attributes, idx2, knownVertices));
                mesh->index.push_back(idx);
                mesh->material.color = (const float3&)materials[materialID].diffuse;
                mesh->diffuseTextureID = loadTexture(model,
                    knownTextures,
                    materials[materialID].diffuse_texname,
                    modelDir);
            }

            if (mesh->vertex.empty())
                delete mesh;
            else
                model->meshes.push_back(mesh);
        }
    }

    // of course, you should be using tbb::parallel_for for stuff
    // like this:
    /*for (auto mesh : model->meshes)
        for (auto vtx : mesh->vertex)
            model->bounds.extend(vtx);*/

    std::cout << "created a total of " << model->meshes.size() << " meshes" << std::endl;
    return model;
}

void addBox(Model* model, Material& mat, const float3& pos, const float3& extend)
{
    const float3 A = make_float3(-extend.x + pos.x, -extend.y + pos.y, extend.z + pos.z);
    const float3 B = make_float3(extend.x + pos.x, -extend.y + pos.y, extend.z + pos.z);
    const float3 C = make_float3(extend.x + pos.x, extend.y + pos.y, extend.z + pos.z);
    const float3 D = make_float3(-extend.x + pos.x, extend.y + pos.y, extend.z + pos.z);

    const float3 E = make_float3(-extend.x + pos.x, -extend.y + pos.y, -extend.z + pos.z);
    const float3 F = make_float3(extend.x + pos.x, -extend.y + pos.y, -extend.z + pos.z);
    const float3 G = make_float3(extend.x + pos.x, extend.y + pos.y, -extend.z + pos.z);
    const float3 H = make_float3(-extend.x + pos.x, extend.y + pos.y, -extend.z + pos.z);

    const float3 vertices[] = {
        // Front 
        A,B,C,
        A,C,D,
        // back
        E, H, G,
        E, G, F,
        // left
        E, A, D,
        E, D, H,
        // right
        B, F, G,
        B, G, C,
        // top
        D, C, G,
        D, G, H,
        // bottom
        E, A, B,
        E, B, F
    };

    const float3 normalFront = make_float3(0, 0, 1); // front
    const float3 normalRight = make_float3(1, 0, 0); // right
    const float3 normalBack = make_float3(0, 0, -1); // back
    const float3 normalLeft = make_float3(-1, 0, 0); // left
    const float3 normalBottom = make_float3(0, -1, 0); // bottom
    const float3 normalTop = make_float3(0, 1, 0); // top	

    const float3 normals[] = {
        normalFront, normalFront, normalFront,normalFront, normalFront, normalFront,
        normalBack, normalBack, normalBack,normalBack, normalBack, normalBack,
        normalLeft, normalLeft, normalLeft,normalLeft, normalLeft, normalLeft,
        normalRight, normalRight, normalRight,normalRight, normalRight, normalRight,
        normalTop, normalTop, normalTop,normalTop, normalTop, normalTop,
        normalBottom, normalBottom, normalBottom,normalBottom, normalBottom, normalBottom
    };

    const int indices[] = {
        0,1,2,3,4,5,
        6,7,8,9,10,11,
        12,13,14,15,16,17,
        18,19,20,21,22,23,
        24,25,26,27,28,29,
        30,31,32,33,34,35
    };

    TriangleMesh* mesh = new TriangleMesh;
    for (int i = 0; i < 36; ++i) {
        mesh->vertex.push_back(vertices[i]);
        mesh->normal.push_back(normals[i]);
        mesh->texcoord.push_back(make_float2(0.f, 0.f));
    }

    for (int i = 0; i < 12; ++i) {
        mesh->index.push_back(make_uint3(indices[i * 3], indices[i * 3 + 1], indices[i * 3 + 2]));
    }

    mesh->material = mat;

    model->meshes.push_back(mesh);
}
