#include <vector>
#include <string>

#include <vector_types.h>

#include "Material.h"

/*! a simple indexed triangle mesh that our sample renderer will
      render */
struct TriangleMesh {
    std::vector<float3> vertex;
    std::vector<float3> normal;
    std::vector<float2> texcoord;
    std::vector<uint3> index;

    Material material;

    int                diffuseTextureID{ -1 };
};

struct Texture {
    ~Texture()
    {
        if (pixel) delete[] pixel;
    }

    uint32_t* pixel{ nullptr };
    int2     resolution{ -1 };
};

struct Model {
    ~Model()
    {
        for (auto mesh : meshes) delete mesh;
        for (auto texture : textures) delete texture;
    }

    std::vector<TriangleMesh*> meshes;
    std::vector<Texture*>      textures;
    //! bounding box of all vertices in the model
    //box3f bounds;
};

Model* loadOBJ(const std::string& objFile);

void addBox(Model* model, Material& mat, const float3& pos, const float3& extend);
