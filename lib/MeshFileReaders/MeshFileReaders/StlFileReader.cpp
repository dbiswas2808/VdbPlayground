#include <MeshFileReaders/StlFileReader.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
namespace VdbFields {
MeshFileReaders::StlData MeshFileReaders::readStlFile(const std::string& filename) {
    Assimp::Importer importer;
    unsigned int aiProcessFlags =
        aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType;
    importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
                                aiComponent_NORMALS | aiComponent_TANGENTS_AND_BITANGENTS);
    importer.SetPropertyInteger(AI_CONFIG_PP_SLM_VERTEX_LIMIT,
                                65535);                      // Limit the number of vertices
    importer.SetPropertyBool(AI_CONFIG_PP_FD_REMOVE, true);  // Remove degenerate triangles
    importer.SetPropertyBool(AI_CONFIG_PP_GSN_MAX_SMOOTHING_ANGLE,
                             true);  // Enable maximum smoothing angle
    importer.SetPropertyFloat(AI_CONFIG_PP_GSN_MAX_SMOOTHING_ANGLE, 80.0f);

    const aiScene* scene = importer.ReadFile(filename, aiProcessFlags);

    if (!scene) {
        throw std::runtime_error("Failed to load STL file");
    }


    StlData data;
    if (scene && scene->HasMeshes()) {
        for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
            aiMesh* mesh = scene->mMeshes[i];
            for (unsigned int j = 0; j < mesh->mNumVertices; j++) {
                data.vertices.emplace_back(
                    std::array{mesh->mVertices[j].x, mesh->mVertices[j].y, mesh->mVertices[j].z});
            }

            for (unsigned int j = 0; j < mesh->mNumFaces; j++) {
                data.faces.emplace_back(std::array{static_cast<int>(mesh->mFaces[j].mIndices[0]),
                                                   static_cast<int>(mesh->mFaces[j].mIndices[1]),
                                                   static_cast<int>(mesh->mFaces[j].mIndices[2])});
            }
            // Access mesh data (e.g., mesh->mVertices, mesh->mNormals)
        }
    }

    importer.FreeScene();
    return data;
}

std::vector<MeshFileReaders::ObjData> MeshFileReaders::readObjFile(const std::string& filename) {
    Assimp::Importer importer;
    unsigned int aiProcessFlags =
        aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType;
    importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
                                aiComponent_NORMALS | aiComponent_TANGENTS_AND_BITANGENTS);
    importer.SetPropertyInteger(AI_CONFIG_PP_SLM_VERTEX_LIMIT,
                                65535);                      // Limit the number of vertices
    importer.SetPropertyBool(AI_CONFIG_PP_FD_REMOVE, true);  // Remove degenerate triangles
    importer.SetPropertyBool(AI_CONFIG_PP_GSN_MAX_SMOOTHING_ANGLE,
                             true);  // Enable maximum smoothing angle
    importer.SetPropertyFloat(AI_CONFIG_PP_GSN_MAX_SMOOTHING_ANGLE, 80.0f);

    const aiScene* scene = importer.ReadFile(filename, aiProcessFlags);

    std::vector<ObjData> meshes;
    int numVertices = 0;
    int numFaces = 0;
    if (scene && scene->HasMeshes()) {
        for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
            ObjData data;
            aiMesh* mesh = scene->mMeshes[i];
            numVertices = mesh->mNumVertices;
            for (unsigned int j = 0; j < mesh->mNumVertices; j++) {
                data.vertices.emplace_back(
                    std::array{mesh->mVertices[j].x, mesh->mVertices[j].y, mesh->mVertices[j].z});
                data.normals.emplace_back(
                    std::array{mesh->mNormals[j].x, mesh->mNormals[j].y, mesh->mNormals[j].z});
            }

            numFaces = mesh->mNumFaces;
            for (unsigned int j = 0; j < mesh->mNumFaces; j++) {
                data.faces.emplace_back(std::array{static_cast<int>(mesh->mFaces[j].mIndices[0]),
                                                   static_cast<int>(mesh->mFaces[j].mIndices[1]),
                                                   static_cast<int>(mesh->mFaces[j].mIndices[2])});
            }

            meshes.push_back(std::move(data));
        }
    }

    if (!scene) {
        throw std::runtime_error("Failed to load OBJ file");
    }

    return meshes;
}
}  // namespace VdbFields