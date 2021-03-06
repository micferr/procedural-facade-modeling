//
// # Yocto/glTF: Tiny C++ Library for glTF loading/saving.
//
// Yocto/glTF is a C++ library for loading and saving Khronos gkTF 2.0 with
// support for the whole 2.0 specification and the Pbr and binary glTF
// extensions. All parsing and writing code is autogenerated form the schema.
//
// The library provides a low  level interface that is a direct
// C++ translation of the glTF schemas and should be used if one wants
// complete control over the format or an application wants to have their
// own scene code added. A higher-level interface is provided by Yocto/Scene.
//
// glTF is a very complex file format and was designed mainly with untyped
// languages in mind. We attempt to match the glTF low-level interface
// to C++ as best as it can. Since the code is generated from the schema, we
// follow glTF naming conventions and typing well. To simplify adoption
// and keep the API relatively simple we use vector as arrays and use
// pointers to reference to all glTF objects. While this makes it less
// efficient than it might have been, glTF heavy use of optional values makes
// this necessary. At the same time, we do not keep track of set/unset values
// for basic types (int, float, bool) as a compromise for efficiency.
//
// glTF uses integer indices to access objects.
// While writing code ourselves we found that we add significant problems
// since we would use an index to access the wrong type of scene objects.
// For this reasons, we use an explicit index `glTFid<T>` that can only access
// an object of type T. Internally this is just the same old glTF index. But
// this can used to access the scene data with `glTF::get<T>(index)`.
//
// ## Usage
//
// 1. load a glTF model with `load_gltf()`
// 2. look at the `glTFXXX` data structures for access to individual elements
// 3. save glTF back to disk with `save_gltf()`
//
//

//
// LICENSE:
//
// Copyright (c) 2016 -- 2018 Fabio Pellacini
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
// LICENSE OF INCLUDED CODE FOR BASE64 (base64.h, base64.cpp)
//
// Copyright (C) 2004-2008 René Nyffenegger
//
// This source code is provided 'as-is', without any express or implied
// warranty. In no event will the author be held liable for any damages
// arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
//
// 1. The origin of this source code must not be misrepresented; you must not
// claim that you wrote the original source code. If you use this source code
// in a product, an acknowledgment in the product documentation would be
// appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not be
// misrepresented as being the original source code.
//
// 3. This notice may not be removed or altered from any source distribution.
//
// René Nyffenegger rene.nyffenegger@adp-gmbh.ch
//
//
//

#ifndef _YGL_GLTF_H_
#define _YGL_GLTF_H_

// -----------------------------------------------------------------------------
// COMPILATION OPTIONS AND INCLUDES
// -----------------------------------------------------------------------------

#ifndef YGL_GLTFJSON
#define YGL_GLTFJSON 0
#endif

#include "yocto_math.h"

#include <map>
#include <string>
#include <vector>

// include json for glTF
#if YGL_GLTFJSON
#include "ext/json.hpp"
#endif

// -----------------------------------------------------------------------------
// KHRONOS GLTF SUPPORT
// -----------------------------------------------------------------------------
namespace ygl {

// Generic buffer data.
using buffer_data = std::vector<unsigned char>;

// Id for glTF references.
template <typename T>
struct glTFid {
    // Defaoult constructor to an invalid id.
    glTFid() : _id(-1) {}
    // Explicit conversion from integer.
    explicit glTFid(int id) : _id(id) {}
    // Explicit convcersion to integer.
    explicit operator int() const { return _id; }
    // Check if it is valid.
    bool is_valid() const { return _id >= 0; }
    // Check if it is valid.
    explicit operator bool() const { return _id >= 0; }

   private:
    // id
    int _id = -1;
};

// Generic glTF object.
struct glTFProperty {
#if YGL_GLTFJSON
    // Extensions.
    map<string, nlohmann::json> extensions = {};
    // Extra data.
    nlohmann::json extras = {};
#endif
};

// #codegen begin gltf-type

// forward declaration
struct glTFChildOfRootProperty;
struct glTFAccessorSparseIndices;
struct glTFAccessorSparseValues;
struct glTFAccessorSparse;
struct glTFAccessor;
struct glTFAnimationChannelTarget;
struct glTFAnimationChannel;
struct glTFAnimationSampler;
struct glTFAnimation;
struct glTFAsset;
struct glTFBuffer;
struct glTFBufferView;
struct glTFCameraOrthographic;
struct glTFCameraPerspective;
struct glTFCamera;
struct glTFImage;
struct glTFTextureInfo;
struct glTFTexture;
struct glTFMaterialNormalTextureInfo;
struct glTFMaterialOcclusionTextureInfo;
struct glTFMaterialPbrMetallicRoughness;
struct glTFMaterialPbrSpecularGlossiness;
struct glTFMaterial;
struct glTFMeshPrimitive;
struct glTFMesh;
struct glTFNode;
struct glTFSampler;
struct glTFScene;
struct glTFSkin;
struct glTF;

// Generic glTF named object
struct glTFChildOfRootProperty : glTFProperty {
    // The user-defined name of this object.
    std::string name = "";
};

// Values for glTFAccessorSparseIndices::componentType
enum class glTFAccessorSparseIndicesComponentType {
    // Not set
    NotSet = -1,
    // UnsignedByte
    UnsignedByte = 5121,
    // UnsignedShort
    UnsignedShort = 5123,
    // UnsignedInt
    UnsignedInt = 5125,
};

// Indices of those attributes that deviate from their initialization value.
struct glTFAccessorSparseIndices : glTFProperty {
    // The index of the bufferView with sparse indices. Referenced bufferView
    // can't have ARRAY_BUFFER or ELEMENT_ARRAY_BUFFER target. [required]
    glTFid<glTFBufferView> bufferView = {};
    // The offset relative to the start of the bufferView in bytes. Must be
    // aligned.
    int byteOffset = 0;
    // The indices data type. [required]
    glTFAccessorSparseIndicesComponentType componentType =
        glTFAccessorSparseIndicesComponentType::NotSet;
};

// Array of size `accessor.sparse.count` times number of components storing the
// displaced accessor attributes pointed by `accessor.sparse.indices`.
struct glTFAccessorSparseValues : glTFProperty {
    // The index of the bufferView with sparse values. Referenced bufferView
    // can't have ARRAY_BUFFER or ELEMENT_ARRAY_BUFFER target. [required]
    glTFid<glTFBufferView> bufferView = {};
    // The offset relative to the start of the bufferView in bytes. Must be
    // aligned.
    int byteOffset = 0;
};

// Sparse storage of attributes that deviate from their initialization value.
struct glTFAccessorSparse : glTFProperty {
    // Number of entries stored in the sparse array. [required]
    int count = 0;
    // Index array of size `count` that points to those accessor attributes
    // that deviate from their initialization value. Indices must strictly
    // increase. [required]
    glTFAccessorSparseIndices* indices = nullptr;
    // Array of size `count` times number of components, storing the displaced
    // accessor attributes pointed by `indices`. Substituted values must have
    // the same `componentType` and number of components as the base accessor.
    // [required]
    glTFAccessorSparseValues* values = nullptr;

    ~glTFAccessorSparse() {
        if (indices) delete indices;
        if (values) delete values;
    }
};

// Values for glTFAccessor::componentType
enum class glTFAccessorComponentType {
    // Not set
    NotSet = -1,
    // Byte
    Byte = 5120,
    // UnsignedByte
    UnsignedByte = 5121,
    // Short
    Short = 5122,
    // UnsignedShort
    UnsignedShort = 5123,
    // UnsignedInt
    UnsignedInt = 5125,
    // Float
    Float = 5126,
};

// Values for glTFAccessor::type
enum class glTFAccessorType {
    // Not set
    NotSet = -1,
    // Scalar
    Scalar = 0,
    // Vec2
    Vec2 = 1,
    // Vec3
    Vec3 = 2,
    // Vec4
    Vec4 = 3,
    // Mat2
    Mat2 = 4,
    // Mat3
    Mat3 = 5,
    // Mat4
    Mat4 = 6,
};

// A typed view into a bufferView.  A bufferView contains raw binary data.  An
// accessor provides a typed view into a bufferView or a subset of a bufferView
// similar to how WebGL's `vertexAttribPointer()` defines an attribute in a
// buffer.
struct glTFAccessor : glTFChildOfRootProperty {
    // The index of the bufferView.
    glTFid<glTFBufferView> bufferView = {};
    // The offset relative to the start of the bufferView in bytes.
    int byteOffset = 0;
    // The datatype of components in the attribute. [required]
    glTFAccessorComponentType componentType = glTFAccessorComponentType::NotSet;
    // Specifies whether integer data values should be normalized.
    bool normalized = false;
    // The number of attributes referenced by this accessor. [required]
    int count = 0;
    // Specifies if the attribute is a scalar, vector, or matrix. [required]
    glTFAccessorType type = glTFAccessorType::NotSet;
    // Maximum value of each component in this attribute.
    std::vector<float> max = {};
    // Minimum value of each component in this attribute.
    std::vector<float> min = {};
    // Sparse storage of attributes that deviate from their initialization
    // value.
    glTFAccessorSparse* sparse = nullptr;

    ~glTFAccessor() {
        if (sparse) delete sparse;
    }
};

// Values for glTFAnimationChannelTarget::path
enum class glTFAnimationChannelTargetPath {
    // Not set
    NotSet = -1,
    // Translation
    Translation = 0,
    // Rotation
    Rotation = 1,
    // Scale
    Scale = 2,
    // Weights
    Weights = 3,
};

// The index of the node and TRS property that an animation channel targets.
struct glTFAnimationChannelTarget : glTFProperty {
    // The index of the node to target. [required]
    glTFid<glTFNode> node = {};
    // The name of the node's TRS property to modify, or the "weights" of the
    // Morph Targets it instantiates. [required]
    glTFAnimationChannelTargetPath path =
        glTFAnimationChannelTargetPath::NotSet;
};

// Targets an animation's sampler at a node's property.
struct glTFAnimationChannel : glTFProperty {
    // The index of a sampler in this animation used to compute the value for
    // the target. [required]
    glTFid<glTFAnimationSampler> sampler = {};
    // The index of the node and TRS property to target. [required]
    glTFAnimationChannelTarget* target = nullptr;

    ~glTFAnimationChannel() {
        if (target) delete target;
    }
};

// Values for glTFAnimationSampler::interpolation
enum class glTFAnimationSamplerInterpolation {
    // Not set
    NotSet = -1,
    // The animated values are linearly interpolated between keyframes. When
    // targeting a rotation, spherical linear interpolation (slerp) should be
    // used to interpolate quaternions. The number output of elements must equal
    // the number of input elements.
    Linear = 0,
    // The animated values remain constant to the output of the first keyframe,
    // until the next keyframe. The number of output elements must equal the
    // number of input elements.
    Step = 1,
    // The animation's interpolation is computed using a cubic spline with
    // specified tangents. The number of output elements must equal three times
    // the number of input elements. For each input element, the output stores
    // three elements, an in-tangent, a spline vertex, and an out-tangent. There
    // must be at least two keyframes when using this interpolation.
    CubicSpline = 3,
};

// Combines input and output accessors with an interpolation algorithm to
// define a keyframe graph (but not its target).
struct glTFAnimationSampler : glTFProperty {
    // The index of an accessor containing keyframe input values, e.g., time.
    // [required]
    glTFid<glTFAccessor> input = {};
    // Interpolation algorithm.
    glTFAnimationSamplerInterpolation interpolation =
        glTFAnimationSamplerInterpolation::Linear;
    // The index of an accessor, containing keyframe output values. [required]
    glTFid<glTFAccessor> output = {};
};

// A keyframe animation.
struct glTFAnimation : glTFChildOfRootProperty {
    // An array of channels, each of which targets an animation's sampler at a
    // node's property. Different channels of the same animation can't have
    // equal targets. [required]
    std::vector<glTFAnimationChannel*> channels = {};
    // An array of samplers that combines input and output accessors with an
    // interpolation algorithm to define a keyframe graph (but not its target).
    // [required]
    std::vector<glTFAnimationSampler*> samplers = {};

    // typed access for nodes
    glTFAnimationChannel* get(const glTFid<glTFAnimationChannel>& id) const {
        if (!id) return nullptr;
        return channels.at((int)id);
    }
    // typed access for nodes
    glTFAnimationSampler* get(const glTFid<glTFAnimationSampler>& id) const {
        if (!id) return nullptr;
        return samplers.at((int)id);
    }

    ~glTFAnimation() {
        for (auto v : channels) delete v;
        for (auto v : samplers) delete v;
    }
};

// Metadata about the glTF asset.
struct glTFAsset : glTFProperty {
    // A copyright message suitable for display to credit the content creator.
    std::string copyright = "";
    // Tool that generated this glTF model.  Useful for debugging.
    std::string generator = "";
    // The glTF version that this asset targets. [required]
    std::string version = "";
    // The minimum glTF version that this asset targets.
    std::string minVersion = "";
};

// A buffer points to binary geometry, animation, or skins.
struct glTFBuffer : glTFChildOfRootProperty {
    // The uri of the buffer.
    std::string uri = "";
    // The length of the buffer in bytes. [required]
    int byteLength = 0;
    // Stores buffer content after loading. [required]
    buffer_data data = {};
};

// Values for glTFBufferView::target
enum class glTFBufferViewTarget {
    // Not set
    NotSet = -1,
    // ArrayBuffer
    ArrayBuffer = 34962,
    // ElementArrayBuffer
    ElementArrayBuffer = 34963,
};

// A view into a buffer generally representing a subset of the buffer.
struct glTFBufferView : glTFChildOfRootProperty {
    // The index of the buffer. [required]
    glTFid<glTFBuffer> buffer = {};
    // The offset into the buffer in bytes.
    int byteOffset = 0;
    // The length of the bufferView in bytes. [required]
    int byteLength = 0;
    // The stride, in bytes.
    int byteStride = 0;
    // The target that the GPU buffer should be bound to.
    glTFBufferViewTarget target = glTFBufferViewTarget::NotSet;
};

// An orthographic camera containing properties to create an orthographic
// projection matrix.
struct glTFCameraOrthographic : glTFProperty {
    // The floating-point horizontal magnification of the view. [required]
    float xmag = 0;
    // The floating-point vertical magnification of the view. [required]
    float ymag = 0;
    // The floating-point distance to the far clipping plane. `zfar` must be
    // greater than `znear`. [required]
    float zfar = 0;
    // The floating-point distance to the near clipping plane. [required]
    float znear = 0;
};

// A perspective camera containing properties to create a perspective
// projection matrix.
struct glTFCameraPerspective : glTFProperty {
    // The floating-point aspect ratio of the field of view.
    float aspectRatio = 0;
    // The floating-point vertical field of view in radians. [required]
    float yfov = 0;
    // The floating-point distance to the far clipping plane.
    float zfar = 0;
    // The floating-point distance to the near clipping plane. [required]
    float znear = 0;
};

// Values for glTFCamera::type
enum class glTFCameraType {
    // Not set
    NotSet = -1,
    // Perspective
    Perspective = 0,
    // Orthographic
    Orthographic = 1,
};

// A camera's projection.  A node can reference a camera to apply a transform
// to place the camera in the scene.
struct glTFCamera : glTFChildOfRootProperty {
    // An orthographic camera containing properties to create an orthographic
    // projection matrix.
    glTFCameraOrthographic* orthographic = nullptr;
    // A perspective camera containing properties to create a perspective
    // projection matrix.
    glTFCameraPerspective* perspective = nullptr;
    // Specifies if the camera uses a perspective or orthographic projection.
    // [required]
    glTFCameraType type = glTFCameraType::NotSet;

    ~glTFCamera() {
        if (orthographic) delete orthographic;
        if (perspective) delete perspective;
    }
};

// Values for glTFImage::mimeType
enum class glTFImageMimeType {
    // Not set
    NotSet = -1,
    // ImageJpeg
    ImageJpeg = 0,
    // ImagePng
    ImagePng = 1,
};

// Image data used to create a texture. Image can be referenced by URI or
// `bufferView` index. `mimeType` is required in the latter case.
struct glTFImage : glTFChildOfRootProperty {
    // The uri of the image.
    std::string uri = "";
    // The image's MIME type.
    glTFImageMimeType mimeType = glTFImageMimeType::NotSet;
    // The index of the bufferView that contains the image. Use this instead of
    // the image's uri property.
    glTFid<glTFBufferView> bufferView = {};
};

// Reference to a texture.
struct glTFTextureInfo : glTFProperty {
    // The index of the texture. [required]
    glTFid<glTFTexture> index = {};
    // The set index of texture's TEXCOORD attribute used for texture
    // coordinate mapping.
    int texCoord = 0;
};

// A texture and its sampler.
struct glTFTexture : glTFChildOfRootProperty {
    // The index of the sampler used by this texture. When undefined, a sampler
    // with repeat wrapping and auto filtering should be used.
    glTFid<glTFSampler> sampler = {};
    // The index of the image used by this texture.
    glTFid<glTFImage> source = {};
};

// Normal texture information.
struct glTFMaterialNormalTextureInfo : glTFTextureInfo {
    // The scalar multiplier applied to each normal vector of the normal
    // texture.
    float scale = 1;
};

// Occlusion texture information.
struct glTFMaterialOcclusionTextureInfo : glTFTextureInfo {
    // A scalar multiplier controlling the amount of occlusion applied.
    float strength = 1;
};

// A set of parameter values that are used to define the metallic-roughness
// material model from Physically-Based Rendering (PBR) methodology.
struct glTFMaterialPbrMetallicRoughness : glTFProperty {
    // The material's base color factor.
    vec4f baseColorFactor = {1, 1, 1, 1};
    // The base color texture.
    glTFTextureInfo* baseColorTexture = nullptr;
    // The metalness of the material.
    float metallicFactor = 1;
    // The roughness of the material.
    float roughnessFactor = 1;
    // The metallic-roughness texture.
    glTFTextureInfo* metallicRoughnessTexture = nullptr;

    ~glTFMaterialPbrMetallicRoughness() {
        if (baseColorTexture) delete baseColorTexture;
        if (metallicRoughnessTexture) delete metallicRoughnessTexture;
    }
};

// glTF extension that defines the specular-glossiness material model from
// Physically-Based Rendering (PBR) methodology.
struct glTFMaterialPbrSpecularGlossiness : glTFProperty {
    // The reflected diffuse factor of the material.
    vec4f diffuseFactor = {1, 1, 1, 1};
    // The diffuse texture.
    glTFTextureInfo* diffuseTexture = nullptr;
    // The specular RGB color of the material.
    vec3f specularFactor = {1, 1, 1};
    // The glossiness or smoothness of the material.
    float glossinessFactor = 1;
    // The specular-glossiness texture.
    glTFTextureInfo* specularGlossinessTexture = nullptr;

    ~glTFMaterialPbrSpecularGlossiness() {
        if (diffuseTexture) delete diffuseTexture;
        if (specularGlossinessTexture) delete specularGlossinessTexture;
    }
};

// Values for glTFMaterial::alphaMode
enum class glTFMaterialAlphaMode {
    // Not set
    NotSet = -1,
    // The alpha value is ignored and the rendered output is fully opaque.
    Opaque = 0,
    // The rendered output is either fully opaque or fully transparent depending
    // on the alpha value and the specified alpha cutoff value.
    Mask = 1,
    // The alpha value is used to composite the source and destination areas.
    // The rendered output is combined with the background using the normal
    // painting operation (i.e. the Porter and Duff over operator).
    Blend = 2,
};

// The material appearance of a primitive.
struct glTFMaterial : glTFChildOfRootProperty {
    // A set of parameter values that are used to define the metallic-roughness
    // material model from Physically-Based Rendering (PBR) methodology. When
    // not specified, all the default values of `pbrMetallicRoughness` apply.
    glTFMaterialPbrMetallicRoughness* pbrMetallicRoughness = nullptr;
    // A set of parameter values that are used to define the
    // specular-glossiness material model from Physically-Based Rendering (PBR)
    // methodology. When not specified, all the default values of
    // `pbrMetallicRoughness` apply.
    glTFMaterialPbrSpecularGlossiness* pbrSpecularGlossiness = nullptr;
    // The normal map texture.
    glTFMaterialNormalTextureInfo* normalTexture = nullptr;
    // The occlusion map texture.
    glTFMaterialOcclusionTextureInfo* occlusionTexture = nullptr;
    // The emissive map texture.
    glTFTextureInfo* emissiveTexture = nullptr;
    // The emissive color of the material.
    vec3f emissiveFactor = {0, 0, 0};
    // The alpha rendering mode of the material.
    glTFMaterialAlphaMode alphaMode = glTFMaterialAlphaMode::Opaque;
    // The alpha cutoff value of the material.
    float alphaCutoff = 0.5;
    // Specifies whether the material is double sided.
    bool doubleSided = false;

    ~glTFMaterial() {
        if (pbrMetallicRoughness) delete pbrMetallicRoughness;
        if (pbrSpecularGlossiness) delete pbrSpecularGlossiness;
        if (normalTexture) delete normalTexture;
        if (occlusionTexture) delete occlusionTexture;
        if (emissiveTexture) delete emissiveTexture;
    }
};

// Values for glTFMeshPrimitive::mode
enum class glTFMeshPrimitiveMode {
    // Not set
    NotSet = -1,
    // Points
    Points = 0,
    // Lines
    Lines = 1,
    // LineLoop
    LineLoop = 2,
    // LineStrip
    LineStrip = 3,
    // Triangles
    Triangles = 4,
    // TriangleStrip
    TriangleStrip = 5,
    // TriangleFan
    TriangleFan = 6,
};

// Geometry to be rendered with the given material.
struct glTFMeshPrimitive : glTFProperty {
    // A dictionary object, where each key corresponds to mesh attribute
    // semantic and each value is the index of the accessor containing
    // attribute's data. [required]
    std::map<std::string, glTFid<glTFAccessor>> attributes = {};
    // The index of the accessor that contains the indices.
    glTFid<glTFAccessor> indices = {};
    // The index of the material to apply to this primitive when rendering.
    glTFid<glTFMaterial> material = {};
    // The type of primitives to render.
    glTFMeshPrimitiveMode mode = glTFMeshPrimitiveMode::Triangles;
    // An array of Morph Targets, each  Morph Target is a dictionary mapping
    // attributes (only `POSITION`, `NORMAL`, and `TANGENT` supported) to their
    // deviations in the Morph Target.
    std::vector<std::map<std::string, glTFid<glTFAccessor>>> targets = {};
};

// A set of primitives to be rendered.  A node can contain one mesh.  A node's
// transform places the mesh in the scene.
struct glTFMesh : glTFChildOfRootProperty {
    // An array of primitives, each defining geometry to be rendered with a
    // material. [required]
    std::vector<glTFMeshPrimitive*> primitives = {};
    // Array of weights to be applied to the Morph Targets.
    std::vector<float> weights = {};

    ~glTFMesh() {
        for (auto v : primitives) delete v;
    }
};

// A node in the node hierarchy.  When the node contains `skin`, all
// `mesh.primitives` must contain `JOINTS_0` and `WEIGHTS_0` attributes.  A
// node can have either a `matrix` or any combination of
// `translation`/`rotation`/`scale` (TRS) properties. TRS properties are
// converted to matrices and postmultiplied in the `T * R * S` order to compose
// the transformation matrix; first the scale is applied to the vertices, then
// the rotation, and then the translation. If none are provided, the transform
// is the identity. When a node is targeted for animation (referenced by an
// animation.channel.target), only TRS properties may be present; `matrix` will
// not be present.
struct glTFNode : glTFChildOfRootProperty {
    // The index of the camera referenced by this node.
    glTFid<glTFCamera> camera = {};
    // The indices of this node's children.
    std::vector<glTFid<glTFNode>> children = {};
    // The index of the skin referenced by this node.
    glTFid<glTFSkin> skin = {};
    // A floating-point 4x4 transformation matrix stored in column-major order.
    mat4f matrix = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    // The index of the mesh in this node.
    glTFid<glTFMesh> mesh = {};
    // The node's unit quaternion rotation in the order (x, y, z, w), where w
    // is the scalar.
    vec4f rotation = {0, 0, 0, 1};
    // The node's non-uniform scale.
    vec3f scale = {1, 1, 1};
    // The node's translation.
    vec3f translation = {0, 0, 0};
    // The weights of the instantiated Morph Target. Number of elements must
    // match number of Morph Targets of used mesh.
    std::vector<float> weights = {};
};

// Values for glTFSampler::magFilter
enum class glTFSamplerMagFilter {
    // Not set
    NotSet = -1,
    // Nearest
    Nearest = 9728,
    // Linear
    Linear = 9729,
};

// Values for glTFSampler::minFilter
enum class glTFSamplerMinFilter {
    // Not set
    NotSet = -1,
    // Nearest
    Nearest = 9728,
    // Linear
    Linear = 9729,
    // NearestMipmapNearest
    NearestMipmapNearest = 9984,
    // LinearMipmapNearest
    LinearMipmapNearest = 9985,
    // NearestMipmapLinear
    NearestMipmapLinear = 9986,
    // LinearMipmapLinear
    LinearMipmapLinear = 9987,
};

// glTFSampler::wrapS
enum class glTFSamplerWrapS {
    // Not set
    NotSet = -1,
    // ClampToEdge
    ClampToEdge = 33071,
    // MirroredRepeat
    MirroredRepeat = 33648,
    // Repeat
    Repeat = 10497,
};

// glTFSampler::wrapT
enum class glTFSamplerWrapT {
    // Not set
    NotSet = -1,
    // ClampToEdge
    ClampToEdge = 33071,
    // MirroredRepeat
    MirroredRepeat = 33648,
    // Repeat
    Repeat = 10497,
};

// Texture sampler properties for filtering and wrapping modes.
struct glTFSampler : glTFChildOfRootProperty {
    // Magnification filter.
    glTFSamplerMagFilter magFilter = glTFSamplerMagFilter::NotSet;
    // Minification filter.
    glTFSamplerMinFilter minFilter = glTFSamplerMinFilter::NotSet;
    // s wrapping mode.
    glTFSamplerWrapS wrapS = glTFSamplerWrapS::Repeat;
    // t wrapping mode.
    glTFSamplerWrapT wrapT = glTFSamplerWrapT::Repeat;
};

// The root nodes of a scene.
struct glTFScene : glTFChildOfRootProperty {
    // The indices of each root node.
    std::vector<glTFid<glTFNode>> nodes = {};
};

// Joints and matrices defining a skin.
struct glTFSkin : glTFChildOfRootProperty {
    // The index of the accessor containing the floating-point 4x4 inverse-bind
    // matrices.  The default is that each matrix is a 4x4 identity matrix,
    // which implies that inverse-bind matrices were pre-applied.
    glTFid<glTFAccessor> inverseBindMatrices = {};
    // The index of the node used as a skeleton root. When undefined, joints
    // transforms resolve to scene root.
    glTFid<glTFNode> skeleton = {};
    // Indices of skeleton nodes, used as joints in this skin. [required]
    std::vector<glTFid<glTFNode>> joints = {};
};

// The root object for a glTF asset.
struct glTF : glTFProperty {
    // Names of glTF extensions used somewhere in this asset.
    std::vector<std::string> extensionsUsed = {};
    // Names of glTF extensions required to properly load this asset.
    std::vector<std::string> extensionsRequired = {};
    // An array of accessors.
    std::vector<glTFAccessor*> accessors = {};
    // An array of keyframe animations.
    std::vector<glTFAnimation*> animations = {};
    // Metadata about the glTF asset. [required]
    glTFAsset* asset = nullptr;
    // An array of buffers.
    std::vector<glTFBuffer*> buffers = {};
    // An array of bufferViews.
    std::vector<glTFBufferView*> bufferViews = {};
    // An array of cameras.
    std::vector<glTFCamera*> cameras = {};
    // An array of images.
    std::vector<glTFImage*> images = {};
    // An array of materials.
    std::vector<glTFMaterial*> materials = {};
    // An array of meshes.
    std::vector<glTFMesh*> meshes = {};
    // An array of nodes.
    std::vector<glTFNode*> nodes = {};
    // An array of samplers.
    std::vector<glTFSampler*> samplers = {};
    // The index of the default scene.
    glTFid<glTFScene> scene = {};
    // An array of scenes.
    std::vector<glTFScene*> scenes = {};
    // An array of skins.
    std::vector<glTFSkin*> skins = {};
    // An array of textures.
    std::vector<glTFTexture*> textures = {};

    // typed access for nodes
    glTFAccessor* get(const glTFid<glTFAccessor>& id) const {
        if (!id) return nullptr;
        return accessors.at((int)id);
    }
    // typed access for nodes
    glTFAnimation* get(const glTFid<glTFAnimation>& id) const {
        if (!id) return nullptr;
        return animations.at((int)id);
    }
    // typed access for nodes
    glTFBuffer* get(const glTFid<glTFBuffer>& id) const {
        if (!id) return nullptr;
        return buffers.at((int)id);
    }
    // typed access for nodes
    glTFBufferView* get(const glTFid<glTFBufferView>& id) const {
        if (!id) return nullptr;
        return bufferViews.at((int)id);
    }
    // typed access for nodes
    glTFCamera* get(const glTFid<glTFCamera>& id) const {
        if (!id) return nullptr;
        return cameras.at((int)id);
    }
    // typed access for nodes
    glTFImage* get(const glTFid<glTFImage>& id) const {
        if (!id) return nullptr;
        return images.at((int)id);
    }
    // typed access for nodes
    glTFMaterial* get(const glTFid<glTFMaterial>& id) const {
        if (!id) return nullptr;
        return materials.at((int)id);
    }
    // typed access for nodes
    glTFMesh* get(const glTFid<glTFMesh>& id) const {
        if (!id) return nullptr;
        return meshes.at((int)id);
    }
    // typed access for nodes
    glTFNode* get(const glTFid<glTFNode>& id) const {
        if (!id) return nullptr;
        return nodes.at((int)id);
    }
    // typed access for nodes
    glTFSampler* get(const glTFid<glTFSampler>& id) const {
        if (!id) return nullptr;
        return samplers.at((int)id);
    }
    // typed access for nodes
    glTFScene* get(const glTFid<glTFScene>& id) const {
        if (!id) return nullptr;
        return scenes.at((int)id);
    }
    // typed access for nodes
    glTFSkin* get(const glTFid<glTFSkin>& id) const {
        if (!id) return nullptr;
        return skins.at((int)id);
    }
    // typed access for nodes
    glTFTexture* get(const glTFid<glTFTexture>& id) const {
        if (!id) return nullptr;
        return textures.at((int)id);
    }

    ~glTF() {
        for (auto v : accessors) delete v;
        for (auto v : animations) delete v;
        if (asset) delete asset;
        for (auto v : buffers) delete v;
        for (auto v : bufferViews) delete v;
        for (auto v : cameras) delete v;
        for (auto v : images) delete v;
        for (auto v : materials) delete v;
        for (auto v : meshes) delete v;
        for (auto v : nodes) delete v;
        for (auto v : samplers) delete v;
        for (auto v : scenes) delete v;
        for (auto v : skins) delete v;
        for (auto v : textures) delete v;
    }
};
// #codegen end gltf-type

// Load a gltf file `filename` from disk.
glTF* load_gltf(const std::string& filename, bool load_bin = true);
// Load a binary gltf file `filename` from disk.
glTF* load_binary_gltf(const std::string& filename, bool load_bin = true);

// Save a gltf file `filename` to disk.
void save_gltf(
    const std::string& filename, const glTF* gltf, bool save_bin = true);
// Save a gltf file `filename` to disk.
void save_binary_gltf(
    const std::string& filename, const glTF* gltf, bool save_bin = true);

// Computes the local node transform and its inverse.
inline mat4f node_transform(const glTFNode* node) {
    return frame_to_mat(translation_frame(node->translation) *
                        rotation_frame(node->rotation) *
                        scaling_frame(node->scale)) *
           node->matrix;
}

// A view for gltf array buffers that allows for typed access.
struct accessor_view {
    // Construct a view from an accessor.
    accessor_view(const glTF* gltf, const glTFAccessor* accessor);

    // Number of elements in the view.
    int size() const { return _size; }
    // Number of elements in the view
    int count() const { return _size; }
    // Number of components per element
    int ncomp() const { return _ncomp; }
    // Check whether the view is valid.
    bool valid() const { return _valid; }

    // Get the idx-th element of fixed length width default values.
    vec2f getv2f(int idx, const vec2f& def = {0, 0}) const {
        auto v = def;
        for (auto i = 0; i < min(_ncomp, 2); i++) (&v.x)[i] = get(idx, i);
        return v;
    }
    // Get the idx-th element of fixed length width default values.
    vec3f getv3f(int idx, const vec3f& def = {0, 0, 0}) const {
        auto v = def;
        for (auto i = 0; i < min(_ncomp, 3); i++) (&v.x)[i] = get(idx, i);
        return v;
    }
    // Get the idx-th element of fixed length width default values.
    vec4f getv4f(int idx, const vec4f& def = {0, 0, 0, 0}) const {
        auto v = def;
        for (auto i = 0; i < min(_ncomp, 4); i++) (&v.x)[i] = get(idx, i);
        return v;
    }

    // Get the idx-th element of fixed length as a matrix.
    mat4f getm4f(int idx) const {
        auto v = mat4f();
        auto vm = &v.x.x;
        for (auto i = 0; i < 16; i++) vm[i] = get(idx, i);
        return v;
    }

    // Get the c-th component of the idx-th element.
    float get(int idx, int c = 0) const;

    // Get the idx-th element as integer with fixed length.
    vec2i getv2i(int idx, const vec2i& def = {0, 0}) const {
        auto v = def;
        for (auto i = 0; i < min(_ncomp, 2); i++) { (&v.x)[i] = geti(idx, i); }
        return v;
    }
    // Get the idx-th element as integer with fixed length.
    vec3i getv3i(int idx, const vec3i& def = {0, 0, 0}) const {
        auto v = def;
        for (auto i = 0; i < min(_ncomp, 3); i++) { (&v.x)[i] = geti(idx, i); }
        return v;
    }
    // Get the idx-th element as integer with fixed length.
    vec4i getv4i(int idx, const vec4i& def = {0, 0, 0, 0}) const {
        auto v = def;
        for (auto i = 0; i < min(_ncomp, 4); i++) { (&v.x)[i] = geti(idx, i); }
        return v;
    }

    // Get the c-th component of the idx-th element as integer.
    int geti(int idx, int c = 0) const;

   private:
    const unsigned char* _data = nullptr;
    int _size = 0;
    int _stride = 0;
    int _ncomp = 0;
    glTFAccessorComponentType _ctype;
    bool _normalize = false;
    bool _valid = false;

    static int _num_components(glTFAccessorType type);
    static int _ctype_size(glTFAccessorComponentType componentType);
};

}  // namespace ygl

#endif
