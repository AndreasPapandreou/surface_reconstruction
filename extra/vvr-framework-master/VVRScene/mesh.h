#ifndef VVR_MESH_H
#define VVR_MESH_H

#include "vvrscenedll.h"
#include "utils.h"
#include "scene.h"
#include <MathGeoLib.h>
#include <vector>
#include <string>
#include <list>
#include <set>

namespace vvr {

    math::AABB aabbFromVertices(const std::vector<math::vec> &vertices);

/**
 * Enum used to control what to draw in a call of draw()
 */
enum VVRScene_API Style {
    SOLID = (1 << 0),
    WIRE = (1 << 1),
    NORMALS = (1 << 2),
    BOUND = (1 << 3),
    AXES = (1 << 4),
};

struct VVRScene_API Triangle
{
    /**
     * Indices to the veclist
     */
    union
    {
        struct { int vi1, vi2, vi3; };
        int v[3];
    };

    /**
     * Vector3fer to the std::vector containing the vertices
     */
    std::vector<vec> *vecList;

    /**
     * Plane equation coefficients
     */
    double A, B, C, D;

    Triangle(std::vector<vec> *vecList, int v1 = 0, int v2 = 0, int v3 = 0) :
        vi1(v1), vi2(v2), vi3(v3), vecList(vecList)
    {
        update();
    }

    /**
     * Calculate the coefficients of the plane from the vertices.
     */
    void update();

    const vec &v1() const;

    const vec &v2() const;

    const vec &v3() const;

    /**
     * Returns the normal of this triangle
     */
    const vec getNormal() const;

    /**
     * Returns the center Vector3f of this triangle
     */
    const vec getCenter() const;

    /**
     * Evaluates the plane equation of this triangle's plane
     * for the given Vector3f.
     * @param r The Vector3f at which we find the value of the plane equation.
     * @return The value of the plane equation at the given Vector3f.
     */
    double planeEquation(const vec &r) const;
};

/** 
 * Class that handles a 3D model.
 */
class VVRScene_API Mesh
{
public:
    Mesh();
    Mesh(const std::string &objFile, const std::string &texFile=std::string(), bool ccw = true);
    Mesh(const Mesh &original);
    void operator=(const Mesh &src);
    void exportToObj(const std::string &filename);

#ifdef VVR_USE_BOOST
    VAR_CLASS_DEFS(Mesh)
#endif

private:
    std::vector<vec>        mVertices;              ///< Vertex list
    std::vector<Triangle>   mTriangles;             ///< Triangle list | contains indices to the Vertex list
    std::vector<vec>        mVertexNormals;         ///< Normals per vertex
    math::float3x4          mTransform;             ///< Model rotation around its local axis
    math::AABB              mAABB;                  ///< The bounding box of the model
    bool                    mCCW;                   ///< Clockwise-ness

private:
    void updateTriangleData();                      ///< Recalculates the plane equations of the triangles
    void createNormals();                           ///< Create a normal for each vertex
    void drawTriangles(Colour col, bool wire = 0);  ///< Draw the triangles. This is the actual model drawing.
    void drawNormals(Colour col);                   ///< Draw the normals of each vertex
    void drawAxes();

public:
    void draw(Colour col, Style style);             ///< Draw the mesh with the specified style
    void move(const math::vec &p);                  ///< Move the mesh in the world.
    void rotate(const math::vec &p);                ///< Rotate mesh around its local axis
    void setBigSize(float size);                    ///< Set the meshes size according to the max size of three (x|y|z)
    void cornerAlign();                             ///< Align the mesh to the corner of each local axis
    void centerAlign();                             ///< Align the mesh to the center of each local axis
    void update(const bool recomputeAABB=false);    ///< Call after making changes to the vertices
    void setTransform(const math::float3x4 &transform) { mTransform = transform; }

    std::vector<vec> &getVertices() { return mVertices; }
    std::vector<Triangle> &getTriangles() { return mTriangles; }
    const std::vector<vec> &getVertices() const { return mVertices; }
    const std::vector<Triangle> &getTriangles() const { return mTriangles; }
    math::float3x4 getTransform() const { return mTransform; }
    math::AABB getAABB() const { return mAABB; }
    float getMaxSize() const;
};

}

#endif // VVR_MESH_H
