#ifndef LAB0_SURFACERECONSTRUCTION_H
#define LAB0_SURFACERECONSTRUCTION_H

#include "opencv2/opencv.hpp"
#include <VVRScene/canvas.h>
#include <VVRScene/mesh.h>
#include <VVRScene/settings.h>
#include <VVRScene/utils.h>
#include <MathGeoLib.h>
#include "ImageRGB.h"
#include "ImageRGBD.h"
#include "PointCloud.h"

#define FLAG_SHOW_AXES       1
#define FLAG_SHOW_WIRE       2
#define FLAG_SHOW_SOLID      4
#define FLAG_SHOW_NORMALS    8
#define FLAG_SHOW_PLANE     16
#define FLAG_SHOW_AABB      32

using namespace std;
using namespace vvr;
using namespace cv;

class surfaceReconstruction : public vvr::Scene {

public:
    surfaceReconstruction();
    const char* getName() const { return "Surface Reconstruction"; }
//    void keyEvent(unsigned char key, bool up, int modif) override;
//    void arrowEvent(vvr::ArrowDir dir, int modif) override;

private:
    void run();
//    void matToVector(Mat &matrix_image, vector<float> &vector_image);
    void createMesh(const vector< pair <Point3d,Vec3b>> &image_points, Mesh &mesh);
    void draw() override;
    void drawMesh(const vector<Point3d> &points);
    void reset() override;
    void resize() override;
//    void rotateX3D(PointCloud &pcloud, float angle);
//    void rotateX3D(Mesh &mesh, float angle);
//    void rotateY3D(PointCloud &pcloud, float angle);
//    void rotateY3D(Mesh &mesh, float angle);
//    void rotateZ3D(PointCloud &pcloud, float angle);


private:
    int m_style_flag;
    float m_plane_d;
    vvr::Colour m_obj_col;
    vvr::Mesh m_model_original, m_model;
    math::vec m_center_mass;
    math::Plane m_plane;
    PointCloud* m_pClouds;
    Mesh m_mesh;
    vector< pair <Point3d,Vec3b>> m_points;
};

#endif //LAB0_SURFACERECONSTRUCTION_H