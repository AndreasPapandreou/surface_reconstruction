#ifndef LAB0_SURFACERECONSTRUCTION_H
#define LAB0_SURFACERECONSTRUCTION_H

#include <iostream>
#include <stdlib.h>
#include <VVRScene/mesh.h>
#include <VVRScene/utils.h>
#include "PointCloud.h"
#include "ImageRGB.h"

using namespace std;
using namespace vvr;

typedef std::vector<float4> VecArray4;

class surfaceReconstruction : public vvr::Scene {
    public:
        surfaceReconstruction(int &index);

    private:
        void initialize(int &index);
        void draw() override;
        void reset() override;
        void resize() override;
        void createGui();
        void showFrames(int index);
        void getPointCLoud(vector< pair <vec,Vec3b>> &point_cloud, int &index);

        static void change_frame(int x, void*);
        static void drawFrame(int index, void* object);
        static void drawLeftFrame(int x, void*);
        static void drawRightFrame(int x, void*);
        static void alignFrames(int x, void*);
        static void alignFramesKnn(int x, void*);
        static void alignFramesKnnSobel(int x, void*);
        void drawAdjacentPoints();

        Mat getFrame(int index);
        void getDepthImage(int frame_index);
        void getImage(int frame_index);
        VecArray getFirstData(vector< pair <vec,Vec3b>> &paired_data);
        VecArray getData(VecArray points, int num);
        vector<int> removePoints(VecArray & l_points, VecArray &r_points, float threshold);
//        vector<int> removePoints(VecArray4 & l_points, VecArray4 &r_points, float threshold);
        float vectorSum(const vector<float> &v);
        void normalize(vector<float> &values);
//        void normalize(VecArray4 &values);
        float min(const vector<float> &values);
//        vec min(const VecArray4 &values);
        float max(const vector<float> &values);
//        vec max(const VecArray4 &values);
//        void RGBtoHSV(const Vec3b color, float& fH, float& fS, float& fV);
//        void getColorPoints(const vector< pair <vec,Vec3b>> &p1, VecArray4 &p2);

    private:
        Mat depth_mat, image_mat, r_frame, l_frame;
        vector< pair <vec,Vec3b>> l_points, r_points, all_points;
        vvr::Colour m_obj_col;
        math::Plane m_plane;
        PointCloud pcloud;
//        Mesh m_mesh;

        int l_frame_index, r_frame_index;
        int num_images;
        int slider_value{1};
        float m_plane_d;
        bool m_flag;

        string image_prefix;
};

#endif //LAB0_SURFACERECONSTRUCTION_H