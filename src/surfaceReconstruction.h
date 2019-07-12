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

//typedef std::vector<float4> VecArray4;

class surfaceReconstruction : public vvr::Scene {
    public:
        surfaceReconstruction(int &index);
        ~surfaceReconstruction() {delete mesh;}

    private:
        void initialize(int &index);
        void draw() override;
        void reset() override;
        void resize() override;
        void createGui();
        void showFrames(int index);
        void getPointCLoud(vector< pair <vec,Vec3b>> &point_cloud, int &index);

        static void test(int x, void*);
        static void change_frame(int x, void*);
        static void drawFrame(int index, void* object);
        static void drawLeftFrame(int x, void*);
        static void drawRightFrame(int x, void*);
        static void alignFrames(int x, void*);
        static void alignFramesKnn(int x, void*);
        static void alignFramesKnnSobel(int x, void*);
//        static void alignFramesKnnSobelNormals(int x, void*);
        void drawAdjacentPoints();

        Mat getFrame(int index);
        void getDepthImage(int frame_index);
        void getImage(int frame_index);
        VecArray getFirstData(vector< pair <vec,Vec3b>> &paired_data);
        VecArray getData(VecArray points, int num);
        vector<int> removePoints(VecArray & l_points, VecArray &r_points, float threshold);
        float vectorSum(const vector<float> &v);

        template <typename T>
        T min(const vector<T> &values);
        template <typename T>
        T max(const vector<T> &values);
        template <typename T>
        void normalize(vector<T> &values);

private:
        Mat depth_mat, image_mat, r_frame, l_frame;
        VecArray m_normals, m_vertices;
        vector<PointFeatures> pointFeatures;
//        vector<vector<PointFeatures>> segments, test_segments;
        vector<vector<int>> segments;
        vector<vector<VecArray>> test_segments;
        Mesh* mesh = nullptr;
        vector< pair <vec,Vec3b>> l_points, r_points, all_points;
        vvr::Colour m_obj_col;
        PointCloud pcloud;
        vec tangent_vec_x, tangent_vec_y;
        vector<int> *tri_indices;
        vector<float> m_curvature;
        Point3D testPoint;
        int used;
        math::Plane m_plane;
        float m_plane_d;
        vector<math::Plane> planes;

        int l_frame_index, r_frame_index;
        int num_images;
        int slider_value{1};
        bool m_flag;

        string image_prefix;
};

#endif //LAB0_SURFACERECONSTRUCTION_H