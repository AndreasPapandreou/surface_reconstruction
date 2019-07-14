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

        static void changeFrame(int x, void*);
        static void setNumOfFrames(int x, void*);
        static void setThreshold(int x, void*);
        static void alignTwoFrames(int index, void* object);
        static void drawFrame(int index, void* object);
        static void drawLeftFrame(int x, void*);
        static void drawRightFrame(int x, void*);
        static void alignFramesKnn(int x, void*);
        static void alignFramesKnnSobel(int x, void*);
        static void segmentation(int x, void*);
        static void setLeftFrame(int x, void*);
        static void setRightFrame(int x, void*);

        void getDepthImage(int frame_index);
        void getImage(int frame_index);
        VecArray getFirstData(vector< pair <vec,Vec3b>> &paired_data);
        Mat getFrame(int index);

private:
        Mat depth_mat, image_mat, r_frame, l_frame;
        VecArray l_points, r_points;
        VecArray m_normals;
        vector<PointFeatures> pointFeatures;
        vector<vector<int>> segments;
        vector<vector<VecArray>> final_segments;
        vector< pair <vec,Vec3b>> all_points;
        vector<float> m_curvature;
        vvr::Colour m_obj_col;
        PointCloud pcloud;
        math::Plane m_plane;
        float m_plane_d;
        vector<math::Plane> planes;
        Mesh* mesh = nullptr;

        bool lFrame{false}, rFrame{false};
        /// boolean variables for drawing
        bool draw_frame{false}, draw_lines{false}, draw_segmentation{false};

        int l_frame_index, r_frame_index;
        int num_images, num_frames{2};
        int slider_value{1}, slider_value2{2},slider_value3{10};
        int threshold{50};

        string image_prefix;
};

#endif //LAB0_SURFACERECONSTRUCTION_H