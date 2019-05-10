#ifndef LAB0_IMAGERGBD_H
#define LAB0_IMAGERGBD_H

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

class ImageRGBD {
    private:
        static int m_idGenerator;
        const string m_path;
        Mat m_depth_map;

    protected:
        int m_id;

    public:
        explicit ImageRGBD(string path) : m_path(path){ m_id = m_idGenerator++; }
        void convertToMat();
        void getMat(Mat &image);
        int getId();

    friend class PointCloud;
};

#endif //LAB0_IMAGERGBD_H