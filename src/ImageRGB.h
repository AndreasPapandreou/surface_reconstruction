#ifndef LAB0_IMAGERGB_H
#define LAB0_IMAGERGB_H

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

class ImageRGB {
    private:
        static int m_idGenerator;
        const string m_path;
        Mat m_image_mat;

    protected:
        int m_id;

    public:
        explicit ImageRGB(string path) : m_path(path){ m_id = m_idGenerator++; }
        void convertToMat();
        void getMat(Mat &image);
        int getId();

    friend class PointCloud;
};

#endif //LAB0_IMAGERGB_H