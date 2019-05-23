#include "PointCloud.h"
#include "ImageRGB.h"
#include <random>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

/* ------------------------------------------------------------------------------
 * Inputs       : An image in Mat and its depth one version in Mat
 * Description  : It creates the point cloud of each image. and stores all points
 *                and their colours in vector< pair <Point3d,Vec3b>>.
 * Return       : -
 * ------------------------------------------------------------------------------
*/
void PointCloud::create(const Mat &image, const Mat &depth_image)
{
    CameraConstants camera;
    Mat xgrid, ygrid;
    xgrid.create(camera.image_height,camera.image_width, CV_16SC1);
    ygrid.create(camera.image_height,camera.image_width, CV_16SC1);

    for (unsigned short i=0; i<camera.image_height; i++) {
        for (unsigned short j=0; j<camera.image_width; j++) {
            xgrid.at<short>(i,j) = static_cast<short>(j + 1 + (camera.topLeft[0] - 1) - camera.center[0]);
            ygrid.at<short>(i,j) = static_cast<short>(i + 1 + (camera.topLeft[1] - 1) - camera.center[1]);
        }
    }

    for (unsigned short i=0; i<camera.image_height; i++) {
        for (unsigned short j=0; j<camera.image_width; j++) {
            Point3d point;
            point.x = xgrid.at<short>(i,j)*depth_image.at<unsigned short>(i,j)/camera.constant/camera.mm_per_m;
            point.y = ygrid.at<short>(i,j)*depth_image.at<unsigned short>(i,j)/camera.constant/camera.mm_per_m;
            point.z = depth_image.at<unsigned short>(i,j)/camera.mm_per_m;
//            if (point.x != 0 && point.y != 0 && point.z != 0) {
//            if (point.z != 0) {
                m_points.emplace_back(point, image.at<Vec3b>(i,j));
//            }
        }
    }
}

pair<Point3d,Vec3b> PointCloud::convertTo3d(const Mat &image, const Mat &depth_image, Point2d &point)
{
    CameraConstants camera;
    short xgrid, ygrid;
    xgrid = static_cast<short>(point.x + 1 + (camera.topLeft[0] - 1) - camera.center[0]);
    ygrid = static_cast<short>(point.y + 1 + (camera.topLeft[1] - 1) - camera.center[1]);

    pair <Point3d,Vec3b> res;
    res.first.x = xgrid*depth_image.at<unsigned short>(point.y,point.x)/camera.constant/camera.mm_per_m;
    res.first.y = ygrid*depth_image.at<unsigned short>(point.y,point.x)/camera.constant/camera.mm_per_m;
    res.first.z = depth_image.at<unsigned short>(point.y,point.x)/camera.mm_per_m;
    res.second = image.at<Vec3b>(point.y,point.x);
    return res;
}

/* ------------------------------------------------------------------------------
 * Inputs       : One vector< pair <Point3d,Vec3b>>
 * Description  : Gets the instance's point cloud.
 * Return       : -
 * ------------------------------------------------------------------------------
*/
void PointCloud::getPoints(vector< pair <Point3d,Vec3b>> &points)
{
    points = m_points;
}

void PointCloud::clearPoints()
{
    m_points.clear();
}

/* ------------------------------------------------------------------------------
 * Inputs       : One depth image
 * Description  : Gets the id of instances' depth image.
 * Return       : The id of current depth image.
 * ------------------------------------------------------------------------------
*/
int PointCloud::getImageId(const ImageRGBD &image)
{
    return image.m_id;
}

/* ------------------------------------------------------------------------------
 * Inputs       : Vec3d with degree per axis in order x-y-z
 * Description  : Calculates rotation matrix given euler angles per axis.
 * Return       : The rotation matrix
 * ------------------------------------------------------------------------------
*/
Mat PointCloud::rotationMatrix(const Vec3d &degree)
{
    Vec3d radian;
    radian[0] = degree[0]*M_PI/180.; // x_axis
    radian[1] = degree[1]*M_PI/180.; // y_axis
    radian[2] = degree[2]*M_PI/180.; // z_axis

    // calculate rotation about x axis
    Mat rotation_x = (Mat_<double>(3,3) <<
            1, 0, 0,
            0, cos(radian[0]), -sin(radian[0]),
            0, sin(radian[0]), cos(radian[0]));

    // calculate rotation about y axis
    Mat rotation_y = (Mat_<double>(3,3) <<
            cos(radian[1]), 0, sin(radian[1]),
            0, 1, 0,
            -sin(radian[1]), 0, cos(radian[1]));

    // calculate rotation about z axis
    Mat rotation_z = (Mat_<double>(3,3) <<
            cos(radian[2]), -sin(radian[2]), 0,
            sin(radian[2]), cos(radian[2]), 0,
            0, 0, 1);

    // get the final rotation matrix
    Mat rotation = rotation_z * rotation_y * rotation_x;
    return rotation;

}

/* ------------------------------------------------------------------------------
 * Inputs       : The rotation matrix
 * Description  : Rotate point cloud given the rotation matrix
 * Return       : -
 * ------------------------------------------------------------------------------
*/
void PointCloud::rotate(const Mat &rotation_mat) {
    for (auto &point : m_points) {
        Mat current_point = (Mat_<double>(3,1) << point.first.x, point.first.y, point.first.z);
        Mat result = rotation_mat*current_point;
        point.first.x = result.at<double>(0);
        point.first.y = result.at<double>(1);
        point.first.z = result.at<double>(2);
    }
}

void PointCloud::findCorrespondingPoints(const Mat &l_frame_rgb, const Mat &r_frame_rgb, const Mat &l_frame_rgbd, const Mat &r_frame_rgbd, vector< pair <Point3d,Vec3b>> &l_points, vector< pair <Point3d,Vec3b>> &r_points) {
    CameraConstants camera;

    // the parameter slide show how many pixels to slide the window in each direction
    int window_size{70}, slide{180};
    int top_row, top_col, width, height;

    int num_points{10};
    int searching_points[num_points];
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 eng(rd()); // seed the generator
    int left_range{20*camera.image_height/100};
    int right_range{80*camera.image_height/100};
    std::uniform_int_distribution<> distr(left_range, right_range); // define the range
    for(int n=0; n<num_points; ++n)
        searching_points[n] = distr(eng);

    for (int row=0; row<num_points; row++) {
        for (int col=0; col<num_points; col++) {
             cv::Rect img_roi(searching_points[col], searching_points[row], window_size, window_size);
             Mat img_template = l_frame_rgb(img_roi);

             top_col = searching_points[col]-slide;
             top_row = searching_points[row]-slide;
             width = 2*slide;
             height = 2*slide;

             validate(top_col, top_row, width, height);

             cv::Rect cube_roi(top_col, top_row, width, height);
             Mat img_cube = r_frame_rgb(cube_roi);

             Mat result;
             matchTemplate(img_cube, img_template, result, 5);

             cv::Point best_match;
             minMaxLoc(result, nullptr, nullptr, nullptr, &best_match);

             Point2d left_point(searching_points[col], searching_points[row]);
             l_points.emplace_back(convertTo3d(l_frame_rgb, l_frame_rgbd, left_point));

             Point2d right_point = Point2d(best_match.x+top_col, best_match.y+top_row);
             r_points.emplace_back(convertTo3d(r_frame_rgb, r_frame_rgbd, right_point));

//             namedWindow("test_left", WINDOW_NORMAL);
//             namedWindow("test_right", WINDOW_NORMAL);
//             circle(l_frame_rgb, left_point, 1, Scalar(0, 0, 255), FILLED, LINE_8);
//             circle(r_frame_rgb, right_point, 1, Scalar(0, 0, 255), FILLED, LINE_8);
//             imshow("test_left", l_frame_rgb);
//             imshow("test_right", r_frame_rgb);
        }
    }
}

//void PointCloud::findAdjacentPoints(const Mat &l_frame_rgb, const Mat &r_frame_rgb, const Mat &l_frame_rgbd, const Mat &r_frame_rgbd, vector< pair <Point3d,Vec3b>> &l_points, vector< pair <Point3d,Vec3b>> &r_points) {
//    // the parameter slide show how many pixels to slide the window in each direction
////    int window_size{50}, slide{150};
//    int window_size{30}, slide{100};
//    int top_row, top_col, width, height;
//    bool update{false};
//    int left{0}, right{0}, up{0}, down{0}; //data for the new point 0->left, 1->right, 2->up, 3->down
//
//    for (int row=0; row<r_frame_rgb.rows - window_size; row++) {
//        cout << "row = " << row << endl;
//        for (int col=0; col<r_frame_rgb.cols - window_size; col++) {
////    for (int row=0; row<200; row++) {
////        cout << "row = " << row << endl;
////        for (int col=0; col<10; col++) {
////            cout << "col = " << col << endl;
//             cv::Rect img_roi(col, row, window_size, window_size);
//             Mat img_template = l_frame_rgb(img_roi);
//
//             if (update) {
//                 if (left > right) {
//                    top_col = col - slide;
//                 }
//                 else {
//                    top_col = col;
//                 }
//                 if (up > down) {
//                     top_row = row - slide;
//                 }
//                 else {
//                     top_row = row;
//                 }
//                 width = slide;
//                 height = slide;
//
////                 cout << "btop_col =" << top_col << endl;
////                 cout << "btop_row =" << top_row << endl;
////                 cout << "bwidth =" << width << endl;
////                 cout << "bheight =" << height << endl << endl;
//             }
//             else {
//                 top_col = col - slide;
//                 top_row = row - slide;
//                 width = 2 * slide;
//                 height = 2 * slide;
//             }
//
//             validate(top_col, top_row, width, height, slide);
//
////            cout << "atop_col =" << top_col << endl;
////            cout << "atop_row =" << top_row << endl;
////            cout << "awidth =" << width << endl;
////            cout << "aheight =" << height << endl;
////            cout << "col =" << col << endl << endl;
//
//             cv::Rect cube_roi(top_col, top_row, width, height);
//             Mat img_cube = r_frame_rgb(cube_roi);
//
//             Mat result;
//             matchTemplate(img_cube, img_template, result, 5);
//
//             cv::Point best_match;
//             minMaxLoc(result, nullptr, nullptr, nullptr, &best_match);
//
//             Point2d left_point(col, row);
//             l_points.emplace_back(convertTo3d(l_frame_rgb, l_frame_rgbd, left_point));
//
//             Point2d right_point =  Point2d(best_match.x+top_col, best_match.y+top_row);
//             r_points.emplace_back(convertTo3d(r_frame_rgb, r_frame_rgbd, right_point));
//
//
////            if(row == 0 && col == 49) {
//            if(row == 1 && col == 0) {
//                update = true;
//            }
//            else {
//                if (right_point.x <= left_point.x)
//                    left++;
//                else
//                    right++;
//                if (right_point.y <= left_point.y)
//                    up++;
//                else
//                    down++;
//            }
//        }
//    }
//}

//void PointCloud::findAdjacentPoints(const Mat &l_frame, const Mat &r_frame) {
//    int flag{1};
//    // how many pixels to slide the window in each direction
//    int window_size{50}, slide{150};
//    int top_row, top_col, width, height;
//
////    cout << "last = " << (r_frame.cols - window_size)*(r_frame.rows - window_size) << endl; = 253700
//    for (int row=0; row<r_frame.rows - window_size; row++) {
//        for (int col=0; col<r_frame.cols - window_size; col++) {
//
//            if (flag++ == 53701) {
//                cv::Rect img_roi(col, row, window_size, window_size);
//                Mat img_template = l_frame(img_roi);
//
//                top_col = col-slide;
//                top_row = row-slide;
//                width = 2*slide;
//                height = 2*slide;
//
//                validate(top_col, top_row, width, height);
//
//                cv::Rect cube_roi(top_col, top_row, width, height);
//                Mat img_cube = r_frame(cube_roi);
//
//                Mat result;
//                matchTemplate(img_cube, img_template, result, 5);
//
//                cv::Point best_match;
//                minMaxLoc(result, nullptr, nullptr, nullptr, &best_match);
//
//                namedWindow("img_template", WINDOW_NORMAL);
//                imshow("img_template", img_template);
//                namedWindow("img_cube", WINDOW_NORMAL);
//                imshow("img_cube", img_cube);
//                cout << "template size = " << img_template.size << endl;
//                cout << "img_cube size = " << img_cube.size << endl;
//
//                Point pt_old =  Point(col, row);
//                Point pt_new =  Point(best_match.x+top_col, best_match.y+top_row);
////                Point pt_new =  Point(best_match.x, best_match.y);
//
//                Mat my_l_depth_mat, my_r_depth_mat;
//                ImageRGB depth_image("../data/meeting_small_1/meeting_small_1_28.png");
//                depth_image.convertToMat();
//                depth_image.getMat(my_l_depth_mat);
//                ImageRGB depth_image2("../data/meeting_small_1/meeting_small_1_29.png");
//                depth_image2.convertToMat();
//                depth_image2.getMat(my_r_depth_mat);
//
//                namedWindow("test_left", WINDOW_NORMAL);
//                namedWindow("test_right", WINDOW_NORMAL);
//
//                circle( my_l_depth_mat, pt_old, 1, Scalar( 0, 0, 255 ), FILLED, LINE_8 );
//                circle( my_r_depth_mat, pt_new, 1, Scalar( 0, 0, 255 ), FILLED, LINE_8 );
//
//                imshow("test_left", my_l_depth_mat);
//                imshow("test_right", my_r_depth_mat);
//
//                cout << "old x = " << col << endl;
//                cout << "old y = " << row << endl;
//                cout << "new x = " << best_match.x << endl;
//                cout << "new y = " << best_match.y << endl;
//            }
//        }
//    }
//}

//void PointCloud::validate(int &top_col, int &top_row, int &width, int &height, int &slide) {
//    if (top_col < 0) {
//        width += top_col;
//        top_col = 0;
//    }
//
//    if (top_row < 0) {
//        height += top_row;
//        top_row = 0;
//    }
//
//    if ((width + top_col) > 640)
//        width = 640 - top_col;
//
//    if ((height + top_row) > 480)
//        height = 480 - top_row;
//
//    // check it
////    if (width == 0)
////        width = slide;
////    if (height == 0)
////        height = slide;
//}

void PointCloud::validate(int &top_col, int &top_row, int &width, int &height) {
    CameraConstants camera;

    if (top_col < 0) {
        width += top_col;
        top_col = 0;
    }

    if (top_row < 0) {
        height += top_row;
        top_row = 0;
    }

    if ((width + top_col) > camera.image_width)
        width = camera.image_width - top_col;

    if ((height + top_row) > camera.image_height)
        height = camera.image_height - top_row;
}

/*
//    double padding = (double)window_size/(2.0*100.0);
void PointCloud::zeroPad(const Mat &image, const double size, Mat &new_image) {

    // We give them a value of size% the size of src.
    int top = (int) (size*image.rows);
    int bottom = (int) (size*image.rows);
    int left = (int) (size*image.cols);
    int right = (int) (size*image.cols);

    copyMakeBorder(image, new_image, top, bottom, left, right, BORDER_CONSTANT, 0);

//    namedWindow("before padding", WINDOW_NORMAL);
//    imshow("before padding", image);
//    cout << "before size = " << image.size << endl;
}
*/

pair<Eigen::Matrix3d, Eigen::Vector3d> PointCloud::computeRigidTransform(const vector<Point3d> &src, const vector<Point3d> &dst) {
    Point3d src_center, dst_center;
    int size = static_cast<int>(src.size());

    // compute the centroids of both point sets
    for (int i=0; i<size; ++i)
    {
        src_center += src.at(i);
        dst_center += dst.at(i);
    }
    src_center/=size;
    dst_center/=size;

    vector<Point3d> src_centered_points, dst_centered_points;
    // compute the centered vectors
    for (int i=0; i<size; i++) {
        src_centered_points.emplace_back(src.at(i) - src_center);
        dst_centered_points.emplace_back(dst.at(i) - dst_center);
    }

    Eigen::MatrixXd X(3,size), Y(3,size);
    convertToEigenMat(dst_centered_points, src_centered_points, X, Y);

    // create diagonal matrix
    Eigen::MatrixXd W(size, size);
    for (int i=0; i<size; i++) {
        for (int j=0; j<size; j++) {
            if (i==j)
                W(i,j) = 1;
            else
                W(i,j) = 0;
        }
    }

    // compute the covariance matrix
    Eigen::Matrix3d S = X*W*Y.transpose();

    // compute the singular value decomposition
    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    svd.compute(S, Eigen::ComputeThinU | Eigen::ComputeThinV );
    if (!svd.computeU() || !svd.computeV()) {
        std::cerr << "decomposition error" << endl;
    }

    // extract right singular vectors
    Eigen::Matrix3d V = svd.matrixV();

    // extract left singular vectors
    Eigen::Matrix3d U = svd.matrixU();

    // create diagonal matrix
    Eigen::MatrixXd diag_mat(3, 3);
    diag_mat.setZero();
    diag_mat(0,0) = 1;
    diag_mat(1,1) = 1;
    diag_mat(2,2) = V.determinant()*U.transpose().determinant();

    // compute rotation matrix
    Eigen::Matrix3d R = V*diag_mat*U.transpose();

    // compute translation vector
    Eigen::Vector3d t = convertToEigenVector3d(src_center) - R*convertToEigenVector3d(dst_center);
    return pair<Eigen::Matrix3d, Eigen::Vector3d>(R, t);
}

void PointCloud::convertToEigenMat(const vector<Point3d> &l_points, const vector<Point3d> &r_points, Eigen::MatrixXd &l_mat, Eigen::MatrixXd &r_mat) {
    int size = static_cast<int>(l_points.size());
    for (int i=0; i<size; i++) {
        l_mat(0,i) = l_points.at(i).x;
        l_mat(1,i) = l_points.at(i).y;
        l_mat(2,i) = l_points.at(i).z;

        r_mat(0,i) = r_points.at(i).x;
        r_mat(1,i) = r_points.at(i).y;
        r_mat(2,i) = r_points.at(i).z;
    }
}

void PointCloud::convertToEigenMat(const vector<Point3d> points, Eigen::MatrixXd &mat) {
    int size = static_cast<int>(points.size());
    for (int i=0; i<size; i++) {
        mat(i,0) = points.at(i).x;
        mat(i,1) = points.at(i).y;
        mat(i,2) = points.at(i).z;
    }
}

void PointCloud::convertToVector(const Eigen::MatrixXd &mat, vector<Point3d> &points) {
    int size = static_cast<int>(points.size());
    for (int i=0; i<size; i++) {
        points.at(i).x = mat(i,0);
        points.at(i).y = mat(i,1);
        points.at(i).z = mat(i,2);
    }
}

Eigen::Vector3d PointCloud::convertToEigenVector3d(const Point3d &point) {
    Eigen::Vector3d res;
    res(0,0) = point.x;
    res(1,0) = point.y;
    res(2,0) = point.z;
    return res;
}

void PointCloud::tranformPoints(pair<Eigen::Matrix3d, Eigen::Vector3d> &R_t, vector<Point3d> &points) {
    int size = static_cast<int>(points.size());
    Eigen::MatrixXd mat(size,3);
    convertToEigenMat(points, mat);

    Eigen::MatrixXd new_points(size,3);
    new_points = mat*R_t.first;
    for (int i=0; i<size; i++) {
        new_points(i,0) += R_t.second(0,0);
        new_points(i,1) += R_t.second(1,0);
        new_points(i,2) += R_t.second(2,0);
    }
    convertToVector(new_points, points);
}





