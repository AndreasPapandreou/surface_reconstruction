#include "PointCloud.h"
#include "dataTypes.h"

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
            vec point;
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

//pair<Point3d,Vec3b> PointCloud::convertTo3d(const Mat &image, const Mat &depth_image, Point2d &point)
//{
//    CameraConstants camera;
//    short xgrid, ygrid;
//    xgrid = static_cast<short>(point.x + 1 + (camera.topLeft[0] - 1) - camera.center[0]);
//    ygrid = static_cast<short>(point.y + 1 + (camera.topLeft[1] - 1) - camera.center[1]);
//
//    pair <Point3d,Vec3b> res;
//    res.first.x = xgrid*depth_image.at<unsigned short>(point.y,point.x)/camera.constant/camera.mm_per_m;
//    res.first.y = ygrid*depth_image.at<unsigned short>(point.y,point.x)/camera.constant/camera.mm_per_m;
//    res.first.z = depth_image.at<unsigned short>(point.y,point.x)/camera.mm_per_m;
//    res.second = image.at<Vec3b>(point.y,point.x);
//    return res;
//}

/* ------------------------------------------------------------------------------
 * Inputs       : One vector< pair <Point3d,Vec3b>>
 * Description  : Gets the instance's point cloud.
 * Return       : -
 * ------------------------------------------------------------------------------
*/
void PointCloud::getPoints(vector< pair <vec,Vec3b>> &points)
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
int PointCloud::getRgbdId(const ImageRGBD &image)
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
    Mat rotation_x = (Mat_<float>(3,3) <<
            1, 0, 0,
            0, cos(radian[0]), -sin(radian[0]),
            0, sin(radian[0]), cos(radian[0]));

    // calculate rotation about y axis
    Mat rotation_y = (Mat_<float>(3,3) <<
            cos(radian[1]), 0, sin(radian[1]),
            0, 1, 0,
            -sin(radian[1]), 0, cos(radian[1]));

    // calculate rotation about z axis
    Mat rotation_z = (Mat_<float>(3,3) <<
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
        Mat current_point = (Mat_<float>(3,1) << point.first.x, point.first.y, point.first.z);
        Mat result = rotation_mat*current_point;
        point.first.x = result.at<float>(0);
        point.first.y = result.at<float>(1);
        point.first.z = result.at<float>(2);
    }
}

//void PointCloud::findCorrespondingPoints(const Mat &l_frame_rgb, const Mat &r_frame_rgb, const Mat &l_frame_rgbd, const Mat &r_frame_rgbd, vector< pair <Point3d,Vec3b>> &l_points, vector< pair <Point3d,Vec3b>> &r_points) {
//    CameraConstants camera;
//
//    // the parameter slide show how many pixels to slide the window in each direction
//    int window_size{70}, slide{180};
//    int top_row, top_col, width, height;
//
//    int num_points{10};
//    int searching_points[num_points];
//    std::random_device rd; // obtain a random number from hardware
//    std::mt19937 eng(rd()); // seed the generator
//    int left_range{20*camera.image_height/100};
//    int right_range{80*camera.image_height/100};
//    std::uniform_int_distribution<> distr(left_range, right_range); // define the range
//    for(int n=0; n<num_points; ++n)
//        searching_points[n] = distr(eng);
//
//    for (int row=0; row<num_points; row++) {
//        for (int col=0; col<num_points; col++) {
//             cv::Rect img_roi(searching_points[col], searching_points[row], window_size, window_size);
//             Mat img_template = l_frame_rgb(img_roi);
//
//             top_col = searching_points[col]-slide;
//             top_row = searching_points[row]-slide;
//             width = 2*slide;
//             height = 2*slide;
//
//             validate(top_col, top_row, width, height);
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
//             Point2d left_point(searching_points[col], searching_points[row]);
//             l_points.emplace_back(convertTo3d(l_frame_rgb, l_frame_rgbd, left_point));
//
//             Point2d right_point = Point2d(best_match.x+top_col, best_match.y+top_row);
//             r_points.emplace_back(convertTo3d(r_frame_rgb, r_frame_rgbd, right_point));
//
////             namedWindow("test_left", WINDOW_NORMAL);
////             namedWindow("test_right", WINDOW_NORMAL);
////             circle(l_frame_rgb, left_point, 1, Scalar(0, 0, 255), FILLED, LINE_8);
////             circle(r_frame_rgb, right_point, 1, Scalar(0, 0, 255), FILLED, LINE_8);
////             imshow("test_left", l_frame_rgb);
////             imshow("test_right", r_frame_rgb);
//        }
//    }
//}

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

// not considering the colors
void PointCloud::kNearest(const VecArray &src, VecArray &nearestPoints,  vector<float> &dist, int kn) {
    float distance;
    const float t = vvr::getSeconds();

    for (auto src_pt : src) {
        for (int j=0; j<kn; j++) {
            const auto **nearests = new const KDNode*[kn];
            memset(nearests, NULL, kn * sizeof(KDNode*));

            m_dst_KDTree->kNearest(j, src_pt, m_dst_KDTree->root(), nearests, &distance);
            nearestPoints.emplace_back((*nearests)->split_point);
            dist.emplace_back(distance);
        }
    }

    const float KDTree_knn_time = vvr::getSeconds() - t;
//    echo(KDTree_knn_time);
}

// considering the colors
//void PointCloud::kNearest(const VecArray4 &src, VecArray4 &nearestPoints,  vector<float> &dist, int kn, float &weight) {
//    float distance;
//    const float t = vvr::getSeconds();
//
//    for (auto src_pt : src) {
//        for (int j=0; j<kn; j++) {
//            const auto **nearests = new const KDNode2*[kn];
//            memset(nearests, NULL, kn * sizeof(KDNode2*));
//
//            m_dst_KDTree->kNearest(j, src_pt, m_dst_KDTree->root(), nearests, &distance, weight);
//            nearestPoints.emplace_back((*nearests)->split_point);
//            dist.emplace_back(distance);
//        }
//    }
//
//    const float KDTree_knn_time = vvr::getSeconds() - t;
//    echo(KDTree_knn_time);
//}

//void PointCloud::validate(int &top_col, int &top_row, int &width, int &height) {
//    CameraConstants camera;
//
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
//    if ((width + top_col) > camera.image_width)
//        width = camera.image_width - top_col;
//
//    if ((height + top_row) > camera.image_height)
//        height = camera.image_height - top_row;
//}

pair<Eigen::Matrix3f, Eigen::Vector3f> PointCloud::computeRigidTransform(const VecArray &src, const VecArray &dst) {
    int size = dst.size();
    vec src_center = getCentroid(src);
    vec dst_center = getCentroid(dst);

    vec src_centered_point, dst_centered_point;
    Eigen::MatrixXf X(3,size), Y(3,size);
    for (int i=0; i<size; i++) {
        src_centered_point = src.at(i) - src_center;
        dst_centered_point = dst.at(i) - dst_center;

        X(0,i) = src_centered_point.x/(double)size;
        X(1,i) = src_centered_point.y/(double)size;
        X(2,i) = src_centered_point.z/(double)size;

        Y(0,i) = dst_centered_point.x/(double)size;
        Y(1,i) = dst_centered_point.y/(double)size;
        Y(2,i) = dst_centered_point.z/(double)size;
    }

    // compute the covariance matrix
    Eigen::Matrix3f S = Y*X.transpose();

    // compute the singular value decomposition
    Eigen::JacobiSVD<Eigen::MatrixXf> svd;
    svd.compute(S, Eigen::ComputeThinU | Eigen::ComputeThinV );
    if (!svd.computeU() || !svd.computeV()) {
        std::cerr << "decomposition error" << endl;
    }

    // extract right singular vectors
    Eigen::Matrix3f V = svd.matrixV();

    // extract left singular vectors
    Eigen::Matrix3f U = svd.matrixU();

    // create diagonal matrix
    Eigen::MatrixXf diag_mat(3, 3);
    diag_mat.setZero();
    diag_mat(0,0) = 1;
    diag_mat(1,1) = 1;
    diag_mat(2,2) = V.determinant()*U.transpose().determinant();

    // compute rotation matrix
    Eigen::Matrix3f R = V*diag_mat*U.transpose();

    // compute translation vector
    Eigen::Vector3f t = dataTypes::convertToEigenVector(src_center) - R*dataTypes::convertToEigenVector(dst_center);

    return pair<Eigen::Matrix3f, Eigen::Vector3f>(R, t);
}

//pair<Eigen::Matrix3f, Eigen::Vector3f> PointCloud::computeRigidTransform(const VecArray4 &src4d, const VecArray4 &dst4d) {
//    VecArray src, dst;
//    vec p;
//    for (auto i : src4d) {
//        p.x = i.x; p.y = i.y; p.z = i.z;
//        src.emplace_back(p);
//    }
//    for (auto i : dst4d) {
//        p.x = i.x; p.y = i.y; p.z = i.z;
//        dst.emplace_back(p);
//    }
//
//    int size = dst.size();
//    vec src_center = getCentroid(src);
//    vec dst_center = getCentroid(dst);
//
//    vec src_centered_point, dst_centered_point;
//    Eigen::MatrixXf X(3,size), Y(3,size);
//    for (int i=0; i<size; i++) {
//        src_centered_point = src.at(i) - src_center;
//        dst_centered_point = dst.at(i) - dst_center;
//
//        X(0,i) = src_centered_point.x/(double)size;
//        X(1,i) = src_centered_point.y/(double)size;
//        X(2,i) = src_centered_point.z/(double)size;
//
//        Y(0,i) = dst_centered_point.x/(double)size;
//        Y(1,i) = dst_centered_point.y/(double)size;
//        Y(2,i) = dst_centered_point.z/(double)size;
//    }
//
//    // compute the covariance matrix
//    Eigen::Matrix3f S = Y*X.transpose();
//
//    // compute the singular value decomposition
//    Eigen::JacobiSVD<Eigen::MatrixXf> svd;
//    svd.compute(S, Eigen::ComputeThinU | Eigen::ComputeThinV );
//    if (!svd.computeU() || !svd.computeV()) {
//        std::cerr << "decomposition error" << endl;
//    }
//
//    // extract right singular vectors
//    Eigen::Matrix3f V = svd.matrixV();
//
//    // extract left singular vectors
//    Eigen::Matrix3f U = svd.matrixU();
//
//    // create diagonal matrix
//    Eigen::MatrixXf diag_mat(3, 3);
//    diag_mat.setZero();
//    diag_mat(0,0) = 1;
//    diag_mat(1,1) = 1;
//    diag_mat(2,2) = V.determinant()*U.transpose().determinant();
//
//    // compute rotation matrix
//    Eigen::Matrix3f R = V*diag_mat*U.transpose();
//
//    // compute translation vector
//    Eigen::Vector3f t = dataTypes::convertToEigenVector(src_center) - R*dataTypes::convertToEigenVector(dst_center);
//
//    return pair<Eigen::Matrix3f, Eigen::Vector3f>(R, t);
//}

vec PointCloud::getCentroid(const VecArray &points) {
    vec center(0,0,0);
    for (const auto &i : points) {
        center += i;
    }
    return center/(float)points.size();
}

void PointCloud::transformPoints(pair<Eigen::Matrix3f, Eigen::Vector3f> &R_t, VecArray &points) {
    int size = static_cast<int>(points.size());

    Eigen::MatrixXf mat(3,size);
    dataTypes::convertToEigenMat(points, mat);

    Eigen::MatrixXf new_points(3,size);
    new_points = R_t.first*mat;
    for (int i=0; i<size; i++) {
        new_points(0,i) += R_t.second(0,0);
        new_points(1,i) += R_t.second(1,0);
        new_points(2,i) += R_t.second(2,0);
    }
    dataTypes::convertToVector(new_points, points);
}

//void PointCloud::transformPoints(pair<Eigen::Matrix3f, Eigen::Vector3f> &R_t, VecArray4 &points) {
//    int size = static_cast<int>(points.size());
//
//    Eigen::MatrixXf mat(3,size);
//    dataTypes::convertToEigenMat(points, mat);
//
//    Eigen::MatrixXf new_points(3,size);
//    new_points = R_t.first*mat;
//    for (int i=0; i<size; i++) {
//        new_points(0,i) += R_t.second(0,0);
//        new_points(1,i) += R_t.second(1,0);
//        new_points(2,i) += R_t.second(2,0);
//    }
//    dataTypes::convertToVector(new_points, points);
//}

void PointCloud::sobel(const Mat &img, Mat &new_img) {
    Mat img_gray;
    std::string window_name = "Sobel";
    int scale = 1;
    int delta = 0;
    int ddepth = CV_8UC1;

    GaussianBlur( img, img, Size(3,3), 0, 0, BORDER_DEFAULT );

    /// Convert it to gray
    cvtColor( img, img_gray, CV_BGR2GRAY );

    /// Create window
    namedWindow( window_name, CV_WINDOW_AUTOSIZE );

    /// Generate grad_x and grad_y
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    Sobel( img_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    Sobel( img_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, new_img );

    imshow( window_name, new_img );
//    waitKey(0);
}

void PointCloud::laplacian(const Mat &img, Mat &new_img) {

    Mat img_gray;
    int kernel_size = 3;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_8UC1;
    std::string window_name = "Laplace";

    int c;

    /// Remove noise by blurring with a Gaussian filter
    GaussianBlur( img, img, Size(3,3), 0, 0, BORDER_DEFAULT );

    /// Convert the image to grayscale
    cvtColor( img, img_gray, CV_BGR2GRAY );

    /// Create window
    namedWindow( window_name, CV_WINDOW_AUTOSIZE );

    /// Apply Laplace function
    Mat abs_dst;

    Laplacian( img_gray, new_img, ddepth, kernel_size, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( new_img, abs_dst );

    /// Show what you got
    imshow( window_name, abs_dst );

//    waitKey(0);


}

void PointCloud::bilateral(const Mat &img, Mat &new_img) {
//    std::string window_name = "Bilateral";
//    namedWindow( window_name, CV_WINDOW_AUTOSIZE );
    bilateralFilter(img, new_img, 15, 80, 80);
//    imshow( window_name, new_img );
//    waitKey(0);
}

void PointCloud::cannyThreshold(const Mat &img, Mat &new_img)
{
    Mat img_gray, detected_edges;
    int edgeThresh = 1;
    int lowThreshold = 50;
    int const max_lowThreshold = 100;
    int ratio = 3;
    int kernel_size = 3;
    std::string window_name = "Edge Map";

    new_img.create( img.size(), img.type() );

    /// Convert the image to grayscale
    cvtColor( img, img_gray, CV_BGR2GRAY );

    /// Reduce noise with a kernel 3x3
    blur(img_gray, detected_edges, Size(3,3) );

    /// Canny detector
    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

    /// Using Canny's output as a mask, we display our result
    new_img = Scalar::all(0);

    img.copyTo( new_img, detected_edges);
    imshow( window_name, new_img );

//    waitKey(0);
}

// img must br gray
void PointCloud::thresholding(const Mat &img, Mat &new_img) {
//  variable that representing the value that is to be given if pixel value is more than the threshold value.
    double max_value = 255.0;
    std::string window_name = "THRESH_BINARY";
    namedWindow( window_name, CV_WINDOW_AUTOSIZE );
    adaptiveThreshold(img, new_img, max_value, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 11, 12);
    imshow( window_name, new_img );

//    char* window_name = "OTSU";
//    adaptiveThreshold(img,new_img,255,CV_ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,11,2);

//    double thres = threshold(img, new_img, 0.0, 255.0, THRESH_OTSU);
//    cout << "otsu value is = " << thres << endl;
//    threshold(img, new_img, thres, 255.0, THRESH_OTSU);
//    imshow( window_name, new_img );

//    waitKey(0);


}

void PointCloud::getPixels(const Mat &img, VecArray &edges, const int &value) {
    CameraConstants camera;
    uchar val;

    for (int i=0; i<camera.image_height; i++) {
        for (int j=0; j<camera.image_width; j++) {
            val = img.at<uchar>(i,j);
            if ((int)val == value) {
//            if ((int)val == 255) {
                edges.emplace_back(m_points.at(i*camera.image_width + j).first);
            }
        }
    }
}

pair<Eigen::Matrix3f, Eigen::Vector3f> PointCloud::icp(VecArray &src_points, vector<float> &dist, float &mean_distance, float &error, int &iterations) {
    vector<pair<Eigen::Matrix3f, Eigen::Vector3f>> all_R_t;
    VecArray nearestPoints;

    int counter{0};
    while(counter++ < iterations) {
        nearestPoints.clear();
        dist.clear();

        kNearest(src_points, nearestPoints, dist, 1);

        all_R_t.emplace_back(computeRigidTransform(nearestPoints, src_points));

        transformPoints(all_R_t.at(all_R_t.size()-1), src_points);

        mean_distance = vectorSum(dist)/(float)dist.size();
        normalize(dist);
        error = vectorSum(dist)/(float)dist.size();

        cout << "iter = " << counter << endl;
        cout << "mean_dist = " << mean_distance << endl;
        cout << "error = " << error << endl;
    }

    pair<Eigen::Matrix3f, Eigen::Vector3f> R_t;
    R_t = all_R_t.at(all_R_t.size()-1);
    for(int i=all_R_t.size()-2; i>=0; i--) {
        R_t.first *= all_R_t.at(i).first;
        R_t.second += all_R_t.at(i).second;
    }
    return R_t;
}

void PointCloud::normalize(vector<float> &values) {
    float min_value = min(values);
    float max_value = max(values);
    float diff = max_value - min_value;
    for (float &value : values) {
        value = (value - min_value)/diff;
    }
}

float PointCloud::vectorSum(const vector<float> &v) {
    float initial_sum{0.0f};
    return accumulate(v.begin(), v.end(), initial_sum);
}

float PointCloud::min(const vector<float> &values) {
    float min_value{0.0f};
    for (const auto &i : values) {
        if (i < min_value)
            min_value = i;
    }
    return min_value;
}

float PointCloud::max(const vector<float> &values) {
    float max_value{values.at(0)};
    for(int i=1; i<values.size(); i++) {
        if (i > max_value)
            max_value = i;
    }
    return max_value;
}

void PointCloud::computeNormals(const Mat &img, const Mat &depth_img, Mat &normals) {
//    std::string window_name = "original img";
//    namedWindow( window_name, CV_WINDOW_AUTOSIZE );
//    imshow(window_name, img);

    // 1 : smoothing using bilateral filter
    Mat filtered_img;
    bilateral(img, filtered_img);

//    std::string window_name2 = "bilateral img";
//    namedWindow( window_name2, CV_WINDOW_AUTOSIZE );
//    imshow(window_name2, filtered_img);

    // 2 : depth gradient computation
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16SC1;
    Mat img_gray;

    /// Convert it to gray
    cvtColor(filtered_img, img_gray, CV_BGR2GRAY);

    /// Generate grad_x and grad_y
    Mat der_z_x, der_z_y, sobel_img; // der_z_x = derivative of z to x

    /// Gradient X
    Sobel(img_gray, der_z_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);

    /// Gradient Y
    Sobel(img_gray, der_z_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
    addWeighted( der_z_x, 0.5, der_z_y, 0.5, 0, sobel_img );

//    std::string window_name3 = "Sobel img";
//    namedWindow( window_name3, CV_WINDOW_AUTOSIZE );
//    imshow(window_name3, sobel_img);
//
//    std::string window_name4 = "der_z_x";
//    namedWindow( window_name4, CV_WINDOW_AUTOSIZE );
//    imshow(window_name4, der_z_x);
//
//    std::string window_name5 = "der_z_y";
//    namedWindow( window_name5, CV_WINDOW_AUTOSIZE );
//    imshow(window_name5, der_z_y);

    CameraConstants camera;

    // 3 : normal estimation from depth gradients
    // parameterize a 3D point (X,Y,Z) as a function of a pixel (x,y)
    Mat xgrid, ygrid;
    xgrid.create(camera.image_height,camera.image_width, CV_16SC1);
    ygrid.create(camera.image_height,camera.image_width, CV_16SC1);

    for (int i=0; i<camera.image_height; i++) {
        for (int j=0; j<camera.image_width; j++) {
            xgrid.at<short>(i,j) = static_cast<short>(j + 1 + (camera.topLeft[0] - 1) - camera.center[0]);
            ygrid.at<short>(i,j) = static_cast<short>(i + 1 + (camera.topLeft[1] - 1) - camera.center[1]);
        }
    }

    Mat der_x_x, der_x_y, der_y_x, der_y_y, x, y, z;
    der_x_x.create(camera.image_height,camera.image_width, CV_16SC1);
    der_x_y.create(camera.image_height,camera.image_width, CV_16SC1);
    der_y_x.create(camera.image_height,camera.image_width, CV_16SC1);
    der_y_y.create(camera.image_height,camera.image_width, CV_16SC1);
    x.create(camera.image_height,camera.image_width, CV_16SC1);
    y.create(camera.image_height,camera.image_width, CV_16SC1);
    z.create(camera.image_height,camera.image_width, CV_16SC1);
//    res.create(camera.image_height,camera.image_width, CV_16SC3);

      ///    if x == i
    short c = short(camera.constant)/short(camera.mm_per_m);
    for (int i=0; i<camera.image_height; i++) {
        for (int j=0; j<camera.image_width; j++) {
            der_x_x.at<short>(i,j) = xgrid.at<short>(i, j)*der_z_x.at<short>(i,j)/c;
            der_x_y.at<short>(i,j) = (depth_img.at<short>(i,j) + xgrid.at<short>(i,j)*der_z_y.at<short>(i,j))/c;

            der_y_x.at<short>(i,j) = (depth_img.at<short>(i,j) + ygrid.at<short>(i,j)*der_z_x.at<short>(i,j))/c;
            der_y_y.at<short>(i,j) = (ygrid.at<short>(i,j)*der_z_y.at<short>(i,j))/c;

            // compute the cross product between tangent vectors
            // u_x = vec(der_x_x, der_y_x, der_z_x)
            // u_y = vec(der_x_y, der_y_y, der_z_y)
            x.at<short>(i,j) = der_y_x.at<short>(i,j) * der_z_y.at<short>(i,j) - der_z_x.at<short>(i,j) * der_y_y.at<short>(i,j);
            y.at<short>(i,j) = der_z_x.at<short>(i,j) * der_x_y.at<short>(i,j) - der_x_x.at<short>(i,j) * der_z_y.at<short>(i,j);
            z.at<short>(i,j) = der_x_x.at<short>(i,j) * der_y_y.at<short>(i,j) - der_y_x.at<short>(i,j) * der_x_y.at<short>(i,j);

            // cross product
            //    A = a1 * i + a2 * j + a3 * k
            //    B = b1 * i + b2 * j + b3 * k.
            //    (a2*b3 - a3*b2) * i
            //    (a3*b1 - a1*b3) * j
            //    (a1*b2 - a2*b1) * k

            normals.at<Vec3s>(i,j).val[0] = z.at<short>(i,j);
            normals.at<Vec3s>(i,j).val[1] = y.at<short>(i,j);
            normals.at<Vec3s>(i,j).val[2] = x.at<short >(i,j);
        }
    }
//    std::string window_name6 = "x";
//    namedWindow( window_name6, CV_WINDOW_AUTOSIZE );
//    imshow(window_name6, x);
//
//    std::string window_name7 = "y";
//    namedWindow( window_name7, CV_WINDOW_AUTOSIZE );
//    imshow(window_name7, y);
//
//    std::string window_name8 = "z";
//    namedWindow( window_name8, CV_WINDOW_AUTOSIZE );
//    imshow(window_name8, z);


    std::string window_name9 = "result";
    namedWindow( window_name9, CV_WINDOW_AUTOSIZE );
    imshow(window_name9, normals);
}

void PointCloud::edgeDetection(const Mat &img, const Mat &depth_img) {
    double t_rgb1=40.0, t_rgb2=100.0, t_hc1=0.6, t_hc2=1.2, t_dd=0.04, t_search=100;
    Mat img_gray;

    /// convert it to gray
    cvtColor(img, img_gray, CV_BGR2GRAY);
    std::string window_name = "grey img";
    namedWindow( window_name, CV_WINDOW_AUTOSIZE );
    imshow( window_name, img_gray );
    /// convert it to gray

    /// canny
    Mat Ergb, canny_img;
    int kernel_size = 3;

    canny_img.create( img.size(), img.type() );

    // Convert the image to grayscale
    cvtColor( img, img_gray, CV_BGR2GRAY );

    // Reduce noise with a kernel 3x3
    blur(img_gray, Ergb, Size(3,3) );

    // Canny detector
    Canny( Ergb, Ergb, t_rgb1, t_rgb2, kernel_size );

    // Using Canny's output as a mask, we display our result
    canny_img = Scalar::all(0);

    img.copyTo( canny_img, Ergb);
    std::string window_name2 = "canny of rgb";
    namedWindow( window_name2, CV_WINDOW_AUTOSIZE );
    imshow( window_name2, canny_img );
    /// canny

    /// normal estimation
    CameraConstants camera;
    Mat normals;
    normals.create(camera.image_height,camera.image_width, CV_16SC3);
    computeNormals(img, depth_img, normals);
    /// normal estimation
}
