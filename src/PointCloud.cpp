#include "PointCloud.h"
#include "ImageRGB.h"

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

void PointCloud::findAdjacentPoints(const Mat &l_frame, const Mat &r_frame, Mat &adjacency) {
/*
    //-> read first column
    for (int i=0; i<l_frame.cols; i++) {
        for (int j=0; j<l_frame.rows; j++) {
            if(i==0)
                l_frame.at<unsigned short>(j,i)
        }
    }
*/

    int window_size{50};
    int flag{1};
    int top_row, top_col, width, height;

    // how many pixels to slide the window in each direction (100 pixels aristera, 100 deksia, 100 panw kai 100 katw apo to trexwn (i,j))
    int slide{150};

    for (int col=0; col<r_frame.cols - window_size; col++) {
        for (int row=0; row<r_frame.rows - window_size; row++) {

//            /*
            if (flag++ == 58950) {
                cv::Rect img_roi(col, row, window_size, window_size);
                Mat img_template = l_frame(img_roi);

                top_col = col-slide;
                top_row = row-slide;
                width = 2*slide;
                height = 2*slide;

                validate(top_col, top_row, width, height);

                cv::Rect cube_roi(top_col, top_row, width, height);
                Mat img_cube = r_frame(cube_roi);

                Mat result;
                matchTemplate(img_cube, img_template, result, 5);

                cv::Point best_match;
                minMaxLoc(result, nullptr, nullptr, nullptr, &best_match);

                namedWindow("img_template", WINDOW_NORMAL);
                imshow("img_template", img_template);
                namedWindow("img_cube", WINDOW_NORMAL);
                imshow("img_cube", img_cube);
                cout << "template size = " << img_template.size << endl;
                cout << "img_cube size = " << img_cube.size << endl;

                Point pt_old =  Point(col, row);
//                Point pt_new =  Point(best_match.x, best_match.y);
                Point pt_new =  Point(best_match.x+top_col, best_match.y+top_row);

                Mat my_l_depth_mat, my_r_depth_mat;
                ImageRGB depth_image("../data/meeting_small_1/meeting_small_1_1.png");
                depth_image.convertToMat();
                depth_image.getMat(my_l_depth_mat);
                ImageRGB depth_image2("../data/meeting_small_1/meeting_small_1_2.png");
                depth_image2.convertToMat();
                depth_image2.getMat(my_r_depth_mat);

                namedWindow("test_left", WINDOW_NORMAL);
                namedWindow("test_right", WINDOW_NORMAL);

                circle( my_l_depth_mat, pt_old, 1, Scalar( 0, 0, 255 ), FILLED, LINE_8 );
                circle( my_r_depth_mat, pt_new, 1, Scalar( 0, 0, 255 ), FILLED, LINE_8 );

                imshow("test_left", my_l_depth_mat);
                imshow("test_right", my_r_depth_mat);

                cout << "old x = " << col << endl;
                cout << "old y = " << row << endl;
                cout << "new x = " << best_match.x << endl;
                cout << "new y = " << best_match.y << endl;
            }
//            */

        }
    }
}

void PointCloud::validate(int &top_col, int &top_row, int &width, int &height) {
    if (top_col < 0) {
        width += top_col;
        top_col = 0;
    }

    if (top_row < 0) {
        height += top_row;
        top_row = 0;
    }

    if ((width + top_col) > 640)
        width = 640 - top_col;

    if ((height + top_row) > 480)
        height = 480 - top_row;
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


























