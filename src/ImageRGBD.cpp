#include "ImageRGBD.h"

int ImageRGBD::m_idGenerator = 1;

/* ------------------------------------------------------------------------------
 * Inputs       : -
 * Description  : Reads the image given its path, converts it to signed char type
 *                and stores it to Mat.
 * Return       : -
 * ------------------------------------------------------------------------------
*/
void ImageRGBD::convertToMat()
{
    m_depth_map = imread(m_path, IMREAD_ANYDEPTH); // Reads the image given its path
    m_depth_map.convertTo(m_depth_map, CV_16SC1); // converts data to signed char type, 16 bits per item and 1 channel
}

/* ------------------------------------------------------------------------------
 * Inputs       : Mat image
 * Description  : Gets the instance's depth image.
 * Return       : -
 * ------------------------------------------------------------------------------
*/
void ImageRGBD::getMat(Mat &image)
{
    image = m_depth_map;
}

/* ------------------------------------------------------------------------------
 * Inputs       : -
 * Description  : Gets the id of instances' depth image.
 * Return       : The id of current depth image.
 * ------------------------------------------------------------------------------
*/
int ImageRGBD::getId()
{
    return ImageRGBD::m_id;
}