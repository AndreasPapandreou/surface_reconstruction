#include "ImageRGB.h"

int ImageRGB::m_idGenerator = 1;

/* ------------------------------------------------------------------------------
 * Inputs       : -
 * Description  : Reads the image given its path, converts it to unsigned char
 *                type and stores it to Mat.
 * Return       : -
 * ------------------------------------------------------------------------------
*/
void ImageRGB::convertToMat()
{
    m_image_mat = imread(m_path, IMREAD_COLOR); // reads the image given its path
    if (m_image_mat.channels() == 1)
        m_image_mat.convertTo(m_image_mat, CV_8UC1); // converts data to unsigned char type, 8 bits per item and 1 channel

    if (m_image_mat.channels() == 3)
        m_image_mat.convertTo(m_image_mat, CV_8UC3); // converts data to unsigned char type, 8 bits per item and 1 channels
}

/* ------------------------------------------------------------------------------
 * Inputs       : Mat image
 * Description  : Gets the instance's image.
 * Return       : -
 * ------------------------------------------------------------------------------
*/
void ImageRGB::getMat(Mat &image)
{
    image = m_image_mat;
}

/* ------------------------------------------------------------------------------
 * Inputs       : -
 * Description  : Gets the id of instances' image.
 * Return       : The id of current image.
 * ------------------------------------------------------------------------------
*/
int ImageRGB::getId()
{
    return ImageRGB::m_id;
}