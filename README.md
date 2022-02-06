# Surface reconstruction from RGB-D images #

## Intro ##
This repository contains an assignment in the Computational Geometry course that took place during my master's degree. More specifically, given RGB-D images, our first goal was to reconstruct the 3D surface and generate a 3D mesh object. That was performed using the ICP algorithm while achieving faster convergence using filtering techniques. Later, a segmentation technique was implemented to distinguish between the different 3D mesh objects applying the SVD technique, using normal vectors and 3D planes.

## Compile & Link ##

For the C++ implementation:

```sh
mkdir build && cd build
cmake ..
make <-j # of cores>
```

## Run ##

While on the root directory of the cloned repository
```sh
cd build/
./Lab0
```
The next GUI will be shown that allows you to run each task separately. The window with the title "VVR Framework Scene" indicates the selected frame from the data.
![alt text](https://github.com/AndreasPapandreou/surface_reconstruction/blob/master/res/init.png?raw=true)

## Some results ##
**3D reconstruction using ICP for 15 frames**

![alt text](https://github.com/AndreasPapandreou/surface_reconstruction/blob/master/res/im1.png?raw=true)

**Segment 3D mesh object**

![alt text](https://github.com/AndreasPapandreou/surface_reconstruction/blob/master/res/plane.png?raw=true)

![alt text](https://github.com/AndreasPapandreou/surface_reconstruction/blob/master/res/im2.png?raw=true)
