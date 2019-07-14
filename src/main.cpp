#include <iostream>
#include "surfaceReconstruction.h"

using namespace std;

int main(int argc, char* argv[])
{
    /// there are two datasets : type 0 for the meeting_small_1 and 1 for desk_1
    /// all of our experiments has been conducted to the firsτ dataset, so we set i to 0
    int i{0};
    try {
        return vvr::mainLoop(argc, argv, new surfaceReconstruction(i));
    }
    catch (std::string exc) {
        cerr << exc << endl;
        return 1;
    }
    catch (...)
    {
        cerr << "Unknown exception" << endl;
        return 1;
    }
}