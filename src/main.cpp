#include <iostream>
#include "surfaceReconstruction.h"

using namespace std;

int main(int argc, char* argv[])
{
    try {
        return vvr::mainLoop(argc, argv, new surfaceReconstruction);
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