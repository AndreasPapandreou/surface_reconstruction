#include <iostream>
#include "surfaceReconstruction.h"

using namespace std;

int main(int argc, char* argv[])
{
//    int i;
//    cout << "Please choose the dataset : \n";
//    cout << "Type 1 for MEETING_SMALL_1 : \n";
//    cin >> i;
    //TODO add checks for user's input

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