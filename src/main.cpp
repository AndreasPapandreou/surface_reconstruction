#include <iostream>
#include "surfaceReconstruction.h"
#include <vector>
#include <numeric>      // std::iota
#include <algorithm>    // std::sort

using namespace std;

template <typename T>
vector<size_t> sort_indexes(const vector<T> &v) {

    // initialize original index locations
    vector<size_t> idx(v.size());
    iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    sort(idx.begin(), idx.end(),
         [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

    return idx;
}

int main(int argc, char* argv[])
{
//    int i;
//    cout << "Please choose the dataset : \n";
//    cout << "Type 1 for MEETING_SMALL_1 : \n";
//    cin >> i;
//    TODO add checks for user's input

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


//    std::vector<int> myvector;
//
//    // set some values (from 1 to 10)
//    for (int i=1; i<=10; i++) myvector.push_back(i);
//
//    // erase the first 3 elements:
//    myvector.erase (myvector.begin(),myvector.begin()+1);
//
//    std::cout << "myvector contains:";
//    for (unsigned i=0; i<myvector.size(); ++i)
//        std::cout << ' ' << myvector[i];
//    std::cout << '\n';


//    vector<int> v;
//    v.emplace_back(3);
//    v.emplace_back(10);
//    v.emplace_back(1);
//
//    for (auto i: sort_indexes(v)) {
//        cout << v[i] << endl;
//    }
//    cout << endl;
//    for (auto i: sort_indexes(v)) {
//        cout << i << endl;
//    }
}