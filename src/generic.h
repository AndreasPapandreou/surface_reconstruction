#ifndef LAB0_GENERIC_H
#define LAB0_GENERIC_H

#include <iostream>

using namespace std;

namespace generic {

    enum DatasetType {
        meeting_small_1 = 1
    };

    enum DatasetSize {
        meeting_small_1_size = 180
    };

    DatasetType convertToDatasetType(const int &type) {
        switch (type) {
            case 0 :
                return meeting_small_1;
            default:break;
        }
    }

    DatasetSize convertToDatasetSize(const int &type) {
        switch (type) {
            case 0 :
                return meeting_small_1_size;
            default:break;
        }
    }

    std::string convertToStr(const DatasetType &type) {
        switch (type) {
            case meeting_small_1 :
                return "meeting_small_1";
            default:break;
        }
    }

    std::string convertToStr(int index) {
        ostringstream stream;
        stream << index;
        return stream.str();
    }
};

#endif //LAB0_GENERIC_H
