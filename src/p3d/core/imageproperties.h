#ifndef PROPERTY_H
#define PROPERTY_H

#include <vector>
#include <string>
#include <iostream>

#include <opencv2/core.hpp>

#include "p3d/core/fundmat.h"
#include "p3d/core/utils.h"

class Property
{
public:
    Property(){
    }
    ~Property(){}

    std::vector<std::string> get(){
        std::vector<std::string> out(3);
        out[0] = _varName;
        out[1] = _varType;
        out[2] = _varSize;
        return out;
    }

    virtual void show() = 0;
    virtual bool isEmpty() = 0;
protected:
    std::string _varName = "-";
    std::string _varType = "-";
    std::string _varSize = "";
};

class CVMatProperty : virtual public Property{
public:
    CVMatProperty(cv::Mat * ptr, std::string varName){
        _ptrToMat = ptr;
        _varName  = varName;
        _varType  = "Mat";
        std::ostringstream os;
        os << _ptrToMat->rows << "x" << _ptrToMat->cols;
        _varSize  = os.str();
    }
    void show(){
    }
    bool isEmpty(){
        return _ptrToMat->empty();
    }
private:
    cv::Mat * _ptrToMat;
};

//ImageInterface
/* Property
 *      std::string name;
 *      std::string type;
 *      std::string size;
 *      virtual bool isEmpty() = 0;
 *      virtual std::string getType() = 0;
 *      virtual std::string getName() = 0;
 *      virtual std::string getSize() = 0;
 *      virtual void show() = 0;
 *      virtual void saveToFile(std::string fileName){}// or stringstream...
 * slot:
 *      virtual void clicked{ show() }
 *
 * StringProperty : Property
 *      has a * string
 * CvMatProperty : cv::Mat, Property
 *      has a * cv::Mat
 * StdVectorMatProperty : std::
 *      has a * std::vector;
*/

//class ImageProperties{
//public:
//    struct Property{
//        void * ptr;
//        std::string name;
//        std::string type;
//        std::string size;
//    };

//    enum propertiesName{Path, Features, }

//    std::vector<Property> properties;
//};

#endif // PROPERTY_H
