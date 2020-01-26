#include "project_data.h"

#include <iostream>


#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "p3d/core/matching.h"
#include "p3d/core/fundmat.h"
#include "p3d/core/rectification.h"
#include "p3d/core/core.h"

ProjectData::ProjectData() : Serializable(){
    clear();

    static bool firstCall = true;
    if (firstCall) {
        firstCall = false;
        meta::reflect<ProjectData>(p3d_hashStr("ProjectData"))
            .data<&ProjectData::setProjectPath,&ProjectData::getProjectPath>(p3d_hash(p3dData_projectPath))
            .data<&ProjectData::set_isDummy>(p3d_hash(p3dData_dummy))
            .data<&ProjectData::_images>(p3d_hash(p3dData_images));
    }
    //}
}

void ProjectData::clear()
{
    _images.clear();
    _imagesPairs.clear();
}

Image *ProjectData::getREFACTOR(const int index)
{
    return &_images[index];
}

ImagePair *ProjectData::getPairREFACTOR(const int index)
{
    return &_imagesPairs[index];
}

/*

void ProjectData::write(cv::FileStorage &fs) const                        //Write serialization for this class
{
    LOG_DBG("...saving project");

    int nbImages = static_cast<int>(_images.size());

    fs << "p" + std::to_string(p3dData_projectPath) << getProjectPath();
    fs << "imageNumber" << nbImages;

    meta::resolve<ProjectData>().data([&](meta::data data){
        data.type().construct();
    });

    for (int i = 0; i < nbImages; i++){
        fs << std::string("Image" + std::to_string(i));
        fs << "{";
        _images[i].write(fs);
        fs << "}";
    }

//    for (int i=0; i < _images.size() - 1; i++){
//        std::ostringstream ostr;
//        ostr << "ImagePair" << i;
//        fs << ostr.str();
//        fs << "{";
//        _imagesPairs[i].write(fs);
//        fs << "}";
//    }

//    fs << "MatchesTable" << _matchesTableFull;
//    fs << "FullMeasurementMatrix" << _measurementMatrixFull;
//    fs << "MeasurementMatrix" << _measurementMatrix;
    //_optimizationProblem->write(fs);
}

void ProjectData::read(const cv::FileStorage &fs)                          //Read serialization for this class
{
    int imNb = (int) fs["imageNumber"];
    std::string projectPath = fs["projectPath"];
    setProjectPath(projectPath);

    LOG_INFO("PRO_LOAD, im #%i", imNb);

    for (int i = 0; i < imNb; i++){
        std::ostringstream ostr;
        ostr << "Image" << i;
        Image im;
        fs[ostr.str()] >> im;
        _images.push_back(im);
    }

//    for (int i = 0; i < imNb - 1; i++){
//        std::ostringstream ostr;
//        ostr << "ImagePair" << i;
//        ImagePair imPair(&_images[i],&_images[i+1]);
//        fs[ostr.str()] >> imPair;
//        _imagesPairs.push_back(imPair);
//    }

//    fs["MatchesTable"] >> _matchesTableFull;
//    fs["FullMeasurementMatrix"] >> _measurementMatrixFull;
//    fs["MeasurementMatrix"] >> _measurementMatrix;
//    updateCameraMatrices();
    //_optimizationProblem->read(fs);

}
*/

void ProjectData::estimateMeasurementMatrixFull()
{
    int nbIm = (int) _images.size();

    std::vector<cv::Mat> matchingCols;
    matchingCols.clear();
    for (int i = 0; i < nbIm - 1; i++){
        matchingCols.push_back(_imagesPairs[i]._matcher->getMatchesAsIdxColumn());
    }

    cv::Mat table = cv::Mat::zeros(0, nbIm, CV_64F);

    for (int im = 0 ; im < nbIm - 1; im++){
        for (int j = 0; j < matchingCols[im].cols; j++){
            cv::Mat pointCorrespondence = cv::Mat::zeros(1, nbIm, CV_64F);

            int correspNb = 1;
            int a = j; // cols iterator - graph iterator
            pointCorrespondence.at<double>(im) = j;

            for (int im2 = im; im2 < nbIm - 1; im2++){

                if (a < matchingCols[im2].cols) {
                    int temp = a; // to put zero in a cell that was allready written
                    a = matchingCols[im2].at<double>(a);
                    matchingCols[im2].at<double>(temp) = 0;

                    if (a !=0 ){
                        pointCorrespondence.at<double>(im2 + 1) = a;
                        correspNb++;
                    }
                }
                else {
                    im2 = nbIm; // to go out from for
                }
            }

            if (correspNb > 1) table.push_back(pointCorrespondence);
         }
    }

    LOG_INFO("Table size: %ix%i", table.rows, table.cols);
    _matchesTableFull = table;

    /*************************************************/
    /*** FROM TABLE TO FULL Wf MEASUREMENT MATRIX */
    /*************************************************/

    cv::Mat tempTable;
    table.copyTo(tempTable);
    tempTable = tempTable.t();    // nbIm*3 rows, nbCorrespondence cols

    _measurementMatrixFull = cv::Mat::zeros(nbIm*3, tempTable.cols, CV_64F);

    for (int im = 0; im < nbIm; im++){ //rows
        for (int p = 0; p < tempTable.cols; p++){ //cols

            int number = (int) tempTable.at<double>(im,p);

            if (number != 0){
                _measurementMatrixFull.at<double>(3*im    ,p) = _images[im].getKeyPoints()[number].pt.x; //x
                _measurementMatrixFull.at<double>(3*im + 1,p) = _images[im].getKeyPoints()[number].pt.y; //y
                _measurementMatrixFull.at<double>(3*im + 2,p) = 1;                          //1
            }
        }
    }


    LOG_INFO( "Full measurement matrix size: %ix%i", _measurementMatrixFull.rows, _measurementMatrixFull.cols);

    //CvMatShowWidget::show(_measurementMatrixFull, "Full measurement matrix");
}

void ProjectData::estimateMeasurementMatrix()
{
    if (_measurementMatrixFull.empty()){
        this->estimateMeasurementMatrixFull();
    }

    int nbIm = (int) _images.size();

    // **** TABLE FILTERING

    // From table with 0 1 1 0 to 1 1 1 1: points visible in all images;

    cv::Mat tableFiltered = cv::Mat::zeros(0, nbIm, CV_64F);    // nbIm cols, 0 rows

    for (int i = 0; i < _matchesTableFull.rows; i++){

        double a = _matchesTableFull.at<double>(i,0);
        for (int j = 1; j < _matchesTableFull.cols; j++){
            a = a*_matchesTableFull.at<double>(i,j);
        }

        if ( a != 0 ) tableFiltered.push_back(_matchesTableFull.row(i));
    }

    // **** FROM FILTERED TABLE TO W MEASUREMENT MATRIX */
    tableFiltered = tableFiltered.t();

    _measurementMatrix = cv::Mat::ones(nbIm*3, tableFiltered.cols, CV_64F);

    for (int im = 0; im < nbIm; im++){ //rows
        for (int p = 0; p < tableFiltered.cols; p++){ //cols

            int number = (int) tableFiltered.at<double>(im,p);

            _measurementMatrix.at<double>(3*im    ,p) = _images[im].getKeyPoints()[number].pt.x; //x
            _measurementMatrix.at<double>(3*im + 1,p) = _images[im].getKeyPoints()[number].pt.y; //y
        }
    }

    LOG_INFO( "Measurement matrix size: %i", _measurementMatrix.rows, _measurementMatrix.cols );
}

void ProjectData::estimateMeasurementMatrixKLT()
{
    LOG_INFO("Number of images: %i", nbImages());
    std::vector<std::vector<cv::Point2f>> points(nbImages(), std::vector<cv::Point2f>(0));
    cv::TermCriteria termcrit;
    cv::Size subPixWinSize, winSize;
    termcrit = cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);
    subPixWinSize = cv::Size(10,10);
    winSize = cv::Size(51,51);

    bool needToInit = (this->getREFACTOR(0)->getNbFeatures() == 0);
    if( needToInit )
    {
        LOG_INFO("Detecting new points");
        cv::Mat image0 = this->getREFACTOR(0)->getREFACTOR().clone();
        std::vector<cv::KeyPoint> kpts0;
        cv::Mat desc0;

        cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.005f);
        akaze->detectAndCompute(image0, cv::noArray(), kpts0, desc0);

        for (int i = 0; i < (int)kpts0.size(); i++){
            points[0].push_back(cv::Point2f((float)kpts0[i].pt.x, (float)kpts0[i].pt.y));
        }
        LOG_INFO("First image point nb: %i", points[0].size());
    }else{
        for (int i = 0; i < (int)this->getREFACTOR(0)->_kpts.size(); i++){
            points[0].push_back(cv::Point2f((float)this->getREFACTOR(0)->_kpts[i].pt.x, (float)this->getREFACTOR(0)->_kpts[i].pt.y));
        }
        LOG_INFO("Using predefined points");
    }

    for (int im = 0; im < this->nbImages()-1; im++){
        //cv::imshow("LK Demo", (cv::Mat)(this->get(i)->get()));

        cv::Mat image0 = this->getREFACTOR(im)->getREFACTOR();
        cv::Mat image1 = this->getREFACTOR(im+1)->getREFACTOR();

        if( !points[0].empty() )
        {
            std::vector<uchar> status;
            std::vector<float> err;
            cv::calcOpticalFlowPyrLK(image0, image1, points[im], points[im+1], status, err, winSize, 3, termcrit, 0, 0.04);
            for( auto k = 0; k < points[0].size(); k++ )
            {
                if( !status[k] ){
                    points[im+1][k] = cv::Point2f(0,0);
                }
            }
        }
    }

    int nbIm = (int) this->nbImages();
    int nbPts = (int) points[0].size();
    cv::Mat Wfull = cv::Mat::zeros(3*nbIm,nbPts,cv::DataType<double>::type);
    for(int im = 0; im < nbIm; im++){
        for(int p = 0; p < nbPts; p++){
            Wfull.at<double>(3*im + 0,p) = points[im][p].x;
            Wfull.at<double>(3*im + 1,p) = points[im][p].y;
            Wfull.at<double>(3*im + 2,p) = 1;
        }
        this->getREFACTOR(im)->_kpts.clear();
    }

    cv::Mat W = cv::Mat::zeros(2*nbIm,0,cv::DataType<double>::type);
    for(int p = 0; p < nbPts; p++){
        double min, max;
        cv::minMaxLoc(cv::Mat(Wfull.col(p)), &min, &max);
        if (min > 0){
            W.push_back(Wfull.col(p).t());
            for(int im = 0; im < nbIm; im++){
                this->getREFACTOR(im)->_kpts.push_back(cv::KeyPoint(points[im][p].x,points[im][p].y,1.f));
            }
        }
    }
    W = W.t();

    nbPts = W.cols;
    this->_measurementMatrix.release();
    this->_measurementMatrix = W.clone();

    for (int i = 0; i < this->_imagesPairs.size(); i++){
        cv::Mat matches = cv::Mat::zeros(0,2,cv::DataType<double>::type);
        for(int p = 0; p < nbPts; p++){
            cv::Mat row = (cv::Mat_<double>(1,2) << p,p);
            matches.push_back(row);
        }

        ImagePair * pair = &this->_imagesPairs[i];
        pair->_matcher->setMatchesTable(matches);
    }
    LOG_INFO( "Obtained measurement matrix: %ix%i", W.rows, W.cols );
}

void ProjectData::updateCameraMatrices()
{
    if ( ! this->getREFACTOR(0)->P.empty()){
        int nbCam = (int) this->nbImages();
        std::vector<cv::Mat> Parray(nbCam);

        for (int i = 0; i < nbCam; i++){
            Parray[i] = this->getREFACTOR(i)->P;
        }

        _Pm = utils::concatenateMat(Parray);
    }
}

/*
void ProjectData::updatePAAAroperties()
{

    _properties.clear();
    if (!_matchesTableFull.empty()) _properties.push_back(new CVMatProperty(&_matchesTableFull, "Matches table"));
    if (!_measurementMatrixFull.empty()) _properties.push_back(new CVMatProperty(&_measurementMatrixFull, "Wf"));
    if (!_measurementMatrix.empty()) _properties.push_back(new CVMatProperty(&_measurementMatrix, "W"));
    _properties.push_back(new CVMatProperty(&_Pm, "Camera matrices"));
    //_properties.push_back(new CVMatProperty(&_optimizationProblem->_Xm, "Points 3D"));
    _properties.push_back(new CVMatProperty(&_sparsePointCloudMat, "PCD Sparse"));
}
*/
