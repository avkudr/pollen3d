/*!
 *  \brief     Stereo image dense DenseMatcher
 *  \details   This class is used to demonstrate a number of section commands.
 *  \author    Andrey Kudryavtsev
 *  \version   0.1
 *  \date      03/06/2016
 */

#ifndef DenseMatcher_H
#define DenseMatcher_H

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include "p3d/core/core.h"

class DenseMatcher
{
    friend class DenseMatcherWidget;

public:
    DenseMatcher();
    DenseMatcher(int method);
    ~DenseMatcher();

    void init(cv::Mat * lftIm, cv::Mat * rgtIm) { _lftIm = lftIm; _rgtIm = rgtIm;}
    void setMethod( int method ) { _method = method; }
    void setBounds( int lowerBound, int upperBound) {_params.lowerBound = lowerBound; _params.upperBound = upperBound;}
    void setBlockSize( int blockSize) {_params.blockSize = blockSize;}

    void calculateDisparityMap();
    void plotDisparityMap();

    void filterDisparity(int newVal, int maxSpeckleSize, int maxDiff);
    void plotDisparityFiltered();

    cv::Mat getDisparityToDisplay() const;

    enum methods{
        MODE_HH        = cv::StereoSGBM::MODE_HH,          ///< Perform linear interpolation on the table
        MODE_SGBM      = cv::StereoSGBM::MODE_SGBM,          ///< Perform parabolic interpolation on the table
        MODE_SGBM_3WAY = cv::StereoSGBM::MODE_SGBM_3WAY           ///< Perform parabolic interpolation on the table
    };

    bool isInitialized()        { return ! (_lftIm == NULL || _rgtIm == NULL) && ! _lftIm->empty() && ! _rgtIm->empty() ; } // _lftIm and _rgtIm are initialized
    bool isDisparityEmpty()     { return _disp.empty(); }
    bool isDisparityFiltered()  { return _dispFiltered.empty(); }

    cv::Mat _dispFiltered;
    cv::Mat _disp;
    cv::Mat _dispValues;

    void write(cv::FileStorage& fs) const;
    void read(const cv::FileNode& node);

private:

    int _method = MODE_SGBM; // Default method

    cv::Mat * _lftIm = NULL;
    cv::Mat * _rgtIm = NULL;

    struct Params{
        int lowerBound = -1;
        int upperBound = 2;
        int blockSize  = 9;
    };
    Params _params;

    struct ParamsFilter{
        int newVal = 0; // The disparity value used to paint-off the speckles
        int maxSpeckleSize = 260; // The maximum speckle size to consider it a speckle. Larger blobs are not affected by the algorithm
        int maxDiff = 10; // Maximum difference between neighbor disparity pixels to put them into the same blob. Note that since StereoBM,
        //StereoSGBM and may be other algorithms return a fixed-point disparity map, where disparity values are multiplied by 16,
        //this scale factor should be taken into account when specifying this parameter value.
    };
    ParamsFilter _paramsFilter;
};

static void read(const cv::FileNode& node, DenseMatcher& x, const DenseMatcher& default_value = DenseMatcher()){
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

/**************************
 *
 * CONTROL PART
 *
 * */


/**************************
 *
 * GUI PART
 *
 * */

#ifdef __GUI_QT__

#include <QWidget>
#include "ui_densematcherwidget.h"

namespace Ui {
class DenseMatcherWidget;
}

class DenseMatcherWidget : public QWidget
{
    Q_OBJECT

public:
    explicit DenseMatcherWidget(QWidget *parent = 0): QWidget(parent), ui(new Ui::DenseMatcherWidget) {
        ui->setupUi(this);
        update();
    }
    ~DenseMatcherWidget(){ delete ui;}

    void setDenseMatcher(DenseMatcher * dm){
        _dm = dm;
        ui->parLowerBound->setText(QString::number(_dm->_params.lowerBound));
        ui->parUpperBound->setText(QString::number(_dm->_params.upperBound));
        ui->parBlockSize ->setText(QString::number(_dm->_params.blockSize));
        update();
    }

private:
    Ui::DenseMatcherWidget *ui;
    DenseMatcher * _dm = new DenseMatcher();

    void update(){
        ui->calculateDisparity->setEnabled(_dm->isInitialized());
        ui->showDisparity->setEnabled( ! _dm->isDisparityEmpty());
        ui->filterBox->setEnabled(! _dm->isDisparityEmpty());
        ui->showDisparityFiltered->setEnabled(! _dm->isDisparityFiltered());
    }

signals:
    void changed();

public slots:
    void on_calculateDisparity_clicked(){
        _dm->setBounds(ui->parLowerBound->text().toInt(),ui->parUpperBound->text().toInt());
        _dm->setBlockSize(ui->parBlockSize->text().toInt());
        _dm->setMethod(ui->methodsList->currentIndex());
        _dm->calculateDisparityMap();
        update();
        emit changed();
    }
    void on_showDisparity_clicked(){
        _dm->plotDisparityMap();
        update();
    }
    void on_showDisparityFiltered_clicked(){
        _dm->plotDisparityFiltered();
        update();
    }
    void on_filterDisparity_clicked(){
        _dm->filterDisparity(ui->parNewValue->text().toInt(),
                             ui->parMaxSpeckleSize->text().toInt(),
                             ui->parMaxDiff->text().toInt());
        update();
        emit changed();
    }
};
#endif

#endif // DenseMatcher_H


