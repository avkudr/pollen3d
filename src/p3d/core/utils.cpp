#include "utils.h"

#ifdef _OPENMP
#include <omp.h>
#endif

void utils::saveFileToMatlab(std::string fileName, cv::Mat a, std::string varName){
    int rows = (int) a.rows;
    int cols = (int) a.cols;

    std::ofstream myfile;
    myfile.open (fileName);
    myfile << "%File generated with Pollen3D \n";

    myfile << varName << " = [";

    for (int row = 0; row < rows; row++){
        for (int col = 0; col < cols; col++){
            myfile << a.at<double>(row,col) << " ";
        }
        if ( row != rows - 1) myfile << ";\n";
        else                  myfile << " ];\n";
    }

    myfile.close();
}


double utils::rad2deg(double angRad){
    return angRad / M_PI * 180.0;
}

double utils::deg2rad(double angDeg){
    return angDeg / 180.0 * M_PI;
}

/*! Make matrix non-homogenious. It is mainly used for measurement matrix \f$W\f$.
 * If ((W mod 3) == 0) it will supress every third row
 */
void utils::makeNonHomogenious(cv::Mat &m)
{
    // Make W non-homogenious
    bool isNonHomogenious = !( (m.rows % 3) == 0 && (m.at<double>(2,0) == 1));
    if (isNonHomogenious)
        return;

    cv::Mat newM;

    for (int i = 0 ; i < m.rows / 3 ; i++){
        newM.push_back(m.row(3*i));
        newM.push_back(m.row(3*i + 1));
    }

    m.release();
    m = newM.clone();
}

/*! Copy elements of matrix with indices idx to vector
 * Equivalent to MATLAB: v = M(idx);
 */
void utils::copyMatElementsToVector(const cv::Mat &mat, const cv::Mat &idx, std::vector<double> &vec)
{
    int nbIdx = (int) idx.total();

    for (int i = 0; i < nbIdx; i++){
        int col = (int) idx.at<cv::Point>(i).x;
        int row = (int) idx.at<cv::Point>(i).y;

        vec[i] = mat.at<double>(row,col);
    }
}

/*! Copy vector to matrix elements with indices idx
 * Equivalent to MATLAB: M(idx) = v;
 */
void utils::copyVectorToMatElements(const std::vector<double> &vec, const cv::Mat &idx, cv::Mat &mat)
{
    int nbIdx = (int) idx.total();

    for (int i = 0; i < nbIdx; i++){
        int col = (int) idx.at<cv::Point>(i).x;
        int row = (int) idx.at<cv::Point>(i).y;

        mat.row(row).col(col) = vec[i];
    }
}

/*! Concatenate vector of matrices
 * \param matArray vector of matrices
 * \param method concatenation method
 * - CONCAT_HORIZONTAL (Horizontally) : M = [M1 M2 M3 M4]
 * - CONCAT_VERTICAL (Vertically)  : M = [M1;M2;M3;M4]
 */
cv::Mat utils::concatenateMat(const std::vector<cv::Mat> &matArray, int method)
{
    cv::Mat outMat;
    if (method == CONCAT_VERTICAL){
        for(auto& m:matArray){
            outMat.push_back(m.clone());
        }
    }else{
        for(auto& m:matArray){
            cv::Mat mt = m.t();
            outMat.push_back(mt.clone());
        }
        outMat = outMat.t();
    }
    return outMat.clone();
}


std::string utils::baseNameFromPath(const std::string &path) {
    std::string name = path;
    const size_t last_slash_idx = name.find_last_of("\\/");
    if (std::string::npos != last_slash_idx)
    {
        name.erase(0, last_slash_idx + 1);
    }
    return name;
}

bool utils::endsWith(const std::string &value, const std::string &ending)
{
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

std::vector<std::string> utils::split(const std::string &str, const std::string &delim)
{
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == std::string::npos) pos = str.length();
        std::string token = str.substr(prev, pos-prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    }
    while (pos < str.length() && prev < str.length());

    return tokens;
}

int utils::nbAvailableThreads(){
#ifdef WIN32
    return omp_get_num_procs();
#endif
    return omp_get_num_procs();
}
