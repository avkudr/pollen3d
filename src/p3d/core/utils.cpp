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
#ifdef WITH_OPENMP
#ifdef WIN32
    return omp_get_num_procs();
#endif
    return omp_get_num_procs();
#endif
    return 1;
}

std::pair<Vec2, Vec2> utils::lineIntersectBox(const Vec3 &line, double w, double h){
    std::vector<Vec2> intesecPoints;
    intesecPoints.reserve(4);
    intesecPoints.emplace_back(Vec2(0,-line(2)/line(1)));
    intesecPoints.emplace_back(Vec2(w,-(line(2)+line(0)*w)/line(1)));
    intesecPoints.emplace_back(Vec2(-line(2)/line(0),0));
    intesecPoints.emplace_back(Vec2(-(line(2)+line(1)*h)/line(0),h));

    if ( intesecPoints[3][0] < 0 || intesecPoints[3][0] > w )
        intesecPoints.erase( intesecPoints.begin() + 3);
    if ( intesecPoints[2][0] < 0 || intesecPoints[2][0] > w )
        intesecPoints.erase( intesecPoints.begin() + 2);
    if ( intesecPoints[1][1] < 0 || intesecPoints[1][1] > h )
        intesecPoints.erase( intesecPoints.begin() + 1);
    if ( intesecPoints[0][1] < 0 || intesecPoints[0][1] > h )
        intesecPoints.erase( intesecPoints.begin() + 0);

    return {intesecPoints[0],intesecPoints[1]};
}

void utils::matchesMapsToTable(std::vector<std::map<int,int>> matchesMaps, Mati &table)
{
    auto nbPairs = matchesMaps.size();
    auto nbIm = nbPairs + 1;

    table.setZero(nbIm,0);

    Veci landmark;
    for (auto i = 0; i < nbPairs ; i++){
        for (auto it1 = matchesMaps[i].begin(); it1 != matchesMaps[i].end(); ++it1) {
            landmark.setOnes(nbIm);
            landmark *= -1;
            landmark[i] = it1->first;
            auto j = i + 1;
            auto final = it1->second;
            while (1) {
                landmark[j] = final;
                if (j >= nbPairs) break;
                if (matchesMaps[j].count(final) > 0) {
                    auto newFinal = matchesMaps[j].at(final);
                    matchesMaps[j].erase(final);
                    final = newFinal;
                }
                else break;
                ++j;
            }

            table.conservativeResize(Eigen::NoChange, table.cols()+1);
            table.rightCols(1) = landmark;
        }
    }
}

bool utils::equalsCvMat(const cv::Mat &lhs, const cv::Mat &rhs)
{
    if (lhs.rows != rhs.rows) return false;
    if (lhs.cols != rhs.cols) return false;
    if (lhs.type() != rhs.type()) return false;
    if (lhs.empty() || rhs.empty()) return false;
    return cv::norm(lhs - rhs) < 1e-5;
}

void utils::EulerZYZfromR(const Mat3 &R, double &t1, double &rho, double &t2){
    const auto &rotmat = R;

    if (rotmat(2,2) < 1){
        if (rotmat(2,2) > -1){
            rho = acos(rotmat(2,2));
            t1  = atan2(rotmat(1,2),rotmat(0,2));
            t2  = atan2(rotmat(2,1),-rotmat(2,0));
        }
        else{
            rho = M_PI;
            t1  = -atan2(rotmat(1,0),rotmat(1,1));
            t2  = 0;
        }
    }
    else{
        rho = 0;
        t1  = atan2(rotmat(1,0),rotmat(1,1));
        t2  = 0;
    }
}

Mat3 utils::skewSym(const Vec3 &a)
{
    Mat3 out;
    out <<    0, -a(2),  a(1),
           a(2),     0, -a(0),
          -a(1),  a(0),     0;
    return out;
}
