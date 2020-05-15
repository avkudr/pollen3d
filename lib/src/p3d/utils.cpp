#include "utils.h"

#ifdef _WIN32
#include <windows.h>
#elif defined TARGET_OS_MAC

#elif defined __linux__
#include <linux/limits.h>
#include <unistd.h>
#endif

#ifdef WITH_OPENMP
#include <omp.h>
#endif

#include <Eigen/Eigen>

#include "p3d/logger.h"

using namespace p3d;

void utils::saveFileToMatlab(std::string fileName, cv::Mat a, std::string varName)
{
    int rows = (int)a.rows;
    int cols = (int)a.cols;

    std::ofstream myfile;
    myfile.open(fileName);
    myfile << "%File generated with pollen3d \n";

    myfile << varName << " = [";

    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            myfile << a.at<double>(row, col) << " ";
        }
        if (row != rows - 1)
            myfile << ";\n";
        else
            myfile << " ];\n";
    }

    myfile.close();
}

/*! Make matrix non-homogenious. It is mainly used for measurement matrix \f$W\f$.
 * If ((W mod 3) == 0) it will supress every third row
 */
void utils::makeNonHomogenious(Mat &m)
{
    if (m.rows() == 0) return;
    if (m.cols() == 0) return;

    // Make W non-homogenious
    bool isNonHomogenious = !((m.rows() % 3) == 0 && (m(2, 0) == 1));
    if (isNonHomogenious)
        return;

    Mat newM;
    newM.setOnes(2 * m.rows() / 3, m.cols());

    for (int i = 0; i < m.rows() / 3; i++) {
        newM.row(2 * i) = m.row(3 * i);
        newM.row(2 * i + 1) = m.row(3 * i + 1);
    }

    m = newM;
}

/*! Copy elements of matrix with indices idx to vector
 * Equivalent to MATLAB: v = M(idx);
 */
void utils::copyMatElementsToVector(const Mat &mat, const std::vector<std::pair<int, int>> &idx, std::vector<double> &vec)
{
    int nbIdx = (int)idx.size();

    for (int i = 0; i < nbIdx; i++) {
        int row = (int)idx[i].first;
        int col = (int)idx[i].second;

        vec[i] = mat(row, col);
    }
}

/*! Copy vector to matrix elements with indices idx
 * Equivalent to MATLAB: M(idx) = v;
 */
void utils::copyVectorToMatElements(const std::vector<double> &vec, const std::vector<std::pair<int, int>> &idx, Mat &mat)
{
    int nbIdx = (int)idx.size();

    for (int i = 0; i < nbIdx; i++) {
        int row = (int)idx[i].first;
        int col = (int)idx[i].second;

        mat(row, col) = vec[i];
    }
}

/*! Concatenate vector of matrices
 * \param matArray vector of matrices
 * \param method concatenation method
 * - CONCAT_HORIZONTAL (Horizontally) : M = [M1 M2 M3 M4]
 * - CONCAT_VERTICAL (Vertically)  : M = [M1;M2;M3;M4]
 */
cv::Mat utils::concatenateCvMat(const std::vector<cv::Mat> &matArray, int method)
{
    cv::Mat outMat;
    if (method == CONCAT_VERTICAL) {
        for (auto &m : matArray) {
            outMat.push_back(m.clone());
        }
    } else {
        for (auto &m : matArray) {
            cv::Mat mt = m.t();
            outMat.push_back(mt.clone());
        }
        outMat = outMat.t();
    }
    return outMat.clone();
}

std::string utils::baseNameFromPath(const std::string &path)
{
    std::string name = path;
    const size_t last_slash_idx = name.find_last_of("\\/");
    if (std::string::npos != last_slash_idx) {
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
    do {
        pos = str.find(delim, prev);
        if (pos == std::string::npos) pos = str.length();
        std::string token = str.substr(prev, pos - prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    } while (pos < str.length() && prev < str.length());

    return tokens;
}

int utils::nbAvailableThreads()
{
#ifdef WITH_OPENMP
#ifdef WIN32
    return omp_get_num_procs();
#endif
    return omp_get_num_procs();
#endif
    return 1;
}

std::pair<Vec2, Vec2> utils::lineIntersectBox(const Vec3 &line, double w, double h)
{
    std::vector<Vec2> intesecPoints;
    intesecPoints.reserve(4);
    intesecPoints.emplace_back(Vec2(0, -line(2) / line(1)));
    intesecPoints.emplace_back(Vec2(w, -(line(2) + line(0) * w) / line(1)));
    intesecPoints.emplace_back(Vec2(-line(2) / line(0), 0));
    intesecPoints.emplace_back(Vec2(-(line(2) + line(1) * h) / line(0), h));

    if (intesecPoints[3][0] < 0 || intesecPoints[3][0] > w)
        intesecPoints.erase(intesecPoints.begin() + 3);
    if (intesecPoints[2][0] < 0 || intesecPoints[2][0] > w)
        intesecPoints.erase(intesecPoints.begin() + 2);
    if (intesecPoints[1][1] < 0 || intesecPoints[1][1] > h)
        intesecPoints.erase(intesecPoints.begin() + 1);
    if (intesecPoints[0][1] < 0 || intesecPoints[0][1] > h)
        intesecPoints.erase(intesecPoints.begin() + 0);

    return {intesecPoints[0], intesecPoints[1]};
}

void utils::matchesMapsToTable(std::vector<std::map<int, int>> matchesMaps, Mati &table)
{
    LOG_DBG("TO DO: Move to MatchingUtil!");

    auto nbPairs = matchesMaps.size();
    auto nbIm = nbPairs + 1;

    table.setZero(nbIm, 0);

    Veci landmark;
    for (auto i = 0; i < nbPairs; i++) {
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
                } else
                    break;
                ++j;
            }

            table.conservativeResize(Eigen::NoChange, table.cols() + 1);
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

Mat3 utils::skewSym(const Vec3 &a)
{
    Mat3 out;
    out << 0, -a(2), a(1),
        a(2), 0, -a(0),
        -a(1), a(0), 0;
    return out;
}

double utils::nullspace(const Eigen::Ref<const Mat> &A, Vec *nullsp)
{
    if (!nullsp) return -1.0;

    if (A.rows() >= A.cols()) {
        Eigen::JacobiSVD<Mat> svd(A, Eigen::ComputeFullV);
        *nullsp = svd.matrixV().col(A.cols() - 1);
        return svd.singularValues()(A.cols() - 1);
    }
    // Extend A with rows of zeros to make it square. It's a hack, but it is
    // necessary until Eigen supports SVD with more columns than rows.
    Mat A_extended(A.cols(), A.cols());
    A_extended.block(A.rows(), 0, A.cols() - A.rows(), A.cols()).setZero();
    A_extended.block(0, 0, A.rows(), A.cols()) = A;
    return nullspace(A_extended, nullsp);
}

bool utils::exportToPly(const Mat3Xf &vec_points_white,
                        const std::string &sFileName, const Mat3Xf &colors)
{
    std::ofstream outfile;
    outfile.open(sFileName.c_str(), std::ios_base::out);
    outfile << "ply" << '\n'
            << "format ascii 1.0" << '\n'
            << "element vertex " << vec_points_white.cols() << '\n'
            << "property float x" << '\n'
            << "property float y" << '\n'
            << "property float z" << '\n'
            << "property uchar red" << '\n'
            << "property uchar green" << '\n'
            << "property uchar blue" << '\n'
            << "end_header" << std::endl;

    Mat3Xf col;
    if (colors.cols() == vec_points_white.cols())
        col = colors;
    else {
        col.setOnes(3, vec_points_white.cols());
        col *= 255;
    }
    for (auto i = 0; i < vec_points_white.cols(); ++i) {
        outfile << vec_points_white.col(i).transpose() << " "
                << col.col(i).cast<int>().transpose() << "\n";
    }
    outfile << "\n";
    outfile.flush();
    bool bOk = outfile.good();
    outfile.close();
    return bOk;
}

Vec4 utils::triangulate(const std::vector<Vec2> &x, const std::vector<Mat34> &Ps)
{
    int nviews = x.size();
    assert(nviews == Ps.size());

    Mat design = Mat::Zero(3 * nviews, 4 + nviews);
    for (int i = 0; i < nviews; i++) {
        design.block<3, 4>(3 * i, 0) = -Ps[i];
        design(3 * i + 0, 4 + i) = x[i](0);
        design(3 * i + 1, 4 + i) = x[i](1);
        design(3 * i + 2, 4 + i) = 1.0;
    }
    Vec X_and_alphas;
    utils::nullspace(design, &X_and_alphas);
    Vec4 X = X_and_alphas.head(4);
    X /= X.w();
    return X;
}

void utils::EulerZYZtfromR(const Mat3 &R, double &t1, double &rho, double &t2)
{
    const auto &rotmat = R;

    if (rotmat(2, 2) < 1) {
        if (rotmat(2, 2) > -1) {
            rho = acos(rotmat(2, 2));
            t1 = atan2(rotmat(1, 2), rotmat(0, 2));
            t2 = atan2(-rotmat(2, 1), -rotmat(2, 0));
        } else {
            rho = M_PI;
            t1 = -atan2(rotmat(1, 0), rotmat(1, 1));
            t2 = 0;
        }
    } else {
        rho = 0;
        t1 = atan2(rotmat(1, 0), rotmat(1, 1));
        t2 = 0;
    }

    if (t1 < M_PI_2) {
        t1 += M_PI;
        rho = -rho;
    }
    if (t1 > M_PI_2) {
        t1 -= M_PI;
        rho = -rho;
    }
    wrapHalfPI(t2);
}

void utils::EulerZYZtfromRinv(const Mat3 &R, double &t1, double &rho, double &t2)
{
    double a = 0, b = 0, c = 0;
    utils::EulerZYZtfromR(R, a, b, c);
    t2 = a;
    rho = -b;
    t1 = c;
}

void utils::convert(const std::vector<Vec3f> &src, Mat3Xf &dst)
{
    dst.setZero(3, src.size());
    for (int i = 0; i < src.size(); ++i) dst.col(i) = src[i];
}

void utils::convert(const std::vector<Vec3> &src, Mat3Xf &dst)
{
    dst.setZero(3, src.size());
    for (int i = 0; i < src.size(); ++i) dst.col(i) = src[i].cast<float>();
}

void utils::convert(const std::vector<Vec4> &src, Mat4X &dst)
{
    dst.setZero(4, src.size());
    for (int i = 0; i < src.size(); ++i) dst.col(i) = src[i];
}

void utils::convert(const std::vector<Vec3> &src, Mat4X &dst)
{
    dst.setOnes(4, src.size());
    for (int i = 0; i < src.size(); ++i) dst.col(i).topRows(3) = src[i];
}

void utils::wrapHalfPI(double &angleRad)
{
    while (angleRad < -M_PI / 2) angleRad += M_PI;
    while (angleRad > M_PI / 2) angleRad -= M_PI;
}

std::string utils::to_string(const std::vector<int> &vec)
{
    std::string s = "";
    for (const auto &v : vec) {
        if (s != "") s += ",";
        s += std::to_string(v);
    }
    return "{" + s + "}";
}

std::string utils::getExecPath()
{
#ifdef __linux__
    char buff[PATH_MAX];
    ssize_t len = ::readlink("/proc/self/exe", buff, sizeof(buff) - 1);
    if (len != -1) {
        buff[len] = '\0';
        auto file = std::string(buff);
        auto found = file.find_last_of("/\\");
        return file.substr(0, found);
    }
#endif
    return ".";
}

void utils::findFullSubMeasurementMatrix(const Mat &W, Mat &Wfsub, std::vector<int> camIds, std::vector<int> *selectedCols)
{
    Wfsub = Mat();
    if (W.cols() == 0 || W.rows() == 0) {
        LOG_ERR("Full measurement matrix can't be empty");
        return;
    }

    if (W.rows() % 3 != 0) {
        LOG_ERR("Measurement matrix must have 3*nbCams rows");
        return;
    }

    int nbImgsTotal = W.rows() / 3;
    if (nbImgsTotal < 2) return;

    if (camIds.empty()) {
        for (int i = 0; i < nbImgsTotal; i++) camIds.emplace_back(i);
    }

    if (camIds.size() > nbImgsTotal) return;
    if (camIds.size() < 2) return;

    const int nbCams = camIds.size();

    // same as measurement matrix but only for selected images
    Mat Wsub{Mat::Zero(3*nbCams,W.cols())};
    for (auto n = 0; n < camIds.size(); n++) {
        int c = camIds[n];
        Wsub.middleRows(3*n,3) = W.middleRows(3*c,3);
    }

    std::vector<int> selCols;
    Wfsub.setZero(Wsub.rows(), 0);
    for (auto c = 0; c < Wsub.cols(); ++c) {
        if (std::abs(Wsub.col(c).prod()) > 1e-5) {  // there is no zeros in the column
            Wfsub.conservativeResize(Eigen::NoChange, Wfsub.cols() + 1);
            Wfsub.rightCols(1) = Wsub.col(c);
            selCols.emplace_back(c);
        }
    }
    if (selectedCols) *selectedCols = selCols;
}

std::vector<std::vector<int> > utils::generateBatches(int nbCams, int batchSize)
{
    std::vector<std::vector<int> > batches{};
    if (batchSize < 3) return batches;
    int overlap = 1;

    int i = 0;
    while (i < nbCams - 1) {
        std::vector<int> batch;
        if (i + batchSize - 1 < nbCams) {
        } else {
            i = nbCams - batchSize;
        }

        for (int c = 0; c < batchSize; c++)
            batch.emplace_back(i+c);

        batches.push_back(batch);

        i += batchSize - overlap;
    }
    return batches;
}

void utils::convert(const Mat2X &src, std::vector<Vec2> &dst)
{
    dst.clear();
    dst.reserve(src.cols());
    for (int i = 0; i < src.cols(); i++) {
        dst.emplace_back(src.col(i));
    }
}

void utils::convert(const std::vector<double> &src, Vec &dst)
{
    dst.setZero(src.size());
    for (int i = 0; i < src.size(); i++) {
        dst(i) = src[i];
    }
}

void utils::EulerZYXfromR(const Mat3 &R, double &rx, double &ry, double &rz)
{
    using T = double;
    Eigen::Matrix<T, 3, 3> M = R.transpose();
    for (const int i : {0,1,2}){
        T s = M.col(i).norm();
        M.col(i).array() /= s;
    }

    rx = (T) std::atan2 (M(1,2), M(2,2));

    Eigen::Matrix<T, 3, 3> N;
    N = Eigen::Matrix<T, 3, 3>(Eigen::AngleAxis<T>(-rx, Eigen::Matrix<T, 3, 1>::UnitX())).transpose() * M;

    T cy =   (T) std::sqrt (N(0,0)*N(0,0) + N(0,1)*N(0,1));
    ry = (T) std::atan2 (-N(0,2), cy);
    rz = (T) std::atan2 (-N(1,0), N(1,1));
}

Vec utils::reprojectionError(const Mat &W, const Mat &P, const Mat4X &X, std::vector<int> selCams) {
    Mat err = W - P * X;
    int nbCams = W.rows() / 3;
    int nbPts = W.cols();
    if (selCams.empty()){
        for (int p = 0; p < nbCams; ++p) selCams.push_back(p);
    }

    std::vector<double> errs;
    for (int p = 0; p < nbPts; ++p) {
        if (X(3,p) != 1) continue;

        double meanErr = 0;
        int nbCamsVis = 0;
        for (int ic = 0; ic < selCams.size(); ++ic) {
            int c = selCams[ic];
            if (W(3*c+2,p) != 1) continue;

            meanErr += err.block(3*c,p,2,1).norm();
            nbCamsVis++;

            errs.push_back(err.block(3*c,p,2,1).norm());
        }

        if (nbCamsVis == 0) continue;

    }

    Vec e;
    utils::convert(errs,e);

    return e;
}

float utils::reprojectionErrorPt(const std::vector<Vec2> &x, const std::vector<Mat34> &P,
                                 const Vec4 &X)
{
    int nbCams = x.size();
    if (nbCams == 0) return std::numeric_limits<float>::max();

    double meanErr = 0;
    for (int c = 0; c < nbCams; ++c) {
        Vec3 q = P[c] * X;
        q = q / q(2);
        meanErr += Vec2(x[c] - q.topRows(2)).norm();
    }

    meanErr /= float(nbCams);
    return meanErr;
}
