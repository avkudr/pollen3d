#pragma once

#include <vector>

namespace p3d
{
enum p3dBundleIdx_ {
    p3dBundleIdx_Alpha = 0,  ///< fx/fy, aspect ratio
    p3dBundleIdx_Skew = 1,   ///< skew
    p3dBundleIdx_Focal = 2,  ///< focal length

    p3dBundleIdx_Theta1 = 3,  ///< theta_1
    p3dBundleIdx_Rho = 4,     ///< rho
    p3dBundleIdx_Theta2 = 5,  ///< theta_2

    p3dBundleIdx_t = 6,  ///< translation
    p3dBundleIdx_TOTAL = 7
};

enum p3dBundleParam_ {
    p3dBundleParam_Alpha = 1 << p3dBundleIdx_Alpha,  ///< fx/fy, aspect ratio
    p3dBundleParam_Skew = 1 << p3dBundleIdx_Skew,    ///< skew
    p3dBundleParam_Focal = 1 << p3dBundleIdx_Focal,  ///< skew
    p3dBundleParam_K =
        p3dBundleParam_Focal + p3dBundleParam_Skew + p3dBundleParam_Alpha,

    p3dBundleParam_Theta1 = 1 << p3dBundleIdx_Theta1,  // theta_1
    p3dBundleParam_Rho = 1 << p3dBundleIdx_Rho,        // rho
    p3dBundleParam_Theta2 = 1 << p3dBundleIdx_Theta2,  // theta_2
    p3dBundleParam_R =
        p3dBundleParam_Theta1 + p3dBundleParam_Rho + p3dBundleParam_Theta2,

    p3dBundleParam_t = 1 << p3dBundleIdx_t,

    p3dBundleParam_ALL = 0xFFFF
};

/**
 * @brief BundleParams is a class collecting the parameters of bundle
 * adjustment (BA) problem
 *
 * It mostly serves to define which parameters will be held constant during BA
 * and which will not
 */

class BundleParams{

public:
    BundleParams() = delete;
    BundleParams(int nbCams){
        m_constCams = std::vector<unsigned int>(nbCams,0);
    }

    inline int nbCams() const { return m_constCams.size(); }
    int getParIdx(p3dBundleParam_ p) { return 0; }

    bool isConst(p3dBundleParam_ param, size_t camIdx) const
    {
        return ((m_constCams[camIdx] & param) != 0);
    }
    bool isVarying(p3dBundleParam_ param, size_t camIdx) const
    {
        return !isConst(param, camIdx);
    }

    void setConst(p3dBundleParam_ param, std::vector<int> cams){ setParam(param,cams,true);}
    void setConstAllCams(p3dBundleParam_ param){ setConst(param,{}); }
    void setConstAllParams(std::vector<int> cams){ setConst(p3dBundleParam_ALL,cams); }
    void setConstAll(){ setConst(p3dBundleParam_ALL,{}); setConstPts(); }

    void setVarying(p3dBundleParam_ param, std::vector<int> cams){ setParam(param,cams,false);}
    void setVaryingAllCams(p3dBundleParam_ param){ setVarying(param,{}); }
    void setVaryingAllParams(std::vector<int> cams){ setVarying(p3dBundleParam_ALL,cams); }
    void setVaryingAll(){ setVarying(p3dBundleParam_ALL,{}); setVaryingPts(); }

    bool isConstPts() const { return constPts3D; }
    void setConstPts(){ constPts3D = true; }
    void setVaryingPts(){ constPts3D = false; }

    std::vector<std::pair<int,int>> getVaryingPairsCamParam() {
        std::vector<std::pair<int,int>> varyingPars;
        for (auto c = 0; c < m_constCams.size(); ++c) {
            for (auto i = 0; i < p3dBundleIdx_TOTAL; ++i) {
                bool isConst = m_constCams[c] & (1 << i);
                if (!isConst)
                    varyingPars.push_back({c,i});
            }
        }
        return varyingPars;
    }

private:
    void setParam(unsigned int param, std::vector<int> cams, bool isConst){
        if (cams.empty())
            for (auto i = 0; i < m_constCams.size(); ++i) cams.push_back(i);

        for (const auto & i : cams) {
            if (isConst) m_constCams[i] |= param;
            else         m_constCams[i] &= ~param;
        }
    }

    std::vector<unsigned int> m_constCams{};
    bool constPts3D{false};
};
}  // namespace p3d
