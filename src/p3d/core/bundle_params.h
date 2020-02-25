#pragma once

#include <vector>

enum BundleIdx_{
    BundleIdx_Alpha  = 0, ///< fx/fy, misscaling of axis
    BundleIdx_Skew   = 1, ///< skew
    BundleIdx_Focal  = 2, ///< skew

    BundleIdx_Theta1 = 3, // theta_1
    BundleIdx_Rho    = 4, // rho
    BundleIdx_Theta2 = 5, // theta_2

    BundleIdx_t      = 6,
    BundleIdx_TOTAL  = 7
};

enum BundleParam_{
    BundleParam_Alpha  = 1 << BundleIdx_Alpha, ///< fx/fy, misscaling of axis
    BundleParam_Skew   = 1 << BundleIdx_Skew, ///< skew
    BundleParam_Focal  = 1 << BundleIdx_Focal, ///< skew
    BundleParam_K = BundleParam_Focal +
                    BundleParam_Skew +
                    BundleParam_Alpha,

    BundleParam_Theta1 = 1 << BundleIdx_Theta1, // theta_1
    BundleParam_Rho    = 1 << BundleIdx_Rho, // rho
    BundleParam_Theta2 = 1 << BundleIdx_Theta2, // theta_2
    BundleParam_R = BundleParam_Theta1 +
                    BundleParam_Rho +
                    BundleParam_Theta2,

    BundleParam_t      = 1 << BundleIdx_t,

    BundleParam_ALL = 0xFFFF
};

class BundleParams{

public:



    BundleParams() = delete;
    BundleParams(int nbCams){
        m_constCams = std::vector<unsigned int>(nbCams,0);
    }

    int getParIdx(BundleParam_ p) { return 0; }

    bool isConst(BundleParam_ param, size_t camIdx) const { return ((m_constCams[camIdx] & param) != 0);}
    bool isVarying(BundleParam_ param, size_t camIdx) const { return !isConst(param,camIdx); }

    void setConst(BundleParam_ param, std::vector<int> cams){ setParam(param,cams,true);}
    void setConstAllCams(BundleParam_ param){ setConst(param,{}); }
    void setConstAllParams(std::vector<int> cams){ setConst(BundleParam_ALL,cams); }
    void setConstAll(){ setConst(BundleParam_ALL,{}); setConstPts(); }

    void setVarying(BundleParam_ param, std::vector<int> cams){ setParam(param,cams,false);}
    void setVaryingAllCams(BundleParam_ param){ setVarying(param,{}); }
    void setVaryingAllParams(std::vector<int> cams){ setVarying(BundleParam_ALL,cams); }
    void setVaryingAll(){ setVarying(BundleParam_ALL,{}); setVaryingPts(); }

    bool isConstPts() const { return constPts3D; }
    void setConstPts(){ constPts3D = true; }
    void setVaryingPts(){ constPts3D = false; }

    std::vector<std::pair<int,int>> getVaryingPairsCamParam() {
        std::vector<std::pair<int,int>> varyingPars;
        for (auto c = 0; c < m_constCams.size(); ++c) {
            for (auto i = 0; i < BundleIdx_TOTAL; ++i) {
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
