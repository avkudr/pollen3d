#pragma once

#include <vector>

enum Tab {
    Tab_General = 0,
    Tab_Image,
    Tab_Stereo,
    Tab_Multiview,
    Tab_PointCloud,
    Tab_COUNT
};

enum Section_ {
    Section_Default = 0,
    Section_Matches = Section_Default,
    Section_Epilines,
    Section_Rectified,
    Section_DisparityMap,
};

class AppState {
public:
    AppState() {}

    void reset() {
        m_updateTexture = true;
        m_updateViewer3DFull = true;
        m_updateViewer3DVisibility = true;
    }

    bool textureNeedsUpdate() const { return m_updateTexture; }
    bool viewer3dNeedsUpdateFull() const { return m_updateViewer3DFull; }
    bool viewer3dNeedsUpdateVisibility() const { return m_updateViewer3DVisibility; }

    void setTextureNeedsUpdate(bool v) { m_updateTexture = v; }
    void setViewer3dNeedsUpdateFull(bool v) { m_updateViewer3DFull = v; }
    void setViewer3dNeedsUpdateVisibility(bool v) { m_updateViewer3DVisibility = v; }

    // ***** current section

    auto section() const { return m_currentSection; }
    void setSection(int s) { m_currentSection = s;}
    bool isSection(int s) const { return m_currentSection == s;}
    bool isSection(const std::vector<int> & s) const { return isOneOf(m_currentSection,s);}

    // ***** current tab

    auto tab() const { return m_currentTab; }
    void setTab(int t) { m_currentTab = t;}
    bool isTab(int t) const { return m_currentTab == t;}
    bool isTab(const std::vector<int> & t) const { return isOneOf(m_currentTab,t);}

    // ***** current item

    auto itemIdx() const { return m_currentItemIdx; }
    void setItemIdx(int i) { m_currentItemIdx = i;}

private:

    template <typename T>
    inline bool isOneOf(const T &v, const std::vector<T> &vec) const
    {
        for (size_t i = 0; i < vec.size(); ++i) {
            if (vec[i] == v) return true;
        }
        return false;
    }

    bool m_updateTexture{false};
    bool m_updateViewer3DFull{false};
    bool m_updateViewer3DVisibility{false};

    int m_currentSection{Section_Default};
    int m_currentTab{Tab_Image};
    int m_currentItemIdx{0};
};
