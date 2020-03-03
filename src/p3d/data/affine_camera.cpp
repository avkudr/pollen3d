#include "affine_camera.h"

#include "p3d/core/utils.h"

int dummyAffineCamera_ = AffineCamera::initMeta();

int AffineCamera::initMeta()
{
    static bool firstCall = true;
    if (firstCall) {
        std::cout << "Reflecting: AffineCamera" << std::endl;
        entt::meta<AffineCamera>()
            .type("AffineCamera"_hs)
            .data<&AffineCamera::setAlpha, &AffineCamera::getAlpha> (P3D_ID_TYPE(p3dAffineCamera_Alpha))
            .data<&AffineCamera::setSkew,  &AffineCamera::getSkew>  (P3D_ID_TYPE(p3dAffineCamera_Skew))
            .data<&AffineCamera::setFocal, &AffineCamera::getFocal> (P3D_ID_TYPE(p3dAffineCamera_Focal))
                ;

        SERIALIZED_ADD_READ_WRITE(AffineCamera,"AffineCamera"_hs);
        SERIALIZE_TYPE_VECS(AffineCamera,"vector_AffineCamera"_hs);
        firstCall = false;
    }
    return 0;
}

AffineCamera::AffineCamera(const Mat2 &A)
{
    m_skew  = A(0,1);
    m_focal = A(1,1);
    if (std::fabs(m_focal) < 1e-5) m_focal = 1;
    m_alpha = A(0,0) / m_focal;
    updateA();
}

AffineCamera::~AffineCamera()
{

}

bool AffineCamera::operator==(const AffineCamera &i) const{
    if (!utils::floatEq(getAlpha() ,i.getAlpha() )) return false;
    if (!utils::floatEq(getSkew()  ,i.getSkew()  )) return false;
    if (!utils::floatEq(getFocal() ,i.getFocal() )) return false;
    return true;
}


void AffineCamera::updateA(){
    m_A << m_focal * m_alpha,  m_skew,
            0, m_focal;
}

Mat3 AffineCamera::getK() const {
    Mat3 K;
    K.setIdentity();
    K.topLeftCorner(2,2) = m_A;
    return K;
}

