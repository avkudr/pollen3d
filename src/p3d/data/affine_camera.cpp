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
            .data<&AffineCamera::setTheta1,&AffineCamera::getTheta1>(P3D_ID_TYPE(p3dAffineCamera_Theta1))
            .data<&AffineCamera::setRho,   &AffineCamera::getRho>   (P3D_ID_TYPE(p3dAffineCamera_Rho))
            .data<&AffineCamera::setTheta2,&AffineCamera::getTheta2>(P3D_ID_TYPE(p3dAffineCamera_Theta2))
            .data<&AffineCamera::setTranslation,&AffineCamera::getTranslation>(P3D_ID_TYPE(p3dAffineCamera_t))
                ;

        SERIALIZED_ADD_READ_WRITE(AffineCamera,"AffineCamera"_hs);
        SERIALIZE_TYPE_VECS(AffineCamera,"vector_AffineCamera"_hs);
        firstCall = false;
    }
    return 0;
}

AffineCamera::~AffineCamera()
{

}

bool AffineCamera::operator==(const AffineCamera &i) const{
    if (!utils::floatEq(getAlpha() ,i.getAlpha() )) return false;
    if (!utils::floatEq(getSkew()  ,i.getSkew()  )) return false;
    if (!utils::floatEq(getFocal() ,i.getFocal() )) return false;
    if (!utils::floatEq(getTheta1(),i.getTheta1())) return false;
    if (!utils::floatEq(getRho()   ,i.getRho()   )) return false;
    if (!utils::floatEq(getTheta2(),i.getTheta2())) return false;
    if (!utils::floatEq(m_t[0],i.getTranslation()[0])) return false;
    if (!utils::floatEq(m_t[1],i.getTranslation()[1])) return false;
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

void AffineCamera::updateR(){
    m_Rrelative = utils::RfromEulerZYZt(m_theta1,m_rho,m_theta2);
}
