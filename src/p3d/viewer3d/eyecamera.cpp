#include "eyecamera.h"

#include <iostream>

EyeCamera::EyeCamera()
{
    resetParams();
}

EyeCamera::~EyeCamera()
{

}

void EyeCamera::resetParams()
{
    position = Eigen::Vector3f(0,0,distance);
    viewAxis = Eigen::Vector3f(0,0,1);
    upAxis   = Eigen::Vector3f(0,1,0);
    vertAngle = 45.0f;
}

Eigen::Matrix4f EyeCamera::getWorldToViewMatrix() const
{
    Eigen::Matrix4f mat;
    mat.setIdentity();
    mat.topRightCorner(3,1) = position;
    return mat;
}

void EyeCamera::moveForward()
{
    position += movementStep * viewAxis;
}

void EyeCamera::moveBackward()
{
    position += -movementStep * viewAxis;
}

void EyeCamera::strafeLeft()
{
    position += -movementStep * viewAxis.cross(upAxis);
}

void EyeCamera::strafeRight()
{
    position +=  movementStep * viewAxis.cross(upAxis);
}

void EyeCamera::moveUp()
{
    position += movementStep * upAxis;
}

void EyeCamera::moveDown()
{
    position += -movementStep * upAxis;
}

void EyeCamera::moveBackForward(float step)
{
    distance = step * distance;
    position(2) = distance;
    std::cout << "distance: " << position(2) << std::endl;
}

void EyeCamera::strafeLeftRight(float step)
{
    float sceneSize = 2 * distance * tan(vertAngle /180.0f * M_PI / 2.0f) * aspect;
    position += sceneSize * step * viewAxis.cross(upAxis);
}

void EyeCamera::moveUpDown(float step)
{
    float sceneSize = 2 * distance * tan(vertAngle /180.0f * M_PI / 2.0f);
    position += sceneSize * step * upAxis;
}

void EyeCamera::rotate(float angle1, float angle2)
{
    (void)(angle1);
    (void)(angle2);
}

void EyeCamera::lookAtZ()
{

}

void EyeCamera::lookAtX()
{

}

void EyeCamera::multiplyVertAngle(float coeff)
{
    float x = 2*distance*tan(vertAngle /180 * 3.1415 / 2);
    vertAngle = std::abs(coeff*vertAngle);

    distance = x / 2.0 / tan(vertAngle  /180 * 3.1415 / 2);

    position(2) = distance;
}

Eigen::Matrix4f EyeCamera::getProjMatrix() const
{
    return perspective(vertAngle, aspect, nearClip, farClip);
}

Eigen::Matrix4f EyeCamera::perspective(float verticalAngle, float aspectRatio, float nearPlane, float farPlane) const
{
    if (nearPlane == farPlane || aspectRatio == 0.0f)
        return Eigen::Matrix4f::Identity();

    Eigen::Matrix4f m;
    float radians = (verticalAngle / 2.0f) / 180.0f * M_PI;
    float sine = std::sin(radians);
    if (sine == 0.0f)
        return Eigen::Matrix4f::Identity();
    float cotan = std::cos(radians) / sine;
    float clip = farPlane - nearPlane;
    m(0,0) = cotan / aspectRatio;
    m(1,0) = 0.0f;
    m(2,0) = 0.0f;
    m(3,0) = 0.0f;
    m(0,1) = 0.0f;
    m(1,1) = cotan;
    m(2,1) = 0.0f;
    m(3,1) = 0.0f;
    m(0,2) = 0.0f;
    m(1,2) = 0.0f;
    m(2,2) = -(nearPlane + farPlane) / clip;
    m(3,2) = -(2.0f * nearPlane * farPlane) / clip;
    m(0,3) = 0.0f;
    m(1,3) = 0.0f;
    m(2,3) = -1.0f;
    m(3,3) = 0.0f;
    return m;
}
