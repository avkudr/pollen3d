#pragma once

#include "Eigen/Dense"

class EyeCamera
{
public:
    EyeCamera();
    ~EyeCamera();

    //Position and orientation

    Eigen::Quaternionf rotQuat;
    Eigen::Vector3f position;
    Eigen::Vector3f viewAxis;
    float distance{-10000.0};

    const float movementStep = 0.05f;

    void resetParams();
    Eigen::Matrix4f getWorldToViewMatrix() const;

    void moveForward();
    void moveBackward();
    void strafeLeft ();
    void strafeRight();
    void moveUp     ();
    void moveDown   ();

    void moveBackForward (float step);
    void strafeLeftRight (float step);
    void moveUpDown      (float step);
    void rotate          (float dr, float dphi);

    void lookAtZ();
    void lookAtX();

    Eigen::Vector3f getPosition() const { return position; }
    float     getDistance() const { return distance; }

    //Projection
    float vertAngle = 45.0f;
    float aspect   = 1.0f;
//    float nearClip = 0.001f;
//    float farClip = 20000.0f;

    float nearClip = 0.1f;
    float farClip = 1000.0f;

    float getVertAngle() const {return vertAngle;}
    void multiplyVertAngle(float coeff);
    void setAspectRatio(float value){ aspect = value; }
    Eigen::Matrix4f getProjMatrix() const;

    const Eigen::Vector3f & getUpAxis() const { return upAxis; }

private:
    Eigen::Vector3f   upAxis;

    Eigen::Matrix4f perspective(float verticalAngle, float aspectRatio, float nearPlane, float farPlane) const;
};
