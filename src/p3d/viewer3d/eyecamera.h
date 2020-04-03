#pragma once

#include "Eigen/Dense"

class EyeCamera
{
public:
    EyeCamera();
    ~EyeCamera();

    void resetParams();
    Eigen::Matrix4f getWorldToViewMatrix() const;

    void moveForward();
    void moveBackward();
    void strafeLeft();
    void strafeRight();
    void moveUp();
    void moveDown();

    void moveBackForward(float step);
    void strafeLeftRight(float step);
    void moveUpDown(float step);
    void rotate(float dr, float dphi);

    void lookAtZ();
    void lookAtX();

    Eigen::Vector3f getPosition() const { return position; }
    float getDistance() const { return distance; }

    float getVertAngleDeg() const { return vertAngleDeg; }
    void multiplyVertAngle(float coeff);
    void setAspectRatio(float value) { aspect = value; }
    Eigen::Matrix4f getProjMatrix() const;

    const Eigen::Vector3f& getUpAxis() const { return upAxis; }

    const Eigen::Vector3f& getViewAxis() const;
    void setViewAxis(const Eigen::Vector3f& value);

private:
    Eigen::Matrix4f perspective(float verticalAngle, float aspectRatio,
                                float nearPlane, float farPlane) const;

    // Position and orientation
    Eigen::Quaternionf rotQuat;
    Eigen::Vector3f position{.0, .0, .0};
    Eigen::Vector3f upAxis{0.0, 0.0, 1.0};
    Eigen::Vector3f viewAxis{0.0, 0.0, 1.0};
    float distance{-10000.0};

    const float movementStep{0.05f};

    float vertAngleDeg{45.0f};
    float aspect{1.0f};
    float nearClip{0.1f};
    float farClip{1000.0f};
};
