#include "ThrowingMode.h"

ThrowingMode::ThrowingMode(float topTargetSpeed, float bottomTargetSpeed) :
    topTargetSpeed(topTargetSpeed),
    bottomTargetSpeed(bottomTargetSpeed) {}

void ThrowingMode::setSpeed(float topTargetSpeed, float bottomTargetSpeed)
{
    this->topTargetSpeed = topTargetSpeed;
    this->bottomTargetSpeed = bottomTargetSpeed;
}

void ThrowingMode::setSpeed(ThrowingMode throwingMode)
{
    this->topTargetSpeed = throwingMode.getTopTargetSpeed();
    this->bottomTargetSpeed = throwingMode.getBottomTargetSpeed();
}

float ThrowingMode::getTopTargetSpeed()
{
    return topTargetSpeed;
}

float ThrowingMode::getBottomTargetSpeed()
{
    return bottomTargetSpeed;
}

// string ThrowingMode::toString() const
// {
//     stringstream ss;
//     ss << "Top Target Speed: " << topTargetSpeed << ", Bottom Target Speed: " << bottomTargetSpeed;
//     return ss.str();
// }
