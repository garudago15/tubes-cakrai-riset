#ifndef __THROWING_MODE_H__
#define __THROWING_MODE_H__


using namespace std;

class ThrowingMode
{
public:
    ThrowingMode(float topTargetSpeed, float bottomTargetSpeed);

    // setter
    void setSpeed(float topTargetSpeed, float bottomTargetSpeed);
    void setSpeed(ThrowingMode throwingMode);

    // getter
    float getTopTargetSpeed();
    float getBottomTargetSpeed();

    // string toString() const;

private:
    float topTargetSpeed;
    float bottomTargetSpeed;
};

#endif // __THROWING_MODE_H__
