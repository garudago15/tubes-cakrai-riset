#include "Reloader.h"

Reloader::Reloader(servoKRAI *servo, DigitalIn *limit)
{
    this->limit = limit;
    this->servo = servo;
}

void Reloader::run()
{
    limitReached = limit->read();

    switch(this->state){
        case STATE_RELOADER_GOING_STANDBY_POSITION:
            {
                this->targetAngle = this->standbyPositionAngle;
                if(us_ticker_read() - reloader_last_moved > DURATION_OF_GOING_UP_OR_DOWN){
                    this->state = STATE_RELOADER_STANDBY_POSITION;
                }

                break;
            }
        case STATE_RELOADER_GOING_DOWN:
            {
                if(us_ticker_read() - reloader_last_moved > DURATION_FOR_GOING_DOWN_DELAY_MS * 1000){
                    if (POSITION_DOWN_ANGLE > this->targetAngle){
                        this->targetAngle++;
                    } else {
                        this->targetAngle = this->positionDownAngle;
                        this->state = STATE_RELOADER_DOWN;
                    }

                    reloader_last_moved = us_ticker_read();
                }

                // printf("targetAngle: %f\n", this->targetAngle);
                break;
            }
        case STATE_RELOADER_STANDBY_POSITION:
            {
                this->targetAngle = this->standbyPositionAngle;

                break;
            }
        case STATE_RELOADER_DOWN:
            {
                this->targetAngle = this->positionDownAngle;
                break;
            }
        case STATE_RELOADER_SHAKE_UP:
        {
            if(us_ticker_read() - reloader_last_moved > DURATION_FOR_GOING_DOWN_DELAY_MS * 1000){
                // SHAKE MUNDUR PERLAHAN SEBELUM DILAKUKAN GERAKAN CEPAT SUPAYA RINGNYA TIDAK BERSERAKAN
                if (this->targetAngle >= (this->standbyPositionAngle+ this->shakeDegree)){
                    this->targetAngle++;
                } else {
                    this->targetAngle = this->standbyPositionAngle + this->shakeDegree;
                    this->state = STATE_RELOADER_SHAKE_MIDDLE;
                }

                reloader_last_moved = us_ticker_read();
            }

            break;
        }

        case STATE_RELOADER_SHAKE_DOWN:
        {
            // SHAKE MAJU SECARA CEPAT SUPAYA RINGNYA RATA
            this->targetAngle = this->standbyPositionAngle;
            this->state = STATE_RELOADER_STANDBY_POSITION;
            break;
        }

    }

    this->servo->position(this->targetAngle);
    printf("Sudut: %f, limit: %d\n", this->targetAngle, this->limitReached);
}

void Reloader::toggleReloader()
{
    if(us_ticker_read() - this->reloader_last_toggle > RELOADER_TOGGLE_DELAY)
    {
        this->reloader_last_toggle = us_ticker_read();
        if((this->state == STATE_RELOADER_DOWN)|| (this->state == STATE_RELOADER_GOING_DOWN))
        {
            this->state = STATE_RELOADER_GOING_STANDBY_POSITION;
        }
        else if((this->state == STATE_RELOADER_STANDBY_POSITION) || (this->state == STATE_RELOADER_GOING_STANDBY_POSITION))
        {
            this->state = STATE_RELOADER_GOING_DOWN;
        }
    }
}


void Reloader::reset()
{
    this->state = STATE_RELOADER_GOING_DOWN;
}

void Reloader::shakeUp()
{
    if(this->state != STATE_RELOADER_STANDBY_POSITION){
        // printf("BELUM BISA SHAKE SHAKE SHAKE BANG");
        return;
    }

    this->state = STATE_RELOADER_SHAKE_UP;
}

void Reloader::shakeDown()
{
    if(this->state != STATE_RELOADER_SHAKE_MIDDLE){
        // printf("BELUM BISA SHAKE SHAKE SHAKE BANG");
        return;
    }

    this->state = STATE_RELOADER_SHAKE_DOWN;
}


// SETTER
void Reloader::setState(char state)
{
    this->state = state;
}

void Reloader::setStandbyPositionAngle(float angle)
{
    this->standbyPositionAngle = angle;
}

void Reloader::setPositionDownAngle(float angle)
{
    this->positionDownAngle = angle;
}


void Reloader::setShakeDegree(float degree)
{
    this->shakeDegree = degree;
}

// GETTER
char Reloader::getState()
{
    return this->state;
}

int Reloader::getTargetAngle()
{
    return this->targetAngle;
}

float Reloader::getStandbyPositionAngle()
{
    return this->standbyPositionAngle;
}

float Reloader::getPositionDownAngle()
{
    return this->positionDownAngle;
}

float Reloader::getShakeDegree()
{
    return this->shakeDegree;
}

bool Reloader::isLimitReached()
{
    return this->limitReached;
}
