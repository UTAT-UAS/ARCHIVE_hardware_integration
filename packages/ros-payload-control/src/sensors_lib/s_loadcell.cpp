#include <payload_control.hpp>

void PayloadControl::LoadCellSetup()
{
    const long LOADCELL_DIVIDER = -2220;

    loadcell_.begin();
    loadcell_.setScale(LOADCELL_DIVIDER);
}

void PayloadControl::LoadCellRead()
{
    static bool firstTime = true;
    static byte initialCount = 0;
    if (loadcell_.update()) {
        if (firstTime) {
            loadCellOffset_ = Tare(5); // tare load cell
            firstTime = false;
        } else {
            rawWeight_ = loadcell_.getUnits() - loadCellOffset_; // read weight
            weight_ = weightAlpha_ * rawWeight_ + (1 - weightAlpha_) * weight_;
        }
    } else {
        if (loadCellTweakCount_ >= 5) {
            loadCellIsTweaking_ = true;
        }
        loadCellTweakCount_++;
    }
}

float PayloadControl::Tare(int32_t num)
{
    int offset = 0;
    for (int i = 0; i < num; i++) {
        offset += loadcell_.getUnits();
    }
    return offset / num;
}