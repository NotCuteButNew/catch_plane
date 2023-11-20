import time
from ArmIK.ArmMoveIK import *

if __name__ == "__main__":
    AK = ArmIK()
    setBusServoPulse(1,500,800)
    time.sleep(1)
    AK.setPitchRangeMoving((15, 0, 12), 0, -90, 0, 3000)
    time.sleep(3)
    AK.setPitchRangeMoving((25, -10, 12), 0, -90, 0, 3000)
    time.sleep(3)
    AK.setPitchRangeMoving((30, -16, 16), 0, -90, 0, 3000)
    time.sleep(1)
    setBusServoPulse(1,200,800)
    time.sleep(1)
    AK.setPitchRangeMoving((10,5, 16), 0, -90, 0, 2000)
    time.sleep(2)
    AK.setPitchRangeMoving((0, 15, 15), 0, -90, 0, 2000)
    time.sleep(1)
