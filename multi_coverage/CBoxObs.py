import numpy as np

class CBoxObs():
    # Common base class for box obstacles
    def __init__(obj, pr, obsID):
        obj.dim_ = pr.dim
        obj.id_ = obsID
        obj.pos_real_ = np.zeros((obj.dim_, 1))
        obj.size_ = np.zeros((obj.dim_, 1))
        obj.yaw_ = 0
