import numpy as np 
from scipy.spatial.transform import Rotation as R

camera_to_map_translation = [1.892, -0.726, 1.278]
camera_to_map_orientation = [0.126, 0.923, 0.317, -0.177]


camera_to_map_translation1 = [-1.615, -1.217, 2.799]
camera_to_map_orientation1 = [0.999, 0.033, -0.028, 0.005]

camera_to_map_translation2 = [1.709, -1.042, 2.483]
camera_to_map_orientation2 = [0.999, 0.025, 0.031, 0.011]

class LowpassFilter:
    def __init__(self, alpha, beta) -> None:
        self.position = None 
        self.orientation = None 
        self.alpha = alpha
        self.beta = beta 
        # camera to map 
        # self.__CtM = self.to_transform_matrix(camera_to_map_translation, camera_to_map_orientation)

    def __call__(self, *args, **kwds):

        self.position, self.orientation = self.convert(args[0], 3), self.convert(args[1], 4)
        # position, orientation = self.convert(args[0], 3), self.convert(args[1], 4)
        # camera to robot 
        # RtM = self.to_transform_matrix(position, orientation) - self.__CtM 
        # MtR = np.linalg.pinv(RtM) * self.__CtM         
        # position, orientation = self.decompose_transformation(MtR)

        # self.update(position, orientation)

    def update(self, position, orientation):
        if self.position is None or self.orientation is None:
            self.position = position
            self.orientation = orientation
            return
        self.position = self.position * self.alpha + (1 - self.alpha) * position
        self.orientation = self.orientation * self.beta + (1 - self.beta) * orientation 
    
    def convert(self, arg, param):
        if param == 4:
            return np.array([arg.x, arg.y, arg.z, arg.w])
        else:
            return np.array([arg.x, arg.y, arg.z])

    def to_transform_matrix(self, position, orientation):
        r = R.from_quat(orientation).as_matrix()
        t = np.identity(4)
        for i in range(3):
            for j in range(3):
                t[i, j] = r[i, j]
        t[0, -1], t[1, -1], t[2, -1] = position
        return t

    def decompose_transformation(self, t):
        rotation = t[:3, :3]
        translation = t[:3, -1]
        orientation = R.from_matrix(rotation).as_quat()
        return translation, orientation