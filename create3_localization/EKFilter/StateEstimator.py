import numpy as np
from .EKFBase import ExtendedKalmanFilter
from scipy.spatial.transform import Rotation as R


class Measurement:

  def __init__(self, x, y, theta) -> None:
    self.x = x
    self.y = y
    self.theta = theta

  @classmethod
  def from_transformation_matrix(cls, mat):
    rotation = mat[:3, :3]
    translation = mat[:3, -1]
    euler_angles = R.from_matrix(rotation).as_euler('xyz', degrees=False)
    return cls(translation[0], translation[1], euler_angles[-1])

  @staticmethod
  def get_transform_from_tf_msg(p, q):
    quat = [q.x, q.y, q.z, q.w] if not isinstance(q, list) else q 
    euler_angles = R.from_quat(quat)
    transform = np.identity(4)
    transform[:3, :3] = euler_angles.as_matrix()
    transform[:3, -1] = np.array([p.x, p.y, p.z])
    return transform



  def __repr__(self) -> str:
    return f"x = {self.x:.3f}, y = {self.y:.3f}, theta = {self.theta:.3f}"

  def update(self, v, w, dt):
    self.theta += w * dt
    self.x += v * np.cos(self.theta)
    self.y += v * np.sin(self.theta)

  @property
  def ndarray(self):
    return np.array([[self.x], [self.y], [self.theta]])


class StateEstimator(ExtendedKalmanFilter):

  def __init__(self) -> None:
    super().__init__()
    self.xEst = np.zeros((4, 1))
    self.PEst = np.eye(4)

  def __call__(self, v, w, z):
    ud = np.array([[v], [w]])
    self.xEst, self.PEst = self.estimation(self.xEst, self.PEst, z.ndarray, ud)

    return np.squeeze(self.observation_model(self.xEst))

  def to_msg(self):
    position = np.squeeze(self.xEst)[:3]
    position[-1] = 0 

    quaternion = R.from_rotvec(np.array([0, 0, self.xEst[2, 0]])).as_quat()
    return position, np.squeeze(quaternion)
