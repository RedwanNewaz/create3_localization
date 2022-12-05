import math
import numpy as np


class ExtendedKalmanFilter:

  def __init__(self) -> None:
    # Covariance for EKF simulation
    self.Q = np.diag([
      0.1,  # variance of location on x-axis
      0.1,  # variance of location on y-axis
      np.deg2rad(1.0),  # variance of yaw angle
      1.0  # variance of velocity
    ])**2  # predict state covariance
    self.R = np.diag(
      [1.0, 1.0,
       0.0])**2  # Observation x,y, theta -> position & orientation covariance
    self.DT = 0.15  # time tick [s]

  @staticmethod
  def motion_model(x, u, dt):
    F = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0,
                                                                   0]])

    B = np.array([[dt * math.cos(x[2, 0]), 0], [dt * math.sin(x[2, 0]), 0],
                  [0.0, dt], [1.0, 0.0]])

    x = F @ x + B @ u

    return x

  @staticmethod
  def observation_model(x):
    H = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])

    z = H @ x

    return z

  @staticmethod
  def __jacob_f(x, u, dt):
    """
        Jacobian of Motion Model

        motion model
        x_{t+1} = x_t+v*dt*cos(yaw)
        y_{t+1} = y_t+v*dt*sin(yaw)
        yaw_{t+1} = yaw_t+omega*dt
        v_{t+1} = v{t}
        so
        dx/dyaw = -v*dt*sin(yaw)
        dx/dv = dt*cos(yaw)
        dy/dyaw = v*dt*cos(yaw)
        dy/dv = dt*sin(yaw)
        """
    yaw = x[2, 0]
    v = u[0, 0]
    jF = np.array([[1.0, 0.0, -dt * v * math.sin(yaw), dt * math.cos(yaw)],
                   [0.0, 1.0, dt * v * math.cos(yaw), dt * math.sin(yaw)],
                   [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])

    return jF

  @staticmethod
  def __jacob_h():
    # Jacobian of Observation Model
    jH = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])

    return jH

  def estimation(self, xEst, PEst, z, u):
    #  Predict
    xPred = self.motion_model(xEst, u, self.DT)
    jF = self.__jacob_f(xEst, u, self.DT)
    PPred = jF @ PEst @ jF.T + self.Q

    #  Update
    jH = self.__jacob_h()
    zPred = self.observation_model(xPred)
    y = z - zPred
    S = jH @ PPred @ jH.T + self.R
    K = PPred @ jH.T @ np.linalg.inv(S)
    xEst = xPred + K @ y
    PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
    return xEst, PEst
