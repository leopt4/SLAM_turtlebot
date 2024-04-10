from .GFLocalization import *
from .EKF import *
from .Pose import *

class EKF_3DOFDifferentialDriveInputDisplacement(GFLocalization, EKF):
    """
    This class implements an EKF localization filter for a 3 DOF Diffenteial Drive using an input displacement motion model incorporating
    yaw measurements from the compass sensor.
    It inherits from :class:`GFLocalization.GFLocalization` to implement a localization filter, from the :class:`DR_3DOFDifferentialDrive.DR_3DOFDifferentialDrive` class and, finally, it inherits from
    :class:`EKF.EKF` to use the EKF Gaussian filter implementation for the localization.
    """
    def __init__(self, x0, P0, *args):
        """
        Constructor. Creates the list of  :class:`IndexStruct.IndexStruct` instances which is required for the automated plotting of the results.
        Then it defines the inital stawe vecto mean and covariance matrix and initializes the ancestor classes.

        :param kSteps: number of iterations of the localization loop
        :param robot: simulated robot object
        :param args: arguments to be passed to the base class constructor
        """

        # this is required for plotting
        super().__init__(x0, P0, *args)

    def f(self, xk_1, uk):
        # TODO: To be completed by the student
        posBk_1 = Pose3D(xk_1[0:3])
        xk_bar  = posBk_1.oplus(uk)
        return xk_bar

    def Jfx(self, xk_1, uk):
        # TODO: To be completed by the student
        posBk_1 = Pose3D(xk_1[0:3])
        J=posBk_1.J_1oplus(uk)
        return J

    def Jfw(self, xk_1):
        # TODO: To be completed by the student
        posBk_1 = Pose3D(xk_1[0:3])
        J = posBk_1.J_2oplus()
        return J

    def h(self, xk):  #:hm(self, xk):
        # TODO: To be completed by the student
        # Obserse the heading of the robot
        h   = xk[2,0]
        return h  # return the expected observations

    def GetInput(self):
        """

        :return: uk,Qk
        """
        # TODO: To be completed by the student
        # Get output of encoder via ReadEncoder() function
        uk_pulse, Qk     = self.robot.ReadEncoders()
        
        # Compute travel distance of 2 wheels [meter] from output of the encoder
        d_L     = uk_pulse[0, 0] * (2*np.pi*self.wheelRadius/self.robot.pulse_x_wheelTurns)
        d_R     = uk_pulse[1, 0] * (2*np.pi*self.wheelRadius/self.robot.pulse_x_wheelTurns)

        # Compute travel distance of the center point of robot between k-1 and k
        d       = (d_L + d_R) / 2.
        # Compute rotated angle of robot around the center point between k-1 and k
        delta_theta_k   = np.arctan2(d_R - d_L, self.wheelBase)

        # Compute xk from xk_1 and the travel distance and rotated angle. Got the equations from chapter 1.4.1: Odometry 
        uk              = np.array([[d],
                                    [0],
                                    [delta_theta_k]])
        
        Qk = np.diag(np.array([0.01 ** 2, 0.001 ** 2, np.deg2rad(0.1) ** 2]))  # covariance of simulated displacement noise

        noise = np.random.normal(0, np.sqrt(Qk.diagonal())).reshape((len(Qk),1))

        uk += noise

        return uk, Qk

    def GetMeasurements(self):  # override the observation model
        """

        :return: zk, Rk, Hk, Vk
        """
        # TODO: To be completed by the student
        # Read compass sensor
        zk, Rk  = self.robot.ReadCompass()

        # Raise flag got measurement
        if len(zk) != 0:
            # Compute H matrix
            ns      = len(self.xk)
            Hk      = np.zeros((1,ns))
            Hk[0,2] = 1
            # Compute V matrix
            Vk      = np.diag([1.])

            self.headingData = True
        else:
            # Compute H matrix
            ns      = len(self.xk)
            Hk      = np.zeros((0,ns))
            # Compute V matrix
            Vk      = np.zeros((0,0))
        return zk, Rk, Hk, Vk
