from RobotArm import*
from GreenPaper import*

class GreenArena ( ):
    def __init__(self, *args, **kw):
        RobotSimInterface.__init__(self, *args, **kw, azimuthIn,skewIn, inclineIn, lengthIn, widthIn)
        '''
        might we want to initialize these to be pointing in the positive y direction?
        '''
        self.arm = GreenRobotArm()
        self.paper = GreenPaper(azimuthIn,skewIn, inclineIn, lengthIn, widthIn)


