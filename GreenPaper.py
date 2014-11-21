class GreenPaper( ):
    def __init__(self, *args, **kw):
        RobotSimInterface.__init__(self, *args, **kw, azimuthIn,skewIn, inclineIn, lengthIn, widthIn)
        '''
        An object oriented implementation of a peice of paper that exists within an arena
        for the purposes of the EECS 498 Project 2 task
        '''
        self.length = lengthIn
        self.width = widthIn
        self.azimuth = azimuthIn
        self.inclination= inclineIn
        self.skew= skewIn #skew is angle that the paper's long axis is  

    def getLength():
        return self.length

    def getWidth():
        return self.width

    def getAzimuth():
        return self.azimuth

    def getSkew():
        return self.skew

    
    #This one might be tricky. calculate the corner positions of the paper based 
    #solely on the paper's dimensions and angles of orientation. 
    def getCorners():




