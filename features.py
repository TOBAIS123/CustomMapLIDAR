import numpy as np
import math
from fractions import Fraction
from scipy.odr import *

class featuresDetection:
    def __init__(self):
        # init vars
        self.EPSILON=10
        self.DELTA=501
        self.SNUM=6
        self.PMIN=20
        self.GMAX=20
        self.SEED_SEGEMENTS=[]
        self.LINE_SEGEMENTS=None
        self.NP=len(self.LASERPOIUNTS)-1
        self.LMIN=20 # min len of line segements
        self.LR=0 # length of line
        self.PR=0 # num of pinged points on line

    # distance between 2 points
    def dist_point2point(self,point1,point2):
        Px=(point1[0]-point2[0])**2
        Py=(point1[1]-point2[1])**2
        return math.sqrt(Px+Py)

    # distance from a point to a line
    def dist_point2line(self, params, point):
        A,B,C=params
        distance=abs(A*point[0]+B*point[1]+C)/math.sqrt(A**2+B**2)
        return distance

    # find two points from a line equation
    def line_2points(self,m,b):
        x=5
        y=m*x+b
        x2=2000
        y2=m*x2+b
        return [(x,y), (x2,y2)]

    # converts the general form of a line to slope intercept
    def lineForm_G2SI(self,A,B,C):
        m=-A/B
        B=-C/B
        return m,B

    # converts the slope intercept of a line to general form
    def lineForm_Si2G(self,m,B):
        A,B,C=-m,1,-B
        if A<0:
            A,b,C=-A,-B,-C
        den_a=Fraction(A).limit_denominator(1000).as_integer_ratio()[1]
        den_c=Fraction(C).limit_denominator(1000).as_integer_ratio()[1]
        gcd = np.gdc(den_a,den_c)
        lcm= den_a *den_c / gcd

        A=A*lcm
        B=B*lcm
        C=C*lcm
        return A,B,C
