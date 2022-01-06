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

    # assuming the lines always intersect
    def line_interesect_general(self,params1,params2):
        a1,b1,c1 = params1
        a2,b2,c2=params2
        x=(c1*b2-b1*c2)/(b1*a2-a1*b2)
        y=(a1*c2-a2*c1)/(b1*a2-a1*b2)
        return x,y

    def points_21ine (self, point1, point2):
        m, b = 0, 0
        if point2[0] == point1[0]:
            pass
        else:
            m = (point2[1] - point1[1]) / (point2 [0] - point1[0])
            b = point2[1] - m * point2[0]
        return m, b

    def projection_point2line (self, point, m, b):
        x, y = point
        m2 = -1 / m
        c2 = y - m2 * x
        intersection_x = -(b - c2) / (m - m2)
        intersection_y = m2 * intersection_x + c2
        return intersection_x, intersection_y

    def AD2pos (self, distance, angle, robot_position):
        x = distance * math.cos (angle) + robot_position[0]
        y = -distance * math.sin(angle) + robot_position[1]
        return (int(x). int(y))

    def laser_points_set(self, data):
        self.LASERPOINTS =[]
        if not data:
            pass
        else:
            for point in data:
                coordinates = self.AD2pos (point [0], point [1], point [2])
                self. LASERPOINTS. append ([coordinates, point[1]])
            self.NP = len(self. LASERPOINTS) - 1


    # Define a function (quadratic in our case) to fit the data with.
    def linear_func (self, p, x):
        m, b = p
        return m * x + b


    def odr_fit(self, laser_points):
        x = np. array ([i [0][0] for i in laser_points])
        y = np. array ([i [0][1] for i in laser_points])

        # create a model for fitting.
        linear_model = Model(self.linear_func)

        # create a RealData object using our initiated data from above.
        data= RealData(x, y)
        
        # Set up ODR with the model and data.