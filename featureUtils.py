import numpy as np
import math
from fractions import Fraction
from pygame.draw import line
from scipy.odr import *
from scipy.odr import ODR
from scipy.odr.odrpack import Model, RealData

Landmarks=[]

class findFeatures:
    def __init__(self):
        # init vars
        self.EPSI = 20
        self.DEL = 20
        self.SNUM = 6
        self.PMIN = 20
        self.LMAX = 20
        self.SEED_SEGEMENTS = []
        self.LINE_SEGEMENTS = []
        self.LASERNUM = []
        self.LINE_PARAMS = None
        self.NP = len(self.LASERNUM)-1
        self.LMIN = 20  # min len of line segements
        self.LR = 0  # length of line
        self.PR = 0  # num of pinged points on line
        self.FEATURES=[]

    # distance between 2 points
    @staticmethod
    def dist_points(point1, point2):
        Px = (point1[0]-point2[0])**2
        Py = (point1[1]-point2[1])**2
        return math.sqrt(Px+Py)

    # distance from a point to a line
    def dist_point2line(self, params, point):
        A, B, C = params
        distance = abs(A*point[0]+B*point[1]+C)/math.sqrt(A**2+B**2)
        return distance

    # find two points from a line equation
    def line_2points(self, m, b):
        x = 5
        y = m*x+b
        x2 = 2000
        y2 = m*x2+b
        return [(x, y), (x2, y2)]

    # converts the general form of a line to slope intercept
    def general2Slope(self, A, B, C):
        m = -A/B
        B = -C/B
        return m, B

    # converts the slope intercept of a line to general form
    def slope2General(self, m, B):
        A, B, C = -m, 1, -B
        if A < 0:
            A, B, C = -A, -B, -C
        den_a = Fraction(A).limit_denominator(1000).as_integer_ratio()[1]
        den_c = Fraction(C).limit_denominator(1000).as_integer_ratio()[1]
        gcd = np.gcd(den_a, den_c)
        lcm = den_a * den_c / gcd

        A = A*lcm
        B = B*lcm
        C = C*lcm
        return A, B, C

    # assuming the lines always intersect
    def line_interesect_general(self, params1, params2):
        a1, b1, c1 = params1
        a2, b2, c2 = params2
        x = (c1*b2-b1*c2)/(b1*a2-a1*b2)
        y = (a1*c2-a2*c1)/(b1*a2-a1*b2)
        return x, y

    def points_21ine(self, point1, point2):
        m, b = 0, 0
        if point2[0] == point1[0]:
            pass
        else:
            m = (point2[1] - point1[1]) / (point2[0] - point1[0])
            b = point2[1] - m * point2[0]
        return m, b

    def projection_point2line(self, point, m, b):
        x, y = point
        m2 = -1 / m
        c2 = y - m2 * x
        intersection_x = -(b - c2) / (m - m2)
        intersection_y = m2 * intersection_x + c2
        return intersection_x, intersection_y

    def rob_position(self, distance, angle, robot_position):
        x = distance * math.cos(angle) + robot_position[0]
        y = -distance * math.sin(angle) + robot_position[1]
        return (int(x), int(y))

    def set_points(self, data):
        self.LASERNUM = []
        if not data:
            pass
        else:
            for point in data:
                coordinates = self.rob_position(point[0], point[1], point[2])
                self.LASERNUM.append([coordinates, point[1]])
        self.NP = len(self.LASERNUM) - 1

    # Define a function (quadratic in our case) to fit the data with.

    def linear_func(self, p, x):
        m, b = p
        return m * x + b

    def fitpoints_odr(self, laser_points):
        x = np.array([i[0][0] for i in laser_points])
        y = np.array([i[0][1] for i in laser_points])

        # create a model for fitting.
        linear_model = Model(self.linear_func)

        # create a RealData object using our initiated data from above.
        data = RealData(x, y)

        # Set up ODR with the model and data.
        odr_model = ODR(data, linear_model, beta0=[0., 0.])

        # regression model
        out = odr_model.run()
        m, b = out.beta
        return m, b

    def predictPoint(self, line_params, sensed_point, robotpos):
        m, b = self.points_21ine(robotpos, sensed_point)
        params1 = self.slope2General(m, b)
        predx, predy = self.line_interesect_general(params1, line_params)
        return predx, predy

    def find_segement(self, robot_position, break_point_ind):
        flag = True
        self.NP = max(0, self.NP)
        self.SEED_SEGEMENTS = []
        for i in range(break_point_ind, (self.NP-self.PMIN)):
            predicted_points_to_draw = []
            j = i+self.SNUM
            m, c = self.fitpoints_odr(self.LASERNUM[i:j])

            params = self.slope2General(m, c)

            for k in range(i, j):
                predicted_points = self.predictPoint(params, self.LASERNUM[k][0], robot_position)
                predicted_points_to_draw.append(predicted_points)
                d1 = self.dist_points(predicted_points, self.LASERNUM[k][0])

                if d1 > self.DEL:
                    flag = False
                    break
                d2 = self.dist_point2line(params, self.LASERNUM[k][0])

                if d2 > self.EPSI:
                    flag = False
                    break
            if flag:
                self.LINE_PARAMS = params
                return [self.LASERNUM[i:j], predicted_points_to_draw, (i, j)]
        return False

    def generate_segement(self, indicies, break_points):
        line_eq = self.LINE_PARAMS
        i, j = indicies
        # start and end of points in line segement
        PB, PF = max(break_points, i-1), min(j+1, len(self.LASERNUM)-1)

        while self.dist_point2line(line_eq, self.LASERNUM[PF][0]) < self.EPSI:
            if PF > self.NP-1:
                break
            else:
                m, b = self.fitpoints_odr(self.LASERNUM[PB:PF])
                line_eq = self.slope2General(m, b)

                POINT = self.LASERNUM[PF][0]

            PF = PF+1
            NEXTPOINT = self.LASERNUM[PF][0]
            if self.dist_points(POINT, NEXTPOINT) > self.LMAX:
                break
        PF = PF-1

        while self.dist_point2line(line_eq, self.LASERNUM[PB][0]) < self.EPSI:
            if PB < break_points:
                break
            else:
                m, b = self.fitpoints_odr(self.LASERNUM[PB:PF])
                line_eq = self.slope2General(m, b)
                POINT = self.LASERNUM[PB][0]

            PB = PB-1
            NEXTPOINT = self.LASERNUM[PB][0]
            if self.dist_points(POINT, NEXTPOINT) > self.LMAX:
                break

        PB = PB+ 1
        LR = self.dist_points(self.LASERNUM[PB][0], self.LASERNUM[PF][0])
        PR = len(self.LASERNUM[PB:PF])

        if (LR >= self.LMIN) and (PR >= self.PMIN):
            self.LINE_PARAMS = line_eq
            m, b = self.general2Slope(line_eq[0], line_eq[1], line_eq[2])
            self.two_points = self.line_2points(m, b)
            self.LINE_SEGEMENTS.append((self.LASERNUM[PB+1][0], self.LASERNUM[PF-1][0]))
            return [self.LASERNUM[PB:PF], self.two_points, (self.LASERNUM[PB+1][0], self.LASERNUM[PF-1][0]), PF, line_eq, (m, b)]
        else:
            return False

    def lineFeature2point(self):
        new_rep=[] #representation of feature

        for feature in self.FEATURES:
            projection=self.projection_point2line((0,0),feature[0][0], feature[0][1])
            new_rep.append([feature[0],feature[1],projection])
        return new_rep

def landmark_association(landmarks):
    thresh=10
    for l in landmarks:

        flag=False
        for i, Landmark in enumerate(Landmarks):
            dist=findFeatures.dist_points(l[2],Landmark[2])
            if dist < thresh:
                if not is_overlap(l[1],Landmark[1]):
                    continue
                else:
                    Landmarks.pop(i)
                    Landmarks.insert(i,l)
                    flag=True
                    
                    break
        if not flag:
            Landmarks.append(l)

def is_overlap(seg1:int,seg2:int)->bool:
    """
    >>> is_overlap
    """
    length1=findFeatures.dist_points(seg1[0],seg1[1])
    length2=findFeatures.dist_points(seg2[0],seg2[1])
    center1=((seg1[0][0]+seg1[1][0])/2, (seg1[0][1]+seg1[1][1])/2)
    center2=((seg2[0][0]+seg2[1][0])/2, (seg2[0][1]+seg2[1][1])/2)
    dist=findFeatures.dist_points(center1,center2)
    if dist > (length1+length2)/2:
        return False
    else:
        return True
    