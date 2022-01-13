import math
import pygame
class buildEnvironment:
    def __init__(self,MapDimensions):
        pygame.init()
        self.pointCloud=[]
        self.externalMap=pygame.image.load('2d-floor-plan.jpg')
        self.maph, self.mapw = MapDimensions
        self.MapWindowName = 'LIDAR'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.mapw, self.maph))
        self.map.blit(self.externalMap,(-100,-50))

    def AD2pos(self,distance,angle,robotPosition):
        x = distance * math.cos(angle)+robotPosition[0]
        y = -distance * math.sin(angle)+robotPosition[1]
        return (int(x),int(y))
        

    def show_sensorData(self):
        self.infomap=self.map.copy()
        for point in self.pointCloud:
            self.infomap.set_at((int(point[0]),int(point[1])),(255,0,0))
            pygame.draw.circle(self.infomap, (255,0,0), point, 1, 0)

    def dataStorage(self,data):
        if data!=False:
            for element in data:
                point=self.AD2pos(element[0],element[1],element[2])
                if point not in self.pointCloud:
                    self.pointCloud.append(point)





















