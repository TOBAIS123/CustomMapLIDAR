import math
import pygame
class worldEnviroment:
    def __init__(self,dim):
        pygame.init()
        self.pointCloud=[]
        self.externalMap=pygame.image.load('map.png')
        self.MapWindowName="RRT path"
        pygame.display.set_caption(self.MapWindowName)
        self.map=pygame.display.set_mode((self.mapw,self.maph))
        self.map.blt(self.externalMap,(0,0))
        self.black(0,0,0)
        self.Red=(255,0,0)