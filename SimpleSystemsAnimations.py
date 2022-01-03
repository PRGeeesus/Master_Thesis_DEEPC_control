import pygame
from pygame.constants import SCRAP_SELECTION

class PendulumAnimation():
    def __init__(self) -> None:
        self.screen_size = (720, 480)
        successes, failures = pygame.init()
        self.screen = pygame.display.set_mode((720, 480))  # Notice the tuple! It's not 2 arguments.
        self.clock = pygame.time.Clock()
        self.FPS = 30  # This variable will define how many frames we update per second.
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)
        self.BLUE = (0,0,255)
        self.cart_size = (64, 32)
        self.pole_size = (5, 64)
        self.cart_start_pt = (300, 200)
        self.cart = pygame.Rect(self.cart_start_pt, self.cart_size)  # First tuple is position, second is size.
        self.pole = pygame.Rect((self.cart_start_pt[0]+self.cart_size[0]/2,self.cart_start_pt[1]), self.pole_size)
        top_center_x = self.cart_start_pt[0]+self.cart_size[0]/2
        top_center_y = self.cart_start_pt[1]
        self.pole_vertecies = [[top_center_x-self.cart_size[0]/2,top_center_x+self.cart_size[0]/2 ]]
        self.cart_x_pos = 0

    def drawCart(self,x_pos):
        self.cart_x_pos = x_pos
        self.cart = pygame.Rect((self.cart_start_pt[0]+self.cart_x_pos,self.cart_start_pt[1]), self.cart_size)  # First tuple is position, second is size.
        #self.cart.move_ip(x_pos,0)
        pygame.draw.rect(self.screen,self.BLACK,self.cart)
    def drawPole(self,angle):
        pass
    def updateAnimation(self,x_pos,angle):
        self.clock.tick(self.FPS)
        self.screen.fill(self.WHITE)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:  # The user pressed the close button in the top corner of the window.
                quit()
        
        self.drawCart(x_pos)
        pygame.display.update()  # Or 'pygame.display.flip()'.

