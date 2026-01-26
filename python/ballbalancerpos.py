import pygame
import serial


pygame.init()
screen = pygame.display.set_mode((935, 705))
clock = pygame.time.Clock()
running = True
dt = 0

arduinoData = serial.Serial('COM12', 9600, timeout=1)

class PlayerSprite(pygame.sprite.Sprite):
    def __init__(self, x, y):
        super().__init__()
        self.image = pygame.Surface((77, 77))  
        self.image.fill((128, 128, 128))  
        self.rect = self.image.get_rect(center=(x, y))

    def follow_cursor(self):
        self.rect.center = pygame.mouse.get_pos()

    def get_position(self):
        return self.rect.center

player = PlayerSprite(screen.get_width() / 2, screen.get_height() / 2)
all_sprites = pygame.sprite.Group(player)

def screen_to_custom_coords(pos, screen_width, screen_height, min_range=-512, max_range=512):
    x = (pos[0] / screen_width) * (max_range - min_range) + min_range
    y = (pos[1] / screen_height) * (max_range - min_range) + min_range
    return x, y

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill((186, 91, 6))  

    player.follow_cursor()

    all_sprites.draw(screen)

    player_pos = player.get_position()
    custom_coords = screen_to_custom_coords(player_pos, screen.get_width(), screen.get_height())
    coords_str = f"{custom_coords[0]:.2f},{custom_coords[1]:.2f}\n"
    arduinoData.write(coords_str.encode())


    if arduinoData.in_waiting > 0:
        response = arduinoData.readline().decode().strip()
        print(f"Arduino says: {response}")

    pygame.display.flip()  
    dt = clock.tick(60) / 1000 

arduinoData.close()
pygame.quit()
