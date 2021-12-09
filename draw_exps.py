import pygame


if __name__ == '__main__':
  BLACK = (0,0,0)
  WHITE = (255,255,255)
  draw_size = 3

  pygame.init()

  screen = pygame.display.set_mode((800, 800))
  screen.fill(WHITE)

  done = False
  clock = pygame.time.Clock()

  while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    
    mouse_state = pygame.mouse.get_pressed()[0]
    mouse_x = pygame.mouse.get_pos()[0]
    mouse_y = pygame.mouse.get_pos()[1]

    if mouse_state == 1:
      pygame.draw.line(screen, BLACK, (past_mouse_x, past_mouse_y), (mouse_x, mouse_y), draw_size)
    
    past_mouse_x = mouse_x
    past_mouse_y = mouse_y

    pygame.display.flip()
    clock.tick(60)
  
  pygame.quit()