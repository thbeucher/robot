import gym
import pygame


if __name__ == "__main__":
  env = gym.make('gym_robot_arm:robot-arm-v0')
  env.render()

  done = False
  follow_cursor = False

  while not done:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    
    mouse_state = pygame.mouse.get_pressed()[0]
    mouse_x = pygame.mouse.get_pos()[0]
    mouse_y = pygame.mouse.get_pos()[1]

    # Start follow cursor
    if env.x_eff - 10 <= mouse_x <= env.x_eff + 10 and env.y_eff - 10 <= mouse_y <= env.y_eff:
      follow_cursor = True
    
    if follow_cursor:
      env.inverse_kinematics(mouse_x, mouse_y)

    past_mouse_x = mouse_x
    past_mouse_y = mouse_y
    
    env.render(draw_target=False)
  
  env.close()