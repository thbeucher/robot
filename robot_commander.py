import pygame
import socket
import imagezmq

# Initialize Pygame
pygame.init()

# Set the size of the window
WINDOW_SIZE = (1000, 800)

# Create the window
screen = pygame.display.set_mode(WINDOW_SIZE)

# Set the title of the window
pygame.display.set_caption('Pygame Keyboard Events')

# Set up the font for the text
font = pygame.font.Font(None, 24)

# Create image receiver
image_hub = imagezmq.ImageHub()

# Set up the network connection
SERVER_IP = '192.168.1.58'  # Replace with the IP address of your Raspberry Pi
PORT = 5050
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((SERVER_IP, PORT))

# Loop until the user clicks the close button
done = False

# Start the game loop
while not done:
  # Loop over all events in the event queue
  for event in pygame.event.get():
    # Check if the event is a key press
    if event.type == pygame.KEYDOWN:
      # Check which key was pressed
      if event.key == pygame.K_UP:
        sock.sendall(b'up')
      elif event.key == pygame.K_DOWN:
        sock.sendall(b'down')
      elif event.key == pygame.K_LEFT:
        sock.sendall(b'left')
      elif event.key == pygame.K_RIGHT:
        sock.sendall(b'right')
      elif event.key == pygame.K_SPACE:
        sock.sendall(b'space')
      elif event.key == pygame.K_ESCAPE:
        done = True
      elif event.key == pygame.K_z:
        sock.sendall(b'z')
      elif event.key == pygame.K_s:
        sock.sendall(b's')
      elif event.key == pygame.K_q:
        sock.sendall(b'q')
      elif event.key == pygame.K_d:
        sock.sendall(b'd')
      elif event.key == pygame.K_p:
        sock.sendall(b'p')  # claw squeeze
      elif event.key == pygame.K_o:
        sock.sendall(b'o')  # claw unsqueeze
      elif event.key == pygame.K_l:
        sock.sendall(b'l')  # claw left rotation
      elif event.key == pygame.K_m:
        sock.sendall(b'm')  # claw right rotation
      elif event.key == pygame.K_u:
        sock.sendall(b'u')  # claw base up
      elif event.key == pygame.K_i:
        sock.sendall(b'i')  # claw base down
      elif event.key == pygame.K_j:
        sock.sendall(b'j')  # claw head up
      elif event.key == pygame.K_k:
        sock.sendall(b'k')  # claw head down

  # Fill the background with white
  screen.fill((255, 255, 255))
  # <- x -> | y = vertical

  # Render the text onto a surface
  text_surface = font.render('Camera control:', True, (0, 0, 0))
  text_position = text_surface.get_rect()  # Get the position of the text surface
  text_position.centerx = 70
  text_position.centery = 20
  screen.blit(text_surface, text_position)  # Blit the text surface onto the window surface
  
  # Draw the cross for zqsd camera command
  pygame.draw.line(screen, (0, 0, 0), (60, 50), (60, 90), 2)  # |
  pygame.draw.line(screen, (0, 0, 0), (40, 70), (80, 70), 2)  # -
  z_text = font.render('z', True, (0, 0, 0))
  s_text = font.render('s', True, (0, 0, 0))
  q_text = font.render('q', True, (0, 0, 0))
  d_text = font.render('d', True, (0, 0, 0))
  screen.blit(z_text, (57, 30))
  screen.blit(s_text, (57, 95))
  screen.blit(q_text, (20, 62))
  screen.blit(d_text, (90, 62))

  # Receive image from imageHub
  rpi_name, frame = image_hub.recv_image()
  image_hub.send_reply(b'OK')
  frame_surface = pygame.surfarray.make_surface(frame)  # Convert frame to Pygame surface
  frame_surface = pygame.transform.rotate(frame_surface, -90)
  frame_width, frame_height = frame_surface.get_rect().size  # Get frame dimensions
  # Calculate frame position in the center of the window
  frame_x = (WINDOW_SIZE[0] - frame_width) // 2
  frame_y = (WINDOW_SIZE[1] - frame_height) // 2
  screen.blit(frame_surface, (frame_x, frame_y))  # Draw frame on window

  # Update the screen
  pygame.display.flip()

# Close the network connection and quit Pygame
sock.close()
pygame.quit()
image_hub.close()
