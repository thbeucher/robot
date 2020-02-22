import math as m


def compute_x_y(l1=105, l2=95, l3=30, l4=160, servo1=None, servo2=None):
  '''
  Direct Geometric Model
  Computes the coordinates of the effective point of the robot based on given servos angle

  x and y are given with an origin on the ground below the arm of the robot
  x axe is vertical and point to the sky
  y axe is horizontal and point to the front of the robot
  '''
  ra = input('Angle SERVO1 : ') if servo1 is None else servo1
  rb = input('Angle SERVO2 : ') if servo2 is None else servo2
  ra, rb = int(ra), int(rb)

  theta = m.atan(l4/l3)
  u = m.sqrt(m.pow(l3, 2) + m.pow(l4, 2))

  alpha = m.pi/2 - m.radians(ra)
  beta = m.radians(-rb)

  epsilon = theta + beta

  x = l1 + l2*m.cos(alpha) + u*m.cos(alpha + epsilon)
  y = l2*m.sin(alpha) + u*m.sin(alpha + epsilon)

  return int(round(x)), int(round(y))


def test_compute_x_y():
  x, y = compute_x_y(servo1=90, servo2=0)
  print(f'Expected : x = 230 | y = 160\nReceived : x = {x} | y = {y}\n')
  
  x, y = compute_x_y(servo1=90, servo2=90)
  print(f'Expected : x = 360 | y = -30\nReceived : x = {x} | y = {y}\n')
  
  x, y = compute_x_y(servo1=180, servo2=0)
  print(f'Expected : x = 265 | y = -125\nReceived : x = {x} | y = {y}')


def compute_a_b(l1=105, l2=95, l3=30, l4=160, x=None, y=None, elbow='down'):
  '''
  Inverse Geometric Model
  Computes servos angle needed to reach the given coordinates
  '''
  x = input('Coordonnee x : ') if x is None else x
  y = input('Coordonnee y : ') if y is None else y
  x, y = int(x), int(y)

  theta = m.atan(l4/l3)
  u = m.sqrt(m.pow(l3, 2) + m.pow(l4, 2))

  epsilon = m.acos((m.pow(x-l1, 2) + m.pow(y, 2) - m.pow(l2, 2) - m.pow(u, 2)) / (2*l2*u))
  
  epsilon = -epsilon if elbow == 'up' else epsilon

  k1 = l2 + u*m.cos(epsilon)
  k2 = u*m.sin(epsilon)

  alpha = m.atan((k1*y - k2*(x - l1)) / (k1*(x - l1) + k2*y))

  alpha = m.pi/2 - alpha
  beta = theta - epsilon

  return int(round(m.degrees(alpha))), int(round(m.degrees(beta)))


def test_compute_a_b():
  a, b = compute_a_b(x=230, y=160, elbow='down')
  print(f'Expected : a = 90 | b = 0\nReceived : a = {a} | b = {b}\n')
  
  a, b = compute_a_b(x=360, y=-30, elbow='up')
  print(f'Expected : a = 90 | b = 90\nReceived : a = {a} | b = {b}\n')


if __name__ == "__main__":
  rep = input('Compute arm position? (y or n): ')
  if rep == 'y':
    test_compute_x_y()
  
  rep = input('Compute servos angle? (y or n): ')
  if rep == 'y':
    test_compute_a_b()