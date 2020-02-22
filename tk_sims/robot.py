import os
import time
import math as m
import cmath as cm
import numpy as np
import tkinter as tk
import threading as th
import skimage.io as ski_io


class CommandWindow(tk.Frame):
  def __init__(self, master=None):
    super().__init__()
    self.master = master
    self.frame = tk.Frame(master=self.master, width=50, height=50)
    self.frame.pack()

    self.new_win = tk.Toplevel(self.master)
    self.robot_win = RobotSimulation(master=self.new_win)

    self.var_theta1 = tk.StringVar(self.master)
    self.L1 = tk.Label(self.master, text="Theta1/x : ")
    self.L1.pack()
    self.E1 = tk.Entry(self.master, bd=5, textvariable=self.var_theta1)
    self.E1.pack()

    self.var_theta2 = tk.StringVar(self.master)
    self.L2 = tk.Label(self.master, text="Theta2/y : ")
    self.L2.pack()
    self.E2 = tk.Entry(self.master, bd=5, textvariable=self.var_theta2)
    self.E2.pack()

    self.validate_button = tk.Button(self.master, text='Validate', command=self.call_activate)
    self.validate_button.pack()
  
  def call_activate(self):
    # self.robot_win.command_servo1(int(self.var_theta1.get()))
    # self.robot_win.command_servo2(int(self.var_theta2.get()))

    # self.get_image()

    t = th.Thread(target=self.go_to_target)
    t.start()
  
  def get_image(self):
    # save canvas to .eps (postscript) file
    self.robot_win.win.postscript(file="tmp_canvas.eps", colormode="color",
                                  width=self.robot_win.width, height=self.robot_win.height,
                                  pagewidth=self.robot_win.width-1, pageheight=self.robot_win.height-1)
    data = ski_io.imread("tmp_canvas.eps")  # read the postscript data
    ski_io.imsave("canvas_image.png", data)  # write a rasterized png file
    
    os.remove('tmp_canvas.eps')
    os.remove('canvas_image.png')
  
  def go_to_target(self):
    x = int(self.var_theta1.get())
    y = int(self.var_theta2.get())
    theta1, theta2 = self.robot_win.compute_thetas(x, y)
    self.robot_win.draw_target(x, y)
    self.smooth_move(theta1, theta2)
  
  def smooth_move(self, theta1, theta2):
    theta1_start = self.robot_win.servo1_theta
    theta2_start = self.robot_win.servo2_theta

    def move_servo1():
      s1_obj = theta1 + 1 if theta1 > theta1_start else theta1 - 1

      for theta in range(theta1_start, s1_obj, 1 if theta1 > theta1_start else -1):
        self.robot_win.command_servo1(theta)
        self.robot_win.command_servo2(theta2_start)
        time.sleep(0.01)
    
    def move_servo2():
      s2_obj = theta2 + 1 if theta2 > theta2_start else theta2 - 1

      for theta in range(theta2_start, s2_obj, 1 if theta2 > theta2_start else -1):
        self.robot_win.command_servo2(theta)
        time.sleep(0.01)

    move_servo1()
    move_servo2()

    self.robot_win.servo1_theta = theta1
    self.robot_win.servo2_theta = theta2


class RobotSimulation(tk.Frame):
  def __init__(self, master=None):
    super().__init__()
    self.master = master
    self.master.title('Simulation')
    self.pack()

    self.width = 800
    self.height = 400
    self.win = tk.Canvas(master=master, width=self.width, height=self.height)
    self.win.pack()

    self.create_robot()
    self.init_servos_vars()

    self.target = None
  
  def init_servos_vars(self):
    self.scx0 = self.ab1_x1 + self.arm_block1_width/2
    self.scy0 = self.ab1_y2 + self.char_height

    self.scx1 = self.ab1_x1 + self.arm_block1_width/2  # Servo_Center_X
    self.scy1 = self.ab1_y1  # Servo_Center_Y

    self.scx2 = self.ab2_coords[0] + self.arm_block2_width/2
    self.scy2 = self.ab2_coords[1]

    self.ab2_mi_diag = m.sqrt(m.pow(self.arm_block2_width/2, 2) + m.pow(self.arm_block2_height, 2))
    self.ab2_mi_diag_theta = m.atan(self.arm_block2_width/2/self.arm_block2_height)
    self.ab2h_ab3h = self.arm_block2_height + self.arm_block3_height

    self.servo1_theta, self.servo2_theta = 90, 90
    self.theta_rad1 = m.radians(90)

    self.mgi_arm1 = self.char_height + self.arm_block1_height
    self.mgi_arm2 = self.arm_block2_height
    self.mgi_arm3 = 30
    self.mgi_arm4 = self.arm_block4_width + 40
  
  def create_robot(self):
    self.char_width, self.char_height = 220, 55
    self.char = self.win.create_rectangle(self.width/2 - self.char_width/2, self.height - self.char_height,
                                          self.width/2 + self.char_width/2, self.height)
    self.char_x1, self.char_y1, self.char_x2, self.char_y2 = self.win.coords(self.char)
    
    self.ultrasonic_width, self.ultrasonic_height = 20, 35
    self.ultrasonic = self.win.create_rectangle(self.char_x1, self.char_y1 - self.ultrasonic_height,
                                                self.char_x1 + self.ultrasonic_width, self.char_y1)
    self.ultrasonic_x1, self.ultrasonic_y1, self.ultrasonic_x2, self.ultrasonic_y2 = self.win.coords(self.ultrasonic)
    
    self.create_arm()
  
  def create_arm(self):
    self.arm_block1_width, self.arm_block1_height = 25, 50
    self.arm_block1 = self.win.create_rectangle(self.ultrasonic_x2 + 20, self.ultrasonic_y2 - self.arm_block1_height,
                                                self.ultrasonic_x2 + 20 + self.arm_block1_width, self.ultrasonic_y2)
    self.ab1_x1, self.ab1_y1, self.ab1_x2, self.ab1_y2 = self.win.coords(self.arm_block1)

    self.arm_block2_width, self.arm_block2_height = 25, 95
    self.arm_block2 = self.win.create_polygon(self.ab1_x1, self.ab1_y1 - self.arm_block2_height,
                                              self.ab1_x1 + self.arm_block2_width, self.ab1_y1 - self.arm_block2_height,
                                              self.ab1_x1 + self.arm_block2_width, self.ab1_y1,
                                              self.ab1_x1, self.ab1_y1, fill='', outline='black')
    self.ab2_coords = self.win.coords(self.arm_block2)

    self.arm_block3_width, self.arm_block3_height = 60, 40
    self.arm_block3 = self.win.create_polygon(self.ab2_coords[0] - 27.5, self.ab2_coords[1] - self.arm_block3_height,
                                              self.ab2_coords[0] + 32.5, self.ab2_coords[1] - self.arm_block3_height,
                                              self.ab2_coords[0] + 32.5, self.ab2_coords[1],
                                              self.ab2_coords[0] - 27.5, self.ab2_coords[1], fill='', outline='black')
    self.ab3_coords = self.win.coords(self.arm_block3)

    self.arm_block4_width, self.arm_block4_height = 120, 20
    self.arm_block4 = self.win.create_polygon(self.ab3_coords[0] - self.arm_block4_width, self.ab3_coords[1],
                                              self.ab3_coords[0], self.ab3_coords[1],
                                              self.ab3_coords[0], self.ab3_coords[1] + self.arm_block4_height,
                                              self.ab3_coords[0] - self.arm_block4_width, self.ab3_coords[1] + self.arm_block4_height,
                                              fill='', outline='black')
    self.ab4_coords = self.win.coords(self.arm_block4)
  
  def get_new_coords(self, scx, scy, lengths, thetas):
    new_coords = []
    lengths = np.repeat(lengths, 2)
    thetas = np.repeat(thetas, 2)
    for i in range(len(lengths)):
      if i % 2 == 0:
        new_coords.append(scx - (lengths[i] * m.cos(thetas[i])))
      else:
        new_coords.append(scy - (lengths[i] * m.sin(thetas[i])))
    return new_coords
  
  def command_servo1(self, theta):
    self.theta_rad1 = m.radians(theta)
    self.scx2 = self.scx1 - self.arm_block2_height * m.cos(self.theta_rad1)
    self.scy2 = self.scy1 - self.arm_block2_height * m.sin(self.theta_rad1)

    x0 = self.scx1 - (self.ab2_mi_diag * m.cos(self.theta_rad1 - self.ab2_mi_diag_theta))
    y0 = self.scy1 - (self.ab2_mi_diag * m.sin(self.theta_rad1 - self.ab2_mi_diag_theta))
    x1 = self.scx1 - (self.ab2_mi_diag * m.cos(self.theta_rad1 + self.ab2_mi_diag_theta))
    y1 = self.scy1 - (self.ab2_mi_diag * m.sin(self.theta_rad1 + self.ab2_mi_diag_theta))
    x2 = self.scx1 - self.arm_block2_width/2 * m.cos(self.theta_rad1 + m.pi/2)
    y2 = self.scy1 - self.arm_block2_width/2 * m.sin(self.theta_rad1 + m.pi/2)
    x3 = self.scx1 - self.arm_block2_width/2 * m.cos(self.theta_rad1 - m.pi/2)
    y3 = self.scy1 - self.arm_block2_width/2 * m.sin(self.theta_rad1 - m.pi/2)
    self.win.coords(self.arm_block2, (x0, y0, x1, y1, x2, y2, x3, y3))
    self.ab2_coords = self.win.coords(self.arm_block2)
  
  def move_all_from_servo1(self):
    l0 = m.sqrt(m.pow(40, 2) + m.pow(self.ab2h_ab3h, 2))
    l1 = m.sqrt(m.pow(20, 2) + m.pow(self.ab2h_ab3h, 2))
    l2 = m.sqrt(m.pow(20, 2) + m.pow(self.arm_block2_height, 2))
    l3 = m.sqrt(m.pow(40, 2) + m.pow(self.arm_block2_height, 2))
    theta0 = self.theta_rad1 - m.atan(40 / self.ab2h_ab3h)
    theta1 = self.theta_rad1 + m.atan(20 / self.ab2h_ab3h)
    theta2 = self.theta_rad1 + m.atan(20 / self.arm_block2_height)
    theta3 = self.theta_rad1 - m.atan(40 / self.arm_block2_height)
    self.win.coords(self.arm_block3, self.get_new_coords(self.scx1, self.scy1, [l0, l1, l2, l3], [theta0, theta1, theta2, theta3]))

    l0 = m.sqrt(m.pow(40 + self.arm_block4_width, 2) + m.pow(self.ab2h_ab3h, 2))
    l1 = m.sqrt(m.pow(40, 2) + m.pow(self.ab2h_ab3h, 2))
    l2 = m.sqrt(m.pow(40, 2) + m.pow(self.ab2h_ab3h - self.arm_block4_height, 2))
    l3 = m.sqrt(m.pow(40 + self.arm_block4_width, 2) + m.pow(self.ab2h_ab3h - self.arm_block4_height, 2))
    theta0 = self.theta_rad1 - m.atan((40 + self.arm_block4_width) / self.ab2h_ab3h)
    theta1 = self.theta_rad1 - m.atan(40 / self.ab2h_ab3h)
    theta2 = self.theta_rad1 - m.atan(40 / (self.ab2h_ab3h - self.arm_block4_height))
    theta3 = self.theta_rad1 - m.atan((40 + self.arm_block4_width) / (self.ab2h_ab3h - self.arm_block4_height))
    self.win.coords(self.arm_block4, self.get_new_coords(self.scx1, self.scy1, [l0, l1, l2, l3], [theta0, theta1, theta2, theta3]))
    
  def command_servo2(self, theta):
    theta_rad = self.theta_rad1 + m.radians(theta) - m.pi/2

    l0 = m.sqrt(m.pow(40, 2) + m.pow(self.arm_block3_height, 2))
    l1 = m.sqrt(m.pow(20, 2) + m.pow(self.arm_block3_height, 2))
    theta0 = theta_rad - m.atan(40 / self.arm_block3_height)
    theta1 = theta_rad + m.atan(20 / self.arm_block3_height)
    theta2 = theta_rad + m.pi/2
    theta3 = theta_rad - m.pi/2
    self.win.coords(self.arm_block3, self.get_new_coords(self.scx2, self.scy2, [l0, l1, 20, 40], [theta0, theta1, theta2, theta3]))

    l0 = m.sqrt(m.pow(40 + self.arm_block4_width, 2) + m.pow(self.arm_block3_height, 2))
    l1 = m.sqrt(m.pow(40, 2) + m.pow(self.arm_block3_height, 2))
    l2 = m.sqrt(m.pow(40, 2) + m.pow(self.arm_block3_height - self.arm_block4_height, 2))
    l3 = m.sqrt(m.pow(40 + self.arm_block4_width, 2) + m.pow(self.arm_block3_height - self.arm_block4_height, 2))
    theta0 = theta_rad - m.atan((40 + self.arm_block4_width) / self.arm_block3_height)
    theta1 = theta_rad - m.atan(40 / self.arm_block3_height)
    theta2 = theta_rad - m.atan(40 / (self.arm_block3_height - self.arm_block4_height))
    theta3 = theta_rad - m.atan((40 + self.arm_block4_width) / (self.arm_block3_height - self.arm_block4_height))
    self.win.coords(self.arm_block4, self.get_new_coords(self.scx2, self.scy2, [l0, l1, l2, l3], [theta0, theta1, theta2, theta3]))
  
  def compute_thetas(self, x, y, elbow='down'):
    theta = m.atan(self.mgi_arm4/self.mgi_arm3)
    u = m.sqrt(m.pow(self.mgi_arm3, 2) + m.pow(self.mgi_arm4, 2))

    epsilon = m.acos((m.pow(x-self.mgi_arm1, 2) + m.pow(y, 2) - m.pow(self.mgi_arm2, 2) - m.pow(u, 2)) / (2*self.mgi_arm2*u))
    
    epsilon = -epsilon if elbow == 'up' else epsilon

    k1 = self.mgi_arm2 + u*m.cos(epsilon)
    k2 = u*m.sin(epsilon)

    alpha = m.atan((k1*y - k2*(x - self.mgi_arm1)) / (k1*(x - self.mgi_arm1) + k2*y))

    alpha = m.pi/2 - alpha
    beta = theta - epsilon

    return int(round(m.degrees(alpha))), int(round(m.degrees(beta))) + 90
  
  def draw_target(self, x, y, size=10):
    new_x = self.scx0 - y
    new_y = self.scy0 - x
    coords = [new_x - size, new_y - size, new_x + size, new_y + size]
    if self.target is None:
      self.target = self.win.create_oval(coords, fill='red')
    else:
      self.win.coords(self.target, coords)


if __name__ == "__main__":
  root = tk.Tk()
  app = CommandWindow(master=root)
  app.mainloop()