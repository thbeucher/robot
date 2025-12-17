import time
import socket
import argparse
import imagezmq

from imutils.video import VideoStream


def start_client(server_ip):
  sender = imagezmq.ImageSender(connect_to='tcp://{}:5555'.format(server_ip))
  rpi_name = socket.gethostname()

  # vs = VideoStream(usePiCamera=True).start()
  vs = VideoStream(src=0).start()
  
  time.sleep(2)

  while True:
    frame = vs.read()
    sender.send_image(rpi_name, frame)


if __name__ == '__main__':
  argparser = argparse.ArgumentParser(prog='stream_webcam_client.py', description='')
  argparser.add_argument('--server_ip', '-s', default='192.168.1.36', type=str)
  args = argparser.parse_args()

  start_client(args.server_ip)
