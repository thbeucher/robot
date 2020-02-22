import cv2
import imutils
import imagezmq


def start_server():
  imageHub = imagezmq.ImageHub()

  while True:
    rpiName, frame = imageHub.recv_image()
    imageHub.send_reply(b'OK')
    cv2.imshow(f'Webcam {rpiName}', frame)
    key = cv2.waitKey(1) & 0xFF


if __name__ == "__main__":
  start_server()