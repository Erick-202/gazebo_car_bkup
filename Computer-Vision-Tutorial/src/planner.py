import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from std_msgs.msg import String
import pygame
import time
import numpy as np

#awsd
command_pub = rospy.Publisher('/motor_commands', String, queue_size=10)
bridge = CvBridge()
pygame.init()
pygame.joystick.init()
current_command = "STOP"
control_connected = False

"""
[
[0,0,0],[255,255,255],[100,100,100]
[0,0,0],[255,255,255],[100,100,100]
[0,0,0],[255,255,255],[100,100,100]
]

"""

sistema_armado = False

thr = 60  #-----------> THRESH VALUE

#--> Dimensiones ROI
cam_h = 800
cam_w = 800
roi_h1, roi_w1 = 450, 0
roi_h2, roi_w2 = 800 , 800

MaskLI = np.array([36, 79, 72], np.uint8)
MaskLS = np.array([69, 231, 172], np.uint8)

angulo_actual = 0

pendiente = "NULL" #CRECIENTE o DECRECIENTE


# Tener al menos un controlador conectado
if pygame.joystick.get_count() == 0:
      print("No se detecto Control.")
      #pygame.quit()
else:
    print("Control Detectado")
    control_connected = True


#publicar comandos en base a una interrupcion
def setCommand(key):
  global current_command, sistema_armado

  #VALORES EN ASCII
  if key == 105 or key == 7 :  # 'i' or '7', RB
    command = "GO"

  elif key == 108 or key == 5:  # 'l' or '5', RU
    command = "RIGHT"

  elif key == 106 or key == 4 :  # 'j' or '4', LU
    command = "LEFT"

  elif key == 107 or key == 6 :  # 'k' or '6', LB
    command = "BACK"

  elif key == 32 or key == 1 :  # 'espacio' or '1', circulo
    command = "STOP"

  elif key == 97 or key == 0:  # 'a' or '0', X

    #PODER intercambiar entre armado y desarmado
    if sistema_armado:
      print('DES - armado')
      sistema_armado = False

    else:
      print('armado')
      sistema_armado = True

    #NOTA: Si presiono con el control se mandan muchas senales en pocos segundos por eso el time.sleep
    if control_connected:
      time.sleep(0.2)

    command = current_command



  if command != current_command:
    current_command = command
    print ("comando :" , command)
    command_pub.publish(command)
    sistema_armado = False





def imgCallback(data):
  global control_connected, sistema_armado, thr, roi_h1, roi_w1, roi_h2, roi_w2,\
    MaskLI, MaskLS, angulo_actual, current_command, pendiente

  ##  IMAGENES --------------------------------------
  cv_image = bridge.imgmsg_to_cv2(data,'bgr8')

  #print(cv_image)

  roi = cv_image[roi_h1:roi_h2, roi_w1:roi_h2]


  roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
  green_mask = cv2.inRange(roi_hsv,MaskLI,MaskLS)

  roi_not_green = cv2.bitwise_not(green_mask)

  roi_final = cv2.bitwise_and(roi,roi, mask=roi_not_green)

  gray_roi = cv2.cvtColor(roi_final, cv2.COLOR_BGR2GRAY)
  ret, thresh = cv2.threshold(gray_roi, thr, 255, cv2.THRESH_BINARY)

  filtered_result = cv2.medianBlur(thresh, ksize=5)

  #obtener las dimesiones de la camra
  #cam_h, cam_w = cv_image.shape[0:2]
  #print('h: ', cam_h, 'w: ', cam_w)
  #---------------------------------------------------------------

  key = cv2.waitKey(1)

  if key != -1:
    setCommand(key)


  # Tener al menos un controlador conectado
  if control_connected == True:
      joystick = pygame.joystick.Joystick(0)
      joystick.init()

      for event in pygame.event.get():
        if event.type == pygame.JOYBUTTONDOWN:
          # Obtener el codigo del boton presionado
          button = event.button

          setCommand(button)

  if sistema_armado == True:
    cv2.putText(cv_image,'ARMADO',(20,30),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),3)
    #current_command = "GO"

    cv2.putText(cv_image, str(current_command), (250, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3)

    cv2.putText(cv_image, str(angulo_actual), (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

    cv2.putText(cv_image, pendiente, (550, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 3)

    angulo_actual = 0

    ##############################   ATUTONOMOUS DRIVE   ###########################


    #contornos en la imagen umbralizada
    _, contours, _ = cv2.findContours(filtered_result, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
      # Encuentra el contorno mas grande
      largest_contour = max(contours, key=cv2.contourArea)

      # Ajusta una linea recta que mejor se ajusta al contorno
      [vx, vy, x, y] = cv2.fitLine(largest_contour, cv2.DIST_L2, 0, 0.01, 0.01)

      print (vx,'   ',vy,'   ',x,'  ',y)

      # Encontrar dos puntos en la linea
      point1 = (int(x - 1000 * vx), int(y - 1000 * vy) + 450)
      point2 = (int(x + 1000 * vx), int(y + 1000 * vy) + 450)

      # Dibujar la linea en la imagen
      cv2.line(cv_image, point1, point2, (255, 0, 0), 4)

      # Calcula la pendiente
      m = round((vy / vx) * 180)

      if (angulo_actual < m - 1 or angulo_actual > m + 1) and angulo_actual < 120:
        angulo_actual = m


        if point1[1] > point2[1]:
          pendiente = "CRECIENTE"

        else:
          pendiente = "DECRECIENTE"

        if angulo_actual >= 50 and angulo_actual<= 70 and current_command != "GO":
          current_command = "GO"

        elif angulo_actual < 50 and current_command != "RIGHT":
          current_command = "RIGHT"

        elif angulo_actual > 70 and current_command != "LEFT":
          current_command = "LEFT"

      else:
        current_command = "GO"

      command_pub.publish(current_command)
    








    ################################################################################

  else:
    cv2.putText(cv_image, 'DES-ARMADO', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)

  cv2.imshow('Filtered Result', filtered_result)
  cv2.imshow('roi_final', roi_final)
  cv2.imshow('thresh', thresh)
  cv2.imshow('roi', roi)
  cv2.imshow('Raw Image', cv_image)
  #cv2.waitKey(3)


def main():
  print("Hey HEY Universe!")
  rospy.init_node('my_planner_node')
  img_sub = rospy.Subscriber('/camera/image_raw', Image, imgCallback)

  rospy.spin()

if __name__ == "__main__":
  main()

