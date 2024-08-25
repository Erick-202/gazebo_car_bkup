import cv2

#       punto - guion bajo, guion bajo version
print(cv2.__version__)
      # __version__

img = cv2.imread("/home/erick/Escritorio/water.jpeg",1)

########################

cv2.imshow("IMagen",img)

cv2.waitKey(0)

cv2.destroyAllWindows()