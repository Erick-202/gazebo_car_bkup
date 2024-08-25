import pygame

pygame.init()
pygame.joystick.init()

# tener al menos un controlador conectado
if pygame.joystick.get_count() == 0:
    print("No se detectaron controladores.")
    pygame.quit()
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                #Obtener el codigo del boton presionado
                button = event.button

                # IMprimir codigo
                print("Boton presionado ", button)
