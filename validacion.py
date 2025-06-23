import pygame
import numpy as np
import tensorflow as tf
import math

# Inicializar pygame
pygame.init()
screen = pygame.display.set_mode((600, 600))
pygame.display.set_caption("Péndulo controlado por red neuronal")

# Parámetros del péndulo
length = 200  # Longitud en píxeles
mass = 0.25   # kg
g = 9.81
I = mass * (length / 1000)**2  # Inercia (convertimos a metros)
dt = 0.02  # paso de tiempo
torque_max = 0.3  # Nm

# Inicialización (θ = 0 arriba)
theta = np.pi
omega = 0.0

# Cargar modelo entrenado
trained_model = tf.keras.models.load_model("TrainedModel.keras")

# Centro de rotación
center = (300, 300)

running = True
perturb = False

clock = pygame.time.Clock()

while running:
    screen.fill((255, 255, 255))

    # Eventos
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
            perturb = True

    # Calcular torque desde red neuronal
    input_data = np.array([[theta, omega]])
    tau = float(trained_model.predict(input_data, verbose=0))

    # Saturar torque si es necesario
    tau = np.clip(tau, -torque_max, torque_max)

    # Perturbación si se presiona espacio
    if perturb:
        tau += 5  # pequeña perturbación
        perturb = False

    # Dinámica del péndulo
    alpha = (tau - mass * g * (length / 1000) * np.sin(theta)) / I
    omega += alpha * dt
    theta += omega * dt

    # Dibujar péndulo
    x = center[0] + length * math.sin(theta)
    y = center[1] - length * math.cos(theta)  # restar porque Y crece hacia abajo en pantalla

    pygame.draw.line(screen, (0, 0, 0), center, (x, y), 5)
    pygame.draw.circle(screen, (0, 0, 255), (int(x), int(y)), 10)
    print(tau)

    pygame.display.flip()
    clock.tick(60)
