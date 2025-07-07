import pygame

# Initialize pygame joystick module
pygame.init()
pygame.joystick.init()

# Check for connected joysticks
if pygame.joystick.get_count() == 0:
    print("No joystick connected.")
    exit()

# Use the first connected joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Joystick name: {joystick.get_name()}")

# Main loop to read input
try:
    while True:
        pygame.event.pump()  # Process event queue

        # Read axes (assuming 4 axes)
        axes = [joystick.get_axis(i) for i in range(4)]
        print("Axes:", ["{:.2f}".format(a) for a in axes])

        # Read buttons (assuming 16 buttons)
        buttons = {"a": joystick.get_button(0), 
                   "b": joystick.get_button(1),
                   "x": joystick.get_button(2),
                   "y": joystick.get_button(3),
                   "start/stop": joystick.get_button(4)}
        
        print("Buttons:", buttons)

        pygame.time.wait(500)  # Wait 100 ms

except KeyboardInterrupt:
    print("Exiting...")

finally:
    pygame.quit()
