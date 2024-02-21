import time
import os

def clear_console():
    os.system('cls' if os.name == 'nt' else 'clear')

def animate_sandclock():
    frames = ["   --\n  \\  /\n   --", "   --\n  /  \\\n   --"]  # Two simple frames to simulate a sandclock
    while True:  # You should replace this with a condition that checks if your code is still running
        for frame in frames:
            clear_console()  # Clear the console to draw the next frame
            print(frame)
            time.sleep(0.5)  # Adjust the speed of the animation

# Example usage, simulating a long-running process
try:
    animate_sandclock()
except KeyboardInterrupt:  # Stops the animation when you interrupt the program (e.g., by pressing Ctrl+C)
    pass