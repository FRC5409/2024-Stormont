import customtkinter as ctk
from tkinter import PhotoImage
from tkinter import Tk, Frame, Canvas, Label
import numpy as np
import json

FIELD_HEIGHT = 8.18

def cubic_bezier(p0, p1, p2, p3):
    # Calculate cubic Bezier curve points.

    t = np.linspace(0, 1, 100)

    p0 = np.array(p0)
    p1 = np.array(p1)
    p2 = np.array(p2)
    p3 = np.array(p3)
    return ((1 - t)**3)[:, np.newaxis] * p0 + \
           (3 * (1 - t)**2 * t)[:, np.newaxis] * p1 + \
           (3 * (1 - t) * t**2)[:, np.newaxis] * p2 + \
           (t**3)[:, np.newaxis] * p3

def loadPath(pathName : str) -> dict:
    pathLocation = f"src/main/deploy/pathplanner/paths/{pathName}.path"

    with open(pathLocation) as file:
        data = json.load(file)

    return data

def getCurveFromPath(pathName : str, mul=1.0):
    pathData = loadPath(pathName)

    waypoints = pathData['waypoints']

    paths = []

    for i in range(len(waypoints) - 1):
        p0 = (waypoints[i]['anchor']['x'], FIELD_HEIGHT - waypoints[i]['anchor']['y'])
        p1 = (waypoints[i]['nextControl']['x'], FIELD_HEIGHT - waypoints[i]['nextControl']['y']) if waypoints[i]['nextControl'] else p0
        p2 = (waypoints[i + 1]['prevControl']['x'], FIELD_HEIGHT - waypoints[i + 1]['prevControl']['y']) if waypoints[i + 1]['prevControl'] else p0
        p3 = (waypoints[i + 1]['anchor']['x'], FIELD_HEIGHT - waypoints[i + 1]['anchor']['y'])
        
        paths.append(cubic_bezier(p0, p1, p2, p3))

    points = [(x * mul, y * mul) for path in paths for x, y in path]

    return points

class ImageSliderApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        # Load image
        self.image = PhotoImage(file="autoplanner/field24.png")
        self.image = self.image.subsample(3)

        self.screenWidth = self.image.width() + 250
        self.screenHeight = self.image.height() + 50

        self.title("Auto Planner")
        self.geometry(f"{self.screenWidth}x{self.screenHeight}")

        # Create and place canvas for background image
        self.canvas = Canvas(self, width=self.screenWidth, height=self.screenHeight, bg='gray')
        self.canvas.pack(fill="both", expand=True)
        self.canvas.create_image(0, 0, anchor="nw", image=self.image)

        self.meterToPixel = self.image.height() / FIELD_HEIGHT

        self.pathPoints = getCurveFromPath("ASTART_TO_MNOTE1", mul=self.meterToPixel)
        self.canvas.create_line(self.pathPoints, fill="blue", smooth=True, width=2)

        # Create sidebar
        self.sidebar = ctk.CTkFrame(self, width=200, corner_radius=0)
        self.sidebar.place(relx=1.0, rely=0.0, anchor="ne", relheight=1.0)

        # Add a slider to the sidebar
        self.slider = ctk.CTkSlider(self.sidebar, from_=0, to=100)
        self.slider.pack(padx=20, pady=20)

        # Add label to display slider value
        self.label = ctk.CTkLabel(self.sidebar, text="Value: 0")
        self.label.pack(padx=20, pady=(0, 20))

        # Update label with slider value
        self.slider.bind("<Motion>", self.update_label)

    def update_label(self, event):
        value = self.slider.get()
        self.label.configure(text=f"Value: {value:.1f}")

if __name__ == "__main__":
    app = ImageSliderApp()
    app.mainloop()
