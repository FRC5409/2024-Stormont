import customtkinter as ctk
from tkinter import PhotoImage, Tk, Frame, Canvas, Label, Menu, simpledialog, Toplevel, Label, Entry, Button, StringVar
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

class Item(ctk.CTkFrame):
    def __init__(self, master, on_delete, labelIn : str, add_command=None) -> None:
        super().__init__(master)

        self.label = labelIn

        self.pack(fill="x", padx=5, pady=5)

        # Add header with delete button
        header_frame = ctk.CTkFrame(self)
        header_frame.pack(fill="x")

        header_label = ctk.CTkLabel(header_frame, text=self.label)
        header_label.pack(side="left", padx=10, pady=5)

        if self.label == "":
            self.add_button = ctk.CTkButton(header_frame, text="+", width=20, command=add_command)
            self.add_button.pack(side="right", padx=10)
        else:
            delete_button = ctk.CTkButton(header_frame, text="X", width=20, command=self.delete)
            delete_button.pack(side="right", padx=10)

        self.on_delete = on_delete

    def delete(self):
        self.destroy()
        self.on_delete(self)

class Path(Item):
    def __init__(self, master, on_delete, labelIn: str, path_name : str, conversion_factor=1.0) -> None:
        super().__init__(master, on_delete, labelIn)
        
        self.path_name = path_name

        self.points = getCurveFromPath(self.path_name, mul=conversion_factor)

class ConditionalGroup(Item):
    def __init__(self, master, on_delete, labelIn: str, condtion_name : str, add_path, delete_path) -> None:
        super().__init__(master, on_delete, labelIn)

        self.items = [Item(self, on_delete, "", add_command=lambda: self.add_command(0)), Item(self, on_delete, "", add_command=lambda: self.add_command(1)), Item(self, on_delete, condtion_name)]

        self.condtion_name = condtion_name

        # Create a dropdown menu for add button
        self.add_menu = Menu(self, tearoff=0)
        self.add_menu.add_command(label="Add Path", command=self.prompt_add_path)
        self.add_menu.add_command(label="Add Conditional Group", command=self.prompt_add_conditional)

        self.add_path = add_path
        self.delete_path = delete_path

        self.pressed = 0

        self.on_delete = on_delete

    def add_item(self, item: Item, index: int):
        # Clear the current items
        for existing_item in self.items:
            existing_item.destroy()
        self.items = []  # Reset the list of items
    
        # Logic to add items based on the reset conditions
        if self.pressed == 1:
            self.items.append(Item(self, self.on_delete, "", add_command=lambda: self.add_command(0)))
    
        # Add the new item
        self.items.append(item)
    
        # Conditionally add a second placeholder if needed
        if self.pressed == 0:
            self.items.append(Item(self, self.on_delete, "", add_command=lambda: self.add_command(1)))
    
        # Add the conditional name item
        self.items.append(Item(self, self.on_delete, self.condtion_name))


    def add_command(self, button_pressed):
        self.pressed = button_pressed
        self.show_add_menu()

    def show_add_menu(self):
        # Show dropdown menu when add button is clicked
        self.add_menu.post(self.items[self.pressed].add_button.winfo_rootx(), self.items[self.pressed].add_button.winfo_rooty() + self.items[self.pressed].add_button.winfo_height())

    def prompt_add_path(self):
        path_name = simpledialog.askstring("Add Path", "Enter path name:")
        if path_name:
            self.add_item(self.add_path(path_name, self), self.pressed)

    def prompt_add_conditional(self):
        condition = simpledialog.askstring("Condition Name", "Condition name:")

        if condition:
            self.add_item(self.add_conditional_group(condition, self), self.pressed)

    def add_conditional_group(self, condition : str, master):
        ConditionalGroup(master, self.delete_path, "Conditional Command", condition, self.add_path, self.delete_path)

class AutoPlannerApp(ctk.CTk):
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

        self.paths = []
        self.pathPoints = []
        
        self.update_path()

        # Create sidebar
        self.sidebar = ctk.CTkFrame(self, width=250, corner_radius=0)
        self.sidebar.place(relx=1.0, rely=0.0, anchor="ne", relheight=1.0)

        # Create a frame to hold the list of paths and groups
        self.pathListFrame = ctk.CTkFrame(self.sidebar)
        self.pathListFrame.pack(padx=20, pady=20, fill="both", expand=True)

        # Add button to add new paths or groups
        self.add_button = ctk.CTkButton(self.sidebar, text="Add", command=self.show_add_menu)
        self.add_button.pack(padx=20, pady=10)

        # Create a dropdown menu for add button
        self.add_menu = Menu(self, tearoff=0)
        self.add_menu.add_command(label="Add Path", command=self.prompt_add_path)
        self.add_menu.add_command(label="Add Conditional Group", command=self.prompt_add_conditional)

    def show_add_menu(self):
        # Show dropdown menu when add button is clicked
        self.add_menu.post(self.add_button.winfo_rootx(), self.add_button.winfo_rooty() + self.add_button.winfo_height())

    def prompt_add_path(self):
        path_name = simpledialog.askstring("Add Path", "Enter path name:")
        if path_name:
            self.add_path(path_name)

    def add_path(self, path_name : str, master=None):
        self.paths.append(path_name)
        self.update_path()

        if master is None:
            master = self.pathListFrame

        return Path(master, self.delete_path, path_name, path_name, conversion_factor=self.meterToPixel)

    def delete_path(self, item : Item):
        if isinstance(item, Path):
            self.paths.remove(item.path_name)

        self.update_path()

    def prompt_add_conditional(self):
        condition = simpledialog.askstring("Condition Name", "Condition name:")

        if condition:
            self.add_conditional_group(condition)

    def add_conditional_group(self, condition : str, master=None):
        if master is None:
            master = self.pathListFrame

        return ConditionalGroup(master, self.delete_path, "Conditional Command", condition, self.add_path, self.delete_path)

    def update_path(self):
        # Clear previously drawn paths
        for line in self.pathPoints:
            self.canvas.delete(line)
        self.pathPoints = []  # Reset the list of path lines

        # Draw new paths
        for item in self.paths:
            if isinstance(item, ConditionalGroup):
                for path in item.paths:
                    line = self.canvas.create_line(path.points, fill="blue", smooth=True, width=2)
                    self.pathPoints.append(line)
            else:
                pathPoints = getCurveFromPath(item, mul=self.meterToPixel)
                line = self.canvas.create_line(pathPoints, fill="blue", smooth=True, width=2)
                self.pathPoints.append(line)

if __name__ == "__main__":
    app = AutoPlannerApp()
    app.mainloop()
