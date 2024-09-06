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

    try:
        with open(pathLocation) as file:
            data = json.load(file)
    except FileNotFoundError:
        print(f"Couldnt find file {pathLocation}")
        return None

    return data

def getStartingPos(pathName : str) -> dict[str]:
    pathData = loadPath(pathName)

    return {
        "x" : pathData['waypoints'][0]['anchor']['x'],
        "y" : pathData['waypoints'][0]['anchor']['y']
    }

def getCurveFromPath(pathName : str, mul=1.0):
    pathData = loadPath(pathName)

    if pathData is None:
        return

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
    def __init__(self, master, on_delete, labelIn : str, add_command=None, destroyable=True) -> None:
        super().__init__(master)

        self.label = labelIn

        self.pack(fill="x", padx=5, pady=5)

        # Add header with delete button
        header_frame = ctk.CTkFrame(self)
        header_frame.pack(fill="x")

        if destroyable:
            delete_button = ctk.CTkButton(header_frame, text="X", width=20, command=self.delete)
            delete_button.pack(side="right", padx=10)

        header_label = ctk.CTkLabel(header_frame, text=self.label)
        header_label.pack(side="left", padx=10, pady=5)

        self.on_delete = on_delete

    def delete(self):
        self.destroy()
        self.on_delete(self)

class Path(Item):
    def __init__(self, master, on_delete, labelIn: str, path_name : str, conversion_factor=1.0) -> None:
        super().__init__(master, on_delete, labelIn)
        
        self.path_name = path_name

        self.points = getCurveFromPath(self.path_name, mul=conversion_factor)

    def toJSON(self) -> dict[str]:
        return {
            "type" : "path",
            "name" : self.path_name
        }

class MenuGroup(Item):
    def __init__(self, master, on_delete, labelIn: str, add_path, destroyable=True) -> None:
        super().__init__(master, on_delete, labelIn, destroyable=destroyable)

        self.items = []

        self.add_button = ctk.CTkButton(self, text="Add", command=self.show_add_menu)
        self.add_button.pack(pady=5)

        # Context menu for adding paths or conditionals
        self.add_menu = Menu(self, tearoff=0)
        self.add_menu.add_command(label="Add Path", command=self.prompt_add_path)
        self.add_menu.add_command(label="Add Conditional Group", command=self.prompt_add_conditional)
        self.add_menu.add_command(label="Add Sequential Group", command=self.add_sequential)
        self.add_menu.add_command(label="Add Parallel Group", command=self.add_parallel)
        self.add_menu.add_command(label="Add Deadline Group", command=self.add_deadline)
        self.add_menu.add_command(label="Add Race Group", command=self.add_race)

        self.add_path = add_path
        self.on_delete = on_delete

    def show_add_menu(self):
        # Show dropdown menu when add button is clicked
        self.add_menu.post(self.add_button.winfo_rootx(), self.add_button.winfo_rooty() + self.add_button.winfo_height())

    def prompt_add_path(self):
        path_name = simpledialog.askstring("Add Path", "Enter path name:")
        if path_name:
            self.add_path_to_group(path_name)

    def prompt_add_conditional(self):
        condition = simpledialog.askstring("Add Condition", "Enter condition name:")
        if condition:
            self.add_conditional_to_group(condition)

    def add_path_to_group(self, path_name):
        path_item = self.add_path(path_name, master=self)
        self.items.append(path_item)

    def add_conditional_to_group(self, condition_name):
        conditional_group = ConditionalGroup(self, self.on_delete, "Conditional Group", condition_name, self.add_path)
        conditional_group.pack(fill="x")
        self.items.append(conditional_group)

    def add_sequential(self):
        self.items.append(SequentialGroup(self, self.on_delete, self.add_path))

    def add_parallel(self):
        self.items.append(ParallelGroup(self, self.on_delete, self.add_path))

    def add_deadline(self):
        self.items.append(DeadlineGroup(self, self.on_delete, self.add_path))

    def add_race(self):
        self.items.append(RaceGroup(self, self.on_delete, self.add_path))

    def delete(self) -> dict[str]:
        # Delete Items
        for item in self.items:
            item.delete()
            app.items.remove(item)
        super().delete()

        self.on_delete(self)

class SequentialGroup(MenuGroup):
    def __init__(self, master, on_delete, add_path, destroyable=True) -> None:
        super().__init__(master, on_delete, "Sequential", add_path, destroyable=destroyable)

    def toJSON(self) -> dict[str]:
        return {
            "type" : "sequential",
            "commands" : [item.toJSON() for item in self.items]
        }

class ParallelGroup(MenuGroup):
    def __init__(self, master, on_delete, add_path) -> None:
        super().__init__(master, on_delete, "Parallel", add_path)

    def toJSON(self) -> dict[str]:
        return {
            "type" : "parallel",
            "commands" : [item.toJSON() for item in self.items]
        }

class RaceGroup(MenuGroup):
    def __init__(self, master, on_delete, add_path) -> None:
        super().__init__(master, on_delete, "Race", add_path)

    def toJSON(self) -> dict[str]:
        return {
            "type" : "race",
            "commands" : [item.toJSON() for item in self.items]
        }

class DeadlineGroup(MenuGroup):
    def __init__(self, master, on_delete, add_path) -> None:
        super().__init__(master, on_delete, "Deadline", add_path)

    def toJSON(self) -> dict[str]:
        return {
            "type" : "deadline",
            "commands" : [item.toJSON() for item in self.items]
        }

class ConditionalGroup(MenuGroup):
    def __init__(self, master, on_delete, labelIn: str, condition_name: str, add_path) -> None:
        super().__init__(master, on_delete, labelIn, add_path)
        
        # Add header label for the condition
        self.condition_label = ctk.CTkLabel(self, text=f"Condition: {condition_name}")
        self.condition_label.pack(pady=5)

        self.condition_name = condition_name

        self.add_path = add_path
        self.on_delete = on_delete

    def toJSON(self) -> dict[str]:
        return {
            "type" : "conditional",
            "condition" : self.condition_name,
            "commands" : [item.toJSON() for item in self.items]
        }


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
        
        # Create sidebar
        self.sidebar = ctk.CTkFrame(self, width=250, corner_radius=0)
        self.sidebar.place(relx=1.0, rely=0.0, anchor="ne", relheight=1.0)

        # Create a frame to hold the list of paths and groups
        self.pathListFrame = ctk.CTkFrame(self.sidebar)  # Initialize pathListFrame before update_path()
        self.pathListFrame.pack(padx=20, pady=20, fill="both", expand=True)

        # Add button to save project to a auto file
        self.add_button = ctk.CTkButton(self.sidebar, text="Save", command=self.save)
        self.add_button.pack(padx=20, pady=10)

        self.update_path()

        self.auto_name = None
        self.commands = SequentialGroup(self.pathListFrame, self.delete_path, self.add_path, destroyable=False)

    def save(self):
        if self.auto_name is None:
            self.auto_name = simpledialog.askstring("Auto Name", "Enter auto name:")

        if self.auto_name is None:
            return

        startingPose = getStartingPos(self.commands.items[0].path_name)
        startingPose['rot'] = float(simpledialog.askstring("Starting Rotation", "Enter robot rotation:"))

        with open(f"src/main/deploy/autoplanner/autos/{self.auto_name}.auto", "w") as file:
            json.dump(
                {
                    "startingPos" : startingPose,
                    "command" : self.commands.toJSON()
                },
                file,
                indent=2
            )

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

    def update_path(self):
        # Clear previously drawn paths
        for line in self.pathPoints:
            self.canvas.delete(line)
        self.pathPoints = []  # Reset the list of path lines

        # Draw new paths
        for path in self.paths:
            pathPoints = getCurveFromPath(path, mul=self.meterToPixel)

            if pathPoints is None:
                continue

            line = self.canvas.create_line(pathPoints, fill="blue", smooth=True, width=2)
            self.pathPoints.append(line)
    
if __name__ == "__main__":
    app = AutoPlannerApp()
    app.mainloop()
