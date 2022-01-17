import random
from enum import Enum
from typing import List

from pyrep.objects.shape import Shape
from pyrep.const import PrimitiveShape

from Workspace import Workspace


class Color(Enum):
    Transparent = -1
    Red = [255, 0, 0]
    Blue = [0, 0, 255]
    Yellow = [255, 255, 0]
    Purple = [255, 0, 255]
    Cyan = [0, 255, 255]
    Black = [0, 0, 0]
    White = [255, 255, 255]
    Green = [0, 255, 0]
    # Dark_Gray = [64, 64, 64]
    # Gray = [128, 128, 128]
    # Light_Gray = [172, 172, 172]
    # Magenta = [0, 172, 172]
    # Magenta2 = [172, 0, 172]
    # Orange = [255, 128, 0]
    # Lime = [128, 255, 0]


class Position:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return f"x:{self.x}, y:{self.y}, z:{self.z}"


class Box:
    def __init__(self, position: Position, color: Color, size: float):
        self.position = position
        self.color = color
        self.size = size
        self.object = Shape.create(type=PrimitiveShape.CUBOID,
                                   color=self.color.value,
                                   size=[self.size, self.size, self.size],
                                   position=[self.position.x, self.position.y, self.position.z])

    def get_position(self) -> Position:
        position = self.object.get_position()
        return Position(x=position[0], y=position[1], z=position[2])


class BackgroundPlane:
    def __init__(self, color: Color, position: Position):
        self.color = color
        self.position = position
        if self.color.value != Color.Transparent.value:
            print(self.color.value)
            self.object = Shape.create(type=PrimitiveShape.CUBOID,
                                       color=self.color.value,
                                       size=[1.0, 1.0, 0.001],
                                       position=[self.position.x, self.position.y, self.position.z])

    def get_position(self) -> Position:
        position = self.object.get_position()
        return Position(x=position[0], y=position[1], z=position[2])


class Stack:
    def __init__(self, position: Position, size: float):
        self.position = position
        self.color = Color.Green
        self.size = size
        self.object = Shape.create(type=PrimitiveShape.CUBOID,
                                   color=self.color.value,
                                   size=[self.size, self.size, 0.01],
                                   position=[self.position.x, self.position.y, self.position.z])

    def get_position(self) -> Position:
        position = self.object.get_position()
        return Position(x=position[0], y=position[1], z=position[2])


class Context:
    def __init__(self, num_boxes: int, goal_box_idx: int, stack_position: Position = None,
                 box_positions: List[Position] = None, background_color: Color = Color.Transparent):
        self.box_size = 0.06
        self.stack_size = 0.1

        self.num_boxes = max(min(num_boxes, 5), 0)
        self.goal_box_idx = goal_box_idx

        self.workspace = Workspace()
        self.minx, self.maxx, self.miny, self.maxy, self.minz, self.maxz = self.workspace.get_bounding_box()

        self.background_color = background_color
        self.background_plane = BackgroundPlane(color=self.background_color,
                                                position=Position(x=0.35,
                                                                  y=(self.maxy - self.miny) / 2 + self.miny,
                                                                  z=self.minz + 0.005))

        self.boxes = self.create_boxes(self.num_boxes, box_positions)
        stack_position = stack_position if stack_position is not None else self.generate_stack_position()
        self.stack = Stack(position=stack_position, size=self.stack_size)

    def create_boxes(self, num_boxes: int, box_positions: List[Position] = None) -> List[Box]:
        boxes = []
        for box_i in range(num_boxes):
            if box_positions is not None:
                box_position = box_positions[box_i]
            else:
                while True:
                    box_position = Position(
                        random.uniform(self.minx + self.box_size / 2, self.maxx - self.box_size / 2),
                        random.uniform(self.miny + self.box_size / 2, self.maxy - self.box_size / 2),
                        self.minz + 0.01 + self.box_size / 2)
                    valid = True
                    for box in boxes:
                        if self.__intersect(box.position, self.box_size, box_position, self.box_size):
                            valid = False
                    if valid:
                        break
            boxes.append(Box(position=box_position, color=list(Color)[box_i + 1], size=self.box_size))
        return boxes

    def generate_stack_position(self) -> Position:
        while True:
            stack_position = Position(random.uniform(0, 0.4),
                                      random.uniform(self.miny + self.stack_size / 2,
                                                     self.maxy - self.stack_size / 2),
                                      self.minz + 0.005)
            valid = True
            for box in self.boxes:
                if self.__intersect(box.position, self.box_size, stack_position, self.stack_size):
                    valid = False
            if valid:
                return stack_position

    def reset(self):
        for box in self.boxes:
            position = [box.position.x, box.position.y, box.position.z]
            box.object.set_position(position)
        position = [self.stack.position.x, self.stack.position.y, self.stack.position.z]
        self.stack.object.set_position(position)

    @staticmethod
    def __intersect(p_1: Position, dim_1: float, p_2: Position, dim_2: float) -> bool:
        # Bottom left of square1
        s1_0 = p_1.x - dim_1
        s1_1 = p_1.y - dim_1

        # Top right of square1
        s1_2 = p_1.x + dim_1
        s1_3 = p_1.y + dim_1

        # Bottom left of square2
        s2_0 = p_2.x - dim_2
        s2_1 = p_2.y - dim_2

        # Top right of square2
        s2_2 = p_2.x + dim_2
        s2_3 = p_2.y + dim_2

        B1 = [s1_0, s1_1, s1_2, s1_3]
        B2 = [s2_0, s2_1, s2_2, s2_3]

        if (B1[0] >= B2[2]) or (B1[2] <= B2[0]) or (B1[3] <= B2[1]) or (B1[1] >= B2[3]):
            return False
        else:
            return True
