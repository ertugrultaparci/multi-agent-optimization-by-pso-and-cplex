
class Cell:
    def __init__(self):
        self.Id = None
        self.row = None
        self.col = None
        self.center = None
        self.corner1 = None
        self.corner2 = None
        self.corner3 = None
        self.corner4 = None
        self.edgeLength = None
        self.edgeWidth = None
        self.isDenied = False
        self.coverageRequirement = 2

    def print(self):
        print(
            f"id {self.Id}, center {self.center}, corner1 {self.corner1}, corner2 {self.corner2}, " 
            f"corner3 {self.corner3}, corner4 {self.corner4}, row {self.row}, col {self.col} ")

    def printID(self):
        print(f'id {self.Id}')

    def getID(self):
        return int(self.Id)

    def getCenter(self):
        return self.center

    def isInCell(self, x, y):
        if self.corner2[0] >= x >= self.corner1[0] and self.corner4[1] >= y >= self.corner1[1]:
            return True
        else:
            return False

    def setDenied(self):
        self.isDenied = True



