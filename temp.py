class TargetGrid:
    def __init__(self):
        self.x_start, self.x_end, self.x_step = 0, 0.5, 0.05
        self.y_start, self.y_end, self.y_step = -0.3, 0.3, 0.05
        self.z_start, self.z_end, self.z_step = 0.036, 0.072, 0.004
        
        self.x = self.x_start - self.x_step  # Start before the first value
        self.y = self.y_start
        self.z = self.z_start
        self.first_pass = True

    def __iter__(self):
        return self

    def __next__(self):
        if self.x < self.x_end:
            self.x += self.x_step
        elif self.y < self.y_end:
            self.y += self.y_step
            self.x = self.x_start
        elif self.z < self.z_end:
            self.z += self.z_step
            self.y = self.y_start
            self.x = self.x_start
        else:
            if not self.first_pass:
                raise StopIteration
            self.first_pass = False

        return round(self.x, 2), round(self.y, 2), round(self.z, 3)

# Example usage
grid = TargetGrid()
for _ in range(30):  # Example: Get the first 10 points
    print(next(grid))