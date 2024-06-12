import random
import calibration_matrices


def tab2sim(y, x):
    if y > 1920 or y < 0 or x > 1080 or x < 0:
        print("Invalid tablet coordinates")
        quit()

    y_ratio = 1 - y / 1920
    x_ratio = 1 - x / 1080
    y_sim = -0.26 + y_ratio * 0.52
    x_sim = 0.25 + x_ratio * 0.32

    return x_sim, y_sim


def sim2tab(x, y):
    if x > 0.57 or x < 0.25 or y > 0.26 or y < -0.26:
        print("Invalid sim coordinates for tablet")
        quit()

    x_ratio = 1 - (x - 0.25) / 0.32
    y_ratio = 1 - (y - -0.26) / 0.52
    x_sim = x_ratio * 1080
    y_sim = y_ratio * 1920

    return y_sim, x_sim


# for i in range(10):
#     y = random.randint(0, 1921)
#     x = random.randint(0, 1081)
#     print(f'y = {y}, x = {x}, sim = {tab2sim(y, x)}')
#
#     x = round(random.uniform(0.25, 0.57), 2)
#     y = round(random.uniform(-0.26, 0.26), 2)
#     print(f'x = {x}, y = {y}, tab = {sim2tab(x, y)}')

# print(tab2sim(1920, 1080))
# print(tab2sim(0, 0))
# print(sim2tab(0.25, -0.26))
# print(sim2tab(0.57, 0.26))

# for i in range(10):
#     x, y, z = calibration_matrices.target_line_vertical_1(i)
#     print(sim2tab(x, y))
