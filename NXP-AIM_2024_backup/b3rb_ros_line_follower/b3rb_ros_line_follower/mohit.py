import numpy as np
import math

# dist = 159.5 - 160
# half_width = 160
# max_turn = 1
# # print(np.interp(dist, [-half_width, half_width], [-max_turn, max_turn]))

# turn = 0.943
# car_turn_max = math.pi/4

# a1 = np.interp(dist, [-car_turn_max, car_turn_max], [-max_turn, max_turn])
# print(f"a1 = {a1}\n")

# # b1 = np.interp(turn, [-max_turn, max_turn], [-car_turn_max, car_turn_max])
# b1 = np.interp(turn,[-car_turn_max, car_turn_max], [-max_turn, max_turn])
# # b1 = np.interp(turn, [-half_width, half_width], [-car_turn_max, car_turn_max])
# print(f"b1 = {b1}\n")
# print(f"a1 + b1 = {a1+b1}")


# arr = np.zeros(40)
# array = np.arange(200, 159,-1)
# arr = array
# print(arr)
# del_inclination = math.radians(30)
# turn = np.interp(del_inclination, [0, math.pi/2.0], [0, 1])
# print(turn)


def calculate_exponential_sum(numbers):

    # Calculate the negative exponential of each number
    negative_exponentials = np.exp(-numbers)

    # Subtract exponentials from 1 to 5
    subtract_part = np.sum(negative_exponentials[:5])

    # Add exponentials from 6 to 10
    add_part = np.sum(-negative_exponentials[5:])

    # Total value
    total_value = subtract_part + add_part

    return total_value


arr = np.array([5.2, 4.7, 4.3, float("inf"), float("inf"), float("inf"), 1.5, 1.8, 2.3, 2.7])
print(calculate_exponential_sum(arr))