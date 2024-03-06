import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm

from queue import PriorityQueue

mode = "minimalis tavolsag"

def create_3d_plot(optpath, points):
    x_points, y_points, z_points, b_points = np.array(points).T
    x_path, y_path, z_path, _ = np.array(optpath).T
    obstacle_points = np.array([point[:3] for point in points if point[3] == 1])

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, projection='3d')

    intensity = z_points / np.max(z_points)

    ax.scatter(x_points, y_points, z_points, c=intensity, cmap='hot', marker=',', alpha=0.1)
    ax.scatter(obstacle_points[:, 0], obstacle_points[:, 1], obstacle_points[:, 2], c='black', marker='.', alpha=1)
    ax.plot(x_path, y_path, z_path, c='b', linewidth=2, marker='o')

    cbar = fig.colorbar(cm.ScalarMappable(cmap='hot'), ax=ax)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()


def read_points(filename):
    points = []
    with open(filename, 'r') as file:
        for line in file:
            x, y, z, b = map(float, line.split())
            point = (x, y, z, int(b))
            points.append(point)
    return points


def read_end_points(filename):
    with open(filename, 'r') as file:
        start_line = file.readline()
        end_line = file.readline()

        start = tuple(map(float, start_line.split()))
        finish = tuple(map(float, end_line.split()))

    return start, finish


def get_end_points(start, finish, points):
    start_p = next((point for point in points if point[:2] == start), None)
    finish_p = next((point for point in points if point[:2] == finish), None)

    return start_p, finish_p


def distance3D(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2 + (point1[3] - point2[3]) ** 2)


def distance2D(point1, point2):
    return max(abs(point2[0] - point1[0]), abs(point2[1] - point1[1]))


def heuristic(mode, point1, point2):
    if mode == "minimalis tavolsag":
        return distance3D(point1, point2)
    elif mode == "minimalis lepesszam":
        return distance2D(point1, point2)


def get_neighbors(point, points_set):
    neighbors = []
    x, y = point[:2]
    for dx in range(-1, 2):
        for dy in range(-1, 2):
            neighbor = (x + dx, y + dy)
            if neighbor != point[:2] and neighbor in points_set:
                neighbors.append(points_set[neighbor])
    return neighbors


def a_star_search(points, start, finish):
    start_p, finish_p = get_end_points(start, finish, points)
    points_set = {point[:2]: point for point in points}
    pq = PriorityQueue()
    pq.put((0, start_p))
    optpath = {start_p: None}
    cost = {start_p: 0}

    while not pq.empty():
        current_point = pq.get()[1]

        if current_point == finish_p:
            correctpath = reconstruct_path(optpath, start_p, finish_p)
            return correctpath, cost[finish_p]

        for neighbor in get_neighbors(current_point, points_set):
            newcost = cost[current_point] + heuristic(mode, current_point, neighbor)

            if neighbor not in cost or newcost < cost.get(neighbor, float('inf')) and neighbor[3] == 0:
                cost[neighbor] = newcost
                priority = newcost + heuristic(mode, neighbor, finish_p)
                pq.put((priority, neighbor))
                optpath[neighbor] = current_point

    return None, None


def reconstruct_path(optpath, start_p, finish_p):
    current = finish_p
    path = []
    while current != start_p:
        path.append(current)
        current = optpath[current]
    return path[::-1]

def write_path_to_file(optpath, filename):
    with open(filename, 'w') as file:
        for point in optpath:
            file.write(f"{point[0]} {point[1]} {point[2]}\n")


def main():
    points = read_points("tests/surface_256x256.txt")
    start, finish = read_end_points("tests/surface_256x256.end_points.txt")
    optpath, optcost = a_star_search(points, start, finish)
    if optpath is not None:
        create_3d_plot(optpath, points)
        print(optcost)
        write_path_to_file(optpath, "src/path.txt")
    else:
        print("No path found.")


if __name__ == "__main__":
    main()