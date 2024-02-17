import numpy as np
import matplotlib.pyplot as plt

rand_area = [[-500, -500], [500, 500]]

start = [0, 0] 

goal = [30, 30]  

path = []

class RRT:
    def __init__(self, start, goal, obstacle_list, rand_area, expand_dis=1.0, goal_sample_rate=20, max_iter=1000):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list

    def planning(self):
        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node, self.obstacle_list):
                near_inds = self.find_near_nodes(new_node)
                self.node_list.append(new_node)

                for i in near_inds:
                    if not self.check_collision(self.node_list[i], self.obstacle_list):
                        continue

                    d = self.get_distance(self.node_list[i], new_node) + self.node_list[i].cost

                    if new_node.cost > d:
                        new_node.cost = d
                        new_node.parent = i

                self.rewire(new_node, near_inds)

        # Generate path
        last_index = self.get_best_last_index()
        if last_index is None:
            return None

        path = self.generate_final_course(last_index)
        return path

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        new_node.cost += d
        new_node.parent = from_node
        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.goal.x, self.goal.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        return path

    def calc_distance_and_angle(self, from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = np.hypot(dx, dy)
        theta = np.arctan2(dy, dx)
        return d, theta

    def get_random_node(self):
        if np.random.randint(0, 100) > self.goal_sample_rate:
            rnd = [np.random.uniform(self.min_rand[0], self.max_rand[0]), np.random.uniform(self.min_rand[1], self.max_rand[1])]
        else:
            rnd = [self.goal.x, self.goal.y]
        return Node(rnd[0], rnd[1])

    @staticmethod
    def get_distance(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        return np.hypot(dx, dy)

    @staticmethod
    def nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2 for node in node_list]
        minind = dlist.index(min(dlist))
        return minind

    def find_near_nodes(self, new_node):
        nnode = len(self.node_list) + 1
        r = 50.0 * np.sqrt((np.log(nnode) / nnode))
        dlist = [(node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2 for node in self.node_list]
        near_inds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return near_inds

    @staticmethod
    def check_collision(node, obstacle_list):
        for (ox, oy, size) in obstacle_list:
            dx = ox - node.x
            dy = oy - node.y
            d = dx * dx + dy * dy
            if d <= size ** 2:
                return False  # collision
        return True  # safe

    def rewire(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.node_list[i]
            d = self.get_distance(near_node, new_node) + new_node.cost
            if near_node.cost > d:
                near_node.parent = len(self.node_list) - 1
                near_node.cost = d

    def get_best_last_index(self):
        # 10.0
        goal_inds = []
        for i in range(len(self.node_list)):
            d, _ = self.calc_distance_and_angle(self.node_list[i], self.goal)
            if d <= self.expand_dis:
                goal_inds.append(i)
        if len(goal_inds) == 0:
            return None

        min_cost = min([self.node_list[i].cost for i in goal_inds])
        for i in goal_inds:
            if self.node_list[i].cost == min_cost:
                return i
        return None


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


def plot_graph(node_list, obstacle_list):
    plt.clf()
    plt.grid(True)
    if obstacle_list:
        for (ox, oy, size) in obstacle_list:
            plt.plot(ox, oy, "sk", ms=20 * size)

    for node in node_list:
        if node.parent:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g")

    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    plt.plot(start[0], start[1], "xr")
    plt.plot(goal[0], goal[1], "xr")
    plt.axis([0, 100, 0, 100])
    plt.gca().invert_yaxis()
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()


def main():
    print("Start " + __file__)

    # ====Search Path with RRT====
    obstacle_list = [
    (20, 20, 10),
    (30, 50, 5),
    (40, 20, 10),
    (70, 50, 10)
    ] # [x, y, radius]
    # Set Initial parameters
    rrt = RRT(start=[10, 10],
              goal=[80, 80],
              rand_area=rand_area,
              obstacle_list=obstacle_list)
    path = rrt.planning()
    if path:
        print("Path found")
        # Draw final path
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.plot(start[0], start[1], "xr")
        plt.plot(goal[0], goal[1], "xr")
        plt.grid(True)
        plt.axis("equal")
        plt.show()
    else:
        print("Cannot find path")

if __name__ == '__main__':
    main()
