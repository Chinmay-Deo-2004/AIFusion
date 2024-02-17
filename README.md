# <h1> Basic implementation of RRT for UAV route optimization: </h1>
This python code is a simple implementation of using the RRT (Rapid Exploring Random Trees) algorithm to find the optimum route for UAVs.

## Initializing data:
There are four data parameters that you need to utilize in order to use the tool:

- `start`: starting co-ordinates of medical supply drop run.
- `goal`: Finishing co-ordinates of medical supply drop run.
- `rand_area`: Range of random co-ordinates from which RRT can sample nodes to trace the path.
- `obstacle list`: List of obstacle co-ordinates. (abscisaa, ordinate, radius)

## Running the project
To run the solution, simply open a terminal session withing the project directory and run the following command: 
`python main.py`

## More about RRT 
RRT (Rapidly-Exploring Random Trees) is a sampling-based algorithm used for path planning in robotics. It incrementally constructs a tree structure by randomly sampling points in the search space and connecting them to the nearest existing node. It biases exploration towards unexplored areas, gradually expanding the tree towards the goal. The algorithm iterates until a path from the start to the goal is found or until a maximum number of iterations is reached. RRT efficiently explores complex environments, making it suitable for dynamic and high-dimensional spaces.
