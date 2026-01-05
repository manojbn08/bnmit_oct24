# Implement Best First Search
import heapq

class Graph:
    def __init__(self):
        self.nodes = {}

    def add_node(self, name, heuristic):
        self.nodes[name] = {
            'heuristic': heuristic,
            'edges': {}
        }

    def add_edge(self, from_node, to_node, cost):
        self.nodes[from_node]['edges'][to_node] = cost

    def best_first_search(self, start, goal):
        # Priority queue ordered by heuristic
        priority_queue = []
        heapq.heappush(priority_queue, (self.nodes[start]['heuristic'], start))

        came_from = {}
        explored = set()

        while priority_queue:
            _, current = heapq.heappop(priority_queue)

            if current in explored:
                continue

            explored.add(current)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.nodes[current]['edges']:
                if neighbor not in explored:
                    came_from[neighbor] = current
                    heapq.heappush(
                        priority_queue,
                        (self.nodes[neighbor]['heuristic'], neighbor)
                    )

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]


def main():
    graph = Graph()

    # Define nodes with heuristics
    graph.add_node('A', 10)
    graph.add_node('B', 5)
    graph.add_node('C', 2)
    graph.add_node('D', 0)

    # Define edges
    graph.add_edge('A', 'B', 1)
    graph.add_edge('A', 'C', 4)
    graph.add_edge('B', 'D', 1)
    graph.add_edge('C', 'D', 2)

    start_node = 'A'
    goal_node = 'D'

    path = graph.best_first_search(start_node, goal_node)

    if path:
        print("Path found:", " -> ".join(path))
    else:
        print("No path found.")


if __name__ == "__main__":
    main()