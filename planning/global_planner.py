import carla
import json
import heapq
from collections import defaultdict


class CarlaGraphPlanner:
    def __init__(self, carla_map, graph_path):
        self.carla_map = carla_map
        self.graph = defaultdict(list)   # {node_id: [(neighbor_id, cost), ...]}
        self.nodes = {}                  # {node_id: node_info_dict}
        self._lane_index = defaultdict(list)

        self._load_graph(graph_path)
        self._build_lane_index()

    def _load_graph(self, graph_path):
        with open(graph_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        self.nodes = data["nodes"]

        for edge in data["edges"]:
            self.graph[edge["src"]].append((edge["dst"], edge["cost"]))

        for node_id in self.nodes:
            if node_id not in self.graph:
                self.graph[node_id] = []

    def _build_lane_index(self):
        self._lane_index.clear()
        for node_id, info in self.nodes.items():
            key = (info["road_id"], info["section_id"], info["lane_id"])
            self._lane_index[key].append(node_id)

    def debug_summary(self):
        edge_count = sum(len(v) for v in self.graph.values())
        return {
            "num_nodes": len(self.nodes),
            "num_edges": edge_count,
        }
    def plan(self, start_location, goal_location):
        start_wp = self.carla_map.get_waypoint(start_location, project_to_road=True)
        goal_wp = self.carla_map.get_waypoint(goal_location, project_to_road=True)

        if start_wp is None or goal_wp is None:
            return []

        start_id = self._get_closest_node_id(start_wp.transform.location, hint_wp=start_wp)
        goal_id = self._get_closest_node_id(goal_wp.transform.location, hint_wp=goal_wp)

        if start_id is None or goal_id is None:
            return []

        path_ids = self._astar(start_id, goal_id)
        if not path_ids:
            return []

        route = []
        for node_id in path_ids:
            wp = self._node_id_to_waypoint(node_id)
            if wp is not None:
                route.append(wp)
        return route

    def _node_id_to_waypoint(self, node_id):
        info = self.nodes[node_id]
        try:
            return self.carla_map.get_waypoint_xodr(
                info["road_id"],
                info["lane_id"],
                info["s"]
            )
        except RuntimeError:
            return None

    def _get_closest_node_id(self, location, hint_wp=None):
        best_id = None
        best_dist = float("inf")

        candidate_ids = None
        if hint_wp is not None:
            lane_key = (hint_wp.road_id, hint_wp.section_id, hint_wp.lane_id)
            if lane_key in self._lane_index and self._lane_index[lane_key]:
                candidate_ids = self._lane_index[lane_key]

        if candidate_ids is None:
            candidate_items = self.nodes.items()
        else:
            candidate_items = ((nid, self.nodes[nid]) for nid in candidate_ids)

        for node_id, info in candidate_items:
            dx = info["x"] - location.x
            dy = info["y"] - location.y
            dz = info["z"] - location.z
            dist = (dx * dx + dy * dy + dz * dz) ** 0.5

            if dist < best_dist:
                best_dist = dist
                best_id = node_id

        return best_id

    def _astar(self, start_id, goal_id):
        if start_id == goal_id:
            return [start_id]

        pq = [(0.0, start_id)]
        g_cost = {start_id: 0.0}
        parent = {}
        visited = set()

        while pq:
            _, current = heapq.heappop(pq)

            if current in visited:
                continue
            visited.add(current)

            if current == goal_id:
                break

            for neighbor, edge_cost in self.graph.get(current, []):
                tentative_g = g_cost[current] + edge_cost

                if neighbor not in g_cost or tentative_g < g_cost[neighbor]:
                    g_cost[neighbor] = tentative_g
                    parent[neighbor] = current
                    f = tentative_g + self._heuristic(neighbor, goal_id)
                    heapq.heappush(pq, (f, neighbor))

        return self._reconstruct_path(parent, start_id, goal_id)

    def _heuristic(self, a_id, b_id):
        a = self.nodes[a_id]
        b = self.nodes[b_id]
        dx = a["x"] - b["x"]
        dy = a["y"] - b["y"]
        dz = a["z"] - b["z"]
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    def _reconstruct_path(self, parent, start_id, goal_id):
        if start_id == goal_id:
            return [start_id]
        if goal_id not in parent:
            return []

        path = [goal_id]
        current = goal_id
        while current != start_id:
            current = parent[current]
            path.append(current)

        path.reverse()
        return path