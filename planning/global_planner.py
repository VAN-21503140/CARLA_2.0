import carla
import json
import heapq
import math
from collections import defaultdict


class RoutePoint:
    def __init__(self, transform):
        self.transform = transform


class CarlaGraphPlanner:
    def __init__(self, carla_map, graph_path):
        self.carla_map = carla_map
        self.graph = defaultdict(list)   # {node_id: [(neighbor_id, cost), ...]}
        self.nodes = {}                  # {node_id: node_info_dict}
        self.edges = []                  # [{id, src, dst, cost, type}, ...]
        self._edge_by_id = {}
        self._lane_index = defaultdict(list)

        self._load_graph(graph_path)
        self._build_lane_index()

    def _load_graph(self, graph_path):
        with open(graph_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        self.nodes = data["nodes"]
        self.edges = data.get("edges", [])
        self._edge_by_id = {
            edge["id"]: edge
            for edge in self.edges
            if "id" in edge
        }

        for edge in self.edges:
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

    def get_edge(self, edge_id):
        return self._edge_by_id.get(edge_id)

    def uturn_edges(self):
        return [edge for edge in self.edges if edge.get("type") == "uturn"]

    def plan_edge_context(
        self,
        edge_id,
        before_steps=6,
        after_steps=8,
        turn_resolution=2.0,
        turn_handle_scale=1.0,
    ):
        edge = self.get_edge(edge_id)
        if edge is None:
            raise RuntimeError(f"Graph edge id {edge_id} does not exist.")
        if edge.get("type") != "uturn":
            raise RuntimeError(
                f"Graph edge id {edge_id} is type {edge.get('type')}, not uturn."
            )

        src_id = edge["src"]
        dst_id = edge["dst"]
        before_ids = self._previous_node_ids(src_id, before_steps)
        after_ids = self._next_node_ids(dst_id, after_steps)
        route_ids = before_ids + [src_id]
        route = []
        for node_id in route_ids:
            wp = self._node_id_to_waypoint(node_id)
            if wp is not None:
                route.append(wp)

        src_wp = self._node_id_to_waypoint(src_id)
        dst_wp = self._node_id_to_waypoint(dst_id)
        if src_wp is None or dst_wp is None:
            raise RuntimeError(f"Could not restore waypoints for edge id {edge_id}.")

        route.extend(
            self._interpolate_turn(
                src_wp,
                dst_wp,
                resolution=turn_resolution,
                handle_scale=turn_handle_scale,
            )
        )

        for node_id in after_ids:
            wp = self._node_id_to_waypoint(node_id)
            if wp is not None:
                route.append(wp)

        if len(route) < 2:
            raise RuntimeError(f"Could not build a route around edge id {edge_id}.")

        return route

    def _interpolate_turn(self, src_wp, dst_wp, resolution=2.0, handle_scale=1.0):
        p0 = src_wp.transform.location
        p3 = dst_wp.transform.location
        distance = max(p0.distance(p3), resolution)
        handle = max(distance * 0.55 * handle_scale, resolution * 2.0)

        start_yaw = math.radians(src_wp.transform.rotation.yaw)
        end_yaw = math.radians(dst_wp.transform.rotation.yaw)

        p1 = carla.Location(
            x=p0.x + math.cos(start_yaw) * handle,
            y=p0.y + math.sin(start_yaw) * handle,
            z=p0.z,
        )
        p2 = carla.Location(
            x=p3.x - math.cos(end_yaw) * handle,
            y=p3.y - math.sin(end_yaw) * handle,
            z=p3.z,
        )

        sample_count = max(4, int(distance / resolution) + 1)
        points = []
        for index in range(1, sample_count):
            t = index / sample_count
            loc = self._cubic_location(p0, p1, p2, p3, t)
            tangent = self._cubic_tangent(p0, p1, p2, p3, t)
            yaw = math.degrees(math.atan2(tangent.y, tangent.x))
            points.append(RoutePoint(carla.Transform(loc, carla.Rotation(yaw=yaw))))

        points.append(dst_wp)
        return points

    def _cubic_location(self, p0, p1, p2, p3, t):
        u = 1.0 - t
        return carla.Location(
            x=(u ** 3) * p0.x + 3 * (u ** 2) * t * p1.x + 3 * u * (t ** 2) * p2.x + (t ** 3) * p3.x,
            y=(u ** 3) * p0.y + 3 * (u ** 2) * t * p1.y + 3 * u * (t ** 2) * p2.y + (t ** 3) * p3.y,
            z=(u ** 3) * p0.z + 3 * (u ** 2) * t * p1.z + 3 * u * (t ** 2) * p2.z + (t ** 3) * p3.z,
        )

    def _cubic_tangent(self, p0, p1, p2, p3, t):
        u = 1.0 - t
        return carla.Vector3D(
            x=3 * (u ** 2) * (p1.x - p0.x) + 6 * u * t * (p2.x - p1.x) + 3 * (t ** 2) * (p3.x - p2.x),
            y=3 * (u ** 2) * (p1.y - p0.y) + 6 * u * t * (p2.y - p1.y) + 3 * (t ** 2) * (p3.y - p2.y),
            z=3 * (u ** 2) * (p1.z - p0.z) + 6 * u * t * (p2.z - p1.z) + 3 * (t ** 2) * (p3.z - p2.z),
        )

    def _previous_node_ids(self, node_id, max_steps):
        reverse_graph = defaultdict(list)
        for src_id, neighbors in self.graph.items():
            for dst_id, cost in neighbors:
                reverse_graph[dst_id].append((src_id, cost))

        ids = []
        current = node_id
        for _ in range(max_steps):
            candidates = reverse_graph.get(current, [])
            if not candidates:
                break
            previous_id, _ = min(candidates, key=lambda item: item[1])
            if previous_id in ids:
                break
            ids.append(previous_id)
            current = previous_id

        ids.reverse()
        return ids

    def _next_node_ids(self, node_id, max_steps):
        ids = []
        current = node_id
        for _ in range(max_steps):
            candidates = self.graph.get(current, [])
            if not candidates:
                break
            next_id, _ = min(candidates, key=lambda item: item[1])
            if next_id in ids:
                break
            ids.append(next_id)
            current = next_id
        return ids

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
