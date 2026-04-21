import carla
import json
from collections import defaultdict


class CarlaGraphBuilder:
    def __init__(
        self,
        carla_map,
        sampling_resolution=2.0,
        lane_change_cost_factor=1.5,
        uturn_cost_factor=5.0,
        enable_lane_change=True,
        enable_uturn=True,
    ):
        self.carla_map = carla_map
        self.sampling_resolution = sampling_resolution
        self.lane_change_cost_factor = lane_change_cost_factor
        self.uturn_cost_factor = uturn_cost_factor
        self.enable_lane_change = enable_lane_change
        self.enable_uturn = enable_uturn

        self.graph = defaultdict(list)   # {node_id: [(neighbor_id, cost, edge_type), ...]}
        self.nodes = {}                  # {node_id: node_info_dict}
        self._lane_index = defaultdict(list)
        self._uturn_source_ids = set()

    # =========================================================
    # 对外接口
    # =========================================================
    def build(self):
        self.graph.clear()
        self.nodes.clear()
        self._lane_index.clear()
        self._uturn_source_ids.clear()

        topology = self.carla_map.get_topology()
        sampled_paths = []

        for entry_wp, exit_wp in topology:
            path = self._sample_segment_path(entry_wp, exit_wp)
            if not path:
                continue

            sampled_paths.append(path)

            for wp in path:
                wp_id = self._waypoint_id(wp)
                self._register_waypoint(wp)
                self._ensure_node(wp_id)

            exit_id = self._waypoint_id(exit_wp)
            if exit_wp.is_junction:
                self._uturn_source_ids.add(exit_id)

        self._connect_sampled_paths(sampled_paths)
        self._connect_next_branches()
        self._build_lane_index()

        if self.enable_lane_change:
            self._connect_lane_changes()

        if self.enable_uturn:
            self._connect_uturns()

    def save(self, save_path):
        data = {
            "meta": {
                "sampling_resolution": self.sampling_resolution,
                "lane_change_cost_factor": self.lane_change_cost_factor,
                "uturn_cost_factor": self.uturn_cost_factor,
                "enable_lane_change": self.enable_lane_change,
                "enable_uturn": self.enable_uturn,
            },
            "nodes": self.nodes,
            "edges": []
        }

        for src_id, neighbors in self.graph.items():
            for dst_id, cost, edge_type in neighbors:
                data["edges"].append({
                    "src": src_id,
                    "dst": dst_id,
                    "cost": cost,
                    "type": edge_type,
                })

        with open(save_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    def debug_summary(self):
        edge_count = sum(len(v) for v in self.graph.values())
        return {
            "num_nodes": len(self.nodes),
            "num_edges": edge_count,
            "sampling_resolution": self.sampling_resolution,
            "lane_change_enabled": self.enable_lane_change,
            "uturn_enabled": self.enable_uturn,
        }

    # =========================================================
    # 注册节点
    # =========================================================
    def _register_waypoint(self, wp):
        wp_id = self._waypoint_id(wp)
        if wp_id in self.nodes:
            return

        loc = wp.transform.location
        rot = wp.transform.rotation
        self.nodes[wp_id] = {
            "road_id": wp.road_id,
            "section_id": wp.section_id,
            "lane_id": wp.lane_id,
            "s": round(wp.s, 2),
            "x": loc.x,
            "y": loc.y,
            "z": loc.z,
            "yaw": rot.yaw,
            "is_junction": wp.is_junction,
        }

    # =========================================================
    # 建图主流程
    # =========================================================
    def _sample_segment_path(self, entry_wp, exit_wp):
        path = [entry_wp]
        current_wp = entry_wp
        visited = set()

        while True:
            current_id = self._waypoint_id(current_wp)
            if current_id in visited:
                break
            visited.add(current_id)

            dist_to_exit = current_wp.transform.location.distance(exit_wp.transform.location)
            if dist_to_exit <= self.sampling_resolution:
                break

            next_wps = current_wp.next(self.sampling_resolution)
            if not next_wps:
                break

            next_wp = min(
                next_wps,
                key=lambda cand: cand.transform.location.distance(exit_wp.transform.location)
            )

            if self._waypoint_id(next_wp) == current_id:
                break

            path.append(next_wp)
            current_wp = next_wp

        if self._waypoint_id(path[-1]) != self._waypoint_id(exit_wp):
            path.append(exit_wp)

        return self._deduplicate_consecutive_waypoints(path)

    def _connect_sampled_paths(self, sampled_paths):
        for path in sampled_paths:
            for i in range(len(path) - 1):
                a = path[i]
                b = path[i + 1]
                a_id = self._waypoint_id(a)
                b_id = self._waypoint_id(b)
                cost = self._waypoint_distance(a, b)
                self._add_edge(a_id, b_id, cost, "forward")

    def _connect_next_branches(self):
        initial_node_ids = list(self.nodes.keys())

        for wp_id in initial_node_ids:
            wp = self._wp_from_node_id(wp_id)
            if wp is None:
                continue

            next_wps = wp.next(self.sampling_resolution)

            for nxt in next_wps:
                nxt_id = self._waypoint_id(nxt)

                if nxt_id not in self.nodes:
                    self._register_waypoint(nxt)
                    self._ensure_node(nxt_id)

                cost = self._waypoint_distance(wp, nxt)
                self._add_edge(wp_id, nxt_id, cost, "branch")

    def _build_lane_index(self):
        self._lane_index.clear()
        for node_id, info in self.nodes.items():
            key = (info["road_id"], info["section_id"], info["lane_id"])
            self._lane_index[key].append(node_id)

    def _connect_lane_changes(self):
        all_node_ids = list(self.nodes.keys())

        for wp_id in all_node_ids:
            wp = self._wp_from_node_id(wp_id)
            if wp is None:
                continue

            left_wp = wp.get_left_lane()
            if left_wp and self._is_valid_lane_change(wp, left_wp, "left"):
                left_id = self._find_closest_existing_node_in_lane(
                    left_wp,
                    max_dist=self.sampling_resolution * 1.5
                )
                if left_id is not None and left_id != wp_id:
                    left_wp_obj = self._wp_from_node_id(left_id)
                    if left_wp_obj is not None:
                        cost = self._waypoint_distance(wp, left_wp_obj) * self.lane_change_cost_factor
                        self._add_edge(wp_id, left_id, cost, "lane_change")

            right_wp = wp.get_right_lane()
            if right_wp and self._is_valid_lane_change(wp, right_wp, "right"):
                right_id = self._find_closest_existing_node_in_lane(
                    right_wp,
                    max_dist=self.sampling_resolution * 1.5
                )
                if right_id is not None and right_id != wp_id:
                    right_wp_obj = self._wp_from_node_id(right_id)
                    if right_wp_obj is not None:
                        cost = self._waypoint_distance(wp, right_wp_obj) * self.lane_change_cost_factor
                        self._add_edge(wp_id, right_id, cost, "lane_change")

    def _connect_uturns(self):
        for wp_id in self._uturn_source_ids:
            wp = self._wp_from_node_id(wp_id)
            if wp is None:
                continue

            target_id = self._find_junction_uturn_target(wp, max_dist=18.0)
            if target_id is not None and target_id != wp_id:
                cost = self._waypoint_distance(wp, self._wp_from_node_id(target_id)) * self.uturn_cost_factor
                self._add_edge(wp_id, target_id, cost, "uturn")

    # =========================================================
    # 工具函数
    # =========================================================
    def _wp_from_node_id(self, node_id):
        info = self.nodes[node_id]
        try:
            return self.carla_map.get_waypoint_xodr(
                info["road_id"],
                info["lane_id"],
                info["s"]
            )
        except RuntimeError:
            return None

    def _waypoint_id(self, waypoint):
        return f"({waypoint.road_id},{waypoint.section_id},{waypoint.lane_id},{round(waypoint.s, 2)})"

    def _waypoint_distance(self, wp1, wp2):
        return wp1.transform.location.distance(wp2.transform.location)

    def _ensure_node(self, wp_id):
        if wp_id not in self.graph:
            self.graph[wp_id] = []

    def _add_edge(self, src_id, dst_id, cost, edge_type, eps=1e-6):
        for nid, old_cost, old_type in self.graph[src_id]:
            if nid == dst_id and abs(old_cost - cost) < eps and old_type == edge_type:
                return
        self.graph[src_id].append((dst_id, cost, edge_type))

    def _deduplicate_consecutive_waypoints(self, path):
        if not path:
            return path
        new_path = [path[0]]
        for wp in path[1:]:
            if self._waypoint_id(wp) != self._waypoint_id(new_path[-1]):
                new_path.append(wp)
        return new_path

    def _find_closest_existing_node_in_lane(self, target_wp, max_dist=3.0):
        best_id = None
        best_dist = float("inf")

        key = (target_wp.road_id, target_wp.section_id, target_wp.lane_id)
        candidate_ids = self._lane_index.get(key, [])
        target_loc = target_wp.transform.location

        for nid in candidate_ids:
            info = self.nodes[nid]
            dx = info["x"] - target_loc.x
            dy = info["y"] - target_loc.y
            dz = info["z"] - target_loc.z
            d = (dx * dx + dy * dy + dz * dz) ** 0.5
            if d < best_dist:
                best_dist = d
                best_id = nid

        return best_id if best_dist <= max_dist else None

    def _is_valid_lane_change(self, current_wp, neighbor_wp, direction):
        if neighbor_wp is None:
            return False

        if (current_wp.road_id != neighbor_wp.road_id or
                current_wp.section_id != neighbor_wp.section_id):
            return False

        if current_wp.lane_id * neighbor_wp.lane_id <= 0:
            return False

        if neighbor_wp.lane_type != carla.LaneType.Driving:
            return False

        lane_change = current_wp.lane_change
        if direction == "left":
            return lane_change in (carla.LaneChange.Left, carla.LaneChange.Both)
        elif direction == "right":
            return lane_change in (carla.LaneChange.Right, carla.LaneChange.Both)

        return False

    def _find_junction_uturn_target(self, wp, max_dist=18.0):
        if not wp.is_junction:
            return None

        best_id = None
        best_score = float("inf")
        src_loc = wp.transform.location
        src_yaw = wp.transform.rotation.yaw
        src_id = self._waypoint_id(wp)

        for nid, info in self.nodes.items():
            if nid == src_id:
                continue

            cand = self._wp_from_node_id(nid)
            if cand is None:
                continue

            if cand.lane_type != carla.LaneType.Driving:
                continue

            if cand.road_id == wp.road_id and cand.lane_id == wp.lane_id:
                continue

            if wp.lane_id * cand.lane_id >=0:
                continue

            d = src_loc.distance(cand.transform.location)
            if d > max_dist:
                continue

            yaw_diff = abs((cand.transform.rotation.yaw - src_yaw + 180) % 360 - 180)
            heading_score = abs(yaw_diff - 180.0)
            score = d + heading_score * 0.12

            if score < best_score:
                best_score = score
                best_id = nid

        return best_id