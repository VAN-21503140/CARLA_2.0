import carla
import json
client = carla.Client("localhost", 2000)
client.set_timeout(10.0)

world = client.get_world()
client.reload_world()
world = client.get_world()
save_path = "D:/carla_pjt/town_graph.json"

EDGE_COLORS = {
    "forward": carla.Color(0, 255, 0),
    "branch": carla.Color(0, 128, 255),
    "lane_change": carla.Color(255, 255, 0),
    "uturn": carla.Color(255, 0, 0),
}


def load_graph(graph_path):
    with open(graph_path, "r", encoding="utf-8") as f:
        return json.load(f)


def draw_graph(world, graph_data, edge_types=None, life_time=60.0, draw_points=False):
    debug = world.debug
    nodes = graph_data["nodes"]
    edges = graph_data["edges"]

    if edge_types is not None:
        edge_types = set(edge_types)

    for edge in edges:
        etype = edge["type"]
        if edge_types is not None and etype not in edge_types:
            continue

        src = nodes[edge["src"]]
        dst = nodes[edge["dst"]]

        src_loc = carla.Location(src["x"], src["y"], src["z"] + 0.25)
        dst_loc = carla.Location(dst["x"], dst["y"], dst["z"] + 0.25)

        color = EDGE_COLORS.get(etype, carla.Color(255, 255, 255))

        debug.draw_line(
            src_loc,
            dst_loc,
            thickness=0.08,
            color=color,
            life_time=life_time
        )
        mid_loc = carla.Location(
            (src["x"] + dst["x"]) / 2,
            (src["y"] + dst["y"]) / 2,
            (src["z"] + dst["z"]) / 2 + 0.8
        )

        edge_id = edge.get("id", "N/A")
        debug.draw_string(
            mid_loc,
            str(edge_id),
            draw_shadow=True,
            color=carla.Color(255, 255, 0),
            life_time=life_time,
            persistent_lines=False
        )
        if draw_points:
            debug.draw_point(src_loc, size=0.07, color=color, life_time=life_time)
            debug.draw_point(dst_loc, size=0.07, color=color, life_time=life_time)

def draw_unique_lane_ids(world, graph_data, life_time=120.0):
    debug = world.debug
    nodes = graph_data["nodes"]

    drawn = set()

    for node_id, node in nodes.items():
        road_id = node.get("road_id")
        section_id = node.get("section_id")
        lane_id = node.get("lane_id")

        key = (road_id, section_id, lane_id)
        if key in drawn:
            continue
        drawn.add(key)

        loc = carla.Location(node["x"], node["y"], node["z"] + 3.0)
        text = f"{road_id}/{section_id}/{lane_id}"

        debug.draw_string(
            loc,
            text,
            draw_shadow=False,
            color=carla.Color(255, 0, 255),
            life_time=life_time,
            persistent_lines=False
        )

def print_stats(graph_data):
    counts = {}
    for e in graph_data["edges"]:
        counts[e["type"]] = counts.get(e["type"], 0) + 1

    print("num_nodes:", len(graph_data["nodes"]))
    print("num_edges:", len(graph_data["edges"]))
    print("edge_type_counts:", counts)


def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    graph_data = load_graph(save_path)

    print_stats(graph_data)

    # 只看变道边
    #draw_graph(world, graph_data, edge_types=["lane_change"], life_time=120.0)

    # 只看掉头边
    draw_graph(world, graph_data, edge_types=["uturn"], life_time=120.0)
    draw_unique_lane_ids(world, graph_data, life_time=120.0)
    # 看全部
    #draw_graph(world, graph_data, edge_types=None, life_time=120.0, draw_points=False)

if __name__ == "__main__":
    main()