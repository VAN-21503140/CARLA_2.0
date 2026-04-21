import json

GRAPH_PATH = "D:/carla_pjt/town_graph.json"
SAVE_PATH = "D:/carla_pjt/town_graph.json"

# 这里填你要删除的边 id
BAD_EDGE_IDS = {1061,1006,1009,3787,3279,3949,4082,1994,1360,4328,4331,1694,458,3646,3729,5008,5011,4961,4964,4380,4374,4377,3427,5292}


def main():
    with open(GRAPH_PATH, "r", encoding="utf-8") as f:
        graph = json.load(f)

    old_edges = graph["edges"]

    # 按 edge["id"] 删除
    new_edges = [
        e for e in old_edges
        if e.get("id") not in BAD_EDGE_IDS
    ]

    graph["edges"] = new_edges

    with open(SAVE_PATH, "w", encoding="utf-8") as f:
        json.dump(graph, f, ensure_ascii=False, indent=2)

    print("原始边数:", len(old_edges))
    print("删除后边数:", len(new_edges))
    print("删除的 edge id:", sorted(BAD_EDGE_IDS))


if __name__ == "__main__":
    main()