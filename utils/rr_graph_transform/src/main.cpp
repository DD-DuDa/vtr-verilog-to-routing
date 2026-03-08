/**
 * @file main.cpp
 * @brief RR graph transformation utility.
 *
 * Modes:
 * - boundary (default): legacy boundary-edge pruning.
 * - cerebras: gate-mediated rewiring + global bypass cleanup.
 *
 * Usage:
 *   rr_graph_transform <input.xml> <output.xml>
 *   rr_graph_transform --mode boundary <input.xml> <output.xml>
 *   rr_graph_transform --mode cerebras <input.xml> <output.xml>
 */

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "pugixml.hpp"

namespace {

enum class TransformMode {
    kBoundary,
    kCerebras,
};

struct NodeInfo {
    int id = -1;
    std::string type;
    int layer = 0;
    int xlow = 0;
    int xhigh = 0;
    int ylow = 0;
    int yhigh = 0;
    int ptc = -1;
    int capacity = 1;
    pugi::xml_node xml_node;
};

struct EdgeInfo {
    int src = -1;
    int sink = -1;
    int switch_id = -1;
};

struct RemovedEdgeInfo {
    int base_src = -1;
    int dst = -1;
    int switch_id = -1;
};

constexpr int kTypeOther = 0;
constexpr int kTypeChanx = 1;
constexpr int kTypeChany = 2;
constexpr int kTypeOpin = 3;
constexpr int kTypeIpin = 4;

int node_type_code(const std::string& type) {
    if (type == "CHANX") return kTypeChanx;
    if (type == "CHANY") return kTypeChany;
    if (type == "OPIN") return kTypeOpin;
    if (type == "IPIN") return kTypeIpin;
    return kTypeOther;
}

struct NodeLookupKey {
    int type_code = 0;
    int layer = 0;
    int x = 0;
    int y = 0;
    int ptc = -1;

    bool operator==(const NodeLookupKey& other) const {
        return type_code == other.type_code
               && layer == other.layer
               && x == other.x
               && y == other.y
               && ptc == other.ptc;
    }
};

struct NodeLookupKeyHash {
    std::size_t operator()(const NodeLookupKey& k) const noexcept {
        std::size_t h = static_cast<std::size_t>(k.type_code);
        h ^= static_cast<std::size_t>(k.layer) * 0x9e3779b1u + (h << 6) + (h >> 2);
        h ^= static_cast<std::size_t>(k.x) * 0x85ebca6bu + (h << 6) + (h >> 2);
        h ^= static_cast<std::size_t>(k.y) * 0xc2b2ae35u + (h << 6) + (h >> 2);
        h ^= static_cast<std::size_t>(k.ptc) * 0x27d4eb2fu + (h << 6) + (h >> 2);
        return h;
    }
};

struct EdgeKey {
    int src = -1;
    int sink = -1;
    int switch_id = -1;

    bool operator==(const EdgeKey& other) const {
        return src == other.src
               && sink == other.sink
               && switch_id == other.switch_id;
    }
};

struct EdgeKeyHash {
    std::size_t operator()(const EdgeKey& k) const noexcept {
        std::size_t h = static_cast<std::size_t>(k.src);
        h ^= static_cast<std::size_t>(k.sink) * 0x9e3779b1u + (h << 6) + (h >> 2);
        h ^= static_cast<std::size_t>(k.switch_id) * 0x85ebca6bu + (h << 6) + (h >> 2);
        return h;
    }
};

struct ParsedGraph {
    pugi::xml_document doc;
    pugi::xml_node rr_graph;
    pugi::xml_node rr_nodes;
    pugi::xml_node rr_edges;
    pugi::xml_node grid;
    pugi::xml_node block_types;

    std::unordered_map<int, NodeInfo> nodes_by_id;
    std::unordered_map<NodeLookupKey, int, NodeLookupKeyHash> node_key_to_id;
    std::unordered_set<long long> pe_tiles_xy;

    std::vector<EdgeInfo> edges;

    pugi::xml_node chany_template;
    int max_node_id = -1;
    int max_grid_layer = 0;
    int max_node_layer = 0;
};

long long xy_key(int x, int y) {
    return (static_cast<long long>(x) << 32) ^ static_cast<unsigned int>(y);
}

int to_int(const pugi::xml_attribute& attr, int default_val = 0) {
    if (!attr) return default_val;
    return attr.as_int(default_val);
}

void set_int_attr(pugi::xml_node node, const char* name, int value) {
    pugi::xml_attribute attr = node.attribute(name);
    if (!attr) {
        attr = node.append_attribute(name);
    }
    attr.set_value(value);
}

int find_node_id(const ParsedGraph& g, const std::string& type, int layer, int x, int y, int ptc) {
    const NodeLookupKey key{node_type_code(type), layer, x, y, ptc};
    const auto it = g.node_key_to_id.find(key);
    if (it == g.node_key_to_id.end()) return -1;
    return it->second;
}

bool is_channel_type(const std::string& type) {
    return type == "CHANX" || type == "CHANY";
}

bool is_unit_node(const NodeInfo& n) {
    return (n.xlow == n.xhigh) && (n.ylow == n.yhigh);
}

bool parse_graph(const char* input_file, ParsedGraph& g) {
    pugi::xml_parse_result result = g.doc.load_file(input_file);
    if (!result) {
        std::cerr << "Error: Failed to parse " << input_file << ": " << result.description() << "\n";
        return false;
    }

    g.rr_graph = g.doc.child("rr_graph");
    if (!g.rr_graph) {
        std::cerr << "Error: No rr_graph root element found\n";
        return false;
    }

    g.rr_nodes = g.rr_graph.child("rr_nodes");
    g.rr_edges = g.rr_graph.child("rr_edges");
    g.grid = g.rr_graph.child("grid");
    g.block_types = g.rr_graph.child("block_types");

    if (!g.rr_nodes) {
        std::cerr << "Error: No rr_nodes section found\n";
        return false;
    }
    if (!g.rr_edges) {
        std::cerr << "Error: No rr_edges section found\n";
        return false;
    }

    const int declared_node_count = to_int(g.rr_nodes.attribute("count"), 0);
    if (declared_node_count > 0) {
        g.nodes_by_id.reserve(static_cast<std::size_t>(declared_node_count));
        g.node_key_to_id.reserve(static_cast<std::size_t>(declared_node_count));
    }

    const int declared_edge_count = to_int(g.rr_edges.attribute("count"), 0);
    if (declared_edge_count > 0) {
        g.edges.reserve(static_cast<std::size_t>(declared_edge_count));
    }

    // Find pe_tile block type id.
    int pe_block_type_id = -1;
    if (g.block_types) {
        for (pugi::xml_node bt = g.block_types.child("block_type"); bt; bt = bt.next_sibling("block_type")) {
            const char* name = bt.attribute("name").value();
            if (name && std::strcmp(name, "pe_tile") == 0) {
                pe_block_type_id = bt.attribute("id").as_int(-1);
                break;
            }
        }
    }

    // Parse grid and track max grid layer.
    if (g.grid) {
        for (pugi::xml_node gl = g.grid.child("grid_loc"); gl; gl = gl.next_sibling("grid_loc")) {
            int layer = to_int(gl.attribute("layer"), 0);
            g.max_grid_layer = std::max(g.max_grid_layer, layer);

            if (pe_block_type_id >= 0 && to_int(gl.attribute("block_type_id"), -1) == pe_block_type_id) {
                int x = to_int(gl.attribute("x"), 0);
                int y = to_int(gl.attribute("y"), 0);
                g.pe_tiles_xy.insert(xy_key(x, y));
            }
        }
    }

    // Parse nodes.
    for (pugi::xml_node node = g.rr_nodes.child("node"); node; node = node.next_sibling("node")) {
        NodeInfo n;
        n.id = to_int(node.attribute("id"), -1);
        if (n.id < 0) continue;

        n.type = node.attribute("type").value();
        n.capacity = to_int(node.attribute("capacity"), 1);
        n.xml_node = node;

        pugi::xml_node loc = node.child("loc");
        if (loc) {
            n.layer = to_int(loc.attribute("layer"), 0);
            n.xlow = to_int(loc.attribute("xlow"), 0);
            n.xhigh = to_int(loc.attribute("xhigh"), n.xlow);
            n.ylow = to_int(loc.attribute("ylow"), 0);
            n.yhigh = to_int(loc.attribute("yhigh"), n.ylow);
            n.ptc = to_int(loc.attribute("ptc"), -1);
        }

        g.max_node_id = std::max(g.max_node_id, n.id);
        g.max_node_layer = std::max(g.max_node_layer, n.layer);

        g.nodes_by_id[n.id] = n;
        if (is_unit_node(n) && n.ptc >= 0) {
            const NodeLookupKey key{node_type_code(n.type), n.layer, n.xlow, n.ylow, n.ptc};
            g.node_key_to_id[key] = n.id;
        }
        if (!g.chany_template && n.type == "CHANY") {
            g.chany_template = node;
        }
    }

    // Parse edges.
    for (pugi::xml_node edge = g.rr_edges.child("edge"); edge; edge = edge.next_sibling("edge")) {
        EdgeInfo e;
        e.src = to_int(edge.attribute("src_node"), -1);
        e.sink = to_int(edge.attribute("sink_node"), -1);
        e.switch_id = to_int(edge.attribute("switch_id"), -1);
        if (e.src >= 0 && e.sink >= 0 && e.switch_id >= 0) {
            g.edges.push_back(e);
        }
    }

    return true;
}

bool write_edges(ParsedGraph& g, const std::vector<EdgeInfo>& edges) {
    // Remove all existing edge children.
    for (pugi::xml_node edge = g.rr_edges.child("edge"); edge;) {
        pugi::xml_node next = edge.next_sibling("edge");
        g.rr_edges.remove_child(edge);
        edge = next;
    }

    for (const auto& e : edges) {
        pugi::xml_node edge = g.rr_edges.append_child("edge");
        edge.append_attribute("sink_node").set_value(e.sink);
        edge.append_attribute("src_node").set_value(e.src);
        edge.append_attribute("switch_id").set_value(e.switch_id);
    }
    return true;
}

int infer_regular_switch_id(const ParsedGraph& g) {
    for (const auto& e : g.edges) {
        auto src_it = g.nodes_by_id.find(e.src);
        auto sink_it = g.nodes_by_id.find(e.sink);
        if (src_it == g.nodes_by_id.end() || sink_it == g.nodes_by_id.end()) continue;
        if (is_channel_type(src_it->second.type) && is_channel_type(sink_it->second.type)) {
            return e.switch_id;
        }
    }
    return 2; // fallback in local archs
}

bool is_pe_xy(const ParsedGraph& g, int x, int y) {
    return g.pe_tiles_xy.count(xy_key(x, y)) > 0;
}

bool is_pe_opin(const ParsedGraph& g, const NodeInfo& n) {
    return n.type == "OPIN" && n.layer == 0 && is_unit_node(n) && is_pe_xy(g, n.xlow, n.ylow);
}

bool is_pe_ipin(const ParsedGraph& g, const NodeInfo& n) {
    return n.type == "IPIN" && n.layer == 0 && is_unit_node(n) && is_pe_xy(g, n.xlow, n.ylow);
}

bool is_base_chany_for_pe(const ParsedGraph& g, const NodeInfo& n) {
    if (!(n.type == "CHANY" && n.layer == 0 && is_unit_node(n) && n.ptc >= 0)) {
        return false;
    }
    return is_pe_xy(g, n.xlow, n.ylow);
}

bool is_base_chanx_for_pe(const ParsedGraph& g, const NodeInfo& n) {
    if (!(n.type == "CHANX" && n.layer == 0 && is_unit_node(n) && n.ptc >= 0)) {
        return false;
    }
    return is_pe_xy(g, n.xlow, n.ylow);
}

bool is_forbidden_base_downward_edge(const ParsedGraph& g, const NodeInfo& src, const NodeInfo& sink) {
    if (!is_base_chany_for_pe(g, src)) return false;
    if (!(sink.layer == 0 && is_channel_type(sink.type))) return false;
    // Keep boundary escape edges toward IO-side channels.
    if (!is_pe_xy(g, sink.xlow, sink.ylow)) return false;
    const int base_y = src.ylow;
    if (sink.type == "CHANY") {
        // Forbid base->CHANY to row above, and also forbid base->CHANY(b-1,0).
        return sink.ylow >= (base_y + 1) || sink.ylow == (base_y - 1);
    }
    if (sink.type == "CHANX") {
        // Prune CHANX at y>=b and y==b-1 (i.e., prune CHANX at y>=b-1).
        return sink.ylow >= (base_y - 1);
    }
    return false;
}

bool is_forbidden_base_chanx_r7_edge(const ParsedGraph& g, const NodeInfo& src, const NodeInfo& sink) {
    if (!is_base_chanx_for_pe(g, src)) return false;
    if (sink.ptc != src.ptc) return false;
    // Keep boundary escape edges toward IO-side channels.
    if (is_channel_type(sink.type) && sink.layer == 0 && !is_pe_xy(g, sink.xlow, sink.ylow)) {
        return false;
    }

    // Prune any base CHANX(a,b,t) -> CHANY(*,*,t) on layer 0.
    if (sink.type == "CHANY" && sink.layer == 0) {
        return true;
    }

    // Prune lateral same-row immediate neighbors on CHANX.
    if (sink.type == "CHANX" && sink.layer == 0 && is_unit_node(sink) && sink.ylow == src.ylow) {
        return (sink.xlow == src.xlow - 1) || (sink.xlow == src.xlow + 1);
    }

    return false;
}

bool is_allowed_base_chany_gate_sink(const ParsedGraph& g, const NodeInfo& src, int sink_id, int gate_layer) {
    if (!is_base_chany_for_pe(g, src)) return false;
    int g0 = find_node_id(g, "CHANY", gate_layer, src.xlow, src.ylow, src.ptc);
    if (sink_id == g0) return true;
    if (src.ylow > 0) {
        int g1 = find_node_id(g, "CHANY", gate_layer, src.xlow, src.ylow - 1, src.ptc);
        if (sink_id == g1) return true;
    }
    return false;
}

bool is_allowed_base_chanx_gate_sink(const ParsedGraph& g, const NodeInfo& src, int sink_id, int gate_layer) {
    if (!is_base_chanx_for_pe(g, src)) return false;
    int g0 = find_node_id(g, "CHANY", gate_layer, src.xlow, src.ylow, src.ptc);
    if (sink_id == g0) return true;
    if (src.xlow > 0) {
        int g1 = find_node_id(g, "CHANY", gate_layer, src.xlow - 1, src.ylow, src.ptc);
        if (sink_id == g1) return true;
    }
    return false;
}

bool should_reattach_removed_edge_from_primary_gate(const NodeInfo& base, const NodeInfo& dst) {
    if (!(dst.layer == 0 && is_channel_type(dst.type))) return false;
    if (!is_unit_node(base) || !is_unit_node(dst)) return false;

    const int base_y = base.ylow;
    if (dst.type == "CHANX") {
        return dst.ylow >= base_y;
    }
    if (dst.type == "CHANY") {
        return dst.xlow == base.xlow
               && (dst.ylow >= (base_y + 1) || dst.ylow == (base_y - 1));
    }
    return false;
}

int fixed_gate_y_for_pe_track(int pe_y) {
    // User policy: gate location is fixed at CHANY(a, b, gate_layer, track),
    // where PE is at (a, b, 0). No y+1 / y-1 fallback.
    return pe_y;
}

int get_or_create_gate_chany_node(ParsedGraph& g, int x, int y, int track, int gate_layer, const NodeInfo* template_src) {
    const NodeLookupKey key{kTypeChany, gate_layer, x, y, track};
    auto it = g.node_key_to_id.find(key);
    if (it != g.node_key_to_id.end()) {
        return it->second;
    }

    pugi::xml_node template_node = pugi::xml_node();
    if (template_src && template_src->xml_node) {
        template_node = template_src->xml_node;
    } else if (g.chany_template) {
        template_node = g.chany_template;
    }

    if (!template_node) {
        return -1;
    }

    pugi::xml_node new_node = g.rr_nodes.append_copy(template_node);
    const int new_id = ++g.max_node_id;

    set_int_attr(new_node, "id", new_id);
    set_int_attr(new_node, "capacity", 1);
    pugi::xml_attribute type_attr = new_node.attribute("type");
    if (!type_attr) type_attr = new_node.append_attribute("type");
    type_attr.set_value("CHANY");

    pugi::xml_node loc = new_node.child("loc");
    if (!loc) loc = new_node.append_child("loc");
    set_int_attr(loc, "layer", gate_layer);
    set_int_attr(loc, "ptc", track);
    set_int_attr(loc, "xlow", x);
    set_int_attr(loc, "xhigh", x);
    set_int_attr(loc, "ylow", y);
    set_int_attr(loc, "yhigh", y);
    // Ensure no pin-side residue on channel nodes.
    pugi::xml_attribute side_attr = loc.attribute("side");
    if (side_attr) {
        loc.remove_attribute(side_attr);
    }

    NodeInfo n;
    n.id = new_id;
    n.type = "CHANY";
    n.layer = gate_layer;
    n.xlow = x;
    n.xhigh = x;
    n.ylow = y;
    n.yhigh = y;
    n.ptc = track;
    n.capacity = 1;
    n.xml_node = new_node;

    g.nodes_by_id[new_id] = n;
    g.node_key_to_id[key] = new_id;
    g.max_node_layer = std::max(g.max_node_layer, gate_layer);
    return new_id;
}

void push_edge_unique(std::vector<EdgeInfo>& out_edges,
                      std::unordered_set<EdgeKey, EdgeKeyHash>& edge_seen,
                      int src, int sink, int sw) {
    if (src < 0 || sink < 0 || sw < 0) return;
    const EdgeKey key{src, sink, sw};
    if (edge_seen.insert(key).second) {
        out_edges.push_back({src, sink, sw});
    }
}

bool transform_boundary(ParsedGraph& g) {
    std::unordered_set<int> target_node_ids;

    for (const auto& kv : g.nodes_by_id) {
        const auto& n = kv.second;
        if (n.type == "CHANY" && n.xlow == 0 && n.xhigh == 0) {
            target_node_ids.insert(n.id);
        } else if (n.type == "CHANX" && n.ylow == 0 && n.yhigh == 0) {
            target_node_ids.insert(n.id);
        }
    }

    std::vector<EdgeInfo> new_edges;
    new_edges.reserve(g.edges.size());
    for (const auto& e : g.edges) {
        if (target_node_ids.count(e.src) > 0 || target_node_ids.count(e.sink) > 0) continue;
        new_edges.push_back(e);
    }

    std::cout << "boundary mode: target nodes=" << target_node_ids.size()
              << ", removed edges=" << (g.edges.size() - new_edges.size()) << "\n";

    return write_edges(g, new_edges);
}

bool transform_cerebras(ParsedGraph& g, int gate_layer) {
    const int regular_switch = infer_regular_switch_id(g);

    if (gate_layer > g.max_grid_layer) {
        std::cout << "Warning: requested gate layer " << gate_layer
                  << " exceeds max grid layer " << g.max_grid_layer
                  << ". Clamping gate layer to " << g.max_grid_layer << ".\n";
        gate_layer = g.max_grid_layer;
    }

    std::unordered_map<int, int> base_to_gate;            // base CHANY node id -> gate node id
    std::unordered_set<int> gate_node_ids;                // ids of all gate nodes used/created
    std::vector<RemovedEdgeInfo> removed_r4_edges;        // for re-attachment from gate

    std::vector<EdgeInfo> kept_edges;
    kept_edges.reserve(g.edges.size());

    size_t removed_r2 = 0;
    size_t removed_r3 = 0;
    size_t removed_r4 = 0;
    size_t removed_r7_base_chanx = 0;
    size_t added_pin_to_gate = 0;

    // First pass: apply channel pruning rules (R4/R7) and add direct pin<->gate rewires.
    // User policy: keep all original pin-connected edges (do not prune OPIN/IPIN edges).
    for (const auto& e : g.edges) {
        auto src_it = g.nodes_by_id.find(e.src);
        auto sink_it = g.nodes_by_id.find(e.sink);
        if (src_it == g.nodes_by_id.end() || sink_it == g.nodes_by_id.end()) {
            continue;
        }
        const NodeInfo& src = src_it->second;
        const NodeInfo& sink = sink_it->second;

        bool remove = false;

        // R2: PE OPIN must handoff via gate (no direct PE OPIN->CHAN*).
        if (is_pe_opin(g, src) && is_channel_type(sink.type) && sink.layer == 0) {
            int gx = src.xlow;
            int gtrack = sink.ptc;
            int gy = fixed_gate_y_for_pe_track(src.ylow);
            int gate_id = get_or_create_gate_chany_node(g, gx, gy, gtrack, gate_layer, &sink);
            if (gate_id >= 0) {
                gate_node_ids.insert(gate_id);
                // preserve switch class from original OPIN edge
                kept_edges.push_back({src.id, gate_id, e.switch_id});
                added_pin_to_gate++;
            }
            remove = true;
            removed_r2++;
        }

        // R3: PE IPIN must be driven via gate (no direct layer0 CHAN*->PE IPIN).
        if (!remove && is_channel_type(src.type) && src.layer == 0 && is_pe_ipin(g, sink)) {
            int gx = sink.xlow;
            int gtrack = src.ptc;
            int gy = fixed_gate_y_for_pe_track(sink.ylow);
            int gate_id = get_or_create_gate_chany_node(g, gx, gy, gtrack, gate_layer, &src);
            if (gate_id >= 0) {
                gate_node_ids.insert(gate_id);
                // preserve switch class from original IPIN edge
                kept_edges.push_back({gate_id, sink.id, e.switch_id});
                added_pin_to_gate++;
            }
            remove = true;
            removed_r3++;
        }

        // R4: remove forbidden downward base-CHANY transitions and record for gate reattach.
        if (!remove && is_forbidden_base_downward_edge(g, src, sink)) {
            int gate_id = -1;
            auto bg_it = base_to_gate.find(src.id);
            if (bg_it != base_to_gate.end()) {
                gate_id = bg_it->second;
            } else {
                gate_id = get_or_create_gate_chany_node(g, src.xlow, src.ylow, src.ptc, gate_layer, &src);
                if (gate_id >= 0) {
                    base_to_gate[src.id] = gate_id;
                    gate_node_ids.insert(gate_id);
                }
            }

            removed_r4_edges.push_back({src.id, sink.id, e.switch_id});
            remove = true;
            removed_r4++;
        }

        // R7 (new): for base CHANX(a,b,t), remove forbidden CHANX/CHANY edges.
        if (!remove && is_forbidden_base_chanx_r7_edge(g, src, sink)) {
            remove = true;
            removed_r7_base_chanx++;
        }

        if (!remove) {
            kept_edges.push_back(e);
        }
    }

    // R5: mandatory base->gate handoff.
    size_t added_base_to_down_gate = 0;
    for (const auto& kv : base_to_gate) {
        int base_id = kv.first;
        int gate_id = kv.second;
        if (base_id == gate_id) continue;
        kept_edges.push_back({base_id, gate_id, regular_switch});
        gate_node_ids.insert(gate_id);

        auto base_it = g.nodes_by_id.find(base_id);
        if (base_it == g.nodes_by_id.end()) continue;
        const NodeInfo& base = base_it->second;
        if (!is_unit_node(base)) continue;
        if (base.ylow <= 0) continue;

        int down_gate_id = get_or_create_gate_chany_node(g, base.xlow, base.ylow - 1, base.ptc, gate_layer, &base);
        if (down_gate_id < 0) continue;

        kept_edges.push_back({base_id, down_gate_id, regular_switch});
        gate_node_ids.insert(down_gate_id);
        added_base_to_down_gate++;
    }

    // R6: reconnect selected removed R4 destinations from primary gate.
    size_t reattached_from_gate = 0;
    size_t skipped_r6 = 0;
    for (const auto& r : removed_r4_edges) {
        auto bg_it = base_to_gate.find(r.base_src);
        if (bg_it == base_to_gate.end()) continue;
        const int gate_id = bg_it->second;

        auto base_it = g.nodes_by_id.find(r.base_src);
        auto dst_it = g.nodes_by_id.find(r.dst);
        if (base_it == g.nodes_by_id.end() || dst_it == g.nodes_by_id.end()) {
            continue;
        }
        const NodeInfo& base = base_it->second;
        const NodeInfo& dst = dst_it->second;

        if (!should_reattach_removed_edge_from_primary_gate(base, dst)) {
            skipped_r6++;
            continue;
        }

        kept_edges.push_back({gate_id, r.dst, r.switch_id});
        reattached_from_gate++;
    }

    // R6.5: enforce gate<->layer0-channel coupling at (x,y,ptc) and
    // immediate positive neighbors used by the gate fabric:
    //   gate(a,b,t) <-> CHANX(a,b,t), CHANX(a+1,b,t), CHANY(a,b,t), CHANY(a,b+1,t)
    size_t added_local_gate_links = 0;
    for (int gate_id : gate_node_ids) {
        const auto gate_it = g.nodes_by_id.find(gate_id);
        if (gate_it == g.nodes_by_id.end()) continue;
        const NodeInfo& gate = gate_it->second;
        if (gate.layer != gate_layer || !is_unit_node(gate) || gate.ptc < 0) continue;

        const int chany0 = find_node_id(g, "CHANY", 0, gate.xlow, gate.ylow, gate.ptc);
        if (chany0 >= 0) {
            kept_edges.push_back({gate_id, chany0, regular_switch});
            kept_edges.push_back({chany0, gate_id, regular_switch});
            added_local_gate_links += 2;
        }

        const int chanx0 = find_node_id(g, "CHANX", 0, gate.xlow, gate.ylow, gate.ptc);
        if (chanx0 >= 0) {
            kept_edges.push_back({gate_id, chanx0, regular_switch});
            kept_edges.push_back({chanx0, gate_id, regular_switch});
            added_local_gate_links += 2;
        }

        const int chany0_up = find_node_id(g, "CHANY", 0, gate.xlow, gate.ylow + 1, gate.ptc);
        if (chany0_up >= 0) {
            kept_edges.push_back({gate_id, chany0_up, regular_switch});
            kept_edges.push_back({chany0_up, gate_id, regular_switch});
            added_local_gate_links += 2;
        }

        const int chanx0_right = find_node_id(g, "CHANX", 0, gate.xlow + 1, gate.ylow, gate.ptc);
        if (chanx0_right >= 0) {
            kept_edges.push_back({gate_id, chanx0_right, regular_switch});
            kept_edges.push_back({chanx0_right, gate_id, regular_switch});
            added_local_gate_links += 2;
        }
    }

    // R7 (new): base CHANX handoff to gate(a,b,t) and gate(a-1,b,t).
    size_t added_base_chanx_to_gate = 0;
    size_t added_base_chanx_to_left_gate = 0;
    std::vector<int> base_chanx_ids;
    base_chanx_ids.reserve(g.nodes_by_id.size());
    for (const auto& kv : g.nodes_by_id) {
        if (is_base_chanx_for_pe(g, kv.second)) {
            base_chanx_ids.push_back(kv.first);
        }
    }
    for (int base_id : base_chanx_ids) {
        auto it = g.nodes_by_id.find(base_id);
        if (it == g.nodes_by_id.end()) continue;
        const NodeInfo& base_cx = it->second;

        int gate_id = get_or_create_gate_chany_node(g, base_cx.xlow, base_cx.ylow, base_cx.ptc, gate_layer, nullptr);
        if (gate_id >= 0) {
            kept_edges.push_back({base_cx.id, gate_id, regular_switch});
            gate_node_ids.insert(gate_id);
            added_base_chanx_to_gate++;
        }

        if (base_cx.xlow > 0) {
            int left_gate_id = get_or_create_gate_chany_node(g, base_cx.xlow - 1, base_cx.ylow, base_cx.ptc, gate_layer, nullptr);
            if (left_gate_id >= 0) {
                kept_edges.push_back({base_cx.id, left_gate_id, regular_switch});
                gate_node_ids.insert(left_gate_id);
                added_base_chanx_to_left_gate++;
            }
        }
    }

    // R8: global bypass cleanup pass.
    std::vector<EdgeInfo> cleaned_edges;
    cleaned_edges.reserve(kept_edges.size());

    size_t removed_r8 = 0;
    size_t removed_r8_base_exact = 0;
    size_t removed_r8_pin_exact = 0;
    for (const auto& e : kept_edges) {
        auto src_it = g.nodes_by_id.find(e.src);
        auto sink_it = g.nodes_by_id.find(e.sink);
        if (src_it == g.nodes_by_id.end() || sink_it == g.nodes_by_id.end()) {
            continue;
        }
        const NodeInfo& src = src_it->second;
        const NodeInfo& sink = sink_it->second;

        bool remove = false;

        const bool src_is_gate = gate_node_ids.count(src.id) > 0;
        const bool sink_is_gate = gate_node_ids.count(sink.id) > 0;

        // Strict gate policy for multi-layer mode:
        // apply allowlist only to edges incident to our inserted gate nodes.
        // Keep non-gate layer-1 edges unchanged (needed for IO RR-graph validity).
        if (gate_layer > 0 && (src_is_gate || sink_is_gate)) {
            bool allow = false;

            // Hard ban CHANX on gate layer in transformed routing usage.
            if ((src.layer == gate_layer && src.type == "CHANX")
                || (sink.layer == gate_layer && sink.type == "CHANX")) {
                allow = false;
            } else
            if (is_pe_opin(g, src) && sink_is_gate) {
                allow = true;
            } else if (src_is_gate && is_pe_ipin(g, sink)) {
                allow = true;
            } else if (is_channel_type(src.type) && src.layer == 0 && sink_is_gate
                       && is_unit_node(src) && is_unit_node(sink)
                       && src.xlow == sink.xlow && src.ylow == sink.ylow + 1) {
                // Allow base CHANY(a,b,0)->down gate CHANY(a,b-1,1) handoff edge.
                allow = true;
            } else if (src.type == "CHANX" && src.layer == 0 && sink_is_gate
                       && is_unit_node(src) && is_unit_node(sink)
                       && src.ptc == sink.ptc
                       && src.xlow == sink.xlow + 1 && src.ylow == sink.ylow) {
                // Allow base CHANX(a,b,0,t)->left gate(a-1,b,1,t) handoff edge (R7).
                allow = true;
            } else if (src_is_gate && sink.layer == 0 && is_channel_type(sink.type)
                       && is_unit_node(src) && is_unit_node(sink)
                       && src.ptc == sink.ptc
                       && ((sink.type == "CHANY" && sink.xlow == src.xlow
                            && (sink.ylow == src.ylow || sink.ylow == src.ylow + 1))
                           || (sink.type == "CHANX" && sink.ylow == src.ylow
                               && (sink.xlow == src.xlow || sink.xlow == src.xlow + 1)))) {
                allow = true;
            } else if (sink_is_gate && src.layer == 0 && is_channel_type(src.type)
                       && is_unit_node(src) && is_unit_node(sink)
                       && src.ptc == sink.ptc
                       && ((src.type == "CHANY" && src.xlow == sink.xlow
                            && (src.ylow == sink.ylow || src.ylow == sink.ylow + 1))
                           || (src.type == "CHANX" && src.ylow == sink.ylow
                               && (src.xlow == sink.xlow || src.xlow == sink.xlow + 1)))) {
                allow = true;
            }

            if (!allow) {
                remove = true;
            }
        }

        // User policy: keep all pin-connected edges; do not enforce pin-only-through-gate.

        // Strict base policy requested by user:
        // base CHANY(a,b,0,t) -> {gate(a,b,1,t), gate(a,b-1,1,t)} only.
        if (!remove && is_base_chany_for_pe(g, src) && sink.type != "IPIN" && sink.type != "OPIN") {
            // Keep boundary escape edges from base channels to IO-side channels.
            if (sink.layer == 0 && is_channel_type(sink.type) && !is_pe_xy(g, sink.xlow, sink.ylow)) {
                // keep
            } else
            if (!is_allowed_base_chany_gate_sink(g, src, sink.id, gate_layer)) {
                remove = true;
                removed_r8_base_exact++;
            }
        }

        // base CHANX(a,b,0,t) -> {gate(a,b,1,t), gate(a-1,b,1,t)} only.
        if (!remove && is_base_chanx_for_pe(g, src) && sink.type != "IPIN" && sink.type != "OPIN") {
            // Keep boundary escape edges from base channels to IO-side channels.
            if (sink.layer == 0 && is_channel_type(sink.type) && !is_pe_xy(g, sink.xlow, sink.ylow)) {
                // keep
            } else
            if (!is_allowed_base_chanx_gate_sink(g, src, sink.id, gate_layer)) {
                remove = true;
                removed_r8_base_exact++;
            }
        }

        // Remove residual forbidden downward base edges unless this is the handoff edge to gate.
        if (!remove && is_forbidden_base_downward_edge(g, src, sink)) {
            auto bg_it = base_to_gate.find(src.id);
            if (bg_it == base_to_gate.end() || sink.id != bg_it->second) {
                remove = true;
            }
        }

        // Remove residual forbidden base CHANX edges globally (R7 policy).
        if (!remove && is_forbidden_base_chanx_r7_edge(g, src, sink)) {
            remove = true;
        }

        // Enforce PE OPIN gate-only policy:
        // remove any residual direct PE OPIN->layer0 CHAN* edges.
        if (!remove && src.type == "OPIN" && src.layer == 0
            && is_pe_xy(g, src.xlow, src.ylow)
            && sink.layer == 0 && is_channel_type(sink.type)) {
            remove = true;
        }

        // Enforce PE IPIN gate-only policy:
        // remove any residual direct layer0 CHAN*->PE IPIN edges.
        if (!remove && sink.type == "IPIN" && sink.layer == 0
            && is_pe_xy(g, sink.xlow, sink.ylow)
            && src.layer == 0 && is_channel_type(src.type)) {
            remove = true;
        }

        if (remove) {
            removed_r8++;
            continue;
        }

        cleaned_edges.push_back(e);
    }

    // Final deduplication.
    std::vector<EdgeInfo> dedup_edges;
    dedup_edges.reserve(cleaned_edges.size());
    std::unordered_set<EdgeKey, EdgeKeyHash> seen;
    seen.reserve(cleaned_edges.size() * 2 + 1);
    for (const auto& e : cleaned_edges) {
        push_edge_unique(dedup_edges, seen, e.src, e.sink, e.switch_id);
    }

    // Boundary cleanup: remove edges touching CHANY(x=0) on any layer, or layer-0 CHANX(y=0).
    // Mirrors the original boundary mode to prevent routing along the left/bottom edge.
    // This includes gate nodes at x=0 (CHANY(x=0, layer=gate_layer)) — they are removed too.
    std::vector<EdgeInfo> boundary_cleaned;
    boundary_cleaned.reserve(dedup_edges.size());
    size_t removed_boundary = 0;
    for (const auto& e : dedup_edges) {
        auto src_it = g.nodes_by_id.find(e.src);
        auto sink_it = g.nodes_by_id.find(e.sink);
        if (src_it == g.nodes_by_id.end() || sink_it == g.nodes_by_id.end()) {
            boundary_cleaned.push_back(e);
            continue;
        }
        const NodeInfo& src = src_it->second;
        const NodeInfo& sink = sink_it->second;

        bool is_boundary = false;
        if (src.type == "CHANY" && src.xlow == 0 && src.xhigh == 0)
            is_boundary = true;
        if (!is_boundary && sink.type == "CHANY" && sink.xlow == 0 && sink.xhigh == 0)
            is_boundary = true;
        if (!is_boundary && src.type == "CHANX" && src.layer == 0 && src.ylow == 0 && src.yhigh == 0)
            is_boundary = true;
        if (!is_boundary && sink.type == "CHANX" && sink.layer == 0 && sink.ylow == 0 && sink.yhigh == 0)
            is_boundary = true;

        if (is_boundary) {
            removed_boundary++;
        } else {
            boundary_cleaned.push_back(e);
        }
    }

    std::cout << "cerebras mode summary:\n"
              << "  removed R2 (OPIN bypass): " << removed_r2 << "\n"
              << "  removed R3 (IPIN bypass): " << removed_r3 << "\n"
              << "  removed R4 (base-downward): " << removed_r4 << "\n"
              << "  removed R7 (base CHANX forbidden edges): " << removed_r7_base_chanx << "\n"
              << "  added pin<->gate rewires: " << added_pin_to_gate << "\n"
              << "  added base->gate handoff: " << base_to_gate.size() << "\n"
              << "  added base->down_gate handoff (R5): " << added_base_to_down_gate << "\n"
              << "  reattached from gate (R6): " << reattached_from_gate << "\n"
              << "  skipped reattach in R6: " << skipped_r6 << "\n"
              << "  added local gate links (R6.5): " << added_local_gate_links << "\n"
              << "  added base CHANX->gate handoff (R7): " << added_base_chanx_to_gate << "\n"
              << "  added base CHANX->left_gate handoff (R7): " << added_base_chanx_to_left_gate << "\n"
              << "  removed by strict base exact policy (R8): " << removed_r8_base_exact << "\n"
              << "  removed by strict pin policy (R8): " << removed_r8_pin_exact << "\n"
              << "  removed in global cleanup (R8): " << removed_r8 << "\n"
              << "  removed boundary CHAN edges (x=0 CHANY any-layer / y=0 CHANX layer-0): " << removed_boundary << "\n"
              << "  gate nodes tracked: " << gate_node_ids.size() << "\n"
              << "  edges out: " << boundary_cleaned.size() << "\n";

    return write_edges(g, boundary_cleaned);
}

TransformMode parse_mode(const std::string& mode_str) {
    if (mode_str == "boundary") return TransformMode::kBoundary;
    if (mode_str == "cerebras") return TransformMode::kCerebras;
    throw std::runtime_error("Unknown mode: " + mode_str);
}

void print_usage(const char* program_name) {
    std::cout << "Usage:\n"
              << "  " << program_name << " <input.xml> <output.xml>\n"
              << "  " << program_name << " --mode <boundary|cerebras> <input.xml> <output.xml>\n"
              << "  " << program_name << " --mode cerebras --gate-layer <N> <input.xml> <output.xml>\n\n"
              << "Modes:\n"
              << "  boundary  Legacy boundary CHAN pruning (default)\n"
              << "  cerebras  Gate-mediated rewiring + global bypass cleanup\n";
}

bool transform_rr_graph(const char* input_file, const char* output_file, TransformMode mode, int gate_layer) {
    auto start_time = std::chrono::high_resolution_clock::now();

    ParsedGraph g;
    if (!parse_graph(input_file, g)) {
        return false;
    }

    auto parse_time = std::chrono::high_resolution_clock::now();
    std::cout << "XML parsing/indexing completed in "
              << std::chrono::duration<double>(parse_time - start_time).count()
              << " seconds\n";
    std::cout << "Parsed nodes=" << g.nodes_by_id.size() << ", edges=" << g.edges.size()
              << ", pe_tiles=" << g.pe_tiles_xy.size()
              << ", max_grid_layer=" << g.max_grid_layer
              << ", max_node_layer=" << g.max_node_layer << "\n";

    bool ok = false;
    if (mode == TransformMode::kBoundary) {
        ok = transform_boundary(g);
    } else {
        ok = transform_cerebras(g, gate_layer);
    }
    if (!ok) return false;

    if (!g.doc.save_file(output_file, "  ", pugi::format_default | pugi::format_write_bom, pugi::encoding_utf8)) {
        std::cerr << "Error: Failed to write output file " << output_file << "\n";
        return false;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Total transform time: "
              << std::chrono::duration<double>(end_time - start_time).count()
              << " seconds\n";
    return true;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        print_usage(argv[0]);
        return 1;
    }

    std::string mode_str = "boundary";
    int gate_layer = 1;

    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--mode") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --mode requires a value\n";
                return 1;
            }
            mode_str = argv[++i];
        } else if (arg == "--gate-layer") {
            if (i + 1 >= argc) {
                std::cerr << "Error: --gate-layer requires a value\n";
                return 1;
            }
            gate_layer = std::atoi(argv[++i]);
        } else if (!arg.empty() && arg[0] == '-') {
            std::cerr << "Error: Unknown option " << arg << "\n";
            return 1;
        } else {
            positional.push_back(arg);
        }
    }

    if (positional.size() != 2) {
        print_usage(argv[0]);
        return 1;
    }

    TransformMode mode;
    try {
        mode = parse_mode(mode_str);
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        print_usage(argv[0]);
        return 1;
    }

    const char* input_file = positional[0].c_str();
    const char* output_file = positional[1].c_str();

    std::cout << "RR Graph Transform Tool\n";
    std::cout << "Mode:   " << mode_str << "\n";
    std::cout << "Input:  " << input_file << "\n";
    std::cout << "Output: " << output_file << "\n";
    if (mode == TransformMode::kCerebras) {
        std::cout << "Gate layer: " << gate_layer << "\n";
    }
    std::cout << "\n";

    if (transform_rr_graph(input_file, output_file, mode, gate_layer)) {
        std::cout << "Transformation completed successfully!\n";
        return 0;
    }

    std::cerr << "Transformation failed!\n";
    return 1;
}
