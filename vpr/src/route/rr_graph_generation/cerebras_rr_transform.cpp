/**
 * @file cerebras_rr_transform.cpp
 * @brief In-memory Cerebras RR graph transform.
 *
 * Ported from vtr/utils/rr_graph_transform/src/main.cpp.
 * See cerebras_rr_transform.h for rule descriptions.
 */

#include "cerebras_rr_transform.h"
#include "rr_graph_type.h"
#include "physical_types.h"
#include "vtr_log.h"
#include "vtr_time.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

// ---- Data structures (mirroring external tool) ----

struct EdgeInfo {
    RRNodeId src;
    RRNodeId dest;
    short switch_id;
};

struct RemovedEdgeInfo {
    RRNodeId base_src;
    RRNodeId dst;
    short switch_id;
};

struct NodeLookupKey {
    e_rr_type type;
    int layer;
    int x;
    int y;
    int ptc;

    bool operator==(const NodeLookupKey& o) const {
        return type == o.type && layer == o.layer
               && x == o.x && y == o.y && ptc == o.ptc;
    }
};

struct NodeLookupKeyHash {
    std::size_t operator()(const NodeLookupKey& k) const noexcept {
        std::size_t h = static_cast<std::size_t>(k.type);
        h ^= static_cast<std::size_t>(k.layer) * 0x9e3779b1u + (h << 6) + (h >> 2);
        h ^= static_cast<std::size_t>(k.x) * 0x85ebca6bu + (h << 6) + (h >> 2);
        h ^= static_cast<std::size_t>(k.y) * 0xc2b2ae35u + (h << 6) + (h >> 2);
        h ^= static_cast<std::size_t>(k.ptc) * 0x27d4eb2fu + (h << 6) + (h >> 2);
        return h;
    }
};

struct EdgeKey {
    RRNodeId src;
    RRNodeId dest;
    short switch_id;

    bool operator==(const EdgeKey& o) const {
        return src == o.src && dest == o.dest && switch_id == o.switch_id;
    }
};

struct EdgeKeyHash {
    std::size_t operator()(const EdgeKey& k) const noexcept {
        std::size_t h = static_cast<std::size_t>(size_t(k.src));
        h ^= static_cast<std::size_t>(size_t(k.dest)) * 0x9e3779b1u + (h << 6) + (h >> 2);
        h ^= static_cast<std::size_t>(k.switch_id) * 0x85ebca6bu + (h << 6) + (h >> 2);
        return h;
    }
};

long long xy_key(int x, int y) {
    return (static_cast<long long>(x) << 32) ^ static_cast<unsigned int>(y);
}

// ---- Helper: node property access wrappers ----

struct NodeProps {
    e_rr_type type;
    int layer;
    short xlow;
    short ylow;
    short xhigh;
    short yhigh;
    int ptc;
};

NodeProps get_node_props(const RRGraphView& rr_graph, RRNodeId id) {
    NodeProps p;
    p.type = rr_graph.node_type(id);
    p.layer = rr_graph.node_layer(id);
    p.xlow = rr_graph.node_xlow(id);
    p.ylow = rr_graph.node_ylow(id);
    p.xhigh = rr_graph.node_xhigh(id);
    p.yhigh = rr_graph.node_yhigh(id);
    p.ptc = rr_graph.node_ptc_num(id);
    return p;
}

bool is_unit_node(const NodeProps& n) {
    return n.xlow == n.xhigh && n.ylow == n.yhigh;
}

bool is_channel_type(e_rr_type t) {
    return t == e_rr_type::CHANX || t == e_rr_type::CHANY;
}

// ---- PE tile identification ----

bool is_pe_xy(const std::unordered_set<long long>& pe_tiles, int x, int y) {
    return pe_tiles.count(xy_key(x, y)) > 0;
}

bool is_pe_opin(const std::unordered_set<long long>& pe_tiles, const NodeProps& n) {
    return n.type == e_rr_type::OPIN && n.layer == 0 && is_unit_node(n)
           && is_pe_xy(pe_tiles, n.xlow, n.ylow);
}

bool is_pe_ipin(const std::unordered_set<long long>& pe_tiles, const NodeProps& n) {
    return n.type == e_rr_type::IPIN && n.layer == 0 && is_unit_node(n)
           && is_pe_xy(pe_tiles, n.xlow, n.ylow);
}

bool is_base_chany_for_pe(const std::unordered_set<long long>& pe_tiles, const NodeProps& n) {
    return n.type == e_rr_type::CHANY && n.layer == 0 && is_unit_node(n) && n.ptc >= 0
           && is_pe_xy(pe_tiles, n.xlow, n.ylow);
}

bool is_base_chanx_for_pe(const std::unordered_set<long long>& pe_tiles, const NodeProps& n) {
    return n.type == e_rr_type::CHANX && n.layer == 0 && is_unit_node(n) && n.ptc >= 0
           && is_pe_xy(pe_tiles, n.xlow, n.ylow);
}

// ---- Rule predicates (same logic as external tool) ----

bool is_forbidden_base_downward_edge(const std::unordered_set<long long>& pe_tiles,
                                     const NodeProps& src, const NodeProps& sink) {
    if (!is_base_chany_for_pe(pe_tiles, src)) return false;
    if (!(sink.layer == 0 && is_channel_type(sink.type))) return false;
    if (!is_pe_xy(pe_tiles, sink.xlow, sink.ylow)) return false;

    int base_y = src.ylow;
    if (sink.type == e_rr_type::CHANY) {
        return sink.ylow >= (base_y + 1) || sink.ylow == (base_y - 1);
    }
    if (sink.type == e_rr_type::CHANX) {
        return sink.ylow >= (base_y - 1);
    }
    return false;
}

bool is_forbidden_base_chanx_r7_edge(const std::unordered_set<long long>& pe_tiles,
                                     const NodeProps& src, const NodeProps& sink) {
    if (!is_base_chanx_for_pe(pe_tiles, src)) return false;
    if (sink.ptc != src.ptc) return false;
    if (is_channel_type(sink.type) && sink.layer == 0 && !is_pe_xy(pe_tiles, sink.xlow, sink.ylow)) {
        return false;
    }
    if (sink.type == e_rr_type::CHANY && sink.layer == 0) {
        return true;
    }
    if (sink.type == e_rr_type::CHANX && sink.layer == 0 && is_unit_node(sink) && sink.ylow == src.ylow) {
        return (sink.xlow == src.xlow - 1) || (sink.xlow == src.xlow + 1);
    }
    return false;
}

bool should_reattach_removed_edge_from_primary_gate(const NodeProps& base, const NodeProps& dst) {
    if (!(dst.layer == 0 && is_channel_type(dst.type))) return false;
    if (!is_unit_node(base) || !is_unit_node(dst)) return false;

    int base_y = base.ylow;
    if (dst.type == e_rr_type::CHANX) {
        return dst.ylow >= base_y;
    }
    if (dst.type == e_rr_type::CHANY) {
        return dst.xlow == base.xlow
               && (dst.ylow >= (base_y + 1) || dst.ylow == (base_y - 1));
    }
    return false;
}

// ---- Node lookup and gate node creation ----

RRNodeId find_node_id(const std::unordered_map<NodeLookupKey, RRNodeId, NodeLookupKeyHash>& lookup,
                      e_rr_type type, int layer, int x, int y, int ptc) {
    NodeLookupKey key{type, layer, x, y, ptc};
    auto it = lookup.find(key);
    if (it == lookup.end()) return RRNodeId::INVALID();
    return it->second;
}

bool is_allowed_base_chany_gate_sink(
    const std::unordered_set<long long>& pe_tiles,
    const std::unordered_map<NodeLookupKey, RRNodeId, NodeLookupKeyHash>& lookup,
    const NodeProps& src, RRNodeId sink_id, int gate_layer) {
    if (!is_base_chany_for_pe(pe_tiles, src)) return false;
    RRNodeId g0 = find_node_id(lookup, e_rr_type::CHANY, gate_layer, src.xlow, src.ylow, src.ptc);
    if (sink_id == g0) return true;
    if (src.ylow > 0) {
        RRNodeId g1 = find_node_id(lookup, e_rr_type::CHANY, gate_layer, src.xlow, src.ylow - 1, src.ptc);
        if (sink_id == g1) return true;
    }
    return false;
}

bool is_allowed_base_chanx_gate_sink(
    const std::unordered_set<long long>& pe_tiles,
    const std::unordered_map<NodeLookupKey, RRNodeId, NodeLookupKeyHash>& lookup,
    const NodeProps& src, RRNodeId sink_id, int gate_layer) {
    if (!is_base_chanx_for_pe(pe_tiles, src)) return false;
    RRNodeId g0 = find_node_id(lookup, e_rr_type::CHANY, gate_layer, src.xlow, src.ylow, src.ptc);
    if (sink_id == g0) return true;
    if (src.xlow > 0) {
        RRNodeId g1 = find_node_id(lookup, e_rr_type::CHANY, gate_layer, src.xlow - 1, src.ylow, src.ptc);
        if (sink_id == g1) return true;
    }
    return false;
}

/** Create or find a gate CHANY node at (x, y, ptc) on gate_layer. */
RRNodeId get_or_create_gate_chany_node(
    RRGraphBuilder& builder,
    const RRGraphView& rr_graph,
    std::unordered_map<NodeLookupKey, RRNodeId, NodeLookupKeyHash>& lookup,
    std::unordered_set<RRNodeId>& gate_node_ids,
    size_t& next_node_id,
    int x, int y, int ptc, int gate_layer,
    RRIndexedDataId default_cost_index,
    NodeRCIndex default_rc_index) {

    NodeLookupKey key{e_rr_type::CHANY, gate_layer, x, y, ptc};
    auto it = lookup.find(key);
    if (it != lookup.end()) {
        gate_node_ids.insert(it->second);  // track pre-existing layer-1 nodes too
        return it->second;
    }

    // Create new node
    builder.emplace_back();
    RRNodeId new_id = RRNodeId(next_node_id++);

    builder.set_node_type(new_id, e_rr_type::CHANY);
    builder.set_node_coordinates(new_id, x, y, x, y);
    builder.set_node_layer(new_id, gate_layer);
    builder.set_node_ptc_num(new_id, ptc);
    builder.set_node_capacity(new_id, 1);

    // Copy cost_index/rc_index from same-location base CHANY if available, else use defaults
    NodeLookupKey base_key{e_rr_type::CHANY, 0, x, y, ptc};
    auto base_it = lookup.find(base_key);
    if (base_it != lookup.end()) {
        builder.set_node_cost_index(new_id, rr_graph.node_cost_index(base_it->second));
        builder.set_node_rc_index(new_id, NodeRCIndex(rr_graph.node_rc_index(base_it->second)));
    } else {
        builder.set_node_cost_index(new_id, default_cost_index);
        builder.set_node_rc_index(new_id, default_rc_index);
    }

    lookup[key] = new_id;
    gate_node_ids.insert(new_id);
    return new_id;
}

void push_edge_unique(std::vector<EdgeInfo>& out,
                      std::unordered_set<EdgeKey, EdgeKeyHash>& seen,
                      RRNodeId src, RRNodeId dest, short sw) {
    if (!src.is_valid() || !dest.is_valid() || sw < 0) return;
    EdgeKey key{src, dest, sw};
    if (seen.insert(key).second) {
        out.push_back({src, dest, sw});
    }
}

} // anonymous namespace

// ================================================================
// Main transform function
// ================================================================

void cerebras_transform_rr_graph(
    RRGraphBuilder& builder,
    const RRGraphView& rr_graph,
    const DeviceGrid& grid,
    int gate_layer,
    int fixed_region_size,
    int fabric_h,
    int reserved_track_min,
    int reserved_track_max) {

    vtr::ScopedStartFinishTimer timer("Cerebras in-memory RR graph transform");

    // ---- 1. Build PE tile set from grid ----
    std::unordered_set<long long> pe_tiles;
    for (size_t x = 0; x < grid.width(); x++) {
        for (size_t y = 0; y < grid.height(); y++) {
            t_physical_tile_loc loc;
            loc.layer_num = 0;
            loc.x = (int)x;
            loc.y = (int)y;
            auto* tile_type = grid.get_physical_type(loc);
            if (tile_type && tile_type->name == "pe_tile") {
                pe_tiles.insert(xy_key((int)x, (int)y));
            }
        }
    }

    // ---- 2. Build node lookup and cache node properties ----
    size_t num_nodes = rr_graph.num_nodes();
    std::unordered_map<NodeLookupKey, RRNodeId, NodeLookupKeyHash> node_lookup;
    node_lookup.reserve(num_nodes);

    // Cache node properties for fast access during transform
    std::vector<NodeProps> node_cache(num_nodes);
    for (size_t i = 0; i < num_nodes; i++) {
        RRNodeId id = RRNodeId(i);
        node_cache[i] = get_node_props(rr_graph, id);
        const NodeProps& np = node_cache[i];
        if (is_unit_node(np) && np.ptc >= 0) {
            NodeLookupKey key{np.type, np.layer, np.xlow, np.ylow, np.ptc};
            node_lookup[key] = id;
        }
    }

    // Track next node ID for creating gate nodes
    size_t next_node_id = num_nodes;

    // Find a template CHANY node for cost_index/rc_index defaults
    RRIndexedDataId default_chany_cost_index = RRIndexedDataId(0);
    NodeRCIndex default_chany_rc_index = NodeRCIndex(0);
    for (size_t i = 0; i < num_nodes; i++) {
        if (node_cache[i].type == e_rr_type::CHANY && node_cache[i].layer == 0) {
            default_chany_cost_index = rr_graph.node_cost_index(RRNodeId(i));
            default_chany_rc_index = NodeRCIndex(rr_graph.node_rc_index(RRNodeId(i)));
            break;
        }
    }

    // ---- 3. Read all pre-partition edges ----
    size_t num_edges = builder.num_edges_pre_partition();
    std::vector<EdgeInfo> edges;
    edges.reserve(num_edges);
    for (size_t i = 0; i < num_edges; i++) {
        RREdgeId eid = RREdgeId(i);
        // Access through the storage's existing accessors (work before partition)
        RRNodeId src = rr_graph.edge_src_node(eid);
        RRNodeId dest = rr_graph.edge_sink_node(eid);
        short sw = rr_graph.edge_switch(eid);
        edges.push_back({src, dest, sw});
    }

    VTR_LOG("Cerebras transform: %zu nodes, %zu edges, %zu pe_tiles\n",
            num_nodes, num_edges, pe_tiles.size());

    // ---- 4. Infer regular switch ID ----
    short regular_switch = 2; // fallback
    for (const auto& e : edges) {
        size_t si = size_t(e.src);
        size_t di = size_t(e.dest);
        if (si < node_cache.size() && di < node_cache.size()) {
            if (is_channel_type(node_cache[si].type) && is_channel_type(node_cache[di].type)) {
                regular_switch = e.switch_id;
                break;
            }
        }
    }

    // ---- 5. First pass: R2/R3/R4/R7 ----
    std::unordered_set<RRNodeId> gate_node_ids;
    std::unordered_map<RRNodeId, RRNodeId> base_to_gate;
    std::vector<RemovedEdgeInfo> removed_r4_edges;
    std::vector<EdgeInfo> kept_edges;
    kept_edges.reserve(edges.size());

    size_t removed_r2 = 0, removed_r3 = 0, removed_r4 = 0, removed_r7 = 0;
    size_t added_pin_to_gate = 0;

    auto get_props = [&](RRNodeId id) -> const NodeProps& {
        size_t idx = size_t(id);
        if (idx < node_cache.size()) return node_cache[idx];
        // For newly created gate nodes, build props on the fly
        // (gate nodes are always CHANY, unit, on gate_layer)
        static thread_local NodeProps tmp;
        tmp = get_node_props(rr_graph, id);
        return tmp;
    };

    for (const auto& e : edges) {
        const NodeProps& src = get_props(e.src);
        const NodeProps& sink = get_props(e.dest);
        bool remove = false;

        // R2: PE OPIN→CHAN* → replace with OPIN→gate
        if (is_pe_opin(pe_tiles, src) && is_channel_type(sink.type) && sink.layer == 0) {
            int gx = src.xlow;
            int gy = src.ylow; // fixed_gate_y_for_pe_track
            int gtrack = sink.ptc;
            RRNodeId gate_id = get_or_create_gate_chany_node(
                builder, rr_graph, node_lookup, gate_node_ids, next_node_id, gx, gy, gtrack, gate_layer, default_chany_cost_index, default_chany_rc_index);
            if (gate_id.is_valid()) {
                kept_edges.push_back({e.src, gate_id, e.switch_id});
                added_pin_to_gate++;
            }
            remove = true;
            removed_r2++;
        }

        // R3: CHAN*→PE IPIN → replace with gate→IPIN
        if (!remove && is_channel_type(src.type) && src.layer == 0 && is_pe_ipin(pe_tiles, sink)) {
            int gx = sink.xlow;
            int gy = sink.ylow;
            int gtrack = src.ptc;
            RRNodeId gate_id = get_or_create_gate_chany_node(
                builder, rr_graph, node_lookup, gate_node_ids, next_node_id, gx, gy, gtrack, gate_layer, default_chany_cost_index, default_chany_rc_index);
            if (gate_id.is_valid()) {
                kept_edges.push_back({gate_id, e.dest, e.switch_id});
                added_pin_to_gate++;
            }
            remove = true;
            removed_r3++;
        }

        // R4: forbidden downward base-CHANY transitions
        if (!remove && is_forbidden_base_downward_edge(pe_tiles, src, sink)) {
            RRNodeId gate_id;
            auto bg_it = base_to_gate.find(e.src);
            if (bg_it != base_to_gate.end()) {
                gate_id = bg_it->second;
            } else {
                gate_id = get_or_create_gate_chany_node(
                    builder, rr_graph, node_lookup, gate_node_ids, next_node_id,
                    src.xlow, src.ylow, src.ptc, gate_layer, default_chany_cost_index, default_chany_rc_index);
                if (gate_id.is_valid()) {
                    base_to_gate[e.src] = gate_id;
                }
            }
            removed_r4_edges.push_back({e.src, e.dest, e.switch_id});
            remove = true;
            removed_r4++;
        }

        // R7: forbidden base CHANX edges
        if (!remove && is_forbidden_base_chanx_r7_edge(pe_tiles, src, sink)) {
            remove = true;
            removed_r7++;
        }

        if (!remove) {
            kept_edges.push_back(e);
        }
    }

    // We need to update node_cache for newly created gate nodes
    // Resize cache to include new nodes
    node_cache.resize(next_node_id);
    for (size_t i = num_nodes; i < next_node_id; i++) {
        node_cache[i] = get_node_props(rr_graph, RRNodeId(i));
    }

    // ---- R5: mandatory base→gate handoff ----
    size_t added_base_to_down_gate = 0;
    for (const auto& kv : base_to_gate) {
        RRNodeId base_id = kv.first;
        RRNodeId gate_id = kv.second;
        if (base_id == gate_id) continue;
        kept_edges.push_back({base_id, gate_id, regular_switch});

        const NodeProps& base = node_cache[size_t(base_id)];
        if (!is_unit_node(base) || base.ylow <= 0) continue;

        RRNodeId down_gate_id = get_or_create_gate_chany_node(
            builder, rr_graph, node_lookup, gate_node_ids, next_node_id,
            base.xlow, base.ylow - 1, base.ptc, gate_layer, default_chany_cost_index, default_chany_rc_index);
        if (down_gate_id.is_valid()) {
            kept_edges.push_back({base_id, down_gate_id, regular_switch});
            added_base_to_down_gate++;
        }
    }

    // Update cache again for any new nodes from R5
    node_cache.resize(next_node_id);
    for (size_t i = node_cache.size(); i < next_node_id; i++) {
        node_cache[i] = get_node_props(rr_graph, RRNodeId(i));
    }

    // ---- R6: reconnect selected R4 destinations from gate ----
    size_t reattached = 0, skipped_r6 = 0;
    for (const auto& r : removed_r4_edges) {
        auto bg_it = base_to_gate.find(r.base_src);
        if (bg_it == base_to_gate.end()) continue;
        RRNodeId gate_id = bg_it->second;

        const NodeProps& base = node_cache[size_t(r.base_src)];
        const NodeProps& dst = node_cache[size_t(r.dst)];

        if (!should_reattach_removed_edge_from_primary_gate(base, dst)) {
            skipped_r6++;
            continue;
        }
        kept_edges.push_back({gate_id, r.dst, r.switch_id});
        reattached++;
    }

    // ---- R6.5: gate↔layer0 coupling ----
    size_t added_local_gate_links = 0;
    for (RRNodeId gate_id : gate_node_ids) {
        size_t gi = size_t(gate_id);
        if (gi >= node_cache.size()) continue;
        const NodeProps& gate = node_cache[gi];
        if (gate.layer != gate_layer || !is_unit_node(gate) || gate.ptc < 0) continue;

        RRNodeId chany0 = find_node_id(node_lookup, e_rr_type::CHANY, 0, gate.xlow, gate.ylow, gate.ptc);
        if (chany0.is_valid()) {
            kept_edges.push_back({gate_id, chany0, regular_switch});
            kept_edges.push_back({chany0, gate_id, regular_switch});
            added_local_gate_links += 2;
        }

        RRNodeId chanx0 = find_node_id(node_lookup, e_rr_type::CHANX, 0, gate.xlow, gate.ylow, gate.ptc);
        if (chanx0.is_valid()) {
            kept_edges.push_back({gate_id, chanx0, regular_switch});
            kept_edges.push_back({chanx0, gate_id, regular_switch});
            added_local_gate_links += 2;
        }

        RRNodeId chany0_up = find_node_id(node_lookup, e_rr_type::CHANY, 0, gate.xlow, gate.ylow + 1, gate.ptc);
        if (chany0_up.is_valid()) {
            kept_edges.push_back({gate_id, chany0_up, regular_switch});
            kept_edges.push_back({chany0_up, gate_id, regular_switch});
            added_local_gate_links += 2;
        }

        RRNodeId chanx0_right = find_node_id(node_lookup, e_rr_type::CHANX, 0, gate.xlow + 1, gate.ylow, gate.ptc);
        if (chanx0_right.is_valid()) {
            kept_edges.push_back({gate_id, chanx0_right, regular_switch});
            kept_edges.push_back({chanx0_right, gate_id, regular_switch});
            added_local_gate_links += 2;
        }
    }

    // ---- R7 part 2: base CHANX→gate handoff ----
    size_t added_chanx_to_gate = 0, added_chanx_to_left_gate = 0;
    // Collect base CHANX node IDs
    std::vector<RRNodeId> base_chanx_ids;
    for (size_t i = 0; i < num_nodes; i++) {
        if (is_base_chanx_for_pe(pe_tiles, node_cache[i])) {
            base_chanx_ids.push_back(RRNodeId(i));
        }
    }
    for (RRNodeId base_id : base_chanx_ids) {
        const NodeProps& base_cx = node_cache[size_t(base_id)];

        RRNodeId gate_id = get_or_create_gate_chany_node(
            builder, rr_graph, node_lookup, gate_node_ids, next_node_id,
            base_cx.xlow, base_cx.ylow, base_cx.ptc, gate_layer, default_chany_cost_index, default_chany_rc_index);
        if (gate_id.is_valid()) {
            kept_edges.push_back({base_id, gate_id, regular_switch});
            added_chanx_to_gate++;
        }

        if (base_cx.xlow > 0) {
            RRNodeId left_gate_id = get_or_create_gate_chany_node(
                builder, rr_graph, node_lookup, gate_node_ids, next_node_id,
                base_cx.xlow - 1, base_cx.ylow, base_cx.ptc, gate_layer, default_chany_cost_index, default_chany_rc_index);
            if (left_gate_id.is_valid()) {
                kept_edges.push_back({base_id, left_gate_id, regular_switch});
                added_chanx_to_left_gate++;
            }
        }
    }

    // Final cache update for any R7 nodes
    node_cache.resize(next_node_id);
    for (size_t i = node_cache.size(); i < next_node_id; i++) {
        node_cache[i] = get_node_props(rr_graph, RRNodeId(i));
    }

    // ---- R8: global bypass cleanup ----
    std::vector<EdgeInfo> cleaned_edges;
    cleaned_edges.reserve(kept_edges.size());

    size_t removed_r8 = 0, removed_r8_base_exact = 0;
    for (const auto& e : kept_edges) {
        size_t si = size_t(e.src);
        size_t di = size_t(e.dest);
        if (si >= node_cache.size() || di >= node_cache.size()) continue;

        const NodeProps& src = node_cache[si];
        const NodeProps& sink = node_cache[di];
        bool remove = false;

        // Identify gate-layer nodes by layer property (not just tracked set).
        // VPR's original RR graph creates CHANY nodes on all layers with full
        // connectivity.  Every layer-1 edge must pass the gate whitelist.
        bool src_is_gate = (src.layer == gate_layer);
        bool sink_is_gate = (sink.layer == gate_layer);

        // Strict gate policy for multi-layer mode
        if (gate_layer > 0 && (src_is_gate || sink_is_gate)) {
            bool allow = false;

            // Hard ban CHANX on gate layer
            if ((src.layer == gate_layer && src.type == e_rr_type::CHANX)
                || (sink.layer == gate_layer && sink.type == e_rr_type::CHANX)) {
                allow = false;
            } else if (is_pe_opin(pe_tiles, src) && sink_is_gate) {
                allow = true;
            } else if (src_is_gate && is_pe_ipin(pe_tiles, sink)) {
                allow = true;
            } else if (is_channel_type(src.type) && src.layer == 0 && sink_is_gate
                       && is_unit_node(src) && is_unit_node(sink)
                       && src.xlow == sink.xlow && src.ylow == sink.ylow + 1) {
                allow = true;
            } else if (src.type == e_rr_type::CHANX && src.layer == 0 && sink_is_gate
                       && is_unit_node(src) && is_unit_node(sink)
                       && src.ptc == sink.ptc
                       && src.xlow == sink.xlow + 1 && src.ylow == sink.ylow) {
                allow = true;
            } else if (src_is_gate && sink.layer == 0 && is_channel_type(sink.type)
                       && is_unit_node(src) && is_unit_node(sink)
                       && src.ptc == sink.ptc
                       && ((sink.type == e_rr_type::CHANY && sink.xlow == src.xlow
                            && (sink.ylow == src.ylow || sink.ylow == src.ylow + 1))
                           || (sink.type == e_rr_type::CHANX && sink.ylow == src.ylow
                               && (sink.xlow == src.xlow || sink.xlow == src.xlow + 1)))) {
                allow = true;
            } else if (sink_is_gate && src.layer == 0 && is_channel_type(src.type)
                       && is_unit_node(src) && is_unit_node(sink)
                       && src.ptc == sink.ptc
                       && ((src.type == e_rr_type::CHANY && src.xlow == sink.xlow
                            && (src.ylow == sink.ylow || src.ylow == sink.ylow + 1))
                           || (src.type == e_rr_type::CHANX && src.ylow == sink.ylow
                               && (src.xlow == sink.xlow || src.xlow == sink.xlow + 1)))) {
                allow = true;
            }

            if (!allow) remove = true;
        }

        // Strict base CHANY policy
        if (!remove && is_base_chany_for_pe(pe_tiles, src) && sink.type != e_rr_type::IPIN && sink.type != e_rr_type::OPIN) {
            if (sink.layer == 0 && is_channel_type(sink.type) && !is_pe_xy(pe_tiles, sink.xlow, sink.ylow)) {
                // keep boundary escape
            } else if (!is_allowed_base_chany_gate_sink(pe_tiles, node_lookup, src, e.dest, gate_layer)) {
                remove = true;
                removed_r8_base_exact++;
            }
        }

        // Strict base CHANX policy
        if (!remove && is_base_chanx_for_pe(pe_tiles, src) && sink.type != e_rr_type::IPIN && sink.type != e_rr_type::OPIN) {
            if (sink.layer == 0 && is_channel_type(sink.type) && !is_pe_xy(pe_tiles, sink.xlow, sink.ylow)) {
                // keep boundary escape
            } else if (!is_allowed_base_chanx_gate_sink(pe_tiles, node_lookup, src, e.dest, gate_layer)) {
                remove = true;
                removed_r8_base_exact++;
            }
        }

        // Remove residual forbidden downward base edges
        if (!remove && is_forbidden_base_downward_edge(pe_tiles, src, sink)) {
            auto bg_it = base_to_gate.find(e.src);
            if (bg_it == base_to_gate.end() || e.dest != bg_it->second) {
                remove = true;
            }
        }

        // Remove residual forbidden CHANX R7 edges
        if (!remove && is_forbidden_base_chanx_r7_edge(pe_tiles, src, sink)) {
            remove = true;
        }

        // PE OPIN gate-only: remove residual direct PE OPIN→layer0 CHAN*
        if (!remove && src.type == e_rr_type::OPIN && src.layer == 0
            && is_pe_xy(pe_tiles, src.xlow, src.ylow)
            && sink.layer == 0 && is_channel_type(sink.type)) {
            remove = true;
        }

        // PE IPIN gate-only: remove residual direct layer0 CHAN*→PE IPIN
        if (!remove && sink.type == e_rr_type::IPIN && sink.layer == 0
            && is_pe_xy(pe_tiles, sink.xlow, sink.ylow)
            && src.layer == 0 && is_channel_type(src.type)) {
            remove = true;
        }

        if (remove) {
            removed_r8++;
        } else {
            cleaned_edges.push_back(e);
        }
    }

    // ---- Deduplication ----
    std::vector<EdgeInfo> dedup_edges;
    dedup_edges.reserve(cleaned_edges.size());
    std::unordered_set<EdgeKey, EdgeKeyHash> seen;
    seen.reserve(cleaned_edges.size() * 2 + 1);
    for (const auto& e : cleaned_edges) {
        push_edge_unique(dedup_edges, seen, e.src, e.dest, e.switch_id);
    }

    // ---- Boundary cleanup: CHANX(y=0) and CHANY(x=0) on layer 0 ----
    std::vector<EdgeInfo> boundary_cleaned;
    boundary_cleaned.reserve(dedup_edges.size());
    size_t removed_boundary = 0;
    for (const auto& e : dedup_edges) {
        size_t si = size_t(e.src);
        size_t di = size_t(e.dest);
        if (si >= node_cache.size() || di >= node_cache.size()) {
            boundary_cleaned.push_back(e);
            continue;
        }
        const NodeProps& src = node_cache[si];
        const NodeProps& sink = node_cache[di];

        bool is_boundary = false;
        if (src.type == e_rr_type::CHANX && src.layer == 0 && src.ylow == 0 && src.yhigh == 0)
            is_boundary = true;
        if (!is_boundary && sink.type == e_rr_type::CHANX && sink.layer == 0 && sink.ylow == 0 && sink.yhigh == 0)
            is_boundary = true;
        if (!is_boundary && src.type == e_rr_type::CHANY && src.layer == 0 && src.xlow == 0 && src.xhigh == 0)
            is_boundary = true;
        if (!is_boundary && sink.type == e_rr_type::CHANY && sink.layer == 0 && sink.xlow == 0 && sink.xhigh == 0)
            is_boundary = true;

        if (is_boundary) {
            removed_boundary++;
        } else {
            boundary_cleaned.push_back(e);
        }
    }

    // ---- Track reservation (hybrid layout) ----
    std::vector<EdgeInfo>* final_edges = &boundary_cleaned;
    std::vector<EdgeInfo> reserved_cleaned;
    size_t removed_reserved = 0;

    if (fixed_region_size > 0 && reserved_track_min <= reserved_track_max) {
        // Default: top-left corner (x starts at 1, y starts at fabric_h - size + 1)
        int fixed_x_start = 1;
        int fixed_y_start = fabric_h - fixed_region_size + 1;

        // Custom position from env vars
        const char* fxs_env = std::getenv("VPR_FIXED_REGION_X_START");
        if (fxs_env) fixed_x_start = std::atoi(fxs_env);
        const char* fys_env = std::getenv("VPR_FIXED_REGION_Y_START");
        if (fys_env) fixed_y_start = std::atoi(fys_env);

        int x_boundary = fixed_x_start + fixed_region_size;  // one col beyond
        int y_boundary_low = fixed_y_start - 1;  // one row below
        int y_boundary_high = fixed_y_start + fixed_region_size - 1;  // top of fixed region

        VTR_LOG("  fixed region: x=[%d,%d] y=[%d,%d], boundary: x_boundary=%d, y_low=%d, y_high=%d\n",
                fixed_x_start, fixed_x_start + fixed_region_size - 1,
                fixed_y_start, y_boundary_high,
                x_boundary, y_boundary_low, y_boundary_high + 1);

        // Parse boundary kept tracks from env vars
        std::unordered_set<int> boundary_kept_x, boundary_kept_y;
        auto parse_csv_ints = [](const char* csv, std::unordered_set<int>& out) {
            if (!csv) return;
            std::string s(csv);
            size_t pos = 0;
            while ((pos = s.find(',')) != std::string::npos) {
                out.insert(std::atoi(s.substr(0, pos).c_str()));
                s.erase(0, pos + 1);
            }
            if (!s.empty()) out.insert(std::atoi(s.c_str()));
        };
        parse_csv_ints(std::getenv("VPR_BOUNDARY_KEPT_TRACKS_X"), boundary_kept_x);
        parse_csv_ints(std::getenv("VPR_BOUNDARY_KEPT_TRACKS_Y"), boundary_kept_y);

        bool has_boundary_tracks = !boundary_kept_x.empty() || !boundary_kept_y.empty();
        if (has_boundary_tracks) {
            VTR_LOG("  boundary kept tracks X:");
            for (int t : boundary_kept_x) VTR_LOG(" %d", t);
            VTR_LOG(", Y:");
            for (int t : boundary_kept_y) VTR_LOG(" %d", t);
            VTR_LOG("\n");
        }

        // Optional: remove ALL edges in deep interior fixed region
        bool remove_interior_rr = false;
        int interior_buffer = 2;
        const char* rir_env = std::getenv("VPR_REMOVE_INTERIOR_RR");
        if (rir_env && std::strcmp(rir_env, "1") == 0) {
            remove_interior_rr = true;
            const char* buf_env = std::getenv("VPR_INTERIOR_RR_BUFFER");
            if (buf_env) interior_buffer = std::atoi(buf_env);
            if (interior_buffer < 0) interior_buffer = 0;
            VTR_LOG("  remove_interior_rr: enabled (buffer=%d)\n", interior_buffer);
        }

        reserved_cleaned.reserve(boundary_cleaned.size());
        for (const auto& e : boundary_cleaned) {
            size_t si = size_t(e.src);
            size_t di = size_t(e.dest);

            // Track reservation with optional deep-interior removal:
            //   Deep interior:  remove ALL CHANX/CHANY (when remove_interior_rr)
            //   Buffer zone:    remove tracks 1-10 only (normal reservation)
            //   Boundary column: CHANX keep boundary_kept_x, CHANY keep all
            //   Boundary row:    CHANY keep boundary_kept_y, CHANX keep all
            //   Corner:          CHANX remove all, CHANY keep all
            auto should_remove_reserved = [&](size_t idx) -> bool {
                if (idx >= node_cache.size()) return false;
                const NodeProps& n = node_cache[idx];
                if (n.layer != 0) return false;
                if (n.type != e_rr_type::CHANX && n.type != e_rr_type::CHANY) return false;

                bool in_x_interior = (fixed_x_start <= n.xlow && n.xlow <= fixed_x_start + fixed_region_size - 1);
                bool in_x_boundary = (n.xlow == x_boundary);
                bool in_y_interior = (fixed_y_start <= n.ylow && n.ylow <= y_boundary_high);
                bool in_y_boundary_low = (n.ylow == y_boundary_low);
                bool in_y_boundary_high = (n.ylow == y_boundary_high + 1);
                bool in_y_boundary = in_y_boundary_low || in_y_boundary_high;

                if (!(in_x_interior || in_x_boundary)) return false;
                if (!(in_y_interior || in_y_boundary)) return false;

                // Deep interior: remove ALL tracks (not just reserved range)
                // Buffer zone near boundary keeps normal behavior
                if (remove_interior_rr && in_x_interior && in_y_interior) {
                    bool in_x_buffer = (n.xlow > fixed_x_start + fixed_region_size - 1 - interior_buffer);
                    bool in_y_buffer = (n.ylow < fixed_y_start + interior_buffer);
                    if (!in_x_buffer && !in_y_buffer) {
                        return true;
                    }
                }

                // Buffer zone + boundary: only remove reserved track range
                if (n.ptc < reserved_track_min || n.ptc > reserved_track_max) return false;

                // Without boundary tracks info, fall back to blanket removal
                if (!has_boundary_tracks) return true;

                // Corner (x=boundary, y=boundary): no fixed PEs here
                //   CHANX: remove all, CHANY: keep all
                if (in_x_boundary && in_y_boundary) {
                    return (n.type == e_rr_type::CHANX);
                }

                // Boundary column (x=boundary, y=interior): reserve tracks 1-5
                //   same as interior — remove reserved tracks for both CHANX and CHANY
                //   (boundary_kept_x is computed but not used here to avoid
                //    free PE nets routing on tracks reserved for fixed-region comm)
                if (in_x_boundary) {
                    return true;
                }

                // Boundary row (y=boundary, x=interior): y-axis traffic crosses
                //   CHANY: keep boundary_kept_y only, CHANX: keep all
                if (in_y_boundary) {
                    if (n.type == e_rr_type::CHANX) return false;
                    return boundary_kept_y.find(n.ptc) == boundary_kept_y.end();
                }

                // Interior (within buffer): remove reserved tracks only
                return true;
            };

            if (should_remove_reserved(si) || should_remove_reserved(di)) {
                removed_reserved++;
            } else {
                reserved_cleaned.push_back(e);
            }
        }
        final_edges = &reserved_cleaned;
    }

    // ---- Print summary ----
    VTR_LOG("cerebras in-memory transform summary:\n"
            "  removed R2 (OPIN bypass): %zu\n"
            "  removed R3 (IPIN bypass): %zu\n"
            "  removed R4 (base-downward): %zu\n"
            "  removed R7 (base CHANX forbidden): %zu\n"
            "  added pin<->gate rewires: %zu\n"
            "  added base->gate handoff: %zu\n"
            "  added base->down_gate handoff (R5): %zu\n"
            "  reattached from gate (R6): %zu\n"
            "  skipped reattach R6: %zu\n"
            "  added local gate links (R6.5): %zu\n"
            "  added base CHANX->gate (R7): %zu\n"
            "  added base CHANX->left_gate (R7): %zu\n"
            "  removed R8 base exact: %zu\n"
            "  removed R8 total: %zu\n"
            "  removed boundary: %zu\n"
            "  removed reserved tracks: %zu\n"
            "  gate nodes: %zu\n"
            "  edges out: %zu\n",
            removed_r2, removed_r3, removed_r4, removed_r7,
            added_pin_to_gate, base_to_gate.size(), added_base_to_down_gate,
            reattached, skipped_r6, added_local_gate_links,
            added_chanx_to_gate, added_chanx_to_left_gate,
            removed_r8_base_exact, removed_r8,
            removed_boundary, removed_reserved,
            gate_node_ids.size(), final_edges->size());

    // ---- Clear old edges and rebuild ----
    builder.clear_edges_only();
    for (const auto& e : *final_edges) {
        builder.emplace_back_edge(e.src, e.dest, e.switch_id, false);
    }
    // Recompute fan-in for all nodes (including new gate nodes)
    builder.init_fan_in();

    VTR_LOG("Cerebras transform: rebuilt %zu edges (was %zu)\n",
            final_edges->size(), num_edges);
}
