#pragma once

/** @file Header implementations for templated net routing fns. */

#include "route_net.h"

#include <tuple>
#include <functional>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include "connection_based_routing.h"
#include "connection_router_interface.h"
#include "describe_rr_node.h"
#include "draw.h"
#include "route_common.h"
#include "route_debug.h"
#include "route_profiling.h"
#include "routing_predictor.h"
#include "rr_graph_fwd.h"
#include "spatial_route_tree_lookup.h"
#include "vtr_dynamic_bitset.h"

/** Helper function to get all candidate tracks from OPIN node
 * Returns a vector of (track_number, congestion_score) pairs, sorted by congestion (lower is better)
 * This is used to intelligently select tracks for multicast nets */
inline std::vector<std::pair<int, float>> get_opin_candidate_tracks_with_congestion(
    RRNodeId opin_node,
    const std::vector<RRNodeId>& sink_nodes,
    const RRGraphView& rr_graph,
    const t_rr_node_route_inf* rr_node_route_inf) {

    std::vector<std::pair<int, float>> track_scores;

    // Get all edges from the OPIN
    auto& device_ctx = g_vpr_ctx.device();
    auto& route_ctx = g_vpr_ctx.routing();

    // Compute bounding box that covers all sinks
    int min_x = std::numeric_limits<int>::max();
    int max_x = std::numeric_limits<int>::min();
    int min_y = std::numeric_limits<int>::max();
    int max_y = std::numeric_limits<int>::min();

    for (RRNodeId sink : sink_nodes) {
        int x = rr_graph.node_xlow(sink);
        int y = rr_graph.node_ylow(sink);
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
    }

    // Also include OPIN location
    int opin_x = rr_graph.node_xlow(opin_node);
    int opin_y = rr_graph.node_ylow(opin_node);
    min_x = std::min(min_x, opin_x);
    max_x = std::max(max_x, opin_x);
    min_y = std::min(min_y, opin_y);
    max_y = std::max(max_y, opin_y);

    // Collect all unique tracks connected to this OPIN
    std::unordered_set<int> seen_tracks;

    for (RREdgeId edge : rr_graph.edge_range(opin_node)) {
        RRNodeId to_node = rr_graph.edge_sink_node(edge);
        e_rr_type to_type = rr_graph.node_type(to_node);

        if (to_type == e_rr_type::CHANX || to_type == e_rr_type::CHANY) {
            int track = rr_graph.node_track_num(to_node);
            if (seen_tracks.find(track) == seen_tracks.end()) {
                seen_tracks.insert(track);

                // Compute congestion score for this track within the bounding box
                // Lower score = less congestion = better
                float congestion_score = 0.0f;
                int segments_checked = 0;

                // Check CHANX segments in the bounding box
                for (int y = min_y; y <= max_y; y++) {
                    for (int x = min_x; x <= max_x; x++) {
                        // Try to find CHANX node at this position with this track
                        RRNodeId chanx_node = rr_graph.node_lookup().find_node(
                            0, x, y, e_rr_type::CHANX, track);
                        if (chanx_node.is_valid()) {
                            // Check congestion: occ / capacity
                            int occ = route_ctx.rr_node_route_inf[chanx_node].occ();
                            int cap = rr_graph.node_capacity(chanx_node);
                            if (cap > 0) {
                                congestion_score += (float)occ / cap;
                                segments_checked++;
                            }
                        }

                        // Try to find CHANY node at this position with this track
                        RRNodeId chany_node = rr_graph.node_lookup().find_node(
                            0, x, y, e_rr_type::CHANY, track);
                        if (chany_node.is_valid()) {
                            int occ = route_ctx.rr_node_route_inf[chany_node].occ();
                            int cap = rr_graph.node_capacity(chany_node);
                            if (cap > 0) {
                                congestion_score += (float)occ / cap;
                                segments_checked++;
                            }
                        }
                    }
                }

                // Normalize by number of segments checked
                if (segments_checked > 0) {
                    congestion_score /= segments_checked;
                }

                track_scores.push_back({track, congestion_score});
            }
        }
    }

    // Sort by congestion score (lower is better)
    std::sort(track_scores.begin(), track_scores.end(),
              [](const auto& a, const auto& b) { return a.second < b.second; });

    return track_scores;
}

/** Helper function to find the OPIN node connected to the SOURCE in the RR graph
 * The route tree initially only has SOURCE, so we look at RR graph edges to find OPIN
 * Returns RRNodeId::INVALID() if not found */
inline RRNodeId find_opin_from_source(const RouteTree& tree, const RRGraphView& rr_graph) {
    // Get the SOURCE node from the route tree root
    RRNodeId source_node = tree.root().inode;
    e_rr_type source_type = rr_graph.node_type(source_node);

    // Verify it's a SOURCE
    if (source_type != e_rr_type::SOURCE) {
        return RRNodeId::INVALID();
    }

    // Look at edges from SOURCE to find OPIN
    for (RREdgeId edge : rr_graph.edge_range(source_node)) {
        RRNodeId to_node = rr_graph.edge_sink_node(edge);
        e_rr_type to_type = rr_graph.node_type(to_node);

        if (to_type == e_rr_type::OPIN) {
            return to_node;
        }
    }

    return RRNodeId::INVALID();
}

/** Helper function to extract track assignments from existing route tree
 * Returns a map from OPIN RRNodeId to the track number used by its first CHANX/CHANY child */
inline std::unordered_map<RRNodeId, int> extract_opin_tracks_from_route_tree(const RouteTree& tree) {
    std::unordered_map<RRNodeId, int> opin_tracks;
    const auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;
    
    std::function<void(const RouteTreeNode&)> traverse = [&](const RouteTreeNode& node) {
        RRNodeId node_id = node.inode;
        e_rr_type node_type = rr_graph.node_type(node_id);
        
        // If this is an OPIN, find its first CHANX/CHANY child
        if (node_type == e_rr_type::OPIN) {
            for (const auto& child : node.child_nodes()) {
                e_rr_type child_type = rr_graph.node_type(child.inode);
                if (child_type == e_rr_type::CHANX || child_type == e_rr_type::CHANY) {
                    int track = rr_graph.node_track_num(child.inode);
                    opin_tracks[node_id] = track;
                    break; // Only need the first track
                }
            }
        }
        
        // Recursively traverse children
        for (const auto& child : node.child_nodes()) {
            traverse(child);
        }
    };
    
    traverse(tree.root());
    return opin_tracks;
}

inline bool is_heuristic_bcast_net_name(const std::string& net_name) {
    static const bool k_heuristic_routing = []() {
        const char* v = std::getenv("VPR_HEURISTIC_ROUTING");
        return v != nullptr && std::atoi(v) > 0;
    }();
    if (!k_heuristic_routing) return false;

    // nullptr  → env var not set → backward-compat: all "bcast" nets use W-1
    // ""       → env var set to empty → explicit empty list: no heuristic bcast nets
    // "a,b,c"  → explicit allow-list: only those nets use W-1
    static const char* k_raw = std::getenv("VPR_HEURISTIC_BCAST_NETS");

    static const std::unordered_set<std::string> k_heuristic_nets = []() {
        std::unordered_set<std::string> out;
        if (k_raw == nullptr || k_raw[0] == '\0') return out;
        std::stringstream ss(k_raw);
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            if (!tok.empty()) out.insert(tok);
        }
        return out;
    }();

    // If env var was set (even to empty string), honour the explicit list.
    // If env var was never set (nullptr), fall back to "all bcast nets" behaviour.
    if (k_raw != nullptr) {
        return k_heuristic_nets.find(net_name) != k_heuristic_nets.end();
    }
    return net_name.find("bcast") != std::string::npos;
}

/**
 * Return true when net_name is in the heuristic reduce allowlist, and set
 * forced_track_offset to 2 (W-2) or 3 (W-3) depending on which list it
 * belongs to.
 *
 * Controlled by:
 *   VPR_HEURISTIC_ROUTING=1               (master switch, same as broadcast)
 *   VPR_HEURISTIC_REDUCE_NETS_W2=...     comma-separated list → track W-2
 *   VPR_HEURISTIC_REDUCE_NETS_W3=...     comma-separated list → track W-3
 */
inline bool is_heuristic_reduce_net_name(const std::string& net_name,
                                          int& forced_track_offset) {
    static const bool k_heuristic_routing = []() {
        const char* v = std::getenv("VPR_HEURISTIC_ROUTING");
        return v != nullptr && std::atoi(v) > 0;
    }();
    if (!k_heuristic_routing) return false;

    static const std::unordered_set<std::string> k_reduce_w2_nets = []() {
        std::unordered_set<std::string> out;
        const char* raw = std::getenv("VPR_HEURISTIC_REDUCE_NETS_W2");
        if (raw == nullptr) return out;
        std::stringstream ss(raw);
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            if (!tok.empty()) out.insert(tok);
        }
        return out;
    }();

    static const std::unordered_set<std::string> k_reduce_w3_nets = []() {
        std::unordered_set<std::string> out;
        const char* raw = std::getenv("VPR_HEURISTIC_REDUCE_NETS_W3");
        if (raw == nullptr) return out;
        std::stringstream ss(raw);
        std::string tok;
        while (std::getline(ss, tok, ',')) {
            if (!tok.empty()) out.insert(tok);
        }
        return out;
    }();

    if (k_reduce_w2_nets.count(net_name)) {
        forced_track_offset = 2;
        return true;
    }
    if (k_reduce_w3_nets.count(net_name)) {
        forced_track_offset = 3;
        return true;
    }
    return false;
}

/** Attempt to route a single net.
 *
 * @param router The ConnectionRouterType instance
 * @param net_list Input netlist
 * @param net_id
 * @param itry # of iteration
 * @param pres_fac
 * @param router_opts
 * @param connections_inf
 * @param router_stats
 * @param pin_criticality
 * @param rt_node_of_sink Lookup from target_pin-like indices (indicating SINK nodes) to RouteTreeNodes
 * @param net_delay
 * @param netlist_pin_lookup
 * @param timing_info
 * @param pin_timing_invalidator
 * @param budgeting_inf
 * @param worst_neg_slack
 * @param routing_predictor
 * @param choking_spots
 * @param is_flat
 * @param net_bb Bounding box for the net (Routing resources outside net_bb will not be used)
 * @param should_setup Should we reset/prune the existing route tree first?
 * @param sink_mask Which sinks to route? Assumed all sinks if nullopt, otherwise a mask of [1..num_sinks+1] where set bits request the sink to be routed
 * @return NetResultFlags for this net */
template<typename ConnectionRouterType>
inline NetResultFlags route_net(ConnectionRouterType& router,
                                const Netlist<>& net_list,
                                const ParentNetId& net_id,
                                int itry,
                                float pres_fac,
                                const t_router_opts& router_opts,
                                CBRR& connections_inf,
                                RouterStats& router_stats,
                                NetPinsMatrix<float>& net_delays,
                                const ClusteredPinAtomPinsLookup& netlist_pin_lookup,
                                SetupHoldTimingInfo* timing_info,
                                NetPinTimingInvalidator* pin_timing_invalidator,
                                route_budgets& budgeting_inf,
                                float worst_negative_slack,
                                const RoutingPredictor& routing_predictor,
                                const std::vector<std::unordered_map<RRNodeId, int>>& choking_spots,
                                bool is_flat,
                                const t_bb& net_bb,
                                bool should_setup = true,
                                vtr::optional<const vtr::dynamic_bitset<>&> sink_mask = vtr::nullopt) {
    auto& route_ctx = g_vpr_ctx.mutable_routing();

    NetResultFlags flags;
    flags.success = true;

    if (!should_route_net(net_list, net_id, connections_inf, budgeting_inf, worst_negative_slack, true))
        return flags;

    // track time spent vs fanout
    profiling::net_fanout_start();

    flags.was_rerouted = true; // Flag to record whether routing was actually changed

    auto& device_ctx = g_vpr_ctx.device();
    const auto& rr_graph = device_ctx.rr_graph;

    unsigned int num_sinks = net_list.net_sinks(net_id).size();

    VTR_LOGV_DEBUG(f_router_debug, "Routing Net %zu (%zu sinks)\n", size_t(net_id), num_sinks);

    /* Prune or rip-up existing routing for the net */
    if (should_setup) {
        setup_net(
            itry,
            net_id,
            net_list,
            connections_inf,
            router_opts,
            worst_negative_slack);
    }

    VTR_ASSERT(route_ctx.route_trees[net_id]);
    RouteTree& tree = route_ctx.route_trees[net_id].value();

    bool high_fanout = is_high_fanout(num_sinks, router_opts.high_fanout_threshold);

    SpatialRouteTreeLookup spatial_route_tree_lookup;
    if (high_fanout) {
        spatial_route_tree_lookup = build_route_tree_spatial_lookup(net_list,
                                                                    route_ctx.route_bb,
                                                                    net_id,
                                                                    tree.root());
    }

    /* 1-indexed! */
    std::vector<float> pin_criticality(tree.num_sinks() + 1, 0);

    // after this point the route tree is correct
    // remaining_targets from this point on are the **pin indices** that have yet to be routed
    auto remaining_targets_mask = ~tree.get_is_isink_reached();
    if (sink_mask)
        remaining_targets_mask &= sink_mask.value();

    auto remaining_targets = sink_mask_to_vector(remaining_targets_mask, num_sinks);

    // calculate criticality of remaining target pins
    for (int ipin : remaining_targets) {
        auto pin = net_list.net_pin(net_id, ipin);
        pin_criticality[ipin] = get_net_pin_criticality(timing_info,
                                                        netlist_pin_lookup,
                                                        router_opts.max_criticality,
                                                        router_opts.criticality_exp,
                                                        net_id,
                                                        pin,
                                                        is_flat);
    }

    // Check if this is a multicast/broadcast net
    std::string net_name = net_list.net_name(net_id);
    bool is_multicast_net = (net_name.find("bcast") != std::string::npos) && (num_sinks > 1);
    bool is_heuristic_bcast = is_multicast_net && is_heuristic_bcast_net_name(net_name);

    if (is_heuristic_bcast) {
        // Under heuristic routing: sort by Manhattan distance from source, nearest first.
        RRNodeId source_rr = route_ctx.net_rr_terminals[net_id][0];
        int src_x = rr_graph.node_xlow(source_rr);
        int src_y = rr_graph.node_ylow(source_rr);

        std::stable_sort(begin(remaining_targets), end(remaining_targets), [&](int a, int b) {
            RRNodeId sink_a = route_ctx.net_rr_terminals[net_id][a];
            RRNodeId sink_b = route_ctx.net_rr_terminals[net_id][b];

            int ax = rr_graph.node_xlow(sink_a);
            int ay = rr_graph.node_ylow(sink_a);
            int bx = rr_graph.node_xlow(sink_b);
            int by = rr_graph.node_ylow(sink_b);

            int dist_a = std::abs(ax - src_x) + std::abs(ay - src_y);
            int dist_b = std::abs(bx - src_x) + std::abs(by - src_y);
            if (dist_a != dist_b) return dist_a < dist_b;

            // Deterministic tie-breakers.
            if (ay != by) return ay < by;
            return ax < bx;
        });
        VTR_LOG("Multicast net '%s': sorted sinks by Manhattan distance from source (%d,%d), nearest first\n",
                net_name.c_str(), src_x, src_y);
    } else {
        // Original VPR behavior: sort by criticality.
        std::stable_sort(begin(remaining_targets), end(remaining_targets), [&](int a, int b) {
            return pin_criticality[a] > pin_criticality[b];
        });
    }

    /* Update base costs according to fanout and criticality rules */
    update_rr_base_costs(num_sinks);

    t_conn_delay_budget conn_delay_budget;
    t_conn_cost_params cost_params;
    cost_params.astar_fac = router_opts.astar_fac;
    cost_params.astar_offset = router_opts.astar_offset;
    cost_params.post_target_prune_fac = router_opts.post_target_prune_fac;
    cost_params.post_target_prune_offset = router_opts.post_target_prune_offset;
    cost_params.bend_cost = router_opts.bend_cost;
    cost_params.pres_fac = pres_fac;
    cost_params.delay_budget = ((budgeting_inf.if_set()) ? &conn_delay_budget : nullptr);

    // Pre-route to clock source for clock nets (marked as global nets)
    if (net_list.net_is_global(net_id) && router_opts.two_stage_clock_routing) {
        auto& route_constraints = route_ctx.constraints;
        std::string net_name = net_list.net_name(net_id);

        // If there is no routing constraint for the current global net
        // and the clock modelling is set to dedicated network or
        //there is a routing constraint for the current net setting routing model
        // to the dedicated network run the first stage router.
        if ((!route_constraints.has_routing_constraint(net_name) && router_opts.clock_modeling == e_clock_modeling::DEDICATED_NETWORK)
            || route_constraints.get_route_model_by_net_name(net_name) == e_clock_modeling::DEDICATED_NETWORK) {
            std::string clock_network_name = "";

            // If a user-specified routing constratins exists for the curret net get the clock network name
            // from the constraints file, otherwise use the default clock network name
            if (route_constraints.has_routing_constraint(net_name)) {
                clock_network_name = route_constraints.get_routing_network_name_by_net_name(net_name);
            } else {
                auto& arch = device_ctx.arch;
                clock_network_name = arch->default_clock_network_name;
            }

            RRNodeId sink_node = rr_graph.virtual_clock_network_root_idx(clock_network_name.c_str());

            enable_router_debug(router_opts, net_id, sink_node, itry, &router);

            VTR_LOGV_DEBUG(f_router_debug, "Pre-routing global net %zu\n", size_t(net_id));

            // Set to the max timing criticality which should intern minimize clock insertion
            // delay by selecting a direct route from the clock source to the virtual sink
            cost_params.criticality = router_opts.max_criticality;

            if (sink_node == RRNodeId::INVALID()) {
                VPR_FATAL_ERROR(VPR_ERROR_ROUTE, "Cannot route net \"%s\" through given clock network. Unknown clock network name \"%s\"", net_name.c_str(), clock_network_name.c_str());
            }

            flags = pre_route_to_clock_root(router,
                                            net_id,
                                            net_list,
                                            sink_node,
                                            cost_params,
                                            router_opts.high_fanout_threshold,
                                            tree,
                                            spatial_route_tree_lookup,
                                            router_stats,
                                            is_flat);

            if (flags.success == false)
                return flags;
        }
    }

    if (budgeting_inf.if_set()) {
        budgeting_inf.set_should_reroute(net_id, false);
    }

    // Track blacklist for multi-fanout retry mechanism
    std::unordered_map<RRNodeId, std::unordered_set<int>> blacklisted_tracks;
    // Use maximum channel width as the retry limit to allow trying all available tracks
    // This ensures we can try all tracks (0 to max_chan_width-1) before giving up
    int MAX_TRACK_RETRY_ATTEMPTS = device_ctx.chan_width.max;
    int retry_attempt = 0;

    // === SMART TRACK SELECTION FOR MULTICAST NETS ===
    // For broadcast/multicast nets, pre-analyze track feasibility and prefer
    // tracks with lower congestion across the routing area
    // Note: is_multicast_net and net_name are already defined above during sink ordering

    // Store fallback tracks separately so we can unlock them if initial selection fails
    std::unordered_map<RRNodeId, std::unordered_set<int>> fallback_tracks;
    bool fallback_unlocked = false;
    RRNodeId multicast_opin_node = RRNodeId::INVALID();
    size_t initial_tracks_count = 0;

    // === TRACK GROUP RELAXATION STATE ===
    // When single-track routing fails for all tracks, allow creating multiple track groups
    // Each track group uses a different track from the OPIN
    std::unordered_map<RRNodeId, std::unordered_set<int>> existing_opin_tracks;
    bool allow_new_track_group = false;
    bool track_group_relaxed = false;
    unsigned relaxation_start_sink = 0;  // The sink index where relaxation was triggered
    unsigned best_itarget_reached = 0;   // Highest itarget successfully routed across all retries
    bool fallback_attempted = false;     // Prevent infinite fallback loops
    unsigned track_groups_created = 0;   // Count of track groups created
    const unsigned MAX_TRACK_GROUPS = 5; // Limit to match CSL template's 3 color slots (_0, _1, _2)

    if (is_multicast_net && !is_heuristic_bcast) {
        VTR_LOG("Detected multicast net '%s' with %u sinks\n", net_name.c_str(), num_sinks);

        // Collect all sink RR nodes
        std::vector<RRNodeId> sink_nodes;
        for (unsigned i = 0; i < remaining_targets.size(); i++) {
            int pin = remaining_targets[i];
            RRNodeId sink_rr = route_ctx.net_rr_terminals[net_id][pin];
            sink_nodes.push_back(sink_rr);
        }

        // Find the OPIN node by looking at RR graph edges from SOURCE
        multicast_opin_node = find_opin_from_source(tree, rr_graph);

        if (!multicast_opin_node.is_valid()) {
            VTR_LOG("  Warning: Could not find OPIN for multicast net '%s'\n", net_name.c_str());
        }

        if (multicast_opin_node.is_valid() && !sink_nodes.empty()) {
            // Get candidate tracks sorted by congestion score (lower is better)
            auto track_scores = get_opin_candidate_tracks_with_congestion(
                multicast_opin_node, sink_nodes, rr_graph, route_ctx.rr_node_route_inf.data());

            if (!track_scores.empty()) {
                VTR_LOG("Smart track selection for multicast net '%s' (OPIN %zu):\n",
                        net_name.c_str(), size_t(multicast_opin_node));

                // Log top tracks by congestion score
                for (size_t i = 0; i < std::min(track_scores.size(), size_t(5)); i++) {
                    VTR_LOG("  Track %d: congestion score %.3f\n",
                            track_scores[i].first, track_scores[i].second);
                }

                // Strategy: Only allow the top N tracks with lowest congestion
                // This focuses the router on the most promising tracks first
                const size_t MAX_INITIAL_TRACKS = 5; // Try top 5 least congested tracks first
                initial_tracks_count = std::min(track_scores.size(), MAX_INITIAL_TRACKS);

                if (track_scores.size() > MAX_INITIAL_TRACKS) {
                    VTR_LOG("  Pre-blacklisting %zu tracks with higher congestion (will unlock as fallback if needed)\n",
                            track_scores.size() - MAX_INITIAL_TRACKS);

                    // Store fallback tracks (can be unlocked later if initial selection fails)
                    for (size_t i = MAX_INITIAL_TRACKS; i < track_scores.size(); i++) {
                        fallback_tracks[multicast_opin_node].insert(track_scores[i].first);
                        blacklisted_tracks[multicast_opin_node].insert(track_scores[i].first);
                    }
                }
            }
        }
    }
    // === END SMART TRACK SELECTION ===

    // explore in order of decreasing criticality (no longer need sink_order array)
    for (unsigned itarget = 0; itarget < remaining_targets.size(); ++itarget) {
        int target_pin = remaining_targets[itarget];

        RRNodeId sink_rr = route_ctx.net_rr_terminals[net_id][target_pin];

        enable_router_debug(router_opts, net_id, sink_rr, itry, &router);

        cost_params.criticality = pin_criticality[target_pin];

        if (budgeting_inf.if_set()) {
            conn_delay_budget.max_delay = budgeting_inf.get_max_delay_budget(net_id, target_pin);
            conn_delay_budget.target_delay = budgeting_inf.get_delay_target(net_id, target_pin);
            conn_delay_budget.min_delay = budgeting_inf.get_min_delay_budget(net_id, target_pin);
            conn_delay_budget.short_path_criticality = budgeting_inf.get_crit_short_path(net_id, target_pin);
            conn_delay_budget.routing_budgets_algorithm = router_opts.routing_budgets_algorithm;
        }

        profiling::conn_start();

        // === FALLBACK RELAXATION: Check if we've reached the pre-set split point ===
        // In fallback mode, track_group_relaxed is true but allow_new_track_group is false
        // When we reach relaxation_start_sink, enable the new track group
        if (track_group_relaxed && !allow_new_track_group && itarget == relaxation_start_sink) {
            VTR_LOG("  Reached pre-set split point at sink %u, enabling new track group\n", itarget);
            allow_new_track_group = true;
            track_groups_created++;  // Now creating group 2

            // Record the tracks used so far (sinks 0 to itarget-1)
            auto opin_tracks = extract_opin_tracks_from_route_tree(tree);
            for (const auto& [opin_id, track_num] : opin_tracks) {
                existing_opin_tracks[opin_id].insert(track_num);
                VTR_LOG("  Track group 1: OPIN %zu uses track %d (sinks 0-%u)\n",
                        size_t(opin_id), track_num, itarget - 1);
            }
        }

        // build a branch in the route tree to the target
        auto sink_flags = route_sink(router,
                                     net_list,
                                     net_id,
                                     itarget,
                                     target_pin,
                                     cost_params,
                                     router_opts,
                                     tree,
                                     spatial_route_tree_lookup,
                                     router_stats,
                                     budgeting_inf,
                                     routing_predictor,
                                     choking_spots,
                                     is_flat,
                                     net_bb,
                                     blacklisted_tracks,
                                     existing_opin_tracks,
                                     allow_new_track_group);

        flags.retry_with_full_bb |= sink_flags.retry_with_full_bb;

        if (!sink_flags.success) {
            // Multi-fanout retry mechanism: try ripping up and re-routing with the current track blacklisted
            // For multicast nets, this applies even when the first sink fails (itarget == 0)
            bool can_retry = retry_attempt < MAX_TRACK_RETRY_ATTEMPTS;
            bool should_retry = ((itarget > 0) || (is_multicast_net && itarget == 0)) && !is_heuristic_bcast;

            if (should_retry && can_retry) {
                VTR_LOG("Routing failed for sink %d of net %d, attempting retry with track blacklisting\n",
                        target_pin, size_t(net_id));

                // Extract which tracks were used by OPINs in the current (failed) routing
                auto opin_tracks = extract_opin_tracks_from_route_tree(tree);

                // Blacklist these tracks (only blacklist tracks NOT in existing_opin_tracks when in relaxed mode)
                for (const auto& [opin_id, track_num] : opin_tracks) {
                    // In relaxed mode, don't blacklist tracks from the first group
                    if (track_group_relaxed) {
                        auto it = existing_opin_tracks.find(opin_id);
                        if (it != existing_opin_tracks.end() && it->second.find(track_num) != it->second.end()) {
                            // This is a track from group 1, don't blacklist it
                            continue;
                        }
                    }
                    blacklisted_tracks[opin_id].insert(track_num);
                    VTR_LOG("  Blacklisting track %d for OPIN %d\n", track_num, size_t(opin_id));
                }

                // For multicast nets: check if we've exhausted initial tracks and need to unlock fallback
                // (Only applies before relaxation is triggered)
                if (is_multicast_net && !track_group_relaxed && !fallback_unlocked && multicast_opin_node.is_valid()) {
                    auto it = blacklisted_tracks.find(multicast_opin_node);
                    if (it != blacklisted_tracks.end()) {
                        // Count how many of the initial tracks have been blacklisted
                        size_t initial_blacklisted = 0;
                        for (int track : it->second) {
                            // Check if this track was NOT a fallback track (i.e., it was an initial track)
                            auto fb_it = fallback_tracks.find(multicast_opin_node);
                            if (fb_it == fallback_tracks.end() ||
                                fb_it->second.find(track) == fb_it->second.end()) {
                                initial_blacklisted++;
                            }
                        }

                        // If all initial tracks are blacklisted, unlock fallback tracks
                        if (initial_blacklisted >= initial_tracks_count && !fallback_tracks.empty()) {
                            VTR_LOG("  All %zu initial tracks exhausted, unlocking %zu fallback tracks\n",
                                    initial_tracks_count, fallback_tracks[multicast_opin_node].size());

                            // Remove fallback tracks from blacklist
                            for (int fallback_track : fallback_tracks[multicast_opin_node]) {
                                blacklisted_tracks[multicast_opin_node].erase(fallback_track);
                            }
                            fallback_unlocked = true;
                        }
                    }
                }

                retry_attempt++;

                if (track_group_relaxed) {
                    // In relaxed mode: DON'T rip up the tree, just retry current sink
                    // The sinks in group 1 (0 to relaxation_start_sink-1) stay routed

                    if (itarget == relaxation_start_sink) {
                        // First sink of new group failed - retry with different new track
                        VTR_LOG("  Retry attempt %d/%d (relaxed mode): retrying first sink of group 2 (sink %u) with blacklisted tracks\n",
                                retry_attempt, MAX_TRACK_RETRY_ATTEMPTS, itarget);

                        // Re-enable allow_new_track_group since we need to try another new track
                        allow_new_track_group = true;

                        // Retry just this sink
                        itarget--;  // Will be incremented back to relaxation_start_sink
                        continue;
                    } else {
                        // A sink AFTER the first sink of new group failed
                        // This means the track established for the current group can't reach this sink
                        // Create another track group for the remaining sinks (if under limit)
                        if (track_groups_created >= MAX_TRACK_GROUPS) {
                            VTR_LOG("  Sink %u failed, but max track groups (%u) reached, giving up\n",
                                    itarget, MAX_TRACK_GROUPS);
                            // Fall through to failure handling
                        } else {
                            VTR_LOG("  Sink %u failed within current track group (started at sink %u)\n",
                                    itarget, relaxation_start_sink);
                            VTR_LOG("  Creating new track group for remaining sinks\n");

                            // Add the current group's track to existing_opin_tracks
                            auto current_tracks = extract_opin_tracks_from_route_tree(tree);
                            for (const auto& [opin_id, track_num] : current_tracks) {
                                existing_opin_tracks[opin_id].insert(track_num);
                            }

                            // Clear blacklist and start fresh for the new group
                            blacklisted_tracks.clear();

                            // Update relaxation_start_sink to current sink
                            relaxation_start_sink = itarget;

                            // Re-enable allow_new_track_group for the new group
                            allow_new_track_group = true;

                            // Reset retry counter for the new group
                            retry_attempt = 0;

                            // Increment track group counter
                            track_groups_created++;

                            // Retry current sink
                            itarget--;
                            continue;
                        }
                    }
                } else {
                    // === CHECK IF WE SHOULD ENABLE TRACK GROUP RELAXATION ===
                    // Before ripping up, check if we've nearly exhausted all track retries
                    // and should switch to track group relaxation instead.
                    // We trigger at MAX-2 because:
                    //   - After rip-up, a different (earlier) sink may fail
                    //   - If itarget becomes 0, relaxation can't trigger (no sinks to preserve)
                    //   - Triggering 2 retries early gives buffer for the failing sink to change
                    bool should_relax = is_multicast_net && !is_heuristic_bcast
                                        && retry_attempt >= MAX_TRACK_RETRY_ATTEMPTS - 2
                                        && track_groups_created < MAX_TRACK_GROUPS;

                    // Primary case: current failing sink is not sink 0, so we can preserve earlier sinks
                    bool can_relax_current = should_relax && itarget > 0;

                    // Fallback case: current sink is 0, but we know from previous attempts that
                    // some sinks CAN be routed. Use best_itarget_reached as the split point.
                    // This requires a fresh start with relaxation enabled from the beginning.
                    // Only attempt fallback once to prevent infinite loops.
                    bool can_relax_fallback = should_relax && itarget == 0 && best_itarget_reached > 0
                                              && retry_attempt >= MAX_TRACK_RETRY_ATTEMPTS
                                              && !fallback_attempted;

                    if (can_relax_current) {
                        VTR_LOG("\n=== TRACK GROUP RELAXATION for multicast net '%s' ===\n", net_name.c_str());
                        VTR_LOG("After %d retries, enabling track group relaxation for sink %d (pin %d)\n",
                                retry_attempt, itarget, target_pin);

                        // Record which tracks are currently used by OPINs in the partially successful routing
                        // These sinks (0 to itarget-1) are successfully routed; we want to keep them
                        auto opin_tracks = extract_opin_tracks_from_route_tree(tree);
                        for (const auto& [opin_id, track_num] : opin_tracks) {
                            existing_opin_tracks[opin_id].insert(track_num);
                            VTR_LOG("  Track group 1: OPIN %zu uses track %d (sinks 0-%u)\n",
                                    size_t(opin_id), track_num, itarget - 1);
                        }

                        // Clear blacklist - we'll try all tracks again, but with relaxation enabled
                        blacklisted_tracks.clear();

                        // Enable track group relaxation mode
                        // This tells the router to NOT use the existing tracks (forces new track)
                        allow_new_track_group = true;
                        track_group_relaxed = true;
                        relaxation_start_sink = itarget;

                        // Reset retry counter for the relaxed routing phase
                        retry_attempt = 0;

                        // Increment track group counter (initial relaxation creates group 2)
                        track_groups_created = 2;  // Group 1 (existing) + Group 2 (new)

                        // DON'T rip up the tree - keep sinks 0..itarget-1 routed on their track
                        // Just retry routing this sink with relaxed constraint
                        VTR_LOG("  Attempting to route sink %d and remaining sinks on a NEW track\n", itarget);
                        itarget--;  // Will be incremented to retry current sink
                        continue;
                    } else if (can_relax_fallback) {
                        // Fallback: sink 0 is failing, but we know from best_itarget_reached that
                        // sinks 0 to best_itarget_reached-1 CAN be routed on some track.
                        // Strategy: rip up, then route with preemptive split at best_itarget_reached
                        VTR_LOG("\n=== TRACK GROUP RELAXATION (FALLBACK) for multicast net '%s' ===\n", net_name.c_str());
                        VTR_LOG("Sink 0 failing after %d retries, but previously routed up to sink %u\n",
                                retry_attempt, best_itarget_reached - 1);
                        VTR_LOG("Will split at sink %u on next attempt\n", best_itarget_reached);

                        // Mark fallback as attempted to prevent infinite loops
                        fallback_attempted = true;

                        // Clear blacklist for fresh start
                        blacklisted_tracks.clear();

                        // Enable relaxation mode, but DON'T set allow_new_track_group yet
                        // It will be set when we reach the split point (best_itarget_reached)
                        track_group_relaxed = true;
                        relaxation_start_sink = best_itarget_reached;

                        // Set track groups (will create group 2 when split point is reached)
                        track_groups_created = 1;  // Group 1 exists, group 2 will be created at split

                        // Reset retry counter
                        retry_attempt = 0;

                        // Rip up and restart - relaxation will kick in at the split point
                        setup_net(itry, net_id, net_list, connections_inf, router_opts, worst_negative_slack);

                        if (high_fanout) {
                            spatial_route_tree_lookup = build_route_tree_spatial_lookup(net_list,
                                                                                        route_ctx.route_bb,
                                                                                        net_id,
                                                                                        tree.root());
                        }

                        itarget = static_cast<unsigned>(-1);
                        continue;
                    }

                    // Normal mode: rip up the net and retry from the beginning
                    VTR_LOG("  Retry attempt %d/%d: ripping up net and re-routing with blacklisted tracks\n",
                            retry_attempt, MAX_TRACK_RETRY_ATTEMPTS);

                    // Reset the routing for this net
                    setup_net(itry, net_id, net_list, connections_inf, router_opts, worst_negative_slack);

                    // Reset high fanout lookup if needed
                    if (high_fanout) {
                        spatial_route_tree_lookup = build_route_tree_spatial_lookup(net_list,
                                                                                    route_ctx.route_bb,
                                                                                    net_id,
                                                                                    tree.root());
                    }

                    // Restart routing from the first sink
                    itarget = static_cast<unsigned>(-1); // Will be incremented to 0 in next iteration
                    continue;
                }
            }

            // If retry failed or wasn't applicable, give up
            flags.success = false;
            VTR_LOG("Routing failed for sink %d of net %d\n", target_pin, size_t(net_id));
            return flags;
        }

        profiling::conn_finish(size_t(route_ctx.net_rr_terminals[net_id][0]),
                               size_t(sink_rr),
                               pin_criticality[target_pin]);

        // Track the highest itarget successfully routed (for relaxation decision)
        if (!track_group_relaxed && itarget + 1 > best_itarget_reached) {
            best_itarget_reached = itarget + 1;  // +1 because we've successfully routed up to and including itarget
        }

        // === TRACK GROUP RELAXATION: After first sink in new group routed ===
        // After successfully routing the first sink in relaxed mode, turn off allow_new_track_group
        // so subsequent sinks use the normal same-track constraint (within the new group)
        if (allow_new_track_group && itarget == relaxation_start_sink) {
            VTR_LOG("  Successfully routed first sink of track group 2 (sink %u)\n", itarget);

            // Record the new track used by this sink
            auto new_opin_tracks = extract_opin_tracks_from_route_tree(tree);
            for (const auto& [opin_id, track_num] : new_opin_tracks) {
                // Only record if this is a NEW track (not in existing_opin_tracks)
                auto it = existing_opin_tracks.find(opin_id);
                if (it == existing_opin_tracks.end() || it->second.find(track_num) == it->second.end()) {
                    VTR_LOG("  Track group 2: OPIN %zu will use track %d\n", size_t(opin_id), track_num);
                }
            }

            // Turn off relaxation mode - subsequent sinks should use same track as this one
            allow_new_track_group = false;
            VTR_LOG("  Disabled track group relaxation, remaining sinks will use same track\n");
        }
        // === END TRACK GROUP RELAXATION SUCCESS ===

        ++router_stats.connections_routed;
    } // finished all sinks

    // === TRACK GROUP RELAXATION SUMMARY ===
    if (track_group_relaxed && is_multicast_net) {
        VTR_LOG("\n=== Track Group Relaxation Summary for '%s' ===\n", net_name.c_str());
        auto final_tracks = extract_opin_tracks_from_route_tree(tree);

        // Count unique tracks used
        std::unordered_set<int> unique_tracks;
        for (const auto& [opin_id, track_num] : final_tracks) {
            unique_tracks.insert(track_num);
        }

        VTR_LOG("  Successfully routed %u sinks using %zu track group(s)\n",
                num_sinks, existing_opin_tracks.empty() ? 1 : existing_opin_tracks.begin()->second.size() + 1);
        VTR_LOG("  Tracks used: ");
        for (int t : unique_tracks) {
            VTR_LOG("%d ", t);
        }
        VTR_LOG("\n");
    }
    // === END TRACK GROUP RELAXATION SUMMARY ===

    ++router_stats.nets_routed;
    profiling::net_finish();

    /* For later timing analysis. */

    float* net_delay = net_delays[net_id].data();

    // may have to update timing delay of the previously legally reached sinks since downstream capacitance could be changed
    update_net_delays_from_route_tree(net_delay,
                                      net_list,
                                      net_id,
                                      timing_info,
                                      pin_timing_invalidator);

    if (router_opts.update_lower_bound_delays) {
        for (int ipin : remaining_targets) {
            connections_inf.update_lower_bound_connection_delay(net_id, ipin, net_delay[ipin]);
        }
    }

    VTR_ASSERT_MSG(g_vpr_ctx.routing().rr_node_route_inf[tree.root().inode].occ() <= rr_graph.node_capacity(tree.root().inode), "SOURCE should never be congested");
    VTR_LOGV_DEBUG(f_router_debug, "Routed Net %zu (%zu sinks)\n", size_t(net_id), num_sinks);

    router.empty_rcv_route_tree_set(); // ?

    profiling::net_fanout_end(net_list.net_sinks(net_id).size());

    route_ctx.net_status.set_is_routed(net_id, true);
    return flags;
}

/** Route to a "virtual sink" in the netlist which corresponds to the start point
 * of the global clock network. */
template<typename ConnectionRouterType>
inline NetResultFlags pre_route_to_clock_root(ConnectionRouterType& router,
                                              ParentNetId net_id,
                                              const Netlist<>& net_list,
                                              RRNodeId sink_node,
                                              const t_conn_cost_params cost_params,
                                              int high_fanout_threshold,
                                              RouteTree& tree,
                                              SpatialRouteTreeLookup& spatial_rt_lookup,
                                              RouterStats& router_stats,
                                              bool is_flat) {
    const auto& device_ctx = g_vpr_ctx.device();
    auto& route_ctx = g_vpr_ctx.mutable_routing();
    auto& m_route_ctx = g_vpr_ctx.mutable_routing();

    NetResultFlags out;
    out.was_rerouted = true;

    bool high_fanout = is_high_fanout(net_list.net_sinks(net_id).size(), high_fanout_threshold);

    VTR_LOGV_DEBUG(f_router_debug, "Net %zu pre-route to (%s)\n", size_t(net_id), describe_rr_node(device_ctx.rr_graph, device_ctx.grid, device_ctx.rr_indexed_data, sink_node, is_flat).c_str());
    profiling::sink_criticality_start();

    t_bb bounding_box = route_ctx.route_bb[net_id];

    router.clear_modified_rr_node_info();

    bool found_path, retry_with_full_bb;
    RTExploredNode cheapest;
    ConnectionParameters conn_params(net_id,
                                     -1,
                                     false,
                                     std::unordered_map<RRNodeId, int>());

    std::tie(found_path, retry_with_full_bb, cheapest) = router.timing_driven_route_connection_from_route_tree(
        tree.root(),
        sink_node,
        cost_params,
        bounding_box,
        router_stats,
        conn_params);

    // TODO: Parts of the rest of this function are repetitive to code in route_sink. Should refactor.
    if (!found_path) {
        ParentBlockId src_block = net_list.net_driver_block(net_id);
        VTR_LOG("Failed to route connection from '%s' to '%s' for net '%s' (#%zu)\n",
                net_list.block_name(src_block).c_str(),
                describe_rr_node(device_ctx.rr_graph, device_ctx.grid, device_ctx.rr_indexed_data, sink_node, is_flat).c_str(),
                net_list.net_name(net_id).c_str(),
                size_t(net_id));
        if (f_router_debug) {
            update_screen(ScreenUpdatePriority::MAJOR, "Unable to route connection.", ROUTING, nullptr);
        }
        router.reset_path_costs();
        out.success = false;
        out.retry_with_full_bb = retry_with_full_bb;
        return out;
    }

    profiling::sink_criticality_end(cost_params.criticality);

    /* This is a special pre-route to a sink that does not correspond to any    *
     * netlist pin, but which can be reached from the global clock root drive   *
     * points. Therefore, we can set the net pin index of the sink node to      *
     * UNDEFINED (meaning illegal) as it is not meaningful for this sink.            */
    vtr::optional<const RouteTreeNode&> new_branch, new_sink;
    std::tie(new_branch, new_sink) = tree.update_from_heap(&cheapest, UNDEFINED, ((high_fanout) ? &spatial_rt_lookup : nullptr), is_flat);

    VTR_ASSERT_DEBUG(!high_fanout || validate_route_tree_spatial_lookup(tree.root(), spatial_rt_lookup));

    if (f_router_debug) {
        std::string msg = vtr::string_fmt("Routed Net %zu connection to RR node %d successfully", size_t(net_id), sink_node);
        update_screen(ScreenUpdatePriority::MAJOR, msg.c_str(), ROUTING, nullptr);
    }

    if (new_branch)
        pathfinder_update_cost_from_route_tree(new_branch.value(), 1);

    // need to guarantee ALL nodes' path costs are HUGE_POSITIVE_FLOAT at the start of routing to a sink
    // do this by resetting all the path_costs that have been touched while routing to the current sink
    router.reset_path_costs();

    // Post route cleanup:
    // - remove sink from route tree and fix routing for all nodes leading to the sink ("freeze")
    // - free up virtual sink occupancy
    tree.freeze();
    m_route_ctx.rr_node_route_inf[sink_node].set_occ(0);

    // routed to a sink successfully
    out.success = true;
    return out;
}

/** Attempt to route a single sink (target_pin) in a net.
 * In the process, update global pathfinder costs, rr_node_route_inf and extend the global RouteTree
 * for this net.
 *
 * @param router The ConnectionRouterType instance
 * @param net_list Input netlist
 * @param net_id
 * @param itarget # of this connection in the net (only used for debug output)
 * @param target_pin # of this sink in the net (TODO: is it the same thing as itarget?)
 * @param cost_params
 * @param router_opts
 * @param[in, out] tree RouteTree describing the current routing state
 * @param rt_node_of_sink Lookup from target_pin-like indices (indicating SINK nodes) to RouteTreeNodes
 * @param spatial_rt_lookup
 * @param router_stats
 * @param budgeting_inf
 * @param routing_predictor
 * @param choking_spots
 * @param is_flat
 * @param net_bb Bounding box for the net (Routing resources outside net_bb will not be used)
 * @param blacklisted_tracks Map from OPIN to blacklisted track numbers (for multi-fanout retry)
 * @return NetResultFlags for this sink to be bubbled up through route_net */
template<typename ConnectionRouterType>
inline NetResultFlags route_sink(ConnectionRouterType& router,
                                 const Netlist<>& net_list,
                                 ParentNetId net_id,
                                 unsigned itarget,
                                 int target_pin,
                                 const t_conn_cost_params cost_params,
                                 const t_router_opts& router_opts,
                                 RouteTree& tree,
                                 SpatialRouteTreeLookup& spatial_rt_lookup,
                                 RouterStats& router_stats,
                                 route_budgets& budgeting_inf,
                                 const RoutingPredictor& routing_predictor,
                                 const std::vector<std::unordered_map<RRNodeId, int>>& choking_spots,
                                 bool is_flat,
                                 const t_bb& net_bb,
                                 const std::unordered_map<RRNodeId, std::unordered_set<int>>& blacklisted_tracks = {},
                                 const std::unordered_map<RRNodeId, std::unordered_set<int>>& existing_opin_tracks = {},
                                 bool allow_new_track_group = false) {
    const auto& device_ctx = g_vpr_ctx.device();
    auto& route_ctx = g_vpr_ctx.mutable_routing();

    NetResultFlags flags;

    profiling::sink_criticality_start();

    RRNodeId sink_node = route_ctx.net_rr_terminals[net_id][target_pin];
    VTR_LOGV_DEBUG(f_router_debug, "Net %zu Target %d (%s)\n", size_t(net_id), itarget, describe_rr_node(device_ctx.rr_graph, device_ctx.grid, device_ctx.rr_indexed_data, sink_node, is_flat).c_str());

    router.clear_modified_rr_node_info();

    bool found_path;
    RTExploredNode cheapest;

    bool net_is_global = net_list.net_is_global(net_id);
    bool high_fanout = is_high_fanout(net_list.net_sinks(net_id).size(), router_opts.high_fanout_threshold);
    constexpr float HIGH_FANOUT_CRITICALITY_THRESHOLD = 0.9;
    bool sink_critical = (cost_params.criticality > HIGH_FANOUT_CRITICALITY_THRESHOLD);
    bool net_is_clock = route_ctx.is_clock_net[net_id] != 0;

    bool router_opt_choke_points = ((int)choking_spots[target_pin].size() != 0) && router_opts.has_choke_point;

    // === HEURISTIC FORCED TRACK ===
    // When VPR_HEURISTIC_ROUTING=1:
    //   broadcast nets  → forced to track W-1   (one dedicated column track)
    //   reduce nets W-2 → forced to track W-2   (even hops in the reduce chain)
    //   reduce nets W-3 → forced to track W-3   (odd  hops in the reduce chain)
    int heuristic_forced_track = -1;
    {
        std::string route_net_name = net_list.net_name(net_id);
        int effective_chan_width = router_opts.fixed_channel_width;
        if (effective_chan_width <= 0) {
            effective_chan_width = device_ctx.chan_width.max;
        }
        if (is_heuristic_bcast_net_name(route_net_name)) {
            if (route_net_name.find("bcast") != std::string::npos) {
                // Prefer the explicit fixed channel width from router opts; if unavailable,
                // fall back to the device channel width so heuristic routing remains active.
                if (effective_chan_width > 0) {
                    heuristic_forced_track = effective_chan_width - 1;
                }
            }
        } else {
            int reduce_track_offset = -1;
            if (is_heuristic_reduce_net_name(route_net_name, reduce_track_offset)) {
                if (effective_chan_width > 0 && reduce_track_offset > 0) {
                    heuristic_forced_track = effective_chan_width - reduce_track_offset;
                }
            }
        }
    }
    // === END HEURISTIC FORCED TRACK ===

    ConnectionParameters conn_params(net_id, target_pin, router_opt_choke_points, choking_spots[target_pin],
                                     blacklisted_tracks, existing_opin_tracks, allow_new_track_group,
                                     heuristic_forced_track);

    //We normally route high fanout nets by only adding spatially close-by routing to the heap (reduces run-time).
    //However, if the current sink is 'critical' from a timing perspective, we put the entire route tree back onto
    //the heap to ensure it has more flexibility to find the best path.
    if (high_fanout && !sink_critical && !net_is_global && !net_is_clock && -routing_predictor.get_slope() > router_opts.high_fanout_max_slope) {
        std::tie(found_path, flags.retry_with_full_bb, cheapest) = router.timing_driven_route_connection_from_route_tree_high_fanout(tree.root(),
                                                                                                                                     sink_node,
                                                                                                                                     cost_params,
                                                                                                                                     net_bb,
                                                                                                                                     spatial_rt_lookup,
                                                                                                                                     router_stats,
                                                                                                                                     conn_params);
    } else {
        std::tie(found_path, flags.retry_with_full_bb, cheapest) = router.timing_driven_route_connection_from_route_tree(tree.root(),
                                                                                                                         sink_node,
                                                                                                                         cost_params,
                                                                                                                         net_bb,
                                                                                                                         router_stats,
                                                                                                                         conn_params);
    }

    if (!found_path) {
        ParentBlockId src_block = net_list.net_driver_block(net_id);
        ParentBlockId sink_block = net_list.pin_block(*(net_list.net_pins(net_id).begin() + target_pin));
        VTR_LOG("Failed to route connection from '%s' to '%s' for net '%s' (#%zu)\n",
                net_list.block_name(src_block).c_str(),
                net_list.block_name(sink_block).c_str(),
                net_list.net_name(net_id).c_str(),
                size_t(net_id));
        if (f_router_debug) {
            update_screen(ScreenUpdatePriority::MAJOR, "Unable to route connection.", ROUTING, nullptr);
        }
        flags.success = false;
        router.reset_path_costs();
        return flags;
    }

    profiling::sink_criticality_end(cost_params.criticality);

    vtr::optional<const RouteTreeNode&> new_branch, new_sink;
    std::tie(new_branch, new_sink) = tree.update_from_heap(&cheapest, target_pin, ((high_fanout) ? &spatial_rt_lookup : nullptr), is_flat);

    VTR_ASSERT_DEBUG(!high_fanout || validate_route_tree_spatial_lookup(tree.root(), spatial_rt_lookup));

    if (f_router_debug) {
        std::string msg = vtr::string_fmt("Routed Net %zu connection %d to RR node %d successfully", size_t(net_id), itarget, sink_node);
        update_screen(ScreenUpdatePriority::MAJOR, msg.c_str(), ROUTING, nullptr);
    }

    if (budgeting_inf.if_set() && cheapest.rcv_path_backward_delay != std::numeric_limits<float>::infinity() && cost_params.delay_budget) {
        if (cheapest.rcv_path_backward_delay < cost_params.delay_budget->min_delay) {
            budgeting_inf.set_should_reroute(net_id, true);
        }
    }

    /* update global occupancy from the new branch */
    if (new_branch)
        pathfinder_update_cost_from_route_tree(new_branch.value(), 1);

    // need to guarantee ALL nodes' path costs are HUGE_POSITIVE_FLOAT at the start of routing to a sink
    // do this by resetting all the path_costs that have been touched while routing to the current sink
    router.reset_path_costs();

    // routed to a sink successfully
    flags.success = true;
    return flags;
}
