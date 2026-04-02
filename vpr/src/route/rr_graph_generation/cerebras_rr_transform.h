#pragma once

/**
 * @file cerebras_rr_transform.h
 * @brief In-memory Cerebras-style RR graph transform.
 *
 * Ports the logic from the external rr_graph_transform tool
 * (vtr/utils/rr_graph_transform/src/main.cpp) into VPR's internal
 * data structures, eliminating the XML round-trip.
 *
 * Rules applied (same as external tool):
 *   R2: PE OPIN→CHAN* replaced with OPIN→gate(CHANY)
 *   R3: CHAN*→PE IPIN replaced with gate(CHANY)→IPIN
 *   R4: Remove forbidden downward base-CHANY transitions
 *   R5: Add base→gate handoff edges
 *   R6: Reattach selected R4 edges from gate
 *   R6.5: Gate↔layer0 coupling at (x,y,ptc) and neighbors
 *   R7: Remove forbidden base-CHANX edges, add CHANX→gate handoffs
 *   R8: Global bypass cleanup
 *   Boundary cleanup: remove CHANX(y=0) and CHANY(x=0) on layer 0
 *   Track reservation: remove reserved-track edges in fixed region
 *
 * Must be called AFTER edges are built but BEFORE
 * alloc_and_load_rr_switch_inf() and partition_edges().
 */

#include "rr_graph_builder.h"
#include "rr_graph_view.h"
#include "device_grid.h"

/**
 * Apply Cerebras RR graph transform in-memory.
 *
 * @param builder        Mutable graph builder (to clear+rebuild edges, add nodes)
 * @param rr_graph       Read-only graph view (for node properties)
 * @param grid           Device grid (to identify PE tiles)
 * @param gate_layer     Gate layer index (default 1)
 * @param fixed_region_size  Size of fixed region per side (0 = no track reservation)
 * @param fabric_h       Fabric height in PE rows (for reservation boundary calc)
 * @param reserved_track_min  Minimum reserved track number (inclusive, default 1)
 * @param reserved_track_max  Maximum reserved track number (inclusive, default 10)
 */
void cerebras_transform_rr_graph(
    RRGraphBuilder& builder,
    const RRGraphView& rr_graph,
    const DeviceGrid& grid,
    int gate_layer = 1,
    int fixed_region_size = 0,
    int fabric_h = 0,
    int reserved_track_min = 1,
    int reserved_track_max = 10);
