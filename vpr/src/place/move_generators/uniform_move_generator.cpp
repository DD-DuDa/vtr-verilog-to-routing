#include "uniform_move_generator.h"

#include "globals.h"
#include "physical_types_util.h"
#include "place_constraints.h"
#include "place_macro.h"
#include "placer_state.h"
#include "move_utils.h"

UniformMoveGenerator::UniformMoveGenerator(PlacerState& placer_state,
                                           const PlaceMacros& place_macros,
                                           const NetCostHandler& net_cost_handler,
                                           e_reward_function reward_function,
                                           vtr::RngContainer& rng)
    : MoveGenerator(placer_state, place_macros, net_cost_handler, reward_function, rng) {}

e_create_move UniformMoveGenerator::propose_move(t_pl_blocks_to_be_moved& blocks_affected,
                                                 t_propose_action& proposed_action,
                                                 float rlim,
                                                 const t_placer_opts& placer_opts,
                                                 const PlacerCriticalities* /*criticalities*/) {
    const auto& cluster_ctx = g_vpr_ctx.clustering();
    const auto& placer_state = placer_state_.get();
    const auto& block_locs = placer_state.block_locs();
    const auto& blk_loc_registry = placer_state.blk_loc_registry();

    //Find a movable block based on blk_type
    ClusterBlockId b_from = propose_block_to_move(placer_opts,
                                                  proposed_action.logical_blk_type_index,
                                                  /*highly_crit_block=*/false,
                                                  /*placer_criticalities=*/nullptr,
                                                  /*net_from=*/nullptr,
                                                  /*pin_from=*/nullptr,
                                                  placer_state,
                                                  rng_);

    VTR_LOGV_DEBUG(g_vpr_ctx.placement().f_placer_debug, "Uniform Move Choose Block %d - rlim %f\n", size_t(b_from), rlim);

    if (!b_from) { //No movable block found
        VTR_LOGV_DEBUG(g_vpr_ctx.placement().f_placer_debug, "\tNo movable block found\n");
        return e_create_move::ABORT;
    }

    t_pl_loc from = block_locs[b_from].loc;
    auto cluster_from_type = cluster_ctx.clb_nlist.block_type(b_from);
    auto grid_from_type = g_vpr_ctx.device().grid.get_physical_type({from.x, from.y, from.layer});
    VTR_ASSERT(is_tile_compatible(grid_from_type, cluster_from_type));

    t_pl_loc to;

    // --- MACRO VALID-HEAD FAST PATH ---
    // If b_from is part of a macro and we have precomputed valid head positions,
    // sample directly from that set instead of using find_to_loc_uniform.
    // This eliminates aborts from illegal macro placements.
    if (macro_legal_pos_ && macro_legal_pos_->has_macros()) {
        int imacro = place_macros_.get_imacro_from_iblk(b_from);
        if (imacro >= 0) {
            // Find b_from's offset within the macro
            t_pl_offset b_from_offset{0, 0, 0, 0};
            for (const auto& member : place_macros_[imacro].members) {
                if (member.blk_index == b_from) {
                    b_from_offset = member.offset;
                    break;
                }
            }

            // Get the current head location
            ClusterBlockId head_blk = macro_legal_pos_->head_block(imacro);
            t_pl_loc head_loc = block_locs[head_blk].loc;

            // Sample a valid head position within rlim
            t_pl_loc new_head = macro_legal_pos_->sample_valid_head(imacro, head_loc, rlim, rng_);

            if (new_head.x >= 0) {
                // Place b_from at its offset relative to the new head
                to = t_pl_loc{new_head.x + b_from_offset.x,
                              new_head.y + b_from_offset.y,
                              from.sub_tile,
                              new_head.layer};

                e_create_move create_move_result = ::create_move(blocks_affected, b_from, to,
                                                                 blk_loc_registry, place_macros_);
                if (!floorplan_legal(blocks_affected)) return e_create_move::ABORT;
                return create_move_result;
            }
            // No valid head within rlim.
            // Falling through to find_to_loc_uniform + create_move would almost
            // always abort anyway (dense grids leave no room for a complete macro
            // footprint), so abort immediately to avoid the expensive scan.
            return e_create_move::ABORT;
        }
    }
    // --- END MACRO VALID-HEAD FAST PATH ---

    if (!find_to_loc_uniform(cluster_from_type, rlim, from, to, b_from, blk_loc_registry, rng_)) {
        return e_create_move::ABORT;
    }

#if 0
    auto& grid = g_vpr_ctx.device().grid;
    const auto& grid_to_type = grid.get_physical_type(to.x, to.y, to.layer);
	VTR_LOG( "swap [%d][%d][%d][%d] %s block %zu \"%s\" <=> [%d][%d][%d][%d] %s block ",
		from.x, from.y, from.sub_tile,from.layer, grid_from_type->name, size_t(b_from), (b_from ? cluster_ctx.clb_nlist.block_name(b_from).c_str() : ""),
		to.x, to.y, to.sub_tile, to.layer, grid_to_type->name);
    if (b_to) {
        VTR_LOG("%zu \"%s\"", size_t(b_to), cluster_ctx.clb_nlist.block_name(b_to).c_str());
    } else {
        VTR_LOG("(EMPTY)");
    }
    VTR_LOG("\n");
#endif

    e_create_move create_move = ::create_move(blocks_affected, b_from, to, blk_loc_registry, place_macros_);

    //Check that all the blocks affected by the move would still be in a legal floorplan region after the swap
    if (!floorplan_legal(blocks_affected)) {
        return e_create_move::ABORT;
    }

    return create_move;
}
