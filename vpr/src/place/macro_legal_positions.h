#pragma once

#include "vpr_types.h"
#include "vtr_ndmatrix.h"
#include "vtr_random.h"

#include <vector>

class PlaceMacros;
class BlkLocRegistry;
struct t_pl_blocks_to_be_moved;

/**
 * @brief Maintains precomputed sets of valid head positions for each placement macro.
 *
 * For each macro m, stores all grid positions (hx, hy) where the macro's entire
 * footprint is free of non-m blocks. This allows UniformMoveGenerator to sample
 * a legal macro destination directly, eliminating abort-heavy random sampling.
 *
 * After every accepted move, update() incrementally adjusts the valid-head sets
 * based on which grid cells were vacated and occupied.
 */
class MacroLegalPositions {
  public:
    /**
     * @brief Scans all grid positions for each macro and builds initial obstruction
     * counts and valid-head lists. Must be called once after initial placement.
     */
    void init(const PlaceMacros& place_macros,
              const BlkLocRegistry& blk_loc_registry,
              int grid_w, int grid_h);

    /**
     * @brief Incrementally updates obstruction counts and valid-head lists based
     * on a committed move. Must be called after commit_move_blocks().
     * @param blocks_affected The move transaction; moved_blocks holds old_loc and new_loc.
     * @param place_macros Used to check macro membership of moved blocks.
     */
    void update(const t_pl_blocks_to_be_moved& blocks_affected,
                const PlaceMacros& place_macros);

    /**
     * @brief Samples a random valid head position for macro imacro within rlim
     * of current_head (Chebyshev distance).
     * @return A valid head t_pl_loc, or {-1,-1,0,0} if no candidate found.
     */
    t_pl_loc sample_valid_head(int imacro,
                               const t_pl_loc& current_head,
                               float rlim,
                               vtr::RngContainer& rng) const;

    /// @brief Returns true if the given head position is valid (obstruction_count == 0).
    bool is_valid(int imacro, const t_pl_loc& head) const;

    /// @brief Returns the head block (members[0]) of macro imacro.
    ClusterBlockId head_block(int imacro) const;

    /// @brief Returns true if any macros have been registered.
    bool has_macros() const { return !valid_heads_.empty(); }

    /// @brief Returns the number of registered macros.
    int num_macros() const { return static_cast<int>(valid_heads_.size()); }

  private:
    void add_valid_head_(int imacro, int hx, int hy);
    void remove_valid_head_(int imacro, int hx, int hy);
    bool head_in_bounds_(int imacro, int hx, int hy) const;

    // Per macro: [hx][hy] = # non-m blocks occupying m's footprint with head at (hx,hy)
    std::vector<vtr::NdMatrix<int, 2>> obstruction_count_;

    // Per macro: flat list of (hx,hy) where obstruction_count == 0
    std::vector<std::vector<t_pl_loc>> valid_heads_;

    // Per macro: [hx][hy] = index into valid_heads_[m], or -1 if not valid
    // Enables O(1) add/remove via swap-and-pop.
    std::vector<vtr::NdMatrix<int, 2>> valid_idx_;

    // Per macro: head block and member offsets (cached from PlaceMacros)
    std::vector<ClusterBlockId> head_blk_;
    std::vector<std::vector<t_pl_offset>> offsets_;
    std::vector<int> macro_w_; // max x-offset + 1
    std::vector<int> macro_h_; // max y-offset + 1
};
