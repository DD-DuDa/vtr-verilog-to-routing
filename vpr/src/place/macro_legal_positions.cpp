#include "macro_legal_positions.h"

#include "blk_loc_registry.h"
#include "move_transactions.h"
#include "place_macro.h"

#include <algorithm>
#include <cmath>
#include <limits>

void MacroLegalPositions::init(const PlaceMacros& place_macros,
                               const BlkLocRegistry& blk_loc_registry,
                               int grid_w, int grid_h) {
    const auto& macros = place_macros.macros();
    int num_macros = static_cast<int>(macros.size());

    if (num_macros == 0) return;

    obstruction_count_.resize(num_macros);
    valid_heads_.resize(num_macros);
    valid_idx_.resize(num_macros);
    head_blk_.resize(num_macros);
    offsets_.resize(num_macros);
    macro_w_.resize(num_macros, 0);
    macro_h_.resize(num_macros, 0);

    const auto& grid_blocks = blk_loc_registry.grid_blocks();

    for (int m = 0; m < num_macros; m++) {
        const auto& macro = macros[m];

        // Head block is members[0] (offset 0,0,0,0)
        head_blk_[m] = macro.members[0].blk_index;

        // Collect member offsets and compute macro bounding box
        int max_ox = 0, max_oy = 0;
        offsets_[m].reserve(macro.members.size());
        for (const auto& member : macro.members) {
            offsets_[m].push_back(member.offset);
            max_ox = std::max(max_ox, member.offset.x);
            max_oy = std::max(max_oy, member.offset.y);
        }
        macro_w_[m] = max_ox + 1;
        macro_h_[m] = max_oy + 1;

        // Valid head positions: hx in [0, grid_w - max_ox], hy in [0, grid_h - max_oy]
        int valid_w = grid_w - max_ox;  // number of valid x positions = grid_w - max_ox
        int valid_h = grid_h - max_oy;

        if (valid_w <= 0 || valid_h <= 0) {
            // Macro is too large to fit in the grid; leave matrices default-empty
            // head_in_bounds_ checks empty() and returns false, so no heads added
            continue;
        }

        obstruction_count_[m].resize({(size_t)valid_w, (size_t)valid_h}, 0);
        valid_idx_[m].resize({(size_t)valid_w, (size_t)valid_h}, -1);

        // Populate obstruction counts by scanning every valid head position
        for (int hx = 0; hx < valid_w; hx++) {
            for (int hy = 0; hy < valid_h; hy++) {
                int count = 0;
                for (const auto& offset : offsets_[m]) {
                    t_pl_loc loc{hx + offset.x, hy + offset.y, 0, 0};
                    ClusterBlockId blk_at = grid_blocks.block_at_location(loc);
                    if (blk_at != ClusterBlockId::INVALID()) {
                        // Occupied — only count non-m members as obstructions
                        int blk_macro = place_macros.get_imacro_from_iblk(blk_at);
                        if (blk_macro != m) {
                            count++;
                        }
                    }
                }
                obstruction_count_[m][hx][hy] = count;
                if (count == 0) {
                    add_valid_head_(m, hx, hy);
                }
            }
        }
    }
}

void MacroLegalPositions::update(const t_pl_blocks_to_be_moved& blocks_affected,
                                 const PlaceMacros& place_macros) {
    int num_macros = static_cast<int>(obstruction_count_.size());
    if (num_macros == 0) return;

    for (const auto& moved_block : blocks_affected.moved_blocks) {
        ClusterBlockId blk = moved_block.block_num;
        int blk_macro = place_macros.get_imacro_from_iblk(blk);

        const t_pl_loc& old_loc = moved_block.old_loc;
        const t_pl_loc& new_loc = moved_block.new_loc;

        for (int m = 0; m < num_macros; m++) {
            // Skip: a macro's own members do not obstruct itself
            if (blk_macro == m) continue;

            for (const auto& offset : offsets_[m]) {
                int ox = offset.x;
                int oy = offset.y;

                // old_loc is being vacated → obstruction count decreases
                {
                    int hx = old_loc.x - ox;
                    int hy = old_loc.y - oy;
                    if (head_in_bounds_(m, hx, hy)) {
                        if (--obstruction_count_[m][hx][hy] == 0) {
                            add_valid_head_(m, hx, hy);
                        }
                    }
                }

                // new_loc is being occupied → obstruction count increases
                {
                    int hx = new_loc.x - ox;
                    int hy = new_loc.y - oy;
                    if (head_in_bounds_(m, hx, hy)) {
                        if (obstruction_count_[m][hx][hy]++ == 0) {
                            remove_valid_head_(m, hx, hy);
                        }
                    }
                }
            }
        }
    }
}

t_pl_loc MacroLegalPositions::sample_valid_head(int imacro,
                                                const t_pl_loc& current_head,
                                                float rlim,
                                                vtr::RngContainer& rng) const {
    const auto& candidates = valid_heads_[imacro];
    if (candidates.empty()) {
        return t_pl_loc{-1, -1, 0, 0};
    }

    bool unlimited = std::isinf(rlim);
    int irlim = unlimited ? std::numeric_limits<int>::max()
                          : static_cast<int>(std::ceil(rlim));

    // Collect indices of valid heads within rlim, excluding current position
    std::vector<int> within_rlim;
    within_rlim.reserve(candidates.size());

    for (int i = 0; i < static_cast<int>(candidates.size()); i++) {
        const auto& head = candidates[i];
        // Exclude the current head position (moving to same spot is useless)
        if (head.x == current_head.x && head.y == current_head.y) continue;
        int dx = std::abs(head.x - current_head.x);
        int dy = std::abs(head.y - current_head.y);
        if (dx <= irlim && dy <= irlim) {
            within_rlim.push_back(i);
        }
    }

    if (within_rlim.empty()) {
        return t_pl_loc{-1, -1, 0, 0};
    }

    int chosen = within_rlim[rng.irand(static_cast<int>(within_rlim.size()) - 1)];
    return candidates[chosen];
}

bool MacroLegalPositions::is_valid(int imacro, const t_pl_loc& head) const {
    if (!head_in_bounds_(imacro, head.x, head.y)) return false;
    return obstruction_count_[imacro][head.x][head.y] == 0;
}

ClusterBlockId MacroLegalPositions::head_block(int imacro) const {
    return head_blk_[imacro];
}

void MacroLegalPositions::add_valid_head_(int imacro, int hx, int hy) {
    int idx = static_cast<int>(valid_heads_[imacro].size());
    valid_heads_[imacro].push_back(t_pl_loc{hx, hy, 0, 0});
    valid_idx_[imacro][hx][hy] = idx;
}

void MacroLegalPositions::remove_valid_head_(int imacro, int hx, int hy) {
    int idx = valid_idx_[imacro][hx][hy];
    if (idx < 0) return; // Not in valid_heads; should not happen

    auto& heads = valid_heads_[imacro];
    auto& idx_mat = valid_idx_[imacro];

    int last_idx = static_cast<int>(heads.size()) - 1;
    if (idx != last_idx) {
        // Swap with the last element and update its index
        t_pl_loc last_head = heads[last_idx];
        heads[idx] = last_head;
        idx_mat[last_head.x][last_head.y] = idx;
    }
    heads.pop_back();
    idx_mat[hx][hy] = -1;
}

bool MacroLegalPositions::head_in_bounds_(int imacro, int hx, int hy) const {
    if (obstruction_count_[imacro].empty()) return false;
    int valid_w = static_cast<int>(obstruction_count_[imacro].dim_size(0));
    int valid_h = static_cast<int>(obstruction_count_[imacro].dim_size(1));
    return hx >= 0 && hx < valid_w && hy >= 0 && hy < valid_h;
}
