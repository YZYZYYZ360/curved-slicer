#pragma once

#include <Eigen/Dense>

namespace cslc {

class MeshRepair {
public:
    struct Report {
        enum class UnitGuess { MILLIMETERS, METERS, AMBIGUOUS };

        int n_vertices_in = 0;
        int n_triangles_in = 0;
        double bbox_diagonal_mm = 0.0;

        bool is_manifold = false;
        int n_non_manifold_edges = 0;
        bool is_orientable = false;
        int n_boundary_edges = 0;
        int n_degenerate_tris = 0;
        int n_duplicate_vertices = 0;

        UnitGuess unit_guess = UnitGuess::AMBIGUOUS;

        int n_vertices_merged = 0;
        int n_triangles_removed_degenerate = 0;
        int n_triangles_flipped = 0;
        int n_holes_filled = 0;

        int n_vertices_out = 0;
        int n_triangles_out = 0;
        bool is_manifold_after = false;
        bool is_orientable_after = false;
        int n_boundary_edges_after = 0;
    };

    struct Options {
        double duplicate_vertex_eps_mm = 1e-4;
        double degenerate_area_eps_mm2 = 1e-6;
        bool fill_holes = true;
        bool reorient_outward = true;
        double meters_threshold_bbox_diagonal = 1.0;
    };

    static Report repair(
        const Eigen::MatrixXd& V_in,
        const Eigen::MatrixXi& F_in,
        const Options& opt,
        Eigen::MatrixXd& V_out,
        Eigen::MatrixXi& F_out);
};

}  // namespace cslc
