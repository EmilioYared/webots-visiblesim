// catom3d_forces.cpp -- internal-force analysis of a Catoms3D structure (domain-transfer demo).
//
// Bridges the Webots controller's FCC lattice geometry (catom3d_core.hpp: gridToWorld,
// fccNeighbor, analyzeStructure) to the VALIDATED ReactiveBuild truss-FEM (catom3d_fem.hpp).
// Given a set of occupied FCC cells it reports which BONDS carry the load and how close each is
// to its tensile/shear limit -- the statics the controller's ODE run cannot give reliably (the
// "popcorn" problem). This does NOT move anything; it analyses a snapshot.
//
// Usage:
//   catom3d_forces cantilever [out_dir]        # built-in stress scenario (base + arm)
//   catom3d_forces <cells.csv> [out_dir]       # any structure: CSV lines "gx,gy,gz"
//
// Writes <out_dir>/catom3d_bond_forces.csv and catom3d_module_load.csv (for the viz command in
// the README) and prints a summary incl. an equilibrium check and the worst-loaded bond.
#include "../../../controllers/catom3d_controller/catom3d_core.hpp"
#include "../../../controllers/catom3d_controller/scene_config.hpp"
#include "../../../controllers/catom3d_controller/scenario.hpp"
#include "reactivebuild/catom3d_fem.hpp"

#include <cstdio>
#include <fstream>
#include <map>
#include <string>
#include <tuple>
#include <vector>

static std::vector<GridPos> loadCellsCsv(const char* path) {
    std::vector<GridPos> cells;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#' || line[0] == 'g' || line[0] == 'x') continue;  // header/comment
        int x, y, z;
        char c1, c2;
        std::istringstream ss(line);
        if (ss >> x >> c1 >> y >> c2 >> z) cells.push_back({x, y, z});
    }
    return cells;
}

int main(int argc, char** argv) {
    std::string which = argc > 1 ? argv[1] : "cantilever";
    std::string out_dir = argc > 2 ? argv[2] : "reactivebuild/results";

    std::vector<GridPos> cells;
    if (which == "cantilever") {
        SceneConfig cfg;
        cfg.scenarioType = "cantilever";
        cells = buildCantilever(cfg);
        std::printf("scenario: built-in cantilever (base %dx%dx%d + arm %d)\n",
                    cfg.cantBaseW, cfg.cantBaseD, cfg.cantBaseH, cfg.cantArmLen);
    } else {
        cells = loadCellsCsv(which.c_str());
        std::printf("scenario: %d cells from %s\n", (int)cells.size(), which.c_str());
    }
    if (cells.empty()) { std::fprintf(stderr, "no cells\n"); return 1; }

    // FCC cells -> module centres (world) + bond list (adjacent modules, deduped i<j).
    std::map<std::tuple<int, int, int>, int> idx;
    for (int i = 0; i < (int)cells.size(); ++i)
        idx[{cells[i].x, cells[i].y, cells[i].z}] = i;
    auto findCell = [&](GridPos g) -> int {
        auto it = idx.find({g.x, g.y, g.z});
        return it == idx.end() ? -1 : it->second;
    };

    std::vector<rb::geom::Vec3> centers(cells.size());
    int minz = cells[0].z;
    for (auto& c : cells) minz = std::min(minz, c.z);
    std::vector<char> fixed(cells.size(), 0);
    for (int i = 0; i < (int)cells.size(); ++i) {
        double w[3];
        gridToWorld(cells[i], w);
        centers[i] = rb::geom::Vec3(w[0], w[1], w[2]);
        fixed[i] = (cells[i].z == minz) ? 1 : 0;  // bottom layer rests on the floor
    }
    std::vector<std::array<int, 2>> bonds;
    for (int i = 0; i < (int)cells.size(); ++i)
        for (int con = 0; con < 12; ++con) {
            int j = findCell(fccNeighbor(cells[i], con));
            if (j > i) bonds.push_back({i, j});
        }

    // Force analysis (validated truss-FEM) + graph integrity (existing pure-geometry check).
    rb::Catom3DFEMParams p;
    rb::Catom3DForces F = rb::catom3d_forces(centers, bonds, fixed, p);
    StructuralReport S = analyzeStructure(cells);

    std::printf("modules=%d  bonds=%d  fixed(base)=%d\n", (int)cells.size(), (int)bonds.size(),
                (int)std::count(fixed.begin(), fixed.end(), (char)1));
    std::printf("graph: components=%d  articulation(SPOF)=%d  overhang=%d  COM-in-support=%s\n",
                S.components, S.articulation, S.overhanging, S.comInSupport ? "yes" : "no");
    if (!F.solved) {
        std::printf("FORCE ANALYSIS: %s\n", F.note.c_str());
        return 0;
    }
    std::printf("total weight = %.3f N   support reaction = (%.3f, %.3f, %.3f) N  [z should ~= +%.3f]\n",
                F.total_weight, F.reaction_sum.x(), F.reaction_sum.y(), F.reaction_sum.z(),
                F.total_weight);
    std::printf("worst bond in TENSION: util=%.2f  (%.0f%% of the %.1f N limit)",
                F.max_util_tension, 100.0 * F.max_util_tension, p.tensile_strength);
    if (F.worst_tension_bond >= 0) {
        auto& b = F.bonds[F.worst_tension_bond];
        std::printf("  between modules %d and %d  (axial %.3f N)", b.a, b.b, b.axial);
    }
    std::printf("\nworst bond in SHEAR:   util=%.2f  (%.0f%% of the %.1f N limit)\n",
                F.max_util_shear, 100.0 * F.max_util_shear, p.shear_strength);
    int broken = 0;
    for (auto& b : F.bonds) if (b.util_tension >= 1.0 || b.util_shear >= 1.0) ++broken;
    std::printf("bonds predicted to BREAK under gravity (util >= 1): %d\n", broken);

    // CSVs for the visualiser.
    std::ofstream bf(out_dir + "/catom3d_bond_forces.csv");
    bf << "a,b,xa,ya,za,xb,yb,zb,axial,shear,bending,util_tension,util_shear\n";
    for (auto& b : F.bonds) {
        auto& ca = centers[b.a];
        auto& cb = centers[b.b];
        bf << b.a << ',' << b.b << ',' << ca.x() << ',' << ca.y() << ',' << ca.z() << ','
           << cb.x() << ',' << cb.y() << ',' << cb.z() << ',' << b.axial << ',' << b.shear << ','
           << b.bending << ',' << b.util_tension << ',' << b.util_shear << '\n';
    }
    std::ofstream mf(out_dir + "/catom3d_module_load.csv");
    mf << "module,gx,gy,gz,x,y,z,load,overhang,articulation,fixed\n";
    for (int i = 0; i < (int)cells.size(); ++i)
        mf << i << ',' << cells[i].x << ',' << cells[i].y << ',' << cells[i].z << ',' << centers[i].x()
           << ',' << centers[i].y() << ',' << centers[i].z() << ',' << F.module_load[i] << ','
           << (S.cellOverhang[i] ? 1 : 0) << ',' << (S.cellArticulation[i] ? 1 : 0) << ','
           << (int)fixed[i] << '\n';
    std::printf("  wrote %s/catom3d_bond_forces.csv\n  wrote %s/catom3d_module_load.csv\n",
                out_dir.c_str(), out_dir.c_str());
    return 0;
}
