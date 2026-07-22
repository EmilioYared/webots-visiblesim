// =============================================================================
// test_scenario.cpp — stress scenarios + per-module structural classification.
//
// Verifies that buildCantilever produces a structure with the properties the
// live risk visualization relies on, and that analyzeStructure's per-cell flags
// line up with the aggregate counts and with the intended geometry.
// =============================================================================
#include "../controllers/catom3d_controller/catom3d_core.hpp"
#include "../controllers/catom3d_controller/scene_config.hpp"
#include "../controllers/catom3d_controller/scenario.hpp"
#include <iostream>
#include <set>
#include <tuple>

static int g_fail = 0, g_pass = 0;
static void check(bool c, const std::string& w){ if(c)++g_pass; else {++g_fail; std::cout<<"  FAIL: "<<w<<"\n";} }

int main() {
    // ---- a flat, fully-supported sheet has no risk flags ----
    {
        std::vector<GridPos> sheet;
        for (int y=0;y<4;++y) for (int x=0;x<5;++x) sheet.push_back({x,y,2});
        StructuralReport r = analyzeStructure(sheet);
        check(r.components == 1,   "sheet is one component");
        check(r.articulation == 0, "sheet has no articulation points");
        check(r.overhanging == 0,  "sheet has no overhangs (all on min layer)");
        check(r.comInSupport,      "sheet CoM in support");
        check((int)r.cellArticulation.size() == (int)sheet.size(), "flag array sized to cells");
        bool anyFlag=false;
        for (size_t i=0;i<sheet.size();++i) if (r.cellArticulation[i]||r.cellOverhang[i]) anyFlag=true;
        check(!anyFlag, "no per-cell risk flags on a healthy sheet");
    }

    // ---- cantilever: overhang, SPOFs, and CoM out of support ----
    {
        SceneConfig c;                       // defaults: 2x2x3 base, arm 8, x0=5,y0=1
        std::vector<GridPos> cells = buildCantilever(c);
        int expected = c.cantBaseW*c.cantBaseD*c.cantBaseH + c.cantArmLen;
        check((int)cells.size() == expected, "cantilever module count matches params");

        // No duplicate cells.
        std::set<std::tuple<int,int,int>> uniq;
        for (auto& g : cells) uniq.insert(std::make_tuple(g.x,g.y,g.z));
        check(uniq.size() == cells.size(), "cantilever cells are unique");

        StructuralReport r = analyzeStructure(cells);
        check(r.components == 1,    "cantilever is connected (one component)");
        check(r.articulation > 0,   "cantilever has single-points-of-failure");
        check(r.overhanging > 0,    "cantilever arm overhangs");
        check(!r.comInSupport,      "long arm pushes CoM out of the base footprint");

        // Flags line up with counts.
        int artCount=0, ovhCount=0;
        for (size_t i=0;i<cells.size();++i){ if(r.cellArticulation[i])artCount++; if(r.cellOverhang[i])ovhCount++; }
        check(artCount == r.articulation, "cellArticulation count == aggregate");
        check(ovhCount == r.overhanging,  "cellOverhang count == aggregate");

        // The pedestal (z==0 layer) is never an overhang; the far arm tip is.
        const int armZ = c.cantBaseH - 1;
        for (size_t i=0;i<cells.size();++i){
            if (cells[i].z == 0) check(!r.cellOverhang[i], "ground-layer cell is not overhanging");
        }
        // The last cell is the arm tip — must be overhanging.
        check(r.cellOverhang.back(), "arm tip is flagged overhanging");
        // Arm tip sits on the arm layer, far from the base in x.
        check(cells.back().z == armZ && cells.back().x == c.cantX0 + c.cantBaseW + c.cantArmLen - 1,
              "arm tip is at the expected cell");
    }

    // ---- a short arm keeps the CoM over the base (no tip-over) ----
    {
        SceneConfig c; c.cantArmLen = 1;
        StructuralReport r = analyzeStructure(buildCantilever(c));
        check(r.comInSupport, "short arm keeps CoM in support");
    }

    // ---- buildScenarioCells dispatches on type ----
    {
        SceneConfig c;
        c.scenarioType = "walk";
        check(buildScenarioCells(c).empty(), "walk scenario yields no static cells");
        c.scenarioType = "cantilever";
        check(!buildScenarioCells(c).empty(), "cantilever scenario yields cells");
    }

    std::cout << "test_scenario: " << g_pass << " passed, " << g_fail << " failed\n";
    return g_fail == 0 ? 0 : 1;
}
