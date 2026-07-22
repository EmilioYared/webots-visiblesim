// =============================================================================
// test_json.cpp — unit tests for json_min.hpp (no Webots, no test framework).
// =============================================================================
#include "../controllers/catom3d_controller/json_min.hpp"
#include <iostream>
#include <cmath>

static int g_fail = 0, g_pass = 0;

static void check(bool cond, const std::string& what) {
    if (cond) { ++g_pass; }
    else { ++g_fail; std::cout << "  FAIL: " << what << "\n"; }
}
static void checkNear(double got, double want, const std::string& what) {
    bool ok = std::fabs(got - want) < 1e-9;
    if (!ok) std::cout << "  (got " << got << ", want " << want << ")\n";
    check(ok, what);
}

int main() {
    using namespace jsonm;

    // ---- scalars ----
    {
        Result r = parse("{\"a\":1,\"b\":\"two\",\"c\":true,\"d\":false,\"e\":null}");
        check(r.ok, "parse flat object: " + r.error);
        checkNear(numberOr(r.value, "a", -1), 1, "number");
        check(stringOr(r.value, "b", "") == "two", "string");
        check(boolOr(r.value, "c", false) == true,  "bool true");
        check(boolOr(r.value, "d", true)  == false, "bool false");
        const Value* e = r.value.at("e");
        check(e && e->isNull(), "null");
    }

    // ---- number forms ----
    {
        Result r = parse("{\"a\":-2.5,\"b\":1e-5,\"c\":1.5E3,\"d\":0,\"e\":1E+2}");
        check(r.ok, "parse numbers: " + r.error);
        checkNear(numberOr(r.value, "a", 0), -2.5,   "negative decimal");
        checkNear(numberOr(r.value, "b", 0), 1e-5,   "negative exponent");
        checkNear(numberOr(r.value, "c", 0), 1500.0, "capital E exponent");
        checkNear(numberOr(r.value, "d", -1), 0.0,   "zero");
        checkNear(numberOr(r.value, "e", 0), 100.0,  "explicit + exponent");
    }

    // ---- nesting and dotted paths ----
    {
        Result r = parse("{\"world\":{\"floor\":{\"size\":[1.3,0.45,0.01],\"on\":true}}}");
        check(r.ok, "parse nested: " + r.error);
        double v[3] = {0,0,0};
        check(vec3Into(r.value, "world.floor.size", v), "vec3 lookup");
        checkNear(v[0], 1.3,  "vec3[0]");
        checkNear(v[1], 0.45, "vec3[1]");
        checkNear(v[2], 0.01, "vec3[2]");
        check(boolOr(r.value, "world.floor.on", false), "deep bool");
        check(r.value.at("world.floor.missing") == nullptr, "missing leaf -> nullptr");
        check(r.value.at("nope.nope")          == nullptr, "missing branch -> nullptr");
        checkNear(numberOr(r.value, "world.floor.absent", 42), 42, "default on missing");
        // wrong type must fall back to the default rather than coerce
        checkNear(numberOr(r.value, "world.floor.on", 7), 7, "default on type mismatch");
    }

    // ---- arrays ----
    {
        Result r = parse("[1,\"a\",{\"b\":2},[3]]");
        check(r.ok, "parse top-level array: " + r.error);
        check(r.value.isArray() && r.value.arr.size() == 4, "array size");
        check(r.value.arr[2].isObject(), "object inside array");
        checkNear(r.value.arr[3].arr[0].num, 3, "nested array element");
        Result e = parse("[]");
        check(e.ok && e.value.isArray() && e.value.arr.empty(), "empty array");
        Result eo = parse("{}");
        check(eo.ok && eo.value.isObject() && eo.value.obj.empty(), "empty object");
    }

    // ---- string escapes ----
    {
        Result r = parse("{\"s\":\"a\\\"b\\\\c\\nd\\te\\u0041\\u00e9\"}");
        check(r.ok, "parse escapes: " + r.error);
        std::string s = stringOr(r.value, "s", "");
        check(s == "a\"b\\c\nd\teA\xc3\xa9", "escape decoding incl. \\u -> UTF-8");
    }

    // ---- comment extension ----
    {
        Result r = parse("{ // line comment\n \"a\":1, /* block\n comment */ \"b\":2 }");
        check(r.ok, "parse with comments: " + r.error);
        checkNear(numberOr(r.value, "a", 0), 1, "value before comment");
        checkNear(numberOr(r.value, "b", 0), 2, "value after comment");
    }

    // ---- malformed input must be rejected, not silently accepted ----
    {
        const char* bad[] = {
            "{",                    // unterminated object
            "{\"a\":}",             // missing value
            "{\"a\" 1}",            // missing colon
            "{\"a\":1,}",           // trailing comma
            "[1,2",                 // unterminated array
            "{\"a\":\"unterminated}",
            "{\"a\":1} extra",      // trailing content
            "{\"a\":--1}",          // malformed number
            "{\"a\":1e}",           // malformed exponent
            "tru",                  // bad literal
            ""                      // empty input
        };
        for (size_t i = 0; i < sizeof(bad)/sizeof(bad[0]); ++i) {
            Result r = parse(bad[i]);
            check(!r.ok, std::string("must reject: ") + bad[i]);
            check(!r.ok && !r.error.empty(), std::string("error message for: ") + bad[i]);
        }
    }

    // ---- leaf-path collection (drives unknown-key detection) ----
    {
        Result r = parse("{\"a\":{\"b\":1,\"c\":[1,2]},\"d\":2}");
        std::vector<std::string> paths;
        collectPaths(r.value, "", paths);
        check(paths.size() == 3, "three leaf paths");
        bool hasAB = false, hasAC = false, hasD = false;
        for (size_t i = 0; i < paths.size(); ++i) {
            if (paths[i] == "a.b") hasAB = true;
            if (paths[i] == "a.c") hasAC = true;   // array counts as a leaf
            if (paths[i] == "d")   hasD  = true;
        }
        check(hasAB && hasAC && hasD, "leaf paths are dotted and complete");
    }

    // ---- missing file ----
    {
        Result r = parseFile("definitely_not_here_12345.json");
        check(!r.ok && r.error.find("cannot open") != std::string::npos,
              "missing file reports cleanly");
    }

    std::cout << "test_json: " << g_pass << " passed, " << g_fail << " failed\n";
    return g_fail == 0 ? 0 : 1;
}
