// rb_test.hpp -- tiny assert-style test helper, matching the tools/test_*.cpp convention.
#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

namespace rbtest {

inline int g_pass = 0;
inline int g_fail = 0;

inline void check(bool cond, const std::string& what) {
    if (cond) { ++g_pass; }
    else { ++g_fail; std::cout << "  FAIL: " << what << "\n"; }
}

// absolute tolerance
inline void checkNear(double got, double want, const std::string& what, double tol = 1e-9) {
    bool ok = std::fabs(got - want) <= tol;
    if (!ok) std::cout << "  (got " << got << ", want " << want << ")\n";
    check(ok, what);
}

// relative tolerance (falls back to absolute near zero)
inline void checkClose(double got, double want, const std::string& what, double rel = 1e-9) {
    double denom = std::max(1.0, std::fabs(want));
    bool ok = std::fabs(got - want) <= rel * denom;
    if (!ok) std::cout << "  (got " << got << ", want " << want << ")\n";
    check(ok, what);
}

inline int summary(const char* name) {
    std::cout << name << ": " << g_pass << " passed, " << g_fail << " failed\n";
    return g_fail == 0 ? 0 : 1;
}

}  // namespace rbtest
