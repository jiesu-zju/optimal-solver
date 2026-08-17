# benchmarks/

`benchmark_spec.md` defines, in mathematical terms, every problem reported in
the manuscript — B1 (Nav2D–Hard bicycle navigation with quadratic soft-penetration
obstacles), B2 (planar quadruped template with soft contact costs), B3 (5-DOF
planar arm with UP/DOWN homotopy seeds) and the five-problem Gate suite used
for host validation.

The specification is **implementation-independent**: it includes exact dynamics,
cost and constraint formulas, all numerical constants, the RNG conventions
(`std::mt19937`, uniform/normal distributions), the seed-generation procedures,
and the frozen protocol parameters (reward, χ threshold, budgets, RNG ids).
Any constrained trajectory-optimization stack can instantiate these problems
from the document alone.

The same definitions are compiled into the released host binaries
(`../host/bin/`); `../verify/` checks that the released traces satisfy every
number printed in the paper.
