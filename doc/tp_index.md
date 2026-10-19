# Technical Proof Citation Index

This document maps the physics constants and model equations in `pool_physics`
to their published sources.  All TP citations refer to Dr. Dave Alciatore's
Technical Proof series at **billiards.colostate.edu**.

No equations or figures from these papers are reproduced verbatim in the
codebase.  The citations are provided so that readers can independently verify
the numerical values used.

---

## Cue–tip impact  (`lib/src/cue_impact.dart`)

| Constant / formula | Source | Notes |
|---|---|---|
| Tip-offset miscue limit 0.5R | TP 2.1 | Empirical maximum before tip skips |
| Energy transfer efficiency formula | TP A.22 | Accounts for contact eccentricity |
| Draw-offset geometric limit | TP A.30 | Cue-butt clearance constraint |
| Backspin-to-speed ratio at impact | TP B.7 | Tip-offset transfer function |
| Ball speed ratio from cue impact | TP B.1 | Mass ratio and restitution |
| Squirt angle — low-deflection cue: ±1.8° at 0.5R | TP A.31 | Measured for LD shafts |
| Squirt angle — standard cue: ±2.5° at 0.5R | TP A.31 | Measured for standard shafts |
| Effective lateral offset after squirt | TP B.7 | Resolves tip offset into post-squirt spin |

---

## Ball–cloth interaction  (`lib/src/sim.dart`)

| Constant / formula | Source | Notes |
|---|---|---|
| Sliding deceleration: `a = μ_s · g` | TP 4.1 | Coulomb kinetic friction |
| Natural-roll transition formula | TP A.18 | Time from sliding to rolling as function of spin and speed |
| Rolling deceleration: `a = μ_r · g` | TP B.12 | Rolling resistance |
| Cloth sliding friction range: 0.15–0.25 | Dr. Dave / TP A.17 | Typical pool-felt range; default 0.20 |
| Cloth rolling resistance range: 0.005–0.015 | TP B.12 | Typical pool-felt range; default 0.010 |
| Stopping distance at 0.174R optimum: 30.508 ft | TP B.12 | Calibration reference at 3 mph cue speed |
| Vertical-axis spin decay rate: 5–15 rad/s² | Dr. Dave | Measured from high-speed video |
| Maximum physical spin: 2500 RPM | Dr. Dave | Upper bound from all published measurements |

---

## Ball–ball collision  (`lib/src/sim.dart`)

| Constant / formula | Source | Notes |
|---|---|---|
| Ball restitution e = 0.93 | TP A.5 | Measured for standard pool balls |
| Normal impulse formula (equal masses) | TP A.5 | Newton's cradle with `e` |
| Tangential impulse (friction limit) | TP A.14 | Coulomb + no-slip reversal cap |
| Friction–slip velocity curve | TP A.14 / Marlow | `μ = 0.009951 + 0.108·exp(−1.088·v)` |
| Throw angle formula | TP A.24 | Verified for stun, follow, and draw |
| Equal-travel cut angle ≈ 33.5° | TP A.16 | Natural-roll condition for equal CB/OB distances |
| 5/7 rolling fraction at transition | TP A.20 | Analytic solution for rolling OB exit angle |
| Gearing English transfer efficiency | TP A.27 | Sidespin fraction passed to OB at contact |
| ¼-cue-point carom estimate | TP B.24 | Geometric approximation; <3.06° error over all cuts |
| 30° tangent-line rule | TP 3.3 | Natural-roll half-ball hit |

---

## Touching-ball chains  (`lib/src/hertz_contact.dart`)

| Constant / formula | Source | Notes |
|---|---|---|
| Hertz compliance constant: 7.266 × 10⁻⁷ 1/N | TP B.29 / Marlow | Derived from Marlow's measured contact duration |
| Three-ball velocity ratios: −0.071, 0.076, 0.995 | TP B.29 | Reference for elastic chain |
| Integration step: contact-duration / 10 000 | TP B.29 | Matches the paper's reported scheme |
| Viscoelastic damping for e = 0.93 | TP B.29 extension | Added dissipation to match two-ball restitution |

---

## Cushion rebound  (`lib/src/sim.dart`)

| Constant / formula | Source | Notes |
|---|---|---|
| 3-D impact geometry (nose height, approach angle) | Mathavan et al. (2010) | _Proc. IMechE Part C_, see below |
| Rail nose height — standard pool: 36.5 mm | Mathavan Table 1 | Nominal pool-table value |
| Rail nose height — Diamond Red Label: 34 mm | Measured / Diamond specs | Lower nose characteristic of Diamond tables |
| Cushion restitution: 0.75 | TP 6.3 / Mathavan | Speed-averaged value for typical felt-covered cushion |
| Pocket facing restitution: 0.72 (0.70 Diamond) | Diamond specs / TP 3.7 | 3/16″ 60 Shore A neoprene facing over wood sub-rail vs. 40–45 Shore A natural gum cushion |
| Impact duration (standard pool): 0.45 ms | Mathavan | Used for finite-contact damping weight |
| Impact duration (Diamond): 0.52 ms | Diamond characterisation | Slightly longer for harder rubber |
| Running English amplification | Mathavan Fig. 10 | Angle opens with same-direction sidespin |
| Reverse English reduction | Mathavan Fig. 10 | Angle narrows with opposite sidespin |
| Bank angle departure from geometric | TP 6.3 | Speed and friction dependence |

---

## Table geometry  (`lib/src/table_constants.dart`)

| Constant / formula | Source | Notes |
|---|---|---|
| Corner pocket facing angle: 38.32° | TP B.15 | Derived from 4.5″ mouth, 3.75″ throat, 2.0″ cushion depth |
| Side pocket mouth / facing angle: 5.0″ / 13.0° | TP 3.5 / BCA / WPA | 103° cut angle to rail nose across 2.25″ cushion depth |
| Side pocket shelf / hole radius: 0.1875″ / 2.5″ | TP 3.5 / BCA / Diamond | 3/16″ nominal shelf (0″–3/8″ BCA spec) before 2.5″-radius hole |
| Nine-foot table play surface: 100″ × 50″ | Diamond Pro-Am specs | `PhysicsTableConfig.nineFootDiamond` |
| Rail width: 7/114 of table width | Normalized from Diamond specs | Same fraction applies to both short and long rails |
| Ball diameter: 2.25 in (0.05715 m) | BCA / WPA | Standard pool ball |
| Ball mass: 6 oz (0.170 kg) | BCA / WPA | Standard pool ball |

---

## External references

**Dr. Dave Alciatore**, billiards.colostate.edu/technical_proofs.html
— TP series cited throughout.  The site is the authoritative source; do not
  rely on archived or third-party copies.

**Mathavan, S., Jackson, M. R., & Parkin, R. M.** (2010).
"A theoretical analysis of billiard ball dynamics under cushion impacts."
_Proceedings of the Institution of Mechanical Engineers, Part C:
Journal of Mechanical Engineering Science_, 224(9), 1863–1873.

**Marlow, W. C.** (1995). _The Physics of Pocket Billiards_.
ISBN 0-9643620-0-6.
— Source for the Hertz compliance constant and the ball–ball friction curve
  coefficients.
