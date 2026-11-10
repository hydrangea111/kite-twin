"""Physical parameters for the tethered quad-line kite rig.

Values tagged MEASURE are estimates standing in for bench measurements.
Replace them once the hardware arrives -- every number downstream depends
on these, and the digital twin will fit the aero coefficients from flight logs.
"""

from dataclasses import dataclass
from math import sqrt

G = 9.81
RHO_AIR = 1.225


@dataclass(frozen=True)
class Kite:
    area: float = 0.50          # m^2, sail area net of the vent hole   MEASURE
    mass_canopy: float = 0.15   # kg, nylon + bamboo + bridle           MEASURE
    mass_payload: float = 0.30  # kg, perovskite panel or CanSat rig    MEASURE

    # Flat vented kite: low lift, high drag (L/D ~ 2). A delta or foil would
    # be 3-4x better, but the shield shape is fixed by the research framing.
    cl: float = 0.75            # FIT from flight log
    cd: float = 0.38            # FIT from flight log

    @property
    def mass(self) -> float:
        return self.mass_canopy + self.mass_payload


@dataclass(frozen=True)
class Payload:
    """Hangs below the kite on its own suspension line, free to swing."""
    mass: float = 0.30        # kg                                     MEASURE
    area: float = 0.05        # m^2, frontal area of panel / CanSatrig MEASURE
    cd: float = 1.2           # bluff body


@dataclass(frozen=True)
class Tether:
    name: str
    EA: float             # N, axial stiffness (Young's modulus * cross-section)
    damping_ratio: float  # fraction of critical, axial structural damping
    mass_per_m: float     # kg/m
    break_load: float     # N
    diameter: float = 0.0010  # m, for crossflow drag

    def stiffness(self, length: float) -> float:
        return self.EA / length

    def natural_freq(self, length: float, mass: float) -> float:
        """Longitudinal bounce mode [Hz]. Keep controller bandwidth well below this.

        `mass` is the inertia that actually resists the bounce, which is not
        the airframe mass alone: the canopy drags air along with it. Callers
        that care about the absolute number should pass
        kite.mass + model.added_mass(kite), which lowers this by 17% for the
        present kite. Ratios between tethers are untouched -- the same air
        rides on all of them -- so comparative conclusions do not depend on
        getting the added mass exactly right.
        """
        return sqrt(self.stiffness(length) / mass) / (2 * 3.141592653589793)


# EA back-calculated from break load / break strain, which is what a pull test
# on a spring scale actually gives you. Silk broke at 50-60 kgf in our test.
SILK = Tether(
    name="명주실 (silk)",
    EA=3000.0,          # 540 N / 0.18 break strain      MEASURE via stretch test
    damping_ratio=0.03,  # natural fibre, fairly lossy
    mass_per_m=0.0010,
    break_load=540.0,
)

DYNEEMA = Tether(
    name="다이니마 (UHMWPE)",
    EA=15000.0,         # same break load, ~3.5% break strain
    damping_ratio=0.01,
    mass_per_m=0.0006,
    break_load=540.0,
    diameter=0.0008,
)

RIGID = Tether(
    name="강체 근사 (rigid ref)",
    EA=150000.0,
    damping_ratio=0.01,
    mass_per_m=0.0010,
    break_load=540.0,
)

# A stiff line with a spring spliced into it. In series the spring dominates,
# so EA = k * L and the stiffness can be dialled to anything -- here matched to
# silk. What it cannot copy is silk's damping: a metal spring loses almost
# nothing per cycle where a natural fibre loses a few percent. Holding EA fixed
# and changing only damping_ratio is what separates the two effects.
SPRUNG = Tether(
    name="강체줄 + 용수철",
    EA=3000.0,           # same as silk, by choosing k = EA / L
    damping_ratio=0.002,  # steel spring, essentially lossless
    mass_per_m=0.0006,
    break_load=540.0,
    diameter=0.0008,
)

TETHERS = (SILK, SPRUNG, DYNEEMA, RIGID)

# Short silk segment between kite bridle and payload.
SUSPENSION = Tether(
    name="목줄 (suspension)",
    EA=3000.0,
    damping_ratio=0.05,
    mass_per_m=0.0010,
    break_load=540.0,
)

LINE_LENGTH = 50.0        # m, commanded unstretched length
SUSPENSION_LENGTH = 0.50  # m, kite-to-payload drop               MEASURE
WIND_BASE = 6.0           # m/s
CD_CYLINDER = 1.1         # crossflow drag coefficient of the line
