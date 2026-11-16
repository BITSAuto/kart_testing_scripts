"""Drive-by-wire frame format, as in Drive_By_Wire/ver1.py.

    *A{a}B{throttle}C{c}D{steering}E{left_ind}F{horn}G{light}H{right_ind}I{brake}J{reverse}#

Throttle and steering are 0..100; steering 50 stops the steering motor. The rest are 0/1.
"""
import re
from dataclasses import asdict, dataclass, replace

FIELDS = ("a", "throttle", "c", "steering", "left_ind", "horn", "light", "right_ind", "brake",
          "reverse")
LETTERS = "ABCDEFGHIJ"
FRAME_RE = re.compile(r"\*A(\d+)B(\d+)C(\d+)D(\d+)E(\d+)F(\d+)G(\d+)H(\d+)I(\d+)J(\d+)#")


@dataclass(frozen=True)
class Command:
    a: int = 0
    throttle: int = 0
    c: int = 0
    steering: int = 50
    left_ind: int = 0
    horn: int = 0
    light: int = 0
    right_ind: int = 0
    brake: int = 0
    reverse: int = 0

    def frame(self) -> str:
        values = [int(getattr(self, f)) for f in FIELDS]
        return "*" + "".join(f"{l}{v}" for l, v in zip(LETTERS, values)) + "#"

    def with_(self, **changes) -> "Command":
        return replace(self, **changes)

    def as_dict(self):
        return asdict(self)


def parse_frame(text: str):
    """Command from a frame string, or None."""
    m = FRAME_RE.search(text)
    if not m:
        return None
    return Command(**{f: int(v) for f, v in zip(FIELDS, m.groups())})
