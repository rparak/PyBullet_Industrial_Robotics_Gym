from dataclasses import dataclass, field
import typing as tp

@dataclass
class Test_Str:
    a: float

@dataclass
class Cuboid_Str:
    a: tp.Union[None, Test_Str] = field(default_factory=tp.Union[None, Test_Str])

C = Cuboid_Str(a=1)

print(C.a)
