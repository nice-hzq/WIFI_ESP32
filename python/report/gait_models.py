# gait_models.py - 数据模型定义
import json
from dataclasses import dataclass, asdict, field
from typing import Dict, Any

@dataclass
class CadenceData:
    left: float = 0.0
    right: float = 0.0

@dataclass
class WalkingSpeedData:
    left: float = 0.0
    right: float = 0.0

@dataclass
class StrideTimeData:
    left: float = 0.0
    right: float = 0.0

@dataclass
class StepLengthData:
    left: float = 0.0
    right: float = 0.0

@dataclass
class StepTimeData:
    left: float = 0.0
    right: float = 0.0

@dataclass
class ValidStepsData:
    left: int = 0
    right: int = 0

@dataclass
class SupportPhaseData:
    left: float = 0.0
    right: float = 0.0

@dataclass
class SwingPhaseData:
    left: float = 0.0
    right: float = 0.0

@dataclass
class ShankImpact:
    left: float = 0.0
    right: float = 0.0
@dataclass
class FootImpact:
    left: float = 0.0
    right: float = 0.0

@dataclass
class BasicParameters:
    turnSteps: int = 0
    turnDuration: float = 0.0
    stepWidth: float = 0.0
    cadence: CadenceData = None
    gaitCycle: float = 0.0
    walkingSpeed: WalkingSpeedData = None
    walkingDistance: float = 0.0
    totalSteps: int = 0
    strideTime: StrideTimeData = None
    
    def __post_init__(self):
        if self.cadence is None:
            self.cadence = CadenceData()
        if self.walkingSpeed is None:
            self.walkingSpeed = WalkingSpeedData()
        if self.strideTime is None:
            self.strideTime = StrideTimeData()

@dataclass
class StepParameters:
    stepLength: StepLengthData = None
    stepLengthDeviation: float = 0.0
    strideLength: float = 0.0
    stepTime: StepTimeData = None
    validSteps: ValidStepsData = None
    footLiftHeight: StepLengthData = None
    
    def __post_init__(self):
        if self.stepLength is None:
            self.stepLength = StepLengthData()
        if self.stepTime is None:
            self.stepTime = StepTimeData()
        if self.validSteps is None:
            self.validSteps = ValidStepsData()
        if self.footLiftHeight is None:
            self.footLiftHeight = StepLengthData()

@dataclass
class PhaseParameters:
    doubleSupportPhase: float = 0.0
    supportPhase: SupportPhaseData = None
    swingPhase: SwingPhaseData = None

    def __post_init__(self):
        if self.supportPhase is None:
            self.supportPhase = SupportPhaseData()
        if self.swingPhase is None:
            self.swingPhase = SwingPhaseData()

@dataclass
class ImpactParameters:
    shankImpact: ShankImpact = None
    footImpact: FootImpact = None

    def __post_init__(self):
        if self.shankImpact is None:
            self.shankImpact = ShankImpact()
        if self.footImpact is None:
            self.footImpact = FootImpact()


# ============================================================
# 关节角度 (Joint Angles)
# ============================================================
@dataclass
class JointAngleLR:
    """单个角度的左/右值 (度)"""
    left: float = 0.0
    right: float = 0.0


@dataclass
class SingleJointAngles:
    """单个关节的角度分量"""
    flexion: JointAngleLR = None     # 屈曲(+) / 伸展(-)
    abduction: JointAngleLR = None   # 外展(+) / 内收(-)
    rotation: JointAngleLR = None    # 内旋(+) / 外旋(-)

    def __post_init__(self):
        if self.flexion is None:
            self.flexion = JointAngleLR()
        if self.abduction is None:
            self.abduction = JointAngleLR()
        if self.rotation is None:
            self.rotation = JointAngleLR()


@dataclass
class JointParameters:
    """下肢三大关节角度 — 髋/膝/踝"""
    hip: SingleJointAngles = None
    knee: SingleJointAngles = None
    ankle: SingleJointAngles = None

    def __post_init__(self):
        if self.hip is None:
            self.hip = SingleJointAngles()
        if self.knee is None:
            self.knee = SingleJointAngles()
        if self.ankle is None:
            self.ankle = SingleJointAngles()


@dataclass
class GaitAnalysisResult:
    version: str = "1.0"
    timestamp: int = 0
    sessionDuration: float = 0.0

    basicParameters: BasicParameters = None
    stepParameters: StepParameters = None
    phaseParameters: PhaseParameters = None
    impactParameters: ImpactParameters = None
    jointParameters: JointParameters = None

    def __post_init__(self):
        if self.basicParameters is None:
            self.basicParameters = BasicParameters()
        if self.stepParameters is None:
            self.stepParameters = StepParameters()
        if self.phaseParameters is None:
            self.phaseParameters = PhaseParameters()
        if self.impactParameters is None:
            self.impactParameters = ImpactParameters()
        if self.jointParameters is None:
            self.jointParameters = JointParameters()

    def to_dict(self):
        return asdict(self)

    def to_json(self):
        return json.dumps(self.to_dict(), ensure_ascii=False, indent=2)