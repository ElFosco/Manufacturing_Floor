"""
Generalized item definition system for flexible manufacturing floor modeling.

This module allows easy definition of new item types with their operation sequences
and stage transitions without modifying the core model code.
"""

from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from utils.costant import K1, K2, B1, B2, P, KB, BP

@dataclass
class StageDefinition:
    """Defines a manufacturing stage with its operations and transitions."""
    name: str
    operations: List[str]  # List of operation codes for this stage
    internal_transitions: List[Tuple[str, str]]  # (from_op, to_op) for same-stage transitions
    next_stage_transition: Optional[str] = None  # Transition code to next stage

@dataclass
class ItemDefinition:
    """Complete definition of an item type with its manufacturing process."""
    name: str
    stages: List[StageDefinition]
    human_required_ops: List[str]  # Operations that require human (cannot be done by arms)
    
    def get_full_route(self) -> List[str]:
        """Get the complete operation route for this item."""
        route = []
        for stage in self.stages:
            route.extend(stage.operations)
        return route
    
    def get_stage_transitions(self) -> List[str]:
        """Get all stage-to-stage transitions for this item."""
        transitions = []
        for stage in self.stages:
            if stage.next_stage_transition:
                transitions.append(stage.next_stage_transition)
        return transitions
    
    def get_internal_transitions(self) -> List[Tuple[str, str]]:
        """Get all internal stage transitions for this item."""
        internal = []
        for stage in self.stages:
            internal.extend(stage.internal_transitions)
        return internal

# Predefined item definitions
ITEM_DEFINITIONS = {
    "FLASHLIGHT_CLIPPED": ItemDefinition(
        name="FLASHLIGHT_CLIPPED",
        stages=[
            StageDefinition(
                name="Kitting",
                operations=[K1],
                internal_transitions=[],
                next_stage_transition=KB
            ),
            StageDefinition(
                name="Assembly", 
                operations=[B1],
                internal_transitions=[],
                next_stage_transition=BP
            ),
            StageDefinition(
                name="Packing",
                operations=[P],
                internal_transitions=[],
                next_stage_transition=None
            )
        ],
        human_required_ops=[]
    ),
    
    "FLASHLIGHT_SCREWS": ItemDefinition(
        name="FLASHLIGHT_SCREWS", 
        stages=[
            StageDefinition(
                name="Kitting",
                operations=[K1, K2],
                internal_transitions=[],
                next_stage_transition=KB
            ),
            StageDefinition(
                name="Assembly",
                operations=[B1, B2], 
                internal_transitions=[(B1, B2)],  # B1 -> B2 transition within assembly
                next_stage_transition=BP
            ),
            StageDefinition(
                name="Packing",
                operations=[P],
                internal_transitions=[],
                next_stage_transition=None
            )
        ],
        human_required_ops=[K2]  # K2 requires human
    )
}

def get_item_definition(item_name: str) -> Optional[ItemDefinition]:
    """Get item definition by name."""
    return ITEM_DEFINITIONS.get(item_name)

def get_all_item_names() -> List[str]:
    """Get all available item names."""
    return list(ITEM_DEFINITIONS.keys())

def add_item_definition(item_def: ItemDefinition):
    """Add a new item definition."""
    ITEM_DEFINITIONS[item_def.name] = item_def

def get_operations_by_stage(item_name: str, stage_name: str) -> List[str]:
    """Get operations for a specific stage of an item."""
    item_def = get_item_definition(item_name)
    if not item_def:
        return []
    
    for stage in item_def.stages:
        if stage.name == stage_name:
            return stage.operations
    return []

def get_stage_transitions(item_name: str) -> List[str]:
    """Get all stage transitions for an item."""
    item_def = get_item_definition(item_name)
    if not item_def:
        return []
    return item_def.get_stage_transitions()

def get_internal_transitions(item_name: str) -> List[Tuple[str, str]]:
    """Get all internal transitions for an item."""
    item_def = get_item_definition(item_name)
    if not item_def:
        return []
    return item_def.get_internal_transitions()

def get_human_required_ops(item_name: str) -> List[str]:
    """Get operations that require human for an item."""
    item_def = get_item_definition(item_name)
    if not item_def:
        return []
    return item_def.human_required_ops
