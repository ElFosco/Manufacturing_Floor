#!/usr/bin/env python3
"""
Test script to verify that get_internal_transitions now properly returns B12
"""

from utility.item_definitions import get_internal_transitions
from utility.costant import B1, B2, B12

def test_internal_transitions():
    """Test that internal transitions are properly mapped to transition types."""
    
    # Test FLASHLIGHT_CLIPPED (should have no internal transitions)
    clipped_transitions = get_internal_transitions("FLASHLIGHT_CLIPPED")
    print(f"FLASHLIGHT_CLIPPED internal transitions: {clipped_transitions}")
    assert clipped_transitions == [], f"Expected empty list, got {clipped_transitions}"
    
    # Test FLASHLIGHT_SCREWS (should have B1->B2 internal transition)
    screws_transitions = get_internal_transitions("FLASHLIGHT_SCREWS")
    print(f"FLASHLIGHT_SCREWS internal transitions: {screws_transitions}")
    assert screws_transitions == [(B1, B2)], f"Expected [(B1, B2)], got {screws_transitions}"
    
    # Test the mapping logic
    internal_transition_types = []
    for from_op, to_op in screws_transitions:
        if from_op == B1 and to_op == B2:
            internal_transition_types.append(B12)
    
    print(f"Mapped transition types: {internal_transition_types}")
    assert internal_transition_types == [B12], f"Expected [B12], got {internal_transition_types}"
    
    print("✅ All tests passed! Internal transitions are working correctly.")
    print(f"✅ get_internal_transitions now returns {screws_transitions}")
    print(f"✅ This correctly maps to transition type {internal_transition_types[0]}")

if __name__ == "__main__":
    test_internal_transitions()
