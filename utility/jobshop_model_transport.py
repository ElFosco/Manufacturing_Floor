from abc import ABC, abstractmethod
from collections import defaultdict
from termios import B2400

import cpmpy as cp
import pandas as pd
import plotly.express as px
from numpy.ma.core import shape
import time
from utility.costant import K1, B1, B2, K2, P, KB, BP, B12, KIT, FLASHLIGHT_CLIPPED, ASM, \
    FLASHLIGHT_SCREWS, PACK
from utility.jobshop_model import FJProblem
from utility.item_definitions import get_item_definition, get_stage_transitions, get_internal_transitions


class FJTransportProblem(FJProblem):
    def __init__(self,ILP_formulation=False,ILP_opt=False,symmetry_breaking=False):
        super().__init__(ILP_formulation=ILP_formulation,ILP_opt=ILP_opt,symmetry_breaking=symmetry_breaking)
        self.connected_via_conveyor = []
        self.set_t_conv(3)

    def add_transport(self, from_machine_id, to_machine_id):
            self.connected_via_conveyor.append((from_machine_id, to_machine_id))


    def set_t_conv(self,t_conv):
        self.t_conv = t_conv

    def get_all_transitions(self):
        """Get all possible transitions across all items."""
        all_transitions = set()
        for item in self.I:
            item_type = self.items_to_build[item]
            stage_transitions = get_stage_transitions(item_type)
            internal_transitions = get_internal_transitions(item_type)
            
            all_transitions.update(stage_transitions)
            
            # Map internal transition pairs to their transition types
            for from_op, to_op in internal_transitions:
                if from_op == B1 and to_op == B2:
                    all_transitions.add(B12)
                # Add more mappings here for future internal transitions
        
        return list(all_transitions)


    def _pair_from_entry(self, entry):
        return (entry[0], entry[1]) if len(entry) == 2 else (entry[1], entry[2])

    def _conv_pairs(self, trans):
        """Get conveyor pairs for a specific transition type."""
        WS_ASM = self.WS_GRIP + self.WS_SCREW
        pairs = []
        
        for entry in self.connected_via_conveyor:
            m1, m2 = self._pair_from_entry(entry)
            
            # Stage transitions (between different stages)
            if trans == KB and (m1 in self.WS_KITTING) and (m2 in WS_ASM):
                pairs.append((m1, m2))
            elif trans == BP and (m1 in WS_ASM) and (m2 in self.WS_PACKING):
                pairs.append((m1, m2))
            
            # Internal transitions (within the same stage)
            elif trans == B12 and (m1 in WS_ASM) and (m2 in WS_ASM):
                pairs.append((m1, m2))
            
            # For future extensions, you can add more transition types here
            # The system is now flexible to handle any transition type
            
        return pairs


    def _conv_opts(self, i, trans):
        opts = []
        pairs = self._conv_pairs(trans)
        for m1, m2 in pairs:
            key = (i, trans, ("conv", m1, m2))
            if key in self.zT:
                opts.append(self.zT[key])
        return opts

    def define_dv(self):
        self.x_h, self.x_a = {}, {}

        # processing vars only for present ops
        for i in self.I:
            for op in self.OPS_i[i]:
                for m in self.M_of[op]:
                    self.x_h[i, op, m] = cp.boolvar(name=f"xH_{i}_{op}_{m}")
                    if self._arm_allowed(op, m):
                        self.x_a[i, op, m] = cp.boolvar(name=f"xA_{i}_{op}_{m}")

        # timing only for present ops
        self.S = {(i, op): cp.intvar(0, self.horizon, name=f"S_{i}_{op}")
                  for i in self.I for op in self.OPS_i[i]}
        self.E = {(i, op): cp.intvar(0, self.horizon, name=f"E_{i}_{op}")
                  for i in self.I for op in self.OPS_i[i]}
        self.makespan = cp.intvar(0, self.horizon, name="Makespan")

        # flags for processing ops
        self.OP_flag, self.H_flag = {}, {}
        for i in self.I:
            for op in self.OPS:
                if not self.z[i, op]: continue
                self.H_flag[i, op] = cp.boolvar(name=f"H_{i}_{op}")
                for m in self.M_of[op]:
                    self.OP_flag[i, op, m] = cp.boolvar(name=f"OP_flag_{i}_{op}_{m}")

        # machine→ops map
        self.OPS_on_WS = {}
        for op in self.OPS:
            for m in self.M_of[op]:
                self.OPS_on_WS.setdefault(m, set()).add(op)

        # transport vars (human or exactly one conveyor link if needed)
        self.S_T, self.E_T, self.zT = {}, {}, {}
        
        # Get all possible transitions dynamically
        all_transitions = self.get_all_transitions()
        
        for i in self.I:
            for trans in all_transitions:
                self.S_T[i, trans] = cp.intvar(0, self.horizon, name=f"S_T_{i}_{trans}")
                self.E_T[i, trans] = cp.intvar(0, self.horizon, name=f"E_T_{i}_{trans}")
                # Only add human transport option if humans are present
                if self.resources.get('human', []):
                    self.zT[i, trans, "human"] = cp.boolvar(name=f"T_{i}_{trans}_human")
                
                # Add variables for direct conveyor paths only
                pairs = self._conv_pairs(trans)
                for m1, m2 in pairs:
                    self.zT[i, trans, ("conv", m1, m2)] = cp.boolvar(name=f"T_{i}_{trans}_conv_{m1}_{m2}")

    def define_constraints(self):
        self.m = cp.Model()

        # exactly-one assignment + flags
        for i in self.I:
            for op in self.OPS:
                if not self.z[i, op]: continue
                if self.resources.get('human', []):
                    self.m += (
                        sum(self.x_h.get((i, op, m), 0) for m in self.M_of[op]) +
                        sum(self.x_a.get((i, op, m), 0) for m in self.M_of[op])
                    ) == 1
                    for m in self.M_of[op]:
                        self.m += self.OP_flag[i, op, m] == (self.x_h.get((i, op, m), 0) + self.x_a.get((i, op, m), 0))
                        self.m += self.OP_flag[i, op, m] <= 1
                    self.m += self.H_flag[i, op] == sum(self.x_h.get((i, op, m), 0) for m in self.M_of[op])
                else:
                    # Only arms available
                    self.m += sum(self.x_a.get((i, op, m), 0) for m in self.M_of[op]) == 1
                    for m in self.M_of[op]:
                        self.m += self.OP_flag[i, op, m] == self.x_a.get((i, op, m), 0)
                        self.m += self.OP_flag[i, op, m] <= 1
                    self.m += self.H_flag[i, op] == 0  # No human work

        # --- Force K1 and K2 on the same kitting workstation ---
        for i in self.I:
            if self.z.get((i, K1), 0) and self.z.get((i, K2), 0):
                for m in self.WS_KITTING:
                    self.m += self.OP_flag[i, K1, m] == self.OP_flag[i, K2, m]

        # durations
        for i in self.I:
            for op in self.OPS:
                if not self.z[i, op]: continue
                # Human duration only if humans are available
                if self.resources.get('human', []):
                    hum_sum = sum(self.HUM(op) * self.x_h[i, op, m] for m in self.M_of[op] if (i, op, m) in self.x_h)
                else:
                    hum_sum = 0
                # Arm duration
                arm_sum = sum(self.ARM(op, m) * self.x_a[i, op, m] for m in self.M_of[op] if (i, op, m) in self.x_a)
                self.m += self.E[i, op] == self.S[i, op] + hum_sum + arm_sum

        # processing precedence from per-item routes
        for i in self.I:
            for pred, succ in self.PREC_i[i]:
                self.m += self.S[i, succ] >= self.E[i, pred]

        # ----- transport precedence (dynamic) & need -----
        def _k_ops(i):  return [op for op in self.OPS_i[i] if op in (K1, K2)]
        def _b_ops(i):  return [op for op in self.OPS_i[i] if op in (B1, B2)]

        WS_ASM = self.WS_GRIP + self.WS_SCREW

        for i in self.I:
            kops = _k_ops(i)
            bops = _b_ops(i)
            item_type = self.items_to_build[i]
            
            # Get transitions for this specific item
            stage_transitions = get_stage_transitions(item_type)
            internal_transitions = get_internal_transitions(item_type)
            
            # Handle stage transitions (between different stages)
            for trans in stage_transitions:
                if trans == KB and kops and bops:
                    # Kitting to Assembly
                    prevK, firstB = kops[-1], bops[0]
                    self.m += self.S_T[i, KB] >= self.E[i, prevK]
                    self.m += self.S[i, firstB] >= self.E_T[i, KB]
                elif trans == BP and bops and self.z[i, P]:
                    # Assembly to Packing
                    lastB = bops[-1]
                    self.m += self.S_T[i, BP] >= self.E[i, lastB]
                    self.m += self.S[i, P] >= self.E_T[i, BP]
            
            # Handle internal transitions (within the same stage)
            for from_op, to_op in internal_transitions:
                if from_op == B1 and to_op == B2 and len(bops) >= 2:
                    # B1 to B2 transition within assembly - use B12 as transition type
                    b_from, b_to = bops[0], bops[1]
                    self.m += self.S_T[i, B12] >= self.E[i, b_from]
                    self.m += self.S[i, b_to] >= self.E_T[i, B12]

            # choose transport modes and durations (dynamic)
            # Map internal transition pairs to their transition types
            internal_transition_types = []
            for from_op, to_op in internal_transitions:
                if from_op == B1 and to_op == B2:
                    internal_transition_types.append(B12)
                # Add more mappings here for future internal transitions
            
            for trans in stage_transitions + internal_transition_types:
                # Determine if this transition is needed for this item
                need_trans = False
                if trans == KB and kops and bops:
                    need_trans = True
                elif trans == BP and bops and self.z[i, P]:
                    need_trans = True
                elif trans == B12 and len(bops) >= 2:
                    need_trans = True
                
                if need_trans:
                    opts = self._conv_opts(i, trans)
                    
                    # Calculate transport time for each option
                    transport_time = 0
                    if (i, trans, "human") in self.zT:
                        transport_time += self.zT[i, trans, "human"] * self.t_hum
                    
                    # Add direct conveyor time
                    pairs = self._conv_pairs(trans)
                    for m1, m2 in pairs:
                        key = (i, trans, ("conv", m1, m2))
                        if key in self.zT:
                            transport_time += self.zT[key] * self.t_conv
                    
                    if (i, trans, "human") in self.zT:
                        self.m += self.zT[i, trans, "human"] + sum(opts) == 1
                    else:
                        # No human transport available, must use conveyor
                        self.m += sum(opts) == 1
                    
                    self.m += self.E_T[i, trans] == self.S_T[i, trans] + transport_time


        # gate conveyor usage by endpoints (dynamic)
        for i in self.I:
            kops = _k_ops(i)
            bops = _b_ops(i)
            item_type = self.items_to_build[i]
            
            # Get transitions for this specific item
            stage_transitions = get_stage_transitions(item_type)
            internal_transitions = get_internal_transitions(item_type)
            
            # Handle stage transitions
            for trans in stage_transitions:
                if trans == KB and kops and bops:
                    prevK, firstB = kops[-1], bops[0]
                    pairs = self._conv_pairs(KB)
                    for m1, m2 in pairs:
                        key = (i, KB, ("conv", m1, m2))
                        if key in self.zT:
                            self.m += self.zT[key] <= self.OP_flag.get((i, prevK, m1), 0)
                            self.m += self.zT[key] <= self.OP_flag.get((i, firstB, m2), 0)
                elif trans == BP and bops and self.z[i, P]:
                    lastB = bops[-1]
                    pairs = self._conv_pairs(BP)
                    for m1, m2 in pairs:
                        key = (i, BP, ("conv", m1, m2))
                        if key in self.zT:
                            self.m += self.zT[key] <= self.OP_flag.get((i, lastB, m1), 0)
                            self.m += self.zT[key] <= self.OP_flag.get((i, P, m2), 0)
            
            # Handle internal transitions
            for from_op, to_op in internal_transitions:
                if from_op == B1 and to_op == B2 and len(bops) >= 2:
                    b_from, b_to = bops[0], bops[1]
                    pairs = self._conv_pairs(B12)
                    for m1, m2 in pairs:
                        key = (i, B12, ("conv", m1, m2))
                        if key in self.zT:
                            self.m += self.zT[key] <= self.OP_flag.get((i, b_from, m1), 0)
                            self.m += self.zT[key] <= self.OP_flag.get((i, b_to, m2), 0)

        # capacities: machines
        for m, ops_here in self.OPS_on_WS.items():
            starts_m, durs_m, ends_m, demands_m = [], [], [], []
            for i in self.I:
                for op in ops_here:
                    if not self.z[i, op]: continue
                    starts_m.append(self.S[i, op])
                    xh = self.x_h.get((i, op, m), 0) if self.resources.get('human', []) else 0
                    xa = self.x_a.get((i, op, m), 0)
                    arm_dur_m = self.ARM(op, m) if (i, op, m) in self.x_a else 0
                    hum_dur_m = self.HUM(op) if self.resources.get('human', []) else 0
                    dur_i_m = hum_dur_m * xh + arm_dur_m * xa
                    durs_m.append(dur_i_m)
                    ends_m.append(self.S[i, op] + dur_i_m)
                    demands_m.append(self.OP_flag[i, op, m])
            if starts_m:
                self.m += cp.Cumulative(starts_m, durs_m, ends_m, demands_m, capacity=1)

        # Capacity: human (processing + human transports) ----------
        # Only apply human capacity constraints if humans are present
        if self.resources.get('human', []):
            starts_h, durs_h, ends_h, demands_h = [], [], [], []
            for i in self.I:
                # processing (unchanged)
                for op in self.OPS:
                    if not self.z[i, op]: continue
                    starts_h.append(self.S[i, op])
                    dH = self.HUM(op) * self.H_flag[i, op]
                    durs_h.append(dH)
                    ends_h.append(self.S[i, op] + dH)  # matches dH
                    demands_h.append(self.H_flag[i, op])

                # human transports (end must match human duration only)
                all_transitions = self.get_all_transitions()
                for trans in all_transitions:
                    if (i, trans, "human") in self.zT:
                        zH = self.zT[i, trans, "human"]
                        starts_h.append(self.S_T[i, trans])
                        durs_h.append(self.t_hum * zH)
                        ends_h.append(self.S_T[i, trans] + self.t_hum * zH)  # << was E_T, now aligned
                        demands_h.append(zH)

            if starts_h:
                self.m += cp.Cumulative(starts_h, durs_h, ends_h, demands_h, capacity=1)

        # ---------- Capacity: conveyors (per physical link) ----------
        from collections import defaultdict
        per_conv = defaultdict(list)  # (m1,m2) -> list of (S_T, zConv)
        for entry in self.connected_via_conveyor:
            m1, m2 = self._pair_from_entry(entry)
            per_conv[(m1, m2)]  # ensure key

        all_transitions = self.get_all_transitions()
        for i in self.I:
            for (m1, m2) in list(per_conv.keys()):
                for trans in all_transitions:
                    key = (i, trans, ("conv", m1, m2))
                    if key in self.zT:
                        zC = self.zT[key]
                        S = self.S_T[i, trans]
                        per_conv[(m1, m2)].append((S, zC))

        for (m1, m2), items in per_conv.items():
            if not items:
                continue
            starts = [S for (S, z) in items]
            durs = [self.t_conv * z for (S, z) in items]
            ends = [S + self.t_conv * z for (S, z) in items]  # << was E_T, now aligned
            demands = [z for (S, z) in items]
            self.m += cp.Cumulative(starts, durs, ends, demands, capacity=1)

    def optimize(self):
        if self.ILP_opt:
            for i in self.I:
                self.m += (self.makespan >= self.E[i, self.OPS_i[i][-1]])
            self.m.minimize(self.makespan)
        else:
            last_E = [self.E[i, self.OPS_i[i][-1]] for i in self.I]
            self.m += (self.makespan == cp.max(last_E))
            self.m.minimize(self.makespan)

    def define_symmetry_breaking(self):
        # """
        # Symmetry breaking (CP-native):
        #   1) Lex order between items of the same class (by their route's start times).
        #   2) Machine activation ordering within identical machine groups (type,subtype).

        #   4) Human lex ordering: if two items both pick Human for an op, order by item id.
        # """
        #
        # -------- helpers --------
        def or_all(lits):
            """OR over a list of Bool expressions/vars; returns None if empty."""
            lit = None
            for v in lits:
                if v is None:
                    continue
                lit = v if lit is None else (lit | v)
            return lit

        # Map machine id -> (type, subtype)
        id2type, id2sub = {}, {}
        for typ, lst in self.resources.items():
            if typ == 'human':
                continue
            for mid, sub in lst:
                id2type[mid] = typ
                id2sub[mid] = sub

        # For each op, group machines by identical (type,subtype) for that op
        def identical_machine_groups_for_op(op):
            groups = {}
            for m in self.M_of.get(op, []):
                key = (id2type.get(m, None), id2sub.get(m, None))
                groups.setdefault(key, []).append(m)
            return [sorted(v) for v in groups.values() if len(v) >= 2]

        # y[i,op,m] = 1 iff (i,op) uses machine m (human or arm)
        y = {}
        for i in self.I:
            for op in self.OPS:
                if not self.z.get((i, op), 0):
                    continue
                for m in self.M_of[op]:
                    lit = or_all([self.x_h.get((i, op, m), None),
                                  self.x_a.get((i, op, m), None)])
                    if lit is not None:
                        y[(i, op, m)] = lit

        # -------- (1) Lex order between items of the same class --------
        # Group items by their type (FLASHLIGHT_CLIPPED, FLASHLIGHT_SCREWS, etc.)
        classes = {}
        for i in self.I:
            classes.setdefault(self.items_to_build[i], []).append(i)

        for items in classes.values():
            items = sorted(items)
            if len(items) < 2:
                continue
            
            # Get the route for this item type using the new system
            item_type = self.items_to_build[items[0]]
            from utility.item_definitions import get_item_definition
            item_def = get_item_definition(item_type)
            if item_def:
                route = item_def.get_full_route()
            else:
                # Fallback to old system
                route = self.OPS_i[items[0]]
            
            for a, b in zip(items[:-1], items[1:]):
                # Lex on start times along the class route
                eq_prefix = None
                for j, op in enumerate(route):
                    if op not in self.OPS_i[a] or op not in self.OPS_i[b]:
                        continue  # Skip operations not present for this item
                    if j == 0:
                        # S[a,op] <= S[b,op]
                        self.m += (self.S[a, op] <= self.S[b, op])
                    else:
                        # if all previous equal, enforce S[a,op] <= S[b,op]
                        prev_op = route[j - 1]
                        if prev_op in self.OPS_i[a] and prev_op in self.OPS_i[b]:
                            cond = (self.S[a, prev_op] == self.S[b, prev_op]) if eq_prefix is None else (
                                        eq_prefix & (self.S[a, prev_op] == self.S[b, prev_op]))
                            eq_prefix = cond
                            self.m += ((~cond) | (self.S[a, op] <= self.S[b, op]))

        # -------- (2) Machine activation ordering within identical groups --------
        # For each op, and each identical (type,subtype) group: if lane m2 used, lane m1 (m1<m2) must be used
        for op in self.OPS:
            for group in identical_machine_groups_for_op(op):
                for m1, m2 in zip(group[:-1], group[1:]):
                    u1 = or_all([y.get((i, op, m1), None) for i in self.I])
                    u2 = or_all([y.get((i, op, m2), None) for i in self.I])
                    if u1 is not None and u2 is not None:
                        self.m += (u2 <= u1)

        # -------- (4) Human lex ordering per op --------
        # If two items both pick Human for an op (on any lane), order starts by id
        for op in self.OPS:
            for items_in_class in classes.values():
                items_op = [i for i in sorted(items_in_class) if self.z.get((i, op), 0) == 1]
                if len(items_op) < 2:
                    continue
                # ha = OR_m x_h[i,op,m]
                ha = {
                    i: or_all([self.x_h.get((i, op, m), None) for m in self.M_of[op] if (i, op, m) in self.x_h])
                    for i in items_op
                }
                for a, b in zip(items_op[:-1], items_op[1:]):
                    if ha[a] is None or ha[b] is None:
                        continue
                    both_h = ha[a] & ha[b]
                    self.m += ((~both_h) | (self.S[a, op] <= self.S[b, op]))

    def make_gantt(self, folder=None,html=False):
        """
        Interactive Gantt with custom lane ordering and flow lines.

        Fixes vs previous:
          - Stable lane order across HTML/PNG/SVG (strict categoryarray + label normalization)
          - Bars aligned per resource (one trace per (item, resource), alignmentgroup/offsetgroup=resource)
          - Consistent look between Jupyter and kaleido (fixed width/height, bargap, bar width)
          - Robust flow-lines (stepped connectors; endpoints validated against categories)
          - Safe save path (folder -> schedule.html/.png/.svg)
        """
        import math
        from pathlib import Path
        import pandas as pd
        import plotly.graph_objects as go
        from plotly.colors import qualitative as q
        import plotly.io as pio

        # ---------- naming ----------
        OP_NAME = {
            K1: "K1 (Kitting)", K2: "K2 (Screw prep)",
            B1: "B1 (Assembly core)", B2: "B2 (Screw insertion)",
            P: "P (Packaging)",
        }

        # ---------- helpers ----------
        def machine_label(mid):
            """Return human-readable label for machine id."""
            type_sub = None
            for t, lst in self.resources.items():
                for entry in lst:
                    if isinstance(entry, (list, tuple)) and len(entry) == 2:
                        id_, subtype = entry
                    else:
                        id_, subtype = entry, None
                    if id_ == mid:
                        type_sub = (t, subtype)
                        break
            if not type_sub:
                return f"WS{mid}"
            t, subtype = type_sub
            if t == KIT:
                return f"KIT #{mid}"
            elif t == PACK:
                return f"PACK #{mid}"
            elif t == ASM and subtype == FLASHLIGHT_CLIPPED:
                return f"GRIP #{mid}"
            elif t == ASM and subtype == FLASHLIGHT_SCREWS:
                return f"GRIP & SCREW #{mid}"
            elif t == "human":
                return "Human"
            else:
                return f"WS{mid}"

        def conv_label(m1, m2):
            """Pretty conveyor label."""
            src, dst = machine_label(m1), machine_label(m2)
            return f"{src} -> {dst}"  # use ASCII arrow for cross-renderer safety

        def resource_for_processing(i, op):
            who, m = self._picked_assignment(i, op)
            if who == "human":
                return "Human"
            if m is not None:
                return machine_label(m)
            return "Unassigned"

        def solid_resource_for_processing(i, op):
            """Return the solid lane for a processing op (machine lane if any)."""
            who, m = self._picked_assignment(i, op)
            if m is not None:
                return machine_label(m)
            return "Human"

        def resource_for_transport(i, trans):
            if (i, trans, "human") in self.zT and self.zT[i, trans, "human"].value() == 1:
                return "Human"  # human transport is solid
            
            # Check direct conveyor paths only
            pairs = self._conv_pairs(trans)
            for m1, m2 in pairs:
                key = (i, trans, ("conv", m1, m2))
                if key in self.zT and self.zT[key].value() == 1:
                    return conv_label(m1, m2)
            return None

        # Label normalization to avoid accidental new categories (spaces, unicode)
        def norm_label(s: str) -> str:
            if s is None:
                return s
            s = str(s).strip()
            s = s.replace("→", "->")
            s = " ".join(s.split())
            return s

        # ---------- collect bars ----------
        rows = []

        # processing ops
        for i in self.I:
            for op in self.OPS:
                if not self.z.get((i, op), 0):
                    continue
                Sv, Ev = self.S[i, op].value(), self.E[i, op].value()
                if Sv is None or Ev is None:
                    continue
                S, E = int(Sv), int(Ev)
                res = resource_for_processing(i, op)
                rows.append({
                    "Item": str(i), "Op": OP_NAME.get(op, str(op)),
                    "Task": f"Item {i} – {OP_NAME.get(op, str(op))}",
                    "Start": S, "End": E, "Duration": E - S,
                    "Resource": res,
                    "Pattern": "solid",
                    "TransType": "",
                })
                who, m = self._picked_assignment(i, op)
                if who == "human" and m is not None:
                    rows.append({
                        "Item": str(i), "Op": OP_NAME.get(op, str(op)),
                        "Task": f"Item {i} – {OP_NAME.get(op, str(op))} (machine occupied)",
                        "Start": S, "End": E, "Duration": E - S,
                        "Resource": machine_label(m),
                        "Pattern": "stripe",
                        "TransType": "",
                    })

        # transport ops (solid; 'x' for human, '.' for conveyor)
        all_transitions = self.get_all_transitions()
        for i in self.I:
            for trans in all_transitions:
                if (i, trans) not in self.S_T:
                    continue
                Sv, Ev = self.S_T[i, trans].value(), self.E_T[i, trans].value()
                if Sv is None or Ev is None:
                    continue
                S, E = int(Sv), int(Ev)
                res = resource_for_transport(i, trans)
                if res is None:
                    continue
                pattern = "x" if res == "Human" else "dot"
                rows.append({
                    "Item": str(i), "Op": f"T_{trans}",
                    "Task": f"Item {i} – Transport {trans}",
                    "Start": S, "End": E, "Duration": E - S,
                    "Resource": res,
                    "Pattern": pattern,
                    "TransType": ("KB" if trans == KB else "B12" if trans == B12 else "BP"),
                })

        df = pd.DataFrame(rows)
        if df.empty:
            raise ValueError("No scheduled tasks to plot (empty dataframe).")

        # ---------- lane order (independent of label text) ----------
        def lane_rank(resource: str, ttype: str) -> int:
            # 1 PACK WS
            # 2 CONV BP (ASM -> PACK)
            # 3 GRIP & SCREW WS
            # 4 CONV B12 (ASM Grip -> ASM Grip & Screw)
            # 5 GRIP WS
            # 6 CONV KB (KIT -> ASM)
            # 7 Human
            if resource.startswith("PACK"):
                return 1
            if ttype == "BP":
                return 2
            if resource.startswith("GRIP & SCREW"):
                return 3
            if ttype == "B12":
                return 4
            if resource.startswith("GRIP #"):
                return 5
            if ttype == "KB":
                return 6
            if resource == "Human":
                return 10
            if resource.startswith("KIT"):
                return 7
            return 9

        # Normalize & rank
        df["Resource"] = df["Resource"].map(norm_label)
        df["LaneRank"] = df.apply(lambda r: lane_rank(r["Resource"], r["TransType"]), axis=1)

        # unique resources ordered by (rank, name)
        rank_map = df.groupby("Resource", sort=False)["LaneRank"].min()
        pairs = sorted(rank_map.items(), key=lambda t: (t[1], str(t[0])))
        order = []
        seen = set()
        for res, _ in pairs:
            if res not in seen:
                order.append(res)
                seen.add(res)
        # force Human bottom
        order = [r for r in order if r != "Human"] + (["Human"] if "Human" in order else [])

        # lock categorical dtype
        from pandas.api.types import CategoricalDtype
        lane_dtype = CategoricalDtype(categories=order, ordered=True)
        df["Resource"] = df["Resource"].astype(lane_dtype)

        # sanity: no stray labels
        stray = sorted(set(df["Resource"].astype(str)) - set(order))
        if stray:
            raise ValueError(f"Unknown lane labels not in category order: {stray}")

        # patterns
        pattern_map = {"solid": "", "stripe": "/", "x": "x", "dot": "."}
        df["PatternShape"] = df["Pattern"].map(pattern_map).fillna("")

        palette = q.Plotly
        item_list = sorted(df["Item"].unique(), key=lambda x: int(x) if x.isdigit() else x)
        item_color = {item: palette[idx % len(palette)] for idx, item in enumerate(item_list)}

        # ---------- figure ----------
        fig = go.Figure()

        for item in item_list:
            dfi = df[df["Item"] == item]
            legend_done = False  # <-- reset at the start of each item

            for res in dfi["Resource"].cat.categories:
                dfr = dfi[dfi["Resource"] == res]
                if dfr.empty:
                    continue

                # --- non-stripe
                dfn = dfr[dfr["Pattern"] != "stripe"]
                if not dfn.empty:
                    hovertext = [
                        f"{row['Task']}<br>Start: {row['Start']} End: {row['End']}"
                        for _, row in dfn.iterrows()
                    ]
                    opacities = [
                        0.4 if (row["Resource"] == "Human" and row["Pattern"] == "solid") else 1.0
                        for _, row in dfn.iterrows()
                    ]
                    fig.add_bar(
                        x=dfn["Duration"],
                        y=dfn["Resource"],
                        base=dfn["Start"],
                        orientation="h",
                        name=str(item),
                        hovertext=hovertext,
                        hoverinfo="text",
                        marker=dict(
                            color=item_color[item],
                            opacity=opacities,
                            line=dict(width=3, color="white"),
                            pattern=dict(shape=dfn["PatternShape"].tolist(), size=10, solidity=0.5),
                        ),
                        legendgroup=str(item),
                        alignmentgroup=str(res),
                        offsetgroup=str(res),
                        showlegend=(not legend_done),  # <- use the flag
                        width=0.5,
                    )
                    legend_done = True  # <- after first non-stripe trace

                # --- stripe
                dfs = dfr[dfr["Pattern"] == "stripe"]
                if not dfs.empty:
                    hovertext = [
                        f"{row['Task']}<br>Start: {row['Start']} End: {row['End']}"
                        for _, row in dfs.iterrows()
                    ]
                    fig.add_bar(
                        x=dfs["Duration"],
                        y=dfs["Resource"],
                        base=dfs["Start"],
                        orientation="h",
                        name=str(item),
                        hovertext=hovertext,
                        hoverinfo="text",
                        marker=dict(
                            color=item_color[item],
                            line=dict(width=3, color="white"),
                            pattern=dict(shape=dfs["PatternShape"].tolist(),
                                         fgcolor="white", size=10, solidity=0.7),
                        ),
                        legendgroup=str(item),
                        alignmentgroup=str(res),
                        offsetgroup=str(res),
                        showlegend=False,  # stripes never get a legend entry
                        width=0.5,
                    )

        # ---------- flow lines to solid endpoints ----------
        def add_segment(acc_x, acc_y, x0, y0, x1, y1):
            # only if both endpoints are valid categories
            if (y0 is None) or (y1 is None):
                return
            acc_x += [x0, x1, math.nan]
            acc_y += [y0, y1, None]

        def safe_cat(label: str):
            lbl = norm_label(label)
            return lbl if (lbl in order) else None

        EPS_IN, EPS_OUT = 0.15, 0.15
        for item in item_list:
            color = item_color[item]
            seg_x, seg_y = [], []

            i = int(item)
            k_ops = [op for op in self.OPS_i[i] if op in (K1, K2)]
            b_ops = [op for op in self.OPS_i[i] if op in (B1, B2)]

            # KB: last K → transport → first B
            if k_ops and b_ops and (i, KB) in self.S_T:
                prevK, firstB = k_ops[-1], b_ops[0]
                S_prev, E_prev = self.S[i, prevK].value(), self.E[i, prevK].value()
                S_T, E_T = self.S_T[i, KB].value(), self.E_T[i, KB].value()
                S_next = self.S[i, firstB].value()
                if None not in (S_prev, E_prev, S_T, E_T, S_next):
                    y_prev = safe_cat(solid_resource_for_processing(i, prevK))
                    y_T = safe_cat(resource_for_transport(i, KB))
                    y_next = safe_cat(solid_resource_for_processing(i, firstB))
                    if y_T:
                        add_segment(seg_x, seg_y, int(E_prev) - EPS_OUT, y_prev, int(S_T) + EPS_IN, y_T)
                        add_segment(seg_x, seg_y, int(E_T) - EPS_OUT, y_T, int(S_next) + EPS_IN, y_next)

            # B12: B1 → transport → B2
            if len(b_ops) >= 2 and (i, B12) in self.S_T:
                b_from, b_to = b_ops[0], b_ops[1]
                S_prev, E_prev = self.S[i, b_from].value(), self.E[i, b_from].value()
                S_T, E_T = self.S_T[i, B12].value(), self.E_T[i, B12].value()
                S_next = self.S[i, b_to].value()
                if None not in (S_prev, E_prev, S_T, E_T, S_next):
                    y_prev = safe_cat(solid_resource_for_processing(i, b_from))
                    y_T = safe_cat(resource_for_transport(i, B12))
                    y_next = safe_cat(solid_resource_for_processing(i, b_to))
                    if y_T:
                        add_segment(seg_x, seg_y, int(E_prev) - EPS_OUT, y_prev, int(S_T) + EPS_IN, y_T)
                        add_segment(seg_x, seg_y, int(E_T) - EPS_OUT, y_T, int(S_next) + EPS_IN, y_next)

            # BP: last B → transport → P
            if b_ops and self.z.get((i, P), 0) == 1 and (i, BP) in self.S_T:
                lastB = b_ops[-1]
                S_prev, E_prev = self.S[i, lastB].value(), self.E[i, lastB].value()
                S_T, E_T = self.S_T[i, BP].value(), self.E_T[i, BP].value()
                S_next = self.S[i, P].value()
                if None not in (S_prev, E_prev, S_T, E_T, S_next):
                    y_prev = safe_cat(solid_resource_for_processing(i, lastB))
                    y_T = safe_cat(resource_for_transport(i, BP))
                    y_next = safe_cat(solid_resource_for_processing(i, P))
                    if y_T:
                        add_segment(seg_x, seg_y, int(E_prev) - EPS_OUT, y_prev, int(S_T) + EPS_IN, y_T)
                        add_segment(seg_x, seg_y, int(E_T) - EPS_OUT, y_T, int(S_next) + EPS_IN, y_next)

            if seg_x:
                fig.add_scatter(
                    x=seg_x, y=seg_y,
                    mode="lines",
                    line=dict(color=color, width=2),  # thinner, diagonal
                    name=str(item) + " flow",
                    showlegend=False,
                    hoverinfo="skip",
                    legendgroup=str(item),
                )

        # ---------- layout ----------
        target_w, target_h = 1305, 450  # match your "good" Jupyter export
        fig.update_layout(
            title=f"Job Shop Schedule - Makespan {self.makespan.value()}",
            width=target_w, height=target_h, autosize=False,
            plot_bgcolor="black", paper_bgcolor="black",
            font=dict(color="white", family="DejaVu Sans, Arial, Liberation Sans, sans-serif", size=14),
            title_font=dict(color="white"),
            legend_title=dict(text="Item", font=dict(color="white")),
            barmode="overlay",
            bargap=0.05, bargroupgap=0.00,
            legend=dict(groupclick="togglegroup"),
            margin=dict(l=80, r=40, t=60, b=50),
            xaxis=dict(showgrid=True, gridcolor="lightgray", title="Time", color="white"),
            yaxis=dict(
                showgrid=True, gridcolor="lightgray", title="Resource", color="white",
                categoryorder="array", categoryarray=order,tickfont=dict(size=10),
            ),
        )
        fig.update_xaxes(range=[0, self.makespan.value()])

        # ---------- save/show ----------
        # make kaleido defaults consistent with layout (optional)
        pio.kaleido.scope.default_width = target_w
        pio.kaleido.scope.default_height = target_h
        pio.kaleido.scope.default_scale = 2

        if folder:
            if not html:
                fig.write_image(folder, scale=2)
            if html:
                fig.write_html(folder)
        else:
            fig.show(config={
                "toImageButtonOptions": {
                    "format": "png", "filename": "schedule",
                    "width": target_w, "height": target_h, "scale": 2
                }
            })

    def make_ws_topology(self, title="Workstation Topology", location=None):
        """
        Workstations as connected blocks, grouped by type:
          Kitting | Build | Packing
        - Build shows only subtype (grip / grip & screws).
        - Human is a small block centered above the groups.
        - Conveyor belts are arrows between nodes.
        - Adds spacing between groups for readability.
        """
        import plotly.graph_objects as go

        # --- helpers -------------------------------------------------------------
        def _pair_from_entry(entry):
            return (entry[0], entry[1]) if len(entry) == 2 else (entry[1], entry[2])

        # id -> (type, subtype)
        id2type, id2sub = {}, {}
        for typ, lst in self.resources.items():
            if typ == 'human':
                continue
            for mid, sub in lst:
                id2type[mid] = typ
                id2sub[mid] = sub

        def node_label(mid):
            t = id2type.get(mid, None);
            s = id2sub.get(mid, None)
            if t == KIT:  return f"id:{mid} kitting"
            if t == PACK: return f"id:{mid} pallet"
            if t == ASM:
                if s == FLASHLIGHT_CLIPPED:  return f"id:{mid} grip"
                if s == FLASHLIGHT_SCREWS:   return f"id:{mid} grip & screws"
                return f"{mid}"
            return f"{mid}"

        # --- columns (groups) ----------------------------------------------------
        ws_kit = sorted([mid for mid, t in id2type.items() if t == KIT])
        ws_build = sorted([mid for mid, t in id2type.items() if t == ASM])
        ws_pack = sorted([mid for mid, t in id2type.items() if t == PACK])

        columns = [("Kitting", ws_kit), ("Build", ws_build), ("Packing", ws_pack)]

        # layout params
        box_w, box_h = 3.0, 0.75
        x_gap, y_gap = 2.3, 1.4
        group_gap = 2.5  # <<< extra spacing between groups
        font_color = "#FFFFFF"

        col_bg = ["rgba(66,135,245,0.10)", "rgba(255,159,67,0.10)", "rgba(235,87,87,0.10)"]

        color_map = {
            "kitting": "rgba(66,135,245,1)",
            "grip": "rgba(40,199,111,1)",
            "screws": "rgba(255,159,67,1)",
            "packing": "rgba(235,87,87,1)",
            "neutral": "rgba(160,160,160,1)",
            "human": "rgba(200,200,200,1)",
        }

        max_rows = max(1, *(len(col_ids) for _, col_ids in columns))
        start_y = (max_rows - 1) * y_gap / 2.0

        pos, shapes, annos = {}, [], []

        # columns and nodes
        x_offset = 0
        for c, (title_c, mids) in enumerate(columns):
            x = x_offset

            # background for this group
            shapes.append(dict(
                type="rect", xref="x", yref="y",
                x0=x - box_w / 1.6, x1=x + box_w / 1.6,
                y0=-start_y - 0.9, y1=start_y + 0.9,
                line=dict(width=0), fillcolor=col_bg[c % len(col_bg)]
            ))

            annos.append(dict(
                x=x, y=start_y + 1.0, xref="x", yref="y",
                text=f"<b>{title_c}</b>", showarrow=False,
                font=dict(size=14, color="#FFFFFF"), xanchor="center"
            ))

            for r, mid in enumerate(mids):
                y = start_y - r * y_gap
                pos[mid] = (x, y)

                t = id2type.get(mid, None);
                s = id2sub.get(mid, None)
                if t == KIT:
                    fill = color_map["kitting"]
                elif t == PACK:
                    fill = color_map["packing"]
                elif t == ASM and s == FLASHLIGHT_CLIPPED:
                    fill = color_map["grip"]
                elif t == ASM and s == FLASHLIGHT_SCREWS:
                    fill = color_map["screws"]
                else:
                    fill = color_map["neutral"]

                shapes.append(dict(
                    type="rect", xref="x", yref="y",
                    x0=x - box_w / 2, x1=x + box_w / 2,
                    y0=y - box_h / 2, y1=y + box_h / 2,
                    line=dict(color="white", width=2), fillcolor=fill
                ))
                annos.append(dict(
                    x=x, y=y, xref="x", yref="y",
                    text=f"<b>{node_label(mid)}</b>", showarrow=False,
                    font=dict(size=12, color=font_color),
                    xanchor="center", yanchor="middle"
                ))

            # update x offset for next group
            x_offset += x_gap + group_gap

        # --- Human block ---------------------------------------------------------
        human_ids = list(self.resources.get('human', []))
        if human_ids:
            x_center = (x_offset - group_gap) / 2.0
            y_h = start_y + 2.0
            hw, hh = 1.2, 0.5
            shapes.append(dict(
                type="rect", xref="x", yref="y",
                x0=x_center - hw / 2, x1=x_center + hw / 2,
                y0=y_h - hh / 2, y1=y_h + hh / 2,
                line=dict(color="white", width=2), fillcolor=color_map["human"]
            ))
            label = "human" if len(human_ids) == 1 else f"{len(human_ids)} humans"
            annos.append(dict(
                x=x_center, y=y_h, xref="x", yref="y",
                text=f"<b>{label}</b>", showarrow=False,
                font=dict(size=12, color="#000000"),
                xanchor="center", yanchor="middle"
            ))

        # --- conveyors -----------------------------------------------------------
        for entry in self.connected_via_conveyor:
            m1, m2 = _pair_from_entry(entry)
            if m1 not in pos or m2 not in pos:
                continue
            x1, y1 = pos[m1]
            x2, y2 = pos[m2]
            
            # Calculate all possible connection points
            # Source rectangle edges
            src_top = (x1, y1 + box_h/2)
            src_bottom = (x1, y1 - box_h/2)
            src_left = (x1 - box_w/2, y1)
            src_right = (x1 + box_w/2, y1)
            
            # Destination rectangle edges
            dst_top = (x2, y2 + box_h/2)
            dst_bottom = (x2, y2 - box_h/2)
            dst_left = (x2 - box_w/2, y2)
            dst_right = (x2 + box_w/2, y2)
            
            # Find the closest pair of edges
            connections = [
                (src_top, dst_bottom, "top-bottom"),
                (src_bottom, dst_top, "bottom-top"),
                (src_left, dst_right, "left-right"),
                (src_right, dst_left, "right-left")
            ]
            
            min_dist = float('inf')
            best_connection = None
            
            for (ax, ay), (x, y), name in connections:
                dist = ((x - ax)**2 + (y - ay)**2)**0.5
                if dist < min_dist:
                    min_dist = dist
                    best_connection = (ax, ay, x, y)
            
            if best_connection:
                ax, ay, x, y = best_connection
                annos.append(dict(
                    x=x, y=y, xref="x", yref="y",
                    ax=ax, ay=ay, axref="x", ayref="y",
                    text="", showarrow=True, arrowhead=3, arrowsize=1.3,
                    arrowwidth=2, arrowcolor="#BBBBBB", opacity=0.9
                ))

        # --- figure --------------------------------------------------------------
        x_min = -x_gap
        x_max = x_offset
        y_top_extra = 2.6 if human_ids else 1.4
        y_min = -start_y - 1.4
        y_max = start_y + y_top_extra

        fig = go.Figure()
        fig.update_layout(
            title=title,
            paper_bgcolor="black", plot_bgcolor="black",
            xaxis=dict(visible=False, range=[x_min, x_max]),
            yaxis=dict(visible=False, range=[y_min, y_max], scaleanchor="x", scaleratio=1),
            margin=dict(l=20, r=20, t=60, b=20),
            shapes=shapes, annotations=annos,
        )
        if location is not None:
            fig.write_image(location)
        else:
            fig.show()

















