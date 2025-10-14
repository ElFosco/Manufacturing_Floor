import math
from abc import ABC, abstractmethod
from collections import defaultdict
from termios import B2400

import cpmpy as cp
import pandas as pd
import plotly.express as px
from cpmpy import LexLessEq
from numpy.ma.core import shape
import time

from utils.jobshop_model import FJProblem
from utils.costant import K1, B1, B2, K2, P, KB, BP, B12, KIT, FLASHLIGHT_CLIPPED, ASM, FLASHLIGHT_SCREWS, PACK
from utils.jobshop_model import FJProblem
from utils.item_definitions import get_item_definition, get_stage_transitions, get_internal_transitions


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

            
        return pairs


    def _conv_opts(self, i, trans):
        opts = []
        pairs = self._conv_pairs(trans)
        for m1, m2 in pairs:
            key = (i, trans, ("conv", m1, m2))
            if key in self.zT:
                opts.append(self.zT[key])
        return opts

    def _min_transport_time_for_item(self, trans):
        """
        Minimum time to perform transition `trans` for *some* pair of endpoints:
        considers human, robot, and any direct conveyor link available for this trans.
        """
        candidates = []
        # human available?
        if self.resources.get('human', []):
            candidates.append(self.t_hum)
        # robot available?
        if self.resources.get('robot', []):
            candidates.append(self.t_robot)
        # any conveyor link for this transition type?
        if self._conv_pairs(trans):  # non-empty list means at least one direct link exists
            candidates.append(self.t_conv)

        return min(candidates) if candidates else int(1e9)  # "infinity" if impossible

    def compute_makespan_bounds(self,safety_factor=1.10):
        """
        Safe makespan upper bound by scheduling items sequentially:
        - For each item i, sum:
            * min processing time per operation along its route
            * min transport time for each required transition for that item
        - Sum across all items (no overlap) → feasible serial schedule length.
        """
        # Make sure parameters are defined
        if not hasattr(self, "OPS_i") or not self.OPS_i:
            self.define_parameters()

        UB = 0
        dict_item_lb = {}
        for i in self.I:
            dict_item_lb[i] = 0
            ops = self.OPS_i[i]
            # processing: min feasible duration per op (human vs arm on eligible machines)
            UB += sum(self._min_proc_dur(op) for op in ops)
            dict_item_lb[i] += sum(self._min_proc_dur(op) for op in ops)


            # transports needed for this item's route
            item_type = self.items_to_build[i]
            stage_transitions = get_stage_transitions(item_type)  # e.g., [KB, BP]
            internal_transitions = get_internal_transitions(item_type)  # e.g., [(B1,B2)]

            # detect which stage transitions are actually used by this route
            b_ops = [o for o in ops if o in (B1, B2)]
            has_k_stage = any(o in (K1, K2) for o in ops)
            has_p_stage = (P in ops)

            # Kitting -> Build (KB)
            if KB in stage_transitions and has_k_stage and b_ops:
                UB += self._min_transport_time_for_item(KB)
                dict_item_lb[i] += self._min_transport_time_for_item(KB)

            # Build internal: B1 -> B2 (B12)
            if (B1, B2) in internal_transitions and len(b_ops) >= 2:
                UB += self._min_transport_time_for_item(B12)
                dict_item_lb[i] += self._min_transport_time_for_item(B12)

            # Build -> Packing (BP)
            if BP in stage_transitions and b_ops and has_p_stage:
                UB += self._min_transport_time_for_item(BP)
                dict_item_lb[i] += self._min_transport_time_for_item(BP)

        self.ub = int(math.ceil(UB * safety_factor))
        self.lb = max(dict_item_lb.values())



    def define_dv(self):
        self.x_h, self.x_a = {}, {}
        self.compute_makespan_bounds()
        # processing vars only for present ops
        for i in self.I:
            for op in self.OPS_i[i]:
                for m in self.M_of[op]:
                    self.x_h[i, op, m] = cp.boolvar(name=f"xH_{i}_{op}_{m}")
                    if self._arm_allowed(op, m):
                        self.x_a[i, op, m] = cp.boolvar(name=f"xA_{i}_{op}_{m}")

        # timing only for present ops
        self.S = {(i, op): cp.intvar(0, self.ub, name=f"S_{i}_{op}")
                  for i in self.I for op in self.OPS_i[i]}
        self.E = {(i, op): cp.intvar(0, self.ub, name=f"E_{i}_{op}")
                  for i in self.I for op in self.OPS_i[i]}
        self.makespan = cp.intvar(self.lb, self.ub, name="Makespan")

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
                self.S_T[i, trans] = cp.intvar(0, self.ub, name=f"S_T_{i}_{trans}")
                self.E_T[i, trans] = cp.intvar(0, self.ub, name=f"E_T_{i}_{trans}")
                # Only add human transport option if humans are present
                if self.resources.get('human', []):
                    self.zT[i, trans, "human"] = cp.boolvar(name=f"T_{i}_{trans}_human")

                # Only add robot transport option if robots are present
                if self.resources.get('robot', []):
                    self.zT[i, trans, "robot"] = cp.boolvar(name=f"T_{i}_{trans}_robot")

                # Add variables for direct conveyor paths only
                pairs = self._conv_pairs(trans)
                for m1, m2 in pairs:
                    self.zT[i, trans, ("conv", m1, m2)] = cp.boolvar(name=f"T_{i}_{trans}_conv_{m1}_{m2}")

        # ----- agent sets -----
        H = list(range(len(self.resources.get('human', []))))  # humans indexed 0..H-1
        R = list(range(len(self.resources.get('robot', []))))  # robots indexed 0..R-1

        # Per-agent assignment variables
        self.aH_proc, self.aH_tr = {}, {}
        self.aR_tr = {}

        # Human used indicators per agent
        self.y_h_use = {h: cp.boolvar(name=f"y_h_use_{h}") for h in H}
        # Robot used indicators per agent
        self.y_r_use = {r: cp.boolvar(name=f"y_r_use_{r}") for r in R}

        # Human processing assignment aH_proc[i,op,h] only if op exists and can be human
        for i in self.I:
            for op in self.OPS:
                if not self.z[i, op]:
                    continue
                if not self.resources.get('human', []):
                    continue
                # we already have a boolean H_flag[i,op] meaning "this op is done by human"
                for h in H:
                    self.aH_proc[i, op, h] = cp.boolvar(name=f"aH_proc_{i}_{op}_{h}")

        # Human transport assignment aH_tr[i,trans,h] (only if human transport var exists)
        all_transitions = self.get_all_transitions()
        for i in self.I:
            for trans in all_transitions:
                if (i, trans, "human") in self.zT:
                    for h in H:
                        self.aH_tr[i, trans, h] = cp.boolvar(name=f"aH_tr_{i}_{trans}_{h}")

        # Robot transport assignment aR_tr[i,trans,r] (only if robot transport var exists)
        for i in self.I:
            for trans in all_transitions:
                if (i, trans, "robot") in self.zT:
                    for r in R:
                        self.aR_tr[i, trans, r] = cp.boolvar(name=f"aR_tr_{i}_{trans}_{r}")


        # --- resource usage indicators ---
        self.y_WS = {}  # machine used?
        for m in self.OPS_on_WS.keys():
            self.y_WS[m] = cp.boolvar(name=f"Y_WS_{m}")

        self.y_human = cp.boolvar(name="Y_HUMAN") if self.resources.get('human', []) else 0
        self.y_robot = cp.boolvar(name="Y_ROBOT") if self.resources.get('robot', []) else 0

        # build vars per physical link
        self.y_conv = {}
        for entry in self.connected_via_conveyor:
            m1, m2 = self._pair_from_entry(entry)
            self.y_conv[(m1, m2)] = cp.boolvar(name=f"Y_CONV_{m1}_{m2}")

        ## --- transport usage indicators ---
        self.tr_count = cp.intvar(0, len(self.y_conv), name="TR_COUNT")



    def define_constraints(self):
        self.m = cp.Model()

        # machines: if any op is processed on m, then y_WS[m] = 1
        for m, ops_here in self.OPS_on_WS.items():
            for i in self.I:
                for op in ops_here:
                    if not self.z[i, op]:
                        continue
                    # OP_flag[i,op,m] == 1 when op i is executed on machine m (human or arm)
                    self.m += self.y_WS[m] >= self.OP_flag.get((i, op, m), 0)


        # human used if any human processing or human transport is used
        if self.resources.get('human', []):
            for i in self.I:
                for op in self.OPS:
                    if not self.z[i, op]:
                        continue
                    self.m += self.y_human >= self.H_flag[i, op]
            all_transitions = self.get_all_transitions()
            for i in self.I:
                for trans in all_transitions:
                    key = (i, trans, "human")
                    if key in self.zT:
                        self.m += self.y_human >= self.zT[key]

        # robot used if any robot transport is used
        if self.resources.get('robot', []):
            all_transitions = self.get_all_transitions()
            for i in self.I:
                for trans in all_transitions:
                    key = (i, trans, "robot")
                    if key in self.zT:
                        self.m += self.y_robot >= self.zT[key]

        # conveyor link (m1,m2) used if any transport uses that link
        for (m1, m2), y in self.y_conv.items():
            all_transitions = self.get_all_transitions()
            z_terms = []
            for i in self.I:
                for trans in all_transitions:
                    key = (i, trans, ("conv", m1, m2))
                    z = self.zT.get(key, None)
                    if z is not None:
                        # if any z is 1 => y must be 1
                        self.m += (y >= z)
                        z_terms.append(z)
            if z_terms:
                # y cannot be 1 unless at least one z is 1
                self.m += (y <= sum(z_terms))
            else:
                # no associated zT vars -> y must be 0
                self.m += (y == 0)

        # ----- human processing: exactly the human doing it sums to H_flag -----
        for i in self.I:
            for op in self.OPS:
                if not self.z[i, op]:
                    continue
                if self.resources.get('human', []):
                    sum_assigned = cp.sum(self.aH_proc[i, op, h] for h in range(len(self.resources['human'])))
                    # If op is done by human => assigned to exactly one human; else 0
                    self.m += (sum_assigned == self.H_flag[i, op])

        # ----- human transports: sum over humans equals mode-choice zT(...,"human") -----
        for i in self.I:
            for trans in self.get_all_transitions():
                keyH = (i, trans, "human")
                if keyH in self.zT:
                    sum_assigned = cp.sum(self.aH_tr[i, trans, h] for h in range(len(self.resources['human'])))
                    self.m += (sum_assigned == self.zT[keyH])

        # ----- robot transports: sum over robots equals mode-choice zT(...,"robot") -----
        for i in self.I:
            for trans in self.get_all_transitions():
                keyR = (i, trans, "robot")
                if keyR in self.zT:
                    sum_assigned = cp.sum(self.aR_tr[i, trans, r] for r in range(len(self.resources['robot'])))
                    self.m += (sum_assigned == self.zT[keyR])


        # tie count variable to the sum of all resource indicators
        self.resource_count = (sum(list(self.y_WS.values())))

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
            # Add more mappings here if needed

            for trans in stage_transitions + internal_transition_types:
                # Determine if this transition is needed for this item
                need = 0
                if trans == KB and kops and bops:
                    need = 1
                elif trans == BP and bops and self.z[i, P]:
                    need = 1
                elif trans == B12 and len(bops) >= 2:
                    need = 1

                # Build the list of transport options (human / robot / conv links)
                transport_options = []
                transport_time = 0

                if (i, trans, "human") in self.zT:
                    transport_options.append(self.zT[i, trans, "human"])
                    transport_time += self.zT[i, trans, "human"] * self.t_hum

                if (i, trans, "robot") in self.zT:
                    transport_options.append(self.zT[i, trans, "robot"])
                    transport_time += self.zT[i, trans, "robot"] * self.t_robot

                pairs = self._conv_pairs(trans)
                for m1, m2 in pairs:
                    key = (i, trans, ("conv", m1, m2))
                    if key in self.zT:
                        transport_options.append(self.zT[key])
                        transport_time += self.zT[key] * self.t_conv

                # Force: if transition not needed -> all options 0; if needed -> exactly one 1
                if transport_options:
                    self.m += (cp.sum(transport_options) == need)
                else:
                    # If needed but no options exist, model must be infeasible (as it should be)
                    self.m += (need == 0)

                # Duration binding stays correct: if need=0, all zT are 0 → E_T == S_T
                self.m += (self.E_T[i, trans] == self.S_T[i, trans] + transport_time)


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

        # ----- per-human capacity cumulatives (capacity = 1 each) -----
        for h in range(len(self.resources.get('human', []))):
            starts, durs, ends, demands = [], [], [], []
            # processing
            for i in self.I:
                for op in self.OPS:
                    if not self.z[i, op]:
                        continue
                    dH = self.HUM(op)
                    starts.append(self.S[i, op])
                    durs.append(dH * self.aH_proc[i, op, h])
                    ends.append(self.S[i, op] + dH * self.aH_proc[i, op, h])
                    demands.append(self.aH_proc[i, op, h])
            # transports
            for i in self.I:
                for trans in self.get_all_transitions():
                    keyH = (i, trans, "human")
                    if keyH in self.zT:
                        starts.append(self.S_T[i, trans])
                        durs.append(self.t_hum * self.aH_tr[i, trans, h])
                        ends.append(self.S_T[i, trans] + self.t_hum * self.aH_tr[i, trans, h])
                        demands.append(self.aH_tr[i, trans, h])

            if starts:  # avoid empty Cumulative
                self.m += cp.Cumulative(starts, durs, ends, demands, capacity=1)

        # ----- per-robot capacity cumulatives (capacity = 1 each) -----
        for r in range(len(self.resources.get('robot', []))):
            starts, durs, ends, demands = [], [], [], []
            for i in self.I:
                for trans in self.get_all_transitions():
                    keyR = (i, trans, "robot")
                    if keyR in self.zT:
                        starts.append(self.S_T[i, trans])
                        durs.append(self.t_robot * self.aR_tr[i, trans, r])
                        ends.append(self.S_T[i, trans] + self.t_robot * self.aR_tr[i, trans, r])
                        demands.append(self.aR_tr[i, trans, r])
            if starts:
                self.m += cp.Cumulative(starts, durs, ends, demands, capacity=1)

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

        # a task assigned to agent implies the agent is "used"
        for h in range(len(self.resources.get('human', []))):
            lits = []
            for i in self.I:
                for op in self.OPS:
                    if not self.z[i, op]: continue
                    lits.append(self.aH_proc[i, op, h])
                for trans in self.get_all_transitions():
                    keyH = (i, trans, "human")
                    if keyH in self.zT:
                        lits.append(self.aH_tr[i, trans, h])
            if lits:
                self.m += (self.y_h_use[h] == (cp.sum(lits) >= 1))
            else:
                self.m += (self.y_h_use[h] == 0)

        for r in range(len(self.resources.get('robot', []))):
            lits = []
            for i in self.I:
                for trans in self.get_all_transitions():
                    keyR = (i, trans, "robot")
                    if keyR in self.zT:
                        lits.append(self.aR_tr[i, trans, r])
            if lits:
                self.m += (self.y_r_use[r] == (cp.sum(lits) >= 1))
            else:
                self.m += (self.y_r_use[r] == 0)

        self.m += (self.tr_count == cp.sum(list(self.y_conv.values())))

        ## HELPERS
        # ---- Force B12 off for items of type 0 ----
        for i in self.I:
            if self.items_to_build[i] == FLASHLIGHT_CLIPPED:
                # kill mode-choice vars (if they exist)
                keyH = (i, B12, "human")
                keyR = (i, B12, "robot")
                if keyH in self.zT: self.m += (self.zT[keyH] == 0)
                if keyR in self.zT: self.m += (self.zT[keyR] == 0)

                # kill per-agent assignments (if they exist)
                for h in range(len(self.resources.get('human', []))):
                    if (i, B12, h) in [(k[0], k[1], k[2]) for k in self.aH_tr.keys()]:
                        self.m += (self.aH_tr[i, B12, h] == 0)
                for r in range(len(self.resources.get('robot', []))):
                    if (i, B12, r) in [(k[0], k[1], k[2]) for k in self.aR_tr.keys()]:
                        self.m += (self.aR_tr[i, B12, r] == 0)

                # kill conveyor choices for B12 (if any were created)
                for (m1, m2) in self._conv_pairs(B12):
                    keyC = (i, B12, ("conv", m1, m2))
                    if keyC in self.zT:
                        self.m += (self.zT[keyC] == 0)

    def optimize(self,objective_type=0):
        self.variables = {}
        self.humans_used = sum(list(self.y_h_use.values()))
        self.robot_used = sum(list(self.y_r_use.values()))
        if objective_type==0:
            if self.ILP_opt:
                for i in self.I:
                    self.m += (self.makespan >= self.E[i, self.OPS_i[i][-1]])
                self.m.minimize(self.makespan)
            else:
                last_E = [self.E[i, self.OPS_i[i][-1]] for i in self.I]
                self.m += (self.makespan == cp.max(last_E))
                self.m.minimize(self.makespan)
            self.variables = {'makespan': self.makespan}
            self.default_values = [1]
        elif objective_type==1:
            last_E = [self.E[i, self.OPS_i[i][-1]] for i in self.I]
            self.m += (self.makespan == cp.max(last_E))
            self.m.minimize(1e3 * self.makespan + self.resource_count)
            self.variables = {'makespan': self.makespan,'resources':self.resource_count}
            self.default_values = [int(1e3), 1]
        elif objective_type==4:
            last_E = [self.E[i, self.OPS_i[i][-1]] for i in self.I]
            self.m += (self.makespan == cp.max(last_E))
            self.m.minimize(int(1e8) * self.makespan + int(1e6)*self.resource_count + int(1e4)*self.tr_count +
                            int(1e2)*self.humans_used  + self.robot_used)
            self.variables = {'makespan': self.makespan, 'resources': self.resource_count, 'transport':self.tr_count,
                              'humans_used':self.humans_used,'robot_used':self.robot_used}
            self.default_values = [int(1e6),int(1e4),int(1e2),1]
        self.objectives_names = list(self.variables.keys())



    def define_symmetry_breaking(self):
        # """
        # Symmetry breaking (CP-native):
        #   1) Lex order between items of the same class (by their route's start times).
        #   2) Human/Robot/Machine lex ordering: if for two items (of the same type) the operation are done
        #   by same Human/Robot/Machine order by item id.
        #   3) Per-conveyor ordering
        #   4) Lex humans/robots


        #   1) Lex order between items of the same class (by their route's start times).
        items_by_type = defaultdict(list)
        for i in self.I:
            items_by_type[self.items_to_build[i]].append(i)

        for t, items in items_by_type.items():
            if len(items) <= 1:
                continue
            items_sorted = sorted(items)
            for i, j in zip(items_sorted[:-1], items_sorted[1:]):
                ops = self.OPS_i[i]  # same route for same type
                lhs = [self.S[i, op] for op in ops]
                rhs = [self.S[j, op] for op in ops]
                self.m += LexLessEq(lhs, rhs)  # lexicographic ≤


        #   2) Human/Robot/Machine lex ordering: if for two items (of the same type) the operation are done
        #   by same Human/Robot/Machine order by item id.
        all_transitions = self.get_all_transitions()
        for t, items in items_by_type.items():
            if len(items) <= 1:
                continue
            items_sorted = sorted(items)

            for i, j in zip(items_sorted[:-1], items_sorted[1:]):
                # --- (1) Processing Human/Arm ordering ---
                for op in self.OPS_i[i]:
                    # Human tie-break
                    self.m += (self.H_flag[i, op] & self.H_flag[j, op]).implies(
                        self.S[i, op] <= self.S[j, op]
                    )
                    # iterate only machines that can process this op
                    for m in self.M_of[op]:
                        ki = (i, op, m)
                        kj = (j, op, m)
                        # Arm on the SAME machine m
                        if ki in self.x_a and kj in self.x_a:
                            self.m += (self.x_a[ki] & self.x_a[kj]).implies(
                                self.S[i, op] <= self.S[j, op]
                            )
                        # Human on the SAME machine m
                        if ki in self.x_h and kj in self.x_h:
                            self.m += (self.x_h[ki] & self.x_h[kj]).implies(
                                self.S[i, op] <= self.S[j, op]
                            )
                # --- (2) Transport Human/Robot ordering ---
                for trans in all_transitions:
                    if (i, trans, "human") in self.zT and (j, trans, "human") in self.zT:
                        self.m += (self.zT[i, trans, "human"] & self.zT[j, trans, "human"]).implies(
                            self.S_T[i, trans] <= self.S_T[j, trans]
                        )

                    if (i, trans, "robot") in self.zT and (j, trans, "robot") in self.zT:
                        self.m += (self.zT[i, trans, "robot"] & self.zT[j, trans, "robot"]).implies(
                            self.S_T[i, trans] <= self.S_T[j, trans]
                        )

        #   3) Per-conveyor ordering
        for t, items in items_by_type.items():
            if len(items) <= 1:
                continue
            items_sorted = sorted(items)
            for i, j in zip(items_sorted[:-1], items_sorted[1:]):
                for trans in all_transitions:
                    # iterate only links that exist for this transition type
                    for (m1, m2) in self._conv_pairs(trans):
                        key_i = (i, trans, ("conv", m1, m2))
                        key_j = (j, trans, ("conv", m1, m2))
                        if key_i in self.zT and key_j in self.zT:
                            self.m += (self.zT[key_i] & self.zT[key_j]).implies(
                                self.S_T[i, trans] <= self.S_T[j, trans]
                            )
        #   4) Symmetry breaking across humans (use lower indices first) =====
        H = len(self.resources.get('human', []))
        if H > 1 and hasattr(self, "y_h_use"):
            for h in range(H - 1):
                # y_h_use[h] >= y_h_use[h+1]
                # => if Human #(h+1) is used, Human #h must be used too
                self.m += (self.y_h_use[h] >= self.y_h_use[h + 1])

        #   4) Symmetry breaking across robots (use lower indices first) =====
        R = len(self.resources.get('robot', []))
        if R > 1 and hasattr(self, "y_r_use"):
            for r in range(R - 1):
                # y_r_use[r] >= y_r_use[r+1]
                # => if Robot #(r+1) is used, Robot #r must be used too
                self.m += (self.y_r_use[r] >= self.y_r_use[r + 1])



    def make_ws_topology(self, title="Workstation Topology", location=None):
        """
        Workstations as connected blocks, grouped by type:
          Kitting | Build | Packing
        - Build shows only subtype (grip / grip & screws).
        - Human is a small block centered above the groups.
        - Conveyor belts are arrows between nodes.
        - Adds spacing between groups for readability.
        - Centers the content and removes extra whitespace at the image borders.
        - Scales boxes and fonts via S.
        """
        import plotly.graph_objects as go

        # ===== Global scale so you can grow/shrink everything together =====
        S = 1.25  # try 1.0, 1.25, 1.5, 2.0 ...

        # --- helpers -------------------------------------------------------------
        def _pair_from_entry(entry):
            return (entry[0], entry[1]) if len(entry) == 2 else (entry[1], entry[2])

        # id -> (type, subtype)
        id2type, id2sub = {}, {}
        for typ, lst in self.resources.items():
            if typ == 'human':
                continue
            if typ == 'robot':
                continue
            for mid, sub in lst:
                id2type[mid] = typ
                id2sub[mid] = sub

        def node_label(mid):
            t = id2type.get(mid, None)
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

        # --- layout params (scaled) ---------------------------------------------
        box_w, box_h = 3.0 * S, 0.75 * S
        x_gap, y_gap = 2.3 * S, 1.4 * S
        group_gap = 3.0 * S
        font_color = "#FFFFFF"

        # Typography (scaled). Adjust these if you want.
        LABEL_FONT_SIZE = 40  # inside workstation boxes
        GROUP_TITLE_FONT_SIZE = int(30 * S)  # "Kitting / Build / Packing"
        HR_FONT_SIZE = 40  # "human / robot"
        TITLE_FONT_SIZE = int(22 * S)

        col_bg = ["rgba(66,135,245,0.10)", "rgba(255,159,67,0.10)", "rgba(235,87,87,0.10)"]

        color_map = {
            "kitting": "rgba(66,135,245,1)",
            "grip": "rgba(40,199,111,1)",
            "screws": "rgba(255,159,67,1)",
            "packing": "rgba(235,87,87,1)",
            "neutral": "rgba(160,160,160,1)",
            "operator": "rgba(200,200,200,1)",
            "robot": "rgba(150,150,150,1)",
        }

        max_rows = max(1, *(len(col_ids) for _, col_ids in columns))
        start_y = (max_rows - 1) * y_gap / 2.0

        pos, shapes, annos = {}, [], []

        # --- columns and nodes ---------------------------------------------------
        x_offset = 0.0
        first_x, last_x = None, None

        for c, (title_c, mids) in enumerate(columns):
            x = x_offset
            if first_x is None:
                first_x = x
            last_x = x

            # background for this group
            shapes.append(dict(
                type="rect", xref="x", yref="y",
                x0=x - box_w / 1.2, x1=x + box_w / 1.2,
                y0=-start_y - 0.9 * S, y1=start_y + 0.9 * S,
                line=dict(width=0), fillcolor=col_bg[c % len(col_bg)]
            ))

            # group title
            annos.append(dict(
                x=x, y=start_y + 1.0 * S, xref="x", yref="y",
                text=f"<b>{title_c}</b>", showarrow=False,
                font=dict(size=GROUP_TITLE_FONT_SIZE, color="#FFFFFF"),
                xanchor="center"
            ))

            # workstations
            for r, mid in enumerate(mids):
                y = start_y - r * y_gap
                pos[mid] = (x, y)

                t = id2type.get(mid, None)
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
                    font=dict(size=LABEL_FONT_SIZE, color=font_color),
                    xanchor="center", yanchor="middle"
                ))

            # update x offset for next group
            x_offset += x_gap + group_gap

        # --- Human and Robot blocks ---------------------------------------------
        human_ids = list(self.resources.get('human', []))
        robot_ids = list(self.resources.get('robot', []))

        if human_ids or robot_ids:
            # Make their boxes bigger than before, scaled with S
            hw, hh = 2.0 * S, 1.0 * S  # width, height of human/robot boxes
            y_h = start_y + 2.5 * S  # push up a bit higher

            # Center over the build section (middle column)
            build_x_center = x_gap + group_gap

            if human_ids and robot_ids:
                x_robot = build_x_center - 1.6 * S
                x_human = build_x_center + 1.6 * S
            elif human_ids:
                x_human = build_x_center
                x_robot = None
            else:
                x_robot = build_x_center
                x_human = None

            # Human block
            if human_ids and x_human is not None:
                shapes.append(dict(
                    type="rect", xref="x", yref="y",
                    x0=x_human - hw / 2, x1=x_human + hw / 2,
                    y0=y_h - hh / 2, y1=y_h + hh / 2,
                    line=dict(color="white", width=2),
                    fillcolor="#C64191"
                ))
                label = "human" if len(human_ids) == 1 else f"{len(human_ids)} operators"
                annos.append(dict(
                    x=x_human, y=y_h, xref="x", yref="y",
                    text=f"<b>{label}</b>", showarrow=False,
                    font=dict(size=HR_FONT_SIZE, color="#000000"),
                    xanchor="center", yanchor="middle"
                ))

            # Robot block
            if robot_ids and x_robot is not None:
                shapes.append(dict(
                    type="rect", xref="x", yref="y",
                    x0=x_robot - hw / 2, x1=x_robot + hw / 2,
                    y0=y_h - hh / 2, y1=y_h + hh / 2,
                    line=dict(color="white", width=2),
                    fillcolor="#DCABDF"  # orange
                ))
                label = "robot" if len(robot_ids) == 1 else f"{len(robot_ids)} robots"
                annos.append(dict(
                    x=x_robot, y=y_h, xref="x", yref="y",
                    text=f"<b>{label}</b>", showarrow=False,
                    font=dict(size=HR_FONT_SIZE, color="#000000"),
                    xanchor="center", yanchor="middle"
                ))

        # --- conveyors (arrows) ---------------------------------------------------
        for entry in self.connected_via_conveyor:
            m1, m2 = _pair_from_entry(entry)
            if m1 not in pos or m2 not in pos:
                continue
            x1, y1 = pos[m1]
            x2, y2 = pos[m2]

            # Source rectangle edges
            src_top = (x1, y1 + box_h / 2)
            src_bottom = (x1, y1 - box_h / 2)
            src_left = (x1 - box_w / 2, y1)
            src_right = (x1 + box_w / 2, y1)

            # Destination rectangle edges
            dst_top = (x2, y2 + box_h / 2)
            dst_bottom = (x2, y2 - box_h / 2)
            dst_left = (x2 - box_w / 2, y2)
            dst_right = (x2 + box_w / 2, y2)

            connections = [
                (src_top, dst_bottom, "top-bottom"),
                (src_bottom, dst_top, "bottom-top"),
                (src_left, dst_right, "left-right"),
                (src_right, dst_left, "right-left")
            ]

            min_dist = float('inf')
            best_connection = None

            for (ax, ay), (tx, ty), _ in connections:
                dist = ((tx - ax) ** 2 + (ty - ay) ** 2) ** 0.5
                if dist < min_dist:
                    min_dist = dist
                    best_connection = (ax, ay, tx, ty)

            if best_connection:
                ax, ay, tx, ty = best_connection
                annos.append(dict(
                    x=tx, y=ty, xref="x", yref="y",
                    ax=ax, ay=ay, axref="x", ayref="y",
                    text="", showarrow=True, arrowhead=3,
                    arrowsize=2.2,  # bigger arrowhead (was 1.3)
                    arrowwidth=4,  # thicker arrow line (was 2)
                    arrowcolor="#DDDDDD",  # slightly brighter for visibility
                    opacity=1.0
                ))

        # --- figure (centered, no outer whitespace) -------------------------------
        # last_x is the x of the last column's center. x_offset has already been
        # incremented once past the last column, so do NOT use x_offset directly.
        if first_x is None:
            first_x = 0.0
        if last_x is None:
            last_x = 0.0

        content_left = first_x - box_w / 1.2
        content_right = last_x + box_w / 1.2

        if human_ids or robot_ids:
            # include the human/robot block in the top bound
            y_top_bg = y_h + (0.5 * S)
        else:
            y_top_bg = start_y + 0.9 * S
        y_bot_bg = -start_y - 0.9 * S

        # Tiny padding so strokes don't get clipped
        pad = 0.05 * S
        x_min = content_left - pad
        x_max = content_right + pad
        y_min = y_bot_bg - pad
        y_max = y_top_bg + pad

        fig = go.Figure()
        fig.update_layout(
            title=dict(text=title, x=0.5, xanchor='center', font=dict(size=TITLE_FONT_SIZE),
                       pad=dict(t=0, b=0, l=0, r=0)),
            paper_bgcolor="black", plot_bgcolor="black",
            xaxis=dict(visible=False, range=[x_min, x_max]),
            yaxis=dict(visible=False, range=[y_min, y_max], scaleanchor="x", scaleratio=1),
            margin=dict(l=0, r=0, t=0, b=0),
            shapes=shapes,
            annotations=annos,
            width=int(1600 * S),  # bigger image as S grows
            height=int(900 * S),
            autosize=False,
        )

        # --- save or show ---------------------------------------------------------
        if location is not None:
            # Faster + crisp: SVG (change extension to .svg to use this)
            # Faster PNG: keep scale=1 (scale multiplies pixel count)
            fig.write_image(location, format="png", width=int(1600 * S), height=int(900 * S), scale=1)
        else:
            fig.update_layout(
                width=1400,  # pick something reasonable
                height=600
            )
            fig.show()

    def _safe_val(self, v):
        """
        Return 0/1 (or numeric) from a cp variable in a solver-agnostic way.
        Falls back to 0 if no value is available.
        """
        try:
            return int(v.value())
        except Exception:
            try:
                import cpmpy as _cp
                return int(_cp.value(v))
            except Exception:
                return 0

    def compute_used_counts(self):
        """
        After solving, return:
          - used_ws: set of workstation IDs that are used
          - used_conv: set of conveyor links (m1,m2) that are used
          - n_humans_used: integer count of used human agents
          - n_robots_used: integer count of used robot agents
        """
        used_ws = {m for m, y in getattr(self, "y_WS", {}).items() if self._safe_val(y) == 1}
        used_conv = {pair for pair, y in getattr(self, "y_conv", {}).items() if self._safe_val(y) == 1}
        n_humans_used = sum(self._safe_val(v) for v in getattr(self, "y_h_use", {}).values())
        n_robots_used = sum(self._safe_val(v) for v in getattr(self, "y_r_use", {}).values())
        return used_ws, used_conv, n_humans_used, n_robots_used

    def make_ws_updated(self, title="Updated Workstation Topology", location=None):
        """
        Same visual style as make_ws_topology:
          - SAME colors, fonts, spacing, and backgrounds.
          - Only show resources that were actually used by the solved plan.
          - Show counts like “2 operators” or “1 robot”.
        """
        import plotly.graph_objects as go

        used_ws, used_conv, n_h_used, n_r_used = self.compute_used_counts()

        # ===== Global scale =====
        S = 1.25  # same as your original

        # --- helpers ---------------------------------------------------------------
        def _pair_from_entry(entry):
            return (entry[0], entry[1]) if len(entry) == 2 else (entry[1], entry[2])

        id2type, id2sub = {}, {}
        for typ, lst in self.resources.items():
            if typ in ("human", "robot"):
                continue
            for mid, sub in lst:
                id2type[mid] = typ
                id2sub[mid] = sub

        def node_label(mid):
            t = id2type.get(mid, None)
            s = id2sub.get(mid, None)
            if t == KIT:  return f"id:{mid} kitting"
            if t == PACK: return f"id:{mid} pallet"
            if t == ASM:
                if s == FLASHLIGHT_CLIPPED:  return f"id:{mid} grip"
                if s == FLASHLIGHT_SCREWS:   return f"id:{mid} grip & screws"
                return f"{mid}"
            return f"{mid}"

        # --- workstation groups ----------------------------------------------------
        ws_kit = sorted([mid for mid, t in id2type.items() if t == KIT and mid in used_ws])
        ws_build = sorted([mid for mid, t in id2type.items() if t == ASM and mid in used_ws])
        ws_pack = sorted([mid for mid, t in id2type.items() if t == PACK and mid in used_ws])
        columns = [("Kitting", ws_kit), ("Build", ws_build), ("Packing", ws_pack)]

        # --- layout parameters (same as original) ----------------------------------
        box_w, box_h = 3.0 * S, 0.75 * S
        x_gap, y_gap = 2.3 * S, 1.4 * S
        group_gap = 3.0 * S
        font_color = "#FFFFFF"

        LABEL_FONT_SIZE = 40
        GROUP_TITLE_FONT_SIZE = int(30 * S)
        HR_FONT_SIZE = 40
        TITLE_FONT_SIZE = int(22 * S)

        col_bg = ["rgba(66,135,245,0.10)", "rgba(255,159,67,0.10)", "rgba(235,87,87,0.10)"]

        color_map = {
            "kitting": "rgba(66,135,245,1)",
            "grip": "rgba(40,199,111,1)",
            "screws": "rgba(255,159,67,1)",
            "packing": "rgba(235,87,87,1)",
            "neutral": "rgba(160,160,160,1)",
        }

        max_rows = max(1, *(len(col_ids) if len(col_ids) > 0 else 1 for _, col_ids in columns))
        start_y = (max_rows - 1) * y_gap / 2.0

        pos, shapes, annos = {}, [], []
        x_offset = 0.0
        first_x, last_x = None, None

        # --- Draw columns and used workstations ------------------------------------
        for c, (title_c, mids) in enumerate(columns):
            x = x_offset
            if first_x is None:
                first_x = x
            last_x = x

            # background for column
            shapes.append(dict(
                type="rect", xref="x", yref="y",
                x0=x - box_w / 1.2, x1=x + box_w / 1.2,
                y0=-start_y - 0.9 * S, y1=start_y + 0.9 * S,
                line=dict(width=0), fillcolor=col_bg[c % len(col_bg)]
            ))

            # group title
            annos.append(dict(
                x=x, y=start_y + 1.0 * S, xref="x", yref="y",
                text=f"<b>{title_c}</b>", showarrow=False,
                font=dict(size=GROUP_TITLE_FONT_SIZE, color="#FFFFFF"),
                xanchor="center"
            ))

            # used workstations
            for r, mid in enumerate(mids):
                y = start_y - r * y_gap
                pos[mid] = (x, y)
                t = id2type.get(mid, None)
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
                    font=dict(size=LABEL_FONT_SIZE, color=font_color),
                    xanchor="center", yanchor="middle"
                ))

            x_offset += x_gap + group_gap

        # --- Humans and Robots (same style, but show “2 robots”) --------------------
        human_ids = list(self.resources.get('human', []))
        robot_ids = list(self.resources.get('robot', []))
        show_h = (n_h_used > 0 and len(human_ids) > 0)
        show_r = (n_r_used > 0 and len(robot_ids) > 0)

        if show_h or show_r:
            hw, hh = 2.0 * S, 1.0 * S
            y_h = start_y + 2.5 * S
            build_x_center = x_gap + group_gap

            if show_h and show_r:
                x_robot = build_x_center - 1.6 * S
                x_human = build_x_center + 1.6 * S
            elif show_h:
                x_human = build_x_center
                x_robot = None
            else:
                x_robot = build_x_center
                x_human = None

            if show_h:
                shapes.append(dict(
                    type="rect", xref="x", yref="y",
                    x0=x_human - hw / 2, x1=x_human + hw / 2,
                    y0=y_h - hh / 2, y1=y_h + hh / 2,
                    line=dict(color="white", width=2),
                    fillcolor="#C64191"
                ))
                label = f"<b>{n_h_used} operator{'s' if n_h_used != 1 else ''}</b>"
                annos.append(dict(
                    x=x_human, y=y_h, xref="x", yref="y",
                    text=label, showarrow=False,
                    font=dict(size=HR_FONT_SIZE, color="#000000"),
                    xanchor="center", yanchor="middle"
                ))

            if show_r:
                shapes.append(dict(
                    type="rect", xref="x", yref="y",
                    x0=x_robot - hw / 2, x1=x_robot + hw / 2,
                    y0=y_h - hh / 2, y1=y_h + hh / 2,
                    line=dict(color="white", width=2),
                    fillcolor="#DCABDF"
                ))
                label = f"<b>{n_r_used} robot{'s' if n_r_used != 1 else ''}</b>"
                annos.append(dict(
                    x=x_robot, y=y_h, xref="x", yref="y",
                    text=label, showarrow=False,
                    font=dict(size=HR_FONT_SIZE, color="#000000"),
                    xanchor="center", yanchor="middle"
                ))

            # --- conveyors: connect edge-to-edge (not centers) -------------------------
            def _edge_anchors(x, y):
                # centers of each rectangle edge
                return [
                    (x, y + box_h / 2),  # top
                    (x, y - box_h / 2),  # bottom
                    (x - box_w / 2, y),  # left
                    (x + box_w / 2, y),  # right
                ]

            def _arrow(ax, ay, tx, ty):
                annos.append(dict(
                    x=tx, y=ty, xref="x", yref="y",
                    ax=ax, ay=ay, axref="x", ayref="y",
                    text="", showarrow=True, arrowhead=3,
                    arrowsize=2.2, arrowwidth=4, arrowcolor="#DDDDDD", opacity=1.0
                ))

            for entry in self.connected_via_conveyor:
                m1, m2 = _pair_from_entry(entry)
                if (m1, m2) not in used_conv:
                    continue
                if m1 not in pos or m2 not in pos:
                    continue

                x1, y1 = pos[m1]
                x2, y2 = pos[m2]

                # pick the shortest edge-to-edge connection
                best = None
                best_d2 = float("inf")
                for ax, ay in _edge_anchors(x1, y1):
                    for tx, ty in _edge_anchors(x2, y2):
                        d2 = (tx - ax) ** 2 + (ty - ay) ** 2
                        if d2 < best_d2:
                            best_d2 = d2
                            best = (ax, ay, tx, ty)

                if best:
                    ax, ay, tx, ty = best
                    _arrow(ax, ay, tx, ty)

        # --- figure bounds and rendering -------------------------------------------
        if first_x is None: first_x = 0.0
        if last_x is None: last_x = 0.0

        content_left = first_x - box_w / 1.2
        content_right = last_x + box_w / 1.2
        y_top_bg = (start_y + 2.5 * S) + (0.5 * S)
        y_bot_bg = -start_y - 0.9 * S
        pad = 0.05 * S

        fig = go.Figure()
        fig.update_layout(
            title=dict(text=title, x=0.5, xanchor='center',
                       font=dict(size=TITLE_FONT_SIZE), pad=dict(t=0, b=0, l=0, r=0)),
            paper_bgcolor="black", plot_bgcolor="black",
            xaxis=dict(visible=False, range=[content_left - pad, content_right + pad]),
            yaxis=dict(visible=False, range=[y_bot_bg - pad, y_top_bg + pad], scaleanchor="x", scaleratio=1),
            margin=dict(l=0, r=0, t=0, b=0),
            shapes=shapes, annotations=annos,
            width=int(1600 * S), height=int(900 * S), autosize=False,
        )

        if location is not None:
            fig.write_image(location, format="png", width=int(1600 * S),
                            height=int(900 * S), scale=1)
        else:
            fig.show()

















