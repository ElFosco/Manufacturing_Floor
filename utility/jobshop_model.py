from abc import ABC, abstractmethod
from collections import defaultdict
from termios import B2400

import cpmpy as cp
import pandas as pd
import plotly.express as px
from numpy.ma.core import shape
import time

from cpmpy import SolverLookup
from utility.costant import K1, B1, B2, K2, P, KB, BP, B12, KIT, FLASHLIGHT_CLIPPED, ASM, \
    FLASHLIGHT_SCREWS, PACK
from utility.item_definitions import get_item_definition, get_all_item_names


class FJProblem(ABC):
    def __init__(self,ILP_formulation = False,ILP_opt=False,symmetry_breaking=False):
        self.ILP_formulation = ILP_formulation
        self.ILP_opt = ILP_opt
        self.symmetry_breaking = symmetry_breaking
        self.dur_hum = {}
        self.dur_arm = {}
        self.max_id_resource = 0
        self.max_id_item = 0
        self.resources = defaultdict(list)
        self.connected_via_conveyor = []
        self.items_to_build = {}
        self.items_jobs = {}
        self.machine_types = {
            0: "WS_KITTING",
            1: "WS_GRIPPER",
            2: "WS_GRIPPER_SCREW",
            3: "WS_PACKING"
        }


    def generate_id_resource(self):
        self.max_id_resource += 1
        return self.max_id_resource

    def generate_id_item(self):
        self.max_id_item += 1
        return self.max_id_item

    def add_workstation(self,machine):
        id = self.generate_id_resource()
        type = machine['type']
        subtype = machine['subtype']
        self.resources[type].append([id,subtype])
        jobs = machine['jobs']
        for action in jobs:
            self.dur_arm[(action,id)] = jobs[action]
        return id

    def add_human(self):
        id = self.generate_id_resource()
        self.resources['human'].append(id)
        return id

    def add_robot(self):
        id = self.generate_id_resource()
        self.resources['robot'].append(id)
        return id


    def add_items_to_build(self,type,qty):
        for _ in range(qty):
            id_item = self.generate_id_item()
            self.items_to_build[id_item] = type

    def remove_items_from_build(self, type, qty):
        """Remove items of a specific type from the items_to_build dictionary."""
        # Find items of the specified type
        items_to_remove = []
        for item_id, item_type in self.items_to_build.items():
            if item_type == type and len(items_to_remove) < qty:
                items_to_remove.append(item_id)
        
        # Remove the items
        for item_id in items_to_remove:
            del self.items_to_build[item_id]
        
        return len(items_to_remove)  # Return actual number removed

    def set_dur_hum(self,dur_hum):
        self.dur_hum = dur_hum.copy()
        self.t_hum = dur_hum['T']

    def set_dur_robot(self, dur_robot):
        self.t_robot = dur_robot['T']

    def route_for_item(self,item):
        """Get the operation route for an item using generalized item definitions."""
        item_type = self.items_to_build[item]
        item_def = get_item_definition(item_type)
        if item_def:
            return item_def.get_full_route()
        else:
            # Fallback to hardcoded definitions for backward compatibility
            if item_type == FLASHLIGHT_CLIPPED:
                return [K1, B1, P]
            elif item_type == FLASHLIGHT_SCREWS:
                return [K1, K2, B1, B2, P]
        return []


    def define_parameters(self):
        # Define workstation groups by type
        self.WS_KITTING = [res[0] for res in self.resources[KIT]]
        self.WS_GRIP = [res[0] for res in self.resources[ASM] if res[1] == FLASHLIGHT_CLIPPED]
        self.WS_SCREW = [res[0] for res in self.resources[ASM] if res[1] == FLASHLIGHT_SCREWS]
        self.WS_PACKING = [res[0] for res in self.resources[PACK]]

        self.I = list(self.items_to_build.keys())
        
        # Dynamic item type flags (backward compatibility)
        self.I_clipped = {i: int(self.items_to_build[i] == FLASHLIGHT_CLIPPED) for i in self.I}
        self.I_screws = {i: int(self.items_to_build[i] == FLASHLIGHT_SCREWS) for i in self.I}

        # Get all operations that will be used across all items
        all_ops = set()
        for item in self.I:
            ops_i = self.route_for_item(item)
            all_ops.update(ops_i)

        # Define machine assignments for operations dynamically
        self.M_of = {}
        for op in all_ops:
            if op in [K1, K2]:
                self.M_of[op] = self.WS_KITTING
            elif op in [B1, B2]:
                self.M_of[op] = self.WS_GRIP + self.WS_SCREW
            elif op == P:
                self.M_of[op] = self.WS_PACKING
            else:
                # For new operations, try to infer machine type from operation name
                # This can be extended for new operation types
                self.M_of[op] = []

        self.z = {}
        self.OPS_i = {}  # ordered list of ops for each item
        self.PREC_i = {}  # precedence edges for each item

        for item in self.I:
            ops_i = self.route_for_item(item)
            self.OPS_i[item] = ops_i
            # presence flags for all operations that might be used
            for op in all_ops:
                self.z[item, op] = 1 if op in ops_i else 0
            # precedence edges follow the per-item sequence
            self.PREC_i[item] = [(ops_i[k], ops_i[k+1]) for k in range(len(ops_i)-1)]

        self.OPS = sorted(all_ops)
        self.OPS_on_WS = {}
        for op in self.OPS:
            for m in self.M_of[op]:
                self.OPS_on_WS.setdefault(m, set()).add(op)

        # A safe UB on makespan
        self.ub = int(1e3)

    def model_problem(self):
        self.define_parameters()
        self.define_dv()
        self.define_constraints()
        if self.symmetry_breaking:
            self.define_symmetry_breaking()
        self.optimize()

    def solve(self, solver='ortools', params=None,timeout=None):
        start = time.time()

        solver = SolverLookup.get(solver, self.m)
        if params is not None:
            solved = solver.solve(time_limit=timeout,**params)
        else:
            solved = solver.solve(time_limit=timeout)

        end = time.time()
        elapsed = end - start

        if solved:
            print('Solution found with makespan:', self.makespan.value())
        else:
            print('No solution found')

        print(f"Computational time: {elapsed:.3f} seconds")



    def _min_arm_dur(self, op):
        vals = []
        for m in self.M_of[op]:
            if self._arm_allowed(op, m):
                vals.append(self.ARM(op, m))
        return min(vals) if vals else int(1e10)  # 'infinity' if arm impossible


    def _min_proc_dur(self, op):
        """Minimum feasible duration for 'op' across human and eligible arms."""
        return min(self.HUM(op), self._min_arm_dur(op))

    def compute_makespan_upper_bound(self):
        UB = 0
        for i in self.I:
            for op in self.OPS_i[i]:
                UB += self._min_proc_dur(op)
        return int(UB)

    def _arm_allowed(self, op, m):
        """Check if an operation can be performed by an arm on a specific machine."""
        # Get human-required operations for this item type
        # This is a simplified version - in practice, you'd need to know which item this is for
        # For now, we'll use the hardcoded rules but this can be made more dynamic
        
        # K2 always requires human
        if op == K2:
            return False
        
        # B2 cannot be done by arms on GRIP workstations
        if op == B2 and m in self.WS_GRIP:
            return False
            
        # For new operations, you can add dynamic rules here
        # For example, check if the operation is in the human_required_ops list
        
        return True


    def HUM(self,op):
        return int(self.dur_hum[op])

    def ARM(self,op, m):
        return int(self.dur_arm.get((op, m), 0))

    def yH(self,i, op):
        if not self.resources.get('human', []):
            return 0
        return sum(self.x_h[i, op, m] for m in self.M_of[op] if (i, op, m) in self.x_h)  # 0/1

    def dH(self,i, op):
        # Human duration of (i,op) regardless of which machine was chosen
        if not self.resources.get('human', []):
            return 0
        return sum(self.HUM(op) * self.x_h[i, op, m] for m in self.M_of[op] if (i, op, m) in self.x_h)

    def make_gantt(self, folder=None):
        """
        Interactive Gantt of the current solution.
        - Human tasks: bar on 'Human' row (solid fill).
        - When human works on a machine, also draw a striped bar on that machine's row.
        - Machine lane names (no 'stage' concept):
            KIT  ->  "<ID> kitting"
            PACK ->  "<ID> pallet"
            ASM & FLASHLIGHT_CLIPPED  -> "<ID> build grip"
            ASM & FLASHLIGHT_SCREWS   -> "<ID> build grip & screws"
        - Legend shows one entry per Item; clicking an Item hides BOTH solid and striped traces.
        """
        import pandas as pd
        import plotly.express as px

        OP_NAME = {
            K1: "K1 (Kitting)",
            K2: "K2 (Screw prep)",
            B1: "B1 (Assembly core)",
            B2: "B2 (Screw insertion)",
            P: "P (Packaging)",
        }

        # ---- Build machine labels from self.resources[type] = [[id, subtype], ...] ----
        id2label = {}
        for type_key, lst in self.resources.items():
            if type_key == 'human':
                continue
            for mid, subtype in lst:
                if type_key == KIT:
                    id2label[mid] = f"{mid} kitting"
                elif type_key == PACK:
                    id2label[mid] = f"{mid} pallet"
                elif type_key == ASM:
                    if subtype == FLASHLIGHT_CLIPPED:
                        id2label[mid] = f"{mid} build grip"
                    elif subtype == FLASHLIGHT_SCREWS:
                        id2label[mid] = f"{mid} build grip & screws"
                    else:
                        id2label[mid] = f"{mid} asm"
                else:
                    id2label[mid] = f"{mid} ws"

        def machine_label(mid):
            type_sub = None
            for t, lst in self.resources.items():
                for entry in lst:
                    if isinstance(entry, (list, tuple)) and len(entry) == 2:
                        id_, subtype = entry
                    else:  # human stored as int
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
            elif t == "human":  # safeguard in case it's asked for human
                return "Human"
            else:
                return f"WS{mid}"

        rows = []
        for i in self.I:
            for op in self.OPS_i[i]:  # only present ops
                Sv = self.S[i, op].value()
                Ev = self.E[i, op].value()
                if Sv is None or Ev is None:
                    continue
                S, E = int(Sv), int(Ev)

                who, m = self._picked_assignment(i, op)

                # Primary bar (Human or Arm)
                primary_resource = "Human" if who == "human" else (machine_label(m) if m is not None else "Unassigned")
                rows.append({
                    "Item": str(i),
                    "Op": OP_NAME.get(op, str(op)),
                    "Task": f"Item {i} – {OP_NAME.get(op, str(op))}",
                    "Start": S, "End": E, "Duration": E - S,
                    "Resource": primary_resource,
                    "Pattern": "solid",
                    "ShowLegend": True,
                })

                # Duplicate bar on the machine lane if human is occupying that machine
                if who == "human" and m is not None:
                    rows.append({
                        "Item": str(i),
                        "Op": OP_NAME.get(op, str(op)),
                        "Task": f"Item {i} – {OP_NAME.get(op, str(op))} (machine occupied)",
                        "Start": S, "End": E, "Duration": E - S,
                        "Resource": machine_label(m),
                        "Pattern": "stripe",
                        "ShowLegend": False,
                    })

        df = pd.DataFrame(rows)
        if df.empty:
            raise ValueError("No scheduled tasks to plot (empty dataframe).")

        # Ensure numerics
        df["Start"] = pd.to_numeric(df["Start"])
        df["End"] = pd.to_numeric(df["End"])
        df["Duration"] = df["End"] - df["Start"]

        # ---- Resource ordering (by category, then by numeric ID prefix) ----
        def id_from_label(r):
            tok = r.split()[0]
            return int(tok) if tok.isdigit() else 10 ** 9

        resources = df["Resource"].unique().tolist()
        kitting = sorted([r for r in resources if r.endswith("kitting")], key=id_from_label)
        grip = sorted([r for r in resources if r.endswith("build grip") and "screws" not in r], key=id_from_label)
        grip_screws = sorted([r for r in resources if r.endswith("build grip & screws")], key=id_from_label)
        pallet = sorted([r for r in resources if r.endswith("pallet")], key=id_from_label)

        order = ["Human"] + kitting + grip + grip_screws + pallet
        # keep any stragglers (e.g., "Unassigned") at the end
        order += [r for r in resources if r not in order]
        category_order = {"Resource": order}

        # ---- Plot ----
        fig = px.bar(
            df,
            x="Duration", y="Resource",
            color="Item",
            pattern_shape="Pattern",
            orientation="h",
            hover_data=["Task", "Op", "Start", "End"],
            base="Start",
            title="Job Shop Schedule",
            category_orders=category_order,
        )

        # ---- Link solid + stripe traces per Item via legend groups ----
        # Make single-click on legend toggle the whole group (solid + all stripes)
        fig.update_layout(legend=dict(groupclick="togglegroup"))

        for tr in fig.data:
            # px names traces like "ITEM, PATTERN" (e.g., "3, solid" / "3, stripe")
            raw_name = tr.name
            item_name = raw_name.split(",")[0].strip()
            tr.legendgroup = item_name  # group by item
            if ", stripe" in raw_name:
                tr.showlegend = False  # hide duplicate legend entry
            else:
                tr.name = item_name  # clean legend label

        # Style
        fig.update_traces(marker=dict(line=dict(width=3, color="white")))
        fig.update_layout(
            plot_bgcolor="black", paper_bgcolor="black",
            xaxis=dict(showgrid=True, gridcolor="lightgray", title="Time", color="white"),
            yaxis=dict(showgrid=True, gridcolor="lightgray", title="Resource", color="white"),
            font=dict(color="white"),
            title_font=dict(color="white"),
            legend_title=dict(text="Item", font=dict(color="white")),
        )

        # Optional: vertical makespan line
        if hasattr(self, "makespan") and self.makespan.value() is not None:
            ms = int(self.makespan.value())
            fig.add_vline(x=ms, line_dash="dash",
                          annotation_text=f"Makespan={ms}",
                          annotation_position="top right", opacity=0.6)

        if folder is not None:
            fig.write_image(f"{folder}")
        else:
            fig.show()



    def _picked_assignment(self, i, op):
        """
        Return a tuple (who, m) with:
          who in {"human","arm", None}, m = machine id or None.
        """
        if (i, op) not in self.S or self.z.get((i, op), 0) == 0:
            return (None, None)
        # Prefer deterministic order of machines
        for m in self.M_of[op]:
            v = self.x_h.get((i, op, m), None)
            if v is not None and v.value() == 1:
                return ("human", m)
        for m in self.M_of[op]:
            v = self.x_a.get((i, op, m), None)
            if v is not None and v.value() == 1:
                return ("arm", m)
        return (None, None)

    def _resource_label(self, who, m):
        if who is None:
            return "absent"
        if who == "human":
            return "Human"
        return f"WS{m} arm"




class FJNoTransportProblem(FJProblem):
    def __init__(self,ILP_formulation=False,ILP_opt=False,symmetry_breaking=False):
        super().__init__(ILP_formulation=ILP_formulation,ILP_opt=ILP_opt,symmetry_breaking=symmetry_breaking)


    def define_dv(self):
        self.x_h, self.x_a = {}, {}

        for i in self.I:
            for op in self.OPS_i[i]:
                for m in self.M_of[op]:
                    self.x_h[i, op, m] = cp.boolvar(name=f"xH_{i}_{op}_{m}")
                    if self._arm_allowed(op, m):
                        self.x_a[i, op, m] = cp.boolvar(name=f"xA_{i}_{op}_{m}")

        self.S = {(i, op): cp.intvar(0, self.ub, name=f"S_{i}_{op}") for i in self.I for op in self.OPS}
        self.E = {(i, op): cp.intvar(0, self.ub, name=f"E_{i}_{op}") for i in self.I for op in self.OPS}
        self.makespan = cp.intvar(0, self.ub, name="Makespan")

        # Build map machine -> ops it can run
        self.OP_flag = {}
        for i in self.I:
            for op in self.OPS:
                if not self.z[i, op]:
                    continue
                for m in self.M_of[op]:
                    self.OP_flag[i, op, m] = cp.boolvar(name=f"u_{i}_{op}_{m}")  # uses machine m

        self.H_flag = {}
        for i in self.I:
            for op in self.OPS:
                if not self.z[i, op]:
                    continue
                self.H_flag[i, op] = cp.boolvar(name=f"h_{i}_{op}")  # done by human



    def define_constraints(self):

        self.m = cp.Model()

        # Always-present ops
        for i in self.I:
            for op in self.OPS:
                if self.z[i, op] == 0:
                    self.m += (self.S[i, op] == 0)
                    self.m += (self.E[i, op] == 0)

        for i in self.I:
            for op in self.OPS:
                if not self.z[i, op]:
                    continue

                if self.resources.get('human', []):
                    # exactly one assignment overall (human somewhere OR arm somewhere)
                    self.m += (sum(self.x_h.get((i, op, m), 0) for m in self.M_of[op]) +
                               sum(self.x_a.get((i, op, m), 0) for m in self.M_of[op])
                               ) == 1

                    # per machine: OP_flag = x_h + x_a ; at most one on the same m
                    for m in self.M_of[op]:
                        self.m += self.OP_flag[i, op, m] == self.x_h.get((i, op, m), 0) + self.x_a.get((i, op, m), 0)
                        self.m += self.OP_flag[i, op, m] <= 1

                    # human flag = 1 iff human chosen on some machine
                    self.m += self.H_flag[i, op] == sum(self.x_h.get((i, op, m), 0) for m in self.M_of[op])
                else:
                    # Only arms available
                    self.m += sum(self.x_a.get((i, op, m), 0) for m in self.M_of[op]) == 1
                    
                    # per machine: OP_flag = x_a only
                    for m in self.M_of[op]:
                        self.m += self.OP_flag[i, op, m] == self.x_a.get((i, op, m), 0)
                        self.m += self.OP_flag[i, op, m] <= 1

                    # human flag = 0 (no human work)
                    self.m += self.H_flag[i, op] == 0

        # Definition duration: E[i,op] = S[i,op] + p_hum[op]*sum_m x_h + p_arm[op]*sum_m x_a
        for i in self.I:
            for op in self.OPS:
                if self.z[i, op] == 0:
                    continue
                # Human duration only if humans are available
                if self.resources.get('human', []):
                    hum_sum = sum(self.HUM(op) * self.x_h[i, op, m] for m in self.M_of[op] if (i, op, m) in self.x_h)
                else:
                    hum_sum = 0
                # Arm duration
                arm_sum = sum(self.ARM(op, m) * self.x_a[i, op, m] for m in self.M_of[op] if (i, op, m) in self.x_a)
                self.m += self.E[i, op] == self.S[i, op] + hum_sum + arm_sum

        # Precedence constraints
        # Per-item precedence (only for the ops that item actually has)
        for i in self.I:
            for pred, succ in self.PREC_i[i]:
                self.m += self.S[i, succ] >= self.E[i, pred]

        # Machine cumulative (capacity 1)
        for m, ops_here in self.OPS_on_WS.items():
            starts_m, durs_m, ends_m, demands_m = [], [], [], []
            for i in self.I:
                for op in ops_here:
                    if not self.z[i, op]:
                        continue
                    starts_m.append(self.S[i, op])
                    dur_i_m = (
                            self.HUM(op) * self.x_h.get((i, op, m), 0) +
                            self.ARM(op, m) * self.x_a.get((i, op, m), 0)
                    )
                    durs_m.append(dur_i_m)
                    ends_m.append(self.S[i, op] + dur_i_m)
                    demands_m.append(self.OP_flag[i, op, m])
            if starts_m:
                self.m += cp.Cumulative(starts_m, durs_m, ends_m, demands_m, capacity=1)

        # Human pool cumulative (capacity 1)
        starts_h, durs_h, ends_h, demands_h = [], [], [], []
        for i in self.I:
            for op in self.OPS:
                if not self.z[i, op]:
                    continue
                starts_h.append(self.S[i, op])
                durs_h.append(self.HUM(op) * self.H_flag[i, op])
                ends_h.append(self.S[i, op] + self.HUM(op) * self.H_flag[i, op])
                demands_h.append(self.H_flag[i, op])
        if starts_h:
            self.m += cp.Cumulative(starts_h, durs_h, ends_h, demands_h, capacity=1)

    def optimize(self):
        if self.ILP_opt:
            for i in self.I:
                self.m += (self.makespan >= self.E[i, P])
        else:
            # Makespan = max over each item's last op
            last_E = [self.E[i, self.OPS_i[i][-1]] for i in self.I]
            self.m += (self.makespan == cp.max(last_E))
            self.m.minimize(self.makespan)


    def define_symmetry_breaking(self):
        """
        Symmetry breaking (CP-native, no Big-M):
          1) Item lexicographic within item-type class: K1 then B1.
          2) Machine activation ordering within identical machine groups.
          3) Machine lexicographic ordering (prefix dominance) within identical groups.
          4) Human lex ordering: if two items both pick Human for an op, order starts by item id.
        """

        # -------- helpers --------
        def or_all(lits):
            """OR of a list of Bool vars; returns None if empty."""
            lit = None
            for v in lits:
                if v is None:
                    continue
                lit = v if lit is None else (lit | v)
            return lit

        items_sorted = sorted(self.I)

        # id -> workstation type (0: KIT, 1: GRIP, 2: GRIP&SCREW, 3: PACK)
        id2type = {}
        for stage, lst in self.resources.items():
            if isinstance(stage, int):
                for mid, t in lst:
                    id2type[mid] = t

        def identical_machine_groups_for_op(op):
            """Group machines truly identical for op (same type, same arm permission, same arm time)."""
            groups = {}
            for m in self.M_of[op]:
                arm_ok = self._arm_allowed(op, m)
                arm_t = self.ARM(op, m) if arm_ok else None
                key = (id2type.get(m, None), arm_ok, arm_t)
                groups.setdefault(key, []).append(m)
            return [sorted(v) for v in groups.values() if len(v) >= 2]

        # y[i,op,m] = OR(x_h[i,op,m], x_a[i,op,m])  (used below)
        y = {}
        for op in self.OPS:
            for i in self.I:
                if self.z.get((i, op), 0) == 0:
                    continue
                for m in self.M_of[op]:
                    lit = or_all([self.x_h.get((i, op, m), None),
                                  self.x_a.get((i, op, m), None)])
                    if lit is not None:
                        y[(i, op, m)] = lit

        # -------- 1) Item lexicographic within class (K1 then B1) --------
        classes = {}
        for i in self.I:
            classes.setdefault(self.items_to_build[i], []).append(i)
        for items in classes.values():
            items = sorted(items)
            for a, b in zip(items[:-1], items[1:]):
                self.m += (
                        (self.S[a, K1] < self.S[b, K1]) |
                        ((self.S[a, K1] == self.S[b, K1]) & (self.S[a, B1] <= self.S[b, B1]))
                )

        # -------- 2) Machine activation ordering (within identical groups) --------
        for op in self.OPS:
            for group in identical_machine_groups_for_op(op):
                for m1, m2 in zip(group[:-1], group[1:]):
                    u1 = or_all([y.get((i, op, m1), None) for i in self.I])
                    u2 = or_all([y.get((i, op, m2), None) for i in self.I])
                    if u1 is not None and u2 is not None:
                        # If higher-id identical machine is used, lower-id must be used
                        self.m += (u2 <= u1)

        # -------- 3) Machine lex ordering (within identical groups) --------
        for op in self.OPS:
            for group in identical_machine_groups_for_op(op):
                for m1, m2 in zip(group[:-1], group[1:]):
                    s1 = 0;
                    s2 = 0
                    for i in items_sorted:
                        s1 = s1 + y.get((i, op, m1), 0)
                        s2 = s2 + y.get((i, op, m2), 0)
                        # prefix dominance -> lex(column_m1) >=_lex column_m2
                        self.m += (s1 >= s2)

        # -------- 4) Human lex ordering (if both pick Human for op) --------
        for op in self.OPS:
            items_op = [i for i in items_sorted if self.z.get((i, op), 0) == 1]
            if len(items_op) < 2:
                continue
            for a, b in zip(items_op[:-1], items_op[1:]):
                ha_or = or_all([self.x_h.get((a, op, m), None) for m in self.M_of[op] if (a, op, m) in self.x_h])
                hb_or = or_all([self.x_h.get((b, op, m), None) for m in self.M_of[op] if (b, op, m) in self.x_h])
                if ha_or is None or hb_or is None:
                    continue
                both_h = ha_or & hb_or
                self.m += ((~both_h) | (self.S[a, op] <= self.S[b, op]))
















