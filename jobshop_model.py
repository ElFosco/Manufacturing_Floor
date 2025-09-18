from abc import ABC, abstractmethod
from collections import defaultdict
from termios import B2400

import cpmpy as cp
import pandas as pd
import plotly.express as px
from numpy.ma.core import shape
import time
from utility.costant import map_type_stage, K1, B1, B2, K2, P, KB, BP, B12, KIT, FLASHLIGHT_CLIPPED, ASM, \
    FLASHLIGHT_SCREWS, PACK


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


    def add_items_to_build(self,type,qty):
        for _ in range(qty):
            id_item = self.generate_id_item()
            self.items_to_build[id_item] = type

    def set_dur_hum(self,dur_hum):
        self.dur_hum = dur_hum
        self.t_hum = dur_hum['T']

    def route_for_item(self,item):
        if self.items_to_build[item] == FLASHLIGHT_CLIPPED:
            route = [K1,B1,P]
        elif self.items_to_build[item] == FLASHLIGHT_SCREWS:
            route = [K1, K2, B1, B2, P]
        return route


    def define_parameters(self):
        self.WS_KITTING = [res[0] for res in self.resources[KIT]]
        self.WS_GRIP = [res[0] for res in self.resources[ASM] if res[1] == FLASHLIGHT_CLIPPED]
        self.WS_SCREW = [res[0] for res in self.resources[ASM] if res[1] == FLASHLIGHT_SCREWS]
        self.WS_PACKING = [res[0] for res in self.resources[PACK]]

        self.I = list(self.items_to_build.keys())
        self.I_clipped = {i: int(self.items_to_build[i] == FLASHLIGHT_CLIPPED) for i in self.I}
        self.I_screws = {i: int(self.items_to_build[i] == FLASHLIGHT_SCREWS) for i in self.I}

        self.M_of = {
            K1: self.WS_KITTING,
            K2: self.WS_KITTING,
            B1: self.WS_GRIP + self.WS_SCREW,
            B2: self.WS_GRIP + self.WS_SCREW,
            P: self.WS_PACKING,
        }


        self.z = {}
        self.OPS_i = {}  # ordered list of ops for each item
        self.PREC_i = {}  # precedence edges for each item

        for item in self.I:
            ops_i = self.route_for_item(item)
            self.OPS_i[item] = ops_i
            # presence flags
            for op in [K1, K2, B1, B2, P]:
                self.z[item, op] = 1 if op in ops_i else 0
            # precedence edges follow the per-item sequence
            self.PREC_i[item] = [(ops_i[k], ops_i[k+1]) for k in range(len(ops_i)-1)]

        self.OPS = sorted({op for i in self.I for op in self.OPS_i[i]})
        self.OPS_on_WS = {}
        for op in self.OPS:
            for m in self.M_of[op]:
                self.OPS_on_WS.setdefault(m, set()).add(op)

        # A safe UB on makespan
        self.horizon = int(1e3)

    def model_problem(self):
        self.define_parameters()
        self.define_dv()
        self.define_constraints()
        if self.symmetry_breaking:
            self.define_symmetry_breaking()
        self.optimize()

    def solve(self,solver='ortools'):
        start = time.time()
        if self.m.solve(solver=solver):
            end = time.time()
            elapsed = end - start
            print('Solution found with makespan:', self.makespan.value())
            print(f"Computational time: {elapsed:.3f} seconds")
        else:
            end = time.time()
            elapsed = end - start
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
        if op == K2:
            return False
        if op == B2 and m in self.WS_GRIP:
            return False
        return True


    def HUM(self,op):
        return int(self.dur_hum[op])

    def ARM(self,op, m):
        return int(self.dur_arm.get((op, m), 0))

    def yH(self,i, op):
        return sum(self.x_h[i, op, m] for m in self.M_of[op] if (i, op, m) in self.x_h)  # 0/1

    def dH(self,i, op):
        # Human duration of (i,op) regardless of which machine was chosen
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

        self.S = {(i, op): cp.intvar(0, self.horizon, name=f"S_{i}_{op}") for i in self.I for op in self.OPS}
        self.E = {(i, op): cp.intvar(0, self.horizon, name=f"E_{i}_{op}") for i in self.I for op in self.OPS}
        self.makespan = cp.intvar(0, self.horizon, name="Makespan")

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

        # Definition duration: E[i,op] = S[i,op] + p_hum[op]*sum_m x_h + p_arm[op]*sum_m x_a
        for i in self.I:
            for op in self.OPS:
                if self.z[i, op] == 0:
                    continue
                hum_sum = sum(self.HUM(op) * self.x_h[i, op, m] for m in self.M_of[op] if (i, op, m) in self.x_h)
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


class FJTransportProblem(FJProblem):
    def __init__(self,ILP_formulation=False,ILP_opt=False,symmetry_breaking=False):
        super().__init__(ILP_formulation=ILP_formulation,ILP_opt=ILP_opt,symmetry_breaking=symmetry_breaking)
        self.connected_via_conveyor = []
        self.set_t_conv(3)

    def add_transport(self, from_machine_id, to_machine_id):
            self.connected_via_conveyor.append((from_machine_id, to_machine_id))


    def set_t_conv(self,t_conv):
        self.t_conv = t_conv


    def _pair_from_entry(self, entry):
        return (entry[0], entry[1]) if len(entry) == 2 else (entry[1], entry[2])

    def _conv_pairs(self, trans):
        WS_ASM = self.WS_GRIP + self.WS_SCREW
        pairs = []
        for entry in self.connected_via_conveyor:
            m1, m2 = self._pair_from_entry(entry)
            if trans == KB  and (m1 in self.WS_KITTING) and (m2 in WS_ASM):
                pairs.append((m1, m2))
            elif trans == B12 and (m1 in WS_ASM) and (m2 in WS_ASM):       # ASM→ASM (B1→B2)
                pairs.append((m1, m2))
            elif trans == BP  and (m1 in WS_ASM) and (m2 in self.WS_PACKING):
                pairs.append((m1, m2))
        return pairs


    def _conv_opts(self, i, trans):
        opts = []
        for (m1, m2) in self._conv_pairs(trans):
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
        for i in self.I:
            for trans in [KB, B12, BP]:
                self.S_T[i, trans] = cp.intvar(0, self.horizon, name=f"S_T_{i}_{trans}")
                self.E_T[i, trans] = cp.intvar(0, self.horizon, name=f"E_T_{i}_{trans}")
                self.zT[i, trans, "human"] = cp.boolvar(name=f"T_{i}_{trans}_human")
                for (m1, m2) in self._conv_pairs(trans):
                    self.zT[i, trans, ("conv", m1, m2)] = cp.boolvar(name=f"T_{i}_{trans}_conv_{m1}_{m2}")

    def define_constraints(self):
        self.m = cp.Model()

        # exactly-one assignment + flags
        for i in self.I:
            for op in self.OPS:
                if not self.z[i, op]: continue
                self.m += (
                    sum(self.x_h.get((i, op, m), 0) for m in self.M_of[op]) +
                    sum(self.x_a.get((i, op, m), 0) for m in self.M_of[op])
                ) == 1
                for m in self.M_of[op]:
                    self.m += self.OP_flag[i, op, m] == (self.x_h.get((i, op, m), 0) + self.x_a.get((i, op, m), 0))
                    self.m += self.OP_flag[i, op, m] <= 1
                self.m += self.H_flag[i, op] == sum(self.x_h.get((i, op, m), 0) for m in self.M_of[op])

        # --- Force K1 and K2 on the same kitting workstation ---
        for i in self.I:
            if self.z.get((i, K1), 0) and self.z.get((i, K2), 0):
                for m in self.WS_KITTING:
                    self.m += self.OP_flag[i, K1, m] == self.OP_flag[i, K2, m]

        # durations
        for i in self.I:
            for op in self.OPS:
                if not self.z[i, op]: continue
                hum_sum = sum(self.HUM(op) * self.x_h[i, op, m] for m in self.M_of[op] if (i, op, m) in self.x_h)
                arm_sum = sum(self.ARM(op, m) * self.x_a[i, op, m] for m in self.M_of[op] if (i, op, m) in self.x_a)
                self.m += self.E[i, op] == self.S[i, op] + hum_sum + arm_sum

        # processing precedence from per-item routes
        for i in self.I:
            for pred, succ in self.PREC_i[i]:
                self.m += self.S[i, succ] >= self.E[i, pred]

        # ----- transport precedence (KB, B12, BP) & need -----
        def _k_ops(i):  return [op for op in self.OPS_i[i] if op in (K1, K2)]
        def _b_ops(i):  return [op for op in self.OPS_i[i] if op in (B1, B2)]

        WS_ASM = self.WS_GRIP + self.WS_SCREW

        for i in self.I:
            kops  = _k_ops(i)
            bops  = _b_ops(i)

            # KB: last K → first B
            if kops and bops:
                prevK, firstB = kops[-1], bops[0]
                self.m += self.S_T[i, KB] >= self.E[i, prevK]
                self.m += self.S[i, firstB] >= self.E_T[i, KB]

            # B12: between first and second B (if both exist)
            if len(bops) >= 2:
                b_from, b_to = bops[0], bops[1]
                self.m += self.S_T[i, B12] >= self.E[i, b_from]
                self.m += self.S[i, b_to]  >= self.E_T[i, B12]

            # BP: last B → P (if both exist)
            if bops and self.z[i, P]:
                lastB = bops[-1]
                self.m += self.S_T[i, BP] >= self.E[i, lastB]
                self.m += self.S[i, P]    >= self.E_T[i, BP]

            # choose transport modes and durations
            # KB
            needKB = 1 if (kops and bops) else 0
            optsKB = self._conv_opts(i, KB)
            self.m += self.zT[i, KB, "human"] + sum(optsKB) == needKB
            self.m += self.E_T[i, KB] == self.S_T[i, KB] + (
                self.zT[i, KB, "human"] * self.t_hum + sum(z * self.t_conv for z in optsKB)
            )

            # B12: requires transport only if B1 and B2 end up on different ASM machines
            optsB12 = self._conv_opts(i, B12)
            if len(bops) >= 2:
                b_from, b_to = bops[0], bops[1]
                s_vars = []
                for m in WS_ASM:
                    s = cp.boolvar(name=f"same_B12_{i}_{m}")
                    self.m += s <= self.OP_flag.get((i, b_from, m), 0)
                    self.m += s <= self.OP_flag.get((i, b_to,   m), 0)
                    self.m += s >= self.OP_flag.get((i, b_from, m), 0) + self.OP_flag.get((i, b_to, m), 0) - 1
                    s_vars.append(s)
                same = cp.intvar(0, 1, name=f"same_B12_{i}")
                self.m += same == sum(s_vars)
                needB12 = 1 - same
                self.m += self.zT[i, B12, "human"] + sum(optsB12) == needB12
                self.m += self.E_T[i, B12] == self.S_T[i, B12] + (
                    self.zT[i, B12, "human"] * self.t_hum + sum(z * self.t_conv for z in optsB12)
                )
            else:
                self.m += self.zT[i, B12, "human"] + sum(optsB12) == 0
                self.m += self.E_T[i, B12] == self.S_T[i, B12]

            # BP
            needBP = 1 if (bops and self.z[i, P]) else 0
            optsBP = self._conv_opts(i, BP)
            self.m += self.zT[i, BP, "human"] + sum(optsBP) == needBP
            self.m += self.E_T[i, BP] == self.S_T[i, BP] + (
                self.zT[i, BP, "human"] * self.t_hum + sum(z * self.t_conv for z in optsBP)
            )

        # gate conveyor usage by endpoints
        for i in self.I:
            kops = _k_ops(i); bops = _b_ops(i)
            if kops and bops:
                prevK, firstB = kops[-1], bops[0]
                for (m1, m2) in self._conv_pairs(KB):
                    key = (i, KB, ("conv", m1, m2))
                    if key in self.zT:
                        self.m += self.zT[key] <= self.OP_flag.get((i, prevK,  m1), 0)
                        self.m += self.zT[key] <= self.OP_flag.get((i, firstB, m2), 0)
            if len(bops) >= 2:
                b_from, b_to = bops[0], bops[1]
                for (m1, m2) in self._conv_pairs(B12):
                    key = (i, B12, ("conv", m1, m2))
                    if key in self.zT:
                        self.m += self.zT[key] <= self.OP_flag.get((i, b_from, m1), 0)
                        self.m += self.zT[key] <= self.OP_flag.get((i, b_to,   m2), 0)
            if bops and self.z[i, P]:
                lastB = bops[-1]
                for (m1, m2) in self._conv_pairs(BP):
                    key = (i, BP, ("conv", m1, m2))
                    if key in self.zT:
                        self.m += self.zT[key] <= self.OP_flag.get((i, lastB, m1), 0)
                        self.m += self.zT[key] <= self.OP_flag.get((i, P,     m2), 0)

        # capacities: machines
        for m, ops_here in self.OPS_on_WS.items():
            starts_m, durs_m, ends_m, demands_m = [], [], [], []
            for i in self.I:
                for op in ops_here:
                    if not self.z[i, op]: continue
                    starts_m.append(self.S[i, op])
                    xh = self.x_h.get((i, op, m), 0)
                    xa = self.x_a.get((i, op, m), 0)
                    arm_dur_m = self.ARM(op, m) if (i, op, m) in self.x_a else 0
                    dur_i_m = self.HUM(op) * xh + arm_dur_m * xa
                    durs_m.append(dur_i_m)
                    ends_m.append(self.S[i, op] + dur_i_m)
                    demands_m.append(self.OP_flag[i, op, m])
            if starts_m:
                self.m += cp.Cumulative(starts_m, durs_m, ends_m, demands_m, capacity=1)

        # Capacity: human (processing + human transports) ----------
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

            # human transports (FIX: end must match human duration only)
            for trans in [KB, B12, BP]:
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

        for i in self.I:
            for (m1, m2) in list(per_conv.keys()):
                for trans in [KB, B12, BP]:
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
        """
        Symmetry breaking (CP-native):
          1) Lex order between items of the same class (by their route's start times).
          2) Machine activation ordering within identical machine groups (type,subtype).
          3) Machine lexicographic ordering (prefix dominance) within identical groups.
          4) Human lex ordering: if two items both pick Human for an op, order by item id.
        """

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

        items_sorted = sorted(self.I)

        # -------- (1) Lex order between items of the same class --------
        # items_to_build[i] gives the class (e.g., FLASHLIGHT_CLIPPED / FLASHLIGHT_SCREWS)
        classes = {}
        for i in self.I:
            classes.setdefault(self.items_to_build[i], []).append(i)

        for items in classes.values():
            items = sorted(items)
            if len(items) < 2:
                continue
            # route is identical within the class
            route = self.OPS_i[items[0]]
            for a, b in zip(items[:-1], items[1:]):
                # Lex on start times along the class route
                # S[a,route[0]] <= S[b,route[0]]  OR  (== and next <= ...)
                # Implemented as a chain of implications:
                # (S[a,k] == S[b,k] for k<j) -> S[a,route[j]] <= S[b,route[j]]
                eq_prefix = None
                for j, op in enumerate(route):
                    if j == 0:
                        # S[a,op] <= S[b,op] OR
                        self.m += (self.S[a, op] <= self.S[b, op])
                    else:
                        # if all previous equal, enforce S[a,op] <= S[b,op]
                        cond = (self.S[a, route[j - 1]] == self.S[b, route[j - 1]]) if eq_prefix is None else (
                                    eq_prefix & (self.S[a, route[j - 1]] == self.S[b, route[j - 1]]))
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
            items_op = [i for i in items_sorted if self.z.get((i, op), 0) == 1]
            if len(items_op) < 2:
                continue
            # ha = OR_m x_h[i,op,m]
            ha = {i: or_all([self.x_h.get((i, op, m), None) for m in self.M_of[op] if (i, op, m) in self.x_h])
                  for i in items_op}
            for a, b in zip(items_op[:-1], items_op[1:]):
                if ha[a] is None or ha[b] is None:
                    continue
                both_h = ha[a] & ha[b]
                self.m += ((~both_h) | (self.S[a, op] <= self.S[b, op]))

    def make_gantt(self, folder=None):
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
            for (m1, m2) in self._conv_pairs(trans):
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
        for i in self.I:
            for trans in [KB, B12, BP]:
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
            if k_ops and b_ops:
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
            if len(b_ops) >= 2:
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
            if b_ops and self.z.get((i, P), 0) == 1:
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
            fig.write_image(folder, scale=2)
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
            x1, y1 = pos[m1];
            x2, y2 = pos[m2]
            dx = (box_w / 2 - 0.05) if x2 > x1 else -(box_w / 2 - 0.05)
            annos.append(dict(
                x=x2 - dx, y=y2, xref="x", yref="y",
                ax=x1 + dx, ay=y1, axref="x", ayref="y",
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

















