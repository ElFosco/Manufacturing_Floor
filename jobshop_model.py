from abc import ABC, abstractmethod
from collections import defaultdict
from termios import B2400

import cpmpy as cp
import pandas as pd
import plotly.express as px
from numpy.ma.core import shape
import time
from utility.costant import map_type_stage, K1, B1, B2, K2, P, KB, BP, B12


class FJProblem(ABC):
    def __init__(self,optimize_ILP = False, bounded_tasks = False):
        self.optimize_ILP = optimize_ILP
        self.bounded_tasks = bounded_tasks
        self.dur_hum = {}
        self.dur_arm = {}
        self.stages = 3
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

    def add_workstation(self,type,cost):
        id = self.generate_id_resource()
        stage = map_type_stage[type]
        self.resources[stage].append([id,type])

        for action in cost:
            self.dur_arm[(action,id)] = cost[action]
        return id

    def add_transport(self,id_1,id_2,stage):
        self.connected_via_conveyor.append((stage,id_1,id_2))

    def add_human(self):
        id = self.generate_id_resource()
        self.resources['human'].append(id)
        return id


    def add_mobile_robot(self):
        id = self.generate_id_resource()
        self.resources[id] = ['mobile_robot']
        return id

    def add_items_to_build(self,type,qty):
        for _ in range(qty):
            id_item = self.generate_id_item()
            self.items_to_build[id_item] = type

    def set_dur_hum(self,dur_hum):
        self.dur_hum = dur_hum
        self.t_hum = dur_hum['T']


    def define_parameters(self):
        self.WS1 = [res[0] for res in self.resources[0]]
        self.WS2_GRIP = [res[0] for res in self.resources[1] if res[1] == 1]
        self.WS2_SCREW = [res[0] for res in self.resources[1] if res[1] == 2]
        self.WS3 = [res[0] for res in self.resources[2]]
        self.WS2 = self.WS2_GRIP + self.WS2_SCREW
        self.I = list(self.items_to_build.keys())
        self.I_screws = {i: int(self.items_to_build[i] == 1) for i in self.I}
        # Group machines by stage for compact looping later
        self.WS_STAGE = {0: self.WS1, 1: self.WS2, 2: self.WS3}
        self.OPS_ALWAYS = [K1, B1, P]  # operations always present
        self.OPS_OPTION = [K2, B2]  # present iff r[i]==1
        self.OPS = [K1, K2, B1, B2, P]
        self.M_of = {
            K1: self.WS1, K2: self.WS1,
            B1: self.WS2, B2: self.WS2,
            P: self.WS3
        }
        self.z = {}
        for i in self.I:
            self.z[i, K1] = 1
            self.z[i, B1] = 1
            self.z[i, P] = 1
            self.z[i, K2] = 1 if self.I_screws[i] == 1 else 0
            self.z[i, B2] = 1 if self.I_screws[i] == 1 else 0
        self.horizon = self.compute_makespan_upper_bound()

    def model_problem(self):
        self.define_parameters()
        self.define_dv()
        self.define_constraints()
        self.optimize()

    def _min_arm_dur(self, op):
        vals = []
        for m in self.M_of[op]:
            if self._arm_allowed(op, m):
                vals.append(self.ARM(op, m))
        return min(vals) if vals else int(1e10)  # 'infinity' if arm impossible

    def _compute_time_windows(self):
        """
        Compute HEAD and TAIL times per (i,op) using minimal feasible durations.
        HEAD[i,op] = earliest start (forward pass)
        TAIL[i,op] = minimal remaining time after op until completion (backward pass)
        """
        self.head, self.tail = {}, {}

        for i in self.I:
            hasK2 = bool(self.z[i, K2])
            hasB2 = bool(self.z[i, B2])

            dK1 = self._min_proc_dur(K1)
            dK2 = self._min_proc_dur(K2) if hasK2 else 0
            dB1 = self._min_proc_dur(B1)
            dB2 = self._min_proc_dur(B2) if hasB2 else 0
            dP = self._min_proc_dur(P)

            # ---- HEAD (earliest starts, forward) ----
            self.head[i, K1] = 0
            if hasK2:
                self.head[i, K2] = self.head[i, K1] + dK1
                self.head[i, B1] = self.head[i, K2] + dK2
            else:
                self.head[i, B1] = self.head[i, K1] + dK1
            if hasB2:
                self.head[i, B2] = self.head[i, B1] + dB1
                self.head[i, P] = self.head[i, B2] + dB2
            else:
                self.head[i, P] = self.head[i, B1] + dB1

            # ---- TAIL (remaining time after op, backward) ----
            self.tail[i, P] = 0
            if hasB2:
                self.tail[i, B2] = dP
                self.tail[i, B1] = dB2 + dP
            else:
                self.tail[i, B1] = dP
            if hasK2:
                self.tail[i, K2] = dB1 + self.tail[i, B1]
            else:
                self.tail[i, K2] = 0  # not used if absent
            self.tail[i, K1] = (dK2 if hasK2 else 0) + dB1 + (dB2 if hasB2 else 0) + dP


    def _min_proc_dur(self, op):
        """Minimum feasible duration for 'op' across human and eligible arms."""
        return min(self.HUM(op), self._min_arm_dur(op))

    def compute_makespan_upper_bound(self):
        """
        Safe UB: sequential schedule sum of minimal feasible durations.
        Always valid; not tight, but instant to compute.
        """
        UB = 0
        for i in self.I:
            hasK2 = bool(self.z[i, K2])
            hasB2 = bool(self.z[i, B2])
            UB += self._min_proc_dur(K1)
            if hasK2: UB += self._min_proc_dur(K2)
            UB += self._min_proc_dur(B1)
            if hasB2: UB += self._min_proc_dur(B2)
            UB += self._min_proc_dur(P)
        return int(UB)


    def _arm_allowed(self,op, m):
        if op == K2: return False
        if op == B2 and m in self.WS2_GRIP: return False
        return True

    def _human_pick(self,i, op):
        return sum(self.x_h[i, op, m] for m in self.M_of[op] if (i, op, m) in self.x_h)  # 0/1 due to exactly-one

    def HUM(self,op):
        return int(self.dur_hum[op])

    def ARM(self,op, m):
        return int(self.dur_arm.get((op, m), 0))

    def yH(self,i, op):
        return sum(self.x_h[i, op, m] for m in self.M_of[op] if (i, op, m) in self.x_h)  # 0/1

    def dH(self,i, op):
        # Human duration of (i,op) regardless of which machine was chosen
        return sum(self.HUM(op) * self.x_h[i, op, m] for m in self.M_of[op] if (i, op, m) in self.x_h)


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

    def make_gantt(self):
        """
        Interactive Gantt of the current solution.
        - Human tasks: bar on 'Human' row (solid fill).
        - When human works on a machine, also draw a striped bar on that machine's row
          to show the machine is occupied by the human.
        - Machines are labeled with functionality: KIT, GRIP, GRIP & SCREW, PACK (+ id).
        - Resource lanes are sorted: Human → KIT → GRIP → GRIP & SCREW → PACK
        - Legend shows only Items (one color per item).
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

        # Map id -> type
        id2type = {}
        for stage in (0, 1, 2, 3):
            if stage in self.resources:
                for mid, t in self.resources[stage]:
                    id2type[mid] = t

        MACHINE_LABEL = {0: "KIT", 1: "GRIP", 2: "GRIP & SCREW", 3: "PACK"}

        def machine_label(mid):
            t = id2type.get(mid, None)
            base = MACHINE_LABEL.get(t, f"WS{mid}")
            return f"{base} #{mid}"

        rows = []
        for i in self.I:
            for op in self.OPS:
                if self.z.get((i, op), 0) == 0:
                    continue
                Sv, Ev = self.S[i, op].value(), self.E[i, op].value()
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
                    "ShowLegend": True,  # main bar contributes to legend
                })

                # Duplicate bar for machine if human chosen (patterned)
                if who == "human" and m is not None:
                    rows.append({
                        "Item": str(i),
                        "Op": OP_NAME.get(op, str(op)),
                        "Task": f"Item {i} – {OP_NAME.get(op, str(op))} (machine occupied)",
                        "Start": S, "End": E, "Duration": E - S,
                        "Resource": machine_label(m),
                        "Pattern": "stripe",
                        "ShowLegend": False,  # suppress legend for duplicates
                    })

        df = pd.DataFrame(rows)
        if df.empty:
            raise ValueError("No scheduled tasks to plot (empty dataframe).")

        # Ensure numerics
        df["Start"] = pd.to_numeric(df["Start"])
        df["End"] = pd.to_numeric(df["End"])
        df["Duration"] = df["End"] - df["Start"]

        # ---- Define resource ordering ----
        order = ["Human"]
        order.extend(sorted([r for r in df["Resource"].unique() if r.startswith("KIT")]))
        order.extend(sorted([r for r in df["Resource"].unique() if r.startswith("GRIP #")]))  # plain GRIP
        order.extend(sorted([r for r in df["Resource"].unique() if r.startswith("GRIP & SCREW")]))
        order.extend(sorted([r for r in df["Resource"].unique() if r.startswith("PACK")]))
        category_order = {"Resource": order}

        # ---- Plot ----
        fig = px.bar(
            df,
            x="Duration", y="Resource",
            color="Item",  # legend by Item
            pattern_shape="Pattern",  # visual distinction only
            orientation="h",
            hover_data=["Task", "Op", "Start", "End"],
            base="Start",
            title="Job Shop Schedule",
            category_orders=category_order,
        )

        # --- hide legend for "stripe" duplicates ---
        for trace in fig.data:
            # each trace has a name like "1, solid" or "1, stripe"
            if ", stripe" in trace.name:  # detect stripe duplicates
                trace.showlegend = False
            else:
                # clean up the name so it just shows the Item
                trace.name = trace.name.split(",")[0]

        # Style: white borders, black background
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

        fig.show()




class FJProblemNoTransportILP(FJProblem):
    def __init__(self,optimize_ILP=False,bounded_tasks=False):
        super().__init__(optimize_ILP=optimize_ILP,bounded_tasks=bounded_tasks)

        self.STAGES = 3


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

    def get_solution(self):
        pass

    def define_dv(self):
        self.x_h, self.x_a = {}, {}
        for i in self.I:
            ops_i = [K1, B1, P] + ([K2, B2] if self.I_screws[i] == 1 else [])
            for op in ops_i:
                for m in self.M_of[op]:
                    # Human var always possible where op present
                    self.x_h[i, op, m] = cp.boolvar(name=f"xH_{i}_{op}_{m}")
                    # Arm var only when allowed
                    if self._arm_allowed(op, m):
                        self.x_a[i, op, m] = cp.boolvar(name=f"xA_{i}_{op}_{m}")

        self.S = {(i, op): cp.intvar(0, self.horizon, name=f"S_{i}_{op}") for i in self.I for op in self.OPS}
        self.E = {(i, op): cp.intvar(0, self.horizon, name=f"E_{i}_{op}") for i in self.I for op in self.OPS}
        self.makespan = cp.intvar(0, self.horizon, name="Makespan")

        # Build map machine -> ops it can run
        self.OPS_on_WS = {}
        for op in self.OPS:
            for m in self.M_of[op]:
                self.OPS_on_WS.setdefault(m, set()).add(op)


    def define_constraints(self):

        self.m = cp.Model()
        if self.bounded_tasks:
            self._compute_time_windows()
            for i in self.I:
                for op in self.OPS:
                    if not self.z[i, op]:
                        continue
                    # Earliest start
                    self.m += self.S[i, op] >= self.head[i, op]
                    # Latest start (simple safe bound using minimal dur and remaining tail)
                    self.m += self.S[i, op] <= self.makespan - (self._min_proc_dur(op) + self.tail[i, op])
                    # Keep ends inside makespan (optional but common)
                    self.m += self.E[i, op] <= self.makespan


        # Always-present ops
        for i in self.I:
            for op in self.OPS:
                if self.z[i, op] == 0:
                    self.m += (self.S[i, op] == 0)
                    self.m += (self.E[i, op] == 0)

        for i in self.I:
            for op in self.OPS:
                picks = [self.x_h[i, op, m] for m in self.M_of[op] if (i, op, m) in self.x_h] + \
                        [self.x_a[i, op, m] for m in self.M_of[op] if (i, op, m) in self.x_a]
                if picks:
                    self.m += sum(picks) == self.z[i, op]

        # Definition duration: E[i,op] = S[i,op] + p_hum[op]*sum_m x_h + p_arm[op]*sum_m x_a
        for i in self.I:
            for op in self.OPS:
                if self.z[i, op] == 0:
                    continue
                hum_sum = sum(self.HUM(op) * self.x_h[i, op, m] for m in self.M_of[op] if (i, op, m) in self.x_h)
                arm_sum = sum(self.ARM(op, m) * self.x_a[i, op, m] for m in self.M_of[op] if (i, op, m) in self.x_a)
                self.m += self.E[i, op] == self.S[i, op] + hum_sum + arm_sum

        # Precedence constraints
        for i in self.I:
            self.m += self.S[i, B1] >= self.E[i, K1]
            if self.z[i, K2]:
                self.m += self.S[i, K2] >= self.E[i, K1]
                self.m += self.S[i, B1] >= self.E[i, K2]
            self.m += self.S[i, P] >= self.E[i, B1]
            if self.z[i, B2]:
                self.m += self.S[i, B2] >= self.E[i, B1]
                self.m += self.S[i, P] >= self.E[i, B2]

        for m in self.OPS_on_WS:
            ops_here = list(self.OPS_on_WS[m])
            for idx_i, i in enumerate(self.I):
                for j in self.I[idx_i + 1:]:
                    for op1 in ops_here:
                        if self.z[i, op1] == 0: continue
                        for op2 in ops_here:
                            if self.z[j, op2] == 0: continue
                            # active iff both choose this machine m (human or arm)
                            use_i = ((i, op1, m) in self.x_h and self.x_h[i, op1, m]) | ((i, op1, m) in self.x_a and self.x_a[i, op1, m])
                            use_j = ((j, op2, m) in self.x_h and self.x_h[j, op2, m]) | ((j, op2, m) in self.x_a and self.x_a[j, op2, m])
                            # durations on m (machine-specific for Arm, op/m for Human if provided)
                            dur_i_m = 0
                            if (i, op1, m) in self.x_h: dur_i_m += self.HUM(op1) * self.x_h[i, op1, m]
                            if (i, op1, m) in self.x_a: dur_i_m += self.ARM(op1,m) * self.x_a[i, op1, m]
                            dur_j_m = 0
                            if (j, op2, m) in self.x_h: dur_j_m += self.HUM(op2) * self.x_h[j, op2, m]
                            if (j, op2, m) in self.x_a: dur_j_m += self.ARM(op2,m) * self.x_a[j, op2, m]
                            # No overlap if both use m (OR-encoding with activation)
                            self.m += (((self.S[i, op1] + dur_i_m) <= self.S[j, op2]) |
                                       ((self.S[j, op2] + dur_j_m) <= self.S[i, op1]) |
                                       ((use_i == 0) | (use_j == 0)))


        # ---------- Pooled Human capacity (OR with activation) ----------
        for idx_i, i in enumerate(self.I):
            for j in self.I[idx_i + 1:]:
                for op1 in self.OPS:
                    if self.z[i, op1] == 0: continue
                    for op2 in self.OPS:
                        if self.z[j, op2] == 0: continue
                        yHi = self.yH(i, op1)
                        yHj = self.yH(j, op2)
                        dHi = self.dH(i, op1)
                        dHj = self.dH(j, op2)
                        self.m += (((self.S[i, op1] + dHi) <= self.S[j, op2]) |
                                   ((self.S[j, op2] + dHj) <= self.S[i, op1]) |
                                   ((yHi == 0) | (yHj == 0)))


    def optimize(self):
        if self.optimize_ILP:
            for i in self.I:
                self.m += (self.makespan >= self.E[i, P])
        else:
            self.m += (self.makespan == cp.max([self.E[i, P] for i in self.I]))
            self.m.minimize(self.makespan)



class FJProblemNoTransportCP(FJProblem):
    def __init__(self,optimize_ILP=False,bounded_tasks=False):
        super().__init__(optimize_ILP=optimize_ILP,bounded_tasks=bounded_tasks)
        self.STAGES = 3


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

    def get_solution(self):
        pass

    def define_dv(self):
        # Flag: op (i,op) uses machine m (either by human or arm)
        self.OP_flag = {}
        for i in self.I:
            ops_i = [K1, B1, P] + ([K2, B2] if self.I_screws[i] == 1 else [])
            for op in ops_i:
                for m in self.M_of[op]:
                    self.OP_flag[i, op, m] = cp.boolvar(name=f"OP_flag_{i}_{op}_{m}")

        # Flag: op (i,op) is done by the human (on some m)
        self.H_flag = {}
        for i in self.I:
            for op in self.OPS:
                if self.z[i, op]:
                    self.H_flag[i, op] = cp.boolvar(name=f"H_{i}_{op}")

        self.x_h, self.x_a = {}, {}

        for i in self.I:
            ops_i = [K1, B1, P] + ([K2, B2] if self.I_screws[i] == 1 else [])
            for op in ops_i:
                for m in self.M_of[op]:
                    # Human var always possible where op present
                    self.x_h[i, op, m] = cp.boolvar(name=f"xH_{i}_{op}_{m}")
                    # Arm var only when allowed
                    if self._arm_allowed(op, m):
                        self.x_a[i, op, m] = cp.boolvar(name=f"xA_{i}_{op}_{m}")

        self.S = {(i, op): cp.intvar(0, self.horizon, name=f"S_{i}_{op}") for i in self.I for op in self.OPS}
        self.E = {(i, op): cp.intvar(0, self.horizon, name=f"E_{i}_{op}") for i in self.I for op in self.OPS}
        self.makespan = cp.intvar(0, self.horizon, name="Makespan")

        # Build map machine -> ops it can run
        self.OPS_on_WS = {}
        for op in self.OPS:
            for m in self.M_of[op]:
                self.OPS_on_WS.setdefault(m, set()).add(op)


    def define_constraints(self):

        self.m = cp.Model()
        if self.bounded_tasks:
            self._compute_time_windows()
            for i in self.I:
                for op in self.OPS:
                    if not self.z[i, op]:
                        continue
                    # Earliest start
                    self.m += self.S[i, op] >= self.head[i, op]
                    # Latest start (simple safe bound using minimal dur and remaining tail)
                    self.m += self.S[i, op] <= self.makespan - (self._min_proc_dur(op) + self.tail[i, op])
                    # Keep ends inside makespan (optional but common)
                    self.m += self.E[i, op] <= self.makespan

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

                # per machine: u = x_h + x_a ; and forbid both on same m
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
        for i in self.I:
            self.m += self.S[i, B1] >= self.E[i, K1]
            if self.z[i, K2]:
                self.m += self.S[i, K2] >= self.E[i, K1]
                self.m += self.S[i, B1] >= self.E[i, K2]
            self.m += self.S[i, P] >= self.E[i, B1]
            if self.z[i, B2]:
                self.m += self.S[i, B2] >= self.E[i, B1]
                self.m += self.S[i, P] >= self.E[i, B2]

        for m, ops_here in self.OPS_on_WS.items():
            starts_m, durs_m, ends_m, demands_m = [], [], [], []
            for i in self.I:
                for op in ops_here:
                    if not self.z[i, op]:
                        continue
                    # shared start for (i,op)
                    starts_m.append(self.S[i, op])
                    # duration on this machine m if chosen (0 otherwise because demand=0)
                    dur_i_m = (
                            self.HUM(op) * self.x_h.get((i, op, m), 0) +
                            self.ARM(op, m) * self.x_a.get((i, op, m), 0)
                    )
                    durs_m.append(dur_i_m)
                    # end aligned with that duration
                    ends_m.append(self.S[i, op] + dur_i_m)
                    # demand = 1 iff (i,op) uses machine m (human or arm)
                    demands_m.append(self.OP_flag[i, op, m])
            if starts_m:
                self.m += cp.Cumulative(starts_m, durs_m, ends_m, demands_m, capacity=1)

        starts_h, durs_h, ends_h, demands_h = [], [], [], []
        for i in self.I:
            for op in self.OPS:
                if not self.z[i, op]:
                    continue
                starts_h.append(self.S[i, op])
                # duration if human; 0 if not human (because demand=0)
                durs_h.append(self.HUM(op) * self.H_flag[i, op])
                ends_h.append(self.S[i, op] + self.HUM(op) * self.H_flag[i, op])
                demands_h.append(self.H_flag[i, op])
        if starts_h:
            self.m += cp.Cumulative(starts_h, durs_h, ends_h, demands_h, capacity=1)



    def optimize(self):
        if self.optimize_ILP:
            for i in self.I:
                self.m += (self.makespan >= self.E[i, P])
            self.m.minimize(self.makespan)
        else:
            self.m += (self.makespan == cp.max([self.E[i, P] for i in self.I]))
            self.m.minimize(self.makespan)


class FJProblemCP(FJProblem):
    def __init__(self,optimize_ILP=False):
        super().__init__(optimize_ILP=optimize_ILP)
        self.STAGES = 3


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

    def get_solution(self):
        pass

    def set_t_conv(self,t_conv):
        self.t_conv = t_conv

    def _conv_opts(self,i, trans):
        opts = []
        for (stage, m1, m2) in self.connected_via_conveyor:
            if trans == KB and stage == 0 and (m1 in self.WS1) and (m2 in self.WS2):
                key = (i, KB, ("conv", m1, m2))
                if key in self.zT: opts.append(self.zT[key])
            if trans == BP and stage == 1 and (m1 in self.WS2) and (m2 in self.WS3):
                key = (i, BP, ("conv", m1, m2))
                if key in self.zT: opts.append(self.zT[key])
            if trans == B12 and stage == 1 and (m1 in self.WS2) and (m2 in self.WS2):
                key = (i, B12, ("conv", m1, m2))
                if key in self.zT: opts.append(self.zT[key])
        return opts

    def define_dv(self):
        # Flag: op (i,op) uses machine m (either by human or arm)
        self.OP_flag = {}
        for i in self.I:
            ops_i = [K1, B1, P] + ([K2, B2] if self.I_screws[i] == 1 else [])
            for op in ops_i:
                for m in self.M_of[op]:
                    self.OP_flag[i, op, m] = cp.boolvar(name=f"OP_flag_{i}_{op}_{m}")

        # Flag: op (i,op) is done by the human (on some m)
        self.H_flag = {}
        for i in self.I:
            for op in self.OPS:
                if self.z[i, op]:
                    self.H_flag[i, op] = cp.boolvar(name=f"H_{i}_{op}")

        self.x_h, self.x_a = {}, {}

        for i in self.I:
            ops_i = [K1, B1, P] + ([K2, B2] if self.I_screws[i] == 1 else [])
            for op in ops_i:
                for m in self.M_of[op]:
                    # Human var always possible where op present
                    self.x_h[i, op, m] = cp.boolvar(name=f"xH_{i}_{op}_{m}")
                    # Arm var only when allowed
                    if self._arm_allowed(op, m):
                        self.x_a[i, op, m] = cp.boolvar(name=f"xA_{i}_{op}_{m}")

        self.S = {(i, op): cp.intvar(0, self.horizon, name=f"S_{i}_{op}") for i in self.I for op in self.OPS}
        self.E = {(i, op): cp.intvar(0, self.horizon, name=f"E_{i}_{op}") for i in self.I for op in self.OPS}
        self.makespan = cp.intvar(0, self.horizon, name="Makespan")

        # Build map machine -> ops it can run
        self.OPS_on_WS = {}
        for op in self.OPS:
            for m in self.M_of[op]:
                self.OPS_on_WS.setdefault(m, set()).add(op)

        # For each item i, each stage transition: KB (kitting→building), BP (building→packing)
        self.S_T, self.E_T, self.zT = {}, {}, {}
        for i in self.I:
            for trans in ["KB", "BP", B12]:  # B12 kept, but shares stage=1 with BP
                self.S_T[i, trans] = cp.intvar(0, self.horizon, name=f"S_T_{i}_{trans}")
                self.E_T[i, trans] = cp.intvar(0, self.horizon, name=f"E_T_{i}_{trans}")
                # Human option
                self.zT[i, trans, "human"] = cp.boolvar(name=f"T_{i}_{trans}_human")

                # Conveyor options from your network:
                for (stage, m1, m2) in self.connected_via_conveyor:
                    # KB: stage 0, WS1 -> WS2
                    if trans == KB and stage == 0 and (m1 in self.WS1) and (m2 in self.WS2):
                        self.zT[i, trans, ("conv", m1, m2)] = cp.boolvar(
                            name=f"T_{i}_{trans}_conv_{m1}_{m2}"
                        )
                    # BP: stage 1, WS2 -> WS3   (same stage id as B12)
                    if trans == BP and stage == 1 and (m1 in self.WS2) and (m2 in self.WS3):
                        self.zT[i, trans, ("conv", m1, m2)] = cp.boolvar(
                            name=f"T_{i}_{trans}_conv_{m1}_{m2}"
                        )
                    # B12: stage 1, WS2 -> WS2   (intra-assembly, same stage id as BP)
                    if trans == B12 and stage == 1 and (m1 in self.WS2) and (m2 in self.WS2):
                        self.zT[i, trans, ("conv", m1, m2)] = cp.boolvar(
                            name=f"T_{i}_{trans}_conv_{m1}_{m2}"
                        )

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

                # per machine: u = x_h + x_a ; and forbid both on same m
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

        #Precedence constraints
        for i in self.I:
            self.m += self.S[i, B1] >= self.E[i, K1]
            self.m += self.S_T[i, KB] >= self.E[i, K1]
            if self.z[i, K2]:
                self.m += self.S_T[i, B12] >= self.E[i, B1]
                self.m += self.S[i, B2] >= self.E_T[i, B12]
                self.m += self.S[i, K2] >= self.E[i, K1]
                self.m += self.S[i, B1] >= self.E[i, K2]
                self.m += self.S_T[i, KB] >= self.E[i, K2]
            self.m += self.S[i, P] >= self.E[i, B1]
            self.m += self.S[i, B1] >= self.E_T[i, KB]
            self.m += self.S_T[i, BP] >= self.E[i, B1]
            if self.z[i, B2]:
                self.m += self.S[i, B2] >= self.E[i, B1]
                self.m += self.S[i, P] >= self.E[i, B2]
                self.m += self.S_T[i, BP] >= self.E[i, B2]
            self.m += self.S[i, P] >= self.E_T[i, BP]

        # for i in self.I:
        #     self.m += self.S_T[i, "KB"] >= self.E[i, K1]
        #     if self.z[i, K2]:
        #         self.m += self.S_T[i, "KB"] >= self.E[i, K2]
        #     self.m += self.S[i, B1] >= self.E_T[i, KB]
        #
        #     # B12: between B1 and B2 if B2 exists (only if WS2 machines differ)
        # for i in self.I:
        #     if self.z[i, B2]:
        #         self.m += self.S_T[i, B12] >= self.E[i, B1]
        #         self.m += self.S[i, B2] >= self.E_T[i, B12]
        #
        #     # BP: after last building, before P (always required)
        # for i in self.I:
        #     self.m += self.S_T[i, "BP"] >= self.E[i, B1]
        #     if self.z[i, B2]:
        #         self.m += self.S_T[i, "BP"] >= self.E[i, B2]
        #     self.m += self.S[i, P] >= self.E_T[i, BP]

        for i in self.I:
            for trans in [KB, BP]:
                conv_options = self._conv_opts(i, trans)
                self.m += self.zT[i, trans, "human"] + sum(conv_options) == 1
                self.m += self.E_T[i, trans] == self.S_T[i, trans] + (
                        self.zT[i, trans, "human"] * self.t_hum +
                        sum(opt * self.t_conv for opt in conv_options)
                )

        # --- B12: need transport iff B1 and B2 on different WS2 ---
        for i in self.I:
            conv_options = self._conv_opts(i, B12)
            if self.z[i, B2]:
                # same-machine linearization over WS2
                s_vars = []
                for m in self.WS2:
                    s = cp.boolvar(name=f"same_B12_{i}_{m}")
                    self.m += s <= self.OP_flag.get((i, B1, m), 0)
                    self.m += s <= self.OP_flag.get((i, B2, m), 0)
                    self.m += s >= self.OP_flag.get((i, B1, m), 0) + self.OP_flag.get((i, B2, m), 0) - 1
                    s_vars.append(s)
                same = cp.intvar(0, 1, name=f"same_B12_{i}")
                self.m += same == sum(s_vars)

                needT = 1 - same
                self.m += self.zT[i, B12, "human"] + sum(conv_options) == needT
                self.m += self.E_T[i, B12] == self.S_T[i, B12] + (
                        self.zT[i, B12, "human"] * self.t_hum +
                        sum(opt * self.t_conv for opt in conv_options)
                )
            else:
                # No B2 → no B12 transport
                self.m += self.zT[i, B12, "human"] + sum(conv_options) == 0
                self.m += self.E_T[i, B12] == self.S_T[i, B12]

        for i in self.I:
            # KB: prev = last kitting, next = B1
            prevK = K2 if self.z[i, K2] else K1
            for (stage, m1, m2) in self.connected_via_conveyor:
                if stage == 0 and (m1 in self.WS1) and (m2 in self.WS2):
                    key = (i, KB, ("conv", m1, m2))
                    if key in self.zT:
                        self.m += self.zT[key] <= self.OP_flag.get((i, prevK, m1), 0)
                        self.m += self.zT[key] <= self.OP_flag.get((i, B1, m2), 0)

            # BP: prev = last building, next = P
            prevB = B2 if self.z[i, B2] else B1
            for (stage, m1, m2) in self.connected_via_conveyor:
                if stage == 1 and (m1 in self.WS2) and (m2 in self.WS3):
                    key = (i, BP, ("conv", m1, m2))
                    if key in self.zT:
                        self.m += self.zT[key] <= self.OP_flag.get((i, prevB, m1), 0)
                        self.m += self.zT[key] <= self.OP_flag.get((i, P, m2), 0)

            # B12: prev = B1, next = B2   (both WS2), same stage id=1 but WS2→WS2
            if self.z[i, B2]:
                for (stage, m1, m2) in self.connected_via_conveyor:
                    if stage == 1 and (m1 in self.WS2) and (m2 in self.WS2):
                        key = (i, B12, ("conv", m1, m2))
                        if key in self.zT:
                            self.m += self.zT[key] <= self.OP_flag.get((i, B1, m1), 0)
                            self.m += self.zT[key] <= self.OP_flag.get((i, B2, m2), 0)



        #cumulative on machines
        for m, ops_here in self.OPS_on_WS.items():
            starts_m, durs_m, ends_m, demands_m = [], [], [], []
            for i in self.I:
                for op in ops_here:
                    if not self.z[i, op]:
                        continue
                    starts_m.append(self.S[i, op])
                    # guard ARM lookup
                    xh = self.x_h.get((i, op, m), 0)
                    xa = self.x_a.get((i, op, m), 0)
                    arm_dur_m = self.ARM(op, m) if (i, op, m) in self.x_a else 0
                    dur_i_m = self.HUM(op) * xh + arm_dur_m * xa
                    durs_m.append(dur_i_m)
                    ends_m.append(self.S[i, op] + dur_i_m)
                    demands_m.append(self.OP_flag[i, op, m])
            if starts_m:
                self.m += cp.Cumulative(starts_m, durs_m, ends_m, demands_m, capacity=1)

        starts_h, durs_h, ends_h, demands_h = [], [], [], []
        for i in self.I:
            for op in self.OPS:
                if not self.z[i, op]:
                    continue
                starts_h.append(self.S[i, op])
                durs_h.append(self.HUM(op) * self.H_flag[i, op])
                ends_h.append(self.S[i, op] + self.HUM(op) * self.H_flag[i, op])
                demands_h.append(self.H_flag[i, op])
            # human transports (KB, B12, BP)
            for trans in [KB, B12, BP]:
                starts_h.append(self.S_T[i, trans])
                durs_h.append(self.t_hum * self.zT[i, trans, "human"])
                ends_h.append(self.E_T[i, trans])
                demands_h.append(self.zT[i, trans, "human"])
        if starts_h:
            self.m += cp.Cumulative(starts_h, durs_h, ends_h, demands_h, capacity=1)

        per_conv = defaultdict(list)  # key: (trans, m1, m2) -> list of (S_T, zT, E_T)
        for (stage, m1, m2) in self.connected_via_conveyor:
            if stage == 0 and (m1 in self.WS1) and (m2 in self.WS2):
                trans = KB
            elif stage == 1 and (m1 in self.WS2) and (m2 in self.WS3):
                trans = BP
            elif stage == 1 and (m1 in self.WS2) and (m2 in self.WS2):
                trans = B12
            else:
                continue
            per_conv[(trans, m1, m2)]  # ensure key exists

        for i in self.I:
            for (trans, m1, m2) in list(per_conv.keys()):
                key = (i, trans, ("conv", m1, m2))
                if key in self.zT:
                    per_conv[(trans, m1, m2)].append((self.S_T[i, trans], self.zT[key], self.E_T[i, trans]))

        for (trans, m1, m2), items in per_conv.items():
            if not items:
                continue
            starts = [S for (S, z, E) in items]
            durs = [self.t_conv * z for (S, z, E) in items]
            ends = [E for (S, z, E) in items]
            demands = [z for (S, z, E) in items]
            self.m += cp.Cumulative(starts, durs, ends, demands, capacity=1)

    def optimize(self):
        if self.optimize_ILP:
            for i in self.I:
                self.m += (self.makespan >= self.E[i, P])
            self.m.minimize(self.makespan)
        else:
            self.m += (self.makespan == cp.max([self.E[i, P] for i in self.I]))
            self.m.minimize(self.makespan)

    def make_gantt(self):
        """
        Interactive Gantt:
        - Human tasks: 'Human' lane (solid).
        - If human operates on a machine, add a striped bar on that machine's lane.
        - Transport:
            * Human transport → bar on 'Human' (pattern 'x')
            * Conveyor (m1→m2) → bar on 'CONV <KB/BP/BB> <from> → <to>' (pattern '.')
        - Machines labeled KIT/GRIP/GRIP & SCREW/PACK + id.
        - Lanes ordered: Human → KIT → GRIP → GRIP & SCREW → PACK → CONV KB → CONV BB/BP.
        - Legend shows only Items (one color per item). Clicking an item hides solid+stripe together.
        """
        import pandas as pd
        import plotly.graph_objects as go

        # --- Helpers / naming ---
        OP_NAME = {
            K1: "K1 (Kitting)", K2: "K2 (Screw prep)",
            B1: "B1 (Assembly core)", B2: "B2 (Screw insertion)",
            P: "P (Packaging)",
        }
        MACHINE_LABEL = {0: "KIT", 1: "GRIP", 2: "GRIP & SCREW", 3: "PACK"}

        # map machine id -> type code (0..3) from self.resources
        id2type = {}
        for stage in (0, 1, 2, 3):
            if stage in self.resources:
                for mid, t in self.resources[stage]:
                    id2type[mid] = t

        def machine_label(mid):
            t = id2type.get(mid, None)
            base = MACHINE_LABEL.get(t, f"WS{mid}")
            return f"{base} #{mid}"

        # normalize transport tag (works if you pass strings or constants)
        def tag_of(trans):
            try:
                # if trans is a string already
                if isinstance(trans, str):
                    return trans
                # else compare by identity/equality to your constants
                if trans == KB:   return "KB"
                if trans == BP:   return "BP"
                if trans == B12 or trans == "BB": return "BB"
            except Exception:
                pass
            return str(trans)

        # Determine label for a conveyor resource line
        def conv_label(m1, m2):
            # infer KB/BP/BB from machine families (WS1->WS2, WS2->WS3, WS2->WS2)
            t1, t2 = id2type.get(m1, None), id2type.get(m2, None)
            if t1 in (0,) and t2 in (1, 2):
                stage_tag = "KB"
            elif t1 in (1, 2) and t2 in (3,):
                stage_tag = "BP"
            elif t1 in (1, 2) and t2 in (1, 2):
                stage_tag = "BB"
            else:
                stage_tag = "CONV"
            return f"CONV {stage_tag} {machine_label(m1)} → {machine_label(m2)}"

        # --- Collect rows ---
        rows = []
        # Processing ops
        for i in self.I:
            for op in self.OPS:
                if self.z.get((i, op), 0) == 0:  # absent
                    continue
                Sv, Ev = self.S[i, op].value(), self.E[i, op].value()
                if Sv is None or Ev is None:  # no value
                    continue
                S, E = int(Sv), int(Ev)
                who, m = self._picked_assignment(i, op)  # ('human'|'arm'|None, m_id|None)

                # main bar (Human or Machine)
                primary_resource = "Human" if who == "human" else (machine_label(m) if m is not None else "Unassigned")
                rows.append({
                    "Item": str(i), "Op": OP_NAME.get(op, str(op)),
                    "Task": f"Item {i} – {OP_NAME.get(op, str(op))}",
                    "Start": S, "End": E, "Duration": E - S,
                    "Resource": primary_resource,
                    "Pattern": "solid",  # will be mapped to ''
                })
                # Duplicate machine occupancy if human operates there
                if who == "human" and m is not None:
                    rows.append({
                        "Item": str(i), "Op": OP_NAME.get(op, str(op)),
                        "Task": f"Item {i} – {OP_NAME.get(op, str(op))} (machine occupied)",
                        "Start": S, "End": E, "Duration": E - S,
                        "Resource": machine_label(m),
                        "Pattern": "stripe",  # will be mapped to '/'
                    })

        # Transport ops (support KB/BP/BB; you may store tags differently)
        # Build a set of all transport tags present in the model
        trans_tags = []
        for t in (KB, BP, B12):
            if all((i, t) in self.S_T for i in self.I):
                trans_tags.append(t)

        for i in self.I:
            for trans in trans_tags:
                Sv, Ev = self.S_T[i, trans].value(), self.E_T[i, trans].value()
                if Sv is None or Ev is None:
                    continue
                S, E = int(Sv), int(Ev)

                # Human transport?
                if (i, trans, "human") in self.zT and self.zT[i, trans, "human"].value() == 1:
                    rows.append({
                        "Item": str(i),
                        "Op": f"T_{trans}",
                        "Task": f"Item {i} – Transport {trans} by Human",
                        "Start": S, "End": E, "Duration": E - S,
                        "Resource": "Human",
                        "Pattern": "x",  # will be mapped to 'x'
                    })
                else:
                    # find chosen conveyor (m1->m2)
                    chosen = None
                    for (stage, m1, m2) in self.connected_via_conveyor:
                        key = (i, trans, ("conv", m1, m2))
                        if key in self.zT and self.zT[key].value() == 1:
                            chosen = (m1, m2)
                            break
                    if chosen:
                        m1, m2 = chosen
                        rows.append({
                            "Item": str(i),
                            "Op": f"T_{trans}",
                            "Task": f"Item {i} – Transport {trans} via Conveyor {m1}→{m2}",
                            "Start": S, "End": E, "Duration": E - S,
                            "Resource": conv_label(m1, m2),
                            "Pattern": "dot",  # will be mapped to '.'
                        })

        df = pd.DataFrame(rows)
        if df.empty:
            raise ValueError("No scheduled tasks to plot (empty dataframe).")

        # numerics
        df["Start"] = pd.to_numeric(df["Start"])
        df["End"] = pd.to_numeric(df["End"])
        df["Duration"] = df["End"] - df["Start"]

        # ---- Resource ordering ----
        used = set(df["Resource"].unique())
        conv_labels_kb, conv_labels_bb, conv_labels_bp = [], [], []
        # Gather conveyor lanes that actually appear
        for (stage, m1, m2) in self.connected_via_conveyor:
            lbl = conv_label(m1, m2)
            if lbl in used:
                if " CONV KB " in lbl:
                    conv_labels_kb.append(lbl)
                elif " CONV BB " in lbl:
                    conv_labels_bb.append(lbl)
                elif " CONV BP " in lbl:
                    conv_labels_bp.append(lbl)
        order = ["Human"]
        order += sorted([r for r in used if r.startswith("KIT")])
        order += sorted([r for r in used if r.startswith("GRIP #")])
        order += sorted([r for r in used if r.startswith("GRIP & SCREW")])
        order += sorted([r for r in used if r.startswith("PACK")])
        order += sorted(conv_labels_kb) + sorted(conv_labels_bb) + sorted(conv_labels_bp)

        # Categorical order for y-axis
        df["Resource"] = pd.Categorical(df["Resource"], categories=order, ordered=True)
        df = df.sort_values(["Resource", "Start", "End", "Item"]).reset_index(drop=True)

        # Map our friendly pattern labels to valid Plotly shapes
        # Valid: '', '/', '\\', 'x', '-', '|', '+', '.'
        pattern_map = {"solid": "", "stripe": "/", "x": "x", "dot": "."}
        df["PatternShape"] = df["Pattern"].map(pattern_map).fillna("")

        # ---- Build one Bar trace per Item (legend only by Item) ----
        fig = go.Figure()
        item_keys = sorted(df["Item"].unique(), key=lambda x: int(x) if x.isdigit() else x)
        for item in item_keys:
            dfi = df[df["Item"] == item]
            hovertext = [
                f"{row['Task']}<br>Start: {row['Start']} End: {row['End']}"
                for _, row in dfi.iterrows()
            ]
            fig.add_bar(
                x=dfi["Duration"],
                y=dfi["Resource"],
                base=dfi["Start"],
                orientation="h",
                name=str(item),  # single legend entry per item
                hovertext=hovertext,
                hoverinfo="text",
                marker=dict(
                    line=dict(width=3, color="white"),  # thick white borders
                    pattern=dict(shape=dfi["PatternShape"].tolist())
                ),
            )

        # Style / theme
        fig.update_layout(
            title="Job Shop Schedule",
            plot_bgcolor="black",
            paper_bgcolor="black",
            xaxis=dict(showgrid=True, gridcolor="lightgray", title="Time", color="white"),
            yaxis=dict(showgrid=True, gridcolor="lightgray", title="Resource", color="white",
                       categoryorder="array", categoryarray=order),
            font=dict(color="white"),
            title_font=dict(color="white"),
            legend_title=dict(text="Item", font=dict(color="white")),
            barmode="overlay",
        )

        # Optional makespan line
        if hasattr(self, "makespan") and self.makespan.value() is not None:
            ms = int(self.makespan.value())
            fig.add_vline(x=ms, line_dash="dash",
                          annotation_text=f"Makespan={ms}",
                          annotation_position="top right", opacity=0.6)

        fig.show()



















