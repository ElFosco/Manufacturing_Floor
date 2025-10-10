from abc import ABC, abstractmethod
from collections import defaultdict
from termios import B2400

import cpmpy as cp
import pandas as pd
import plotly.express as px
from numpy.ma.core import shape
import time

from cpmpy import SolverLookup
from utils.costant import K1, B1, B2, K2, P, KB, BP, B12, KIT, FLASHLIGHT_CLIPPED, ASM, \
    FLASHLIGHT_SCREWS, PACK
from utils.item_definitions import get_item_definition, get_all_item_names


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

    def model_problem(self,objective_type=0):
        self.define_parameters()
        self.define_dv()
        self.define_constraints()
        if self.symmetry_breaking:
            self.define_symmetry_breaking()
        self.optimize(objective_type)

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
        return solved,elapsed,solver.status().exitstatus



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

    def make_gantt(self, folder=None, html=False):
        """
        Interactive Gantt with custom lane ordering and flow lines.

        Changes:
          - Show one lane per Human/Robot (Human #k / Robot #k).
          - Hide conveyor transport bars entirely (and their flow lines).
          - Flow lines connect to the exact Human/Robot lane used.
          - Keep previous look-and-feel and ordering for machines.
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
            return f"{src} -> {dst}"  # ASCII arrow for cross-renderer safety

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
            """Return 'Human' or 'Robot' if chosen; hide conveyors by returning None."""
            # Human transport?
            if (i, trans, "human") in self.zT and self.zT[i, trans, "human"].value() == 1:
                return "Human"
            # Robot transport?
            if (i, trans, "robot") in self.zT and self.zT[i, trans, "robot"].value() == 1:
                return "Robot"
            # Conveyor transport? -> hide as a task
            pairs = self._conv_pairs(trans)
            for m1, m2 in pairs:
                key = (i, trans, ("conv", m1, m2))
                if key in self.zT and self.zT[key].value() == 1:
                    return None
            return None  # default: don't render unknown transports

        # Label normalization to avoid accidental new categories (spaces, unicode)
        def norm_label(s: str) -> str:
            if s is None:
                return s
            s = str(s).strip()
            s = s.replace("→", "->")
            s = " ".join(s.split())
            return s

        def assign_agent_lanes(rows, base_label, count):
            """
            Greedy interval coloring: assign (Start,End) bars whose Resource == base_label
            to lanes 'base_label #k' with k in [1..count], avoiding overlaps per lane.
            Mutates rows in place. Assumes capacity was respected by the model.
            """
            if count <= 1:
                return
            idxs = [idx for idx, r in enumerate(rows)
                    if r.get("Resource") == base_label]
            idxs.sort(key=lambda j: (rows[j]["Start"], rows[j]["End"]))
            avail = [0] * count  # lane k is free at time avail[k]
            for j in idxs:
                S, E = rows[j]["Start"], rows[j]["End"]
                lane = None
                for k in range(count):
                    if avail[k] <= S:
                        lane = k
                        break
                if lane is None:
                    lane = min(range(count), key=lambda k: avail[k])
                avail[lane] = E
                rows[j]["Resource"] = f"{base_label} #{lane + 1}"

        # ---------- collect bars ----------
        rows = []
        transport_lane_map = {}  # (i, trans) -> final lane label (Human #k / Robot #k)
        transport_row_ids = {}  # (i, trans) -> indices in rows (should be one)

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
                who, m = self._picked_assignment(i, op)  # ← moved up so we know who does it

                rows.append({
                    "Item": str(i), "Op": OP_NAME.get(op, str(op)),
                    "Task": f"Item {i} – {OP_NAME.get(op, str(op))}",
                    "Start": S, "End": E, "Duration": E - S,
                    "Resource": res,
                    "Pattern": "stripe" if who == "human" else "solid",  # ← stripe on Human lane
                    "TransType": "",
                })

                if who == "human" and m is not None:
                    rows.append({
                        "Item": str(i), "Op": OP_NAME.get(op, str(op)),
                        "Task": f"Item {i} – {OP_NAME.get(op, str(op))} (machine occupied)",
                        "Start": S, "End": E, "Duration": E - S,
                        "Resource": machine_label(m),
                        "Pattern": "stripe",  # keep stripe on the mirrored machine bar too
                        "TransType": "",
                    })

        # transport ops (only Human/Robot; skip conveyor bars)
        all_transitions = self.get_all_transitions()
        for i in self.I:
            for trans in all_transitions:
                if (i, trans) not in self.S_T:
                    continue
                Sv, Ev = self.S_T[i, trans].value(), self.E_T[i, trans].value()
                if Sv is None or Ev is None:
                    continue
                S, E = int(Sv), int(Ev)
                res = resource_for_transport(i, trans)  # "Human" / "Robot" / None
                if res is None:
                    continue  # hide conveyors
                pattern = "htrans" if res == "Human" else "rtrans"
                row = {
                    "Item": str(i), "Op": f"T_{trans}",
                    "Task": f"Item {i} – Transport {trans}",
                    "Start": S, "End": E, "Duration": E - S,
                    "Resource": res,
                    "Pattern": pattern,  # marks as line-only
                    "TransType": ("KB" if trans == KB else "B12" if trans == B12 else "BP"),
                }
                rows.append(row)
                transport_row_ids.setdefault((i, trans), []).append(len(rows) - 1)

        # --- split pooled agents into lanes and remember exact lane per transport ---
        num_humans = len(self.resources.get('human', []))
        num_robots = len(self.resources.get('robot', []))
        assign_agent_lanes(rows, "Human", num_humans)
        assign_agent_lanes(rows, "Robot", num_robots)
        for (i, trans), idxs in transport_row_ids.items():
            if idxs:
                transport_lane_map[(i, trans)] = rows[idxs[0]]["Resource"]  # e.g., "Human #2"

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
            # 7 KIT WS
            # 8 ROBOT lanes
            # 10 HUMAN lanes
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
            if resource.startswith("KIT"):
                return 7
            if resource.startswith("Robot #"):
                return 8
            if resource.startswith("Human #"):
                return 10
            if resource in ("Human", "Robot"):
                return 10 if resource == "Human" else 8
            if resource == "Unknown":
                return 11
            return 9

        # Normalize & rank
        df["Resource"] = df["Resource"].map(norm_label)
        df = df.dropna(subset=["Resource"])
        df = df[df["Resource"].notna()]
        df = df[df["Resource"] != ""]
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

        # --- Force Robot lanes above Human lanes at bottom ---
        human_lanes = [r for r in order if r.startswith("Human")]
        robot_lanes = [r for r in order if r.startswith("Robot")]
        other_lanes = [r for r in order if not (r.startswith("Human") or r.startswith("Robot"))]
        human_lanes.sort(key=lambda x: int(x.split("#")[1]) if "#" in x else 10 ** 9)
        robot_lanes.sort(key=lambda x: int(x.split("#")[1]) if "#" in x else 10 ** 9)
        order = other_lanes + robot_lanes + human_lanes

        # lock categorical dtype
        from pandas.api.types import CategoricalDtype
        lane_dtype = CategoricalDtype(categories=order, ordered=True)
        df["Resource"] = df["Resource"].astype(lane_dtype)

        stray = sorted(set(df["Resource"].astype(str)) - set(order))
        if stray:
            raise ValueError(f"Unknown lane labels not in category order: {stray}")

        # patterns
        pattern_map = {
            "solid": "",
            "stripe": "/",
            "x": ".", "o": ".", "dot": ".",
            "htrans": "", "rtrans": "",
        }
        df["PatternShape"] = df["Pattern"].map(pattern_map).fillna("")


        palette = q.Plotly
        item_list = sorted(df["Item"].unique(), key=lambda x: int(x) if x.isdigit() else x)
        item_color = {item: palette[idx % len(palette)] for idx, item in enumerate(item_list)}

        # ---------- figure ----------
        fig = go.Figure()

        for item in item_list:
            dfi = df[df["Item"] == item]
            legend_done = False

            for res in dfi["Resource"].cat.categories:
                dfr = dfi[dfi["Resource"] == res]
                if dfr.empty:
                    continue

                # --- non-stripe
                dfn = dfr[(dfr["Pattern"] != "htrans") & (dfr["Pattern"] != "rtrans")]
                if not dfn.empty:
                    hovertext = [
                        f"{row['Task']}<br>Start: {row['Start']} End: {row['End']}"
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
                            line=dict(width=3, color="white"),
                            pattern=dict(shape=dfn["PatternShape"].tolist(), size=10, solidity=0.5),
                        ),
                        legendgroup=str(item),
                        alignmentgroup=str(res),
                        offsetgroup=str(res),
                        showlegend=(not legend_done),
                        width=0.5,
                    )
                    legend_done = True

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
                        showlegend=False,
                        width=0.5,
                    )

            for patt in ("htrans", "rtrans"):
                dtt = dfi[dfi["Pattern"] == patt]
                if not dtt.empty:
                    for _, row in dtt.iterrows():
                        fig.add_scatter(
                            x=[row["Start"], row["End"]],
                            y=[row["Resource"], row["Resource"]],
                            mode="lines",
                            line=dict(
                                color=item_color[item],
                                width=8,
                                dash="dot",
                            ),
                            showlegend=False,
                            hoverinfo="skip",
                            legendgroup=str(item),
                            name=f"{item} transport",
                        )

        # ---------- flow lines to solid endpoints ----------
        def add_poly_segments(fig, color, xys, legendgroup):
            """
            Draws a polyline made of consecutive segments through the list of (x,y) points.
            Skips segments with None y (invalid category).
            """
            import math
            seg_x, seg_y = [], []

            def push(a, b):
                seg_x.extend([a[0], b[0], math.nan]);
                seg_y.extend([a[1], b[1], None])

            last = None
            for pt in xys:
                if last is not None and last[1] is not None and pt[1] is not None:
                    push(last, pt)
                last = pt
            if seg_x:
                fig.add_scatter(
                    x=seg_x, y=seg_y, mode="lines",
                    line=dict(color=color, width=2, dash=None),
                    showlegend=False, hoverinfo="skip",
                    legendgroup=legendgroup,  # tie to item’s legend group
                    name=f"{legendgroup} flow"
                )

        def add_flow_for(i, trans, prev_op, next_op):
            EPS_IN, EPS_OUT = 0.15, 0.15

            S_prev, E_prev = self.S[i, prev_op].value(), self.E[i, prev_op].value()
            S_T, E_T = self.S_T[i, trans].value(), self.E_T[i, trans].value()
            S_next = self.S[i, next_op].value()
            if None in (S_prev, E_prev, S_T, E_T, S_next):
                return

            y_prev = safe_cat(solid_resource_for_processing(i, prev_op))
            y_mid = safe_cat(transport_lane_map.get((i, trans), resource_for_transport(i, trans)))
            y_next = safe_cat(solid_resource_for_processing(i, next_op))

            x_prev_out = int(E_prev) - EPS_OUT
            x_T_in = int(S_T) + EPS_IN
            x_T_out = int(E_T) - EPS_OUT
            x_next_in = int(S_next) + EPS_IN

            color = item_color[str(i)]
            lg = str(i)  # legend group for this item

            if y_mid is not None:
                # ✨ Draw ONLY the connectors to and from the mid lane (no mid→mid segment)
                add_poly_segments(fig, color, [
                    (x_prev_out, y_prev),
                    (x_T_in, y_mid),
                ], legendgroup=lg)

                add_poly_segments(fig, color, [
                    (x_T_out, y_mid),
                    (x_next_in, y_next),
                ], legendgroup=lg)
            else:
                # Conveyor transport: single dashed segment prev → next
                fig.add_scatter(
                    x=[x_prev_out, x_next_in], y=[y_prev, y_next],
                    mode="lines",
                    line=dict(color=color, width=2, dash="dash"),
                    showlegend=False, hoverinfo="skip",
                    legendgroup=lg, name=f"{lg} flow"
                )

        def safe_cat(label: str):
            lbl = norm_label(label)
            if lbl in order:
                return lbl
            if lbl == "Human":
                for r in order:
                    if r.startswith("Human #"): return r
            if lbl == "Robot":
                for r in order:
                    if r.startswith("Robot #"): return r
            return None


        # ----- emit flow lines for each item -----
        for item in item_list:
            i = int(item)
            k_ops = [op for op in self.OPS_i[i] if op in (K1, K2)]
            b_ops = [op for op in self.OPS_i[i] if op in (B1, B2)]

            # KB: last K → T_KB → first B
            if k_ops and b_ops and (i, KB) in self.S_T:
                add_flow_for(i, KB, k_ops[-1], b_ops[0])

            # B12: B1 → T_B12 → B2
            if len(b_ops) >= 2 and (i, B12) in self.S_T:
                add_flow_for(i, B12, b_ops[0], b_ops[1])

            # BP: last B → T_BP → P
            if b_ops and self.z.get((i, P), 0) == 1 and (i, BP) in self.S_T:
                add_flow_for(i, BP, b_ops[-1], P)

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
                categoryorder="array", categoryarray=order, tickfont=dict(size=10),
            ),
        )
        fig.update_xaxes(range=[0, self.makespan.value()])

        # ---------- save/show ----------
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
















