import math
import json
import time
from collections import defaultdict
import cpmpy as cp
from cpmpy import SolverLookup
from utils.utility import compute_makespan_lower_bound, compute_diversification_weights


class FJTransportProblemFinal:
    def __init__(self, symmetry_breaking=False, custom_bound=True,
                 custom_hyper=False, solution_hint=False,
                 json_file='./data/default.json', config_stages=None,
                 obj_type=1, use_no_overlap=False):
        with open(json_file, "r") as f:
            self.data = json.load(f)

        self.use_no_overlap = use_no_overlap
        # Horizon is now computed dynamically in define_decision_variables
        # based on the sum of maximum durations, ensuring a safe upper bound.

        self.orderbook = self.data["orderbook"]
        self.resources = self.data["resources"]
        self.objectives = {}
        # Build agent -> actions (processing + transports)
        self.dict_agent_actions = self.build_agent_actions()
        self.symmetry_breaking = symmetry_breaking
        self.custom_bound = custom_bound

    # ------------------------------------------------------------------
    # Model wrapper
    # ------------------------------------------------------------------
    def make_model(self):
        self.define_decision_variables()
        self.define_constraints()
        if self.symmetry_breaking:
            self.define_symmetry_breaking_constraints()
        self.optimize()


    def duration_expr(self, i, op_name):
        # Transport
        if self.is_transport(op_name):
            terms = []
            for a, z_var in self.Z_tr[i][op_name].items():
                if a.startswith("skill_"):
                    # human skill pool
                    skill_token = a[len("skill_"):]  # e.g. "3"
                    skill_key = skill_token
                    # try to map back to the original key type (int or str)
                    if skill_key not in self.skill_durations:
                        try:
                            skill_key = int(skill_token)
                        except ValueError:
                            # if conversion fails, we just keep the string;
                            # if it's still not in the dict, you'll get a clear error
                            pass

                    if skill_key not in self.skill_durations:
                        raise KeyError(
                            f"Skill {skill_token} (pool id {a}) not found in skill_durations keys "
                            f"{list(self.skill_durations.keys())}"
                        )

                    base_dur = self.skill_durations[skill_key]["T"]

                elif a == "robot_pool":
                    # pooled robots
                    base_dur = self.robot_T_duration
                else:
                    # conveyors and internal WS
                    base_dur = self.dict_agent_actions[a][op_name]

                terms.append(base_dur * z_var)

            if not terms:
                raise ValueError(
                    f"No transport agent duration defined for '{op_name}' on item {i}"
                )
            return cp.sum(terms)

        # Processing (unchanged)
        terms = []
        for ws_id, z_ws in self.Z_auto_ws[i][op_name].items():
            base_dur = self.workstations_auto[ws_id]["durations"][op_name]
            terms.append(base_dur * z_ws)

        for skill, z_skill in self.Z_skill[i][op_name].items():
            base_dur = self.skill_durations[skill][op_name]
            terms.append(base_dur * z_skill)

        if not terms:
            raise ValueError(
                f"No duration terms found for processing action '{op_name}' on item {i}"
            )

        return cp.sum(terms)

    def add_collapsed_task(self, S_real, z, dur_const, tag):
        """
        Create (Sg, Dg, Eg) such that:
          if z=1: Sg=S_real, Dg=dur_const
          if z=0: Sg=0,      Dg=0
        and Eg = Sg + Dg
        No Big-M, only implications.
        """
        H = self.horizon
        Sg = cp.intvar(0, H, name=f"Sg_{tag}")
        Dg = cp.intvar(0, dur_const, name=f"Dg_{tag}")
        Eg = cp.intvar(0, H, name=f"Eg_{tag}")

        # Use multiplication for better solver propagation (If z=1 then Val else 0)
        self.model += (Sg == z * S_real)
        self.model += (Dg == z * dur_const)

        self.model += (Eg == Sg + Dg)
        return Sg, Dg, Eg

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def transport_action_name(a, b):
        return f"{a}->{b}"

    @staticmethod
    def is_transport(op_name):
        return "->" in op_name

    def extract_transport_pairs(self):
        pairs = set()
        for product in self.orderbook.values():
            seq = product["sequence"]
            n = len(seq)
            for i, t in enumerate(seq):
                if t != "T":
                    continue

                j = i - 1
                while j >= 0 and seq[j] == "T":
                    j -= 1

                k = i + 1
                while k < n and seq[k] == "T":
                    k += 1

                if j >= 0 and k < n and seq[j] != "T" and seq[k] != "T":
                    a = seq[j]
                    b = seq[k]
                    pairs.add((a, b))
        return pairs

    def get_agent_ids(self, resource_key):
        return sorted(self.resources.get(resource_key, {}).keys())

    def expand_sequence_with_transports(self, seq):
        ops = []
        n = len(seq)
        for i, t in enumerate(seq):
            if t != "T":
                ops.append(t)
            else:
                j = i - 1
                while j >= 0 and seq[j] == "T":
                    j -= 1
                k = i + 1
                while k < n and seq[k] == "T":
                    k += 1
                if j >= 0 and k < n and seq[j] != "T" and seq[k] != "T":
                    a = seq[j]
                    b = seq[k]
                    ops.append(self.transport_action_name(a, b))
                else:
                    raise ValueError(f"Malformed sequence: {seq}")
        return ops

    # ------------------------------------------------------------------
    # Agent actions (processing + transports, NO stages)
    # ------------------------------------------------------------------
    def build_agent_actions(self):
        workstations = self.resources.get("workstations", {})
        operators = self.resources.get("operators", {})
        robots = self.resources.get("mobile_robots", {})
        conveyors = self.resources.get("conveyor_belts", {})
        ws_human = self.resources.get("ws-human", {})

        transport_pairs = self.extract_transport_pairs()
        agent_actions = {}

        def add_base(group):
            for agent_id, info in group.items():
                agent_actions.setdefault(agent_id, {})
                for act, dur in info.get("durations", {}).items():
                    if act != "T":
                        agent_actions[agent_id][act] = dur

        # ws-human NOT included here (durations come from operators)
        for group in (workstations, operators, robots, conveyors):
            add_base(group)

        # tasks per WS (auto + human)
        ws_tasks = {
            ws_id: set(info.get("durations", {}).keys())
            for ws_id, info in workstations.items()
        }
        for ws_id, info in ws_human.items():
            supported = info.get("supported", {})
            ws_tasks[ws_id] = {t for t, ok in supported.items() if ok}

        # transports
        for a, b in transport_pairs:
            action = self.transport_action_name(a, b)

            # operators & robots: all transports with their 'T' duration
            for group in (operators, robots):
                for agent_id, info in group.items():
                    durations = info.get("durations", {})
                    if "T" in durations:
                        agent_actions.setdefault(agent_id, {})[action] = durations["T"]

            # conveyors: only if from_ws can do a and to_ws can do b
            for c_id, info in conveyors.items():
                durations = info.get("durations", {})
                if "T" not in durations:
                    continue
                from_ws = info.get("from")
                to_ws = info.get("to")
                if from_ws in ws_tasks and to_ws in ws_tasks:
                    if a in ws_tasks[from_ws] and b in ws_tasks[to_ws]:
                        agent_actions.setdefault(c_id, {})[action] = durations["T"]

            # internal transports: WS (auto or human) can do both a and b -> duration 0
            for ws_id, tasks in ws_tasks.items():
                if a in tasks and b in tasks:
                    agent_actions.setdefault(ws_id, {})[action] = 0

        return agent_actions

    # ------------------------------------------------------------------
    # Decision variables Z[i][op_name][*]
    # ------------------------------------------------------------------
    def define_decision_variables(self):
        self.define_Z()

        # Compute a safe horizon now that all operations and resources are defined
        if self.custom_bound:
            self.horizon = self.compute_safe_horizon()
            lb = compute_makespan_lower_bound(self)
        else:
            lb = 0
            self.horizon = int(1e6)
        self.makespan_obj = cp.intvar(lb, self.horizon, name="makespan")

        self.define_time_variables()


    def adjacent_ops_no_transport(self):
        adj = {}
        for product_name, info in self.orderbook.items():
            seq = info["sequence"]
            pairs = []
            for i in range(len(seq) - 1):
                if seq[i] != "T" and seq[i + 1] != "T":
                    pairs.append((seq[i], seq[i + 1]))
            adj[product_name] = pairs
        return adj

    def define_Z(self):
        """
        Decision variables for:
          - processing operations: choice of automatic WS and/or human WS + skill
          - transport operations: choice of transport "pool":
                * pooled robots (robot_pool)
                * conveyors / internal WS (per-agent)
                * human skills (pooled by skill)
        """

        # ------------------------------------------------------------------
        # 1) Items and their expanded operation sequences
        # ------------------------------------------------------------------
        self.items = []
        self.item_operations = {}
        self.item_product = {}

        for product_name, info in self.orderbook.items():
            seq = info["sequence"]
            vol = info["volume"]
            ops_for_product = self.expand_sequence_with_transports(seq)

            for k in range(vol):
                item_id = f"{product_name}_{k}"
                self.items.append(item_id)
                self.item_operations[item_id] = list(ops_for_product)
                self.item_product[item_id] = product_name

        # ------------------------------------------------------------------
        # 2) Resources
        # ------------------------------------------------------------------
        self.workstations_auto = self.resources.get("workstations", {})
        self.workstations_human = self.resources.get("ws-human", {})
        self.operators_res = self.resources.get("operators", {})
        self.robots = self.resources.get("mobile_robots", {})
        self.conveyors = self.resources.get("conveyor_belts", {})

        self.auto_ws_ids = sorted(self.workstations_auto.keys())
        self.human_ws_ids = sorted(self.workstations_human.keys())
        self.operator_ids = sorted(self.operators_res.keys())
        self.robot_ids = sorted(self.robots.keys())
        self.conveyor_ids = sorted(self.conveyors.keys())

        # ------------------------------------------------------------------
        # 2b) Build operator skill pools
        # ------------------------------------------------------------------
        self.skill_to_ops = defaultdict(list)
        for op_id, info in self.operators_res.items():
            skill = info.get("skill")
            if skill is None:
                continue
            self.skill_to_ops[skill].append(op_id)

        self.operator_skills = sorted(self.skill_to_ops.keys())

        self.skill_durations = {}
        for skill, op_ids in self.skill_to_ops.items():
            rep_id = op_ids[0]
            rep_durs = self.operators_res[rep_id].get("durations", {})
            for other_id in op_ids[1:]:
                other_durs = self.operators_res[other_id].get("durations", {})
                if other_durs != rep_durs:
                    raise ValueError(
                        f"Inconsistent durations for operators with skill {skill}: "
                        f"{rep_id} vs {other_id}"
                    )
            self.skill_durations[skill] = dict(rep_durs)

        # ------------------------------------------------------------------
        # 2c) Build robot pool duration (single T duration for all robots)
        # ------------------------------------------------------------------
        self.robot_T_duration = None
        if self.robot_ids:
            rep_id = self.robot_ids[0]
            rep_durs = self.robots[rep_id].get("durations", {})
            if "T" not in rep_durs:
                raise ValueError(
                    f"Robot {rep_id} has no 'T' duration defined."
                )
            base_T = rep_durs["T"]
            for other_id in self.robot_ids[1:]:
                other_durs = self.robots[other_id].get("durations", {})
                if other_durs.get("T", base_T) != base_T:
                    raise ValueError(
                        f"Inconsistent 'T' durations for robots: {rep_id} vs {other_id}"
                    )
            self.robot_T_duration = base_T

        # ------------------------------------------------------------------
        # 3) Decision vars
        # ------------------------------------------------------------------
        self.Z_auto_ws = {}
        self.Z_human_ws = {}
        self.Z_skill = {}   # skill-level processing
        self.Z_tr = {}
        self.Z_op = {}      # per-operator assignment (if use_no_overlap)
        self.Z_robot = {}   # per-robot assignment (if use_no_overlap)

        self.processing_ops = {}
        self.transport_ops = {}

        for i in self.items:
            self.Z_auto_ws[i] = {}
            self.Z_human_ws[i] = {}
            self.Z_skill[i] = {}
            self.Z_tr[i] = {}
            self.Z_op[i] = {}
            self.Z_robot[i] = {}

            self.processing_ops[i] = []
            self.transport_ops[i] = []

            ops_seq = self.item_operations[i]

            for op_name in ops_seq:
                if self.is_transport(op_name):
                    # ---------------- TRANSPORT OP ----------------
                    self.transport_ops[i].append(op_name)
                    self.Z_tr[i][op_name] = {}
                    self.Z_robot[i][op_name] = {}
                    self.Z_op[i][op_name] = {}

                    # (1) non-human, non-robot transport agents:
                    #     conveyors + internal WS remain per-agent
                    feasible_agents = [
                        agent_id
                        for agent_id, acts in self.dict_agent_actions.items()
                        if op_name in acts
                        and agent_id not in self.operator_ids
                        and agent_id not in self.robot_ids
                    ]

                    for a in feasible_agents:
                        self.Z_tr[i][op_name][a] = cp.boolvar(
                            name=f"Z_TR_{i}_{op_name}_{a}"
                        )

                    # (2) pooled robot transport, if robots can perform this op
                    if self.robot_ids:
                        robot_can_do = any(
                            op_name in self.dict_agent_actions.get(r_id, {})
                            for r_id in self.robot_ids
                        )
                        if robot_can_do:
                            self.Z_tr[i][op_name]["robot_pool"] = cp.boolvar(
                                name=f"Z_TR_{i}_{op_name}_robotpool"
                            )
                            if self.use_no_overlap:
                                for r_id in self.robot_ids:
                                    self.Z_robot[i][op_name][r_id] = cp.boolvar(
                                        name=f"Z_ROBOT_{i}_{op_name}_{r_id}"
                                    )

                    # (3) human skill pools for transport (use 'T' duration)
                    for skill, durs in self.skill_durations.items():
                        if "T" in durs:
                            pool_id = f"skill_{skill}"
                            self.Z_tr[i][op_name][pool_id] = cp.boolvar(
                                name=f"Z_TR_{i}_{op_name}_skill{skill}"
                            )
                            if self.use_no_overlap:
                                for op_id in self.skill_to_ops[skill]:
                                    self.Z_op[i][op_name][op_id] = cp.boolvar(
                                        name=f"Z_OP_{i}_{op_name}_{op_id}"
                                    )

                    if not self.Z_tr[i][op_name]:
                        raise ValueError(
                            f"No transport agent can perform action '{op_name}' for item {i}"
                        )

                else:
                    # ---------------- PROCESSING OP ----------------
                    self.processing_ops[i].append(op_name)
                    self.Z_auto_ws[i][op_name] = {}
                    self.Z_human_ws[i][op_name] = {}
                    self.Z_skill[i][op_name] = {}
                    self.Z_op[i][op_name] = {}

                    # (a) automatic workstations that can do this op
                    for ws_id, ws_info in self.workstations_auto.items():
                        if op_name in ws_info.get("durations", {}):
                            self.Z_auto_ws[i][op_name][ws_id] = cp.boolvar(
                                name=f"Z_WS_{i}_{op_name}_{ws_id}"
                            )

                    # (b) human workstations that support this op
                    for hws_id, hws_info in self.workstations_human.items():
                        supported = hws_info.get("supported", {})
                        if supported.get(op_name, False):
                            self.Z_human_ws[i][op_name][hws_id] = cp.boolvar(
                                name=f"Z_HWS_{i}_{op_name}_{hws_id}"
                            )

                    # (c) operator skills that can do this op
                    for skill in self.operator_skills:
                        if op_name in self.skill_durations[skill]:
                            self.Z_skill[i][op_name][skill] = cp.boolvar(
                                name=f"Z_SKILL_{i}_{op_name}_skill{skill}"
                            )
                            if self.use_no_overlap:
                                for op_id in self.skill_to_ops[skill]:
                                    self.Z_op[i][op_name][op_id] = cp.boolvar(
                                        name=f"Z_OP_{i}_{op_name}_{op_id}"
                                    )

                    # --------- feasibility checks ----------
                    auto_possible = bool(self.Z_auto_ws[i][op_name])

                    human_supported_somewhere = any(
                        hws_info.get("supported", {}).get(op_name, False)
                        for hws_info in self.workstations_human.values()
                    )
                    human_possible = human_supported_somewhere and bool(self.Z_skill[i][op_name])

                    if not auto_possible and not human_possible:
                        raise ValueError(
                            f"Processing action '{op_name}' for item {i} cannot be done: "
                            f"no automatic workstation and no human+skill option."
                        )

                    if human_supported_somewhere and not self.Z_skill[i][op_name]:
                        raise ValueError(
                            f"A human workstation supports '{op_name}' but no operator skill "
                            f"has a duration for it (item {i})."
                        )

        # ------------------------------------------------------------------
        # 4) Workstation usage variables
        # ------------------------------------------------------------------
        # 4) Workstation and Pool usage variables
        # ------------------------------------------------------------------
        self.W_auto_used = {
            ws_id: cp.boolvar(name=f"W_auto_{ws_id}")
            for ws_id in self.auto_ws_ids
        }
        self.W_human_used = {
            hws_id: cp.boolvar(name=f"W_human_{hws_id}")
            for hws_id in self.human_ws_ids
        }
        
        # Flexible Capacity: Number of active units per skill pool
        self.O_active_count = {
            skill: cp.intvar(0, len(self.skill_to_ops[skill]), name=f"O_active_{skill}")
            for skill in self.operator_skills
        }
        
        # Flexible Capacity: Number of active robots
        self.R_active_count = cp.intvar(0, len(self.robot_ids), name="R_active_robots")

        if self.use_no_overlap:
            self.O_active = {
                op_id: cp.boolvar(name=f"op_used_{op_id}")
                for op_id in self.operator_ids
            }
            self.R_active = {
                robot_id: cp.boolvar(name=f"robot_used_{robot_id}")
                for robot_id in self.robot_ids
            }

    def define_time_variables(self):
        if not hasattr(self, "horizon"):
            self.horizon = int(1e3)

        S = {}
        D = {}
        for i in self.items:
            S[i] = {}
            D[i] = {}
            for op_name in self.item_operations[i]:
                S[i][op_name] = cp.intvar(0, self.horizon, name=f"S_{i}_{op_name}")

        self.S = S
        self.D = D

    # ------------------------------------------------------------------
    # Constraints
    # ------------------------------------------------------------------
    def define_constraints(self):
        self.model = cp.Model()
        self.op_one_agent()                             #ok
        self.add_same_agent_no_transport_constraints()  #ok
        self.add_precedence_constraints()
        self.add_conveyor_consistency_constraints()             #ok
        self.add_workstation_internal_transport_constraints()   #ok
        self.add_no_overlap_constraints()                       #ok
        self.add_workstation_usage_constraints()
        #
        self.add_item_time_window_constraints(ub=self.horizon)
        self.add_precedence_tightening_constraints(max_jump=4)

    def op_one_agent(self):
        for i in self.items:
            for op in self.item_operations[i]:

                if self.is_transport(op):
                    agents = list(self.Z_tr[i][op].keys())
                    self.model += cp.sum(self.Z_tr[i][op][a] for a in agents) == 1
                    
                    if self.use_no_overlap:
                        # link robot pool to specific robots
                        if "robot_pool" in self.Z_tr[i][op]:
                            z_pool = self.Z_tr[i][op]["robot_pool"]
                            self.model += (z_pool == cp.sum(list(self.Z_robot[i][op].values())))
                        
                        # link skill pool to specific operators
                        for skill in self.operator_skills:
                            pool_id = f"skill_{skill}"
                            if pool_id in self.Z_tr[i][op]:
                                z_pool = self.Z_tr[i][op][pool_id]
                                ops_of_skill = [self.Z_op[i][op][oid] for oid in self.skill_to_ops[skill] if oid in self.Z_op[i][op]]
                                if ops_of_skill:
                                    self.model += (z_pool == cp.sum(ops_of_skill))
                    continue

                auto_ws_ids = list(self.Z_auto_ws[i][op].keys())
                human_ws_ids = list(self.Z_human_ws[i][op].keys())
                skills = list(self.Z_skill[i][op].keys())

                sum_auto = cp.sum(self.Z_auto_ws[i][op][ws] for ws in auto_ws_ids) if auto_ws_ids else 0
                sum_human_ws = cp.sum(self.Z_human_ws[i][op][hw] for hw in human_ws_ids) if human_ws_ids else 0
                sum_skill = cp.sum(self.Z_skill[i][op][s] for s in skills) if skills else 0

                # exactly one workstation (auto OR human)
                self.model += (sum_auto + sum_human_ws == 1)

                # if a human WS is chosen, exactly one skill; if no human WS, no skill
                self.model += (sum_human_ws == sum_skill)

                if self.use_no_overlap:
                    for skill in skills:
                        z_skill = self.Z_skill[i][op][skill]
                        # Only sum operators belonging to this specific skill
                        ops_of_skill = [self.Z_op[i][op][oid] for oid in self.skill_to_ops[skill] if oid in self.Z_op[i][op]]
                        if ops_of_skill:
                            self.model += (z_skill == cp.sum(ops_of_skill))

        if self.use_no_overlap:
            # Link O_active and R_active
            for op_id in self.operator_ids:
                tasks = []
                for i in self.items:
                    for op in self.item_operations[i]:
                        if op_id in self.Z_op[i].get(op, {}):
                            tasks.append(self.Z_op[i][op][op_id])
                if tasks:
                    for t in tasks:
                        self.model += t.implies(self.O_active[op_id])
                else:
                    self.model += (self.O_active[op_id] == 0)

            for robot_id in self.robot_ids:
                tasks = []
                for i in self.items:
                    for op in self.item_operations[i]:
                        if robot_id in self.Z_robot[i].get(op, {}):
                            tasks.append(self.Z_robot[i][op][robot_id])
                if tasks:
                    for t in tasks:
                        self.model += t.implies(self.R_active[robot_id])
                else:
                    self.model += (self.R_active[robot_id] == 0)

            # Link active counts
            for skill in self.operator_skills:
                self.model += (self.O_active_count[skill] == cp.sum([self.O_active[oid] for oid in self.skill_to_ops[skill]]))
                # Symmetry breaking: use operator k before k+1
                op_ids = self.skill_to_ops[skill]
                for k in range(len(op_ids) - 1):
                    self.model += (self.O_active[op_ids[k+1]].implies(self.O_active[op_ids[k]]))

            self.model += (self.R_active_count == cp.sum(list(self.R_active.values())))
            # Symmetry breaking for robots
            for k in range(len(self.robot_ids) - 1):
                self.model += (self.R_active[self.robot_ids[k+1]].implies(self.R_active[self.robot_ids[k]]))

    def add_same_agent_no_transport_constraints(self):
        adj = self.adjacent_ops_no_transport()

        for i in self.items:
            product_name = self.item_product[i]
            for op1, op2 in adj.get(product_name, []):

                if self.is_transport(op1) or self.is_transport(op2):
                    continue

                # auto WS
                all_auto_ws = set(self.Z_auto_ws[i].get(op1, {}).keys()) | set(self.Z_auto_ws[i].get(op2, {}).keys())
                for ws in all_auto_ws:
                    z1 = self.Z_auto_ws[i][op1].get(ws, 0)
                    z2 = self.Z_auto_ws[i][op2].get(ws, 0)
                    self.model += (z1 == z2)

                # human WS
                all_hws = set(self.Z_human_ws[i].get(op1, {}).keys()) | set(self.Z_human_ws[i].get(op2, {}).keys())
                for hws in all_hws:
                    z1 = self.Z_human_ws[i][op1].get(hws, 0)
                    z2 = self.Z_human_ws[i][op2].get(hws, 0)
                    self.model += (z1 == z2)

                # skills (must also be the same skill if it's a human WS)
                all_skills = set(self.Z_skill[i].get(op1, {}).keys()) | set(self.Z_skill[i].get(op2, {}).keys())
                for s in all_skills:
                    z1 = self.Z_skill[i][op1].get(s, 0)
                    z2 = self.Z_skill[i][op2].get(s, 0)
                    self.model += (z1 == z2)
                    
                    if self.use_no_overlap:
                        for op_id in self.skill_to_ops[s]:
                            zo1 = self.Z_op[i].get(op1, {}).get(op_id, 0)
                            zo2 = self.Z_op[i].get(op2, {}).get(op_id, 0)
                            self.model += (zo1 == zo2)

                common_auto = set(self.Z_auto_ws[i].get(op1, {}).keys()) & set(self.Z_auto_ws[i].get(op2, {}).keys())
                common_hws = set(self.Z_human_ws[i].get(op1, {}).keys()) & set(self.Z_human_ws[i].get(op2, {}).keys())

                if not common_auto and not common_hws:
                    raise ValueError(
                        f"No common workstation (auto or human) can perform both {op1} and {op2} "
                        f"for item {i}, but there is no 'T' between them in the route."
                    )

    def add_precedence_constraints(self):
        for i in self.items:
            ops = self.item_operations[i]
            for prev_op, next_op in zip(ops, ops[1:]):
                self.model += (
                    self.S[i][next_op] >= self.S[i][prev_op] + self.duration_expr(i, prev_op)
                )

    def add_conveyor_consistency_constraints(self):
        conveyors = self.resources.get("conveyor_belts", {})

        for i in self.items:
            ops = self.item_operations[i]

            for idx in range(1, len(ops) - 1):
                t_op = ops[idx]
                prev_op = ops[idx - 1]
                next_op = ops[idx + 1]

                if not self.is_transport(t_op):
                    continue

                for c_id, c_info in conveyors.items():
                    if t_op not in self.dict_agent_actions.get(c_id, {}):
                        continue

                    z_t_c = self.Z_tr[i][t_op].get(c_id, None)
                    if z_t_c is None:
                        continue

                    from_ws = c_info.get("from")
                    to_ws = c_info.get("to")

                    z_prev_from = None
                    if from_ws in self.Z_auto_ws[i].get(prev_op, {}):
                        z_prev_from = self.Z_auto_ws[i][prev_op][from_ws]
                    elif from_ws in self.Z_human_ws[i].get(prev_op, {}):
                        z_prev_from = self.Z_human_ws[i][prev_op][from_ws]

                    z_next_to = None
                    if to_ws in self.Z_auto_ws[i].get(next_op, {}):
                        z_next_to = self.Z_auto_ws[i][next_op][to_ws]
                    elif to_ws in self.Z_human_ws[i].get(next_op, {}):
                        z_next_to = self.Z_human_ws[i][next_op][to_ws]

                    if z_prev_from is None or z_next_to is None:
                        continue

                    self.model += z_t_c == (z_prev_from & z_next_to)

    def add_workstation_internal_transport_constraints(self):
        workstations_auto = self.resources.get("workstations", {})
        workstations_human = self.resources.get("ws-human", {})

        for i in self.items:
            ops = self.item_operations[i]

            for idx in range(1, len(ops) - 1):
                t_op = ops[idx]
                if not self.is_transport(t_op):
                    continue

                prev_op = ops[idx - 1]
                next_op = ops[idx + 1]

                # auto WS
                for ws_id in workstations_auto.keys():
                    if t_op not in self.dict_agent_actions.get(ws_id, {}):
                        continue

                    z_t_ws = self.Z_tr[i][t_op].get(ws_id)
                    if z_t_ws is None:
                        continue

                    z_prev_ws = self.Z_auto_ws[i].get(prev_op, {}).get(ws_id)
                    z_next_ws = self.Z_auto_ws[i].get(next_op, {}).get(ws_id)
                    if z_prev_ws is None or z_next_ws is None:
                        continue

                    self.model += z_t_ws == (z_prev_ws & z_next_ws)

                # human WS
                for hws_id in workstations_human.keys():
                    if t_op not in self.dict_agent_actions.get(hws_id, {}):
                        continue

                    z_t_hws = self.Z_tr[i][t_op].get(hws_id)
                    if z_t_hws is None:
                        continue

                    z_prev_hws = self.Z_human_ws[i].get(prev_op, {}).get(hws_id)
                    z_next_hws = self.Z_human_ws[i].get(next_op, {}).get(hws_id)
                    if z_prev_hws is None or z_next_hws is None:
                        continue

                    self.model += z_t_hws == (z_prev_hws & z_next_hws)

    def add_no_overlap_constraints(self):
        """
        Capacity constraints:

          - Automatic workstations: at most one processing op at a time.
          - Human workstations: at most one processing op at a time.
          - Human operators pooled by skill: cumulative (processing + transports).
          - Mobile robots pooled: cumulative for transports.
          - Conveyors: at most one transport at a ttagime.
        """

        # ------------------------------------------------------------
        # 1) Automatic workstations (NO int products)
        # ------------------------------------------------------------
        for ws_id in self.workstations_auto.keys():
            starts, durs, ends = [], [], []

            for i in self.items:
                for op in self.processing_ops[i]:
                    z_ws = self.Z_auto_ws[i][op].get(ws_id)
                    if z_ws is None:
                        continue

                    # constant duration for (ws_id, op)
                    dur_const = self.workstations_auto[ws_id]["durations"][op]
                    tag = f"{i}_{op}_auto_{ws_id}"

                    Sg, Dg, Eg = self.add_collapsed_task(
                        S_real=self.S[i][op],
                        z=z_ws,
                        dur_const=dur_const,
                        tag=tag
                    )

                    starts.append(Sg)
                    durs.append(Dg)
                    ends.append(Eg)

            if len(starts) > 1:
                self.model += cp.NoOverlap(starts, durs, ends)

        # ------------------------------------------------------------
        # 2) Human workstations (NO int products)
        #    Use ws×skill tasks with y = z_hws AND z_skill
        # ------------------------------------------------------------
        for hws_id in self.workstations_human.keys():
            starts, durs, ends = [], [], []

            for i in self.items:
                for op in self.processing_ops[i]:
                    z_hws = self.Z_human_ws[i][op].get(hws_id)
                    if z_hws is None:
                        continue

                    # for each skill that can do this op, make an optional task on this hws
                    for skill, z_skill in self.Z_skill[i][op].items():
                        # duration is constant for (skill, op)
                        dur_const = self.skill_durations[skill][op]

                        # # y = z_hws AND z_skill
                        # y = cp.boolvar(name=f"Y_{i}_{op}_hws{hws_id}_skill{skill}")
                        # # AND linearization:
                        # self.model += (y == z_hws)

                        tag = f"{i}_{op}_human_{hws_id}_skill{skill}"

                        Sg, Dg, Eg = self.add_collapsed_task(
                            S_real=self.S[i][op],
                            z=z_hws,
                            dur_const=dur_const,
                            tag=tag
                        )

                        starts.append(Sg)
                        durs.append(Dg)
                        ends.append(Eg)

            if len(starts) > 1:
                self.model += cp.NoOverlap(starts, durs, ends)

        # ------------------------------------------------------------
        # 3) Human operators pooled by skill (unchanged)
        # ------------------------------------------------------------
        for skill in self.operator_skills:
            if not self.skill_to_ops[skill]:
                continue
            
            if not self.use_no_overlap:
                capacity = self.O_active_count[skill]
                starts, durs, demands, ends = [], [], [], []

                # 3a) processing using this skill
                for i in self.items:
                    for op in self.processing_ops[i]:
                        z = self.Z_skill[i][op].get(skill)
                        if z is None:
                            continue
                        dur_const = self.skill_durations[skill][op]
                        Sg, Dg, Eg = self.add_collapsed_task(self.S[i][op], z, dur_const,
                                                             f"{i}_{op}_skill{skill}")
                        starts.append(self.S[i][op])
                        durs.append(dur_const)
                        ends.append(self.S[i][op] + dur_const)
                        demands.append(z)

                # 3b) transport tasks using this skill pool
                pool_id = f"skill_{skill}"
                if "T" in self.skill_durations[skill]:
                    dur_T = self.skill_durations[skill]["T"]
                    for i in self.items:
                        for t_op in self.transport_ops[i]:
                            z = self.Z_tr[i][t_op].get(pool_id)
                            if z is None:
                                continue
                            # Sg, Dg, Eg = self.add_collapsed_task(self.S[i][t_op], z, dur_T,
                            #                                      f"{i}_{t_op}_skill{skill}")
                            starts.append(self.S[i][t_op])
                            durs.append(dur_T)
                            ends.append(self.S[i][t_op] + dur_T)
                            demands.append(z)

                if starts:
                    self.model += cp.Cumulative(starts, durs, ends, demands, capacity)
            else:
                # Use NoOverlap for each operator of this skill
                for op_id in self.skill_to_ops[skill]:
                    starts, durs, ends = [], [], []
                    
                    # 3a) processing
                    for i in self.items:
                        for op in self.processing_ops[i]:
                            z = self.Z_op[i][op].get(op_id)
                            if z is None:
                                continue
                            dur_const = self.skill_durations[skill][op]
                            Sg, Dg, Eg = self.add_collapsed_task(self.S[i][op], z, dur_const,
                                                                 f"{i}_{op}_op{op_id}")
                            starts.append(Sg)
                            durs.append(Dg)
                            ends.append(Eg)

                    # 3b) transport
                    if "T" in self.skill_durations[skill]:
                        dur_T = self.skill_durations[skill]["T"]
                        for i in self.items:
                            for t_op in self.transport_ops[i]:
                                z = self.Z_op[i][t_op].get(op_id)
                                if z is None:
                                    continue
                                Sg, Dg, Eg = self.add_collapsed_task(self.S[i][t_op], z, dur_T,
                                                                     f"{i}_{t_op}_op{op_id}")
                                starts.append(Sg)
                                durs.append(Dg)
                                ends.append(Eg)

                    if len(starts) > 1:
                        self.model += cp.NoOverlap(starts, durs, ends)

        # ------------------------------------------------------------
        # 4) Mobile robots pooled (unchanged)
        # ------------------------------------------------------------
        if self.robot_ids:
            if not self.use_no_overlap:
                capacity = self.R_active_count
                starts, durs, ends, demands = [], [], [], []
                dur_T = self.robot_T_duration
                for i in self.items:
                    for t_op in self.transport_ops[i]:
                        z = self.Z_tr[i][t_op].get("robot_pool")
                        if z is None:
                            continue
                        # Sg, Dg, Eg = self.add_collapsed_task(self.S[i][t_op], z, dur_T,
                        #                                      f"{i}_{t_op}_robotpool")
                        starts.append(self.S[i][t_op])
                        durs.append(dur_T)
                        ends.append(self.S[i][t_op]+dur_T)
                        demands.append(z)

                if starts:
                    self.model += cp.Cumulative(starts, durs, ends, demands, capacity)
            else:
                # Use NoOverlap for each robot
                dur_T = self.robot_T_duration
                for robot_id in self.robot_ids:
                    starts, durs, ends = [], [], []
                    for i in self.items:
                        for t_op in self.transport_ops[i]:
                            z = self.Z_robot[i][t_op].get(robot_id)
                            if z is None:
                                continue
                            Sg, Dg, Eg = self.add_collapsed_task(self.S[i][t_op], z, dur_T,
                                                                 f"{i}_{t_op}_robot{robot_id}")
                            starts.append(Sg)
                            durs.append(Dg)
                            ends.append(Eg)

                    if len(starts) > 1:
                        self.model += cp.NoOverlap(starts, durs, ends)

        # ------------------------------------------------------------
        # 5) Conveyors (unchanged, you already did tagit right)
        # ------------------------------------------------------------
        for c_id in self.conveyors.keys():
            starts, durs, ends = [], [], []

            for i in self.items:
                for t_op in self.transport_ops[i]:
                    z_tr_c = self.Z_tr[i][t_op].get(c_id)
                    if z_tr_c is None:
                        continue

                    S = self.S[i][t_op]
                    dur_c = self.dict_agent_actions[c_id][t_op]

                    tag = f"{i}_{t_op}_{c_id}"
                    Sg, Dg, Eg = self.add_collapsed_task(S_real=S, z=z_tr_c,
                                                         dur_const=dur_c, tag=tag)
                    starts.append(Sg)
                    durs.append(Dg)
                    ends.append(Eg)

            if len(starts) > 1:
                self.model += cp.NoOverlap(starts, durs, ends)

    def add_workstation_usage_constraints(self):
        """
        Links workstation usage variables (W_*) to task assignment variables (Z_*).
        A workstation is marked as 'used' if any task is assigned to it.
        """
        # Link W_auto_used to Z_auto_ws
        for i in self.items:
            for op, ws_vars in self.Z_auto_ws[i].items():
                for ws_id, z_var in ws_vars.items():
                    self.model += z_var.implies(self.W_auto_used[ws_id])

        # Link W_human_used to Z_human_ws
        for i in self.items:
            for op, hws_vars in self.Z_human_ws[i].items():
                for hws_id, z_var in hws_vars.items():
                    self.model += z_var.implies(self.W_human_used[hws_id])

    # ------------------------------------------------------------------
    # Objective
    # ------------------------------------------------------------------
    def optimize(self):
        """
        Defines the objective function expressions for the model.
        The user is expected to call model.minimize() on one of these objectives
        after the model is created.
        """
        # --- Objective Term 1: Makespan ---
        for i in self.items:
            last_op = self.item_operations[i][-1]
            self.model += (self.makespan_obj >= self.S[i][last_op] + self.duration_expr(i, last_op))
        self.objectives['makespan'] = self.makespan_obj

        # --- Objective Term 2: Number of workstations used ---
        num_auto_ws_used = cp.sum(list(self.W_auto_used.values()))
        num_human_ws_used = cp.sum(list(self.W_human_used.values()))
        self.workstation_obj = num_auto_ws_used + num_human_ws_used
        self.objectives['workstations'] = self.workstation_obj

        # --- Objective Term 3: Number of employees used ---
        self.employee_obj = cp.sum(list(self.O_active_count.values()))
        self.objectives['employees'] = self.employee_obj

        # --- Objective Term 4: Number of robots used ---
        self.robot_obj = self.R_active_count
        self.objectives['robots'] = self.robot_obj


        # --- Objective Term 5: Cumulative Employee Work Time ---
        employee_time_terms = []
        # Processing tasks by humans
        for i in self.items:
            for op in self.processing_ops[i]:
                for skill, z_var in self.Z_skill[i][op].items():
                    dur = self.skill_durations[skill][op]
                    employee_time_terms.append(dur * z_var)

        # Transport tasks by humans
        for i in self.items:
            for t_op in self.transport_ops[i]:
                for pool_id, z_var in self.Z_tr[i][t_op].items():
                    if pool_id.startswith("skill_"):
                        skill_token = pool_id[len("skill_"):]
                        # Handle potential type mismatch for skill key (int vs str)
                        skill_key = skill_token
                        if skill_key not in self.skill_durations:
                            try:
                                skill_key = int(skill_token)
                            except ValueError:
                                pass  # Keep as string if conversion fails

                        if "T" in self.skill_durations.get(skill_key, {}):
                            dur = self.skill_durations[skill_key]["T"]
                            employee_time_terms.append(dur * z_var)
        
        self.employee_work_time_obj = cp.sum(employee_time_terms)
        self.objectives['employee_time'] = self.employee_work_time_obj


    # ------------------------------------------------------------------
    # Symmetry breaking
    # ------------------------------------------------------------------
    def group_items_by_type(self):
        items_by_type = defaultdict(list)
        for i in self.items:
            t = self.item_product[i]
            items_by_type[t].append(i)
        return items_by_type

    def add_lex_order_items(self, items_group):
        """
        For each product type, order identical items by their completion times.
        """
        for t, items in items_group.items():
            if len(items) <= 1:
                continue

            items_sorted = sorted(items)
            k = 2  # first k operations (choose what you want)

            matrix = []
            for i in items_sorted:
                # first k operations of each item (in the intended order)
                ops_i = self.item_operations[i][:k]
                vec_i = [self.S[i][op] for op in ops_i]
                matrix.append(vec_i)

            # lexicographic symmetry breaking: matrix[0] <=_lex matrix[1] <=_lex ...
            self.model += cp.LexChainLessEq(matrix)

    def add_lex_order_operators(self, operator_groups):
        """
        Enforce lexicographical order on task assignments for interchangeable operators.
        If O1 and O2 have the same skill, we prefer assigning 'earlier' tasks to O1.
        """
        if not self.use_no_overlap:
            return

        for group in operator_groups:
            if len(group) <= 1:
                continue
            
            # Find all tasks that can be performed by this skill group
            op_id_rep = group[0]
            skill = self.operators_res[op_id_rep].get("skill")
            
            relevant_tasks = []
            for i in self.items:
                # Processing ops
                for op in self.processing_ops[i]:
                    if skill in self.Z_skill[i][op]:
                        relevant_tasks.append((i, op))
                # Transport ops
                for t_op in self.transport_ops[i]:
                    pool_id = f"skill_{skill}"
                    if pool_id in self.Z_tr[i][t_op]:
                        relevant_tasks.append((i, t_op))
            
            if not relevant_tasks:
                continue
            
            # Sort tasks in a deterministic way: by item then by sequence index
            relevant_tasks.sort()
            
            matrix = []
            for op_id in group:
                vec = [self.Z_op[i][op][op_id] for i, op in relevant_tasks]
                matrix.append(vec)
            
            # matrix[0] >=_lex matrix[1] >=_lex ...
            # which is equivalent to matrix[k+1] <=_lex matrix[k]
            # Since CPMpy only has LexChainLessEq, we pass matrix in reverse order
            self.model += cp.LexChainLessEq(matrix[::-1])

    def add_lex_order_robots(self, robot_groups):
        """
        Enforce lexicographical order on task assignments for identical robots.
        """
        if not self.use_no_overlap:
            return
        
        for group in robot_groups:
            if len(group) <= 1:
                continue
            
            relevant_tasks = []
            for i in self.items:
                for t_op in self.transport_ops[i]:
                    if "robot_pool" in self.Z_tr[i][t_op]:
                        relevant_tasks.append((i, t_op))
            
            if not relevant_tasks:
                continue
            
            relevant_tasks.sort()
            
            matrix = []
            for r_id in group:
                vec = [self.Z_robot[i][t_op][r_id] for i, t_op in relevant_tasks]
                matrix.append(vec)
            
            self.model += cp.LexChainLessEq(matrix[::-1])

    def define_symmetry_breaking_constraints(self):
        # 1) Items: order among identical product items
        items_group = self.group_items_by_type()
        self.add_lex_order_items(items_group)

        # 2) Robots: activation order and lex order for identical robots
        # if self.use_no_overlap:
        #     robot_groups = self.group_robots_all_together()
        #     for group in robot_groups:
        #         for r1, r2 in zip(group[:-1], group[1:]):
        #             self.model += (self.R_active[r1] >= self.R_active[r2])
        #     self.add_lex_order_robots(robot_groups)
        #
        # # 3) Operators: activation order and lex order for interchangeable operators
        # if self.use_no_overlap:
        #     operator_groups = self.group_operators_by_skills()
        #     for group in operator_groups:
        #         for o1, o2 in zip(group[:-1], group[1:]):
        #             self.model += (self.O_active[o1] >= self.O_active[o2])
        #     self.add_lex_order_operators(operator_groups)



    def add_monotone_usage_in_groups(self, groups):
        """
        For each group [a0, a1, ..., ak], enforce:
            total_tasks(a0) >= total_tasks(a1) >= ... >= total_tasks(ak)

        where total_tasks(a) is the sum of all assignment variables involving agent a:

          - If a is a human workstation:
                * processing: Z_human_ws[i][op][a]
                * internal transports: Z_tr[i][t_op][a] (if allowed)
          - If a is a robot:
                * transports: Z_tr[i][t_op][a]
          - (Can be extended similarly for auto WS / conveyors.)

        This keeps lower-index agents at least as "busy" as higher ones within
        each group, which breaks symmetries among interchangeable agents.
        """

        for group in groups:
            if len(group) <= 1:
                continue

            totals = {}

            for a in group:
                terms = []

                is_human_ws = a in self.workstations_human
                is_robot = a in self.robots
                is_auto_ws = a in self.workstations_auto
                is_conveyor = a in self.conveyors

                for i in self.items:
                    # --- processing where 'a' is a workstation ---
                    if is_human_ws:
                        for op_name in self.processing_ops[i]:
                            z = self.Z_human_ws[i][op_name].get(a)
                            if z is not None:
                                terms.append(z)

                    if is_auto_ws:
                        for op_name in self.processing_ops[i]:
                            z = self.Z_auto_ws[i][op_name].get(a)
                            if z is not None:
                                terms.append(z)

                    # # --- transports where 'a' is a transport agent ---
                    # for t_op in self.transport_ops[i]:
                    #     z_tr = self.Z_tr[i][t_op].get(a)
                    #     if z_tr is not None:
                    #         terms.append(z_tr)

                if terms:
                    totals[a] = cp.sum(terms)

            # Chain monotonicity within the group: a0 >= a1 >= ... >= ak
            for a1, a2 in zip(group[:-1], group[1:]):
                if a1 in totals and a2 in totals:
                    self.model += (totals[a1] >= totals[a2])

    def group_human_workstations_by_connectivity(self):
        """
        Group human workstations that are interchangeable.

        Two human workstations h1, h2 are considered interchangeable if:
          - they support exactly the same set of tasks, and
          - they have the same conveyor connectivity pattern:
                * same set of 'from' workstations (incoming conveyors)
                * same set of 'to' workstations (outgoing conveyors)
        """
        from collections import defaultdict

        ws_human = self.resources.get("ws-human", {})
        conveyors = self.resources.get("conveyor_belts", {})

        sig_to_ws = defaultdict(list)

        for hws_id, info in ws_human.items():
            # supported tasks
            supported = info.get("supported", {})
            supported_tasks = tuple(sorted(
                t for t, ok in supported.items() if ok
            ))

            # conveyor connectivity
            incoming_from = set()
            outgoing_to = set()

            for c_id, c_info in conveyors.items():
                frm = c_info.get("from")
                to = c_info.get("to")

                if to == hws_id and frm is not None:
                    incoming_from.add(frm)
                if frm == hws_id and to is not None:
                    outgoing_to.add(to)

            incoming_sig = tuple(sorted(incoming_from))
            outgoing_sig = tuple(sorted(outgoing_to))

            signature = (supported_tasks, incoming_sig, outgoing_sig)
            sig_to_ws[signature].append(hws_id)

        return [
            sorted(group)
            for group in sig_to_ws.values()
            if len(group) > 1
        ]

    def group_operators_by_skills(self):
        """
        Group operators that are interchangeable because they have the exact same skill.
        """
        from collections import defaultdict

        skill_sets = defaultdict(list)
        for op_id, info in self.operators_res.items():
            skill = info.get("skill")
            skill_sets[skill].append(op_id)

        # Return groups of size > 1, sorted to be deterministic.
        return [
            sorted(group)
            for group in skill_sets.values()
            if len(group) > 1
        ]

    def group_robots_all_together(self):
        if len(self.robot_ids) > 1:
            return [sorted(self.robot_ids)]
        else:
            return []


    def min_duration_const(self, i, op_name):
        # Transport
        if self.is_transport(op_name):
            # look at all available choices already created
            choices = self.Z_tr[i][op_name]
            durs = []
            for a in choices:
                if a.startswith("skill_"):
                    skill_token = a[len("skill_"):]
                    skill_key = skill_token
                    if skill_key not in self.skill_durations:
                        try:
                            skill_key = int(skill_token)
                        except ValueError:
                            pass
                    durs.append(self.skill_durations[skill_key]["T"])
                elif a == "robot_pool":
                    durs.append(self.robot_T_duration)
                else:
                    durs.append(self.dict_agent_actions[a][op_name])
            return min(durs)

        # Processing
        durs = []
        # auto WS options
        for ws_id in self.Z_auto_ws[i][op_name]:
            durs.append(self.workstations_auto[ws_id]["durations"][op_name])
        # human skill options (only if human possible)
        for skill in self.Z_skill[i][op_name]:
            durs.append(self.skill_durations[skill][op_name])
        return min(durs)

    def max_duration_const(self, i, op_name):
        # Transport
        if self.is_transport(op_name):
            choices = self.Z_tr[i][op_name]
            durs = []
            for a in choices:
                if a.startswith("skill_"):
                    skill_token = a[len("skill_"):]
                    skill_key = skill_token
                    if skill_key not in self.skill_durations:
                        try:
                            skill_key = int(skill_token)
                        except ValueError:
                            pass
                    durs.append(self.skill_durations[skill_key]["T"])
                elif a == "robot_pool":
                    durs.append(self.robot_T_duration)
                else:
                    durs.append(self.dict_agent_actions[a][op_name])
            return max(durs) if durs else 0

        # Processing
        durs = []
        for ws_id in self.Z_auto_ws[i][op_name]:
            durs.append(self.workstations_auto[ws_id]["durations"][op_name])
        for skill in self.Z_skill[i][op_name]:
            durs.append(self.skill_durations[skill][op_name])
        return max(durs) if durs else 0

    def compute_safe_horizon(self):
        """
        Computes a safe upper bound for the schedule horizon by summing the
        maximum possible duration for every operation of every item.
        """
        total_duration = 0
        for i in self.items:
            for op_name in self.item_operations[i]:
                total_duration += self.max_duration_const(i, op_name)
        return total_duration

    def add_item_time_window_constraints(self, ub=None):
        """
        Tighten domains of S[i][op] using:
          ES(op_k) = sum_{t<k} min_dur(op_t)
          LS(op_k) = UB - sum_{t>=k} min_dur(op_t)
        Implemented as linear constraints (no Big-M).
        """
        UB = self.horizon if ub is None else int(ub)

        for i in self.items:
            ops = self.item_operations[i]
            n = len(ops)

            # min duration for each op in the route
            mind = [self.min_duration_const(i, op) for op in ops]

            # earliest starts (prefix sums)
            ES = [0] * n
            for k in range(1, n):
                ES[k] = ES[k - 1] + mind[k - 1]

            # latest starts (backward, using UB)
            # LS[k] = UB - sum_{t=k..n-1} mind[t]
            LS = [0] * n
            suffix = 0
            for k in range(n - 1, -1, -1):
                suffix += mind[k]
                LS[k] = UB - suffix

            # add constraints
            for k, op in enumerate(ops):
                self.model += (self.S[i][op] >= ES[k])
                self.model += (self.S[i][op] <= LS[k])

    def add_precedence_tightening_constraints(self, max_jump=4):
        """
        Add redundant precedences between non-adjacent operations in each item's route:

            S[i][op_k] >= S[i][op_j] + sum_{t=j..k-1} min_dur(i, op_t)

        We only add jumps up to length max_jump to avoid O(n^2) blow-up.

        max_jump=4 means we add constraints for (j -> j+2), (j -> j+3), (j -> j+4).
        """
        for i in self.items:
            ops = self.item_operations[i]
            n = len(ops)
            if n <= 2:
                continue

            # min duration per op (constant LB)
            mind = [self.min_duration_const(i, op) for op in ops]

            # prefix sums for fast segment sums
            pref = [0]
            for d in mind:
                pref.append(pref[-1] + d)

            # add jump constraints
            for j in range(n):
                # k at least j+2 (non-adjacent)
                k_max = min(n - 1, j + max_jump)
                for k in range(j + 2, k_max + 1):
                    # sum of mind[j..k-1] = pref[k] - pref[j]
                    lb = pref[k] - pref[j]
                    self.model += (self.S[i][ops[k]] >= self.S[i][ops[j]] + lb)

    def get_all_variables(self):
        """
        Collects all CPMpy variables defined in the class.
        Returns only decision variables, excluding expressions.
        """
        vars_list = []
        for i in self.items:
            # Z variables
            for op in self.Z_auto_ws[i]:
                vars_list.extend(self.Z_auto_ws[i][op].values())
            for op in self.Z_human_ws[i]:
                vars_list.extend(self.Z_human_ws[i][op].values())
            for op in self.Z_skill[i]:
                vars_list.extend(self.Z_skill[i][op].values())
            for op in self.Z_tr[i]:
                vars_list.extend(self.Z_tr[i][op].values())
            # S variables
            vars_list.extend(self.S[i].values())

        vars_list.extend(self.W_auto_used.values())
        vars_list.extend(self.W_human_used.values())
        vars_list.extend(self.O_active_count.values())
        vars_list.append(self.R_active_count)
        if self.use_no_overlap:
            vars_list.extend(self.O_active.values())
            vars_list.extend(self.R_active.values())
        
        # Only add makespan_obj if it's an actual variable
        vars_list.append(self.makespan_obj)

        # Filter duplicates and return only actual variables (not expressions)
        unique_vars = []
        seen = set()
        for v in vars_list:
            if isinstance(v, cp.variables._NumVarImpl) and v not in seen:
                unique_vars.append(v)
                seen.add(v)
        return unique_vars

    def get_variable_values(self):
        """
        Returns a dictionary mapping CPMpy variables to their current values.
        """
        vars_list = self.get_all_variables()
        return {v: v.value() for v in vars_list if v.value() is not None}

    def solve(self, objectives=None, image=None, disjunctive=False, iter=None, timeout=360, solver='ortools',
              nadir_points=None, diversification='base', trade_offs=None, c_ucb=2, solution_hint=False,
              return_timings=False):
        solver_name = solver
        if image is None:
            start = time.perf_counter()
            if objectives is None:
                self.model.minimize(self.objectives['makespan'])
            else:
                if nadir_points is None:
                    # If no nadir points are provided, do not normalize.
                    # Simply sum the objectives with their given weights.
                    expr = sum(
                        ((weight / 3600) if obj_name in {"makespan", "employee_time"} else weight) * self.objectives[obj_name]
                        for obj_name, weight in objectives.items()
                    )
                else:
                    norm_w = self._scaled_norm_weights(objectives, nadir_points)

                    expr = sum(
                        ((norm_w[obj_name] / 3600) if obj_name in {"makespan", "employee_time"} else norm_w[obj_name])
                        * self.objectives[obj_name]
                        for obj_name in objectives
                    )
                self.model.minimize(expr)
            objective_end = time.perf_counter()

            solver_obj = SolverLookup.get(solver_name, self.model)
            native_model_end = time.perf_counter()

            if solution_hint:
                # if isinstance(solution_hint, dict):
                #     h_vars = list(solution_hint.keys())
                #     h_vals = list(solution_hint.values())
                #     solver.solution_hint(h_vars, h_vals)
                # else:
                #     vars_to_hint = self.get_all_variables()
                #     vals_to_hint = [v.value() for v in vars_to_hint]
                #     hints = [(v, val) for v, val in zip(vars_to_hint, vals_to_hint) if val is not None]
                #     if hints:
                #         h_vars, h_vals = zip(*hints)
                #         solver.solution_hint(h_vars, h_vals)
                pass

            solve_start = time.perf_counter()
            solver_obj.solve(time_limit=timeout)
            end = time.perf_counter()
            elapsed = end - start
        else:
            start = time.perf_counter()
            expression = 1 / (iter)
            norm_w = self._scaled_norm_weights(objectives, nadir_points)
            expr_1 = (1 - expression) * sum(
                ((norm_w[obj_name] / 3600) if obj_name in {"makespan", "employee_time"} else norm_w[obj_name])
                * self.objectives[obj_name]
                for obj_name in objectives
            )

            w_div = compute_diversification_weights(diversification, objectives, trade_offs, c_ucb=c_ucb)

            img_coeff = {}
            if nadir_points is not None:
                for obj_name in self.objectives:
                    denom = (
                            nadir_points[obj_name]['max']
                            - nadir_points[obj_name]['min']
                    )
                    if denom == 0:
                        denom = 1
                    img_coeff[obj_name] = 10000 * w_div[obj_name] / denom
            else:
                for obj_name in self.objectives:
                    img_coeff[obj_name] = 10000 * w_div[obj_name]

            expr_2 = expression * sum(
                ((img_coeff[obj_name] / 3600) if obj_name in {"makespan", "employee_time"} else img_coeff[obj_name])
                * cp.abs(
                    self.objectives[obj_name]
                    - math.ceil(
                        (image[obj_name] * 3600) if obj_name in {"makespan", "employee_time"} else image[obj_name]
                    )
                )
                for obj_name in self.objectives
            )

            self.model.minimize(expr_1 - expr_2)

            # Create a shared assumption variable for conditional constraints
            conditional_constraint_assumption = cp.boolvar()

            if disjunctive is True:
                dis_constraint = cp.any([
                    self.objectives[obj_name]
                    < (
                        math.ceil(image[obj_name] * 3600)
                        if obj_name in {"makespan", "employee_time"}
                        else image[obj_name]
                    )
                    for obj_name in self.objectives
                ])
                self.model += conditional_constraint_assumption.implies(dis_constraint)
            else: # disjunctive is False: ensure the new solution is not identical to the 'image' solution
                not_same_image_constraint = cp.any([
                    self.objectives[obj_name]
                    != (
                        math.ceil(image[obj_name] * 3600)
                        if obj_name in {"makespan", "employee_time"}
                        else image[obj_name]
                    )
                    for obj_name in self.objectives
                ])
                self.model += conditional_constraint_assumption.implies(not_same_image_constraint)

            objective_end = time.perf_counter()

            solver_obj = SolverLookup.get(solver_name, self.model)
            native_model_end = time.perf_counter()

            if solution_hint:
                # if isinstance(solution_hint, dict):
                #     h_vars = list(solution_hint.keys())
                #     h_vals = list(solution_hint.values())
                #     solver.solution_hint(h_vars, h_vals)
                # else:
                #     vars_to_hint = self.get_all_variables()
                #     vals_to_hint = [v.value() for v in vars_to_hint]
                #     hints = [(v, val) for v, val in zip(vars_to_hint, vals_to_hint) if val is not None]
                #     if hints:
                #         h_vars, h_vals = zip(*hints)
                #         solver.solution_hint(h_vars, h_vals)
                pass

            solve_start = time.perf_counter()
            solver_obj.solve(time_limit=timeout,
                             assumptions=[conditional_constraint_assumption], # Always pass the assumption
                             log_search_progress=False)
            end = time.perf_counter()
            elapsed = end - start
        objs = {
            name: (
                (1 / 3600) * self.objectives[name].value()
                if name in {"makespan", "employee_time"}
                else self.objectives[name].value()
            )
            for name in objectives
        }

        timings = {
            "native_model_build_s": native_model_end - objective_end,
            "solve_call_wall_s": end - solve_start,
        }

        if return_timings:
            return elapsed, solver_obj.status().exitstatus, objs, timings
        return elapsed, solver_obj.status().exitstatus, objs

    # --- helper: compute scaled normalized integer weights once ---
    def _scaled_norm_weights(self, objectives, nadir_points, scale=1e4):
        """
        Returns dict: obj_name -> int(scale * weight / (max-min))
        """
        w = {}
        for obj_name, weight in objectives.items():
            if nadir_points is not None:
                denom = nadir_points[obj_name]['max'] - nadir_points[obj_name]['min']
                if abs(denom) < 1e-4:
                    denom = 1
                w[obj_name] = scale * (weight / denom)
            else:
                w[obj_name] = scale * (weight)
        return w
