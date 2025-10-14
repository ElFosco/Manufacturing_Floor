import os
import webbrowser
import uuid
import shutil
import threading
from datetime import datetime

from utils.jobshop_model_transport import FJTransportProblem
from utils.costant import TYPE_TO_SUBTYPES, WS_KITTING, WS_BUILDING_1, WS_BUILDING_2, WS_PALLETTING, HUMAN_JOBS_TIME, \
    FLASHLIGHT_CLIPPED, FLASHLIGHT_SCREWS, ROBOT_JOBS_TIME
from utils.item_definitions import get_all_item_names, get_item_definition
from utils.utility_classes import FWI

import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk, ImageSequence
import sv_ttk


# Instantiate your problem
problem = FJTransportProblem(symmetry_breaking=True)
solutions_table = None
product_table = None
product_totals = None
product_items_map = None
product_max_per_item = 6
update_ws_registry_hook = None

# Solution storage system
solutions_storage = {}  # Dictionary to store solution data with IDs
solution_counter = 0  # Counter for generating unique IDs

# =========================
# Solution Storage System
# =========================
def generate_solution_id():
    """Generate a unique solution ID"""
    global solution_counter
    solution_counter += 1
    return f"{solution_counter:03d}"

def store_solution_data(solution_id, makespan, resources, transport, employees, robots, topology_path, solution_path):
    """Store solution data with associated image paths"""
    solutions_storage[solution_id] = {
        'id': solution_id,
        'makespan': makespan,
        'resources': resources,
        'transport': transport,
        'employees': employees,
        'robots': robots,
        'topology_path': topology_path,
        'solution_path': solution_path,
        'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    }


def _load_fit_into_label(label, image_path):
    stop_gif(label)
    pil_img = Image.open(image_path)
    label._pil_image = pil_img

    def fit_image_to_label(lab, pil_image):
        if pil_image is None:
            return
        w = lab.winfo_width()
        h = lab.winfo_height()
        if w > 1 and h > 1:
            src_w, src_h = pil_image.size
            target_w = min(w, src_w)
            target_h = min(h, src_h)
            img_copy = pil_image.copy()
            img_copy.thumbnail((target_w, target_h), Image.LANCZOS)
            tk_img = ImageTk.PhotoImage(img_copy)
            lab.configure(image=tk_img)
            lab.image = tk_img

    fit_image_to_label(label, pil_img)
    label.bind("<Configure>", lambda e: fit_image_to_label(label, getattr(label, "_pil_image", None)))


def save_solution_images(solution_id):
    """Save topology and solution images with solution ID"""
    # Create solutions directory if it doesn't exist
    solutions_dir = "./solutions"
    os.makedirs(solutions_dir, exist_ok=True)
    
    # Copy topology image
    topology_src = "./images/topology.jpg"
    topology_dst = f"{solutions_dir}/topology_{solution_id}.jpg"
    if os.path.exists(topology_src):
        shutil.copy2(topology_src, topology_dst)
    
    # Copy solution image
    solution_src = "./images/sol.jpg"
    solution_dst = f"{solutions_dir}/solution_{solution_id}.jpg"
    if os.path.exists(solution_src):
        shutil.copy2(solution_src, solution_dst)
    
    return topology_dst, solution_dst

def show_solution_images(solution_id, topology_img_label, solution_img_label):
    """Display topology and solution images for a given solution ID in existing frames"""
    if solution_id not in solutions_storage:
        messagebox.showerror("Error", f"Solution {solution_id} not found!")
        return
    
    solution_data = solutions_storage[solution_id]
    
    # Load and display topology image in existing topology frame
    try:
        if os.path.exists(solution_data['topology_path']):
            stop_gif(topology_img_label)
            topology_img = Image.open(solution_data['topology_path'])
            topology_img_label._pil_image = topology_img

            def fit_topology_image(label, pil_image):
                if pil_image is None:
                    return
                w = label.winfo_width()
                h = label.winfo_height()
                if w > 1 and h > 1:
                    src_w, src_h = pil_image.size
                    target_w = min(w, src_w)
                    target_h = min(h, src_h)
                    img_copy = pil_image.copy()
                    img_copy.thumbnail((target_w, target_h), Image.LANCZOS)
                    tk_img = ImageTk.PhotoImage(img_copy)
                    label.configure(image=tk_img)
                    label.image = tk_img

            fit_topology_image(topology_img_label, topology_img)
            topology_img_label.bind(
                "<Configure>",
                lambda e: fit_topology_image(
                    topology_img_label, getattr(topology_img_label, "_pil_image", None)
                )
            )
        else:
            topology_img_label.configure(text="Topology image not found")
    except Exception as e:
        topology_img_label.configure(text=f"Error loading topology: {e}")
    
    # Load and display solution image in existing solution frame
    try:
        if os.path.exists(solution_data['solution_path']):
            stop_gif(solution_img_label)
            solution_img = Image.open(solution_data['solution_path'])
            solution_img_label._pil_image = solution_img

            def fit_solution_image(label, pil_image):
                if pil_image is None:
                    return
                w = label.winfo_width()
                h = label.winfo_height()
                if w > 1 and h > 1:
                    src_w, src_h = pil_image.size
                    target_w = min(w, src_w)
                    target_h = min(h, src_h)
                    img_copy = pil_image.copy()
                    img_copy.thumbnail((target_w, target_h), Image.LANCZOS)
                    tk_img = ImageTk.PhotoImage(img_copy)
                    label.configure(image=tk_img)
                    label.image = tk_img

            fit_solution_image(solution_img_label, solution_img)
            solution_img_label.bind(
                "<Configure>",
                lambda e: fit_solution_image(
                    solution_img_label, getattr(solution_img_label, "_pil_image", None)
                )
            )
        else:
            solution_img_label.configure(text="Solution image not found")
    except Exception as e:
        solution_img_label.configure(text=f"Error loading solution: {e}")
    
    # Show solution info in a message box
    # info_text = (f"Solution ID: {solution_id}\n"
    #             f"Makespan: {solution_data['makespan']}\n"
    #             f"Workstations: {solution_data['resources']}\n"
    #             f"Conv. Belts: {solution_data['transport']}\n"
    #             f"Operators: {solution_data['employees']}\n"
    #             f"Robots: {solution_data['robots']}\n"
    #             f"Created: {solution_data['timestamp']}")
    #
    # messagebox.showinfo(f"Solution {solution_id} Details", info_text)

def run_fwi_background(top_k=5, objectives_names=None, default_values=None):
    """Run FWI algorithm in background and populate the solutions table"""
    try:
        # Commit items from UI to model before solving
        if not commit_items_to_model():
            return

        # Set objective type for FWI
        problem.model_problem(objective_type=4)

        # Create FWI instance with app integration
        fwi = FWI(
            jobshop=problem,
            top_k=top_k,
            solutions_table=solutions_table,
            solutions_storage=solutions_storage,
            solution_counter=[solution_counter]  # Pass as list to allow modification
        )
        
        # Run FWI in background thread
        print("Starting FWI algorithm in background...")
        fwi_thread = threading.Thread(target=fwi.start_fwi, daemon=True)
        fwi_thread.start()
        
        # Show progress message
        messagebox.showinfo("FWI Started", f"FWI algorithm is running in background!\nFinding up to {top_k} solutions...\nYou can continue using the app.")
        
    except Exception as e:
        messagebox.showerror("FWI Error", f"Failed to start FWI: {e}")

def run_fwi(top_k=5, objectives_names=None, default_values=None):
    """Run FWI algorithm and populate the solutions table"""
    run_fwi_background(top_k, objectives_names, default_values)

# =========================
# UI helpers (images / GIF)
# =========================
def ensure_dir_for(path: str):
    os.makedirs(os.path.dirname(path), exist_ok=True)

def load_image(path, max_width, max_height):
    img = Image.open(path)
    img.thumbnail((max_width, max_height), Image.LANCZOS)
    return ImageTk.PhotoImage(img)

def load_gif(path, max_width, max_height):
    img = Image.open(path)
    frames = []
    for frame in ImageSequence.Iterator(img):
        f = frame.copy()
        f.thumbnail((max_width, max_height), Image.LANCZOS)
        frames.append(ImageTk.PhotoImage(f))
    return frames

def stop_gif(label):
    job = getattr(label, "_anim_job", None)
    if job is not None:
        try:
            label.after_cancel(job)
        except Exception:
            pass
        label._anim_job = None

def animate_gif(label, frames, delay=100):
    def update(idx=0):
        frame = frames[idx]
        label.configure(image=frame)
        label.image = frame
        label._anim_job = label.after(delay, update, (idx + 1) % len(frames))
    update()

def make_section(parent, title, img_path=None, max_w=1000, max_h=400, border=True):
    """
    Returns (outer_frame, inner_frame, image_label_or_None).
    Caller decides pack/grid for outer to avoid mixing geometry managers.
    """
    outer = tk.Frame(
        parent,
        highlightbackground="white" if border else "",
        highlightthickness=3 if border else 0,
        bd=0
    )

    title_lbl = ttk.Label(outer, text=title, font=("Helvetica", 15, "bold"))
    title_lbl.pack(anchor="w", padx=6, pady=(4, 0))

    inner = ttk.Frame(outer, padding=8)
    inner.pack(expand=True, fill="both")

    img_label = None
    if img_path:
        img_label = ttk.Label(inner, anchor="center")
        img_label.pack(expand=True, fill="both")

        if img_path.lower().endswith(".gif"):
            frames = load_gif(img_path, max_w, max_h)
            animate_gif(img_label, frames, delay=100)
        else:
            im = load_image(img_path, max_w, max_h)
            img_label.image = im
            img_label.configure(image=im)

    return outer, inner, img_label

# =========================
# Helpers for solution log
# =========================
def build_config_data(prob: FJTransportProblem) -> tuple:
    try:
        num_items = len(getattr(prob, "items_to_build", {}) or {})
        num_conns = len(getattr(prob, "connected_via_conveyor", []) or [])
        res = getattr(prob, "resources", {}) or {}
        # Count by high-level roles
        def count_type(key):
            lst = res.get(key, [])
            return len(lst) if lst is not None else 0
        from utils.costant import KIT, ASM, PACK
        kits = count_type(KIT)
        asms = count_type(ASM)
        packs = count_type(PACK)
        humans = len(res.get('human', []) or [])
        robots = len(res.get('robot', []) or [])
        return (num_items, kits, asms, packs, humans, robots, num_conns)
    except Exception:
        return (0, 0, 0, 0, 0, 0, 0)

def build_used_resources_data(prob: FJTransportProblem) -> tuple:
    """
    Count only the resources that are actually used in the solution.
    Returns (num_items, used_kits, used_asms, used_packs, used_humans, used_robots, used_belts)
    """
    try:
        num_res = prob.resource_count.value()
        num_tr = prob.tr_count.value()
        num_human = int(prob.humans_used if isinstance(prob.humans_used, int) else prob.humans_used.value())
        num_robot = int(prob.robot_used if isinstance(prob.robot_used, int) else prob.robot_used.value())
        # Debug: Print available attributes to understand the structure
        print(f"Available attributes: {[attr for attr in dir(prob) if not attr.startswith('_')]}")

        # Return only the values we need for the 4 columns
        return (num_res,num_tr,num_human, num_robot)
    except Exception as e:
        print(f"Error in build_used_resources_data: {e}")
        return (0, 0, 0, 0)



def clear_all_items():
    """Clear all items from both the problem and the UI table."""
    try:
        # Reset model items
        if hasattr(problem, "items_to_build"):
            problem.items_to_build = {}
            if hasattr(problem, "max_id_item"):
                problem.max_id_item = 0
        # Reset UI table
        global product_table, product_totals, product_items_map
        if product_table is not None and product_totals is not None and product_items_map is not None:
            for name in product_items_map.keys():
                product_totals[name] = 0
                try:
                    product_table.item(name, values=(name, 0))
                except Exception:
                    pass
    except Exception:
        pass

def commit_items_to_model():
    """Commit current UI item quantities to the model."""
    try:
        global product_totals, product_items_map
        if product_totals is not None and product_items_map is not None:
            # Clear existing items from model
            if hasattr(problem, "items_to_build"):
                problem.items_to_build = {}
                if hasattr(problem, "max_id_item"):
                    problem.max_id_item = 0
            
            # Add items from UI totals to model
            for name, qty in product_totals.items():
                if qty > 0:
                    problem.add_items_to_build(product_items_map[name], qty)
    except Exception as e:
        messagebox.showerror("Commit error", f"Failed to commit items to model: {e}")
        return False
    return True

# =====================================
# Workstation mapping / validation
# =====================================
def resolve_ws_constant_name(type_str: str, subtype_str: str | None):
    if type_str == "KIT":
        return "WS_KITTING"
    if type_str == "PALLETING":
        return "WS_PALLETTING"
    if type_str == "ASSEMBLY":
        if subtype_str == "GRIP":
            return "WS_BUILDING_1"
        if subtype_str == "GRIP & SCREW":
            return "WS_BUILDING_2"
    return None  # HUMAN, ROBOT or invalid

CONST_MAP = {
    "WS_KITTING": WS_KITTING,
    "WS_BUILDING_1": WS_BUILDING_1,
    "WS_BUILDING_2": WS_BUILDING_2,
    "WS_PALLETTING": WS_PALLETTING,
}

# role labels used for connection checks
CONST_TO_ROLE = {
    "WS_KITTING": "KIT",
    "WS_BUILDING_1": "GRIP",
    "WS_BUILDING_2": "GRIP & SCREW",
    "WS_PALLETTING": "PALLET",
}

def is_allowed_connection(role_from: str, role_to: str) -> bool:
    # Allowed:
    # KIT -> GRIP / GRIP & SCREW
    # GRIP -> GRIP & SCREW
    # GRIP -> PALLET
    # GRIP & SCREW -> PALLET
    if role_from == "KIT" and role_to in ("GRIP", "GRIP & SCREW"):
        return True
    if role_from == "GRIP" and role_to in ("GRIP & SCREW", "PALLET"):
        return True
    if role_from == "GRIP & SCREW" and role_to == "PALLET":
        return True
    return False

# =====================================
# Topology Creation controls (left-top image refresh supported)
# =====================================
def bind_topology_controls(parent_frame, topology_img_label, img_max_w=1000, img_max_h=400):
    """
    parent_frame: inner frame of "Topology Creation"
    topology_img_label: the label showing the 'Workstation Topology' image/GIF (left-top)
    """
    # ---------- Add Workstation ----------
    ws_controls = ttk.Frame(parent_frame, padding=8)
    ws_controls.pack(anchor="w", fill="x")

    ttk.Label(ws_controls, text="Workstation Type:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
    type_var = tk.StringVar()
    type_cb = ttk.Combobox(ws_controls, textvariable=type_var,
                           values=list(TYPE_TO_SUBTYPES.keys()),
                           state="readonly", width=18)
    type_cb.grid(row=0, column=1, padx=5, pady=5, sticky="w")

    ttk.Label(ws_controls, text="Subtype:").grid(row=0, column=2, padx=12, pady=5, sticky="w")
    subtype_var = tk.StringVar()
    subtype_cb = ttk.Combobox(ws_controls, textvariable=subtype_var, state="disabled", width=18)
    subtype_cb.grid(row=0, column=3, padx=5, pady=5, sticky="w")

    add_ws_btn = ttk.Button(ws_controls, text="Add")
    add_ws_btn.grid(row=0, column=4, padx=16, pady=5, sticky="w")

    ttk.Separator(parent_frame, orient="horizontal").pack(fill="x", padx=8, pady=(4, 8))

    # ---------- Add Conveyor Belt ----------
    tr_controls = ttk.Frame(parent_frame, padding=8)
    tr_controls.pack(anchor="w", fill="x")

    ttk.Label(tr_controls, text="From (ID):").grid(row=0, column=0, padx=5, pady=5, sticky="w")
    from_var = tk.StringVar()
    from_cb = ttk.Combobox(tr_controls, textvariable=from_var, state="readonly", width=18)
    from_cb.grid(row=0, column=1, padx=5, pady=5, sticky="w")

    ttk.Label(tr_controls, text="To (ID):").grid(row=0, column=2, padx=12, pady=5, sticky="w")
    to_var = tk.StringVar()
    to_cb = ttk.Combobox(tr_controls, textvariable=to_var, state="readonly", width=18)
    to_cb.grid(row=0, column=3, padx=5, pady=5, sticky="w")

    add_tr_btn = ttk.Button(tr_controls, text="Add")
    add_tr_btn.grid(row=0, column=4, padx=16, pady=5, sticky="w")

    # ---------- State ----------
    ws_registry = []  # entries: {"id": int, "role": str}

    def update_subtype_state(_evt=None):
        t = type_var.get()
        subs = TYPE_TO_SUBTYPES.get(t, [])
        if subs:
            subtype_cb["values"] = subs
            subtype_cb.state(["!disabled", "readonly"])
            subtype_var.set(subs[0])
        else:
            subtype_cb.set("")
            subtype_cb["values"] = []
            subtype_cb.state(["disabled"])

    def preview_topology():
        """Load './images/topology.jpg' into the left-top preview using responsive fit."""
        try:
            stop_gif(topology_img_label)
            from PIL import Image
            pil_img = Image.open("./images/topology.jpg")
            topology_img_label._pil_image = pil_img

            def fit_image_to_label(label, pil_image):
                if pil_image is None:
                    return
                w = label.winfo_width()
                h = label.winfo_height()
                if w > 1 and h > 1:
                    src_w, src_h = pil_image.size
                    target_w = min(w, src_w)
                    target_h = min(h, src_h)
                    img_copy = pil_image.copy()
                    img_copy.thumbnail((target_w, target_h), Image.LANCZOS)
                    tk_img = ImageTk.PhotoImage(img_copy)
                    label.configure(image=tk_img)
                    label.image = tk_img

            fit_image_to_label(topology_img_label, pil_img)
            topology_img_label.bind(
                "<Configure>",
                lambda e: fit_image_to_label(
                    topology_img_label, getattr(topology_img_label, "_pil_image", None)
                )
            )
        except Exception as e:
            messagebox.showerror("Preview error", f"Failed to load topology image: {e}")

    def update_ws_id_options():
        display = [f"{entry['id']} — {entry['role']}" for entry in ws_registry]
        from_cb["values"] = display
        to_cb["values"] = display

    def rebuild_ws_registry_from_problem():
        try:
            ws_registry.clear()
            from utils.costant import KIT, ASM, PACK
            for typ, lst in getattr(problem, "resources", {}).items():
                if typ in ['human', 'robot']:
                    continue
                if typ == KIT:
                    for mid, sub in lst:
                        ws_registry.append({"id": mid, "role": "KIT"})
                elif typ == PACK:
                    for mid, sub in lst:
                        ws_registry.append({"id": mid, "role": "PALLET"})
                elif typ == ASM:
                    for mid, sub in lst:
                        role = "GRIP" if sub == FLASHLIGHT_CLIPPED else ("GRIP & SCREW" if sub == FLASHLIGHT_SCREWS else "ASM")
                        ws_registry.append({"id": mid, "role": role})
            ws_registry.sort(key=lambda e: e["id"])  # stable order
            update_ws_id_options()
        except Exception:
            pass

    def add_workstation():
        t = (type_var.get() or "").strip()
        st = (subtype_var.get() or "").strip() if subtype_cb.instate(["!disabled"]) else None

        if not t:
            messagebox.showwarning("Missing type", "Please choose a Type.")
            return

        if t == "HUMAN":
            try:
                problem.add_human()
                problem.set_dur_hum(HUMAN_JOBS_TIME)
                ensure_dir_for("./images/topology.jpg")
                problem.make_ws_updated(location="./images/topology.jpg")
                preview_topology()
            except Exception as e:
                messagebox.showerror("Problem error", f"Failed to add human: {e}")
            return

        if t == "ROBOT":
            try:
                problem.add_robot()
                problem.set_dur_robot(ROBOT_JOBS_TIME)  # Robot transport time
                ensure_dir_for("./images/topology.jpg")
                problem.make_ws_topology(location="./images/topology.jpg")
                preview_topology()
            except Exception as e:
                messagebox.showerror("Problem error", f"Failed to add robot: {e}")
            return

        const_name = resolve_ws_constant_name(t, st)
        if const_name is None:
            messagebox.showerror("Invalid selection", f"No mapping for Type={t} Subtype={st or '-'}")
            return

        const_value = CONST_MAP.get(const_name)
        if const_value is None:
            messagebox.showerror("Missing constant", f"{const_name} is not defined.")
            return

        try:
            new_id = problem.add_workstation(const_value)
            role = CONST_TO_ROLE[const_name]
            ws_registry.append({"id": new_id, "role": role})
            update_ws_id_options()

            ensure_dir_for("./images/topology.jpg")
            problem.make_ws_topology(location="./images/topology.jpg")
            preview_topology()
            try:
                if update_ws_registry_hook:
                    update_ws_registry_hook()
            except Exception:
                pass
        except Exception as e:
            messagebox.showerror("Problem error", f"Failed to add workstation or update preview: {e}")

    def parse_id(sel: str) -> int | None:
        if not sel:
            return None
        try:
            return int(sel.split("—")[0].strip())
        except Exception:
            try:
                return int(sel.split("-")[0].strip())
            except Exception:
                return None

    def role_of(ws_id: int) -> str | None:
        for entry in ws_registry:
            if entry["id"] == ws_id:
                return entry["role"]
        return None

    def add_transport():
        sel_from = from_var.get().strip()
        sel_to = to_var.get().strip()

        id_from = parse_id(sel_from)
        id_to = parse_id(sel_to)

        if not id_from or not id_to:
            messagebox.showwarning("Missing selection", "Please choose both From and To workstation IDs.")
            return
        if id_from == id_to:
            messagebox.showwarning("Invalid selection", "From and To must be different workstations.")
            return

        r_from = role_of(id_from)
        r_to = role_of(id_to)
        if r_from is None or r_to is None:
            messagebox.showerror("Unknown IDs", "Selected IDs are not in the current topology.")
            return

        if not is_allowed_connection(r_from, r_to):
            messagebox.showerror(
                "Not allowed",
                f"Connection {r_from} → {r_to} is not permitted.\n\n"
                "Allowed:\n"
                "• KIT → GRIP / GRIP & SCREW\n"
                "• GRIP → GRIP & SCREW\n"
                "• GRIP → PALLET\n"
                "• GRIP & SCREW → PALLET"
            )
            return

        try:
            problem.add_transport(id_from, id_to)
            ensure_dir_for("./images/topology.jpg")
            problem.make_ws_topology(location="./images/topology.jpg")
            preview_topology()
        except Exception as e:
            messagebox.showerror("Problem error", f"Failed to add transport or update preview: {e}")

    type_cb.bind("<<ComboboxSelected>>", update_subtype_state)
    add_ws_btn.configure(command=add_workstation)
    add_tr_btn.configure(command=add_transport)

    type_cb.set("KIT")
    update_subtype_state()

    # Expose a hook to refresh registry after external layout changes
    global update_ws_registry_hook
    update_ws_registry_hook = rebuild_ws_registry_from_problem

# =====================================
# Pre-made topology (3 layouts)
# =====================================
def apply_layout_1(prob: FJTransportProblem):
    """
    Example: KIT -> GRIP -> GRIP & SCREW -> PALLET
    """
    # TODO: clear/reset if your API supports it (e.g., prob.reset_topology())
    id_kit = prob.add_workstation(WS_KITTING)
    id_grip = prob.add_workstation(WS_BUILDING_1)
    id_gs  = prob.add_workstation(WS_BUILDING_2)
    id_pal = prob.add_workstation(WS_PALLETTING)

    prob.add_transport(id_kit, id_grip)  # KIT -> GRIP
    prob.add_transport(id_grip, id_gs)   # GRIP -> GRIP & SCREW
    prob.add_transport(id_gs, id_pal)    # GRIP & SCREW -> PALLET

def apply_layout_2(prob: FJTransportProblem):
    """
    Example: KIT feeds both GRIP and GRIP & SCREW in parallel; then both to PALLET
    (GRIP -> GS allowed, but here we show direct paths)
    """
    id_kit = prob.add_workstation(WS_KITTING)
    id_grip = prob.add_workstation(WS_BUILDING_1)
    id_gs  = prob.add_workstation(WS_BUILDING_2)
    id_pal = prob.add_workstation(WS_PALLETTING)

    prob.add_transport(id_kit, id_grip)  # KIT -> GRIP
    prob.add_transport(id_kit, id_gs)    # KIT -> GRIP & SCREW
    prob.add_transport(id_grip, id_pal)  # GRIP -> PALLET
    prob.add_transport(id_gs, id_pal)    # GRIP & SCREW -> PALLET

def apply_layout_3(prob: FJTransportProblem):
    """
    Example: Two KITs -> GRIP -> GRIP & SCREW -> PALLET (slightly larger line)
    """
    id_kit1 = prob.add_workstation(WS_KITTING)
    id_kit2 = prob.add_workstation(WS_KITTING)
    id_grip = prob.add_workstation(WS_BUILDING_1)
    id_gs  = prob.add_workstation(WS_BUILDING_2)
    id_pal = prob.add_workstation(WS_PALLETTING)

    prob.add_transport(id_kit1, id_grip)
    prob.add_transport(id_kit2, id_grip)
    prob.add_transport(id_grip, id_gs)
    prob.add_transport(id_gs, id_pal)

def bind_premade_topology(parent_frame, topology_img_label, img_max_w=1000, img_max_h=400):
    """
    Buttons that apply a predefined layout and refresh the left-top preview.
    Uses GRID consistently to avoid pack/grid conflicts.
    """
    # Make parent_frame grid-expandable
    parent_frame.columnconfigure(0, weight=1)
    parent_frame.rowconfigure(0, weight=1)

    # Row container (grid into parent_frame)
    row = ttk.Frame(parent_frame, padding=4)
    row.grid(row=0, column=0, sticky="nsew")
    row.columnconfigure(0, weight=1)
    row.columnconfigure(1, weight=0)
    row.columnconfigure(2, weight=0)
    row.columnconfigure(3, weight=0)
    row.columnconfigure(4, weight=1)

    def refresh_preview():
        ensure_dir_for("./images/topology.jpg")
        problem.make_ws_topology(location="./images/topology.jpg")
        try:
            stop_gif(topology_img_label)
            from PIL import Image  # local import in case top-level changed
            pil_img = Image.open("./images/topology.jpg")
            topology_img_label._pil_image = pil_img

            def fit_image_to_label(label, pil_image):
                if pil_image is None:
                    return
                w = label.winfo_width()
                h = label.winfo_height()
                if w > 1 and h > 1:
                    src_w, src_h = pil_image.size
                    target_w = min(w, src_w)
                    target_h = min(h, src_h)
                    img_copy = pil_image.copy()
                    img_copy.thumbnail((target_w, target_h), Image.LANCZOS)
                    tk_img = ImageTk.PhotoImage(img_copy)
                    label.configure(image=tk_img)
                    label.image = tk_img

            fit_image_to_label(topology_img_label, pil_img)
            topology_img_label.bind(
                "<Configure>",
                lambda e: fit_image_to_label(
                    topology_img_label, getattr(topology_img_label, "_pil_image", None)
                )
            )
        except Exception as e:
            messagebox.showerror("Preview error", f"Failed to load topology image: {e}")

    def run_layout(apply_fn):
        try:
            apply_fn()  # creates new problem and builds it
            ensure_dir_for("./images/topology.jpg")
            problem.make_ws_topology(location="./images/topology.jpg")
            refresh_preview()
            # Refresh the topology comboboxes with IDs from the rebuilt model
            try:
                if update_ws_registry_hook:
                    update_ws_registry_hook()
            except Exception:
                pass
        except Exception as e:
            messagebox.showerror("Layout error", f"Failed to apply layout: {e}")

    def apply_layout_1():
        global problem
        # Reset the problem first
        problem = FJTransportProblem(symmetry_breaking=True)
        problem.add_human()
        problem.add_human()
        problem.set_dur_hum(HUMAN_JOBS_TIME)
        problem.add_robot()
        problem.add_robot()
        problem.set_dur_robot(ROBOT_JOBS_TIME)

        id_kit_1 = problem.add_workstation(WS_KITTING)
        id_kit_2 = problem.add_workstation(WS_KITTING)
        id_grip = problem.add_workstation(WS_BUILDING_1)
        id_gs = problem.add_workstation(WS_BUILDING_2)
        id_pal = problem.add_workstation(WS_PALLETTING)
        id_pal_2 = problem.add_workstation(WS_PALLETTING)

        problem.add_transport(id_kit_1, id_grip)
        problem.add_transport(id_kit_2, id_gs)
        problem.add_transport(id_grip, id_gs)
        problem.add_transport(id_gs, id_pal)


    def apply_layout_2():
        global problem
        # Reset the problem first
        problem = FJTransportProblem(symmetry_breaking=True)
        problem.add_human()
        problem.set_dur_hum(HUMAN_JOBS_TIME)
        # problem.add_robot()
        # problem.set_dur_robot(ROBOT_JOBS_TIME)

        id_kit_1 = problem.add_workstation(WS_KITTING)
        id_kit_2 = problem.add_workstation(WS_KITTING)

        id_grip_1 = problem.add_workstation(WS_BUILDING_1)
        id_grip_2 = problem.add_workstation(WS_BUILDING_1)
        id_gs = problem.add_workstation(WS_BUILDING_2)
        id_pal = problem.add_workstation(WS_PALLETTING)

        problem.add_transport(id_kit_1, id_grip_1)
        problem.add_transport(id_kit_2,id_grip_2)
        problem.add_transport(id_grip_1, id_pal)
        problem.add_transport(id_grip_2, id_gs)
        problem.add_transport(id_grip_2, id_pal)
        problem.add_transport(id_gs, id_pal)

    def apply_layout_3():
        global problem
        # Reset the problem first
        problem = FJTransportProblem(symmetry_breaking=True)
        problem.add_human()
        problem.set_dur_hum(HUMAN_JOBS_TIME)
        # problem.add_robot()
        # problem.set_dur_robot(ROBOT_JOBS_TIME)

        id_kit1 = problem.add_workstation(WS_KITTING)
        id_kit2 = problem.add_workstation(WS_KITTING)


        id_grip_1 = problem.add_workstation(WS_BUILDING_1)
        id_gs_1 = problem.add_workstation(WS_BUILDING_1)
        id_gs_2 = problem.add_workstation(WS_BUILDING_2)

        id_pal_1 = problem.add_workstation(WS_PALLETTING)
        id_pal_2 = problem.add_workstation(WS_PALLETTING)

        problem.add_transport(id_kit1, id_grip_1)
        problem.add_transport(id_kit1, id_gs_1)
        problem.add_transport(id_grip_1, id_pal_1)
        problem.add_transport(id_gs_1, id_pal_1)

        problem.add_transport(id_kit2, id_gs_2)
        problem.add_transport(id_gs_2, id_pal_2)



    # Buttons fill their grid cells
    ttk.Label(row).grid(row=0, column=0, sticky="ew")  # left spacer
    ttk.Button(row, text="Layout 1", command=lambda: run_layout(apply_layout_1)).grid(row=0, column=1, padx=6, pady=6)
    ttk.Button(row, text="Layout 2", command=lambda: run_layout(apply_layout_2)).grid(row=0, column=2, padx=6, pady=6)
    ttk.Button(row, text="Layout 3", command=lambda: run_layout(apply_layout_3)).grid(row=0, column=3, padx=6, pady=6)
    ttk.Label(row).grid(row=0, column=4, sticky="ew")  # right spacer
    
    # Reset button row (spans half the width of pre-made topology)
    reset_row = ttk.Frame(parent_frame, padding=4)
    reset_row.grid(row=1, column=0, sticky="ew")
    reset_row.columnconfigure(0, weight=1)
    reset_row.columnconfigure(1, weight=0)
    reset_row.columnconfigure(2, weight=1)
    
    def reset_topology():
        try:
            global problem
            problem = FJTransportProblem(symmetry_breaking=True)
            ensure_dir_for("./images/topology.jpg")
            problem.make_ws_topology(location="./images/topology.jpg")
            refresh_preview()
            if update_ws_registry_hook:
                update_ws_registry_hook()
        except Exception as e:
            messagebox.showerror("Reset error", f"Failed to reset topology: {e}")
    
    ttk.Label(reset_row).grid(row=0, column=0, sticky="ew")  # left spacer
    ttk.Button(reset_row, text="Reset", command=reset_topology).grid(row=0, column=1, padx=6, pady=6)
    ttk.Label(reset_row).grid(row=0, column=2, sticky="ew")  # right spacer


# =====================================
# Product selection (items to craft) — totals only
# =====================================
def bind_product_selection(parent_frame):
    """
    Adds 'product selection' controls + totals table.
    Enforces max 6 per item overall.
    """
    # Get all available item names dynamically
    available_items = get_all_item_names()
    ITEMS_MAP = {}
    for item_name in available_items:
        item_def = get_item_definition(item_name)
        if item_def:
            # Map item name to its constant value
            # For backward compatibility, we still use the old constants
            if item_name == "FLASHLIGHT_CLIPPED":
                ITEMS_MAP[item_name] = FLASHLIGHT_CLIPPED
            elif item_name == "FLASHLIGHT_SCREWS":
                ITEMS_MAP[item_name] = FLASHLIGHT_SCREWS
            else:
                # For new items, use the item name as the value
                ITEMS_MAP[item_name] = item_name
    max_per_item = product_max_per_item
    totals = {name: 0 for name in ITEMS_MAP.keys()}

    row = ttk.Frame(parent_frame, padding=8)
    row.pack(anchor="w", fill="x")

    ttk.Label(row, text="Item:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
    item_var = tk.StringVar(value="FLASHLIGHT_CLIPPED")
    item_cb = ttk.Combobox(row, textvariable=item_var, values=list(ITEMS_MAP.keys()),
                           state="readonly", width=22)
    item_cb.grid(row=0, column=1, padx=5, pady=5, sticky="w")

    ttk.Label(row, text="Qty (1–6):").grid(row=0, column=2, padx=12, pady=5, sticky="w")
    qty_var = tk.StringVar(value="1")
    qty_sp = ttk.Spinbox(row, from_=1, to=max_per_item, textvariable=qty_var,
                         width=5, state="readonly")
    qty_sp.grid(row=0, column=3, padx=5, pady=5, sticky="w")

    add_item_btn = ttk.Button(row, text="Add")
    add_item_btn.grid(row=0, column=4, padx=8, pady=5, sticky="w")
    
    remove_item_btn = ttk.Button(row, text="Remove")
    remove_item_btn.grid(row=0, column=5, padx=8, pady=5, sticky="w")

    table = ttk.Treeview(parent_frame, columns=("item", "total"), show="headings", height=2)
    table.pack(fill="both", expand=True, padx=8, pady=(8, 0))
    for col, w in [("item", 220), ("total", 90)]:
        table.heading(col, text=col.upper())
        table.column(col, width=w, anchor="w", stretch=True)

    for name in ITEMS_MAP.keys():
        table.insert("", "end", iid=name, values=(name, 0))

    # expose globals for reset
    global product_table, product_totals, product_items_map
    product_table = table
    product_totals = totals
    product_items_map = ITEMS_MAP

    def add_items():
        name = item_var.get()
        try:
            qty = int(qty_var.get())
        except ValueError:
            messagebox.showwarning("Invalid quantity", "Please select a quantity between 1 and 6.")
            return

        if qty < 1 or qty > max_per_item:
            messagebox.showwarning("Invalid quantity", f"Quantity must be between 1 and {max_per_item}.")
            return

        current = totals[name]
        if current >= max_per_item:
            messagebox.showinfo("Limit reached", f"{name} already at {max_per_item}. Cannot add more.")
            return

        allowed = min(qty, max_per_item - current)
        
        # Only update UI table - don't add to model yet
        totals[name] += allowed
        table.item(name, values=(name, totals[name]))
        if allowed < qty:
            messagebox.showinfo("Adjusted", f"Only {allowed} were added to stay within the limit of {max_per_item}.")

    def remove_items():
        name = item_var.get()
        try:
            qty = int(qty_var.get())
        except ValueError:
            messagebox.showwarning("Invalid quantity", "Please select a quantity between 1 and 6.")
            return

        if qty < 1 or qty > max_per_item:
            messagebox.showwarning("Invalid quantity", f"Quantity must be between 1 and {max_per_item}.")
            return

        current = totals[name]
        if current <= 0:
            messagebox.showinfo("No items", f"No {name} items to remove.")
            return

        # Calculate how many we can actually remove
        to_remove = min(qty, current)
        
        # Only update UI table - don't remove from model yet
        totals[name] -= to_remove
        table.item(name, values=(name, totals[name]))
        
        if to_remove < qty:
            messagebox.showinfo("Adjusted", f"Only {to_remove} were removed (only {current} were available).")

    add_item_btn.configure(command=add_items)
    remove_item_btn.configure(command=remove_items)

def bind_solving_controls(parent_frame, solution_img_label, topology_img_label, img_max_w=800, img_max_h=400):
    """
    Adds a big 'Solve' button that runs the solver and updates the Solution image.
    """
    # Buttons row (Solve + Optimize Resources + Export HTML) centered
    btn_row = ttk.Frame(parent_frame)
    btn_row.pack(expand=False, fill="x")
    try:
        btn_row.columnconfigure(0, weight=1)
        btn_row.columnconfigure(1, weight=0)
        btn_row.columnconfigure(2, weight=0)
        btn_row.columnconfigure(3, weight=0)
        btn_row.columnconfigure(4, weight=0)
        btn_row.columnconfigure(5, weight=1)
    except Exception:
        pass

    solve_btn = ttk.Button(btn_row, text="Minimize Makespan", style="Solve.TButton")
    solve_btn.grid(row=0, column=1, padx=(8, 4), pady=8, ipady=6)

    optimize_btn = ttk.Button(btn_row, text="Minimize Makespan & Resources", style="Solve.TButton")
    optimize_btn.grid(row=0, column=2, padx=(4, 4), pady=8, ipady=6)

    # export_btn = ttk.Button(btn_row, text="Export HTML")
    # export_btn.state(["disabled"])  # enabled after a successful solve
    # export_btn.grid(row=0, column=3, padx=(4, 4), pady=8, ipady=6)

    # FWI controls frame
    fwi_frame = ttk.Frame(btn_row)
    fwi_frame.grid(row=0, column=4, padx=(4, 8), pady=8, sticky="ew")
    
    fwi_btn = ttk.Button(fwi_frame, text="Run FWI", style="Solve.TButton")
    fwi_btn.pack(side="left")
    
    # FWI spinbox
    fwi_spinbox_label = ttk.Label(fwi_frame)
    fwi_spinbox_label.pack(side="left", padx=(10, 5))
    
    fwi_spinbox_var = tk.IntVar(value=5)  # Default value
    fwi_spinbox = ttk.Spinbox(fwi_frame, from_=1, to=20, textvariable=fwi_spinbox_var, 
                             width=5, state="readonly")
    fwi_spinbox.pack(side="left", padx=(0, 5))

    def has_meaningful_content(prob: FJTransportProblem) -> bool:
        try:
            if getattr(prob, "items_to_build", None):
                return True
            res = getattr(prob, "resources", {}) or {}
            # any non-empty resource groups
            for k, v in res.items():
                if v:
                    return True
            return False
        except Exception:
            return True

    def run_solver():
        try:
            if not commit_items_to_model():
                return

            # 1) Solve and generate the fresh images FIRST
            active_prob = problem
            active_prob.model_problem(objective_type=0)
            active_prob.solve(solver="ortools")

            # Regenerate BOTH preview images right now so the sources are "current"
            ensure_dir_for("./images/topology.jpg")
            problem.make_ws_updated(location="./images/topology.jpg")
            active_prob.make_gantt(folder="./images/sol.jpg")  # fresh gantt
            _load_fit_into_label(topology_img_label, "./images/topology.jpg")

            # 2) NOW allocate a new id and copy the just-created images
            solution_id = generate_solution_id()
            topology_path, solution_path = save_solution_images(solution_id)

            # 3) Update the Solution preview from the freshly generated sol.jpg
            stop_gif(solution_img_label)
            pil_img = Image.open("./images/sol.jpg")
            solution_img_label._pil_image = pil_img

            def fit_image_to_label(label, pil_image):
                if pil_image is None: return
                w, h = label.winfo_width(), label.winfo_height()
                if w > 1 and h > 1:
                    src_w, src_h = pil_image.size
                    target_w = min(w, src_w)
                    target_h = min(h, src_h)
                    img_copy = pil_image.copy()
                    img_copy.thumbnail((target_w, target_h), Image.LANCZOS)
                    tk_img = ImageTk.PhotoImage(img_copy)
                    label.configure(image=tk_img)
                    label.image = tk_img

            fit_image_to_label(solution_img_label, pil_img)
            solution_img_label.bind("<Configure>",
                                    lambda e: fit_image_to_label(
                                        solution_img_label, getattr(solution_img_label, "_pil_image", None)
                                    )
                                    )

            # 4) Store row + metadata
            try:
                ms_val = None
                if hasattr(active_prob, "makespan") and getattr(active_prob.makespan, "value", None):
                    try:
                        v = active_prob.makespan.value()
                        ms_val = int(v) if v is not None else None
                    except Exception:
                        ms_val = None
                ms_text = str(ms_val) if ms_val is not None else "-"
                resources,transport,employees,robots,  = build_used_resources_data(active_prob)

                store_solution_data(solution_id, ms_text, resources,transport, employees, robots, topology_path, solution_path)

                if solutions_table is not None:
                    solutions_table.insert("", "end",
                                           values=(solution_id, ms_text, resources,transport, employees, robots))
            except Exception as e:
                print(f"Error storing solution: {e}")
                pass

        except Exception as e:
            messagebox.showerror("Solve error", f"Failed to solve: {e}")

    def run_optimize_resources():
        try:
            # Commit items from UI to model before solving
            if not commit_items_to_model():
                return

            # 1) Solve and generate the fresh images FIRST
            active_prob = problem
            active_prob.model_problem(objective_type=4)  # minimize resources
            active_prob.solve(solver="ortools")

            # Keep topology preview in sync and generate a fresh Gantt
            ensure_dir_for("./images/topology.jpg")
            active_prob.make_ws_updated(location="./images/topology.jpg")
            active_prob.make_gantt(folder="./images/sol.jpg")
            _load_fit_into_label(topology_img_label, "./images/topology.jpg")  # <— NEW: update left-top frame

            # 2) NOW allocate a new id and copy the just-created images
            solution_id = generate_solution_id()
            topology_path, solution_path = save_solution_images(solution_id)

            # 3) Update the Solution preview from the freshly generated sol.jpg
            stop_gif(solution_img_label)
            pil_img = Image.open("./images/sol.jpg")
            solution_img_label._pil_image = pil_img

            def fit_image_to_label(label, pil_image):
                if pil_image is None:
                    return
                w, h = label.winfo_width(), label.winfo_height()
                if w > 1 and h > 1:
                    src_w, src_h = pil_image.size
                    target_w = min(w, src_w)
                    target_h = min(h, src_h)
                    img_copy = pil_image.copy()
                    img_copy.thumbnail((target_w, target_h), Image.LANCZOS)
                    tk_img = ImageTk.PhotoImage(img_copy)
                    label.configure(image=tk_img)
                    label.image = tk_img

            fit_image_to_label(solution_img_label, pil_img)
            solution_img_label.bind(
                "<Configure>",
                lambda e: fit_image_to_label(
                    solution_img_label, getattr(solution_img_label, "_pil_image", None)
                )
            )

            # 4) Store row + metadata
            try:
                ms_val = None
                if hasattr(active_prob, "makespan") and getattr(active_prob.makespan, "value", None):
                    try:
                        v = active_prob.makespan.value()
                        ms_val = int(v) if v is not None else None
                    except Exception:
                        ms_val = None
                ms_text = str(ms_val) if ms_val is not None else "-"

                resources, transport, employees, robots = build_used_resources_data(active_prob)

                # Store solution data (images already saved with this ID)
                store_solution_data(
                    solution_id, ms_text, resources,transport, employees, robots, topology_path, solution_path
                )

                if solutions_table is not None:
                    solutions_table.insert(
                        "", "end", values=(solution_id, ms_text, resources,transport, employees, robots)
                    )
            except Exception as e:
                print(f"Error storing solution: {e}")

        except Exception as e:
            messagebox.showerror("Optimize Resources error", f"Failed to optimize resources: {e}")

    def export_html():
        try:
            # Generate an interactive HTML gantt via the model API
            ensure_dir_for("./images/dummy")
            html_path = "./images/image.html"
            try:
                # Some versions expect 'folder', others 'location'; keep 'folder' for consistency with image case
                problem.make_gantt(folder=html_path, html=True)
            except TypeError:
                # Fallback to location argument name if needed
                problem.make_gantt(folder=html_path, html=True)

            # Open in Chrome (fallback to default browser)
            opened = False
            for name in ("google-chrome", "chrome", "chromium-browser", "chromium"):
                try:
                    b = webbrowser.get(name)
                    b.open_new_tab(os.path.abspath(html_path))
                    opened = True
                    break
                except Exception:
                    pass
            if not opened:
                try:
                    webbrowser.open_new_tab(os.path.abspath(html_path))
                    opened = True
                except Exception:
                    pass

            if not opened:
                messagebox.showinfo("Export HTML", f"Saved: {html_path}\nCould not auto-open in a browser.")
        except Exception as e:
            messagebox.showerror("Export HTML", f"Failed to export HTML: {e}")

    solve_btn.configure(command=run_solver)
    optimize_btn.configure(command=run_optimize_resources)
    # export_btn.configure(command=export_html)
    fwi_btn.configure(command=lambda: run_fwi(top_k=int(fwi_spinbox_var.get())))

# ==============
# Main UI
# ==============
def main():
    root = tk.Tk()
    root.title("Two-Image Viewer")
    sv_ttk.set_theme("dark")

    # Styles
    style = ttk.Style()
    style.configure("TButton", font=("Helvetica", 12))
    style.configure("Solve.TButton", font=("Helvetica", 12, "bold"))

    container = ttk.Frame(root, padding=12)
    container.pack(fill="both", expand=True)

    # Three columns (add right-most 'Solutions' column)
    container.columnconfigure(0, weight=1, minsize=650)
    container.columnconfigure(1, weight=5, minsize=520)  # Increased weight for wider topology frame
    container.columnconfigure(2, weight=0, minsize=350)
    # Three rows: 0 = main two-column area, 1 = left-half Solving, 2 = full-width Solution (small)
    container.rowconfigure(0, weight=1)
    container.rowconfigure(1, weight=0)
    container.rowconfigure(2, weight=6, minsize=1)

    # LEFT COLUMN (topology preview + solution preview)
    left_frame = ttk.Frame(container)
    left_frame.grid(row=0, column=0, sticky="nsew")

    # Top-left: starts with waiting.gif, later replaced with ./images/topology.jpg
    sec1, _inner1, topology_img_label = make_section(
        left_frame, "Workstation Topology", "images/waiting.gif", 1200, 400
    )
    sec1.pack(side="top", expand=True, fill="both", pady=6)

    # (Moved) Solution section will appear full-width below, spanning both columns

    # RIGHT COLUMN (4 rows)
    right_frame = ttk.Frame(container)
    right_frame.grid(row=0, column=1, rowspan=2, sticky="nsew")
    right_frame.columnconfigure(0, weight=1)
    right_frame.rowconfigure(0, weight=2)  # Topology Creation (bigger)
    right_frame.rowconfigure(1, weight=2, minsize=110)  # Pre-made topology (bigger)
    right_frame.rowconfigure(2, weight=1, minsize=200)  # Product selection (minimum height)
    right_frame.rowconfigure(3, weight=1)  # Solving

    # Row 0: Topology Creation (buttons: Add Workstation / Add Conveyor Belt / items under this moved out)
    frame_topology_creation, topology_inner, _ = make_section(right_frame, "Topology Creation", border=True)
    frame_topology_creation.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
    bind_topology_controls(topology_inner, topology_img_label, img_max_w=1000, img_max_h=400)

    # Row 1: Pre-made topology (Layout 1/2/3) that rebuilds a new problem and refreshes left-top image
    frame_premade, premade_inner, _ = make_section(right_frame, "Pre-made topology", border=True)
    frame_premade.grid(row=1, column=0, sticky="nsew", padx=6, pady=6)
    bind_premade_topology(premade_inner, topology_img_label, img_max_w=1000, img_max_h=400)

    # Row 2: Product selection (totals-only table + add items)
    frame_products, products_inner, _ = make_section(right_frame, "Product selection", border=True)
    frame_products.grid(row=2, column=0, sticky="nsew", padx=6, pady=6)
    bind_product_selection(products_inner)

    # Full-width sections below both columns
    # Create full-width Solution preview (spans both columns)
    sec_solution, _inner_solution, solution_img_label = make_section(
        container, "Solution", "images/waiting.gif", 2000, 1000
    )
    sec_solution.grid(row=2, column=0, columnspan=2, sticky="ew", padx=6, pady=6)
    try:
        sec_solution.configure(height=150)
        sec_solution.grid_propagate(False)
    except Exception:
        pass

    # Center the solution image within its section
    try:
        solution_img_label.pack_forget()
        solution_img_label.pack(expand=True, fill="both")
    except Exception:
        pass

    # Create full-width Solving controls (spans both left columns, placed above Solution)
    frame_solving_full, solving_inner_full, _ = make_section(container, "Solving", border=True)
    frame_solving_full.grid(row=1, column=0, columnspan=1, sticky="ew", padx=6, pady=6)
    bind_solving_controls(solving_inner_full, solution_img_label, topology_img_label, img_max_w=400, img_max_h=400)

    # RIGHT-MOST COLUMN: Solutions
    solutions_frame = ttk.Frame(container)
    solutions_frame.grid(row=0, column=2, rowspan=3, sticky="nsew", padx=6, pady=6)
    solutions_section, solutions_inner, _ = make_section(solutions_frame, "Solutions", border=True)
    solutions_section.pack(expand=True, fill="both")

    # Solutions list (separate columns for each info)
    global solutions_table
    cols = ("ID", "SPAN", "STATIONS", "BELTS", "OPERATORS", "ROBOTS")
    solutions_table = ttk.Treeview(solutions_inner, columns=cols, show="headings", height=12)
    for head, w in [("ID", 30), ("SPAN", 45), ("STATIONS", 60),("BELTS", 40), ("OPERATORS", 60), ("ROBOTS", 50)]:
        solutions_table.heading(head, text=head)
        solutions_table.column(head, width=w, anchor="center", stretch=True)
    
    # Add click event handler
    def on_table_click(event):
        """Handle click on table row"""
        # Get the item that was actually clicked using coordinates
        item = solutions_table.identify_row(event.y)
        if item:
            # Get the values from the clicked row
            values = solutions_table.item(item, "values")
            if values and len(values) >= 5:
                solution_id, span, resources,transport, employees, robots = values
                print(f"Clicked row - ID: {solution_id}, SPAN: {span}, "
                      f"RESOURCES: {resources},TRANSPORT: {transport}, EMPLOYEES: {employees}, ROBOTS: {robots}")
                # Show the associated images in existing frames
                show_solution_images(solution_id, topology_img_label, solution_img_label)
    
    # Bind the click event to the table
    solutions_table.bind("<Button-1>", on_table_click)
    
    solutions_table.pack(expand=True, fill="both")

    root.minsize(1100, 750)
    root.mainloop()

if __name__ == "__main__":
    main()
