import os
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk, ImageSequence
import sv_ttk

from jobshop_model import FJTransportProblem
from utility.costant import TYPE_TO_SUBTYPES, WS_KITTING, WS_BUILDING_1, WS_BUILDING_2, WS_PALLETTING, HUMAN_JOBS_TIME, \
    FLASHLIGHT_CLIPPED, FLASHLIGHT_SCREWS

import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk, ImageSequence
import sv_ttk


# Instantiate your problem
problem = FJTransportProblem(symmetry_breaking=True)

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

def make_section(parent, title, img_path=None, max_w=800, max_h=400, border=True):
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
        img_label = ttk.Label(inner)
        img_label.pack(expand=True, fill="both")

        if img_path.lower().endswith(".gif"):
            frames = load_gif(img_path, max_w, max_h)
            animate_gif(img_label, frames, delay=100)
        else:
            im = load_image(img_path, max_w, max_h)
            img_label.image = im
            img_label.configure(image=im)

    return outer, inner, img_label

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
    return None  # HUMAN or invalid

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
def bind_topology_controls(parent_frame, topology_img_label, img_max_w=800, img_max_h=400):
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

    add_ws_btn = ttk.Button(ws_controls, text="Add Workstation")
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

    add_tr_btn = ttk.Button(tr_controls, text="Add Conveyor Belt")
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
        """Load './images/topology.jpg' into the left-top preview."""
        try:
            stop_gif(topology_img_label)
            img = load_image("./images/topology.jpg", img_max_w, img_max_h)
            topology_img_label.image = img
            topology_img_label.configure(image=img)
        except Exception as e:
            messagebox.showerror("Preview error", f"Failed to load topology image: {e}")

    def update_ws_id_options():
        display = [f"{entry['id']} — {entry['role']}" for entry in ws_registry]
        from_cb["values"] = display
        to_cb["values"] = display

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
                problem.make_ws_topology(location="./images/topology.jpg")
                preview_topology()
            except Exception as e:
                messagebox.showerror("Problem error", f"Failed to add human: {e}")
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

def bind_premade_topology(parent_frame, topology_img_label, img_max_w=800, img_max_h=400):
    """
    Buttons that apply a predefined layout and refresh the left-top preview.
    Uses GRID consistently to avoid pack/grid conflicts.
    """
    # Make parent_frame grid-expandable
    parent_frame.columnconfigure(0, weight=1)
    parent_frame.rowconfigure(0, weight=1)

    # Row container (grid into parent_frame)
    row = ttk.Frame(parent_frame, padding=8)
    row.grid(row=0, column=0, sticky="nsew")

    # 3 equal columns in 'row'
    row.columnconfigure(0, weight=1, uniform="layouts")
    row.columnconfigure(1, weight=1, uniform="layouts")
    row.columnconfigure(2, weight=1, uniform="layouts")

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
                    img_copy = pil_image.copy()
                    img_copy.thumbnail((w, h), Image.LANCZOS)
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
        except Exception as e:
            messagebox.showerror("Layout error", f"Failed to apply layout: {e}")

    def apply_layout_1():
        global problem
        problem = FJTransportProblem(symmetry_breaking=True)
        problem.add_human()
        problem.set_dur_hum(HUMAN_JOBS_TIME)

        id_kit = problem.add_workstation(WS_KITTING)
        id_grip = problem.add_workstation(WS_BUILDING_1)
        id_gs = problem.add_workstation(WS_BUILDING_2)
        id_pal = problem.add_workstation(WS_PALLETTING)

        problem.add_transport(id_kit, id_grip)
        problem.add_transport(id_grip, id_gs)
        problem.add_transport(id_gs, id_pal)

    def apply_layout_2():
        global problem
        problem = FJTransportProblem(symmetry_breaking=True)
        problem.add_human()
        problem.set_dur_hum(HUMAN_JOBS_TIME)

        id_kit = problem.add_workstation(WS_KITTING)
        id_grip = problem.add_workstation(WS_BUILDING_1)
        id_gs = problem.add_workstation(WS_BUILDING_2)
        id_pal = problem.add_workstation(WS_PALLETTING)

        problem.add_transport(id_kit, id_grip)
        problem.add_transport(id_kit, id_gs)
        problem.add_transport(id_grip, id_pal)
        problem.add_transport(id_gs, id_pal)

    def apply_layout_3():
        global problem
        problem = FJTransportProblem(symmetry_breaking=True)
        problem.add_human()
        problem.set_dur_hum(HUMAN_JOBS_TIME)

        id_kit1 = problem.add_workstation(WS_KITTING)
        id_kit2 = problem.add_workstation(WS_KITTING)
        id_grip = problem.add_workstation(WS_BUILDING_1)
        id_gs = problem.add_workstation(WS_BUILDING_2)
        id_pal = problem.add_workstation(WS_PALLETTING)

        problem.add_transport(id_kit1, id_grip)
        problem.add_transport(id_kit2, id_grip)
        problem.add_transport(id_grip, id_gs)
        problem.add_transport(id_gs, id_pal)



    # Buttons fill their grid cells
    ttk.Button(row, text="Layout 1", command=lambda: run_layout(apply_layout_1)).pack(side="left", expand=True,
                                                                                      fill="both", padx=6, pady=6,
                                                                                      ipady=8)
    ttk.Button(row, text="Layout 2", command=lambda: run_layout(apply_layout_2)).pack(side="left", expand=True,
                                                                                      fill="both", padx=6, pady=6,
                                                                                      ipady=8)
    ttk.Button(row, text="Layout 3", command=lambda: run_layout(apply_layout_3)).pack(side="left", expand=True,
                                                                                      fill="both", padx=6, pady=6,
                                                                                      ipady=8)


# =====================================
# Product selection (items to craft) — totals only
# =====================================
def bind_product_selection(parent_frame):
    """
    Adds 'product selection' controls + totals table.
    Enforces max 6 per item overall.
    """
    ITEMS_MAP = {
        "FLASHLIGHT_CLIPPED": FLASHLIGHT_CLIPPED,
        "FLASHLIGHT_SCREWS": FLASHLIGHT_SCREWS,
    }
    max_per_item = 6
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

    add_item_btn = ttk.Button(row, text="Add Items")
    add_item_btn.grid(row=0, column=4, padx=16, pady=5, sticky="w")

    table = ttk.Treeview(parent_frame, columns=("item", "total"), show="headings", height=2)
    table.pack(fill="both", expand=True, padx=8, pady=(8, 0))
    for col, w in [("item", 220), ("total", 90)]:
        table.heading(col, text=col.upper())
        table.column(col, width=w, anchor="w", stretch=True)

    for name in ITEMS_MAP.keys():
        table.insert("", "end", iid=name, values=(name, 0))

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
        try:
            problem.add_items_to_build(ITEMS_MAP[name], allowed)
        except Exception as e:
            messagebox.showerror("Problem error", f"Failed to add items: {e}")
            return

        totals[name] += allowed
        table.item(name, values=(name, totals[name]))
        if allowed < qty:
            messagebox.showinfo("Adjusted", f"Only {allowed} were added to stay within the limit of {max_per_item}.")

    add_item_btn.configure(command=add_items)

def bind_solving_controls(parent_frame, solution_img_label, img_max_w=800, img_max_h=400):
    """
    Adds a big 'Solve' button that runs the solver and updates the Solution image.
    """
    solve_btn = ttk.Button(parent_frame, text="Solve", style="Solve.TButton")
    solve_btn.pack(expand=True, fill="both", padx=20, pady=20, ipady=40)

    def run_solver():
        try:
            problem.model_problem()
            problem.solve(solver="ortools")
            problem.make_gantt(folder="./images/sol.jpg")

            # Load solution image into bottom-left Solution preview
            stop_gif(solution_img_label)
            pil_img = Image.open("./images/sol.jpg")
            solution_img_label._pil_image = pil_img

            def fit_image_to_label(label, pil_image):
                if pil_image is None:
                    return
                w, h = label.winfo_width(), label.winfo_height()
                if w > 1 and h > 1:
                    img_copy = pil_image.copy()
                    img_copy.thumbnail((w, h), Image.LANCZOS)
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

        except Exception as e:
            messagebox.showerror("Solve error", f"Failed to solve: {e}")

    solve_btn.configure(command=run_solver)

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
    style.configure("Solve.TButton", font=("Helvetica", 16, "bold"))

    container = ttk.Frame(root, padding=12)
    container.pack(fill="both", expand=True)

    # Two columns
    container.columnconfigure(0, weight=1)
    container.columnconfigure(1, weight=1)

    # LEFT COLUMN (topology preview + solution preview)
    left_frame = ttk.Frame(container)
    left_frame.grid(row=0, column=0, sticky="nsew")

    # Top-left: starts with waiting.gif, later replaced with ./images/topology.jpg
    sec1, _inner1, topology_img_label = make_section(
        left_frame, "Workstation Topology", "images/waiting.gif", 800, 400
    )
    sec1.pack(side="top", expand=True, fill="both", pady=6)

    # Bottom-left: starts with waiting.gif, later replaced with ./images/sol.jpg
    sec2, _inner2, solution_img_label = make_section(
        left_frame, "Solution", "images/waiting.gif", 800, 400
    )
    sec2.pack(side="top", expand=True, fill="both", pady=6)

    # RIGHT COLUMN (4 rows)
    right_frame = ttk.Frame(container)
    right_frame.grid(row=0, column=1, sticky="nsew")
    right_frame.columnconfigure(0, weight=1)
    right_frame.rowconfigure(0, weight=4)  # Topology Creation
    right_frame.rowconfigure(1, weight=2)  # Pre-made topology
    right_frame.rowconfigure(2, weight=2)  # Product selection
    right_frame.rowconfigure(3, weight=1)  # Solving

    # Row 0: Topology Creation (buttons: Add Workstation / Add Conveyor Belt / items under this moved out)
    frame_topology_creation, topology_inner, _ = make_section(right_frame, "Topology Creation", border=True)
    frame_topology_creation.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
    bind_topology_controls(topology_inner, topology_img_label, img_max_w=800, img_max_h=400)

    # Row 1: Pre-made topology (Layout 1/2/3) that rebuilds a new problem and refreshes left-top image
    frame_premade, premade_inner, _ = make_section(right_frame, "Pre-made topology", border=True)
    frame_premade.grid(row=1, column=0, sticky="nsew", padx=6, pady=6)
    bind_premade_topology(premade_inner, topology_img_label, img_max_w=800, img_max_h=400)

    # Row 2: Product selection (totals-only table + add items)
    frame_products, products_inner, _ = make_section(right_frame, "Product selection", border=True)
    frame_products.grid(row=2, column=0, sticky="nsew", padx=6, pady=6)
    bind_product_selection(products_inner)

    # Row 3: Solving (big Solve button -> updates bottom-left Solution image)
    frame_solving, solving_inner, _ = make_section(right_frame, "Solving", border=True)
    frame_solving.grid(row=3, column=0, sticky="nsew", padx=6, pady=6)
    bind_solving_controls(solving_inner, solution_img_label, img_max_w=800, img_max_h=400)

    root.minsize(1100, 750)
    root.mainloop()

if __name__ == "__main__":
    main()
