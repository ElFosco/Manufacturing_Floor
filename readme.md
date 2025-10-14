# 🏭 Factory Jobshop Scheduling Problem

This repository models and optimizes a **Flexible Jobshop Scheduling Problem** (FJSP) with **transport decisions**.  


---

## 🧩 Problem Description

The **Flexible Jobshop Transport Problem** extends the classical jobshop scheduling problem by incorporating **transport times** and **multiple resource types** (humans, robots, conveyors).  

Each product (or *item*) consists of a sequence of **operations** across different **stages**:

- **Kitting** → **Assembly** → **Packing**

Transitions between stages (and between internal sub-stages) require transport, which can be done by:
- a **human operator**
- a **robot**
- or a **conveyor belt** (if available between workstations)

The goal is typically to **minimize the makespan** (total completion time), possibly combined with minimizing:
- the number of active resources,
- the number of conveyors used (`tr_count`).

---

## 🚀 `app.py` — Running the Application

The entry point of the repository is **`app.py`**, which provides an **interactive interface** for running and visualizing solutions.

### 🔧 Key Features
- 🧠 **Model execution:** build and solve jobshop instances with configurable resources and items.  
- 🎯 **Multi-objective optimization:** compare scheduling strategies under various objective combinations.  
- 📊 **Visualization tools:** render workstation layouts, human/robot assignments, and conveyor topologies using **Plotly**.  
- 🔍 **Scenario exploration:** examine the impact of resource availability or conveyor connectivity on system performance.
