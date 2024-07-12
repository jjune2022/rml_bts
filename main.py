# built-in
import os
import copy
import pickle
import logging
import argparse
import numpy as np
import gurobipy as gp
from gurobipy import *

# my own
from planner import *
from visualization import visualization

# initialize logger
logger = logging.getLogger("Logistics optimization for single vehicle")
logger.setLevel(logging.DEBUG)

handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)

formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
handler.setFormatter(formatter)
logger.addHandler(handler)

# status dictionary
status_dict = {1: "loaded",
               2: "optimal",
               3: "infeasible",
               4: "infeasible and unbounded",
               5: "unbounded",
               6: "cut off",
               7: "iteration limit",
               8: "node limit",
               9: "time limit",
               10: "solution limit",
               11: "interrupted",
               12: "numeric",
               13: "suboptimal",
               14: "in progress",
               15: "user objective limit",
               16: "work limit",
               17: "memory limit"}

def cal_travel_time_mat(locs):
    mat = np.zeros((len(locs), len(locs)))

    # planner
    planner = RoutingPlanner(0, 1, 0)   # get the shortest path

    for f_idx, frm in enumerate(locs):
        for t_idx, to in enumerate(locs):
            planner.init()
            mat[f_idx][t_idx] = planner.astar(frm, to)

    return mat

def create_data_model(call, locs, mat, num_veh, penalty):
    data = {}

    data["num_vehicles"] = num_veh
    data["num_stops"] = len(call)
    data["cost"] = mat
    data["stops"] = locs
    data["indices_stops"] = {k: v["loc"] for k, v in call.items()}
    data["stops_indices"] = {v["loc"]: k for k, v in call.items()}
    data["capacity"] = {k: v["cap"] for k, v in call.items()}

    # set depot (artificial location)
    depot = 0
    data["depot"] = 0
    data["num_stops"] = data["num_stops"] + 1
    data["stops"].append(0)
    data["indices_stops"][0] = 0
    data["stops_indices"][0] = 0
    data["capacity"][0] = 0

    # arch's cost for a depot
    c_0 = np.zeros((1, data["cost"].shape[0]))
    data["cost"] = np.vstack((c_0, data["cost"]))
    c_0 = np.zeros((1, data["cost"].shape[0]))
    data["cost"] = np.concatenate((c_0.T, data["cost"]), axis=1)

    data["penalty"] = penalty

    return data

def optimize(data, working_time=None, capacity=None, penalty=None):
    # define and initialize the optimal model
    m = gp.Model()                  # minimize is default
    m.Params.outputFlag = False

    num_v = data["num_vehicles"]
    n = data["num_stops"]

    # re-define distance matrix
    dist = {}
    d = {}
    for i, row in enumerate(data["cost"]):
        for j, elem in enumerate(row):
            for v in range(data["num_vehicles"]):
                if (i != j):
                    dist[(i, j, v)] = data["cost"][i][j]
                    d[(i, j, v)] = 0
    p = {}
    p_ = {}
    for ni in range(n):
        for v in range(data["num_vehicles"]):
            p[(ni, v)] = -1 * data["capacity"][ni]
        if ni != 0:
            p_[ni] = penalty

    # edge
    e_vars = m.addVars(d.keys(), obj=d, vtype=GRB.BINARY, name="e")

    # prize
    p_vars = m.addVars(p.keys(), obj=p, vtype=GRB.BINARY, name="p")

    # penalty
    p_vars_ = m.addVars(p_.keys(), obj=p_, vtype=GRB.BINARY, name="p_")

    # Constraint 1: only one vehicle can visit one stop except for the depot.
    cons1 = m.addConstrs(p_vars.sum(i, "*") <= 1 for i in range(n) if i != 0)
    # Constraint 2: visited node i must have an outgoing edge.
    cons2 = m.addConstrs(e_vars.sum(i, "*", v) == p_vars[(i, v)]
                         for i in range(n) for v in range(num_v))
    # Constraint 3: visited node j must an ingoing edge.
    cons3 = m.addConstrs(e_vars.sum("*", j, v) == p_vars[(j, v)]
                         for j in range(n) for v in range(num_v))
    # Constraint 4: considering the origin.
    cons4 = m.addConstr(p_vars.sum(0, "*") == num_v)
    # Constraint 5: working time limit
    if working_time:
        cons5 = m.addConstr(gp.quicksum(e_vars[i, j, v] * dist[(i, j, v)]
                                        for i in range(n) for j in range(n) for v in range(num_v)
                                        if i != j) <= working_time)
    if capacity:
        cons6 = m.addConstr(gp.quicksum(-1 * p_vars[i, v] * p[(i, v)] for i in range(n) for v in range(num_v))
                             <= capacity)
    if penalty:
        cons7 = m.addConstrs(1 - p_vars.sum(i, "*") == p_vars_[i] for i in range(n) if i != 0)

    def subtourlim(model, where):
        if where == GRB.Callback.MIPSOL:
            # make a list of edges selected in the solution
            vals = model.cbGetSolution(model._vars)
            selected = gp.tuplelist((i, j, k) for i, j, k in model._vars.keys() if vals[i, j, k] > 0.5)
            # find the shortest cycle in the selected edge list
            tour = subtour(selected)
            for v in range(num_v):
                if tour[v]:
                    for tv in tour[v]:
                        if len(tv) < n:
                            # add subtour elimination constraint for every pair of cities in tour
                            model.cbLazy(gp.quicksum(model._vars[i, j, v] for i, j in itertools.permutations(tv, 2))
                                         <= len(tv) - 1)

    def subtour(edges, exclude_depot=True):
        cycle = [[] for v in range(num_v)]

        for v in range(num_v):
            unvisited = list(np.arange(0, n))

            while unvisited:  # true if list is non-empty
                this_cycle = []
                neighbors = unvisited

                while neighbors:
                    current = neighbors[0]
                    this_cycle.append(current)
                    unvisited.remove(current)
                    neighbors = [j for i, j, k in edges.select(current, '*', '*') if (j in unvisited) and (k == v)]

                if len(this_cycle) > 1:
                    if exclude_depot:
                        if not (data["depot"] in this_cycle):
                            cycle[v].append(this_cycle)
        return cycle

    # optimize model
    m._vars = e_vars
    m._dvars = p_vars
    m._ddvars = p_vars_
    m.Params.lazyConstraints = 1
    m.optimize(subtourlim)

    # status
    logger.info("Solve the optimization problem")

    # status
    logger.info("Status of solution is " + str(status_dict[m.status]))

    e_vals = m.getAttr('x', e_vars)

    sol = {}
    for car in range(num_v):
        sol[car] = {}
        for i, j, k in e_vals.keys():
            if (e_vals[i, j, k] > 0.5) and (k == car):
                sol[k][i] = j

    routes = []
    pack = []
    tt = []
    for car in range(num_v):
        route = sol[car]
        i = 0
        num_package = 0
        travel_time = 0
        path = []
        while True:
            i_ = copy.copy(i)
            i = route[i]
            num_package += data["capacity"][i]
            travel_time += data["cost"][i_][i]
            if i == 0:
                break
            path.append(data["indices_stops"][i])
        routes.append(path)
        pack.append(num_package)
        tt.append(travel_time)

    return routes, pack, tt


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Write delivery calls, working time of a deliver, "
                                                 "capacity of a vehicle for delivery, and penalty given "
                                                 "for not visiting a location")
    parser.add_argument("-call", "--call", type=int, required=True, nargs="+", action="append",
                        help="write location id and capacity")
    parser.add_argument("-t", "--time", type=int, required=False,
                        help="working time of a delivery in seconds")
    parser.add_argument("-cap", "--capacity", type=int, required=False,
                        help="capacity of a vehicle for delivery")
    parser.add_argument("-penalty", "--penalty", type=int, required=True,
                        help="penalty given for not visiting a location")

    args = parser.parse_args()

    # make problem
    num_veh = 1     # number of vehicles
    penalty = args.penalty  # penalty given for not visiting a location

    # problem setting
    delivery_call = {idx + 1: {"loc": call[0], "cap": call[1]}
        for idx, call in enumerate(args.call)
    }

    locs = list(map(lambda x: x["loc"], delivery_call.values()))

    if os.path.isfile("./matrix.pickle"):
        with open("./matrix.pickle", "rb") as file:
            matrix = pickle.load(file)
    else:
        matrix = cal_travel_time_mat(locs)
        with open("./matrix.pickle", "wb") as file:
            pickle.dump(matrix, file)

    data = create_data_model(delivery_call, locs, matrix, num_veh, penalty)

    # optimization
    routes, pack, tt = optimize(data, working_time=args.time, capacity=args.capacity, penalty=penalty)

    logger.info("The route is " + str(routes[0]))
    logger.info("The number of serviced packages is " + str(pack[0]))
    logger.info("The total traveling time is " + str(tt[0]))
    logger.info("The number of visited locations is " + str(len(routes[0])))

    planner = RoutingPlanner(0, 1, 0)
    visualization(planner, routes[0])