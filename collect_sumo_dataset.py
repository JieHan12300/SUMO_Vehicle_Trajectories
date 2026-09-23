#!/usr/bin/env python3
"""Generate a one-lane SUMO scenario and stream reproducible trajectory data.

Python >= 3.9 + an installed SUMO (netconvert and tools/traci).
See README.md for scenario details and running instructions.
"""

import argparse
import csv
import gzip
import importlib
import json
import math
import os
from pathlib import Path
import random
import shutil
import statistics
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime

MILE_M = 1609.344
MPH_MPS = 0.44704
VEHICLE_LENGTH_M = 5.0
TRAJECTORY_COLUMNS = [
    "time_s", "vehicle_id", "x_m", "y_m", "lane_id", "position_m",
    "distance_m", "speed_mps", "speed_mph", "acceleration_mps2",
    "target_speed_next_step_mps", "leader_id", "net_gap_m",
    "space_headway_m", "following_headway_s", "in_demand_window",
    "min_gap_m", "target_tau_next_step_s", "desired_gap_at_current_speed_m",
]


@dataclass
class Vehicle:
    vehicle_id: str
    scheduled_depart_s: float
    sampled_headway_s: float
    preferred_speed_mps: float
    min_gap_m: float
    preferred_tau_s: float
    actual_depart_s: float = None
    actual_entry_headway_s: float = None
    sample_count: int = 0


def arguments():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, default=None, help="New/empty output directory")
    p.add_argument("--sumo-home", type=Path, default=None)
    p.add_argument("--duration-hours", type=float, default=6.0)
    p.add_argument("--road-miles", type=float, default=10.0)
    p.add_argument("--traffic-lights", type=int, choices=[0, 2, 3], default=0,
                   help="Number of traffic lights; default 0 generates one uninterrupted road")
    p.add_argument("--green-seconds", type=float, default=45.0)
    p.add_argument("--yellow-seconds", type=float, default=4.0)
    p.add_argument("--red-seconds", type=float, default=35.0)
    p.add_argument("--step-length", type=float, default=1.0, help="Simulation step in seconds")
    p.add_argument("--sample-interval", type=float, default=1.0, help="CSV sampling interval in seconds")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--headway-dist", choices=["gaussian", "shifted-exponential", "uniform", "lognormal"],
                   default="gaussian")
    p.add_argument("--mean-headway", type=float, default=6.0, help="Gaussian mean before truncation, s")
    p.add_argument("--headway-sigma", type=float, default=1.5)
    p.add_argument("--min-headway", type=float, default=2.0, help="Lower bound on scheduled headway, s")
    p.add_argument("--max-headway", type=float, default=12.0, help="Upper bound for Gaussian headways, s")
    p.add_argument("--lognormal-sigma", type=float, default=0.6, help="Log-space SD of excess headway")
    p.add_argument("--preferred-min-mph", type=float, default=40.0)
    p.add_argument("--preferred-max-mph", type=float, default=60.0)
    p.add_argument("--preferred-mean-mph", type=float, default=50.0)
    p.add_argument("--preferred-sigma-mph", type=float, default=5.0)
    p.add_argument("--speed-min-mph", type=float, default=20.0, help="Lower bound on requested speed, not actual speed")
    p.add_argument("--speed-max-mph", type=float, default=65.0)
    p.add_argument("--speed-sigma-mph", type=float, default=4.0, help="Unclipped stationary SD of random speed target")
    p.add_argument("--speed-update", type=float, default=5.0, help="Target speed update interval, s")
    p.add_argument("--speed-correlation", type=float, default=30.0, help="Mean reversion time scale, s")
    p.add_argument("--tau", type=float, default=1.8, help="Mean desired following time gap, s")
    p.add_argument("--tau-sigma", type=float, default=0.3, help="Between-vehicle SD, s")
    p.add_argument("--tau-noise", type=float, default=0.2, help="Within-trip unbounded stationary SD, s")
    p.add_argument("--tau-min", type=float, default=1.0)
    p.add_argument("--tau-max", type=float, default=2.8)
    p.add_argument("--min-gap", type=float, default=3.0, help="Mean standstill gap, m; Gaussian truncated to [1,6]")
    p.add_argument("--gap-sigma", type=float, default=0.75)
    p.add_argument("--accel", type=float, default=1.0, help="Maximum acceleration, m/s^2")
    p.add_argument("--complete-trips", action="store_true", help="Stop demand at 6h, then drain the road")
    p.add_argument("--max-drain-hours", type=float, default=3.0)
    p.add_argument("--split-by-vehicle", action="store_true", help="Also write one CSV per vehicle")
    p.add_argument("--gui", action="store_true", help="Run sumo-gui under Python control")
    p.add_argument("--prepare-only", action="store_true", help="Generate scenario without running it")
    a = p.parse_args()
    positive = ["duration_hours", "road_miles", "step_length", "sample_interval",
                "mean_headway", "min_headway", "lognormal_sigma", "speed_update",
                "speed_correlation", "tau", "max_drain_hours", "speed_min_mph",
                "green_seconds", "yellow_seconds", "red_seconds", "accel"]
    for name in positive:
        if not math.isfinite(getattr(a, name)) or getattr(a, name) <= 0:
            p.error(f"--{name.replace('_', '-')} must be finite and positive")
    if not 0 < a.min_headway < a.mean_headway < a.max_headway < math.inf:
        p.error("Require 0 < min-headway < mean-headway < max-headway")
    if not (a.speed_min_mph <= a.preferred_min_mph <= a.preferred_max_mph <= a.speed_max_mph
            and math.isfinite(a.speed_max_mph)):
        p.error("Require speed-min <= preferred-min <= preferred-max <= speed-max")
    for name in ["speed_sigma_mph", "preferred_sigma_mph", "headway_sigma", "gap_sigma", "tau_sigma", "tau_noise"]:
        if not math.isfinite(getattr(a, name)) or getattr(a, name) < 0:
            p.error(f"{name.replace('_', '-')} must be finite and nonnegative")
    if not a.preferred_min_mph <= a.preferred_mean_mph <= a.preferred_max_mph:
        p.error("preferred-mean-mph must be inside preferred speed bounds")
    if a.preferred_min_mph == a.preferred_max_mph and a.preferred_sigma_mph > 0:
        p.error("Equal preferred speed bounds require preferred-sigma-mph=0")
    if not (a.step_length <= a.tau_min <= a.tau <= a.tau_max < math.inf and a.tau_min < a.tau_max):
        p.error("Require step-length <= tau-min <= tau <= tau-max, with tau-min < tau-max")
    if not 1 <= a.min_gap <= 6:
        p.error("min-gap must be within [1,6] m")
    if a.seed < 0 or a.seed > 2147483647:
        p.error("seed must be in [0, 2147483647]")
    if a.tau < a.step_length:
        p.error("tau must be >= step-length for safe car following")
    if a.road_miles * MILE_M < 100:
        p.error("Use a road at least 100 m long")
    if a.step_length < 0.001 or not math.isclose(a.step_length * 1000, round(a.step_length * 1000), abs_tol=1e-7):
        p.error("SUMO step-length must be an integer number of milliseconds")
    for label, value in [("sample-interval", a.sample_interval), ("speed-update", a.speed_update),
                         ("duration-hours * 3600", a.duration_hours * 3600),
                         ("max-drain-hours * 3600", a.max_drain_hours * 3600)]:
        ratio = value / a.step_length
        if ratio < 1 or not math.isclose(ratio, round(ratio), rel_tol=0, abs_tol=1e-6):
            p.error(f"{label} must be an integer multiple of step-length")
    return a


def sumo_installation(a):
    home = a.sumo_home or (Path(os.environ["SUMO_HOME"]) if os.environ.get("SUMO_HOME") else None)
    if home is None and shutil.which("sumo"):
        home = Path(shutil.which("sumo")).resolve().parent.parent
    exe_suffix = ".exe" if os.name == "nt" else ""

    def binary(name):
        candidate = home / "bin" / (name + exe_suffix) if home else None
        if candidate and candidate.is_file():
            return str(candidate)
        if a.sumo_home:
            raise RuntimeError(f"Missing {name} in specified SUMO installation: {home}")
        result = shutil.which(name)
        if not result:
            raise RuntimeError(f"Cannot find {name}. Set SUMO_HOME or pass --sumo-home.")
        return result

    if home and (home / "tools").is_dir():
        sys.path.insert(0, str(home / "tools"))
    return binary("sumo-gui" if a.gui else "sumo"), binary("netconvert")


def save_xml(path, root):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def sample_headway(rng, a):
    if a.headway_dist == "gaussian":
        return truncated_gaussian(rng, a.mean_headway, a.headway_sigma, a.min_headway, a.max_headway)
    excess_mean = a.mean_headway - a.min_headway
    if a.headway_dist == "shifted-exponential":
        excess = rng.expovariate(1.0 / excess_mean)
    elif a.headway_dist == "uniform":
        excess = rng.uniform(0, 2 * excess_mean)
    else:
        mu = math.log(excess_mean) - 0.5 * a.lognormal_sigma ** 2
        excess = rng.lognormvariate(mu, a.lognormal_sigma)
    return a.min_headway + excess


def truncated_gaussian(rng, mean, sigma, lower, upper):
    # Rejection sampling avoids artificial probability masses at the bounds.
    if sigma == 0:
        return min(upper, max(lower, mean))
    for _ in range(100000):
        value = rng.gauss(mean, sigma)
        if lower <= value <= upper:
            return value
    raise ValueError("Gaussian sampling failed; check the standard deviation and bounds")


def make_demand(a):
    # Separate random streams: changing speed parameters does not change arrivals.
    arrival_rng = random.Random(a.seed)
    preference_rng = random.Random(a.seed + 1)
    following_rng = random.Random(a.seed + 3)
    vehicles = {}
    scheduled = 0.0
    while True:
        headway = sample_headway(arrival_rng, a)
        scheduled += headway  # The first vehicle also has a random arrival time.
        if scheduled >= a.duration_hours * 3600:
            break
        vehicle_id = f"veh_{len(vehicles):06d}"
        vehicles[vehicle_id] = Vehicle(
            vehicle_id, scheduled, headway,
            truncated_gaussian(preference_rng, a.preferred_mean_mph, a.preferred_sigma_mph,
                               a.preferred_min_mph, a.preferred_max_mph) * MPH_MPS,
            truncated_gaussian(following_rng, a.min_gap, a.gap_sigma, 1.0, 6.0),
            truncated_gaussian(following_rng, a.tau, a.tau_sigma, a.tau_min, a.tau_max),
        )
    return vehicles


def make_scenario(a, out, netconvert, vehicles):
    road_m = a.road_miles * MILE_M
    nodes = ET.Element("nodes")
    ET.SubElement(nodes, "node", id="start", x="0", y="0", type="priority")
    for i in range(a.traffic_lights):
        ET.SubElement(nodes, "node", id=f"tls_{i}", x=f"{road_m * (i+1)/(a.traffic_lights+1):.6f}",
                      y="0", type="traffic_light")
    ET.SubElement(nodes, "node", id="end", x=f"{road_m:.6f}", y="0", type="priority")
    save_xml(out / "road.nod.xml", nodes)
    edges = ET.Element("edges")
    node_ids = ["start"] + [f"tls_{i}" for i in range(a.traffic_lights)] + ["end"]
    edge_ids = [f"road_{i:03d}" for i in range(a.traffic_lights+1)]
    for i, edge in enumerate(edge_ids):
        ET.SubElement(edges, "edge", {"id": edge, "from": node_ids[i], "to": node_ids[i+1],
                      "numLanes": "1", "speed": str(a.speed_max_mph * MPH_MPS),
                      "length": f"{road_m/(a.traffic_lights+1):.6f}", "spreadType": "center"})
    save_xml(out / "road.edg.xml", edges)
    connections = ET.Element("connections")
    for i in range(a.traffic_lights):
        ET.SubElement(connections, "connection", {"from": edge_ids[i], "to": edge_ids[i+1],
                      "fromLane": "0", "toLane": "0"})
    save_xml(out / "road.con.xml", connections)
    tls = ET.Element("tlLogics")
    cycle = a.green_seconds + a.yellow_seconds + a.red_seconds
    for i in range(a.traffic_lights):
        logic = ET.SubElement(tls, "tlLogic", id=f"tls_{i}", type="static", programID="0",
                              offset=str(i * cycle / a.traffic_lights))
        for duration, state in [(a.green_seconds, "G"), (a.yellow_seconds, "y"), (a.red_seconds, "r")]:
            ET.SubElement(logic, "phase", duration=str(duration), state=state)
        ET.SubElement(tls, "connection", {"from": edge_ids[i], "to": edge_ids[i+1],
                      "fromLane": "0", "toLane": "0", "tl": f"tls_{i}", "linkIndex": "0"})
    save_xml(out / "signals.tll.xml", tls)
    # Windows SUMO may decode command-line paths with the ANSI code page.
    # Relative ASCII filenames + a Unicode-aware subprocess cwd avoid that issue.
    result = subprocess.run([netconvert, "--node-files", "road.nod.xml",
                             "--edge-files", "road.edg.xml",
                             "--connection-files", "road.con.xml",
                             "--tllogic-files", "signals.tll.xml",
                             "--output-file", "road.net.xml",
                             "--no-turnarounds", "true", "--no-internal-links", "true",
                             "--precision", "6"], cwd=out, capture_output=True, text=True, errors="replace")
    (out / "netconvert.log").write_text(result.stdout + result.stderr, encoding="utf-8")
    if result.returncode:
        raise RuntimeError(f"netconvert failed: {result.stderr}")
    net = ET.parse(out / "road.net.xml").getroot()
    lanes = net.findall("./edge/lane")
    if (len(lanes) != len(edge_ids) or len(net.findall("tlLogic")) != a.traffic_lights
            or any(len(edge.findall("lane")) != 1 for edge in net.findall("edge"))
            or abs(sum(float(lane.get("length")) for lane in lanes) - road_m) > 0.01):
        raise RuntimeError("Generated network failed length/lane/traffic-light checks")

    routes = ET.Element("routes")
    for v in vehicles.values():
        # Individual types apply random gaps before SUMO attempts safe insertion.
        ET.SubElement(routes, "vType", id=f"type_{v.vehicle_id}", vClass="passenger", carFollowModel="Krauss",
                      length=str(VEHICLE_LENGTH_M), minGap=str(v.min_gap_m), accel=str(a.accel), decel="4.5",
                      emergencyDecel="9.0", tau=str(v.preferred_tau_s), sigma="0.3", speedFactor="1.0",
                      speedDev="0", maxSpeed=str(a.speed_max_mph * MPH_MPS))
    ET.SubElement(routes, "route", id="main_route", edges=" ".join(edge_ids))
    with (out / "demand.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["vehicle_id", "scheduled_depart_s", "sampled_headway_s", "preferred_speed_mps",
                         "min_gap_m", "preferred_tau_s"])
        for v in vehicles.values():
            writer.writerow([v.vehicle_id, f"{v.scheduled_depart_s:.6f}",
                             f"{v.sampled_headway_s:.6f}", f"{v.preferred_speed_mps:.6f}",
                             f"{v.min_gap_m:.6f}", f"{v.preferred_tau_s:.6f}"])
            # A numeric depart speed can hold up the entrance until that exact speed is safe.
            # 'max' permits the fastest safe insertion at the fixed origin, including lower speeds.
            ET.SubElement(routes, "vehicle", id=v.vehicle_id, type=f"type_{v.vehicle_id}", route="main_route",
                          depart=f"{v.scheduled_depart_s:.6f}", departLane="0", departPos="0",
                          departSpeed="max", arrivalPos="max")
    save_xml(out / "demand.rou.xml", routes)
    config = ET.Element("configuration")
    inputs = ET.SubElement(config, "input")
    ET.SubElement(inputs, "net-file", value="road.net.xml")
    ET.SubElement(inputs, "route-files", value="demand.rou.xml")
    timing = ET.SubElement(config, "time")
    ET.SubElement(timing, "begin", value="0")
    ET.SubElement(timing, "end", value=str(a.duration_hours * 3600 +
                  (a.max_drain_hours * 3600 if a.complete_trips else 0)))
    ET.SubElement(timing, "step-length", value=str(a.step_length))
    processing = ET.SubElement(config, "processing")
    for key, value in [("time-to-teleport", "-1"), ("collision.action", "warn"), ("max-depart-delay", "-1")]:
        ET.SubElement(processing, key, value=value)
    output = ET.SubElement(config, "output")
    for key, value in [("tripinfo-output", "tripinfo.xml"), ("tripinfo-output.write-unfinished", "true"),
                       ("tripinfo-output.write-undeparted", "true"), ("precision", "6")]:
        ET.SubElement(output, key, value=value)
    randomness = ET.SubElement(config, "random_number")
    ET.SubElement(randomness, "seed", value=str(a.seed))
    save_xml(out / "scenario.sumocfg", config)


class TrajectoryWriter:
    def __init__(self, out, split):
        self.stream = gzip.open(out / "trajectories.csv.gz", "wt", encoding="utf-8", newline="", compresslevel=1)
        self.writer = csv.writer(self.stream)
        self.writer.writerow(TRAJECTORY_COLUMNS)
        self.vehicle_dir = out / "vehicles" if split else None
        if self.vehicle_dir:
            self.vehicle_dir.mkdir()
        self.rows = 0

    def write(self, rows):
        self.writer.writerows(rows)
        self.rows += len(rows)
        if self.vehicle_dir:
            # Open only briefly: no thousands of simultaneous Windows file handles.
            for row in rows:
                path = self.vehicle_dir / f"{row[1]}.csv"
                is_new = not path.exists()
                with path.open("a", encoding="utf-8", newline="") as f:
                    writer = csv.writer(f)
                    if is_new:
                        writer.writerow(TRAJECTORY_COLUMNS)
                    writer.writerow(row)

    def close(self):
        self.stream.close()


def simulate(a, out, sumo, vehicles):
    try:
        traci = importlib.import_module("traci")
        tc = importlib.import_module("traci.constants")
    except ImportError as e:
        raise RuntimeError("Cannot import traci. Set --sumo-home or install: python -m pip install traci") from e
    end_steps = round(a.duration_hours * 3600 / a.step_length)
    limit_steps = end_steps + (round(a.max_drain_hours * 3600 / a.step_length) if a.complete_trips else 0)
    sample_steps = round(a.sample_interval / a.step_length)
    update_steps = round(a.speed_update / a.step_length)
    rho = math.exp(-a.speed_update / a.speed_correlation)
    noise_scale = a.speed_sigma_mph * MPH_MPS * math.sqrt(1 - rho * rho)
    target_rng = random.Random(a.seed + 2)
    tau_rng = random.Random(a.seed + 4)
    tau_noise_scale = a.tau_noise * math.sqrt(1 - rho * rho)
    targets = {}
    taus = {}
    next_updates = {}
    lane_offsets, offset = {}, 0.0
    network = ET.parse(out / "road.net.xml").getroot()
    for edge in sorted(network.findall("edge"), key=lambda e: e.get("id")):
        lane = edge.find("lane")
        lane_offsets[lane.get("id")] = offset
        offset += float(lane.get("length"))
    light_ids = sorted(logic.get("id") for logic in network.findall("tlLogic"))
    departed_ids, arrived_ids = set(), set()
    writer = TrajectoryWriter(out, a.split_by_vehicle)
    light_stream = (out / "traffic_lights.csv").open("w", encoding="utf-8", newline="")
    light_writer = csv.writer(light_stream)
    light_writer.writerow(["time_s", "signal_id", "position_m", "state", "phase_index"])
    status, error = "running", None
    now, last_entry, max_active = 0.0, None, 0
    collision_events, teleport_events = 0, 0
    stopped_vehicle_seconds = 0.0
    connected = False
    version = None
    start_wall = time.perf_counter()
    progress_steps = max(1, round(600 / a.step_length))
    try:
        cmd = [sumo, "-c", "scenario.sumocfg", "--no-step-log", "true",
               "--duration-log.disable", "true", "--log", "sumo.log"]
        if a.gui:
            cmd += ["--start", "--quit-on-end"]
        previous_cwd = Path.cwd()
        try:
            os.chdir(out)
            traci.start(cmd, numRetries=5)
        finally:
            os.chdir(previous_cwd)
        connected = True
        version = traci.getVersion()
        traci.simulation.subscribe([tc.VAR_DEPARTED_VEHICLES_IDS, tc.VAR_ARRIVED_VEHICLES_IDS,
                                    tc.VAR_COLLIDING_VEHICLES_IDS, tc.VAR_TELEPORT_STARTING_VEHICLES_IDS])
        vehicle_vars = [tc.VAR_POSITION, tc.VAR_LANEPOSITION, tc.VAR_SPEED,
                        tc.VAR_ACCELERATION, tc.VAR_DISTANCE, tc.VAR_LANE_ID]
        for light in light_ids:
            traci.trafficlight.subscribe(light, [tc.TL_RED_YELLOW_GREEN_STATE, tc.TL_CURRENT_PHASE])
        for step in range(1, limit_steps + 1):
            traci.simulationStep()
            now = step * a.step_length
            events = traci.simulation.getSubscriptionResults()
            collided = events[tc.VAR_COLLIDING_VEHICLES_IDS]
            teleported = events[tc.VAR_TELEPORT_STARTING_VEHICLES_IDS]
            collision_events += len(collided)
            teleport_events += len(teleported)
            if collided or teleported:
                raise RuntimeError(f"Invalid trajectory event at {now}s: collisions={collided}, teleports={teleported}")
            for vehicle_id in events[tc.VAR_ARRIVED_VEHICLES_IDS]:
                arrived_ids.add(vehicle_id)
                targets.pop(vehicle_id, None)
                taus.pop(vehicle_id, None)
                next_updates.pop(vehicle_id, None)
            new_ids = events[tc.VAR_DEPARTED_VEHICLES_IDS]
            # A single lane and fixed insertion position make departures ordered.
            for vehicle_id in sorted(new_ids):
                v = vehicles[vehicle_id]
                v.actual_depart_s = traci.vehicle.getDeparture(vehicle_id)
                v.actual_entry_headway_s = None if last_entry is None else v.actual_depart_s - last_entry
                last_entry = v.actual_depart_s
                departed_ids.add(vehicle_id)
                traci.vehicle.subscribe(vehicle_id, vehicle_vars)
                traci.vehicle.setSpeedMode(vehicle_id, 31)  # Keep safe speed and accel/decel checks.
                targets[vehicle_id] = v.preferred_speed_mps
                taus[vehicle_id] = v.preferred_tau_s
                traci.vehicle.setSpeed(vehicle_id, targets[vehicle_id])
                next_updates[vehicle_id] = step + update_steps
            states = traci.vehicle.getAllSubscriptionResults()
            # Deterministic id order is important for assigning seeded random draws.
            for vehicle_id in sorted(targets):
                if step >= next_updates[vehicle_id]:
                    mean = vehicles[vehicle_id].preferred_speed_mps
                    targets[vehicle_id] = truncated_gaussian(target_rng,
                        mean + rho * (targets[vehicle_id] - mean), noise_scale,
                        a.speed_min_mph * MPH_MPS, a.speed_max_mph * MPH_MPS)
                    traci.vehicle.setSpeed(vehicle_id, targets[vehicle_id])
                    mean_tau = vehicles[vehicle_id].preferred_tau_s
                    taus[vehicle_id] = truncated_gaussian(tau_rng,
                        mean_tau + rho * (taus[vehicle_id] - mean_tau), tau_noise_scale, a.tau_min, a.tau_max)
                    traci.vehicle.setTau(vehicle_id, taus[vehicle_id])
                    next_updates[vehicle_id] = step + update_steps
            max_active = max(max_active, len(states))
            stopped_vehicle_seconds += sum(s[tc.VAR_SPEED] < 0.1 for s in states.values()) * a.step_length
            if step % sample_steps == 0:
                for i, light in enumerate(light_ids):
                    state = traci.trafficlight.getSubscriptionResults(light)
                    light_writer.writerow([f"{now:.6f}", light,
                        f"{a.road_miles*MILE_M*(i+1)/(a.traffic_lights+1):.6f}",
                        state[tc.TL_RED_YELLOW_GREEN_STATE], state[tc.TL_CURRENT_PHASE]])
                rows = []
                leader_id, leader_pos = "", None
                # A continuous single-lane route: local lane position resets at each light.
                # Global position keeps leader/gap and downstream traffic-field data continuous.
                global_position = lambda state: lane_offsets[state[tc.VAR_LANE_ID]] + state[tc.VAR_LANEPOSITION]
                for vehicle_id, state in sorted(states.items(), key=lambda item: global_position(item[1]), reverse=True):
                    position = global_position(state)
                    speed = state[tc.VAR_SPEED]
                    space = None if leader_pos is None else leader_pos - position
                    gap = None if space is None else space - VEHICLE_LENGTH_M
                    headway = None if space is None or speed <= 0.1 else space / speed
                    x, y = state[tc.VAR_POSITION]
                    def f(value):
                        return "" if value is None else f"{value:.6f}"
                    rows.append([f(now), vehicle_id, f(x), f(y), state[tc.VAR_LANE_ID], f(position),
                                 f(state[tc.VAR_DISTANCE]), f(speed), f(speed / MPH_MPS),
                                 f(state[tc.VAR_ACCELERATION]), f(targets[vehicle_id]), leader_id,
                                 f(gap), f(space), f(headway), int(step <= end_steps),
                                 f(vehicles[vehicle_id].min_gap_m), f(taus[vehicle_id]),
                                 f(vehicles[vehicle_id].min_gap_m + taus[vehicle_id]*speed)])
                    vehicles[vehicle_id].sample_count += 1
                    leader_id, leader_pos = vehicle_id, position
                writer.write(rows)
            if step % progress_steps == 0:
                print(f"t={now / 3600:.2f} h | entered={len(departed_ids)} | arrived={len(arrived_ids)} "
                      f"| active={len(states)} | rows={writer.rows:,}", flush=True)
            if a.complete_trips and step >= end_steps and len(arrived_ids) == len(vehicles):
                break
        status = "drain_limit_reached" if a.complete_trips and len(arrived_ids) < len(vehicles) else "completed"
    except BaseException as e:
        status, error = "failed", str(e) or type(e).__name__
        raise
    finally:
        try:
            if connected:
                traci.close()
        finally:
            writer.close()
            light_stream.close()
            summary = {
                "status": status, "error": error, "sumo_version": version,
                "simulation_end_s": now, "demand_end_s": a.duration_hours * 3600,
                "road_length_m": a.road_miles * MILE_M, "planned_vehicles": len(vehicles),
                "traffic_lights": a.traffic_lights, "stopped_vehicle_seconds": stopped_vehicle_seconds,
                "departed_vehicles": len(departed_ids), "arrived_vehicles": len(arrived_ids),
                "unfinished_vehicles": len(departed_ids - arrived_ids),
                "not_departed_vehicles": len(vehicles) - len(departed_ids),
                "max_active_vehicles": max_active, "trajectory_rows": writer.rows,
                "collision_vehicle_events": collision_events, "teleport_vehicle_events": teleport_events,
                "wall_time_seconds": round(time.perf_counter() - start_wall, 3),
            }
            write_vehicle_summary(out, vehicles, arrived_ids)
            scheduled = [v.sampled_headway_s for v in vehicles.values()]
            actual = [v.actual_entry_headway_s for v in vehicles.values() if v.actual_entry_headway_s is not None]
            summary["sampled_mean_headway_s"] = statistics.mean(scheduled) if scheduled else None
            summary["actual_mean_entry_headway_s"] = statistics.mean(actual) if actual else None
            (out / "run_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")
    return summary


def write_vehicle_summary(out, vehicles, arrived_ids):
    trips = {}
    path = out / "tripinfo.xml"
    if path.exists():
        try:
            for _, element in ET.iterparse(path, events=("end",)):
                if element.tag == "tripinfo":
                    trips[element.get("id")] = dict(element.attrib)
                element.clear()
        except ET.ParseError:
            pass  # An interrupted run can leave incomplete XML; keep Python's observations.
    columns = ["vehicle_id", "status", "scheduled_depart_s", "sampled_headway_s", "actual_depart_s",
               "depart_delay_s", "actual_entry_headway_s", "arrival_s", "duration_s", "route_length_m",
               "waiting_time_s", "time_loss_s", "preferred_speed_mps", "trajectory_samples",
               "min_gap_m", "preferred_tau_s"]
    with (out / "vehicle_summary.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(columns)
        for v in vehicles.values():
            trip = trips.get(v.vehicle_id, {})
            completed = v.vehicle_id in arrived_ids
            state = "completed" if completed else ("unfinished" if v.actual_depart_s is not None else "not_departed")
            delay = None if v.actual_depart_s is None else v.actual_depart_s - v.scheduled_depart_s
            row = [v.vehicle_id, state, v.scheduled_depart_s, v.sampled_headway_s, v.actual_depart_s,
                   delay, v.actual_entry_headway_s, trip.get("arrival", "") if completed else "",
                   trip.get("duration", ""), trip.get("routeLength", ""), trip.get("waitingTime", ""),
                   trip.get("timeLoss", ""), v.preferred_speed_mps, v.sample_count, v.min_gap_m, v.preferred_tau_s]
            writer.writerow([f"{value:.6f}" if isinstance(value, float) else value for value in row])


def main():
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="backslashreplace")
    a = arguments()
    sumo, netconvert = sumo_installation(a)
    out = (a.output or Path("outputs") / datetime.now().strftime("run_%Y%m%d_%H%M%S_%f")).resolve()
    if out.exists() and any(out.iterdir()):
        raise RuntimeError(f"Output directory must be empty (existing data are preserved): {out}")
    out.mkdir(parents=True, exist_ok=True)
    parameters = {key: str(value) if isinstance(value, Path) else value for key, value in vars(a).items()}
    parameters.update(output=str(out), sumo_binary=sumo, netconvert_binary=netconvert,
                      python_version=sys.version, script_sha256=__import__("hashlib").sha256(Path(__file__).read_bytes()).hexdigest())
    (out / "parameters.json").write_text(json.dumps(parameters, ensure_ascii=False, indent=2), encoding="utf-8")
    vehicles = make_demand(a)
    make_scenario(a, out, netconvert, vehicles)
    print(f"Output: {out}\nRoad: {a.road_miles} miles; demand: {a.duration_hours} hours; planned vehicles: {len(vehicles)}", flush=True)
    if a.prepare_only:
        print("Scenario prepared. Run this Python script without --prepare-only into a new output directory to collect data.")
        return 0
    summary = simulate(a, out, sumo, vehicles)
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    return 0 if summary["status"] == "completed" else 2


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
