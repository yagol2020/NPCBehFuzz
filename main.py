import json
import os
import shutil
import time
from collections import deque
from datetime import datetime
from types import SimpleNamespace

import carla
import random
import loguru
import subprocess

import math
from carla import LaneType
from deap import base, tools, algorithms

import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd
import sys

from my_scenarios import Scenario


def mutation(scenario):
    global sps
    if random.random() < 0.5:
        scenario.ego_sp_index = random.randint(0, len(sps) - 1)
    else:
        scenario.ego_dp_index = random.randint(0, len(sps) - 1)
    return (scenario,)


def mate(scenario1, scenario2):
    scenario1.ego_sp_index, scenario2.ego_sp_index = scenario2.ego_sp_index, scenario1.ego_sp_index
    return scenario1, scenario2


@loguru.logger.catch()
def main():
    global sps
    method = sys.argv[1]
    npc_num = int(sys.argv[2])
    print(method, npc_num)
    time.sleep(10)
    start_time = time.time()
    local_time = time.localtime(start_time) 
    formatted_time = time.strftime("%Y_%m_%d_%H_%M", local_time)
    os.mkdir(f"results/{method}_npc{npc_num}_{formatted_time}")
    json.dump(formatted_time, open(f"results/{method}_npc{npc_num}_{formatted_time}/start_fuzzing_time.json", "w"))
    toolbox = base.Toolbox()
    toolbox.register("mutate", mutation)
    toolbox.register("mate", mate)
    toolbox.register("select", tools.selNSGA2)
    try:
        client = carla.Client("localhost", 2000)
        client.set_timeout(10)
        world = client.load_world("Town05")
        setting = world.get_settings()
        setting.synchronous_mode = True
        setting.fixed_delta_seconds = 1 / 20
        world.apply_settings(setting)
        world.tick()
    except:
        subprocess.Popen(["./run_carla.sh", "2000", "False"])
        time.sleep(5)
        client = carla.Client("localhost", 2000)
        client.set_timeout(10)
        world = client.load_world("Town05")
        setting = world.get_settings()
        setting.synchronous_mode = True
        setting.fixed_delta_seconds = 1 / 20
        world.apply_settings(setting)
        world.tick()
    pop_num = 50
    ads_name = "interfuser"
    sps = world.get_map().get_spawn_points()
    current_generation_num = 0
    scenarios = []
    for index in range(pop_num):
        ego_sp_index, egp_dp_index = random.sample(range(len(sps)), k=2)
        s = Scenario()
        s.init_with_data(basic_result_path=f"results/{method}_npc{npc_num}_{formatted_time}",
                         ego_sp_index=ego_sp_index,
                         ego_dp_index=egp_dp_index, ads_name=ads_name, gid=current_generation_num, iid=index,
                         method=method, npc_num=npc_num)
        if not os.path.exists(s.result_path):
            os.mkdir(s.result_path)
        s.to_file()
        os.system(f"python runner.py {s.result_path}/scenario_ready_to_run.json")
        if not os.path.exists(s.result_path + "/simulation_ret.json"):
            continue
        ret = json.load(open(s.result_path + "/simulation_ret.json"))
        obj_values, npc_p_list, collision_detail = ret["obj_values"], ret["npc_p_list"], ret["collision_detail"]
        s.fitness.values = (
            obj_values["min_dis"], obj_values["min_ttc"], obj_values["action_num"], obj_values["moving_rate"])
        print(
            s.fitness
        )
        s.npc_p_list = npc_p_list
        s.collision_detail = collision_detail
        s.save_scenario()
        scenarios.append(s)
    while time.time() - start_time <= 5 * 60 * 60:
        current_generation_num += 1
        offspring_scenarios = algorithms.varOr(
            scenarios, toolbox, lambda_=pop_num, cxpb=0.5, mutpb=0.5
        )
        for index in range(pop_num):
            s = offspring_scenarios[index]
            s.gid = current_generation_num
            s.iid = index
            s.result_path = s.change_result_path(current_generation_num, index)
            s.method = method
            if not os.path.exists(s.result_path):
                os.mkdir(s.result_path)
            s.to_file()
            s.npc_p_list = []
            s.collision_detail = []
            os.system(f"python runner.py {s.result_path}/scenario_ready_to_run.json")
            if not os.path.exists(s.result_path + "/simulation_ret.json"):
                continue
            ret = json.load(open(s.result_path + "/simulation_ret.json"))
            obj_values, npc_p_list, collision_detail = ret["obj_values"], ret["npc_p_list"], ret["collision_detail"]
            s.fitness.values = (
                obj_values["min_dis"], obj_values["min_ttc"], obj_values["action_num"], obj_values["moving_rate"])
            print(s.fitness)
            s.npc_p_list = npc_p_list
            s.collision_detail = collision_detail
            s.save_scenario()
        scenarios = toolbox.select(scenarios + offspring_scenarios, pop_num)
    loguru.logger.success("End of main")


if __name__ == '__main__':
    main()
