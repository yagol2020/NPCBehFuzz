import json
import os
import random
import shutil
import subprocess
import sys
import time
from collections import deque
from types import SimpleNamespace
import loguru
import math
import carla
from agents.navigation.local_planner import RoadOption
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_agent import BehaviorAgent
from InterFuser.team_code.interfuser_agent import InterfuserAgent
from InterFuser.leaderboard.utils.route_manipulation import (
    interpolate_trajectory as interpolate_trajectory_interfuser,
)
from InterFuser.leaderboard.autoagents.agent_wrapper import (
    AgentWrapper as AgentWrapper_interfuser,
)
from InterFuser.srunner.scenariomanager.timer import (
    GameTime as GameTime_interfuser,
)
from InterFuser.srunner.scenariomanager.carla_data_provider import (
    CarlaDataProvider as CarlaDataProvider_interfuser,
)
from my_scenarios import load_scenario_from_file


def _on_invasion(event, state):
    crossed_lanes = event.crossed_lane_markings
    for crossed_lane in crossed_lanes:
        if crossed_lane.lane_change == carla.LaneChange.NONE and crossed_lane.type != carla.LaneMarkingType.NONE:
            detail_info = {
                "frame": event.frame,
                "timestamp": event.timestamp,
                "transform": {
                    "x": event.transform.location.x,
                    "y": event.transform.location.y,
                    "z": event.transform.location.z,
                }
            }
            state.laneinvasion_details.append(detail_info)


def _on_collision(event, state):
    if event.other_actor.type_id != "static.road":
        loguru.logger.error(f"Collision! with {event.other_actor}", event)
        state.collision_flag = True
        detail_info = {}
        detail_info["frame"] = event.frame
        detail_info["timestamp"] = event.timestamp
        detail_info["transform"] = {
            "x": event.transform.location.x,
            "y": event.transform.location.y,
            "z": event.transform.location.z,
        }
        detail_info["other_actor"] = event.other_actor.type_id
        normal_impulse = event.normal_impulse
        impulse_vector = carla.Vector3D(normal_impulse.x, normal_impulse.y, normal_impulse.z)

        vehicle_transform = event.transform
        forward_vector = vehicle_transform.get_forward_vector()  
        right_vector = vehicle_transform.get_right_vector()  


        impulse_magnitude = impulse_vector.length()
        if impulse_magnitude > 0:

            normalized_impulse = impulse_vector / impulse_magnitude

            dot_product = (normalized_impulse.x * forward_vector.x +
                           normalized_impulse.y * forward_vector.y +
                           normalized_impulse.z * forward_vector.z)

            angle = math.acos(dot_product)

            angle_deg = math.degrees(angle)


            if angle_deg < 45:
                detail_info["part"] = "top"
            elif angle_deg > 135:
                detail_info["part"] = "tail"
            else:
                side_dot_product = (normalized_impulse.x * right_vector.x +
                                    normalized_impulse.y * right_vector.y +
                                    normalized_impulse.z * right_vector.z)
                if side_dot_product > 0:
                    detail_info["part"] = "right"
                else:
                    detail_info["part"] = "left"
        else:
        state.collision_detail.append(detail_info)


def _on_front_camera_capture(image):
    image.save_to_disk(f"./fuzzerdata/front-{image.frame}.jpg")


def _on_top_camera_capture(image):
    image.save_to_disk(f"./fuzzerdata/top-{image.frame}.jpg")


def save_video(ads_name):
    if ads_name == "behavior":
        print("Saving front camera video", end=" ")
        vid_filename = f"./fuzzerdata/front.mp4"
        if os.path.exists(vid_filename):
            os.remove(vid_filename)
        cmd_cat = "ls ./fuzzerdata/front-*.jpg | sort -V | xargs -I {} cat {}"
        cmd_ffmpeg = " ".join([
            "ffmpeg",
            "-f image2pipe",
            f"-r 20",
            "-vcodec mjpeg",
            "-i -",
            "-vcodec libx264",
            vid_filename
        ])

        cmd = f"{cmd_cat} | {cmd_ffmpeg} 2> /dev/null"
        os.system(cmd)
        print("(done)")

        cmd = "rm -f ./fuzzerdata/front-*.jpg"
        os.system(cmd)

    print("Saving top camera video", end=" ")

    vid_filename = f"./fuzzerdata/top.mp4"
    if os.path.exists(vid_filename):
        os.remove(vid_filename)

    cmd_cat = "ls ./fuzzerdata/top-*.jpg | sort -V | xargs -I {} cat {}"
    cmd_ffmpeg = " ".join([
        "ffmpeg",
        "-f image2pipe",
        f"-r 20",
        "-vcodec mjpeg",
        "-i -",
        "-vcodec libx264",
        vid_filename
    ])

    cmd = f"{cmd_cat} | {cmd_ffmpeg} 2> /dev/null"
    os.system(cmd)
    print("(done)")

    cmd = "rm -f ./fuzzerdata/top-*.jpg"
    os.system(cmd)


class Road:
    def __init__(self, road_id, lane_id, junction_id):
        self.road_id = road_id
        self.lane_id = lane_id
        self.junction_id = junction_id


def get_same_direction_neighbor_lane(current_waypoint):
    current_lane_id = current_waypoint.lane_id
    current_direction = 1 if current_lane_id > 0 else -1

    left_lane = current_waypoint.get_left_lane()
    if left_lane and left_lane.lane_id * current_direction > 0:
        return left_lane, "left" 

    right_lane = current_waypoint.get_right_lane()
    if right_lane and right_lane.lane_id * current_direction > 0:
        return right_lane, "right" 
    return None, None


def multi_point(waypoints_list):
    wps = waypoints_list
    if isinstance(wps, deque) or isinstance(wps[0], tuple):
        wps = [t[0] for t in wps]
    for w_id, w in enumerate(wps):
        if isinstance(w, carla.Transform):
            world.debug.draw_string(
                w.location, 
                f"WP{w_id}", 
                draw_shadow=True,  
                color=carla.Color(r=255, g=0, b=0), 
                life_time=10000  
            )
        if isinstance(w, carla.Waypoint):
            world.debug.draw_string(
                w.transform.location, 
                f"WP{w_id}",  
                draw_shadow=True, 
                color=carla.Color(r=255, g=0, b=0), 
                life_time=10000
            )
    world.tick()


def point(waypoint, message="P"):
    if isinstance(waypoint, carla.Waypoint):
        loc = waypoint.transform.location
    if isinstance(waypoint, carla.Transform):
        loc = waypoint.location
    if isinstance(waypoint, tuple):
        loc = waypoint[0].transform.location
    world.debug.draw_string(
        loc,  
        message,  
        draw_shadow=True, 
        color=carla.Color(r=255, g=0, b=0),  
        life_time=10000 
    )
    world.tick()


def remove_duplicate_waypoints(waypoints_list, coord_tolerance=0.1, angle_tolerance=10):
    import math
    seen = []

    def is_similar(wp1, wp2):
        coord_close = all([
            math.isclose(wp1.location.x, wp2.location.x, abs_tol=coord_tolerance),
            math.isclose(wp1.location.y, wp2.location.y, abs_tol=coord_tolerance),
            math.isclose(wp1.location.z, wp2.location.z, abs_tol=coord_tolerance)
        ])

        def angle_diff(a, b):
            diff = abs((a % 360) - (b % 360))
            return min(diff, 360 - diff)

        angle_close = all([
            angle_diff(wp1.rotation.pitch, wp2.rotation.pitch) <= angle_tolerance,
            angle_diff(wp1.rotation.yaw, wp2.rotation.yaw) <= angle_tolerance,
            angle_diff(wp1.rotation.roll, wp2.rotation.roll) <= angle_tolerance
        ])

        return coord_close and angle_close

    for wp in waypoints_list:
        if not any(is_similar(wp, seen_wp) for seen_wp in seen):
            seen.append(wp)
    return seen


def angle_diff(a, b):
    diff = (a - b) % 360
    return min(diff, 360 - diff)


def junction_analysis(ego_waypoint):
    junction = ego_waypoint.get_junction()  #
    me_to_opp_waypoint = list() 
    opp_to_me_waypoint_pair = list() 
    left_to_right_waypoint_pair = list()  
    right_to_left_waypoint_pair = list()  
    junction_wps = junction.get_waypoints(carla.LaneType.Driving)  

    ego_yaw = ego_waypoint.transform.rotation.yaw

    straight_pairs = [
        pair for pair in junction_wps
        if angle_diff(pair[0].transform.rotation.yaw, pair[1].transform.rotation.yaw) < 10
    ]

    for pair in straight_pairs:
        entry_yaw = pair[0].transform.rotation.yaw  
        diff = (entry_yaw - ego_yaw) % 360  


        if diff < 45 or diff > 315:
            me_to_opp_waypoint.append(pair)

        elif 135 < diff < 225:
            opp_to_me_waypoint_pair.append(pair)

        elif 45 < diff < 135:
            left_to_right_waypoint_pair.append(pair)

        elif 225 < diff < 315:
            right_to_left_waypoint_pair.append(pair)

    return me_to_opp_waypoint, opp_to_me_waypoint_pair, left_to_right_waypoint_pair, right_to_left_waypoint_pair


def create_npc_by_interaction_type(interaction_type, ego, sps):
    try:
        npc_bp = world.get_blueprint_library().find("vehicle.tesla.model3")
        r_v, g_b, b_v = random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)
        color_str = f"{r_v},{g_b},{b_v}"
        npc_bp.set_attribute("color", color_str)
        if interaction_type == "other_vehicle_merging": 
            waypoint_neigh, direction = get_same_direction_neighbor_lane(
                world.get_map().get_waypoint(ego.get_location())) 
            if len(waypoint_neigh.previous_until_lane_start(1)) < 5:
                return None, None, None
            if len(world.get_map().get_waypoint(ego.get_location()).next_until_lane_end(1)) < 40:
                return None, None, None
            p1 = random.randint(3, 4)
            start_waypoint = waypoint_neigh.previous_until_lane_start(1)[p1] 
            p2 = random.randint(10, 15)
            mid_waypoint_0 = waypoint_neigh.next_until_lane_end(1)[p2]  
            p3 = random.randint(35, 39)
            mid_waypoint = world.get_map().get_waypoint(ego.get_location()).next_until_lane_end(1)[p3]
            final_transform = random.choice(sps)

            start_transform = start_waypoint.transform
            start_transform.location.z += 0.3
            npc = world.try_spawn_actor(npc_bp, start_transform)
            world.tick()
            if not npc:
                return None, None, None

            ego_velocity = ego.get_velocity()
            ego_speed = (ego_velocity.x ** 2 + ego_velocity.y ** 2 + ego_velocity.z ** 2) ** 0.5  
            p4 = random.randint(20, 25)
            npc_speed = ego_speed + p4  

            transform = npc.get_transform().get_forward_vector()
            velocity = carla.Vector3D(transform.x * npc_speed, transform.y * npc_speed,
                                      transform.z * npc_speed)  
            npc.set_target_velocity(velocity)

            npc_agent = BehaviorAgent(npc, behavior="aggressive")
            first_seg = npc_agent.trace_route(start_waypoint, mid_waypoint_0)
            first_seg_1 = npc_agent.trace_route(mid_waypoint_0, mid_waypoint)
            second_seg = npc_agent.trace_route(mid_waypoint, world.get_map().get_waypoint(final_transform.location))
            total_seg = first_seg + first_seg_1 + second_seg
            npc_agent.set_global_plan(total_seg)
            p_list = [p1, p2, p3, p4]
            return npc, npc_agent, p_list
        elif interaction_type == "rear_vehicle_overtaking":  
            ego_lane_waypoint = world.get_map().get_waypoint(ego.get_location())
            if len(ego_lane_waypoint.previous_until_lane_start(1)) < 15: 
                return None, None, None
            if len(ego_lane_waypoint.next_until_lane_end(1)) < 50:  
                return None, None, None
            start_waypoint = ego_lane_waypoint.previous_until_lane_start(1)[random.randint(7, 8)]  # start
            if start_waypoint.transform.location.distance(ego.get_location()) > 50:
                return None, None, None
            waypoint_neigh, direction = get_same_direction_neighbor_lane(ego_lane_waypoint)
            p1 = random.randint(1, 2)
            mid_waypoint = waypoint_neigh.next_until_lane_end(1)[p1]  # p1
            p2 = random.randint(15, 16)
            mid_waypoint_2 = mid_waypoint.next_until_lane_end(1)[p2]  # p2
            p3 = random.randint(45, 49)
            final_transform = ego_lane_waypoint.next_until_lane_end(1)[p3].transform  # p3
            p4 = random.randint(0, len(sps) - 1)
            final_2_transform = sps[p4]  # final
            start_transform = start_waypoint.transform
            start_transform.location.z += 0.3
            if direction == "left":
                start_transform.rotation.yaw -= 15
            elif direction == "right":
                start_transform.rotation.yaw += 15
            npc = world.try_spawn_actor(npc_bp, start_transform)
            world.tick()
            if not npc:
                return None, None, None

            ego_velocity = ego.get_velocity()
            ego_speed = (ego_velocity.x ** 2 + ego_velocity.y ** 2 + ego_velocity.z ** 2) ** 0.5 
            p5 = random.randint(10, 15)
            npc_speed = ego_speed + p5

            transform = npc.get_transform().get_forward_vector()
            velocity = carla.Vector3D(transform.x * npc_speed, transform.y * npc_speed,
                                      transform.z * npc_speed)  
            npc.set_target_velocity(velocity)

            npc_agent = BehaviorAgent(npc, behavior="aggressive")
            first_seg = npc_agent.trace_route(start_waypoint, mid_waypoint)
            first_seg_2 = npc_agent.trace_route(mid_waypoint, mid_waypoint_2)
            second_seg = npc_agent.trace_route(mid_waypoint_2, world.get_map().get_waypoint(final_transform.location))
            third_seg = npc_agent.trace_route(world.get_map().get_waypoint(final_transform.location),
                                              world.get_map().get_waypoint(final_2_transform.location))
            total_seg = first_seg + first_seg_2 + second_seg + third_seg
            npc_agent.set_global_plan(total_seg)
            npc_control = npc_agent.run_step()
            npc.apply_control(npc_control)
            return npc, npc_agent, [p1, p2, p3, p4, p5]
        elif interaction_type == "real_vehicle_accelerating_during_lane_change":  #
            waypoint_neigh, direction = get_same_direction_neighbor_lane(
                world.get_map().get_waypoint(ego.get_location()))
            if len(waypoint_neigh.previous_until_lane_start(1)) <= 20: 
             
                return None, None, None
            if len(waypoint_neigh.next_until_lane_end(1)) <= 5: 
              
                return None, None, None
            p1 = random.randint(15, 20)
            start_waypoint = waypoint_neigh.previous_until_lane_start(1)[p1] 
            p2 = random.randint(0, 5)
            mid_waypoint = waypoint_neigh.next_until_lane_end(1)[p2] 
            p3 = random.randint(0, len(sps) - 1)
            final_transform = sps[p3]
            start_transform = start_waypoint.transform
            start_transform.location.z += 0.3
            npc = world.try_spawn_actor(npc_bp, start_transform)
            world.tick()
            if not npc:
                return None, None, None

            ego_velocity = ego.get_velocity()
            ego_speed = (ego_velocity.x ** 2 + ego_velocity.y ** 2 + ego_velocity.z ** 2) ** 0.5 
            p4 = random.randint(5, 10)
            npc_speed = ego_speed + p4 

            transform = npc.get_transform().get_forward_vector()
            velocity = carla.Vector3D(transform.x * npc_speed, transform.y * npc_speed,
                                      transform.z * npc_speed) 
            npc.set_target_velocity(velocity)

            npc_agent = BehaviorAgent(npc, behavior="aggressive")
            first_seg = npc_agent.trace_route(start_waypoint, mid_waypoint)
            second_seg = npc_agent.trace_route(mid_waypoint, world.get_map().get_waypoint(final_transform.location))
            total_seg = first_seg + second_seg
            npc_agent.set_global_plan(total_seg)

            return npc, npc_agent, [p1, p2, p3, p4]
        elif interaction_type == "opposite_straight_junction_left": 
            ego_lane_waypoint = world.get_map().get_waypoint(ego.get_location())

            me_to_opp_waypoint, opp_to_me_waypoint_pair, left_to_right_waypoint_pair, right_to_left_waypoint_pair = junction_analysis(
                ego_lane_waypoint)
            if len(opp_to_me_waypoint_pair) == 0:
                return None, None, None
            p1 = random.randint(0, len(opp_to_me_waypoint_pair) - 1)
            adv_enter_point, adv_exit_point = opp_to_me_waypoint_pair[p1]

            if len(adv_enter_point.previous(5)) < 1:

                return None, None, None
            adv_enter_point = adv_enter_point.previous(5)[0]

            start_transform = adv_enter_point.transform
            start_transform.location.z += 0.3

            npc = world.try_spawn_actor(npc_bp, start_transform)
            world.tick()
            if not npc:
                loguru.logger.error("在世界中，生成NPC车辆失败")
                return None, None, None

            ego_velocity = ego.get_velocity()
            ego_speed = (ego_velocity.x ** 2 + ego_velocity.y ** 2 + ego_velocity.z ** 2) ** 0.5  
            p2 = random.randint(5, 10)
            npc_speed = ego_speed + p2 

            transform = npc.get_transform().get_forward_vector()
            velocity = carla.Vector3D(transform.x * npc_speed, transform.y * npc_speed,
                                      transform.z * npc_speed) 
            npc.set_target_velocity(velocity)

            npc_agent = BehaviorAgent(npc, behavior="aggressive")
            first_seg = npc_agent.trace_route(adv_enter_point, adv_exit_point)
            total_seg = first_seg
            npc_agent.set_global_plan(total_seg)
            return npc, npc_agent, [p1, p2]
        elif interaction_type == "opposite_right_junction_left": 
            ego_lane_waypoint = world.get_map().get_waypoint(ego.get_location())
            me_to_opp_waypoint, opp_to_me_waypoint_pair, left_to_right_waypoint_pair, right_to_left_waypoint_pair = junction_analysis(
                ego_lane_waypoint)
            if len(opp_to_me_waypoint_pair) == 0 or len(right_to_left_waypoint_pair) == 0:
                return None, None, None
            p1 = random.randint(0, len(opp_to_me_waypoint_pair) - 1)
            p2 = random.randint(0, len(right_to_left_waypoint_pair) - 1)
            adv_enter_point, adv_mid_point = opp_to_me_waypoint_pair[p1][0], right_to_left_waypoint_pair[p2][1]

            if len(adv_enter_point.previous(5)) < 1:

                return None, None, None
            adv_enter_point = adv_enter_point.previous(5)[0]

            start_transform = adv_enter_point.transform
            start_transform.location.z += 0.3

            npc = world.try_spawn_actor(npc_bp, start_transform)
            world.tick()
            if not npc:

                return None, None, None

            ego_velocity = ego.get_velocity()
            ego_speed = (ego_velocity.x ** 2 + ego_velocity.y ** 2 + ego_velocity.z ** 2) ** 0.5 
            p3 = random.randint(0, 5)
            npc_speed = ego_speed + p3  

            transform = npc.get_transform().get_forward_vector()
            velocity = carla.Vector3D(transform.x * npc_speed, transform.y * npc_speed,
                                      transform.z * npc_speed)  
            npc.set_target_velocity(velocity)

            npc_agent = BehaviorAgent(npc, behavior="aggressive")
            first_seg = npc_agent.trace_route(adv_enter_point, adv_mid_point)
            final_transform = random.choice(sps)
            second_seg = npc_agent.trace_route(adv_mid_point, world.get_map().get_waypoint(final_transform.location))
            total_seg = first_seg + second_seg
            npc_agent.set_global_plan(total_seg)
            return npc, npc_agent, [p1, p2, p3]
        elif interaction_type == "straight_junction_right": 
            ego_lane_waypoint = world.get_map().get_waypoint(ego.get_location())
            me_to_opp_waypoint, opp_to_me_waypoint_pair, left_to_right_waypoint_pair, right_to_left_waypoint_pair = junction_analysis(
                ego_lane_waypoint)
            if len(left_to_right_waypoint_pair) == 0:
                return None, None, None
            p1 = random.randint(0, len(left_to_right_waypoint_pair) - 1)
            adv_enter_point, adv_mid_point = left_to_right_waypoint_pair[p1]

            if len(adv_enter_point.next(2)) < 1:

                return None, None, None
            adv_enter_point = adv_enter_point.next(5)[0]

            if len(adv_mid_point.next(5)) < 1:

                return None, None, None
            adv_mid_point = adv_mid_point.next(5)[0]

            start_transform = adv_enter_point.transform
            start_transform.location.z += 0.3

            npc = world.try_spawn_actor(npc_bp, start_transform)
            world.tick()
            if not npc:
                loguru.logger.error("在世界中，生成NPC车辆失败")
                return None, None, None

            ego_velocity = ego.get_velocity()
            ego_speed = (ego_velocity.x ** 2 + ego_velocity.y ** 2 + ego_velocity.z ** 2) ** 0.5  
            p2 = random.randint(0, 10)
            npc_speed = ego_speed + p2 

            transform = npc.get_transform().get_forward_vector()
            velocity = carla.Vector3D(transform.x * npc_speed, transform.y * npc_speed,
                                      transform.z * npc_speed) 
            npc.set_target_velocity(velocity)

            npc_agent = BehaviorAgent(npc, behavior="aggressive")
            first_seg = npc_agent.trace_route(adv_enter_point, adv_mid_point)
            final_transform = random.choice(sps)
            second_seg = npc_agent.trace_route(adv_mid_point, world.get_map().get_waypoint(final_transform.location))
            total_seg = first_seg + second_seg
            npc_agent.set_global_plan(total_seg)
            return npc, npc_agent, [p1, p2]
        elif interaction_type == "me_straight_met_right":
            ego_lane_waypoint = world.get_map().get_waypoint(ego.get_location())
            me_to_opp_waypoint, opp_to_me_waypoint_pair, left_to_right_waypoint_pair, right_to_left_waypoint_pair = junction_analysis(
                ego_lane_waypoint)
            if len(right_to_left_waypoint_pair) == 0 or len(me_to_opp_waypoint) == 0:
                return None, None, None
            p1 = random.randint(0, len(right_to_left_waypoint_pair) - 1)
            p2 = random.randint(0, len(me_to_opp_waypoint) - 1)
            adv_enter_point, adv_mid_point = right_to_left_waypoint_pair[p1][0], \
                me_to_opp_waypoint[p2][1]

            if len(adv_enter_point.previous(5)) < 1:
                return None, None, None
            if len(adv_mid_point.next(5)) < 1:
                return None, None, None
            adv_enter_point = adv_enter_point.previous(5)[0]
            adv_mid_point = adv_mid_point.next(5)[0]

            start_transform = adv_enter_point.transform
            start_transform.location.z += 0.3

            npc = world.try_spawn_actor(npc_bp, start_transform)
            world.tick()
            if not npc:
                loguru.logger.error("在世界中，生成NPC车辆失败")
                return None, None, None

            ego_velocity = ego.get_velocity()
            ego_speed = (ego_velocity.x ** 2 + ego_velocity.y ** 2 + ego_velocity.z ** 2) ** 0.5
            p3 = random.randint(0, 5)
            npc_speed = ego_speed + p3 

            transform = npc.get_transform().get_forward_vector()
            velocity = carla.Vector3D(transform.x * npc_speed, transform.y * npc_speed,
                                      transform.z * npc_speed)  
            npc.set_target_velocity(velocity)

            npc_agent = BehaviorAgent(npc, behavior="aggressive")
            first_seg = npc_agent.trace_route(adv_enter_point, adv_mid_point)
            final_transform = random.choice(sps)
            second_seg = npc_agent.trace_route(adv_mid_point, world.get_map().get_waypoint(final_transform.location))
            total_seg = first_seg + second_seg
            npc_agent.set_global_plan(total_seg)
            return npc, npc_agent, [p1, p2, p3]
    except BaseException as create_error:
        loguru.logger.error(create_error)


def calculate_ttc_dis(vehicle1, vehicle2):
    loc1 = vehicle1.get_location()
    loc2 = vehicle2.get_location()

    distance = loc1.distance(loc2)

    vel1 = vehicle1.get_velocity()
    vel2 = vehicle2.get_velocity()

    vel_rel = vel1 - vel2

    direction = (loc2 - loc1)
    direction_norm = direction.make_unit_vector()

    v_rel = vel_rel.x * direction_norm.x + vel_rel.y * direction_norm.y + vel_rel.z * direction_norm.z

    if v_rel > 0:
        ttc = distance / v_rel
        return ttc, distance
    else:
        return float('inf'), distance  


def check_before_add(max_npc_num, want_to_add, current_npc_list):
    if len(current_npc_list) >= max_npc_num:
       
        return False
    for npc, npc_agent, npc_behavior, idx_npc in current_npc_list:
        if want_to_add == npc_behavior:
            return False
    return True


def calculate_ego_behavior_change(ego_behavior_list):
    ego_behavior_list = [t for t in ego_behavior_list if t != "normal_driving"]
    if len(ego_behavior_list) == 0:
        return []
    unique_behavior = [ego_behavior_list[0]]
    for current_ego_behavior_1 in ego_behavior_list:
        if unique_behavior[-1] == current_ego_behavior_1:
            continue
        else:
            unique_behavior.append(current_ego_behavior_1)
    return unique_behavior


def transform_2_location(transform):
    return carla.Location(
        x=transform.location.x, y=transform.location.y, z=transform.location.z
    )


def run_once(scenario):
    assert len(scenario.npc_p_list) == 0
    assert len(scenario.collision_detail) == 0
    max_npc_num = scenario.npc_num
    sps = world.get_map().get_spawn_points()
    ego_bp = world.get_blueprint_library().find("vehicle.tesla.model3")
    rgb_camera_bp = world.get_blueprint_library().find("sensor.camera.rgb")
    rgb_camera_bp.set_attribute("image_size_x", "800")
    rgb_camera_bp.set_attribute("image_size_y", "600")
    rgb_camera_bp.set_attribute("fov", "105")
    state = SimpleNamespace()
    state.collision_flag = False
    state.collision_detail = []
    state.laneinvasion_details = []
    state.current_ego_behavior = []
    state.location_recorder = []
    state.ego_moving_frame_num = 0
    state.total_frame_num = 0
    state.npc_distance_dict = {}
    state.idx_npc = 0 
    shutil.rmtree("./fuzzerdata")
    os.mkdir("fuzzerdata")
  
    sp_index = scenario.ego_sp_index
    dp_index = scenario.ego_dp_index
   
    ego_sp, ego_dp = sps[sp_index], sps[dp_index]
    print(sp_index, dp_index)
    ego = world.spawn_actor(ego_bp, ego_sp)
    world.tick()
    carla_collision_sensor_bp = world.get_blueprint_library().find(
        "sensor.other.collision"
    )
    carla_collision_sensor = world.spawn_actor(
        carla_collision_sensor_bp, carla.Transform(), attach_to=ego
    )
    carla_collision_sensor.listen(
        lambda event: _on_collision(event, state)
    )
    world.tick()
    ####
    carla_lane_invasion_sensor_bp = world.get_blueprint_library().find(
        "sensor.other.lane_invasion"
    )
    carla_lane_invasion_sensor = world.spawn_actor(
        carla_lane_invasion_sensor_bp, carla.Transform(), attach_to=ego
    )
    carla_lane_invasion_sensor.listen(
        lambda event: _on_invasion(event, state)
    )
    world.tick()
    ######
    if scenario.ads_name == "behavior": 
        camera_tf = carla.Transform(carla.Location(z=1.8))
        camera_front = world.spawn_actor(
            rgb_camera_bp,
            camera_tf,
            attach_to=ego,
            attachment_type=carla.AttachmentType.Rigid
        )
        camera_front.listen(lambda image: _on_front_camera_capture(image))
        world.tick()
    camera_tf = carla.Transform(
        carla.Location(z=50.0),
        carla.Rotation(pitch=-90.0)
    )
    camera_top = world.spawn_actor(
        rgb_camera_bp,
        camera_tf,
        attach_to=ego,
        attachment_type=carla.AttachmentType.Rigid
    )

    camera_top.listen(lambda image: _on_top_camera_capture(image))
    world.tick()
    if scenario.ads_name == "behavior":
        ######################################
        # BehaviorAgent
        ######################################
        ego_agent = BehaviorAgent(ego, behavior="normal")
        ego_agent.set_destination(ego_dp.location)
        world.tick()
        # multi_point(ego_agent.get_local_planner().get_plan())
    elif scenario.ads_name == "interfuser":
        ######################################
        # InterFuser
        ######################################
        CarlaDataProvider_interfuser.set_client(client)
        CarlaDataProvider_interfuser.set_world(world)
        ego_agent = InterfuserAgent("InterFuser/team_code/interfuser_config.py")
        trajectory = [transform_2_location(ego_sp), transform_2_location(ego_dp)]
        gps_route, route = interpolate_trajectory_interfuser(world, trajectory)
        # multi_point(route)
        ego_agent.set_global_plan(gps_route, route)
        agent_wrapper = AgentWrapper_interfuser(ego_agent)
        agent_wrapper.setup_sensors(ego)
        world.tick()

    ######################################
    traj = []
    cur_wp = world.get_map().get_waypoint(ego.get_location())
    road_id = cur_wp.road_id
    lane_id = cur_wp.lane_id
    junction_id = getattr(cur_wp.get_junction(), "id", None)
    traj.append(Road(road_id, lane_id, junction_id))
    spec = world.get_spectator()
    npc_list = []
    npc_p_list = []

    ######################################

    obj_values = SimpleNamespace()
    obj_values.min_dis = 9999
    obj_values.min_ttc = 9999
    obj_values.action_num = 0
    obj_values.moving_rate = 0

    ######################################
    simulation_start_time = time.time()
    while True:
        if state.collision_flag:

            break
        if time.time() - simulation_start_time > 5 * 60:

            break
        world.tick()
        snapshot = world.get_snapshot()
        snapshot_timestamp = snapshot.timestamp
        state.total_frame_num = snapshot.frame
        if scenario.ads_name == "interfuser":
            GameTime_interfuser.on_carla_tick(snapshot_timestamp)
        if scenario.ads_name == "behavior":
            if ego_agent.done() or ego_dp.location.distance(ego.get_location()) < 5:

                distance_to_goal = ego_dp.location.distance(ego.get_location())

                break
        elif scenario.ads_name == "interfuser":
            if ego_agent.initialized and len(ego_agent._route_planner.route) == 0 or ego_dp.location.distance(
                    ego.get_location()) < 5:

                distance_to_goal = ego_dp.location.distance(ego.get_location())

                break
        remaining_npc_list = []
        location_current_infos = {}
        for npc, npc_agent, behavior_name, idx_npc in npc_list:
            if scenario.method != "random" and (
                    npc.get_location().distance(ego.get_location()) > 50 or npc_agent.done()):
                npc.destroy()
                continue
            if scenario.method == "random" and npc_agent.done():
                npc.destroy()
                continue
            #######
            ttc, dis = calculate_ttc_dis(ego, npc)
            loguru.logger.debug(f"{ttc}, {dis}")
            ##
            dis_list = state.npc_distance_dict.get(idx_npc, [])
            dis_list.append({
                "ttc": ttc,
                "dis": dis,
            })
            state.npc_distance_dict[idx_npc] = dis_list
            ##
            location_current_infos[idx_npc] = {
                "x": npc.get_location().x, "y": npc.get_location().y, "z": npc.get_location().z,
            }
            ##

            obj_values.min_ttc = min(obj_values.min_ttc, ttc)
            obj_values.min_dis = min(obj_values.min_dis, dis)
            #######
            npc_control = npc_agent.run_step()
            if behavior_name == "rear_vehicle_overtaking": 
                ego_velocity = ego.get_velocity()
                ego_speed = (ego_velocity.x ** 2 + ego_velocity.y ** 2 + ego_velocity.z ** 2) ** 0.5  #
                npc_speed = ego_speed + random.randint(1, 2)

                transform = npc.get_transform().get_forward_vector()
                velocity = carla.Vector3D(transform.x * npc_speed, transform.y * npc_speed,
                                          transform.z * npc_speed) 
                npc.set_target_velocity(velocity)
            npc.apply_control(npc_control)
            remaining_npc_list.append((npc, npc_agent, behavior_name, idx_npc))
        npc_list[:] = remaining_npc_list
        if scenario.ads_name == "behavior":
            control = ego_agent.run_step()
            ego.apply_control(control)
        elif scenario.ads_name == "interfuser":
            control = ego_agent()
            ego.apply_control(control)
        cur_speed = math.sqrt(ego.get_velocity().x ** 2 + ego.get_velocity().y ** 2)
        loguru.logger.debug(f"Speed: {cur_speed} km/h")
        location_current_infos["ego"] = {
            "x": ego.get_location().x, "y": ego.get_location().y, "z": ego.get_location().z,
        }
        state.location_recorder.append(location_current_infos)
        if cur_speed > 1:
            state.ego_moving_frame_num += 1
        cur_wp = world.get_map().get_waypoint(ego.get_location())
        road_id = cur_wp.road_id
        lane_id = cur_wp.lane_id
        junction_id = getattr(cur_wp.get_junction(), "id", None)
        last_road = traj[-1]
        if last_road.road_id == road_id and last_road.lane_id == lane_id:
            state.current_ego_behavior.append("normal_driving")
        if last_road.junction_id is None and junction_id:
            traj.append(Road(road_id, lane_id, junction_id))
        if scenario.ads_name == "behavior":
            current_plan = ego_agent.get_local_planner().get_plan()[0]
        elif scenario.ads_name == "interfuser":
            current_plan = ego_agent._route_planner.route[0]
        if current_plan[1] == RoadOption.CHANGELANELEFT or current_plan[1].name == "CHANGELANELEFT":
            state.current_ego_behavior.append("changing_lane_to_left")
        if current_plan[1] == RoadOption.CHANGELANERIGHT or current_plan[1].name == "CHANGELANERIGHT":
            state.current_ego_behavior.append("changing_lane_to_right")
        if current_plan[1] == RoadOption.RIGHT or current_plan[1].name == "RIGHT":
            state.current_ego_behavior.append("turning_right")
        if current_plan[1] == RoadOption.LEFT or current_plan[1].name == "LEFT":
            state.current_ego_behavior.append("turning_left")

        if last_road.road_id == road_id and last_road.lane_id != lane_id:
            traj.append(Road(road_id, lane_id, junction_id))
            if state.current_ego_behavior[-1].startswith("changing_lane"):  
                state.current_ego_behavior.append("finished_changing_lane")
        if last_road.road_id != road_id and junction_id is None:
            traj.append(Road(road_id, lane_id, junction_id))
        #########################
        if junction_id is None and world.get_map().get_waypoint(
                ego.get_location()).s > 1.0 and state.current_ego_behavior[
            -1] == "normal_driving" and scenario.method != "random": 
            if random.random() < 0.5:
                npc_behavior = "other_vehicle_merging" 
            else:
                npc_behavior = "rear_vehicle_overtaking" 
            if check_before_add(max_npc_num, npc_behavior, npc_list):
                npc, npc_agent, p_list = create_npc_by_interaction_type(npc_behavior, ego, sps)
                if npc and npc_agent:
                    npc_list.append((npc, npc_agent, npc_behavior, state.idx_npc))
                    state.idx_npc += 1
                    npc_p_list.append((npc_behavior, p_list))
        if junction_id is None and world.get_map().get_waypoint(
                ego.get_location()).s > 1.0 and state.current_ego_behavior[-1].startswith(
            "changing_lane") and scenario.method != "random": 
            npc_behavior = "real_vehicle_accelerating_during_lane_change"
            if check_before_add(max_npc_num, npc_behavior, npc_list):
                npc, npc_agent, p_list = create_npc_by_interaction_type(npc_behavior, ego, sps) 
                if npc and npc_agent:
                    npc_list.append((npc, npc_agent, npc_behavior, state.idx_npc))
                    state.idx_npc += 1
                    npc_p_list.append((npc_behavior, p_list))
        if junction_id and state.current_ego_behavior[
            -1] == "turning_right" and scenario.method != "random":
            if check_before_add(max_npc_num, "straight_junction_right", npc_list):
                npc, npc_agent, p_list = create_npc_by_interaction_type("straight_junction_right", ego, sps) 
                if npc and npc_agent:
                    npc_list.append((npc, npc_agent, "straight_junction_right", state.idx_npc))
                    state.idx_npc += 1
                    npc_p_list.append(("straight_junction_right", p_list))
        if junction_id and state.current_ego_behavior[
            -1] == "turning_left" and scenario.method != "random":
            if random.random() < 0.5:
                npc_behavior = "opposite_straight_junction_left"  
            else:
                npc_behavior = "opposite_right_junction_left" 
            if check_before_add(max_npc_num, npc_behavior, npc_list):
                npc, npc_agent, p_list = create_npc_by_interaction_type(npc_behavior, ego, sps)
                if npc and npc_agent:
                    npc_list.append((npc, npc_agent, npc_behavior, state.idx_npc))
                    state.idx_npc += 1
                    npc_p_list.append((npc_behavior, p_list))
        if junction_id and state.current_ego_behavior[-1] != "turning_right" and state.current_ego_behavior[
            -1] != "turning_left" and scenario.method != "random": 
            if check_before_add(max_npc_num, "me_straight_met_right", npc_list):
                npc, npc_agent, p_list = create_npc_by_interaction_type("me_straight_met_right", ego, sps)  
                if npc and npc_agent:
                    npc_list.append((npc, npc_agent, "me_straight_met_right", state.idx_npc))
                    state.idx_npc += 1
                    npc_p_list.append(("me_straight_met_right", p_list))
        time.sleep(1 / 20)

    ######################################
    if scenario.ads_name == "interfuser":
        if ego_agent._hic:
            ego_agent._hic.stop_recording()
            ego_agent._hic._quit()
        ego_agent.destroy()
        agent_wrapper.cleanup()
        del ego_agent
        del agent_wrapper
    carla_collision_sensor.destroy()
    carla_lane_invasion_sensor.destroy()

    if scenario.ads_name == "behavior":
        camera_front.destroy()
    camera_top.destroy()
    ego.destroy()

    for npc, npc_agent, npc_behavior, _ in npc_list:
        if npc and npc_agent:
            npc.destroy()
    save_video(scenario.ads_name)
    ######################################
    if scenario.ads_name == "behavior":
        shutil.move("./fuzzerdata/front.mp4", scenario.result_path)
    shutil.move("./fuzzerdata/top.mp4", scenario.result_path)
    if scenario.ads_name == "interfuser" and os.path.exists("./fuzzerdata/output.mp4"):
        shutil.move("./fuzzerdata/output.mp4", scenario.result_path)
    ######################################
    obj_values.action_num = len(calculate_ego_behavior_change(state.current_ego_behavior))
    obj_values.moving_rate = state.ego_moving_frame_num / state.total_frame_num
    ######################################
    return obj_values, npc_p_list, state.collision_detail, state.laneinvasion_details, state.npc_distance_dict, state.location_recorder


if __name__ == "__main__":
    scenario_inner_data_file_path = sys.argv[1]
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
        time.sleep(10)
        client = carla.Client("localhost", 2000)
        client.set_timeout(10)
        world = client.load_world("Town05")
        setting = world.get_settings()
        setting.synchronous_mode = True
        setting.fixed_delta_seconds = 1 / 20
        world.apply_settings(setting)
        world.tick()

    ######################################
    scenario = load_scenario_from_file(scenario_inner_data_file_path)
    obj_values, npc_p_list, collision_detail, laneinvasion_detail, distance_dict, location_recorder = run_once(
        scenario)  
    with open(scenario.result_path + "/simulation_ret.json", "w", encoding="utf-8") as f:
        data = {
            "obj_values": {
                "min_dis": obj_values.min_dis,
                "min_ttc": obj_values.min_ttc,
                "action_num": obj_values.action_num,
                "moving_rate": obj_values.moving_rate,
            },
            "npc_p_list": npc_p_list,
            "collision_detail": collision_detail,
            "laneinvasion_detail": laneinvasion_detail,
            "distance_dict": distance_dict,
            "location_recorder": location_recorder,
        }
        json.dump(data, f, indent=4)
    loguru.logger.success("ok")
