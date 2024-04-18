#!/usr/bin/env python

# Copyright (c) 2019 Intel Labs
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to CARLA manual control with steering wheel Logitech G29.

To drive start by preshing the brake pedal.
Change your wheel_config.ini according to your steering wheel.

To find out the values of your steering wheel use jstest-gtk in Ubuntu.

"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
import time




if sys.version_info >= (3, 0):

    from configparser import ConfigParser

else:

    from configparser import RawConfigParser as ConfigParser

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_h

    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

# ================
# -- CustomTimer
# ================
class CustomTimer:
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time
    def time(self):
        return self.timer()

# ================
# -- DisplayManager
# ================

class DisplayManager:
    def __init__(self, grid_size, window_size):
        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode(window_size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.grid_size = grid_size
        self.window_size = window_size
        self.sensor_list = []
    def get_window_size(self):
        return [int(self.window_size[0]), int(self.window_size[1])]
    def get_display_size(self):
        return [int(self.window_size[0] / self.grid_size[1]), int(self.window_size[1] / self.grid_size[0])]
    def get_display_offset(self, gridPos):
        dis_size = self.get_display_size()
        return [int(gridPos[1] * dis_size[0]), int(gridPos[0] * dis_size[1])]
    def add_sensor(self, sensor):
        self.sensor_list.append(sensor)
    def get_sensor_list(self):
        return self.sensor_list
    def render(self):
        if not self.render_enabled():
            return
        for s in self.sensor_list:
            s.render()
        pygame.display.flip()
    def destroy(self):
        for s in self.sensor_list:
            s.destroy()
    def render_enabled(self):
        return self.display != None



# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):                 # World类
    # Python中的__init__方法用于初始化类的对象。它也称为构造函数。
    # 类的构造函数是一种特殊的方法，主要用来在创建对象时初始化对象， 即为对象成员变量赋初始值
    # 类的构造函数会在每次创建类的新对象时执行。 构造函数的名称与类的名称是完全相同的，并且不会返回任何类型。
    # self代表类的实例
    # self在定义类的方法时是必须有的，虽然在调用时不必传入相应的参数。
    # python中类的实例化类似函数调用方式，并通过__init__方法接收参数。
    # __init__方法（构造函数）有三个参数：carla_world, hud, actor_filter
    def __init__(self, carla_world, actor_filter):  # __init__方法：carla_world, hud, actor_filter作为参数
        self.world = carla_world    # 初始化各种成员变量：carla世界对象
        # self.hud = hud
        self.player = None            # 初始化各种成员变量：玩家角色
        self.collision_sensor = None
        self.lane_invasion_sensor = None     # 初始化各种成员变量：车道入侵传感器
        self.gnss_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = actor_filter
        self.restart()
        self.radar_sensor = None
        self.add_radar_sensor()
        # self.world.on_tick(hud.on_world_tick)

    def restart(self):              # restart方法
        # Keep same camera config if the camera manager exists.
        # cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        # cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        # blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))

        blueprint = self.world.get_blueprint_library().find('vehicle.audi.a2')
        # blueprint = self.world.get_blueprint_library().find('vehicle.tesla.model3')
        blueprint.set_attribute('role_name', 'hero')       # 设定hero车辆 即player
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            print(f"generate'hero'vehicle:ID{self.player.id}")
        while self.player is None:
            spawn_points = self.world.get_map().get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            print(f"generate'hero'vehicle:ID{self.player.id}")
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player)
        self.gnss_sensor = GnssSensor(self.player)
        # self.camera_manager = CameraManager(self.player, self.hud)
        # self.camera_manager.transform_index = cam_pos_index
        # self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        # self.hud.notification(actor_type)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        # self.hud.notification('Weather: %s' % preset[1])    # 更新hud通知
        self.player.get_world().set_weather(preset[0])      # 设置世界天气

    # def tick(self, clock):
        # self.hud.tick(self, clock)    # 更新hud通知

    # def render(self, display):        # 委托摄像头管理器和HUD的‘render’方法，用于渲染世界的画面
        # self.camera_manager.render(display)
        # self.hud.render(display)

    def destroy(self):      # 停止并销毁各种传感器，销毁玩家角色
        sensors = [
            # self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()

    def add_radar_sensor(self):
        radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
        radar_bp.set_attribute('horizontal_fov', '30')  # 30度的水平视场
        radar_bp.set_attribute('vertical_fov', '5')    # 5度的垂直视场
        radar_bp.set_attribute('range', '100')          # 20米范围
        radar_transform = carla.Transform(carla.Location(x=2.0, z=1.0))
        self.radar_sensor = self.world.spawn_actor(radar_bp, radar_transform, attach_to=self.player)
        self.radar_sensor.listen(lambda radar_data: self.process_radar_data(radar_data))

    def process_radar_data(self, radar_data):
        close_vehicle_detected = False
        for detection in radar_data:
            if detection.depth < 2.0:  # 检测距离小于2米的对象
                close_vehicle_detected = True
                break

        if close_vehicle_detected:
            if not pygame.mixer.music.get_busy():  # 如果没有音乐正在播放
                pygame.mixer.music.load('C:\mp3\distanceradar.mp3')
                print("detected")
                pygame.mixer.music.play(-1)  # 使用-1使音乐循环播放
        else:
            pygame.mixer.music.stop()  # 停止播放音乐

# ==============================================================================
# -- DualControl -----------------------------------------------------------
# ==============================================================================


class DualControl(object):
    def __init__(self, world, start_in_autopilot):
        pygame.mixer.init()
        pygame.mixer.music.load('C:\mp3\soundblinker.mp3')
        # self.left_blinker_sound = pygame.mixer.Sound('C:\mp3\sound.mp3')
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        # world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

        # initialize steering wheel
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        if joystick_count > 1:
            raise ValueError("Please Connect Just One Joystick")

        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()

        self._parser = ConfigParser()
        self._parser.read('C:\CARLA_0.9.15\WindowsNoEditor\PythonAPI\examples\wheel_config.ini')  # wheel_config.ini的绝对路径
        self._steer_idx = int(
            self._parser.get('G29 Racing Wheel', 'steering_wheel'))
        self._throttle_idx = int(
            self._parser.get('G29 Racing Wheel', 'throttle'))
        self._brake_idx = int(self._parser.get('G29 Racing Wheel', 'brake'))
        self._reverse_idx = int(self._parser.get('G29 Racing Wheel', 'reverse'))
        self._handbrake_idx = int(
            self._parser.get('G29 Racing Wheel', 'handbrake'))
        self._RightBlinker_idx = int(self._parser.get('G29 Racing Wheel', 'RightBlinker'))
        self._LeftBlinker_idx = int(self._parser.get('G29 Racing Wheel', 'LeftBlinker'))
        self._HighBeam_idx = int(self._parser.get('G29 Racing Wheel', 'HighBeam'))

        #0314
        # self._initial_steer_direction = None  # 添加这行来初始化初始方向盘转动方向


    def parse_events(self, world, clock):
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:
                    world.restart()
                # elif event.button == 1:
                #     world.hud.toggle_info()
                # elif event.button == 2:
                #     world.camera_manager.toggle_camera()
                elif event.button == 3:
                    world.next_weather()
                elif event.button == self._reverse_idx:
                    self._control.gear = 1 if self._control.reverse else -1
                # elif event.button == 23:
                    # world.camera_manager.next_sensor()
                elif event.button == self._RightBlinker_idx:
                    current_lights ^= carla.VehicleLightState.RightBlinker
                    if current_lights & carla.VehicleLightState.RightBlinker:
                        if not pygame.mixer.music.get_busy():
                            pygame.mixer.music.play(loops=-1)
                            # 0314 当右转向灯开启时，记录当前方向盘转动方向为初始转动方向
                            # self._initial_steer_direction = self._control.steer >= 0
                    else:
                        pygame.mixer.music.stop()
                        # self._initial_steer_direction = None   # 0314
                elif event.button == self._LeftBlinker_idx:
                    current_lights ^= carla.VehicleLightState.LeftBlinker
                    if current_lights & carla.VehicleLightState.LeftBlinker:
                        if not pygame.mixer.music.get_busy():
                            pygame.mixer.music.play(loops=-1)
                    else:
                        pygame.mixer.music.stop()
                elif event.button == self._HighBeam_idx:
                    current_lights ^= carla.VehicleLightState.HighBeam  # HighBeam效果比较明显
                    # if current_lights & carla.VehicleLightState.HighBeam:
                    #     if not pygame.mixer.music.get_busy():
                    #         pygame.mixer.music.play(loops=-1)
                    # else:
                    #     pygame.mixer.music.stop()
                elif event.button == 5:
                    current_lights ^= carla.VehicleLightState.LowBeam  # LowBeam效果不大不用管
                # elif event.button == self._Interior_idx:
                #     current_lights ^= carla.VehicleLightState.Interior


            # elif event.type == pygame.KEYUP:
            #     if self._is_quit_shortcut(event.key):
            #         return True
            #     elif event.key == K_BACKSPACE:
            #         world.restart()
            #     elif event.key == K_F1:
            #         world.hud.toggle_info()
            #     elif event.key == K_h or (event.key == K_SLASH and pygame.key.get_mods() & KMOD_SHIFT):
            #         world.hud.help.toggle()
            #     elif event.key == K_TAB:
            #         world.camera_manager.toggle_camera()
            #     elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
            #         world.next_weather(reverse=True)
            #     elif event.key == K_c:
            #         world.next_weather()
            #     elif event.key == K_BACKQUOTE:
            #         world.camera_manager.next_sensor()
            #     elif event.key > K_0 and event.key <= K_9:
            #         world.camera_manager.set_sensor(event.key - 1 - K_0)
            #     elif event.key == K_r:
            #         world.camera_manager.toggle_recording()
            #     if isinstance(self._control, carla.VehicleControl):
            #         if event.key == K_q:
            #             self._control.gear = 1 if self._control.reverse else -1
            #         elif event.key == K_m:
            #             self._control.manual_gear_shift = not self._control.manual_gear_shift
            #             self._control.gear = world.player.get_control().gear
            #             world.hud.notification('%s Transmission' %
            #                                    ('Manual' if self._control.manual_gear_shift else 'Automatic'))
            #         elif self._control.manual_gear_shift and event.key == K_COMMA:
            #             self._control.gear = max(-1, self._control.gear - 1)
            #         elif self._control.manual_gear_shift and event.key == K_PERIOD:
            #             self._control.gear = self._control.gear + 1
            #         elif event.key == K_p:
            #             self._autopilot_enabled = not self._autopilot_enabled
            #             world.player.set_autopilot(self._autopilot_enabled)
            #             world.hud.notification('Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._parse_vehicle_wheel()
                self._control.reverse = self._control.gear < 0
                # Set automatic control-related vehicle lights
                if self._control.brake:
                    current_lights |= carla.VehicleLightState.Brake
                else:  # Remove the Brake flag
                    current_lights &= ~carla.VehicleLightState.Brake
                if self._control.reverse:
                    current_lights |= carla.VehicleLightState.Reverse
                else:  # Remove the Reverse flag
                    current_lights &= ~carla.VehicleLightState.Reverse
                if current_lights != self._lights:  # Change the light state only if necessary
                    self._lights = current_lights
                    world.player.set_light_state(carla.VehicleLightState(self._lights))
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
            world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    def _parse_vehicle_wheel(self):
        numAxes = self._joystick.get_numaxes()
        jsInputs = [float(self._joystick.get_axis(i)) for i in range(numAxes)]
        # print (jsInputs)
        jsButtons = [float(self._joystick.get_button(i)) for i in
                     range(self._joystick.get_numbuttons())]

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        # K1 = 0.225  # 0.55   # K1: 方向盘灵敏度。 注意每次都需要打开Logitech G29 Hub开启回正。
        K1 =0.15 # 0.10
        steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])

        K2 = 1.35  #1.3 # 1.6    # K2: 油门灵敏度
        throttleCmd = K2 + (2.05 * math.log10(
            -0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1

        brakeCmd = 1.6 + (2.05 * math.log10(
            -0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1

        self._control.steer = steerCmd
        self._control.brake = brakeCmd
        self._control.throttle = throttleCmd

        #toggle = jsButtons[self._reverse_idx]

        self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

        # # 0314 检查是否需要关闭右转向灯
        # if self._initial_steer_direction is not None:
        #     current_steer_direction = self._control.steer >= 0
        #     # 如果方向盘转动方向与初始方向相反，则关闭右转向灯
        #     if current_steer_direction != self._initial_steer_direction:
        #         self._lights &= ~carla.VehicleLightState.RightBlinker
        #         # self._lights &= ~carla.VehicleLightState.LeftBlinker
        #         pygame.mixer.music.stop()  # 停止播放转向灯声音
        #         self._initial_steer_direction = None  # 重置初始方向盘转动方向
        #         # world.player.set_light_state(carla.VehicleLightState(self._lights))  # 更新车辆的灯光状态

    def _parse_walker_keys(self, keys, milliseconds):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


# class HUD(object):
#     def __init__(self, width, height):
#         self.dim = (width, height)
#         font = pygame.font.Font(pygame.font.get_default_font(), 20)
#         font_name = 'courier' if os.name == 'nt' else 'mono'
#         fonts = [x for x in pygame.font.get_fonts() if font_name in x]
#         default_font = 'ubuntumono'
#         mono = default_font if default_font in fonts else fonts[0]
#         mono = pygame.font.match_font(mono)
#         self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
#         self._notifications = FadingText(font, (width, 40), (0, height - 40))
#         self.help = HelpText(pygame.font.Font(mono, 24), width, height)
#         self.server_fps = 0
#         self.frame = 0
#         self.simulation_time = 0
#         self._show_info = True
#         self._info_text = []
#         self._server_clock = pygame.time.Clock()
#
#     def on_world_tick(self, timestamp):
#         self._server_clock.tick()
#         self.server_fps = self._server_clock.get_fps()
#         self.frame = timestamp.frame
#         self.simulation_time = timestamp.elapsed_seconds
#
#     def tick(self, world, clock):
#         self._notifications.tick(world, clock)
#         if not self._show_info:
#             return
#         t = world.player.get_transform()
#         v = world.player.get_velocity()
#         c = world.player.get_control()
#         heading = 'N' if abs(t.rotation.yaw) < 89.5 else ''
#         heading += 'S' if abs(t.rotation.yaw) > 90.5 else ''
#         heading += 'E' if 179.5 > t.rotation.yaw > 0.5 else ''
#         heading += 'W' if -0.5 > t.rotation.yaw > -179.5 else ''
#         colhist = world.collision_sensor.get_collision_history()
#         collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
#         max_col = max(1.0, max(collision))
#         collision = [x / max_col for x in collision]
#         vehicles = world.world.get_actors().filter('vehicle.*')
#         self._info_text = [
#             'Server:  % 16.0f FPS' % self.server_fps,
#             'Client:  % 16.0f FPS' % clock.get_fps(),
#             '',
#             'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
#             'Map:     % 20s' % world.world.get_map().name.split('/')[-1],
#             'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
#             '',
#             'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
#             u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (t.rotation.yaw, heading),
#             'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
#             'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
#             'Height:  % 18.0f m' % t.location.z,
#             '']
#         if isinstance(c, carla.VehicleControl):
#             self._info_text += [
#                 ('Throttle:', c.throttle, 0.0, 1.0),
#                 ('Steer:', c.steer, -1.0, 1.0),
#                 ('Brake:', c.brake, 0.0, 1.0),
#                 ('Reverse:', c.reverse),
#                 ('Hand brake:', c.hand_brake),
#                 ('Manual:', c.manual_gear_shift),
#                 'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
#         elif isinstance(c, carla.WalkerControl):
#             self._info_text += [
#                 ('Speed:', c.speed, 0.0, 5.556),
#                 ('Jump:', c.jump)]
#         self._info_text += [
#             '',
#             'Collision:',
#             collision,
#             '',
#             'Number of vehicles: % 8d' % len(vehicles)]
#         if len(vehicles) > 1:
#             self._info_text += ['Nearby vehicles:']
#             distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
#             vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
#             for d, vehicle in sorted(vehicles):
#                 if d > 200.0:
#                     break
#                 vehicle_type = get_actor_display_name(vehicle, truncate=22)
#                 self._info_text.append('% 4dm %s' % (d, vehicle_type))
#
#     def toggle_info(self):
#         self._show_info = not self._show_info
#
#     def notification(self, text, seconds=2.0):
#         self._notifications.set_text(text, seconds=seconds)
#
#     def error(self, text):
#         self._notifications.set_text('Error: %s' % text, (255, 0, 0))
#
#     def render(self, display):
#         if self._show_info:
#             info_surface = pygame.Surface((220, self.dim[1]))
#             info_surface.set_alpha(100)
#             display.blit(info_surface, (0, 0))
#             v_offset = 4
#             bar_h_offset = 100
#             bar_width = 106
#             for item in self._info_text:
#                 if v_offset + 18 > self.dim[1]:
#                     break
#                 if isinstance(item, list):
#                     if len(item) > 1:
#                         points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
#                         pygame.draw.lines(display, (255, 136, 0), False, points, 2)
#                     item = None
#                     v_offset += 18
#                 elif isinstance(item, tuple):
#                     if isinstance(item[1], bool):
#                         rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
#                         pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
#                     else:
#                         rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
#                         pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
#                         f = (item[1] - item[2]) / (item[3] - item[2])
#                         if item[2] < 0.0:
#                             rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
#                         else:
#                             rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
#                         pygame.draw.rect(display, (255, 255, 255), rect)
#                     item = item[0]
#                 if item:  # At this point has to be a str.
#                     surface = self._font_mono.render(item, True, (255, 255, 255))
#                     display.blit(surface, (8, v_offset))
#                 v_offset += 18
#         self._notifications.render(display)
#         self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


# class FadingText(object):
#     def __init__(self, font, dim, pos):
#         self.font = font
#         self.dim = dim
#         self.pos = pos
#         self.seconds_left = 0
#         self.surface = pygame.Surface(self.dim)
#
#     def set_text(self, text, color=(255, 255, 255), seconds=2.0):
#         text_texture = self.font.render(text, True, color)
#         self.surface = pygame.Surface(self.dim)
#         self.seconds_left = seconds
#         self.surface.fill((0, 0, 0, 0))
#         self.surface.blit(text_texture, (10, 11))
#
#     def tick(self, _, clock):
#         delta_seconds = 1e-3 * clock.get_time()
#         self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
#         self.surface.set_alpha(500.0 * self.seconds_left)
#
#     def render(self, display):
#         display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


# class HelpText(object):
#     def __init__(self, font, width, height):
#         lines = __doc__.split('\n')
#         self.font = font
#         self.dim = (680, len(lines) * 22 + 12)
#         self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
#         self.seconds_left = 0
#         self.surface = pygame.Surface(self.dim)
#         self.surface.fill((0, 0, 0, 0))
#         for n, line in enumerate(lines):
#             text_texture = self.font.render(line, True, (255, 255, 255))
#             self.surface.blit(text_texture, (22, n * 22))
#             self._render = False
#         self.surface.set_alpha(220)
#
#     def toggle(self):
#         self._render = not self._render
#
#     def render(self, display):
#         if self._render:
#             display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        # self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)

        # 0318加载音频文件
        self.collision_sound = pygame.mixer.Sound("C:\mp3\crash1_volumndowndown.mp3")
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        # self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)
        # 0318检查音频是否已经在播放
        if not pygame.mixer.get_busy():
            # 0318 播放碰撞音效
            self.collision_sound.play()


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        # self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        # self.hud.notification('Crossed line %s' % ' and '.join(text))

# ==============================================================================
# -- GnssSensor --------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


# class CameraManager(object):
#     def __init__(self, parent_actor, hud):
#         self.sensor = None
#         self.surface = None
#         self._parent = parent_actor
#         # self.hud = hud
#         self.recording = False
#         # list
#         self._camera_transforms = [
#             carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)),
#             carla.Transform(carla.Location(x=1.6, z=1.7)),
#             carla.Transform(carla.Location(x=-0.32, y=-0.25,z=1.3), carla.Rotation(pitch=0,yaw=0,roll=0)) # 'vehicle.audi.a2'
#             # carla.Transform(carla.Location(x=-0.25, y=-0.3,z=1.23), carla.Rotation(pitch=0,yaw=0,roll=0))   # vehicle.tesla.model3
#             ]
#         self.transform_index = 1
#         self.sensors = [
#             ['sensor.camera.rgb', cc.Raw, 'Camera RGB'],
#             ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)'],
#             ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)'],
#             ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)'],
#             ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)'],
#             ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
#                 'Camera Semantic Segmentation (CityScapes Palette)'],
#             ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)']]
#         world = self._parent.get_world()
#         bp_library = world.get_blueprint_library()
#         for item in self.sensors:
#             bp = bp_library.find(item[0])
#             if item[0].startswith('sensor.camera'):
#                 bp.set_attribute('image_size_x', str(hud.dim[0]))
#                 bp.set_attribute('image_size_y', str(hud.dim[1]))
#             elif item[0].startswith('sensor.lidar'):
#                 bp.set_attribute('range', '50')
#             item.append(bp)
#         self.index = None
#
#     def toggle_camera(self):
#         self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
#         self.sensor.set_transform(self._camera_transforms[self.transform_index])
#
#     def set_sensor(self, index, notify=True):
#         index = index % len(self.sensors)
#         needs_respawn = True if self.index is None \
#             else self.sensors[index][0] != self.sensors[self.index][0]
#         if needs_respawn:
#             if self.sensor is not None:
#                 self.sensor.destroy()
#                 self.surface = None
#             self.sensor = self._parent.get_world().spawn_actor(
#                 self.sensors[index][-1],
#                 self._camera_transforms[self.transform_index],
#                 attach_to=self._parent)
#             # We need to pass the lambda a weak reference to self to avoid
#             # circular reference.
#             weak_self = weakref.ref(self)
#             self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
#         # if notify:
#             # self.hud.notification(self.sensors[index][2])
#         self.index = index
#
#     def next_sensor(self):
#         self.set_sensor(self.index + 1)
#
#     def toggle_recording(self):
#         self.recording = not self.recording
#         # self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))
#
#     def render(self, display):
#         if self.surface is not None:
#             display.blit(self.surface, (0, 0))
#
#     @staticmethod
#     def _parse_image(weak_self, image):
#         self = weak_self()
#         if not self:
#             return
#         if self.sensors[self.index][0].startswith('sensor.lidar'):
#             points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
#             points = np.reshape(points, (int(points.shape[0] / 4), 4))
#             lidar_data = np.array(points[:, :2])
#             lidar_data *= min(self.hud.dim) / 100.0
#             lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
#             lidar_data = np.fabs(lidar_data) # pylint: disable=E1111
#             lidar_data = lidar_data.astype(np.int32)
#             lidar_data = np.reshape(lidar_data, (-1, 2))
#             lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
#             lidar_img = np.zeros(lidar_img_size)
#             lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
#             self.surface = pygame.surfarray.make_surface(lidar_img)
#         else:
#             image.convert(self.sensors[self.index][1])
#             array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
#             array = np.reshape(array, (image.height, image.width, 4))
#             array = array[:, :, :3]
#             array = array[:, :, ::-1]
#             self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
#         if self.recording:
#             image.save_to_disk('_out/%08d' % image.frame)


# =====================
# -- SensorManager --
# =======================
# 初始化摄像头
class SensorManager:
    def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options, display_pos, reverse, overlay_position=None, overlay_size=None,mask_path=None):
        self.surface = None
        self.world = world
        self.display_man = display_man
        self.display_pos = display_pos
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        self.sensor_options = sensor_options
        self.timer = CustomTimer()
        self.time_processing = 0.0
        self.tics_processing = 0
        self.display_man.add_sensor(self)
        self.reverse = reverse
        self.overlay_position = overlay_position  # 新增悬浮窗口位置属性
        self.overlay_size = overlay_size
        self.mask_path = mask_path
    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == 'RGBCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            print("===size:", str(disp_size[0]), str(disp_size[1]))
            camera_bp.set_attribute('image_size_x', str(disp_size[0]))
            camera_bp.set_attribute('image_size_y', str(disp_size[1]))
            camera_bp.set_attribute('fov', str(40))
            #
            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])
            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            print("camera_attributes:", camera.attributes)
            for i in camera.attributes:
                print("camera_attributes:", i, camera.attributes[i])
            camera.listen(self.save_rgb_image)
            return camera
        else:
            return None
    def get_sensor(self):
        return self.sensor
    def save_rgb_image(self, image):
        t_start = self.timer.time()
        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        if self.reverse == True:
            array = np.flip(array, axis=1)  # 将画面左右翻转
        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        t_end = self.timer.time()
        self.time_processing += (t_end - t_start)
        self.tics_processing += 1

    def apply_mask(self, surface):
        if self.mask_path:
            # 加载遮罩图像
            mask_image = pygame.image.load(self.mask_path).convert_alpha()
            # 调整遮罩图像大小以匹配悬浮窗口大小
            mask_image = pygame.transform.scale(mask_image, self.overlay_size)
            # 创建一个新Surface以应用遮罩
            masked_surface = pygame.Surface(self.overlay_size, pygame.SRCALPHA)
            # 应用遮罩
            masked_surface.blit(surface, (0, 0))
            masked_surface.blit(mask_image, (0, 0), special_flags=pygame.BLEND_RGBA_MIN)
            return masked_surface
        else:
            return surface

    # 遮罩后的后视镜
    def render(self):
        if self.surface is not None:
            if self.overlay_position is not None:
                resized_surface = pygame.transform.scale(self.surface,
                                                         self.overlay_size) if self.overlay_size else self.surface

                # 应用遮罩
                masked_surface = self.apply_mask(resized_surface)
                # 绘制遮罩后的图像
                self.display_man.display.blit(masked_surface, self.overlay_position)

                # # Draw a red border around the overlay
                # border_color = (255, 0, 0)  # Red color
                # border_rect = pygame.Rect(self.overlay_position, self.overlay_size)  # Create a Rect for the border
                # pygame.draw.rect(self.display_man.display, border_color, border_rect, 3)  # 3 is the border thickness
            else:
                offset = self.display_man.get_display_offset(self.display_pos)
                self.display_man.display.blit(self.surface, offset)

    # def render(self):
    #     if self.surface is not None:
    #         if self.overlay_position is not None:
    #             # 创建一个透明的表面
    #             ellipse_surface = pygame.Surface(self.overlay_size, pygame.SRCALPHA)
    #             # 在透明表面上绘制椭圆形
    #             pygame.draw.ellipse(ellipse_surface, (255, 255, 255, 255), (0, 0, *self.overlay_size))
    #
    #             # 将原始图像缩放到椭圆形大小
    #             resized_surface = pygame.transform.scale(self.surface,
    #                                                      self.overlay_size) if self.overlay_size else self.surface
    #
    #             # 使用混合模式将缩放后的图像绘制到椭圆形表面上，仅在椭圆形区域内显示图像
    #             ellipse_surface.blit(resized_surface, (0, 0), special_flags=pygame.BLEND_RGBA_MIN)
    #
    #             # 将带有椭圆形图像的表面绘制到主显示屏上
    #             self.display_man.display.blit(ellipse_surface, self.overlay_position)
    #         else:
    #             offset = self.display_man.get_display_offset(self.display_pos)
    #             self.display_man.display.blit(self.surface, offset)

    # 左右后视镜椭圆，中间后视镜矩形最终版。
    # def render(self):
    #     if self.surface is not None:
    #         if self.overlay_position is not None:
    #             resized_surface = pygame.transform.scale(self.surface, self.overlay_size) if self.overlay_size else self.surface
    #             if self.shape == 'ellipse':
    #                 # 创建椭圆形渲染逻辑
    #                 ellipse_surface = pygame.Surface(self.overlay_size, pygame.SRCALPHA)
    #                 pygame.draw.ellipse(ellipse_surface, (255, 255, 255, 255), (0, 0, *self.overlay_size))
    #                 resized_surface = pygame.transform.scale(self.surface, self.overlay_size) if self.overlay_size else self.surface
    #                 ellipse_surface.blit(resized_surface, (0, 0), special_flags=pygame.BLEND_RGBA_MIN)
    #                 self.display_man.display.blit(ellipse_surface, self.overlay_position)
    #             else:
    #                 # 矩形渲染逻辑不变
    #                 self.display_man.display.blit(resized_surface, self.overlay_position)
    #                 # border_color = (255, 0, 0)
    #                 # border_rect = pygame.Rect(self.overlay_position, self.overlay_size)
    #                 # pygame.draw.rect(self.display_man.display, border_color, border_rect, 3)
    #         else:
    #             offset = self.display_man.get_display_offset(self.display_pos)
    #             self.display_man.display.blit(self.surface, offset)

    def destroy(self):
        self.sensor.destroy()


# ======================
# -- cluster --
# ======================

# 创建字体对象用于绘制文本

def draw_reverse_indicator(surface, vehicle):
    font = pygame.font.Font(pygame.font.get_default_font(), 100)
    font_name = 'courier' if os.name == 'nt' else 'mono'
    fonts = [x for x in pygame.font.get_fonts() if font_name in x]
    default_font = 'ubuntumono'
    # 检查车辆是否在倒挡
    reverse = vehicle.get_control().reverse
    if reverse:
        # 创建包含字母"R"的表面
        text_surface = font.render('R', True, (255, 0, 0))  # 红色字母"R"
        # 获取屏幕尺寸以便正确放置文本
        screen_rect = surface.get_rect()
        # 定位到屏幕左下角
        text_rect = text_surface.get_rect(bottomleft=screen_rect.bottomleft)
        # 绘制文本
        surface.blit(text_surface, text_rect)
        # print("draw R font")


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    display_manager = None
    timer = CustomTimer()
    try:
        client = carla.Client(args.host, args.port)
        client.load_world('Town03')  #20240207加的 可用：10HD/03/
        client.set_timeout(2.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)     # 定义display

        # hud = HUD(args.width, args.height)
        world = World(client.get_world(), args.filter)

        # 设置同步模式
        settings = world.world.get_settings()

        # settings.no_rendering_mode = True          # 0326

        settings.synchronous_mode = True  # 启用同步模式
        settings.fixed_delta_seconds = 0.05  # 每个仿真步骤的时间间隔
        world.world.apply_settings(settings)

        controller = DualControl(world, args.autopilot)
        hero = world.player

        # Display Manager organize all the sensors an its display in a window
        # If can easily configure the grid and the total window size
        # grid_size中第一个元素表示网格的行数，第二个元素代表网格的列数。
        display_manager = DisplayManager(grid_size=[1, 3], window_size=[args.width, args.height])


        # Example for adding overlay positions for rear-view cameras
        # overlay_positions = [(1100, 800), (3500, 150), (5150, 720)] # 示例悬浮位置
        overlay_positions = [(780, 563), (2450, 100), (3650, 510)]
        rearview_sizes = [(280,170),(475,126),(190,130)]

        SensorManager(client.get_world(), display_manager, 'RGBCamera',
                      carla.Transform(carla.Location(x=-0.32, y=-0.25, z=1.3), carla.Rotation(pitch=-2,yaw=-40)),
                      hero, {}, display_pos=[0, 0], reverse=False)
        SensorManager(client.get_world(), display_manager, 'RGBCamera',
                      carla.Transform(carla.Location(x=-0.32, y=-0.25, z=1.3), carla.Rotation(pitch=-2,yaw=+00)),
                      hero, {}, display_pos=[0, 1], reverse=False)
        SensorManager(client.get_world(), display_manager, 'RGBCamera',
                      carla.Transform(carla.Location(x=-0.32, y=-0.25, z=1.3), carla.Rotation(pitch=-2,yaw=+40)),
                      hero, {}, display_pos=[0, 2], reverse=False)

        SensorManager(client.get_world(), display_manager, 'RGBCamera',
                      carla.Transform(carla.Location(x=0.7, y=-1.0, z=1.1), carla.Rotation(yaw=-170)),
                      hero, {}, display_pos=[1, 0], reverse=True, overlay_position = overlay_positions[0],overlay_size= rearview_sizes[0],mask_path="C:\mask\mask1.png")
        SensorManager(client.get_world(), display_manager, 'RGBCamera',
                      carla.Transform(carla.Location(x=0.7, y=0, z=1.3), carla.Rotation(yaw=-180)),
                      hero, {}, display_pos=[1, 1], reverse=True, overlay_position = overlay_positions[1],overlay_size= rearview_sizes[1],mask_path="C:\mask\mask2.png")
        SensorManager(client.get_world(), display_manager, 'RGBCamera',
                      carla.Transform(carla.Location(x=0.7, y=+1.0, z=1.1), carla.Rotation(yaw=+170)),
                      hero, {}, display_pos=[1, 2], reverse=True, overlay_position = overlay_positions[2],overlay_size= rearview_sizes[2],mask_path="C:\mask\mask3.png")

        clock = pygame.time.Clock()
        # list_available_vehicles(world.world)

        # record
        # print("Recording on file: %s" % client.start_recorder(args.recorder_filename))
        print("Recording on file: %s,with additional data:%s" % (args.recorder_filename,True))
        client.start_recorder(args.recorder_filename,True)
        if (args.recorder_time > 0):
            time.sleep(args.recorder_time)

        while True:
            # clock.tick_busy_loop(60)

            # sync添加
            world.world.tick()

            if controller.parse_events(world, clock):
                return
            # world.tick(clock)

            # world.render(display)   # 0308修改：打开了world.render(display),出现了最开始example的驾驶员视角.但不行，反应太慢。
            # pygame.display.flip()  # 更新屏幕

            display_manager.render()   # 0308修改：把display_manager.render()放到world.render(display)之后，出现后视镜
            draw_reverse_indicator(display, hero)
            pygame.display.flip()  # 更新屏幕

    finally:
        if world is not None:
            settings = world.world.get_settings()
            # settings.no_rendering_mode = False  # 0326
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            world.world.apply_settings(settings)

        if display_manager:
            display_manager.destroy()
        if world is not None:
            world.destroy()
        print("world destroyed")

        print("Stop recording")
        client.stop_recorder()

        pygame.quit()   # 退出pygame


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

# def list_available_vehicles(world):
#     blueprint_library = world.get_blueprint_library()
#     vehicle_blueprints = blueprint_library.filter('vehicle.*')
#     print("Available vehicle types:")
#     for blueprint in vehicle_blueprints:
#         print(blueprint.id)

def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        # sdefault='5760x1080',
        default='4080x768',
        # default='3580x668',
        # default='5760x1080',
        help='window resolution (default: 5760x1080)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')

    # record  path:C:\Users\user\AppData\Local\CarlaUE4\Saved
    argparser.add_argument(
        '-f', '--recorder_filename',
        metavar='F',
        default="est1.log",
        help='recorder filename (est1.log)')
    argparser.add_argument(
        '-t', '--recorder_time',
        metavar='T',
        default=0,
        type=int,
        help='recorder duration (auto-stop)')

    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)


    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
