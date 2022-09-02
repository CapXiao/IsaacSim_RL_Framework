#!/home/ubadmin/.local/share/ov/pkg/isaac_sim-2021.2.1/python.sh

import numpy as np
import os
import omni
import carb
from omni.isaac.kit import SimulationApp


import math
from numpy import array
from numpy.linalg import norm


"""
CONFIG = {
    "experience": f'{os.environ["EXP_PATH"]}/omni.isaac.sim.python.kit',
    "renderer": "PathTracing",
    "samples_per_pixel_per_frame": 64,
    "max_bounces": 10,
    "max_specular_transmission_bounces": 6,
    "max_volume_bounces": 4,
    "subdiv_refinement_level": 2,
    "headless": True,
}
"""

CONFIG = {
    "experience": f'{os.environ["EXP_PATH"]}/omni.isaac.sim.python.kit',
    "renderer": "RealTime",
    "headless": False
}

G = 6.67e-11
parameters = {  'Moon': {'mass': 7.342e22,
                        'pose': {'position': [1.3816269e11, -0.6125574e11, 0.00023985e11],
                        'quaternion': [1, 0, 0, 0]},
                        'path': '/World/Planets/Moon',
                        'velocity': {'linear': [0.041422688e4, 0.0934291224e4, 0],
                        'angular': [0, 0, 0]},
                        'applyforce':[100000,100000,100000],
                        'is_fixed': False
                },
                'Earth': {'mass': 5.972e24,
                        'pose': {'position': [1.3855353e11, -0.61246995e11, 0],
                        'quaternion': [1, 0, 0, 0]},
                        'path': '/World/Planets/Earth',
                        'velocity': {'linear': [1.20402209e4, 2.72375014e4, 0],
                        'angular': [0, 0, 0]},
                        'applyforce':[100000,100000,100000],
                        'is_fixed': False
                },
                'Mars': {'mass': 6.39e23,
                        'pose': {'position': [1.997e11, 0.7125e11, -0.0344e11],
                        'quaternion': [1, 0, 0, 0]},
                        'path': '/World/Planets/Mars',
                        'velocity': {'linear': [-0.8088e4, 2.267e4, 0],
                        'angular': [0, 0, 0]},
                        'applyforce':[100000,100000,100000],
                        'is_fixed': False
                },
                'Venus': {'mass': 4.867e24,
                        'pose': {'position': [-0.5595e11, 0.9195e11, 0.0447e11],
                        'quaternion': [1, 0, 0, 0]},
                        'path': '/World/Planets/Venus',
                        'velocity': {'linear': [-2.99175e4, -1.82029e4, 0],
                        'angular': [0, 0, 0]},
                        'applyforce':[100000,100000,100000],
                        'is_fixed': False
                },
                'Mercury': {'mass': 3.285e23,
                        'pose': {'position': [0.03807e11, -0.6899e11, -0.05957e11],
                        'quaternion': [1, 0, 0, 0]},
                        'path': '/World/Planets/Mercury',
                        'velocity': {'linear': [4.7288e4, 0.26098e4, 0],
                        'angular': [0, 0, 0]},
                        'applyforce':[100000,100000,100000],
                        'is_fixed': False
                },
                'Jupiter': {'mass': 1.898e27,
                        'pose': {'position': [7.4346e11, 0.098856e11, -0.16688e11],
                        'quaternion': [1, 0, 0, 0]},
                        'path': '/World/Planets/Jupiter',
                        'velocity': {'linear': [-0.017377e4, 1.30688e4, 0],
                        'angular': [0, 0, 0]},
                        'applyforce':[100000,100000,100000],
                        'is_fixed': False
                },
                'Saturn': {'mass': 5.683e26,
                        'pose': {'position': [11.7407e11, -9.0531e11, -0.30778e11],
                        'quaternion': [1, 0, 0, 0]},
                        'path': '/World/Planets/Saturn',
                        'velocity': {'linear': [0.5911e4, 0.7666e4, 0],
                        'angular': [0, 0, 0]},
                        'applyforce':[100000,100000,100000],
                        'is_fixed': False
                },
                'Uranus': {'mass': 8.681e25,
                        'pose': {'position': [20.5817e11, 21.1660e11, -0.18808e11],
                        'quaternion': [1, 0, 0, 0]},
                        'path': '/World/Planets/Uranus',
                        'velocity': {'linear': [-0.4875e4, 0.4741e4, 0],
                        'angular': [0, 0, 0]},
                        'applyforce':[100000,100000,100000],
                        'is_fixed': False
                },
                'Neptune': {'mass': 1.024e26,
                        'pose': {'position': [44.5789e11, -4.7474e11, -0.9239e11],
                        'quaternion': [1, 0, 0, 0]},
                        'path': '/World/Planets/Neptune',
                        'velocity': {'linear': [0.057501e4, 0.53995e4, 0],
                        'angular': [0, 0, 0]},
                        'applyforce':[100000,100000,100000],
                        'is_fixed': False
                },
                'Sun': {'mass': 1.9884e30,
                        'pose': {'position': [0, 0, 0],
                        'quaternion': [1, 0, 0, 0]},
                        'path': '/World/Planets/Sun',
                        'velocity': {'linear': [0, 0, 0],
                        'angular': [0, 0, 0]},
                        'applyforce':[0,0,0],
                        'is_fixed': True
                }
                          }

if __name__ == "__main__":
    simulation_app = SimulationApp(CONFIG)

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_manager.set_extension_enabled_immediate("omni.physx.bundle", True)
    simulation_app.set_setting("/app/window/drawMouse", True)
    simulation_app.set_setting("/app/livestream/proto", "ws")
    ext_manager.set_extension_enabled_immediate("omni.syntheticdata", True)
    ext_manager.set_extension_enabled_immediate("omni.kit.window.stage", True)
    ext_manager.set_extension_enabled_immediate("omni.kit.property.bundle", True)

    from pxr import Gf, Sdf, UsdPhysics
    from omni.isaac.dynamic_control import _dynamic_control as dyc
    from omni.isaac.core.utils.nucleus import find_nucleus_server
    from omni.isaac.core import World
    import omni.syntheticdata._syntheticdata as gt
    from omni.isaac.synthetic_utils import SyntheticDataHelper
    from omni.isaac.core.utils.stage import is_stage_loading, open_stage


    def get_nucleus_server():
        result, nucleus_server = find_nucleus_server()
        if result is False:
            carb.log_error(
                "Could not find nucleus server. Stopping."
            )
            exit(1)
        return nucleus_server

    class Manipulator:
        def __init__(self, dc, dyc, name): # Constructor, self ~ this
            self.name = name
            self.dc = dc
            self.dyc = dyc
            self.id = None
            self.mass_api = None
            self.object_prim_path = None

        def getId(self):
            self.id = self.dc.get_rigid_body(self.name)
            return self.id

        def setPose(self, position, quaternion):
            if self.id is None:
                self.getId()
            transform = self.dyc.Transform(position, quaternion)
            return self.dc.set_rigid_body_pose(self.id, transform)

        def getPose(self):
            if self.id is None:
                self.getId()
            self.transform = self.dc.get_rigid_body_pose(self.id)
            self.position = self.transform.p
            self.quaternions = self.transform.r
            return self.position, self.quaternions

        def setLinearVelocity(self, vel):
            if self.id is None:
                self.getId()
            return self.dc.set_rigid_body_linear_velocity(self.id, vel)

        def getLinearVelocity(self):
            if self.id is None:
                self.getId()
            return self.dc.get_rigid_body_linear_velocity(self.id)

        def setAngularVelocity(self, vel):
            if self.id is None:
                self.getId()
            return self.dc.set_rigid_body_angular_velocity(self.id, vel)

        def getAngularVelocity(self):
                if self.id is None:
                    self.getId()    
                return self.dc.get_rigid_body_angular_velocity(self.id)

        def setMass(self, mass):
            if self.object_prim_path is None:
                self.object_prim_path = stage.GetPrimAtPath(self.name)
            if self.mass_api is None:    
                self.mass_api = UsdPhysics.MassAPI(self.object_prim_path)
                if self.getMass() is None:
                    UsdPhysics.MassAPI.Apply(self.object_prim_path)
                    self.mass_api = UsdPhysics.MassAPI(self.object_prim_path)
                    self.mass_api.CreateMassAttr(mass)
            return self.mass_api.GetMassAttr().Set(mass)

        def getMass(self):
            if self.object_prim_path is None:
                self.object_prim_path = stage.GetPrimAtPath(self.name)
            if self.mass_api is None:    
                self.mass_api = UsdPhysics.MassAPI(self.object_prim_path)  
            return self.mass_api.GetMassAttr().Get()

        def setForceAtGlobalFrame(self, force):
            if self.id is None:
                self.getId()
            p, _ = self.getPose()
            return self.dc.apply_body_force(self.id, force, p, True)


    class GravityManager:
        def __init__(self, dc, parameters):
            self.parameters = parameters
            self.manipulators = {}
            self.distances = {}
            self.forces = {}
            self.sumofforce = {}
            self.initialize = False
            self.scale = 2e-9
            #self.scaledt = 365*24*60 #one year ~= 15 seconds
            self.scaledt = 365*24*60*10 #one year ~= 15 seconds
            

        #Create a list of objects to manipulate
            for key in self.parameters.keys():
                self.manipulators[key] = Manipulator(dc, dyc, self.parameters[key]['path'])

            self.rescaleValues()

        def rescaleValues(self):
            for key in parameters.keys():
                self.parameters[key]['mass'] = self.parameters[key]['mass']*self.scale**3
                self.parameters[key]['pose']['position'] = np.array(self.parameters[key]['pose']['position'])*self.scale
                self.parameters[key]['velocity']['linear'] = np.array(self.parameters[key]['velocity']['linear'])*(self.scale*self.scaledt)
                #self.parameters[key]['applyforce'] = np.array(self.parameters[key]['applyforce'])*(self.scaledt*self.scale)
        
        def applyInitialConditions(self):
            for key in parameters.keys():
                self.manipulators[key].getId()
                self.manipulators[key].setMass(self.parameters[key]['mass'])
                self.manipulators[key].setPose(self.parameters[key]['pose']['position'], self.parameters[key]['pose']['quaternion'])
                #self.manipulators[key].setAngularVelocity(self.parameters[key]['velocity']['angular'])
                self.manipulators[key].setLinearVelocity(self.parameters[key]['velocity']['linear'])
                #print(self.parameters[key]['velocity']['linear'])
                #self.manipulators[key].setForceAtGlobalFrame(self.parameters[key]['applyforce'])
            #exit(0)

        def computeVector(self, dt):
            self.vectors = {}
            self.sumofforce = {}
            pos = {}
            rot = {}
            for key1 in self.manipulators.keys():
                print("id:", self.manipulators[key1].id)
                self.vectors[key1] = {}
                self.distances[key1] = {}
                self.forces[key1] ={}
                
                for key2 in self.manipulators.keys():
                    if key1 != key2:
                        pos[key1], rot[key1] = self.manipulators[key1].getPose()
                        pos[key1] = np.array(pos[key1])
                        pos[key2], rot[key2] = self.manipulators[key2].getPose()
                        pos[key2] = np.array(pos[key2])
                        self.vectors[key1][key2] = pos[key2] - pos[key1]
                        #print("vect",self.vectors[key1][key2])
                        # Distance
                        self.distances[key1][key2] = np.linalg.norm(self.vectors[key1][key2])
                        print("dist",self.distances[key1][key2] )
                        # Normalized vector
                        self.vectors[key1][key2] = self.vectors[key1][key2]/self.distances[key1][key2]
                        print("vect",self.vectors[key1][key2])
                        # Force vector between 2 objects
                        self.forces[key1][key2] = self.scaledt*self.scaledt*((G*self.manipulators[key1].getMass()*self.manipulators[key2].getMass()/(self.distances[key1][key2]*self.distances[key1][key2]))*self.vectors[key1][key2])
                        print("force",self.forces[key1][key2])

            for key1 in self.manipulators.keys():
                self.sumofforce[key1] = {}
                #self.sumofforce[key1] = np.sum([self.forces[key1][key2] for key2 in self.forces[key1]])
                print(self.forces[key1].values())
                self.sumofforce[key1] = np.sum(list(self.forces[key1].values()),0)
                print(self.sumofforce[key1])
        
        def applyGravity(self):
            for key1 in self.manipulators.keys():
                print(self.sumofforce[key1])
                self.manipulators[key1].setForceAtGlobalFrame(self.sumofforce[key1])

        def update(self, dt):
            if not self.initialize:
                #self.rescaleValues()
                self.applyInitialConditions()
                self.initialize = True
            else:
                self.computeVector(dt)
                self.applyGravity()


    nucleus_server = get_nucleus_server()

    scene_path = "/home/ubadmin/Downloads/solar_system_3.usd"
    dc = dyc.acquire_dynamic_control_interface()
    vi = omni.kit.viewport.get_viewport_interface()
    sdi = gt.acquire_syntheticdata_interface()

    open_stage(scene_path)
    #omni.usd.get_context().open_stage(scene_path)

    # Add a physics scene prim to stage
    stage = omni.usd.get_context().get_stage()


    world = World(stage_units_in_meters=0.01, physics_dt=1/60.0, rendering_dt=1/60.0)
    #scene = UsdPhysics.Scene.Get(stage, Sdf.Path("/World/physicsScene"))  
    #scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    #scene.CreateGravityMagnitudeAttr().Set(0.0)
    world.step()
    world.render()

    GM = GravityManager(dc,parameters)


    while is_stage_loading():
        print("loading")
        world.step()

    world.render()
    world.add_physics_callback("GM.update", GM.update)
    #world.add_render_callback("name", function)

    world.play()
    i = 0
    #while simulation_app.is_running():
    #    world.step(render=True)
    while simulation_app.is_running():
        world.step(render=True)
        i += 1
        if i == 20000:
            break
        
    simulation_app.close()
