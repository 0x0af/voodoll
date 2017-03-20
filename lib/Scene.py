#!/usr/bin/python

import avango
import avango.gua
import avango.gua as av
import avango.script
import time

import math
from avango.script import field_has_changed


class SceneScript(avango.script.Script):
    ## input fields
    sf_reset_button = avango.SFBool()

    ## Default constructor.
    def __init__(self):
        self.super(SceneScript).__init__()

        ### external references ###
        self.CLASS = None  # is set later

        ### resources ###
        self.keyboard_device_sensor = avango.daemon.nodes.DeviceSensor(DeviceService=avango.daemon.DeviceService())
        self.keyboard_device_sensor.Station.value = "gua-device-keyboard0"

        self.sf_reset_button.connect_from(self.keyboard_device_sensor.Button14)  # spacebar key

        self.tempo = None
        self.always_evaluate(True)

    def my_constructor(self, CLASS):
        self.CLASS = CLASS

    ### callbacks ###
    @field_has_changed(sf_reset_button)
    def sf_reset_button_changed(self):
        if self.sf_reset_button.value == True and self.CLASS is not None:  # button pressed
            self.CLASS.reset()

    @staticmethod
    def get_rotation_for_time(time):
        return av.make_rot_mat(2 * math.degrees(time % math.pi), 0.0, 1.0, 0.0)

    def set_tempo(self, tempo):
        self.tempo = tempo

    def evaluate(self):
        if self.tempo is not None:
            _trans_mat = av.make_trans_mat(0.0, 0.05, 0.5)
            _rot_mat = self.get_rotation_for_time(time.time())
            _scale_mat = av.make_scale_mat(self.tempo.Transform.value.get_scale())
            self.tempo.Transform.value = _rot_mat * _trans_mat * _scale_mat


class Scene():
    ## constructor
    def __init__(self,
                 PARENT_NODE=None,
                 PHYSICS=None
                 ):

        ### external reference ###
        self.PARENT_NODE = PARENT_NODE
        self.PHYSICS = PHYSICS

        print(dir(PHYSICS))

        ### resources ###
        self.script = SceneScript()
        self.script.my_constructor(self)

        ## init scene light
        self.scene_light = av.nodes.LightNode(Name="scene_light")
        self.scene_light.Type.value = av.LightType.SPOT
        self.scene_light.Color.value = av.Color(1.0, 0.8, 0.8)
        self.scene_light.Brightness.value = 80.0
        self.scene_light.Falloff.value = 0.1  # exponent
        self.scene_light.EnableShadows.value = True
        self.scene_light.ShadowMapSize.value = 2048
        self.scene_light.ShadowOffset.value = 0.001
        self.scene_light.ShadowNearClippingInSunDirection.value = 0.1 * (1.0 / 4.0)
        self.scene_light.ShadowMaxDistance.value = 10.0  # maximal distance, the shadow is visible
        self.scene_light.ShadowNearClippingInSunDirection.value = 0.05
        self.scene_light.Transform.value = av.make_trans_mat(0.0, 2, 0.5) * \
                                           av.make_rot_mat(80.0, -1, 0, 0) * \
                                           av.make_scale_mat(3.0)
        PARENT_NODE.Children.value.append(self.scene_light)

        ## init scene geometries
        _loader = av.nodes.TriMeshLoader()  # get trimesh loader to load external meshes

        # cube 1
        # self.cube_1_wrapper = av.nodes.TransformNode(Name="Cube_1")
        self.cube_1 = _loader.create_geometry_from_file("cube", "data/objects/cube.obj",
                                                        av.LoaderFlags.DEFAULTS | av.LoaderFlags.MAKE_PICKABLE)
        # self.cube_1_wrapper.Transform.value = av.make_trans_mat(0.0, 0.55, 0.0) * \
        #                                       av.make_scale_mat(0.1, 0.1, 0.1)
        self.cube_1.Transform.value = av.make_trans_mat(0.0, 0.55, 0.0) * \
                                      av.make_scale_mat(0.1, 0.1, 0.1)
        self.cube_1.Material.value.set_uniform("Emissivity", 0.5)
        self.cube_1.Material.value.set_uniform("Metalness", 0.01)
        self.cube_1.Material.value.set_uniform("Roughness", 0.75)
        self.cube_1.Material.value.set_uniform("ColorMap", "data/textures/box1/wood_diffuse.jpg")
        self.cube_1.Material.value.set_uniform("NormalMap", "data/textures/box1/wood_normal.jpg")
        self.cube_1.Tags.value.append("complete_object")
        # self.cube_1_wrapper.Children.value.append(self.cube_1)

        cube_1_rb = av.nodes.RigidBodyNode(
            Name="cube",
            Mass=1.5,
            Friction=0.7,
            Restitution=0.8,
            Transform=self.cube_1.Transform.value
        )
        av.create_box_shape("box", av.Vec3(0.1, 0.1, 0.1))
        cube_1_csn = av.nodes.CollisionShapeNode(
            Name="collision_shape_node",
            ShapeName="box")
        cube_1_csn.Children.value.append(self.cube_1)
        cube_1_rb.Children.value.append(cube_1_csn)
        PARENT_NODE.Children.value.append(cube_1_rb)
        PHYSICS.add_rigid_body(cube_1_rb)

        # # cube 2
        # self.cube_2_wrapper = av.nodes.TransformNode(Name="Cube_2")
        # self.cube_2 = _loader.create_geometry_from_file("cube", "data/objects/cube.obj",
        #                                                 av.LoaderFlags.DEFAULTS | av.LoaderFlags.MAKE_PICKABLE)
        # self.cube_2_wrapper.Transform.value = av.make_trans_mat(0.0, 0.4, 0.0) * \
        #                                       av.make_scale_mat(0.2, 0.2, 0.2)
        # self.cube_2.Material.value.set_uniform("Emissivity", 0.5)
        # self.cube_2.Material.value.set_uniform("Metalness", 0.01)
        # self.cube_2.Material.value.set_uniform("ColorMap", "data/textures/box1/wood_diffuse.jpg")
        # self.cube_2.Material.value.set_uniform("NormalMap", "data/textures/box1/wood_normal.jpg")
        # self.cube_2.Tags.value.append("complete_object")
        # self.cube_2_wrapper.Children.value.append(self.cube_2)
        # PARENT_NODE.Children.value.append(self.cube_2_wrapper)
        #
        # # cube 3
        # self.cube_3_wrapper = av.nodes.TransformNode(Name="Cube_3")
        # self.cube_3 = _loader.create_geometry_from_file("cube", "data/objects/cube.obj",
        #                                                 av.LoaderFlags.DEFAULTS | av.LoaderFlags.MAKE_PICKABLE)
        # self.cube_3_wrapper.Transform.value = av.make_trans_mat(0.0, 0.15, 0.0) * \
        #                                       av.make_scale_mat(0.3, 0.3, 0.3)
        # self.cube_3.Material.value.set_uniform("Emissivity", 0.5)
        # self.cube_3.Material.value.set_uniform("Metalness", 0.01)
        # self.cube_3.Material.value.set_uniform("ColorMap", "data/textures/box1/wood_diffuse.jpg")
        # self.cube_3.Material.value.set_uniform("NormalMap", "data/textures/box1/wood_normal.jpg")
        # self.cube_3.Tags.value.append("complete_object")
        # self.cube_3_wrapper.Children.value.append(self.cube_3)
        # PARENT_NODE.Children.value.append(self.cube_3_wrapper)

        # ground
        self.ground = _loader.create_geometry_from_file("ground", "data/objects/cube.obj",
                                                        av.LoaderFlags.DEFAULTS |
                                                        av.LoaderFlags.LOAD_MATERIALS |
                                                        av.LoaderFlags.NORMALIZE_SCALE |
                                                        av.LoaderFlags.NORMALIZE_POSITION)
        self.ground.Transform.value = av.make_trans_mat(0.0, 0.0, 0.0) * \
                                      av.make_scale_mat(2.0, 0.005, 2.0)
        self.ground.Material.value.set_uniform("ColorMap", "data/textures/ground/bricks_diffuse.jpg")
        self.ground.Material.value.set_uniform("NormalMap", "data/textures/ground/bricks_normal.jpg")
        self.ground.Material.value.set_uniform("Emissivity", 0.05)
        self.ground.Material.value.set_uniform("Metalness", 0.05)
        self.ground.add_and_init_field(av.SFMatrix4(), "HomeMatrix", self.ground.Transform.value)
        self.ground.Tags.value.append("complete_object")

        av.create_box_shape("floor", av.Vec3(2.0, 1.0, 2.0))
        floor_collision_shape = av.nodes.CollisionShapeNode(
            Name="floor_shape",
            ShapeName="floor",
            Children=[self.ground])
        floor_body = av.nodes.RigidBodyNode(Name="floor_body",
                                            Mass=0,
                                            Friction=0.5,
                                            Restitution=0.7,
                                            Children=[floor_collision_shape])
        PARENT_NODE.Children.value.append(floor_body)
        PHYSICS.add_rigid_body(floor_body)

        # tempo
        self.tempo = av.nodes.TransformNode(Name="Tempo")
        self.tempo_geometry = _loader.create_geometry_from_file("tempo", "data/objects/tampo/Tempo.obj",
                                                                av.LoaderFlags.DEFAULTS | av.LoaderFlags.LOAD_MATERIALS | av.LoaderFlags.MAKE_PICKABLE)
        self.tempo_geometry.Transform.value = av.make_rot_mat(-90, 0.0, 1.0, 0.0) * av.make_scale_mat(0.005, 0.005,
                                                                                                      0.005)
        self.tempo.Children.value.append(self.tempo_geometry)
        self.tempo_geometry.Tags.value.append("complete_object")
        PARENT_NODE.Children.value.append(self.tempo)
        self.script.set_tempo(self.tempo)

    ### functions ###
    def reset(self):
        print("reset")
        for _node in self.PARENT_NODE.Children.value:
            if _node.has_field("HomeMatrix") == True:
                _node.Transform.value = _node.HomeMatrix.value
