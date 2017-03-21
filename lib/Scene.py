#!/usr/bin/python

import avango
import avango.gua
import avango.gua as av
import avango.script

from avango.script import field_has_changed

import math

import time


class BallSpawner(avango.script.Script):
    SceneGraph = av.SFSceneGraph()
    Physics = av.SFPhysics()
    MaxBallCount = avango.SFFloat()

    def __init__(self):

        self.super(BallSpawner).__init__()

        self.MaxBallCount.value = 50

        self.__last_spawn_time = -1
        self.__loader = av.nodes.TriMeshLoader()

        self.__spawned_balls = []
        self.red = True
        self.always_evaluate(True)

    def evaluate(self):
        current_time = time.time()

        if self.__last_spawn_time == -1 or current_time - self.__last_spawn_time >= 0.8:
            self.__last_spawn_time = current_time

            if self.red:
                body = av.nodes.RigidBodyNode(
                    Name="body",
                    Mass=1.5,
                    Friction=0.7,
                    RollingFriction=0.04,
                    Restitution=0.8,
                    Transform=av.make_trans_mat(0.0, 5.0, 0.0))
            else:
                body = av.nodes.RigidBodyNode(
                    Name="body",
                    Mass=2.0,
                    Friction=0.6,
                    RollingFriction=0.03,
                    Restitution=0.3,
                    Transform=av.make_trans_mat(0.0, 5.0, 0.0))

            sphere_geometry = self.__loader.create_geometry_from_file(
                "sphere_geometry", "data/objects/sphere.obj")

            sphere_geometry.Transform.value = av.make_scale_mat(0.1, 0.1, 0.1)

            if self.red:
                sphere_geometry.Material.value.set_uniform(
                    # "Color", av.Vec4(0.08, 0.08, 0.09, 1.0))
                    "Color",
                    av.Vec4(0.9, 0.266, 0.136, 1.0))
                sphere_geometry.Material.value.set_uniform("Roughness", 0.75)
                sphere_geometry.Material.value.set_uniform("Metalness", 0.0)

            else:
                sphere_geometry.Material.value.set_uniform(
                    "Color", av.Vec4(1.0, 1.0, 1.0, 1.0))
                sphere_geometry.Material.value.set_uniform("Roughness", 0.2)
                sphere_geometry.Material.value.set_uniform("Metalness", 0.0)

            self.red = not self.red

            collision_shape_node = av.nodes.CollisionShapeNode(
                Name="collision_shape_node",
                ShapeName="sphere")

            collision_shape_node.Children.value.append(sphere_geometry)
            body.Children.value.append(collision_shape_node)
            self.SceneGraph.value.Root.value.Children.value.append(body)
            self.Physics.value.add_rigid_body(body)

            self.__spawned_balls.append(body)

            if len(self.__spawned_balls) > self.MaxBallCount.value:
                to_remove = self.__spawned_balls.pop(0)
                self.Physics.value.remove_rigid_body(to_remove)
                self.SceneGraph.value.Root.value.Children.value.remove(
                    to_remove)


class SceneScript(avango.script.Script):
    ## input fields
    sf_reset_button = avango.SFBool()
    Physics = av.SFPhysics()
    SceneGraph = av.SFSceneGraph()

    ## Default constructor.
    def __init__(self):
        self.super(SceneScript).__init__()

        ### external references ###
        self.CLASS = None  # is set later

        ### resources ###
        self.keyboard_device_sensor = avango.daemon.nodes.DeviceSensor(DeviceService=avango.daemon.DeviceService())
        self.keyboard_device_sensor.Station.value = "gua-device-keyboard0"

        self.sf_reset_button.connect_from(self.keyboard_device_sensor.Button14)  # spacebar key
        self._loader = av.nodes.TriMeshLoader()

        self.tempo = None
        self.cube_1 = None
        self.cube_2 = None
        self.ground = None
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

        if self.ground is None:
            self.ground = self._loader.create_geometry_from_file("ground", "data/objects/cube.obj",
                                                                 av.LoaderFlags.DEFAULTS |
                                                                 av.LoaderFlags.LOAD_MATERIALS |
                                                                 av.LoaderFlags.NORMALIZE_SCALE |
                                                                 av.LoaderFlags.NORMALIZE_POSITION |
                                                                 av.LoaderFlags.OPTIMIZE_GEOMETRY
                                                                 )
            self.ground.Material.value.set_uniform("ColorMap", "data/textures/ground/bricks_diffuse.jpg")
            self.ground.Material.value.set_uniform("NormalMap", "data/textures/ground/bricks_normal.jpg")
            self.ground.Material.value.set_uniform("Emissivity", 0.05)
            self.ground.Material.value.set_uniform("Metalness", 0.05)
            self.ground.add_and_init_field(av.SFMatrix4(), "HomeMatrix", self.ground.Transform.value)

            self.spawn_ground(self.ground)

            if self.cube_1 is None:
                self.cube_1 = self._loader.create_geometry_from_file("cube", "data/objects/cube.obj",
                                                                     av.LoaderFlags.DEFAULTS |
                                                                     av.LoaderFlags.MAKE_PICKABLE |
                                                                     av.LoaderFlags.NORMALIZE_SCALE |
                                                                     av.LoaderFlags.NORMALIZE_POSITION
                                                                     )
                self.cube_1.Material.value.set_uniform("Emissivity", 0.5)
                self.cube_1.Material.value.set_uniform("Metalness", 0.01)
                self.cube_1.Material.value.set_uniform("Roughness", 0.75)
                self.cube_1.Material.value.set_uniform("ColorMap", "data/textures/box1/wood_diffuse.jpg")
                self.cube_1.Material.value.set_uniform("NormalMap", "data/textures/box1/wood_normal.jpg")
                self.cube_1.Tags.value.append("complete_object")

                self.spawn_cube(self.cube_1, 200.0, av.make_trans_mat(0.0, 0.55, 0.0) * av.make_scale_mat(0.1, 0.1, 0.1))
                # else:
                #     print(self.cube_1.WorldTransform.value.get_translate())
                # print(self.cube_1.Parent.value.Parent.value.WorldTransform.value)
                # print(self.cube_1.Parent.value.WorldTransform.value)
                # print(self.cube_1.WorldTransform.value)

            if self.cube_2 is None:
                self.cube_2 = self._loader.create_geometry_from_file("cube", "data/objects/cube.obj",
                                                                     av.LoaderFlags.DEFAULTS |
                                                                     av.LoaderFlags.MAKE_PICKABLE |
                                                                     av.LoaderFlags.NORMALIZE_SCALE |
                                                                     av.LoaderFlags.NORMALIZE_POSITION
                                                                     )
                self.cube_2.Material.value.set_uniform("Emissivity", 0.5)
                self.cube_2.Material.value.set_uniform("Metalness", 0.01)
                self.cube_1.Material.value.set_uniform("Roughness", 0.75)
                self.cube_2.Material.value.set_uniform("ColorMap", "data/textures/box1/wood_diffuse.jpg")
                self.cube_2.Material.value.set_uniform("NormalMap", "data/textures/box1/wood_normal.jpg")
                self.cube_2.Tags.value.append("complete_object")

                self.spawn_cube(self.cube_2, 400.0, av.make_trans_mat(0.0, 0.35, 0.0) * av.make_scale_mat(0.2, 0.2, 0.2))
                # else:
                #     print(self.cube_2.WorldTransform.value.get_translate())

    def spawn_cube(self, cube_geometry, mass, mat):
        cube_rb = av.nodes.RigidBodyNode(Name="cube_rb",
                                         Mass=mass,
                                         Friction=0.6,
                                         Restitution=1.0,
                                         IsKinematic=True,
                                         )
        cube_rb.Transform.value = mat
        self.Physics.value.add_rigid_body(cube_rb)

        cube_cs = av.create_triangle_mesh_shape_from_geometry_file(
            "cube_cs",
            "data/objects/cube.obj",
            False,
            True,
            av.LoaderFlags.DEFAULTS
        )

        # cube_cs.Scaling.value = cube_geometry.Transform.value.get_scale()

        cube_csn = av.nodes.CollisionShapeNode(Name="cube_csn", ShapeName="cube_cs")

        cube_csn.Children.value.append(cube_geometry)
        cube_rb.Children.value.append(cube_csn)

        self.SceneGraph.value.Root.value.Children.value.append(cube_rb)

        SceneScript.print_graph(self.SceneGraph.value.Root.value)
        # print(cube_csn.Transform.value)
        # print(cube_rb.Transform.value)
        # print(cube_geometry.Transform.value)

    def spawn_ground(self, floor_geometry):
        floor_rb = av.nodes.RigidBodyNode(Name="floor_rb",
                                          Mass=0.0,
                                          Friction=0.5,
                                          Restitution=0.7,
                                          # IsKinematic=False,
                                          )
        floor_rb.Transform.value = av.make_scale_mat(8.0, 0.005, 8.0)
        self.Physics.value.add_rigid_body(floor_rb)

        av.create_box_shape("floor_cs", av.Vec3(8.0, 0.005, 8.0))

        floor_csn = av.nodes.CollisionShapeNode(Name="floor_csn", ShapeName="floor_cs")

        floor_csn.Children.value.append(floor_geometry)
        floor_rb.Children.value.append(floor_csn)

        self.SceneGraph.value.Root.value.Children.value.append(floor_rb)

        SceneScript.print_graph(self.SceneGraph.value.Root.value)
        # print(floor_csn.Transform.value)
        # print(floor_rb.Transform.value)
        # print(floor_geometry.Transform.value)

    @staticmethod
    def print_graph(root_node):
        stack = [(root_node, 0)]
        while stack:
            node, level = stack.pop()
            print("│   " * level + "├── {0} <{1}>".format(
                node.Name.value, node.__class__.__name__))
            stack.extend(
                [(child, level + 1) for child in reversed(node.Children.value)])


class Scene():
    ## constructor
    def __init__(self,
                 SCENEGRAPH=None,
                 PARENT_NODE=None,
                 PHYSICS=None
                 ):

        ### external reference ###
        self.PARENT_NODE = PARENT_NODE
        self.PHYSICS = PHYSICS

        av.create_sphere_shape("sphere", 0.1)

        spawner = BallSpawner()
        spawner.SceneGraph.value = SCENEGRAPH
        spawner.Physics.value = PHYSICS

        ### resources ###
        self.script = SceneScript()
        self.script.my_constructor(self)
        self.script.SceneGraph.value = SCENEGRAPH
        self.script.Physics.value = PHYSICS

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
