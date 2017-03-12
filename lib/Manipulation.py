#!/usr/bin/python

### import guacamole libraries ###
import avango
import avango.gua
import avango.script
import math
from avango.script import field_has_changed
import avango.daemon
from lib.VooDoll import VooDoll
from gi.overrides.keysyms import R


class ManipulationManager(avango.script.Script):
    ## input fields
    sf_key_1 = avango.SFBool()
    sf_key_2 = avango.SFBool()
    sf_key_3 = avango.SFBool()
    sf_key_4 = avango.SFBool()
    sf_key_5 = avango.SFBool()

    ## constructor
    def __init__(self):
        self.super(ManipulationManager).__init__()

    def my_constructor(self,
                       SCENEGRAPH=None,
                       NAVIGATION_NODE=None,
                       POINTER_TRACKING_STATION="",
                       TRACKING_TRANSMITTER_OFFSET=avango.gua.make_identity_mat(),
                       POINTER_DEVICE_STATION="",
                       HEAD_NODE=None,
                       ):

        ### variables ###
        self.active_manipulation_technique = None

        ### resources ###
        self.keyboard_sensor = avango.daemon.nodes.DeviceSensor(DeviceService=avango.daemon.DeviceService())
        self.keyboard_sensor.Station.value = "gua-device-keyboard0"

        self.sf_key_1.connect_from(self.keyboard_sensor.Button16)  # key 1
        self.sf_key_2.connect_from(self.keyboard_sensor.Button17)  # key 2
        self.sf_key_3.connect_from(self.keyboard_sensor.Button18)  # key 3
        self.sf_key_4.connect_from(self.keyboard_sensor.Button19)  # key 4
        self.sf_key_5.connect_from(self.keyboard_sensor.Button20)  # key 5

        ## init manipulation techniques
        self.virtualRay = VirtualRay()
        self.virtualRay.my_constructor(SCENEGRAPH, NAVIGATION_NODE, POINTER_TRACKING_STATION,
                                       TRACKING_TRANSMITTER_OFFSET, POINTER_DEVICE_STATION)

        self.virtualHand = VirtualHand()
        self.virtualHand.my_constructor(SCENEGRAPH, NAVIGATION_NODE, POINTER_TRACKING_STATION,
                                        TRACKING_TRANSMITTER_OFFSET, POINTER_DEVICE_STATION)

        self.goGo = GoGo()
        self.goGo.my_constructor(SCENEGRAPH, NAVIGATION_NODE, POINTER_TRACKING_STATION, TRACKING_TRANSMITTER_OFFSET,
                                 POINTER_DEVICE_STATION, HEAD_NODE)

        self.homer = Homer()
        self.homer.my_constructor(SCENEGRAPH, NAVIGATION_NODE, POINTER_TRACKING_STATION, TRACKING_TRANSMITTER_OFFSET,
                                  POINTER_DEVICE_STATION, HEAD_NODE)

        self.voodoll = VooDoll()
        self.voodoll.my_constructor(SCENEGRAPH, NAVIGATION_NODE,
                                    [[POINTER_TRACKING_STATION, TRACKING_TRANSMITTER_OFFSET, "device-pointer-2"],
                                     [POINTER_TRACKING_STATION, TRACKING_TRANSMITTER_OFFSET, "device-pointer-1"]],
                                    HEAD_NODE)

        ### set initial states ###
        self.set_manipulation_technique(0)  # switch to virtual-ray manipulation technique

    ### functions ###
    def set_manipulation_technique(self, INT):
        # possibly disable prior technique
        if self.active_manipulation_technique is not None:
            self.active_manipulation_technique.enable(False)

        # enable new technique
        if INT == 0:  # virtual-ray
            print("switch to Virtual-Ray technique")
            self.active_manipulation_technique = self.virtualRay

        elif INT == 1:  # virtual-hand
            print("switch to Virtual-Hand technique")
            self.active_manipulation_technique = self.virtualHand

        elif INT == 2:  # go-go
            print("switch to Go-Go technique")
            self.active_manipulation_technique = self.goGo

        elif INT == 3:  # HOMER
            print("switch to HOMER technique")
            self.active_manipulation_technique = self.homer

        elif INT == 4: # VooDoll
            print("switch to VooDoll technique")
            self.active_manipulation_technique = self.voodoll


        self.active_manipulation_technique.enable(True)

    ### callback functions ###
    @field_has_changed(sf_key_1)
    def sf_key_1_changed(self):
        if self.sf_key_1.value == True:  # key is pressed
            self.set_manipulation_technique(0)  # switch to Virtual-Ray manipulation technique

    @field_has_changed(sf_key_2)
    def sf_key_2_changed(self):
        if self.sf_key_2.value == True:  # key is pressed
            self.set_manipulation_technique(1)  # switch to Virtual-Hand manipulation technique

    @field_has_changed(sf_key_3)
    def sf_key_3_changed(self):
        if self.sf_key_3.value == True:  # key is pressed
            self.set_manipulation_technique(2)  # switch to Go-Go manipulation technique

    @field_has_changed(sf_key_4)
    def sf_key_4_changed(self):
        if self.sf_key_4.value == True:  # key is pressed
            self.set_manipulation_technique(3)  # switch to HOMER manipulation technique

    @field_has_changed(sf_key_5)
    def sf_key_5_changed(self):
        if self.sf_key_5.value == True:  # key is pressed
            self.set_manipulation_technique(4)  # switch to VooDoll manipulation technique



class ManipulationTechnique(avango.script.Script):
    ## input fields
    sf_button = avango.SFBool()

    ## constructor
    def __init__(self):
        self.super(ManipulationTechnique).__init__()

    def my_constructor(self,
                       SCENEGRAPH=None,
                       NAVIGATION_NODE=None,
                       POINTER_TRACKING_STATION=None,
                       TRACKING_TRANSMITTER_OFFSET=avango.gua.make_identity_mat(),
                       POINTER_DEVICE_STATION=None,
                       ):

        ### external references ###
        self.SCENEGRAPH = SCENEGRAPH

        ### variables ###
        self.enable_flag = False

        ## dragging
        self.dragged_node = None
        self.dragging_offset_mat = avango.gua.make_identity_mat()

        ## picking
        self.pick_result = None

        self.white_list = []
        self.black_list = ["invisible"]

        self.pick_options = avango.gua.PickingOptions.PICK_ONLY_FIRST_OBJECT \
                            | avango.gua.PickingOptions.GET_POSITIONS \
                            | avango.gua.PickingOptions.GET_NORMALS \
                            | avango.gua.PickingOptions.GET_WORLD_POSITIONS \
                            | avango.gua.PickingOptions.GET_WORLD_NORMALS

        ## init sensors
        self.pointer_tracking_sensor = avango.daemon.nodes.DeviceSensor(DeviceService=avango.daemon.DeviceService())
        self.pointer_tracking_sensor.Station.value = POINTER_TRACKING_STATION
        self.pointer_tracking_sensor.TransmitterOffset.value = TRACKING_TRANSMITTER_OFFSET

        self.pointer_device_sensor = avango.daemon.nodes.DeviceSensor(DeviceService=avango.daemon.DeviceService())
        self.pointer_device_sensor.Station.value = POINTER_DEVICE_STATION

        ## init field connections
        self.sf_button.connect_from(self.pointer_device_sensor.Button0)

        ## init nodes
        self.pointer_node = avango.gua.nodes.TransformNode(Name="pointer_node")
        self.pointer_node.Transform.connect_from(self.pointer_tracking_sensor.Matrix)
        self.pointer_node.Tags.value = ["invisible"]
        NAVIGATION_NODE.Children.value.append(self.pointer_node)

        self.ray = avango.gua.nodes.Ray()  # required for trimesh intersection

        self.always_evaluate(True)  # change global evaluation policy

    ### functions ###
    def enable(self, BOOL):
        self.enable_flag = BOOL

        if self.enable_flag == True:
            self.pointer_node.Tags.value = []  # set tool visible
        else:
            self.stop_dragging()  # possibly stop active dragging process

            self.pointer_node.Tags.value = ["invisible"]  # set tool invisible

    def calc_pick_result(self, PICK_MAT=avango.gua.make_identity_mat(), PICK_LENGTH=1.0):
        ## update ray parameters
        self.ray.Origin.value = PICK_MAT.get_translate()

        _vec = avango.gua.make_rot_mat(PICK_MAT.get_rotate_scale_corrected()) * avango.gua.Vec3(0.0, 0.0, -1.0)
        _vec = avango.gua.Vec3(_vec.x, _vec.y, _vec.z)

        self.ray.Direction.value = _vec * PICK_LENGTH

        ## intersect
        _mf_pick_result = self.SCENEGRAPH.ray_test(self.ray, self.pick_options, self.white_list, self.black_list)

        return _mf_pick_result

    def start_dragging(self, NODE):
        self.dragged_node = NODE
        self.dragging_offset_mat = avango.gua.make_inverse_mat(
            self.pointer_node.WorldTransform.value) * self.dragged_node.WorldTransform.value  # object transformation in pointer coordinate system

    def stop_dragging(self):
        self.dragged_node = None
        self.dragging_offset_mat = avango.gua.make_identity_mat()

    def dragging(self):
        if self.dragged_node is not None:  # object to drag
            _new_mat = self.pointer_node.WorldTransform.value * self.dragging_offset_mat  # new object position in world coodinates
            _new_mat = avango.gua.make_inverse_mat(
                self.dragged_node.Parent.value.WorldTransform.value) * _new_mat  # transform new object matrix from global to local space

            self.dragged_node.Transform.value = _new_mat

    ### callback functions ###

    @field_has_changed(sf_button)
    def sf_button_changed(self):
        if self.sf_button.value == True:  # button pressed
            if self.pick_result is not None:  # something was hit
                _node = self.pick_result.Object.value  # get intersected geometry node
                _node = _node.Parent.value  # take the parent node of the geomtry node (the whole object)

                self.start_dragging(_node)

        else:  # button released
            self.stop_dragging()

    def evaluate(self):  # evaluated every frame
        raise NotImplementedError("To be implemented by a subclass.")


class VirtualRay(ManipulationTechnique):
    ## constructor
    def __init__(self):
        self.super(VirtualRay).__init__()

    def my_constructor(self,
                       SCENEGRAPH=None,
                       NAVIGATION_NODE=None,
                       POINTER_TRACKING_STATION=None,
                       TRACKING_TRANSMITTER_OFFSET=avango.gua.make_identity_mat(),
                       POINTER_DEVICE_STATION=None,
                       ):

        ManipulationTechnique.my_constructor(self, SCENEGRAPH, NAVIGATION_NODE, POINTER_TRACKING_STATION,
                                             TRACKING_TRANSMITTER_OFFSET,
                                             POINTER_DEVICE_STATION)  # call base class constructor

        ### additional parameters ###

        ## visualization
        self.ray_length = 2.0  # in meter
        self.ray_thickness = 0.0075  # in meter

        self.intersection_point_size = 0.01  # in meter

        ### additional resources ###
        _loader = avango.gua.nodes.TriMeshLoader()

        self.ray_geometry = _loader.create_geometry_from_file("ray_geometry", "data/objects/cylinder.obj",
                                                              avango.gua.LoaderFlags.DEFAULTS)
        self.ray_geometry.Transform.value = \
            avango.gua.make_trans_mat(0.0, 0.0, self.ray_length * -0.5) * \
            avango.gua.make_rot_mat(-90.0, 1, 0, 0) * \
            avango.gua.make_scale_mat(self.ray_thickness, self.ray_length, self.ray_thickness)
        self.ray_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0, 0.0, 0.0, 1.0))
        self.pointer_node.Children.value.append(self.ray_geometry)

        self.intersection_geometry = _loader.create_geometry_from_file("intersection_geometry",
                                                                       "data/objects/sphere.obj",
                                                                       avango.gua.LoaderFlags.DEFAULTS)
        self.intersection_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0, 0.0, 0.0, 1.0))
        SCENEGRAPH.Root.value.Children.value.append(self.intersection_geometry)

        ### set initial states ###
        self.enable(False)

    ### functions ###
    def enable(self, BOOL):  # extend respective base-class function
        ManipulationTechnique.enable(self, BOOL)  # call base-class function

        if self.enable_flag == False:
            self.intersection_geometry.Tags.value = ["invisible"]  # set intersection point invisible

    def update_ray_visualization(self, PICK_WORLD_POS=None, PICK_DISTANCE=0.0):
        if PICK_WORLD_POS is None:  # nothing hit
            self.ray_geometry.Transform.value = \
                avango.gua.make_trans_mat(0.0, 0.0, self.ray_length * -0.5) * \
                avango.gua.make_rot_mat(-90.0, 1, 0, 0) * \
                avango.gua.make_scale_mat(self.ray_thickness, self.ray_length, self.ray_thickness)

            self.intersection_geometry.Tags.value = ["invisible"]  # set intersection point invisible

        else:  # something hit
            self.ray_geometry.Transform.value = \
                avango.gua.make_trans_mat(0.0, 0.0, PICK_DISTANCE * -0.5) * \
                avango.gua.make_rot_mat(-90.0, 1, 0, 0) * \
                avango.gua.make_scale_mat(self.ray_thickness, PICK_DISTANCE, self.ray_thickness)

            self.intersection_geometry.Tags.value = []  # set intersection point visible
            self.intersection_geometry.Transform.value = avango.gua.make_trans_mat(
                PICK_WORLD_POS) * avango.gua.make_scale_mat(self.intersection_point_size)

    ### callback functions ###
    def evaluate(self):  # implement respective base-class function
        if not self.enable_flag:
            return

        ## calc ray intersection
        _mf_pick_result = self.calc_pick_result(PICK_MAT=self.pointer_node.WorldTransform.value,
                                                PICK_LENGTH=self.ray_length)
        # print("hits:", len(_mf_pick_result.value))

        if len(_mf_pick_result.value) > 0:  # intersection found
            self.pick_result = _mf_pick_result.value[0]  # get first pick result
        else:  # nothing hit
            self.pick_result = None

        ## update visualizations
        if self.pick_result is None:
            self.update_ray_visualization()  # apply default ray visualization
        else:
            _node = self.pick_result.Object.value  # get intersected geometry node

            _pick_pos = self.pick_result.Position.value  # pick position in object coordinate system
            _pick_world_pos = self.pick_result.WorldPosition.value  # pick position in world coordinate system

            _distance = self.pick_result.Distance.value * self.ray_length  # pick distance in ray coordinate system

            print(_node, _pick_pos, _pick_world_pos, _distance)

            self.update_ray_visualization(PICK_WORLD_POS=_pick_world_pos, PICK_DISTANCE=_distance)

        ## possibly update object dragging
        self.dragging()


class VirtualHand(ManipulationTechnique):
    ## constructor
    def __init__(self):
        self.super(VirtualHand).__init__()

    def my_constructor(self,
                       SCENEGRAPH=None,
                       NAVIGATION_NODE=None,
                       POINTER_TRACKING_STATION=None,
                       TRACKING_TRANSMITTER_OFFSET=avango.gua.make_identity_mat(),
                       POINTER_DEVICE_STATION=None,
                       ):

        ManipulationTechnique.my_constructor(self, SCENEGRAPH, NAVIGATION_NODE, POINTER_TRACKING_STATION,
                                             TRACKING_TRANSMITTER_OFFSET,
                                             POINTER_DEVICE_STATION)  # call base class constructor

        ### additional parameters ###
        self.hand_size = 0.1  # in meter

        ### further resources ###
        _loader = avango.gua.nodes.TriMeshLoader()

        self.hand_geometry = _loader.create_geometry_from_file("hand_geometry", "data/objects/hand.obj",
                                                               avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.LOAD_MATERIALS)
        self.hand_geometry.Transform.value = \
            avango.gua.make_trans_mat(0.0, 0.0, self.hand_size)
        self.hand_geometry.Tags.value = ["invisible"]

        self.pointer_node.Children.value.append(self.hand_geometry)

        ### set initial states ###
        self.enable(False)

    ### callback functions ###
    def evaluate(self):  # implement respective base-class function
        if self.enable_flag == False:
            self.hand_geometry.Tags.value = ["invisible"]
            return

        self.hand_geometry.Tags.value = []

        _mf_pick_result = self.calc_pick_result(PICK_MAT=self.hand_geometry.WorldTransform.value,
                                                PICK_LENGTH=self.hand_size)

        if len(_mf_pick_result.value) > 0:
            self.pick_result = _mf_pick_result.value[0]
        else:
            self.pick_result = None

        self.dragging()


class GoGo(ManipulationTechnique):
    ## constructor
    def __init__(self):
        self.super(GoGo).__init__()

    def my_constructor(self,
                       SCENEGRAPH=None,
                       NAVIGATION_NODE=None,
                       POINTER_TRACKING_STATION=None,
                       TRACKING_TRANSMITTER_OFFSET=avango.gua.make_identity_mat(),
                       POINTER_DEVICE_STATION=None,
                       HEAD_NODE=None
                       ):

        ManipulationTechnique.my_constructor(self, SCENEGRAPH, NAVIGATION_NODE, POINTER_TRACKING_STATION,
                                             TRACKING_TRANSMITTER_OFFSET,
                                             POINTER_DEVICE_STATION)  # call base class constructor

        ### additional parameters ###
        self.hand_size = 0.1  # in meter

        ### further resources ###
        _loader = avango.gua.nodes.TriMeshLoader()

        self.hand_geometry = _loader.create_geometry_from_file("hand_geometry", "data/objects/hand.obj",
                                                               avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.LOAD_MATERIALS)
        self.hand_geometry.Transform.value = avango.gua.make_trans_mat(0.0, 0.0, self.hand_size)
        self.pointer_node.Children.value.append(self.hand_geometry)

        # Disconnect from input
        self.pointer_node.Transform.disconnect_from(self.pointer_tracking_sensor.Matrix)

        self.head_node = HEAD_NODE

        ### set initial states ###
        self.enable(False)

    @staticmethod
    def calc_sq(distance):
        return math.pow(distance, 2) + 0.3 * distance + 0.122

    @staticmethod
    def calc_exp(distance):
        return 0.003 * math.exp(10 * distance) + 0.25

    ### callback functions ###
    def evaluate(self):  # implement respective base-class function
        if not self.enable_flag:
            self.hand_geometry.Tags.value = ["invisible"]
            return

        self.hand_geometry.Tags.value = []

        base_head_position = self.head_node.Transform.value.get_translate()
        base_pointer_position = self.pointer_tracking_sensor.Matrix.value.get_translate()

        distance = base_pointer_position - base_head_position
        abs_distance = math.sqrt(math.pow(distance.x, 2) + math.pow(distance.z, 2))  # + math.pow(distance.z, 2))

        # print(base_head_position)
        # print(base_pointer_position)
        # print(distance)

        if abs_distance > 0.35:
            # self.hand_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(0.0, 1.0, 0.0, 1.0))

            distance = distance * GoGo.calc_exp(abs_distance) / abs_distance - distance
            # print(distance)b

            self.pointer_node.Transform.value = avango.gua.make_trans_mat(base_pointer_position.x + distance.x,
                                                                          base_pointer_position.y + distance.y,
                                                                          base_pointer_position.z + distance.z) * \
                                                avango.gua.make_rot_mat(
                                                    self.pointer_tracking_sensor.Matrix.value.get_rotate())
        else:
            # self.hand_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0, 0.0, 0.0, 1.0))

            self.pointer_node.Transform.value = self.pointer_tracking_sensor.Matrix.value

        _mf_pick_result = self.calc_pick_result(PICK_MAT=self.hand_geometry.WorldTransform.value,
                                                PICK_LENGTH=self.hand_size)

        if len(_mf_pick_result.value) > 0:
            self.pick_result = _mf_pick_result.value[0]
        else:
            self.pick_result = None

        self.dragging()


class Homer(ManipulationTechnique):
    ## constructor
    def __init__(self):
        self.super(Homer).__init__()

    def my_constructor(self,
                       SCENEGRAPH=None,
                       NAVIGATION_NODE=None,
                       POINTER_TRACKING_STATION=None,
                       TRACKING_TRANSMITTER_OFFSET=avango.gua.make_identity_mat(),
                       POINTER_DEVICE_STATION=None,
                       HEAD_NODE=None,
                       ):
        ManipulationTechnique.my_constructor(self, SCENEGRAPH, NAVIGATION_NODE, POINTER_TRACKING_STATION,
                                             TRACKING_TRANSMITTER_OFFSET,
                                             POINTER_DEVICE_STATION)  # call base class constructor

        ### external references ###
        self.NAVIGATION_NODE = NAVIGATION_NODE
        self.HEAD_NODE = HEAD_NODE
        self.SCENEGRAPH = SCENEGRAPH

        ### additional parameters ###

        self.ray_length = 2.0
        self.ray_thickness = 0.0075
        self.intersection_point_size = 0.01

        _loader = avango.gua.nodes.TriMeshLoader()

        self.hand_geometry = _loader.create_geometry_from_file("hand_geometry", "data/objects/hand.obj",
                                                               avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.LOAD_MATERIALS)
        self.hand_geometry.Transform.value = avango.gua.make_trans_mat(0.0, 0.0, 0.1)
        self.hand_geometry.Tags.value = []

        self.ray_geometry = _loader.create_geometry_from_file("ray_geometry", "data/objects/cylinder.obj",
                                                              avango.gua.LoaderFlags.DEFAULTS)
        self.ray_geometry.Transform.value = \
            avango.gua.make_trans_mat(0.0, 0.0, self.ray_length * -0.5) * \
            avango.gua.make_rot_mat(-90.0, 1, 0, 0) * \
            avango.gua.make_scale_mat(self.ray_thickness, self.ray_length, self.ray_thickness)
        self.ray_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0, 0.0, 0.0, 1.0))
        self.ray_geometry.Tags.value = []

        self.pointer_node.Children.value.append(self.ray_geometry)
        self.pointer_node.Children.value.append(self.hand_geometry)

        self.intersection_geometry = _loader.create_geometry_from_file("intersection_geometry",
                                                                       "data/objects/sphere.obj",
                                                                       avango.gua.LoaderFlags.DEFAULTS)
        self.intersection_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0, 0.0, 0.0, 1.0))
        self.intersection_geometry.Tags.value = ["invisible"]
        SCENEGRAPH.Root.value.Children.value.append(self.intersection_geometry)

        self.enable(False)

    def evaluate(self):
        if not self.enable_flag:
            return

        _mf_pick_result = self.calc_pick_result(PICK_MAT=self.pointer_node.WorldTransform.value,
                                                PICK_LENGTH=self.ray_length)

        if len(_mf_pick_result.value) > 0:
            self.pick_result = _mf_pick_result.value[0]
            _pick_world_pos = self.pick_result.WorldPosition.value
            _distance = self.pick_result.Distance.value * self.ray_length
            self.ray_geometry.Transform.value = \
                avango.gua.make_trans_mat(0.0, 0.0, _distance * -0.5) * \
                avango.gua.make_rot_mat(-90.0, 1, 0, 0) * \
                avango.gua.make_scale_mat(self.ray_thickness, _distance, self.ray_thickness)

            self.intersection_geometry.Tags.value = []
            self.intersection_geometry.Transform.value = avango.gua.make_trans_mat(
                _pick_world_pos) * avango.gua.make_scale_mat(self.intersection_point_size)
        else:
            self.pick_result = None
            self.ray_geometry.Transform.value = \
                avango.gua.make_trans_mat(0.0, 0.0, self.ray_length * -0.5) * \
                avango.gua.make_rot_mat(-90.0, 1, 0, 0) * \
                avango.gua.make_scale_mat(self.ray_thickness, self.ray_length, self.ray_thickness)
            self.intersection_geometry.Tags.value = ["invisible"]

        self.dragging()

    def position_hand_at_target(self):
        if self.dragged_node is None and self.pick_result is not None:
            pointer_target_vector = self.pick_result.WorldPosition.value - self.pointer_node.WorldTransform.value.get_translate()
            pointer_target_distance = self.get_distance_3d(pointer_target_vector)
            hand_grab_diff = pointer_target_vector / pointer_target_distance * 0.05
            self.hand_geometry.WorldTransform.value = \
                avango.gua.make_scale_mat(self.hand_geometry.WorldTransform.value.get_scale()) * \
                avango.gua.make_trans_mat(self.pick_result.WorldPosition.value - hand_grab_diff) * \
                avango.gua.make_rot_mat(self.pointer_node.WorldTransform.value.get_rotate())
            self.intersection_geometry.Tags.value = ["invisible"]
        elif self.dragged_node is not None:
            pointer_target_vector = self.dragged_node.WorldTransform.value.get_translate() - self.pointer_node.WorldTransform.value.get_translate()
            pointer_target_distance = self.get_distance_3d(pointer_target_vector)
            hand_grab_diff = pointer_target_vector / pointer_target_distance * 0.05
            self.hand_geometry.WorldTransform.value = \
                avango.gua.make_scale_mat(self.hand_geometry.WorldTransform.value.get_scale()) * \
                avango.gua.make_trans_mat(self.dragged_node.WorldTransform.value.get_translate() - hand_grab_diff) * \
                avango.gua.make_rot_mat(self.pointer_node.WorldTransform.value.get_rotate())
            self.intersection_geometry.Tags.value = ["invisible"]
        else:
            self.intersection_geometry.Tags.value = []

    @staticmethod
    def get_distance_3d(vec):
        return math.sqrt(math.pow(vec.x, 2) + math.pow(vec.y, 2) + math.pow(vec.z, 2))

    def start_dragging(self, NODE):
        self.dragged_node = NODE
        self.dragging_offset_mat = avango.gua.make_inverse_mat(self.pointer_node.WorldTransform.value) * self.dragged_node.WorldTransform.value
        self.position_hand_at_target()

        self.origin_head_pointer_vector = self.pointer_node.WorldTransform.value.get_translate() - self.HEAD_NODE.WorldTransform.value.get_translate()
        self.origin_head_target_vector = self.dragged_node.WorldTransform.value.get_translate() - self.HEAD_NODE.WorldTransform.value.get_translate()
        self.origin_pointer_target_vector = self.dragged_node.WorldTransform.value.get_translate() - self.pointer_node.WorldTransform.value.get_translate()

        self.pointer_node.Children.value.remove(self.ray_geometry)

    def stop_dragging(self):
        if self.dragged_node is not None:
            self.dragged_node = None
            # self.dragging_offset_mat = avango.gua.make_identity_mat()
            self.hand_geometry.Transform.value = avango.gua.make_trans_mat(0.0, 0.0, 0.1)
            self.pointer_node.Children.value.append(self.ray_geometry)

    def dragging(self):
        if self.dragged_node is not None:
            head_pointer_vector = self.pointer_node.WorldTransform.value.get_translate() - self.HEAD_NODE.WorldTransform.value.get_translate()
            head_target_vector = self.origin_head_target_vector + (head_pointer_vector - self.origin_head_pointer_vector) * Homer.get_distance_3d(self.origin_head_target_vector) / Homer.get_distance_3d(self.origin_head_pointer_vector)

            self.dragged_node.WorldTransform.value = avango.gua.make_trans_mat(self.HEAD_NODE.WorldTransform.value.get_translate() + head_target_vector) * \
                                                     avango.gua.make_rot_mat(self.pointer_node.WorldTransform.value.get_rotate()) * \
                                                     avango.gua.make_scale_mat(self.dragged_node.WorldTransform.value.get_scale())

            self.position_hand_at_target()