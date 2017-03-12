import avango
import avango.gua
import avango.script
import math
from avango.script import field_has_changed
import avango.daemon
from gi.overrides.keysyms import R
from enum import Enum
import copy


class VooDollState(Enum):
    DOLL_SELECTION = 1
    NEEDLE_SELECTION = 2
    MANIPULATION = 3


class VooDoll(avango.script.Script):
    sf_button_1 = avango.SFBool()
    sf_button_2 = avango.SFBool()

    ## constructor
    def __init__(self):
        self.super(VooDoll).__init__()

    def my_constructor(self,
        SCENEGRAPH = None,
        NAVIGATION_NODE = None,
        TRACKING_TRANSMITTER_OFFSET=avango.gua.make_identity_mat(),
        POINTER_TRACKING_STATION_1=None,
        POINTER_DEVICE_STATION_1=None,
        POINTER_TRACKING_STATION_2=None,
        POINTER_DEVICE_STATION_2=None,
        HEAD_NODE = None):
        """
        :param SCENEGRAPH:
        :param NAVIGATION_NODE:
        :param POINTERS:
        :type POINTERS: list
        :param HEAD_NODE:
        :return:
        """

        ### external references ###
        self.HEAD_NODE = HEAD_NODE
        self.NAVIGATION_NODE = NAVIGATION_NODE
        self.SCENEGRAPH = SCENEGRAPH

        ## visualization
        self.ray_length = 2.0 # in meter
        self.ray_thickness = 0.0075 # in meter
        self.intersection_point_size = 0.01 # in meter

        self.enable_flag = False

        self.pick_result = None
        self.white_list = []
        self.black_list = ["invisible"]
        self.pick_options = avango.gua.PickingOptions.PICK_ONLY_FIRST_OBJECT \
                            | avango.gua.PickingOptions.GET_POSITIONS \
                            | avango.gua.PickingOptions.GET_NORMALS \
                            | avango.gua.PickingOptions.GET_WORLD_POSITIONS \
                            | avango.gua.PickingOptions.GET_WORLD_NORMALS

        self.ray = avango.gua.nodes.Ray()  # required for trimesh intersection

        self.always_evaluate(True)  # change global evaluation policy

        self.state = VooDollState.DOLL_SELECTION

        self.doll = None
        self.doll_ref = None

        self.needle = None
        self.needle_ref = None

        self.doll_pointer = None

        #region Pointer 1
        self.pointer_tracking_sensor_1 = avango.daemon.nodes.DeviceSensor(DeviceService=avango.daemon.DeviceService())
        self.pointer_tracking_sensor_1.Station.value = POINTER_TRACKING_STATION_1
        self.pointer_tracking_sensor_1.TransmitterOffset.value = TRACKING_TRANSMITTER_OFFSET

        self.pointer_device_sensor_1 = avango.daemon.nodes.DeviceSensor(DeviceService=avango.daemon.DeviceService())
        self.pointer_device_sensor_1.Station.value = POINTER_DEVICE_STATION_1

        self.sf_button_1.connect_from(self.pointer_device_sensor_1.Button0)

        self.pointer_node_1 = avango.gua.nodes.TransformNode(Name="pointer_node_1")
        self.pointer_node_1.Transform.connect_from(self.pointer_tracking_sensor_1.Matrix)
        self.pointer_node_1.Tags.value = ["invisible"]

        NAVIGATION_NODE.Children.value.append(self.pointer_node_1)
        #endregion

        #region Pointer 2
        self.pointer_tracking_sensor_2 = avango.daemon.nodes.DeviceSensor(DeviceService=avango.daemon.DeviceService())
        self.pointer_tracking_sensor_2.Station.value = POINTER_TRACKING_STATION_2
        self.pointer_tracking_sensor_2.TransmitterOffset.value = TRACKING_TRANSMITTER_OFFSET

        self.pointer_device_sensor_2 = avango.daemon.nodes.DeviceSensor(DeviceService=avango.daemon.DeviceService())
        self.pointer_device_sensor_2.Station.value = POINTER_DEVICE_STATION_2

        self.sf_button_2.connect_from(self.pointer_device_sensor_2.Button0)

        self.pointer_node_2 = avango.gua.nodes.TransformNode(Name="pointer_node_2")
        self.pointer_node_2.Transform.connect_from(self.pointer_tracking_sensor_2.Matrix)
        self.pointer_node_2.Tags.value = ["invisible"]

        NAVIGATION_NODE.Children.value.append(self.pointer_node_2)
        #endregion

        self._loader = avango.gua.nodes.TriMeshLoader()


        self.ray_geometry_1 = self._loader.create_geometry_from_file("ray_geometry", "data/objects/cylinder.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.ray_geometry_1.Transform.value = \
            avango.gua.make_trans_mat(0.0,0.0,self.ray_length * -0.5) * \
            avango.gua.make_rot_mat(-90.0,1,0,0) * \
            avango.gua.make_scale_mat(self.ray_thickness, self.ray_length, self.ray_thickness)

        self.ray_geometry_1.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,0.0,1.0))

        self.ray_geometry_2 = self._loader.create_geometry_from_file("ray_geometry", "data/objects/cylinder.obj",
                                                                avango.gua.LoaderFlags.DEFAULTS)
        self.ray_geometry_2.Transform.value = \
            avango.gua.make_trans_mat(0.0, 0.0, self.ray_length * -0.5) * \
            avango.gua.make_rot_mat(-90.0, 1, 0, 0) * \
            avango.gua.make_scale_mat(self.ray_thickness, self.ray_length, self.ray_thickness)

        self.ray_geometry_2.Material.value.set_uniform("Color", avango.gua.Vec4(1.0, 0.0, 0.0, 1.0))

        self.pointer_node_1.Children.value.append(self.ray_geometry_1)
        self.pointer_node_2.Children.value.append(self.ray_geometry_2)

        self.intersection_geometry = self._loader.create_geometry_from_file("intersection_geometry", "data/objects/sphere.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.intersection_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,0.0,1.0))

        self.intersection_geometry.Tags.value = ["invisible"]

        SCENEGRAPH.Root.value.Children.value.append(self.intersection_geometry)

        ### set initial states ###
        self.enable(False)

    def clone(self, node):
        r_node = self._loader.create_geometry_from_file("cube", "data/objects/cube.obj", avango.gua.LoaderFlags.DEFAULTS)
        return r_node

    @staticmethod
    def get_largest_expansion(node):
        expansions = node.BoundingBox.value.Max.value - node.BoundingBox.value.Min.value
        print(node.BoundingBox.value.Min.value)
        print(node.BoundingBox.value.Max.value)
        print(max(expansions.x, expansions.y, expansions.z))
        return max(expansions.x, expansions.y, expansions.z)

    @staticmethod
    def get_relative(node, reference):
        n_mat = node.WorldTransform.value
        r_mat = reference.WorldTransform.value
        return avango.gua.make_inverse_mat(n_mat) * r_mat

    ### functions ###
    def enable(self, BOOL): # extend respective base-class function
        self.enable_flag = BOOL

        if BOOL:
            self.pointer_node_1.Tags.value = []
            self.pointer_node_2.Tags.value = []
        else:
            self.pointer_node_1.Tags.value = ["invisible"]
            self.pointer_node_2.Tags.value = ["invisible"]

    def set_intersection_point(self, pointer):
        if pointer == VooDollPointer.POINTER_1:
            elif pointer == VooDollPointerEnum

    ### callback functions ###
    def evaluate(self): # implement respective base-class function
        if self.enable_flag == False:
            return

        self.calc_pick_result()

        if self.state == VooDollState.DOLL_SELECTION:
            if self.pick_result_1 is None:
                self.intersection_geometry_1.Tags = ["invisible"]
            else:
                self.intersection_geometry_1.Transform.value = ["invisible"]
        elif self.state == VooDollState.NEEDLE_SELECTION:
            self.calc_pick_result()
        elif self.state == VooDollState.MANIPULATION:

    @field_has_changed(sf_button_1)
    def sf_button_1_changed(self):
        print("button 1" + str(self.sf_button_1.value))

    @field_has_changed(sf_button_2)
    def sf_button_2_changed(self):
        print("button 2" + str(self.sf_button_2.value))
