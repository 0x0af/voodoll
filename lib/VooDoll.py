import avango
import avango.gua
import avango.script
import math
from avango.script import field_has_changed
import avango.daemon
from gi.overrides.keysyms import R
from enum import Enum


class VooDollState(Enum):
    DOLL_SELECTION = 1
    NEEDLE_SELECTION = 2
    MANIPULATION = 3


class VooDoll(avango.script.Script):
    #sf_buttons = avango.MFBool()

    ## constructor
    def __init__(self):
        self.super(VooDoll).__init__()

    def my_constructor(self,
        SCENEGRAPH = None,
        NAVIGATION_NODE = None,
        POINTERS = None,
        HEAD_NODE = None,
        ):
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

        self.pointer_tracking_sensors = []
        self.pointer_device_sensors = []
        self.pointer_nodes = []
        self.mf_buttons = avango.MFBool()

        for pointer in POINTERS:
            pointer_tracking_sensor = avango.daemon.nodes.DeviceSensor(DeviceService=avango.daemon.DeviceService())
            pointer_tracking_sensor.Station.value = pointer[0]
            pointer_tracking_sensor.TransmitterOffset.value = pointer[1]

            self.pointer_tracking_sensors.append(pointer_tracking_sensor)

            pointer_device_sensor = avango.daemon.nodes.DeviceSensor(DeviceService=avango.daemon.DeviceService())
            pointer_device_sensor.Station.value = pointer[2]

            self.pointer_device_sensors.append(pointer_device_sensor)

            #sf_button = avango.SFBool()
            #sf_button.connect_from(pointer_device_sensor.Button0)

            #self.mf_buttons.value.append(sf_button)

            pointer_node = avango.gua.nodes.TransformNode(Name="pointer_node_" + str(len(self.pointer_tracking_sensors)))
            pointer_node.Transform.connect_from(pointer_tracking_sensor.Matrix)
            pointer_node.Tags.value = ["invisible"]

            self.pointer_nodes.append(pointer_node)
            NAVIGATION_NODE.Children.value.append(pointer_node)

        _loader = avango.gua.nodes.TriMeshLoader()

        self.ray_geometry = _loader.create_geometry_from_file("ray_geometry", "data/objects/cylinder.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.ray_geometry.Transform.value = \
            avango.gua.make_trans_mat(0.0,0.0,self.ray_length * -0.5) * \
            avango.gua.make_rot_mat(-90.0,1,0,0) * \
            avango.gua.make_scale_mat(self.ray_thickness, self.ray_length, self.ray_thickness)

        self.ray_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,0.0,1.0))


        self.intersection_geometry = _loader.create_geometry_from_file("intersection_geometry", "data/objects/sphere.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.intersection_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,0.0,1.0))

        self.intersection_geometry.Tags.value = ["invisible"]

        SCENEGRAPH.Root.value.Children.value.append(self.intersection_geometry)

        ### set initial states ###
        self.enable(False)


    def clone(self, object):
        return object

    def getDiameter(self, object):
        return 1.0

    def getRelative(self, object, target):
        return avango.gua.make_trans_mat(0, 0, 0)

    ### functions ###
    def enable(self, BOOL): # extend respective base-class function
        self.enable_flag = BOOL

        if BOOL:
            pass  # set tool visible
        else:
            pass  # set tool invisible

    ### callback functions ###
    def evaluate(self): # implement respective base-class function
        if self.enable_flag == False:
            return
