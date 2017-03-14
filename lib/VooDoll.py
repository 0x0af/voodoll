import avango
import avango.gua
import avango.script
from avango.script import field_has_changed
import avango.daemon
from gi.overrides.keysyms import R
from enum import Enum
import time

class VooDollState(Enum):
    DOLL_SELECTION = 1
    NEEDLE_SELECTION = 2
    MANIPULATION = 3


class VooDollPointer(Enum):
    POINTER_NONE = 0
    POINTER_1 = 1
    POINTER_2 = 2


class VooDoll(avango.script.Script):
    sf_button_1 = avango.SFBool()
    sf_button_2 = avango.SFBool()
    sf_doll_ref_coord_mat = avango.gua.SFMatrix4()
    sf_pointer_mat_1 = avango.gua.SFMatrix4()
    sf_pointer_mat_2 = avango.gua.SFMatrix4()

    @staticmethod
    def get_largest_expansion(node):
        """
        Approximates the maximum expansion of the given Geometry.

        @:param node: avango.gua.nodes.GeometryNode
        @:return:
        @:rtype: Float
        """
        expansions = node.BoundingBox.value.Max.value - node.BoundingBox.value.Min.value
        return max(expansions.x, expansions.y, expansions.z)

    @staticmethod
    def get_relative(node, reference):
        n_mat = node.WorldTransform.value
        r_mat = reference.WorldTransform.value
        return avango.gua.make_inverse_mat(n_mat) * r_mat

    @staticmethod
    def extract_file_path(node):
        _str = node.Geometry.value

        return _str.split("|")[1]

    @staticmethod
    def filter_pick_result(pick_result_candidate):
        if len(pick_result_candidate.value) > 0:
            return pick_result_candidate.value[0]
        else:
            return None

    def __init__(self):
        self.super(VooDoll).__init__()

        self.pointer_node_1 = None
        self.pointer_node_2 = None

    def create_ray_geometry(self):
        _ray = self._loader.create_geometry_from_file("ray_geometry", "data/objects/cylinder.obj",
                                                      avango.gua.LoaderFlags.DEFAULTS)
        _ray.Transform.value = \
            avango.gua.make_trans_mat(0.0, 0.0, self.ray_length * -0.5) * \
            avango.gua.make_rot_mat(-90.0, 1, 0, 0) * \
            avango.gua.make_scale_mat(self.ray_thickness, self.ray_length, self.ray_thickness)
        _ray.Material.value.set_uniform("Color", avango.gua.Vec4(1.0, 0.0, 0.0, 1.0))

        return _ray

    def create_intersection_geometry(self):
        _inter = avango.gua.nodes.TransformNode(Name="intersection")

        _point = self._loader.create_geometry_from_file("intersection_geometry", "data/objects/sphere.obj",
                                                        avango.gua.LoaderFlags.DEFAULTS)
        _point.Material.value.set_uniform("Color", avango.gua.Vec4(1.0, 0.0, 0.0, 1.0))
        _point.Transform.value = avango.gua.make_scale_mat(self.intersection_point_size)

        _inter.Children.value.append(_point)

        return _inter

    def create_axis_node(self) -> object:
        """
        :rtype : avango.gua.nodes.TransformNode
        :return:
        """
        _trans = avango.gua.nodes.TransformNode(Name="axis")
        _x = self._loader.create_geometry_from_file("x_axis", "data/objects/cylinder.obj",
                                                    avango.gua.LoaderFlags.DEFAULTS)
        _y = self._loader.create_geometry_from_file("y_axis", "data/objects/cylinder.obj",
                                                    avango.gua.LoaderFlags.DEFAULTS)
        _z = self._loader.create_geometry_from_file("z_axis", "data/objects/cylinder.obj",
                                                    avango.gua.LoaderFlags.DEFAULTS)
        _x.Material.value.set_uniform("Color", avango.gua.Vec4(1.0, 0.0, 0.0, 1.0))
        _y.Material.value.set_uniform("Color", avango.gua.Vec4(0.0, 1.0, 0.0, 1.0))
        _z.Material.value.set_uniform("Color", avango.gua.Vec4(1.0, 0.0, 1.0, 1.0))
        _y.Transform.value = \
            avango.gua.make_scale_mat(self.ray_thickness, 5.0, self.ray_thickness)
        _x.Transform.value = \
            avango.gua.make_rot_mat(-90.0, 0, 0, 1) * \
            avango.gua.make_scale_mat(self.ray_thickness, 5.0, self.ray_thickness)
        _z.Transform.value = \
            avango.gua.make_rot_mat(-90.0, 1, 0, 0) * \
            avango.gua.make_scale_mat(self.ray_thickness, 5.0, self.ray_thickness)
        _trans.Children.value.append(_x)
        _trans.Children.value.append(_y)
        _trans.Children.value.append(_z)
        return _trans

    def my_constructor(self,
                       SCENEGRAPH=None,
                       NAVIGATION_NODE=None,
                       TRACKING_TRANSMITTER_OFFSET=avango.gua.make_identity_mat(),
                       POINTER_TRACKING_STATION_1=None,
                       POINTER_DEVICE_STATION_1=None,
                       POINTER_TRACKING_STATION_2=None,
                       POINTER_DEVICE_STATION_2=None,
                       HEAD_NODE=None):
        """
        :param SCENEGRAPH:
        :type SCENEGRAPH: avango.gua.nodes.SceneGraph
        :param NAVIGATION_NODE:
        :type NAVIGATION_NODE: avango.gua.nodes.TransformNode
        :param POINTER_TRACKING_STATION_1:
        :type POINTER_TRACKING_STATION_1: String
        :param POINTER_DEVICE_STATION_1:
        :type POINTER_DEVICE_STATION_1: String
        :param POINTER_TRACKING_STATION_2:
        :type POINTER_TRACKING_STATION_2: String
        :param POINTER_DEVICE_STATION_2:
        :type POINTER_DEVICE_STATION_2: String
        :param HEAD_NODE: avango.gua.nodes.TransformNode
        """

        self.HEAD_NODE = HEAD_NODE
        self.NAVIGATION_NODE = NAVIGATION_NODE
        self.SCENEGRAPH = SCENEGRAPH

        self.ray_length = 5.0
        self.ray_thickness = 0.0075
        self.intersection_point_size = 0.01

        self.enable_flag = False

        self.pick_result_1 = None
        self.pick_result_2 = None

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

        self.doll_pointer = VooDollPointer.POINTER_NONE

        self.doll_scale_start_dist = None
        self.doll_scale_factor = None
        self.needle_scale_start_dist = None
        self.needle_scale_factor = None

        # region Pointers
        self.pointer_tracking_sensor_1 = avango.daemon.nodes.DeviceSensor(DeviceService=avango.daemon.DeviceService())
        self.pointer_tracking_sensor_1.Station.value = POINTER_TRACKING_STATION_1
        self.pointer_tracking_sensor_1.TransmitterOffset.value = TRACKING_TRANSMITTER_OFFSET

        self.pointer_device_sensor_1 = avango.daemon.nodes.DeviceSensor(DeviceService=avango.daemon.DeviceService())
        self.pointer_device_sensor_1.Station.value = POINTER_DEVICE_STATION_1

        self.sf_button_1.connect_from(self.pointer_device_sensor_1.Button0)

        self.pointer_node_1 = avango.gua.nodes.TransformNode(Name="pointer_node_1")
        self.sf_pointer_mat_1.connect_from(self.pointer_tracking_sensor_1.Matrix)
        self.pointer_node_1.Tags.value = ["invisible"]

        NAVIGATION_NODE.Children.value.append(self.pointer_node_1)

        self.pointer_tracking_sensor_2 = avango.daemon.nodes.DeviceSensor(DeviceService=avango.daemon.DeviceService())
        self.pointer_tracking_sensor_2.Station.value = POINTER_TRACKING_STATION_2
        self.pointer_tracking_sensor_2.TransmitterOffset.value = TRACKING_TRANSMITTER_OFFSET

        self.pointer_device_sensor_2 = avango.daemon.nodes.DeviceSensor(DeviceService=avango.daemon.DeviceService())
        self.pointer_device_sensor_2.Station.value = POINTER_DEVICE_STATION_2

        self.sf_button_2.connect_from(self.pointer_device_sensor_2.Button0)

        self.pointer_node_2 = avango.gua.nodes.TransformNode(Name="pointer_node_2")
        self.sf_pointer_mat_2.connect_from(self.pointer_tracking_sensor_2.Matrix)
        self.pointer_node_2.Tags.value = ["invisible"]

        self.object_slot_1 = avango.gua.nodes.TransformNode(Name="pointer_slot_1")
        self.object_slot_2 = avango.gua.nodes.TransformNode(Name="pointer_slot_2")

        self.object_slot_1.Transform.value = avango.gua.make_trans_mat(0.0, 0.0, -0.1)
        self.object_slot_2.Transform.value = avango.gua.make_trans_mat(0.0, 0.0, -0.1)

        self.pointer_node_1.Children.value.append(self.object_slot_1)
        self.pointer_node_2.Children.value.append(self.object_slot_2)

        NAVIGATION_NODE.Children.value.append(self.pointer_node_2)

        # endregion

        self._loader = avango.gua.nodes.TriMeshLoader()

        self.start_time_bp_1 = None
        self.start_time_bp_2 = None

        self.SCALING_TIME_THRESHHOLD = 1.0

        # region Rays

        self.ray_geometry_1 = self.create_ray_geometry()
        self.ray_geometry_2 = self.create_ray_geometry()
        self.pointer_node_1.Children.value.append(self.ray_geometry_1)
        self.pointer_node_2.Children.value.append(self.ray_geometry_2)

        # endregion

        self.doll_ref_axis = self.create_axis_node()
        self.doll_axis = self.create_axis_node()
        self.doll_ref_axis.Tags.value = ["invisible"]
        self.doll_axis.Tags.value = ["invisible"]

        SCENEGRAPH.Root.value.Children.value.append(self.doll_ref_axis)

        # region Intersections

        self.intersection_geometry_1 = self.create_intersection_geometry()
        self.intersection_geometry_2 = self.create_intersection_geometry()
        self.intersection_geometry_1.Tags.value = ["invisible"]
        self.intersection_geometry_2.Tags.value = ["invisible"]
        SCENEGRAPH.Root.value.Children.value.append(self.intersection_geometry_1)
        SCENEGRAPH.Root.value.Children.value.append(self.intersection_geometry_2)

        # endregion

        ### set initial states ###
        self.enable(False)

    def get_normalised_rot(self, mat):
        return avango.gua.make_inverse_mat(avango.gua.make_trans_mat(mat.get_translate())) * mat


    def clone(self, node):
        """
        Clones the given Node without its children.

        @:param node:
        @:type node: avango.gua.nodes.GeometryNode|avango.gua.nodes.TransformNode
        @:return:
        @:rtype: avango.gua.nodes.GeometryNode|avango.gua.nodes.TransformNode
        """
        if node.has_field("Geometry"):
            _obj = self._loader.create_geometry_from_file(node.Name.value, VooDoll.extract_file_path(node),
                                                        avango.gua.LoaderFlags.DEFAULTS)
            _obj.Material.connect_from(node.Material)
        else:
            _obj = avango.gua.nodes.TransformNode(Name=node.Name.value)

        _obj.Transform.value = node.Transform.value

        return _obj

    def calc_ray_intersection(self, pointer_node):
        self.ray.Origin.value = pointer_node.WorldTransform.value.get_translate()
        _vec = avango.gua.make_rot_mat(pointer_node.WorldTransform.value.get_rotate()) * avango.gua.Vec3(0.0, 0.0, -1.0)
        _vec = avango.gua.Vec3(_vec.x, _vec.y, _vec.z)
        self.ray.Direction.value = _vec * self.ray_length
        return self.SCENEGRAPH.ray_test(self.ray, self.pick_options, self.white_list, self.black_list)

    def calc_pick_result(self):
        if self.state == VooDollState.DOLL_SELECTION:
            self.pick_result_1 = VooDoll.filter_pick_result(self.calc_ray_intersection(self.pointer_node_1))
            self.pick_result_2 = VooDoll.filter_pick_result(self.calc_ray_intersection(self.pointer_node_2))
        elif self.state == VooDollState.NEEDLE_SELECTION:
            if self.doll_pointer == VooDollPointer.POINTER_2:
                self.pick_result_1 = VooDoll.filter_pick_result(self.calc_ray_intersection(self.pointer_node_1))
                self.pick_result_2 = None
            elif self.doll_pointer == VooDollPointer.POINTER_1:
                self.pick_result_1 = None
                self.pick_result_2 = VooDoll.filter_pick_result(self.calc_ray_intersection(self.pointer_node_2))
            else:
                self.pick_result_1 = None
                self.pick_result_2 = None

    def enable(self, BOOL):  # extend respective base-class function
        self.enable_flag = BOOL

        if BOOL:
            self.pointer_node_1.Tags.value = []
            self.pointer_node_2.Tags.value = []
            self.ray_geometry_1.Tags.value = []
            self.ray_geometry_2.Tags.value = []
        else:
            self.pointer_node_1.Tags.value = ["invisible"]
            self.pointer_node_2.Tags.value = ["invisible"]

            self.reset()

    def set_intersection_point(self, pointer):
        _pick_result = None
        _intersection_geometry = None

        if pointer == VooDollPointer.POINTER_1:
            _pick_result = self.pick_result_1
            _intersection_geometry = self.intersection_geometry_1
        elif pointer == VooDollPointer.POINTER_2:
            _pick_result = self.pick_result_2
            _intersection_geometry = self.intersection_geometry_2

        if _intersection_geometry is not None:
            if _pick_result is None:
                _intersection_geometry.Tags.value = ["invisible"]
            else:
                _intersection_geometry.Transform.value = avango.gua.make_trans_mat(_pick_result.WorldPosition.value)
                _intersection_geometry.Tags.value = []

    def get_doll_button(self):
        if self.doll_pointer == VooDollPointer.POINTER_1:
            return self.sf_button_1
        elif self.doll_pointer == VooDollPointer.POINTER_2:
            return self.sf_button_2
        else:
            return None

    def get_needle_button(self):
        if self.doll_pointer == VooDollPointer.POINTER_1:
            return self.sf_button_2
        elif self.doll_pointer == VooDollPointer.POINTER_2:
            return self.sf_button_1
        else:
            return None

    def get_doll_button_timer(self):
        if self.doll_pointer == VooDollPointer.POINTER_1:
            return self.start_time_bp_1
        elif self.doll_pointer == VooDollPointer.POINTER_2:
            return self.start_time_bp_2
        else:
            return None

    def get_needle_button_timer(self):
        if self.doll_pointer == VooDollPointer.POINTER_1:
            return self.start_time_bp_2
        elif self.doll_pointer == VooDollPointer.POINTER_2:
            return self.start_time_bp_1
        else:
            return None

    def get_doll_pointer_node(self):
        if self.doll_pointer == VooDollPointer.POINTER_1:
            return self.pointer_node_1
        elif self.doll_pointer == VooDollPointer.POINTER_2:
            return self.pointer_node_2
        else:
            return None

    def get_needle_pointer_node(self):
        if self.doll_pointer == VooDollPointer.POINTER_1:
            return self.pointer_node_2
        elif self.doll_pointer == VooDollPointer.POINTER_2:
            return self.pointer_node_1
        else:
            return None

    def has_timed_out(self, timer, timeout):
        if timer == None:
            return False

        return (time.time() - timer) > timeout

    def evaluate(self):  # implement respective base-class function
        if self.enable_flag == False:
            return

        self.calc_pick_result()

        if self.state == VooDollState.DOLL_SELECTION:
            self.set_intersection_point(VooDollPointer.POINTER_1)
            self.set_intersection_point(VooDollPointer.POINTER_2)
        elif self.state == VooDollState.NEEDLE_SELECTION:
            _doll_button = self.get_doll_button()

            if False and _doll_button is not None and _doll_button.value and self.doll_scale_start_dist is not None and \
                    self.has_timed_out(self.get_doll_button_timer(), self.SCALING_TIME_THRESHHOLD):
                _scale = self.get_distance_to_head(self.get_doll_pointer_node().WorldTransform.value.get_translate())
                self.doll.Transform.value = avango.gua.make_rot_mat(self.doll.Transform.value.get_rotate()) * avango.gua.make_scale_mat(self.doll_scale_factor * _scale)

            if self.doll_pointer == VooDollPointer.POINTER_1:
                self.set_intersection_point(VooDollPointer.POINTER_2)
            else:
                self.set_intersection_point(VooDollPointer.POINTER_1)
        elif self.state == VooDollState.MANIPULATION:
            _rel_mat = avango.gua.make_inverse_mat(self.doll.WorldTransform.value) * self.needle.WorldTransform.value

            _parent_mat = self.needle_ref.Parent.value.WorldTransform.value
            self.needle_ref.Transform.value = avango.gua.make_inverse_mat(
                _parent_mat) * self.doll_ref.WorldTransform.value * _rel_mat

            _needle_button = self.get_needle_button()

            if _needle_button is not None and _needle_button.value and self.needle_scale_start_dist is not None and \
                    self.has_timed_out(self.get_needle_button_timer(), self.SCALING_TIME_THRESHHOLD):
                _scale = self.get_distance_to_head(self.get_needle_pointer_node().WorldTransform.value.get_translate())
                self.needle.Transform.value = avango.gua.make_rot_mat(self.needle.Transform.value.get_rotate()) * avango.gua.make_scale_mat(_scale * self.needle_scale_factor)

    def get_distance_to_head(self, vec):
        _head = self.HEAD_NODE.WorldTransform.value.get_translate()

        return _head.distance_to(vec)

    def click_handler_button(self, button, pick_result, pointer_node, object_slot, ray, intersection_geometry):
        if button.value:
            if pick_result is not None:
                _obj = None

                if self.state == VooDollState.DOLL_SELECTION:
                    self.doll_ref = pick_result.Object.value
                    #self.doll_ref.value.Material.value.set_uniform("Opacity", 0.2)
                    self.sf_doll_ref_coord_mat.connect_from(self.doll_ref.WorldTransform)
                    self.doll_ref_axis.Tags.value = []
                    self.doll_axis.Tags.value = []
                    _obj = self.doll = self.clone(self.doll_ref)

                    _trans_mat = avango.gua.make_inverse_mat(_obj.WorldTransform.value) * object_slot.WorldTransform.value
                    _trans_mat = avango.gua.make_trans_mat(_trans_mat.get_translate())

                    _obj.Transform.value = avango.gua.make_inverse_mat(object_slot.WorldTransform.value) * _obj.WorldTransform.value * _trans_mat * avango.gua.make_scale_mat(0.3)
                    self.doll_axis.Transform.value = self.get_normalised_rot(_obj.Transform.value)

                    object_slot.Children.value.append(self.doll_axis)
                    self.doll_scale_start_dist = self.get_distance_to_head(
                        pointer_node.WorldTransform.value.get_translate())
                    _factor = (1 / self.get_largest_expansion(self.doll)) * 0.1
                    self.doll_scale_factor = avango.gua.Vec3(_factor, _factor, _factor)
                    self.state = VooDollState.NEEDLE_SELECTION
                elif self.state == VooDollState.NEEDLE_SELECTION:
                    self.needle_ref = pick_result.Object.value
                    _obj = self.needle = self.clone(self.needle_ref)
                    self.needle_scale_start_dist = self.get_distance_to_head(
                        pointer_node.WorldTransform.value.get_translate())
                    _mat = avango.gua.make_inverse_mat(self.doll_ref.WorldTransform.value) * self.needle_ref.WorldTransform.value
                    self.needle_scale_factor = (1 / self.get_largest_expansion(self.needle)) * _mat.get_scale()
                    self.state = VooDollState.MANIPULATION

                ray.Tags.value = ["invisible"]
                intersection_geometry.Tags.value = ["invisible"]
                object_slot.Children.value.append(_obj)
        else:
            if self.state == VooDollState.NEEDLE_SELECTION:
                self.doll_scale_start_dist = None
                self.doll_scale_factor = None
            elif self.state == VooDollState.MANIPULATION:
                self.needle_scale_start_dist = None
                self.needle_scale_factor = None

    def reset(self):
        if self.doll_axis.Parent.value is not None:
            self.doll_axis.Parent.value.Children.value.remove(self.doll_axis)

        self.doll_pointer = VooDollPointer.POINTER_NONE

        if self.doll is not None and self.doll.Parent.value is not None:
            self.doll.Parent.value.Children.value.remove(self.doll)

        if self.needle is not None and self.needle.Parent.value is not None:
            self.needle.Parent.value.Children.value.remove(self.needle)

        self.doll = None
        self.needle = None
        self.doll_ref = None
        self.needle_ref = None

        self.ray_geometry_1.Tags.value = []
        self.ray_geometry_2.Tags.value = []
        self.doll_ref_axis.Tags.value = ["invisible"]
        self.doll_axis.Tags.value = ["invisible"]


        self.start_time_bp_1 = None
        self.start_time_bp_2 = None

        self.doll_scale_start_dist = None
        self.needle_scale_start_dist = None
        self.doll_scale_factor = None
        self.needle_scale_factor = None

        self.state = VooDollState.DOLL_SELECTION

    def click_handler(self, pointer):
        if self.state == VooDollState.DOLL_SELECTION:
            if pointer == VooDollPointer.POINTER_1:
                self.click_handler_button(self.sf_button_1, self.pick_result_1, self.pointer_node_1, self.object_slot_1,
                                          self.ray_geometry_1, self.intersection_geometry_1)
            elif pointer == VooDollPointer.POINTER_2:
                self.click_handler_button(self.sf_button_2, self.pick_result_2, self.pointer_node_2, self.object_slot_2,
                                          self.ray_geometry_2, self.intersection_geometry_2)

            self.doll_pointer = pointer
        elif self.state == VooDollState.NEEDLE_SELECTION:
            if pointer == self.doll_pointer and self.get_doll_button().value:
                self.reset()
            else:
                if pointer == VooDollPointer.POINTER_1:
                    self.click_handler_button(self.sf_button_1, self.pick_result_1, self.pointer_node_1,
                                              self.object_slot_1,
                                              self.ray_geometry_1, self.intersection_geometry_1)
                elif pointer == VooDollPointer.POINTER_2:
                    self.click_handler_button(self.sf_button_2, self.pick_result_2, self.pointer_node_2,
                                              self.object_slot_2,
                                              self.ray_geometry_2, self.intersection_geometry_2)
        elif self.state == VooDollState.MANIPULATION:
            if pointer != self.doll_pointer and self.get_needle_button().value:
                self.state = VooDollState.NEEDLE_SELECTION

                if self.doll_pointer == VooDollPointer.POINTER_1:
                    self.object_slot_2.Children.value.remove(self.needle)
                    self.ray_geometry_2.Tags.value = []
                elif self.doll_pointer == VooDollPointer.POINTER_2:
                    self.object_slot_1.Children.value.remove(self.needle)
                    self.ray_geometry_1.Tags.value = []

                self.needle = None
                self.needle_ref = None
            elif pointer == self.doll_pointer and self.get_doll_button().value:
                self.reset()

    @field_has_changed(sf_button_1)
    def sf_button_1_changed(self):
        if self.pointer_node_1 is not None:
            if self.sf_button_1.value:
                self.start_time_bp_1 = time.time()
            else:
                self.start_time_bp_1 = None

            self.click_handler(VooDollPointer.POINTER_1)

    @field_has_changed(sf_button_2)
    def sf_button_2_changed(self):
        if self.pointer_node_2 is not None:
            if self.sf_button_2.value:
                self.start_time_bp_2 = time.time()
            else:
                self.start_time_bp_2 = None

            self.click_handler(VooDollPointer.POINTER_2)

    @field_has_changed(sf_doll_ref_coord_mat)
    def sf_doll_ref_coord_mat_changed(self):
        if self.doll_ref_axis is not None:
            _mat = self.sf_doll_ref_coord_mat.value

            self.doll_ref_axis.Transform.value = \
                avango.gua.make_inverse_mat(self.doll_ref_axis.Parent.value.WorldTransform.value) * \
                avango.gua.make_trans_mat(_mat.get_translate()) * \
                self.get_normalised_rot(_mat)

    @field_has_changed(sf_pointer_mat_1)
    def sf_pointer_mat_1_changed(self):
        if self.pointer_node_1 is not None:
            _mat = self.sf_pointer_mat_1.value

            self.pointer_node_1.Transform.value = avango.gua.make_trans_mat(_mat.get_translate()) * avango.gua.make_rot_mat(_mat.get_rotate())

    @field_has_changed(sf_pointer_mat_2)
    def sf_pointer_mat_2_changed(self):
        if self.pointer_node_2 is not None:
            _mat = self.sf_pointer_mat_2.value

            self.pointer_node_2.Transform.value = avango.gua.make_trans_mat(_mat.get_translate()) * self.get_normalised_rot(_mat)
