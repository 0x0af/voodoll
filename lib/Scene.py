#!/usr/bin/python

### import guacamole libraries
import avango
import avango.gua
import avango.script
from avango.script import field_has_changed

### import python libraries



class SceneScript(avango.script.Script):

    ## input fields
    sf_reset_button = avango.SFBool()

    ## Default constructor.
    def __init__(self):
        self.super(SceneScript).__init__()

        ### external references ###
        self.CLASS = None # is set later
        

        ### resources ###
        self.keyboard_device_sensor = avango.daemon.nodes.DeviceSensor(DeviceService = avango.daemon.DeviceService())
        self.keyboard_device_sensor.Station.value = "gua-device-keyboard0"

        self.sf_reset_button.connect_from(self.keyboard_device_sensor.Button14) # spacebar key


    def my_constructor(self, CLASS):
        self.CLASS = CLASS


    ### callbacks ###  
    @field_has_changed(sf_reset_button)
    def sf_reset_button_changed(self):
        if self.sf_reset_button.value == True and self.CLASS is not None: # button pressed
            self.CLASS.reset()
            


class Scene:

    ## constructor
    def __init__(self,
        PARENT_NODE = None,
        ):


        ### external reference ###
        self.PARENT_NODE = PARENT_NODE

        ### resources ###                
        self.script = SceneScript()
        self.script.my_constructor(self)


        ## init scene light
        self.scene_light = avango.gua.nodes.LightNode(Name = "scene_light")
        self.scene_light.Type.value = avango.gua.LightType.SPOT
        self.scene_light.Color.value = avango.gua.Color(1.0,1.0,0.8)
        self.scene_light.Brightness.value = 20.0
        self.scene_light.Falloff.value = 0.1 # exponent
        self.scene_light.EnableShadows.value = True
        self.scene_light.ShadowMapSize.value = 2048
        self.scene_light.ShadowOffset.value = 0.001
        self.scene_light.ShadowNearClippingInSunDirection.value = 0.1 * (1.0/4.0)
        self.scene_light.ShadowMaxDistance.value = 10.0 # maximal distance, the shadow is visible
        self.scene_light.ShadowNearClippingInSunDirection.value = 0.05
        self.scene_light.Transform.value = avango.gua.make_trans_mat(0.0,1.2,0.5) * \
            avango.gua.make_rot_mat(80.0,-1,0,0) * \
            avango.gua.make_scale_mat(3.0)
        PARENT_NODE.Children.value.append(self.scene_light)

        ## init scene geometries
        _loader = avango.gua.nodes.TriMeshLoader() # get trimesh loader to load external meshes

        # ground
        self.ground = _loader.create_geometry_from_file("ground", "data/objects/cube.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.LOAD_MATERIALS)
        self.ground.Transform.value = avango.gua.make_trans_mat(0.0,-0.17,0.0) * \
            avango.gua.make_scale_mat(1.0,0.005,1.0)
        self.ground.Material.value.set_uniform("Color", avango.gua.Vec4(0.7,0.7,1.0,1.0))
        self.ground.Material.value.set_uniform("Emissivity", 0.5)
        self.ground.Material.value.set_uniform("Metalness", 0.1)
        self.ground.add_and_init_field(avango.gua.SFMatrix4(), "HomeMatrix", self.ground.Transform.value)
        PARENT_NODE.Children.value.append(self.ground)


        # table
        self.table = _loader.create_geometry_from_file("table", "/opt/3d_models/Jacobs_Models/table_ikea/table_ikea.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.LOAD_MATERIALS | avango.gua.LoaderFlags.MAKE_PICKABLE)
        self.table.Transform.value = avango.gua.make_trans_mat(0.0, -0.17, 0.0) * \
            avango.gua.make_rot_mat(90.0,-1,0,0) * \
            avango.gua.make_scale_mat(0.0003)
        self.table.add_and_init_field(avango.gua.SFMatrix4(), "HomeMatrix", self.table.Transform.value)
        PARENT_NODE.Children.value.append(self.table)
        

        # notebook
        self.notebook = _loader.create_geometry_from_file("notebook", "/opt/3d_models/Jacobs_Models/notebook/notebook.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.LOAD_MATERIALS | avango.gua.LoaderFlags.MAKE_PICKABLE)
        self.notebook.Transform.value = avango.gua.make_trans_mat(0.1, 0.055, 0.0) * \
            avango.gua.make_rot_mat(90.0,-1,0,0) * \
            avango.gua.make_rot_mat(10.0,0,0,-1) * \
            avango.gua.make_scale_mat(0.011)
        self.notebook.add_and_init_field(avango.gua.SFMatrix4(), "HomeMatrix", self.notebook.Transform.value)
        PARENT_NODE.Children.value.append(self.notebook)
        

        # tablelamp
        self.tablelamp = _loader.create_geometry_from_file("tablelamp", "/opt/3d_models/Jacobs_Models/tablelamp/tablelamp.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.LOAD_MATERIALS | avango.gua.LoaderFlags.MAKE_PICKABLE)
        self.tablelamp.Transform.value = avango.gua.make_trans_mat(-0.08, 0.215, -0.0) * \
            avango.gua.make_rot_mat(90.0,-1,0,0) * \
            avango.gua.make_rot_mat(135.0,0,0,-1) * \
            avango.gua.make_scale_mat(0.00022)
        self.tablelamp.add_and_init_field(avango.gua.SFMatrix4(), "HomeMatrix", self.tablelamp.Transform.value)
        PARENT_NODE.Children.value.append(self.tablelamp)
                                     


        # telephone
        self.telephone = _loader.create_geometry_from_file("telephone", "/opt/3d_models/Jacobs_Models/telephone/telephone.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.LOAD_MATERIALS | avango.gua.LoaderFlags.MAKE_PICKABLE)
        self.telephone.Transform.value = avango.gua.make_trans_mat(-0.05, 0.065, -0.03) * \
            avango.gua.make_rot_mat(90.0,-1,0,0) * \
            avango.gua.make_rot_mat(65.0,0,0,-1) * \
            avango.gua.make_scale_mat(0.000012)
        self.telephone.add_and_init_field(avango.gua.SFMatrix4(), "HomeMatrix", self.telephone.Transform.value)
        PARENT_NODE.Children.value.append(self.telephone)
        
   
        # penholder
        self.penholder = _loader.create_geometry_from_file("penholder", "/opt/3d_models/Jacobs_Models/penholder/penholder.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.LOAD_MATERIALS | avango.gua.LoaderFlags.MAKE_PICKABLE)
        self.penholder.Transform.value = avango.gua.make_trans_mat(-0.08, 0.2, -0.13) * \
            avango.gua.make_rot_mat(90.0,-1,0,0) * \
            avango.gua.make_scale_mat(0.0002)
        self.penholder.add_and_init_field(avango.gua.SFMatrix4(), "HomeMatrix", self.penholder.Transform.value)
        PARENT_NODE.Children.value.append(self.penholder)
        

        # calculator
        self.calculator = _loader.create_geometry_from_file("calculator", "/opt/3d_models/Jacobs_Models/calculator/calculator.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.LOAD_MATERIALS | avango.gua.LoaderFlags.MAKE_PICKABLE)
        self.calculator.Transform.value = avango.gua.make_trans_mat(-0.16, 0.055, 0.03) * \
            avango.gua.make_rot_mat(90.0,-1,0,0) * \
            avango.gua.make_rot_mat(13.0,0,0,1) * \
            avango.gua.make_scale_mat(0.01)
        self.calculator.add_and_init_field(avango.gua.SFMatrix4(), "HomeMatrix", self.calculator.Transform.value)
        PARENT_NODE.Children.value.append(self.calculator)


    ### functions ###
    def reset(self):
        print("reset")
        for _node in self.PARENT_NODE.Children.value:
            if _node.has_field("HomeMatrix") == True:
                _node.Transform.value = _node.HomeMatrix.value
                
