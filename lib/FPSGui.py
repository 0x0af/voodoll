#!/usr/bin/python

## @file
# Contains class FPSGui.

### import avango-guacamole libraries
import avango
import avango.gua
import avango.gua.gui

### import python libraries
import time


class FPSGui:

    ## constructor
    def __init__( self
                , PARENT_NODE = None
                , WINDOW = None
                , VIEWER = None
                ):

        ### guard ###
        if PARENT_NODE is None or WINDOW is None or VIEWER is None: # guard
            print("ERROR: parameters missing")
            return


        ### external references ###
        self.WINDOW = WINDOW
        self.VIEWER = VIEWER


        ### parameters ###
        self.size = avango.gua.Vec2(512, 64) # in pixel


        ### variables ###

        self.time_sav = time.time()


        ### resources ###
        
        self.gui = avango.gua.gui.nodes.GuiResourceNode(
            TextureName = "fps_gui",
            URL = "asset://gua/data/html/fps_chart.html",
            Size = self.size
            )
        
        self.quad = avango.gua.nodes.TexturedScreenSpaceQuadNode()
        self.quad.Name.value = "fps_quad"
        self.quad.Texture.value = "fps_gui"
        self.quad.Width.value = int(self.size.x)
        self.quad.Height.value = int(self.size.y)        
        self.quad.Anchor.value = avango.gua.Vec2(1.0, -1.0)
        PARENT_NODE.Children.value.append(self.quad)


        ### trigger callbacks ###

        ## @var frame_trigger
        # Triggers framewise evaluation of respective callback method
        self.frame_trigger = avango.script.nodes.Update(Callback = self.frame_callback, Active = True)


    ### callback functions ###
    
    def frame_callback(self):
        if (time.time() - self.time_sav) > 0.1: # plot FPS every 0.1 sec
            _application_fps_string = "{:5.2f}".format(self.VIEWER.ApplicationFPS.value)
            _rendering_fps_string = "{:5.2f}".format(self.WINDOW.RenderingFPS.value)
            
            self.gui.call_javascript("add_value_pair", [_rendering_fps_string, _application_fps_string])

            _max_fps = self.VIEWER.DesiredFPS.value
            self.gui.call_javascript("set_max_fps", [str(_max_fps)])
        
            self.time_sav = time.time()
           
            #print(_application_fps_string, _rendering_fps_string, _max_fps)


            
