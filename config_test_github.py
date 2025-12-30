from pathlib import Path

print('Loading config_test.py...')

def start(IRIS):
    IRIS.add_module("socket_module")

#   IRIS.add_module("imagedisplay_module.py",image_id = "test_string")

    IRIS.add_module("valvecluster_module_can", x_pos = 0, y_pos = 1, width = 2, mask = [1, 1, 1, 1, 1, 0] )
    
    #IRIS.add_module("cell_stopping_jose", x_pos = 4, y_pos = 0, width = 4, mask = [1, 1, 1, 1, 1, 0] )

    IRIS.add_module("cell_stopping_jose_proto_2", x_pos = 4, y_pos = 0, width = 4, mask = [1, 1, 1, 1, 1, 0] )
      
    
    IRIS.add_module("preg_v2_module", x_pos = 1, y_pos = 2, preg_id = 0x31)
    IRIS.add_module("imagedisplay_module_Caro", x_pos = 2, y_pos = 1, camera_id = "img", width = 4, height = 2)

    IRIS.add_module("computer_monitor", x_pos = 4, y_pos = 6)

    #IRIS.add_module("example_workflow_torture", preg_id = 0x34, x_pos = 2, y_pos = 0)
 

