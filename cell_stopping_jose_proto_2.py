import signal
from template_module import template_module
from iris_instruction_class import instruction_set
from PyQt5 import QtCore, QtWidgets, QtGui
import ximea.xiapi as xi
import os
import imageio as iio
import threading

from example_instructiontemplate import microscope_handle

from g_config_in_module import config_in_module

print('Loading cell_stopping_jose.py...')

class valve_cluster(template_module):
    def __init__(self, IRIS, name, x_pos, y_pos, width,  mask):
        super().__init__(IRIS, name, x_pos, y_pos, width = width)
        self.instr_manager = instruction_set()
        self.uscope = microscope_handle()
        self.idle_state = mask
        #Added for cell imaging
        self.cell_number_counter = 1

        #self.is_camara_open = False
        #self.is_imaging_complete = False
        self.imaging_loop_started = False
        
        self.single_imaging_active = False
        
        self.multiple_imaging_active = False
        self.counter_for_multiple_imaging = 0
        self.number_of_cells_to_be_collected = 1
        
        self.type_of_image = 'data'

        #experiment input information
        self.experiment_name = '20YYMMDD_EXP'
        self.cell_to_start_with = 1
        print('init valve_cluster')

        #create instance for first connected camera
        self.cam = xi.Camera()

    def setup_module(self):
        self.add_label("Leak pressure for cell stopping:", 4,0)
        self.add_slider("leak pressure", self.set_leakPressure, 0, 1000, 4, 1, 1, 2, "horizontal")
        self.add_lcd("leak pressure lcd", 4, 3)
        self.leakSlider = self.get_element("leak pressure")
        self.leakLcd = self.get_element("leak pressure lcd")

        # experiment_information
        self.add_label("Experiment name:", 5,0)
        self.add_textboxNumeric("exp_name", 5,1)
        self.add_label("Index of FIRST cell to be acquired:", 7,0)
        self.add_textboxNumeric("first_cell_index", 7,1)
        
        # cell stopping
        self.add_button("stop single cell", self._on_valve_btn_click, 9, 0, True)
        self.add_button("open camera", self.open_cam, 11, 1, True)
        self.add_button("close camera", self.close_cam, 11, 2, True)

        #continuous cell stopping

        self.add_button("collect multiple cells", self._on_valve_btn_click_loop, 12, 0, True)
        self.add_label("Number of cells to be collected:",13,0)
        self.add_slider("number of cells", self.set_number_of_cells, 0, 100, 13, 1, 1, 2, "horizontal")
        self.add_lcd("number of cells lcd", 13, 3)
        self.numberSlider = self.get_element("number of cells")
        self.numberLcd = self.get_element("number of cells lcd")

        # cell stopping
        self.add_button("background images", self.background_image, 11, 0, True)

        #self.add_receiveFunction("state", self.socket_signal) #prototype
        #self.add_receiveFunction("process_manager:capture_success_state_channel", self.attach_capture_state) #prototype

        self.add_receiveFunction("state", self.set_busy)
        #reads register 30 to detect cell
        self.add_receiveFunction(30, self.cell_detect_boolean)

    def camera_imaging (self, cell_number_counter):

        print(f'Inside camera_imaging method for cell number: {cell_number_counter}')

        # More code for the camara --> https://www.ximea.com/support/wiki/apis/XiAPI_Python_Manual#Extended_Device_parameters-isexist

        #input parameters
        experiment_name = self.experiment_name
        cell_to_start_with = self.cell_to_start_with
        
        ## cell imaging
        cell_number_to_be_printed = cell_number_counter + int(cell_to_start_with) -1 #start from a different cell
        
        print('Activated def camera_imaging ():...')

        #settings
        self.cam.set_imgdataformat('XI_MONO16')
        print('image_format_was_set_up')

        #cam.set_exposure(50000) in us
        self.cam.set_exposure(20000)
        print('Exposure was set to %i us' %self.cam.get_exposure())

        #create instance of Image to store image data and metadata
        img = xi.Image()

        #start data acquisition
        print('Starting data acquisition...')

        # PREPARE CAN message
        ###
        
        self.instr_manager.start_exposure(self.uscope)
        msg = self.instr_manager.gen_message()
        self.send_toRelai("instr", msg)
        print('sent to Relai to turn off IR')

        ###
        # FINISH CAN message

        self.cam.start_acquisition()

        self.cam.get_image(img)

        #get raw data from camera
        #for Python2.x function returns string
        #for Python3.x function returns bytes
        data_raw = img.get_image_data_raw()

        #transform data to list
        data_list = list(data_raw)
        data_img = img.get_image_data_numpy()
        #data_to_save = 20*data_img
        data_to_save = data_img

        #print image data and metadata
        print('Image number: ' + str(1))
        print('Image width (pixels):  ' + str(img.width))
        print('Image height (pixels): ' + str(img.height))
        print('First 10 pixels: ' + str(data_list[:10]))
        #print('\n')    

        #stop data acquisition
        print('Stopping acquisition...')
        self.cam.stop_acquisition()

        #stop communication
        #print('Camara CLOSED')
        #self.cam.close_device()
               
        if self.type_of_image == 'background':
            cell_number_to_be_printed = cell_number_counter
            print("brackground image", cell_number_to_be_printed )
            filename_of_image = 'C:/Users/jvasquez/Desktop/test_QPI_images/'+ experiment_name +'_'+ str(cell_number_to_be_printed) +'_HOLO_EMPTY'+ '.tif'
            iio.imwrite(filename_of_image, data_to_save)
            self.type_of_image == 'data' #reset variable
                          
        else: # self.type_of_image == 'data'
            #iio.imwrite('/mnt/c/Users/jvasquez/Desktop/test_QPI_images' + 'hek_' + str(1) + '.tiff', data)
            print("data image", cell_number_to_be_printed )
            filename_of_image = 'C:/Users/jvasquez/Desktop/test_QPI_images/'+ experiment_name +'_'+ str(cell_number_to_be_printed) +'_HOLO'+ '.tif'
            
        iio.imwrite(filename_of_image, data_to_save) #save image data

        print('Image saved...', filename_of_image)
        print('Done.')
        print('\n')

    print('Loaded camera_imaging ():...')

    def open_cam(self, data):
        #START communication
        self.cam.open_device()
        print('Camera OPEN')

    def close_cam(self, data):
        #stop communication
        self.cam.close_device()
        print('Camera CLOSED')

    def background_image(self, data):

        print('\n')
        print('Background images are beign taken')

        self.experiment_name = self.get_element("exp_name").text()
        print('Experiment_name: ', self.experiment_name)
        self.cell_to_start_with = self.get_element("first_cell_index").text()
        print('Cell number index: ', self.cell_to_start_with)

        number_of_devices = self.cam.get_number_devices()
        print('number_of_camaras', number_of_devices)
        device_info = self.cam.get_device_info_string('device_type')
        print('device_info', device_info)

        print('Opening camera...')
        self.cam.open_device()

        for i in range(1,4):
            self.type_of_image = 'background'
            print('*** just  before activating camara imaging')
            self.camera_imaging (i) #THIS fcn activates the imaging

        #stop communication
        self.cam.close_device()
        print('Camara CLOSED')

    def _on_valve_btn_click(self, data):

        print('\n')
        print('Single adquisiton ACTIVATED!')

        self.experiment_name = self.get_element("exp_name").text()
        print('Experiment_name: ', self.experiment_name)
        self.cell_to_start_with = self.get_element("first_cell_index").text()
        print('Cell number index: ', self.cell_to_start_with)

        number_of_devices = self.cam.get_number_devices()
        print('number_of_camaras', number_of_devices)
        device_info = self.cam.get_device_info_string('device_type')
        print('device_info', device_info)

        print('Opening camera...')
        self.cam.open_device()

        self.single_imaging_active = True
        print('set_valve activated')

        self.imaging_loop_started = False #reset imaging loop
        self.set_valve(True)

    def _on_valve_btn_click_loop(self, data):

        print('\n')
        print('MULTIPLE adquisiton ACTIVATED!')

        self.experiment_name = self.get_element("exp_name").text()
        print('Experiment_name: ', self.experiment_name)
        self.cell_to_start_with = self.get_element("first_cell_index").text()
        print('FIRST saved cell index: ', self.cell_to_start_with)
    
        number_of_devices = self.cam.get_number_devices()
        print('number_of_camaras', number_of_devices)
        device_info = self.cam.get_device_info_string('device_type')
        print('device_info', device_info)

        print('Opening camera...')
        self.cam.open_device()

        self.multiple_imaging_active = True
        # target number of cells--> self.number_of_cells_to_be_collected
        self.counter_for_multiple_imaging = 1
        print('set_valve activated')

        self.imaging_loop_started = False #reset imaging loop          
        self.set_valve(True)

    def set_leakPressure(self, value):
        self.leakPressure = int(value)
        self.leakLcd.display(self.leakPressure)

    def set_number_of_cells(self, value):
        self.number_of_cells_to_be_collected = int(value)
        self.numberLcd.display(self.number_of_cells_to_be_collected)

    
    def set_valve(self, valve_states):

        print('Activated set_valve ()...')
        #0x32 -> inject Preg
        self.instr_manager.set_pressure(0x31, self.leakPressure) #continous flow of #
        #Change if necessary
        self.instr_manager.ultra96_usleep(250000) #Wait after wellchange to let the system calm down 250000 us = 250 ms = 0.25 s
        
        self.instr_manager.can_valve(0x11,0,0,0,0,0,0,0) #open the loading valve
        self.instr_manager.ultra96_usleep(10000)
        #Change if necessary
        # (ROI_x, ROI_y, ROI_width, ROI_height, circularity, sensitivity, min_size, timeout)
        #self.instr_manager.ultra96_detect(310,50,160,230,20,18,100,3) # chip 13 PBMC:(350,50,160,230,20,18,25,3) // chip 13 big:(310,50,160,230,20,18,25,3) // chip 12: (350,40,150,190,20,18,25,3)  / (self, ROI_x, ROI_y, ROI_width, ROI_height, circularity, sensitivity, min_size, timeout)
        self.instr_manager.ultra96_detect(340-320,0,300+320,200,20,18,150,5) # Note that min_size was increased to 150 from 100 # previous:(350,50,160,230,20,18,100,3) // chip 13 big:(310,50,160,230,20,18,25,3) // chip 12: (350,40,150,190,20,18,25,3)  / (self, ROI_x, ROI_y, ROI_width, ROI_height, circularity, sensitivity, min_size, timeout)
        #If initial detection succesful close valve 1
        self.instr_manager.store_return(20)
        self.instr_manager.can_valve(0x11,1,0,1,0,0,0,0)
        self.instr_manager.start_if(20, 1, 1)
        self.instr_manager.ultra96_usleep(10000)

        #Change if necessary
        #self.instr_manager.ultra96_detect(350,360,195,40,20,18,80,1) # chip 13 PBMC:(350,350,195,25,20,18,100,1) // chip 13 big:(350,350,195,40,20,18,15,1) // chip 12:(320,290,180,40,20,18,20,1) detect one cell coming 50 was 100 #correct chip orientation 420,300,200,50,20,18,50,1
        self.instr_manager.ultra96_detect(160-100,(516//2)+60,190+40,120+40,20,18,80,5)
        self.instr_manager.store_return(20)
        self.instr_manager.set_pressure(0x31, 0) #stop the flow leak channel
        print('First stop the flow leak channel #HERE I THOUGHT IT SHOULD GO')
        
        #Clean encapsulation site
        #self.instr_manager.set_pressure(0x34, 1200) #start the flow oil counter pressure
        self.instr_manager.ultra96_usleep(200000) #400000 800000
        #self.instr_manager.set_pressure(0x34, 0) #stop the oil flow
        self.instr_manager.end_if()

        self.instr_manager.set_pressure(0x31, 0) #stop the flow leak channel
        print('Second stop the flow leak channel')
       
        self.instr_manager.fetch_return(30,20) #detect one cell coming 
        self.instr_manager.ultra96_usleep(10000) #50000

        #cell detection
        #for implementation when all imaging is controlled by the CAN command
        #self.instr_manager.start_if(30, 1, 1)
        #self.instr_manager.start_exposure(self.uscope) #CAN message
        #self.instr_manager.end_if()

        msg = self.instr_manager.gen_message()
        self.send_toRelai("instr", msg)
        print('after send to Relai')

    def set_busy(self, data):
        if(data == 1):
            self.activate_elements(True, ["stop single cell"])
            self.activate_elements(True, ["collect multiple cells"])

            if self.imaging_loop_started == True: # check if cell was detected
                print('### ENTERED the busy FALSE image process LOOP!')
                self.imaging_loop_started = False

                if self.single_imaging_active == True:
                                    
                    #restart all the cell detection variables
                    self.single_imaging_active = False
                    print('SINGLE acquisition COMPLETED!')
                    #self.is_imaging_complete = True
                    self.cam.close_device()
                    print('Camara CLOSED')

                if self.multiple_imaging_active == True:
                    self.counter_for_multiple_imaging += 1 # counts a new image
                         
                    if self.counter_for_multiple_imaging <= self.number_of_cells_to_be_collected: # check if the correct number of images requested was reached
                        print('Next, imaging cell number ', self.counter_for_multiple_imaging)
                        self.set_valve(True) #Keeps the cell stopping and imaging going!

                    else:
                        print('MULTIPLE acquisition sequence COMPLETED!')
                        #restart all the cell detection variables
                        self.multiple_imaging_active = False
                        #self.is_imaging_complete = True
                        self.cam.close_device()
                        print('Camara CLOSED')
           
        if(data == 0):
            self.activate_elements(False, ["stop single cell"])
            self.activate_elements(False, ["collect multiple cells"])

    def cell_detect_boolean(self, data_cam):
        print(f'Received data on register 30: {data_cam}')
        
        if(data_cam[0] == 1):
            print('*** just  before activating camara imaging')
            print(f'Capturing image for cell number: {self.cell_number_counter}')
            self.camera_imaging (self.cell_number_counter) #THIS fcn activates the imaging
            self.cell_number_counter += 1
     
        if(data_cam[0] == 0):
            print('NO image taken because data from register 30 = False')

        print('SET self.imaging_loop_started = True')
        self.imaging_loop_started = True # boolean to store cell detection event

print('Loaded class valve_cluster():')

def start(IRIS, x_pos, y_pos, width, mask):  
    valve_cluster(IRIS,"StopCell", x_pos, y_pos, width, mask)
    print('Activated def start():')
