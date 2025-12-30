# EPFL_thesis_11521
Stop-and-go microfluidic QPI system for single-cell phenotyping of refractive organelles and cellular components
List of code:

Image processing and analysis code
1. complex_field_holo_extract.ipynb --> code to process the holograms, do Fourier filtering, bacnkground substraction, unwrapping, and cell detection and cropping
2. segmentation_rembg.ipynb --> code to segment the cell images
3. model_classification_senes.ipynb --> code with the phenotypic feature extraction and the logistic regression, XGB, and ResNet18 models

Laser controller and CAN
4. main.c --> code to run the laser controllers
5. can_interface.c --> CAN interface code
6. can interface.h --> CAN interface code h file

User interface and image acquisition
7.