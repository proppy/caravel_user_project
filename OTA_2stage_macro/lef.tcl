gds read OTA_2stage_macro.gds
load OTA_2stage_macro

box values 0 0 0 0

port VDD class inout
port VDD use power
port VSS class inout
port VSS use ground
port VOUT remove
port VIN1 remove
port VIN2 remove
port VP remove

lef write -hide

