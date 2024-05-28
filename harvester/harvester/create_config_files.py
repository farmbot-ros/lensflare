from arena_api.system import system

IP_LIGHT = {
      0 : None,
      1 : "192.168.2.251", # 7 RIGHT     70:B3:D5:DD:90:FD
      2 : "192.168.2.253", # 9 LEFT      70:B3:D5:DD:90:ED
      3 : "192.168.2.252", # - CENTER    70:B3:D5:DD:90:EF
}

GAIN = 20
EXP = 1.2

BY_MAC = { 
    ##  MAC             IP      CHANNEL NAME    EXP     GAIN
    30853686642969: [IP_LIGHT[2],   7,   "9b",   521*EXP,  30],
    30853686643065: [IP_LIGHT[2],   1,   "9a",   521*EXP,  30],
    30853686646563: [IP_LIGHT[2],   2,   "9c",   521*EXP,  30],
    30853686653149: [IP_LIGHT[2],   5,   "9d",   521*EXP,  30],
    30853686643056: [IP_LIGHT[2],   6,   "9e",   521*EXP,  30],
    30853686646554: [IP_LIGHT[1],   6,   "7a",   521*EXP,  30],
    30853686652294: [IP_LIGHT[1],   1,   "7b",   521*EXP,  30],
    30853686445113: [IP_LIGHT[1],   5,   "7c",   521*EXP,  30],
    30853686646528: [IP_LIGHT[1],   2,   "7d",   521*EXP,  30],
    30853686653140: [IP_LIGHT[1],   3,   "7e",   521*EXP,  30],
    30853686643187: [IP_LIGHT[0],   0,   "1",   2847*1.1,  20],
    30853686650340: [IP_LIGHT[3],   6,   "4",    528*2.8,  30],
    30853686646397: [IP_LIGHT[3],   5,   "3",    810*2.8,  20],
    30853686643161: [IP_LIGHT[3],   2,   "13",   805*3.8,  20],
    30853686643152: [IP_LIGHT[3],   1,   "12",   460*3.8,  40],
    30853686646406: [IP_LIGHT[3],   3,   "10",     867.0,  30],
    30853686650497: [IP_LIGHT[3],   7,   "11a",    498.0,  15],
    30853686650366: [IP_LIGHT[3],   7,   "11b",    498.0,  15],
}

# check for available cameras
device_infos_no_dup = []
for device in system.device_infos:
    if device not in device_infos_no_dup:
        device_infos_no_dup.append(device)

for camera in device_infos_no_dup:
    dec_mac = camera["mac"]
    hex_mac = int(dec_mac.replace(":", ""), 16)
    try:
        camera_name = BY_MAC[hex_mac][2]
        print(f"setting up config file for camera {camera_name}")
        config_file_path = '/home/bresilla/ROS_OXBO/src/harvester/configs/'+ camera_name + '.txt'
        device = system.create_device(camera)[0]
        nodemap = device.nodemap
        nodemap.get_node('PixelFormat').value = 'BGR8'
        nodemap.get_node('Width').value = nodemap.get_node('Width').max
        nodemap.get_node('Height').value = nodemap.get_node('Height').max
        nodemap.get_node('DeviceStreamChannelPacketSize').value = nodemap.get_node('DeviceStreamChannelPacketSize').max
        nodemap.write_streamable_node_values_to(config_file_path)
        system.destroy_device(device)
   
    except Exception as e:
        print(f"An error occurred: {str(e)}")