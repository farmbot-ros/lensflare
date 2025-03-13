#include <arv.h>
#include <iostream>

int main() {
    // Refresh the list of devices
    arv_update_device_list();

    // Get the number of available devices
    int num_devices = arv_get_n_devices();
    std::cout << "Found " << num_devices << " device(s):" << std::endl;

    // Iterate through the devices and print their IDs
    for (int i = 0; i < num_devices; ++i) {
        const char *device_id = arv_get_device_id(i);
        if (device_id) {
            std::cout << "Device " << i << ": " << device_id << std::endl;
        } else {
            std::cout << "Device " << i << ": [No device ID available]" << std::endl;
        }
    }

    return 0;
}
