from . import AtlasI2C as atlas

def get_device(address):
    device = atlas.AtlasI2C()
    device_address_list = device.list_i2c_devices()
    
    for i in device_address_list:
        if not i == address:
            continue

        device.set_i2c_address(i)
        moduletype = device.query("i")
        name = device.query("name,?")
        return atlas.AtlasI2C(address = i, moduletype = moduletype, name = name)
    
