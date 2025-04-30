import asyncio
from bleak import BleakScanner, BleakClient
from bleak.backends.characteristic import BleakGATTCharacteristic
import logging
import logging
import struct


logger = logging.getLogger(__name__)
UU_RESPIRATORY_UUID =  "84b45e35-140b-4d33-92c6-386d8bff160d" 
BATTERY_LEVEL_UUID =    "00002A19-0000-1000-8000-00805f9b34fb"


class uutrack_reader:
    
    def __init__(self):
        self.CSC = 0
        self.semaphore = asyncio.Semaphore(0)            
    
    async def notification_handler(self, sender, data : bytearray):                
        voltages_encoded = struct.iter_unpack("<I", data)
        for voltage_encoded in voltages_encoded:            
            voltage = (voltage_encoded[0]/(1<<23)-1)*(2.5/128)
            print(f'{voltage}')

    def on_disconnect(self, client):
        print("ble disconnect detected")
        self.semaphore.release()
              
    async def makeconnectiontodevice(self, device):
        client = BleakClient(device.address, timeout=30, disconnected_callback=self.on_disconnect)
        
        try:
            await client.connect()
            print("connected to device")
            battery_level_byte = await client.read_gatt_char(BATTERY_LEVEL_UUID)
            
            print(f'battery level {int.from_bytes(battery_level_byte, byteorder="little")}')
            await client.start_notify(UU_RESPIRATORY_UUID, self.notification_handler)
            await self.semaphore.acquire()
                
        except Exception as e:
            print(e)
        finally:
            await client.disconnect()
            print(f'disconnected ')
    

class datacollector:
    async def app(self):    
        print("scanning devices")
        devices = await BleakScanner.discover()
        for d in devices:
            if(d.name == "UU_respiratory"):                            
                print(f'device found:{d}')                
                the_reader = uutrack_reader()
                await the_reader.makeconnectiontodevice(d)
                
                    

def main(): 
    collectior = datacollector()
    asyncio.run(collectior.app())

if __name__ == "__main__":
    main()
