# Imports for use audio input and speech recognition

import speech_recognition as sr
# Imports for the use of BLE connectivity
import asyncio
from bleak import BleakClient, BleakScanner 
from bleak.exc import BleakError



# Preparing to use the BLE connectivity
# -- Windows only configuration --
try:
    from ctypes import windll
    windll.shcore.SetProcessDpiAwareness(1)
except:
    pass
    
class myBT:

    def __init__(self):
        self.BTsetup();
        self.connect();


    def BTsetup(self):
        self.the_device = ""
        self.the_service = ""
        self.client = False
        self.loop = False

        self.is_serial = False
        self.is_BLE = False

        self.uuid = "0000ffe0-0000-1000-8000-00805f9b34fb"
        self.uuid2 = "0000ffe1-0000-1000-8000-00805f9b34fb"
        # self.uuid = "A5F4E928-D493-C858-ADFE-E57B3BBFCB6E"


    async def BTsearch(self):
        scanner = BleakScanner(service_uuids=[self.uuid,])
        devices = await scanner.discover(service_uuids=[self.uuid,])
        for i,d in enumerate(devices):
            print(f"[{i}]\t{d.name}\t{d.address}")
            if "BT05" in d.name:
                print(f"Potenitial robot found @ {d.address}")
                self.the_device = d.address


    async def BTgetServices(self ):
        scanner = BleakScanner(service_uuids=[self.uuid,])
        device = await scanner.find_device_by_address(self.the_device, timeout=20.0,service_uuids=[self.uuid])
        
        if not device:
            raise BleakError(f"A device with address {self.the_device} could not be found.")
        else:
            self.the_device = device

        async with BleakClient(device) as client:
            svcs = await client.get_services()
            print("Services:")
            for service in svcs:
                print(service)
                if "Vendor specific" in str(service):
                    print(service.characteristics)
                    for inside in service.characteristics:
                        print(f"Sub info properties: {inside.properties}")
                        print(f"Sub info uuid: {inside.uuid}")
                        self.the_service = inside.uuid

    async def BTconnect(self):
        """
        The class method to connecto tot the BT robot
        Based on the blake library for the BLE device operations.
        """

        print("Connecting...")
        print("trying direct BLE connection...")

        self.client = BleakClient(self.the_device,timeout=10, service_uuids=[self.uuid,self.uuid2])
        await self.client.connect()
        connection_status = self.client.is_connected
        print(f"Connection status: {connection_status}")

        if connection_status:
            print("Connected mode..")
            self.is_serial = False
            self.is_BLE = True
        else:
            self.is_serial = False
            self.is_BLE = False

    async def BTdisconnect(self):
            if self.client:
                if self.client.is_connected:
                    await self.client.disconnect()
                    print("The robot have been disconnected...")
                else:
                    print("No robot to be disconnected!")
            else:
                print("No robot to be disconnected!")

    async def BTwrite(self, the_command, redial=True):
        if self.client.is_connected:
            await self.client.write_gatt_char(self.the_service,bytearray(the_command, "utf-8"), response=not True)
        else:
            print("No device connected.")
            if redial and self.the_service:
                self.loop.run_until_complete(self.BTconnect()) 

    def BLE_sent(self, command):
        self.loop.run_until_complete(self.BTwrite(command))

    def disconnect(self):
        self.loop.run_until_complete(self.BTdisconnect())

    def connect(self):
        if not self.client:
            print("Scanning for BLE devices...")
            self.loop = asyncio.get_event_loop()
            self.loop.run_until_complete(self.BTsearch())

            if self.the_device:
                print(f"there is Mariola at {self.the_device}")
                self.loop.run_until_complete(self.BTgetServices())
                if self.the_service:
                    print(f"Found Vendor sercvice at {self.the_service}")
                    self.loop.run_until_complete(self.BTconnect())
            else:
                print("No Aron BT05 found :(")
                ...
        else:
            print("Shall be already connected...")

# making the BT connection
Aron = myBT()
Aron.connect()


# Some pre set for the recognition
r=sr.Recognizer()
r.energy_threshold=4000
r.dynamic_energy_threshold = False

# some handy helper vals
look_around = True
text = ""

while True:
    print("Say something...")
    with sr.Microphone() as source:
        audio=r.listen(source)

    try:
        text = r.recognize_google(audio, language="pl-PL")
        print("Speech was:" + text)
    except LookupError:
        print('Speech not understood')
    except sr.UnknownValueError:
        print("Nierozpozane")

    text = text.lower()
    if "stop" in text:
        print("stop")
        Aron.BLE_sent("s \n")

    if "aron" in text:
        print("got command!")
        if "koniec" in text:
            Aron.BLE_sent("s \n")
            break
        if "siadaj" in text or "siad" in text:
            print("siad!")
            Aron.BLE_sent("up 50 -50 \n")

        if "stój" in text or "trop" in text:
            print("siad!")
            Aron.BLE_sent("up 50 50 \n")
            
        if "noga" in text:
            print("noga")
            Aron.BLE_sent("test 2 \n")

        if "tyłu" in text or "cofnij" in text:
            print("back")
            Aron.BLE_sent("back 2 \n")

        if "cicho" in text or "spokój" in text or "ruchy" in text:
            print("random")
            Aron.BLE_sent("random \n")

        if "szukaj" in text:
            if look_around:
                Aron.BLE_sent("w 0 -100 \n")    
            else:
                Aron.BLE_sent("w 0 100 \n")    

            look_around = not look_around

    text = ""

Aron.disconnect()