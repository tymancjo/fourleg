# This script act as a voice command remote control for Aron RoboDog.
# It need to connect to the robot via Bluetooth
# And it's based on the SpeechRecognition library with use of the google
# online voice to text service - hence it need a internet connection.


# Imports for use audio input and speech recognition
import speech_recognition as sr

# Imports for the use of BLE connectivity
import asyncio
from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

# general imports
import time
import random

# Preparing to use the BLE connectivity
# making a class to handle BT
class myBT:
    def __init__(self, uuid):

        self.uuid = uuid
        self.BTsetup()
        self.connect()

    def BTsetup(self):
        self.the_device = ""
        self.the_service = ""
        self.client = False
        self.loop = False

        self.is_serial = False
        self.is_BLE = False

        # self.uuid2 = "0000ffe1-0000-1000-8000-00805f9b34fb"

    async def BTsearch(self):
        scanner = BleakScanner(service_uuids=[self.uuid])
        devices = await scanner.discover(
            service_uuids=[
                self.uuid,
            ]
        )
        for i, d in enumerate(devices):
            print(f"[{i}]\t{d.name}\t{d.address}")
            if "BT05" in d.name:
                print(f"Potenitial robot found @ {d.address}")
                self.the_device = d.address

    async def BTgetServices(self):
        scanner = BleakScanner(service_uuids=[self.uuid])
        device = await scanner.find_device_by_address(
            self.the_device, timeout=20.0, service_uuids=[self.uuid]
        )

        if not device:
            raise BleakError(
                f"A device with address {self.the_device} could not be found."
            )
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

        self.client = BleakClient(
            self.the_device, timeout=10, service_uuids=[self.uuid]
        )
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
            await self.client.write_gatt_char(
                self.the_service, bytearray(the_command, "utf-8"), response=not True
            )
        else:
            print("No device connected.")
            if redial and self.the_service:
                self.loop.run_until_complete(self.BTconnect())

    def BLE_sent(self, command):
        self.loop.run_until_complete(self.BTwrite(command + "\n"))

    def disconnect(self):
        self.loop.run_until_complete(self.BTdisconnect())

    def connect(self):
        if not self.client:
            print("Scanning for BLE devices...")
            self.loop = asyncio.get_event_loop()
            self.loop.run_until_complete(self.BTsearch())

            if self.the_device:
                print(f"there is Aron unit at {self.the_device}")
                self.loop.run_until_complete(self.BTgetServices())
                if self.the_service:
                    print(f"Found Vendor services at {self.the_service}")
                    self.loop.run_until_complete(self.BTconnect())
            else:
                print("No Aron BT05 found :(")
                ...
        else:
            print("Shall be already connected...")


# Some helper function to check for the command in text
def isInText(command, text):
    for t in command.split():
        if t in text:
            return True
    return False


# making the BT connection
Aron = myBT("0000ffe0-0000-1000-8000-00805f9b34fb")
Aron.connect()


# Some pre set for the recognition
r = sr.Recognizer()
# This makes the bck noise level sensitivity more reasonable
r.energy_threshold = 1000
r.dynamic_energy_threshold = False
rec_language = "pl-PL"

# some handy helper variables
look_around = True
text = ""

while True:
    print("Powiedz coś...")
    with sr.Microphone() as source:
        audio = r.listen(source)

    try:
        text = r.recognize_google(audio, language=rec_language)
        print("Usłyszano:" + text)
    except LookupError:
        print("Nie rozpoznano")
    except sr.UnknownValueError:
        print("Coś nie zadziałało")

    text = text.lower()
    if "stop" in text:
        print("stop")
        Aron.BLE_sent("s")

    if "aron" in text:
        print("got command!")
        if "koniec" in text:
            Aron.BLE_sent("s")
            break
        elif isInText("siadaj siad świat kwiat", text):
            print("siad!")
            Aron.BLE_sent("s")
            Aron.BLE_sent("up 50 -50")

        elif isInText("leżeć leż", text):
            print("leżeć!")
            Aron.BLE_sent("s")
            Aron.BLE_sent("up -50 -50")

        elif isInText("stój trop", text):
            print("trop!")
            Aron.BLE_sent("s")
            Aron.BLE_sent("up 50 50")

        elif "noga" in text:
            print("noga")
            Aron.BLE_sent("s")
            Aron.BLE_sent("test 2 10")

        elif isInText("cofnij wracaj", text):
            print("back")
            Aron.BLE_sent("s")
            Aron.BLE_sent("back 2 10")

        elif isInText("cicho spokój ruchy", text):
            print("random")
            Aron.BLE_sent("random")

        elif isInText("łapa noga daj łapę mapa", text):
            Aron.BLE_sent("s")
            time.sleep(0.5)
            Aron.BLE_sent("test 4 1")
            time.sleep(random.randint(2, 7))
            Aron.BLE_sent("back 4 1")

        elif isInText("szukaj szuka", text):
            Aron.BLE_sent("s")
            if look_around:
                Aron.BLE_sent("test 3 5")
            else:
                Aron.BLE_sent("back 3 5")

            look_around = not look_around

        elif isInText("lewo", text):
            Aron.BLE_sent("w 0 -100")

        elif isInText("prawo", text):
            Aron.BLE_sent("w 0 100")

        elif isInText("ruszaj przodu", text):
            Aron.BLE_sent("w 100 0")

        elif isInText("tyłu", text):
            Aron.BLE_sent("w -100 0")

    text = ""

Aron.disconnect()
