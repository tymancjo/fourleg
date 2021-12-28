# import asyncio
# from bleak import BleakClient

# address = "50:65:83:79:58:8A"
# MODEL_NBR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
# device_uuid = "A5F4E928-D493-C858-ADFE-E57B3BBFCB6E"
# # device_uuid = "0000ffe1-0000-1000-8000-00805f9b34fb"

# async def main(address):
#     async with BleakClient(address) as client:
#         model_number = await client.read_gatt_char(MODEL_NBR_UUID)
#         print("Model Number: {0}".format("".join(map(chr, model_number))))

# asyncio.run(main(address))


import asyncio
import platform
import sys
import uuid
from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError


ADDRESS = (
    "24:71:89:cc:09:05"  # <--- Change to your device's address here if you are using Windows or Linux
    if platform.system() != "Darwin"
    else "12C60956-CD97-E27E-1C50-E983779B27DF"  # <--- Change to your device's address here if you are using macOS
)

device_uuid = "0000ffe0-0000-1000-8000-00805f9b34fb"



async def main(ble_address: str):
    device = await BleakScanner.find_device_by_address(ble_address, timeout=20.0,service_uuids=[device_uuid])
    if not device:
        raise BleakError(f"A device with address {ble_address} could not be found.")
    async with BleakClient(device) as client:
        svcs = await client.get_services()
        print("Services:")
        for service in svcs:
            print(service)


if __name__ == "__main__":
    asyncio.run(main(sys.argv[1] if len(sys.argv) == 2 else ADDRESS))