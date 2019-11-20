#!/usr/bin/env python

# WS client example

import asyncio
import websockets


hostname = "localhost"
port = 8765
uri = f"ws://{hostname}:{port}"
print(f"Connecting to server at {hostname}:{port}")

async def hello():
    async with websockets.connect(uri) as websocket:
        name = input("What's your name? ")

        await websocket.send(name)
        print(f"Sent: {name}")

        greeting = await websocket.recv()
        print(f"Recieved: {greeting}")

try:
    asyncio.get_event_loop().run_until_complete(hello())
except:
    print("An error occurred. Is the server online at the specified address?")
