#!/usr/bin/env python

# WS server example

import asyncio
import websockets
import time

async def hello(websocket, path):
    name = await websocket.recv()
    print(f"Recieved: {name}")

    time.sleep(1);

    if name == "exit":
        await websocket.send("Exiting.")
        print("Exiting.")
        exit()

    greeting = f"Hello {name}!"
    await websocket.send(greeting)
    print(f"Sent: {greeting}")

hostname = "localhost"
port = 8765

start_server = websockets.serve(hello, "localhost", 8765)

print(f"Starting server on {hostname}:{port}")

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
