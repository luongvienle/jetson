# datalink.py
# Simple async UDP/TCP client for sending telemetry packets to Yamcs.
import asyncio
import socket

class UDPClient:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    async def send(self, data: bytes):
        # non-blocking send via loop.run_in_executor
        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, self.sock.sendto, data, (self.host, self.port))

    async def close(self):
        self.sock.close()

class TCPClient:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.writer = None
        self.reader = None
        self.lock = asyncio.Lock()

    async def connect(self):
        try:
            self.reader, self.writer = await asyncio.open_connection(self.host, self.port)
        except Exception as e:
            # connection failed; will retry on send
            self.reader = None
            self.writer = None

    async def send(self, data: bytes):
        async with self.lock:
            if self.writer is None:
                await self.connect()
                if self.writer is None:
                    # drop packet or raise
                    return
            try:
                self.writer.write(data)
                await self.writer.drain()
            except Exception as e:
                # reset connection and swallow error
                try:
                    self.writer.close()
                except:
                    pass
                self.writer = None

    async def close(self):
        if self.writer:
            self.writer.close()
            await self.writer.wait_closed()
