# Simple test UDP server that prints received packets (useful to validate bridge output).
import socket
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--host', default='0.0.0.0')
parser.add_argument('--port', type=int, default=10015)
args = parser.parse_args()
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((args.host, args.port))
print(f"Listening UDP on {args.host}:{args.port}")
try:
    while True:
        data, addr = s.recvfrom(65536)
        print(f"Got {len(data)} bytes from {addr}")
except KeyboardInterrupt:
    print('exit')