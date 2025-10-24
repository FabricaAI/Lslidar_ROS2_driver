import socket
import time
import struct

BROADCAST_IP = "255.255.255.255"
BROADCAST_PORT = 50000
LOCAL_PORT = 60000
PACKET_SIZE = 285

def build_ping_packet():
    header = b"CH9121_CFG_FLAG\x00"  # null-terminated string
    cmd = bytes([0x04])
    padding = bytes(PACKET_SIZE - len(header) - len(cmd))
    return header + cmd + padding

def send_discovery_ping():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", LOCAL_PORT))  # bind to all interfaces on port 60000

    packet = build_ping_packet()
    sock.sendto(packet, (BROADCAST_IP, BROADCAST_PORT))
    print(f"Sent {len(packet)}-byte discovery packet to {BROADCAST_IP}:{BROADCAST_PORT}")

    sock.settimeout(3.0)
    start = time.time()
    try:
        while True:
            data, addr = sock.recvfrom(512)
            if len(data) >= 16 and data.startswith(b"CH9121_CFG_FLAG\x00"):
                cmd_byte = data[15]
                if cmd_byte == 0x84:
                    print(f"Reply from {addr[0]}:{addr[1]} — Discovery ACK (0x84)")
                else:
                    print(f"Unknown reply from {addr[0]}: cmd=0x{cmd_byte:02X}")
            else:
                print(f"Non-matching packet from {addr}: {data[:20]!r}")
    except socket.timeout:
        print("No more replies.")
    finally:
        sock.close()

if __name__ == "__main__":
    send_discovery_ping()
