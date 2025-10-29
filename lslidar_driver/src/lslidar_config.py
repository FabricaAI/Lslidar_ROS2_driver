import socket
import time

BROADCAST_IP = "255.255.255.255"
BROADCAST_PORT = 50000
LOCAL_PORT = 60000
PACKET_SIZE = 285

# --- Original ping workflow ---
def build_ping_packet():
    header = b"CH9121_CFG_FLAG\x00"
    cmd = bytes([0x04])  # ping command
    padding_len = max(0, PACKET_SIZE - len(header) - len(cmd))
    padding = bytes(padding_len)
    return header + cmd + padding

def send_discovery_ping():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", LOCAL_PORT))

    packet = build_ping_packet()
    sock.sendto(packet, (BROADCAST_IP, BROADCAST_PORT))
    print(f"Sent {len(packet)}-byte ping packet to {BROADCAST_IP}:{BROADCAST_PORT}")

    discovered_macs = []
    sock.settimeout(3.0)
    try:
        while True:
            try:
                data, addr = sock.recvfrom(512)
            except socket.timeout:
                break

            if len(data) >= 23 and data.startswith(b"CH9121_CFG_FLAG\x00"):
                cmd_byte = data[16]
                if cmd_byte == 0x84:
                    lidar_mac = extract_lidar_mac(data)
                    if lidar_mac not in discovered_macs:
                        discovered_macs.append(lidar_mac)
                    print(f"Ping reply from {addr[0]} — LiDAR MAC: {lidar_mac} — Response Code: 0x{cmd_byte:02X}")
                else:
                    print(f"Unknown reply from {addr[0]}: cmd=0x{cmd_byte:02X}")
            else:
                print(f"Non-matching packet from {addr}: {data[:20]!r}")
    finally:
        sock.close()

    print(f"Total discovered LiDARs: {len(discovered_macs)}")
    return discovered_macs

def extract_lidar_mac(data):
    if len(data) < 23:
        raise ValueError("Data too short to contain MAC address")
    
    mac_bytes = data[17:23]
    mac_str = ':'.join(f'{b:02X}' for b in mac_bytes)
    return mac_str

def mac_str_to_bytes(mac_str):
    """
    Convert MAC string "AA:BB:CC:DD:EE:FF" to 6-byte bytes object
    """
    return bytes(int(b, 16) for b in mac_str.split(':'))

def build_read_request_packet_with_mac(lidar_mac_str, local_mac_bytes=None):
    """
    Build a read request packet including the LiDAR MAC
    """
    header = b"CH9121_CFG_FLAG\x00"
    cmd = bytes([0x02])

    lidar_mac = mac_str_to_bytes(lidar_mac_str)
    if local_mac_bytes is None:
        local_mac_bytes = b'\x00'*6  # or use get_local_mac_bytes() to be precise

    assert len(lidar_mac) == 6
    assert len(local_mac_bytes) == 6

    payload = header + cmd + lidar_mac + local_mac_bytes
    padding_len = max(0, PACKET_SIZE - len(payload))
    payload += bytes(padding_len)
    return payload

def send_read_request(lidar_mac_str, local_mac_bytes=None):
    """
    Send a read request packet to the LiDAR using its MAC address.
    Listens for and prints any valid replies.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", LOCAL_PORT))

    # Build packet using the LiDAR's MAC (and optional local MAC)
    packet = build_read_request_packet_with_mac(lidar_mac_str, local_mac_bytes)
    sock.sendto(packet, (BROADCAST_IP, BROADCAST_PORT))
    print(f"Sent {len(packet)}-byte read request to {BROADCAST_IP}:{BROADCAST_PORT}")

    discovered_macs = []
    sock.settimeout(3.0)

    try:
        while True:
            try:
                data, addr = sock.recvfrom(512)
            except socket.timeout:
                break

            # Verify the reply starts correctly and is long enough
            if len(data) >= 23 and data.startswith(b"CH9121_CFG_FLAG\x00"):
                cmd_byte = data[16]

                # 0x82 is the read-reply command byte from CH9121 for a read request
                if cmd_byte == 0x82:
                    lidar_mac = extract_lidar_mac(data)
                    if lidar_mac not in discovered_macs:
                        discovered_macs.append(lidar_mac)
                    print(f"Read reply from {addr[0]} — LiDAR MAC: {lidar_mac} — CMD: 0x{cmd_byte:02X}")
                else:
                    print(f"Unknown reply from {addr[0]} (cmd=0x{cmd_byte:02X})")
            else:
                print(f"Non-matching packet from {addr}: {data[:20]!r}")
    finally:
        sock.close()

    print(f"Total LiDARs that replied: {len(discovered_macs)}")
    return discovered_macs

# --- Example usage ---
if __name__ == "__main__":
    print("=== Ping Discovery ===")
    mac_list = send_discovery_ping()

    print(f"\nDiscovered LiDAR MACs: {mac_list}")

    if not mac_list:
        print("No LiDAR detected. Exiting.")
    else:
        lidar_mac_str = mac_list[0]
        print(f"\n=== Sending Read Request to {lidar_mac_str} ===")
        send_read_request(lidar_mac_str)