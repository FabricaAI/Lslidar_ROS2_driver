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

def parse_lidar_config(data):
    """
    Parse a CH9121 read response packet and extract key configuration fields.
    """

    # --- Basic validation ---
    if not data.startswith(b"CH9121_CFG_FLAG"):
        raise ValueError("Invalid packet header")
    if len(data) < 0xC0:
        raise ValueError("Packet too short for expected fields")

    # --- Core structure decoding ---
    packet_type = data[16]
    lidar_mac_bytes = data[17:23]
    local_mac_bytes = data[23:29]
    # const_cc = data[29]
    # secondary_header = data[0x23:0x2B].rstrip(b'\x00')
    
    # --- Network configuration ---
    lidar_ip = data[0x3E:0x42]
    lidar_gateway = data[0x42:0x46]
    lidar_subnet = data[0x46:0x4A]
    
    # --- UART + destination config ---
    lidar_udp_port_bytes = data[0xAD:0xAF]
    destination_ip = data[0xAF:0xB3]
    destination_udp_port_bytes = data[0xB3:0xB5]
    baudrate_bytes = data[0xB5:0xB9]
    uart_data_bits = data[0xB9]
    uart_stop_bits = data[0xBA]
    uart_parity_bits = data[0xBB]
    # close_connection = data[0xBC]
    max_udp_packet_length_bytes = data[0xBD:0xBF]

    # --- Helper conversions ---
    lidar_mac = ':'.join(f'{b:02X}' for b in lidar_mac_bytes)
    local_mac = ':'.join(f'{b:02X}' for b in local_mac_bytes)

    def ip_to_str(ip_bytes):
        return '.'.join(str(b) for b in ip_bytes)

    def le_to_uint16(b):
        return int.from_bytes(b, 'little')

    def le_to_uint32(b):
        return int.from_bytes(b, 'little')

    # --- Construct readable dict ---
    parsed = {
        # "packet_type": f"0x{packet_type:02X}",
        "LiDAR MAC": lidar_mac,
        "Local MAC": local_mac,
        # "const_cc": f"0x{const_cc:02X}",
        # "secondary_header": secondary_header.decode(errors='ignore'),
        "LiDAR IP": ip_to_str(lidar_ip),
        "LiDAR Gateway": ip_to_str(lidar_gateway),
        "LiDAR Subnet": ip_to_str(lidar_subnet),
        "LiDAR UDP Port": le_to_uint16(lidar_udp_port_bytes),
        "Destination IP": ip_to_str(destination_ip),
        "Destination UDP Port": le_to_uint16(destination_udp_port_bytes),
        "Baudrate": le_to_uint32(baudrate_bytes),
        "UART Data Bits": uart_data_bits,
        "UART Stop Bit": uart_stop_bits,
        "UART Parity Bit": f"0x{uart_parity_bits:02X}",
        # "close_connection": close_connection,
        "UDP Packet Max Length": le_to_uint16(max_udp_packet_length_bytes),
    }

    return parsed

def build_write_request_packet(lidar_mac_str, local_mac_bytes=None):
    """
    Build a CH9121 write request packet that modifies LiDAR network and UART settings.
    Dynamically reads current LiDAR IP and gateway, sets destination IP to local machine.
    """
    header = b"CH9121_CFG_FLAG\x00"
    cmd = bytes([0x01])  # write request

    lidar_mac = mac_str_to_bytes(lidar_mac_str)
    if local_mac_bytes is None:
        local_mac_bytes = b'\x00' * 6

    assert len(lidar_mac) == 6
    assert len(local_mac_bytes) == 6

    # --- Get current LiDAR config ---
    data_from_lidar = send_request(lidar_mac_str, mode="read", return_data=True)
    parsed = parse_lidar_config(data_from_lidar)

    lidar_ip = bytes(int(b) for b in parsed["LiDAR IP"].split('.'))
    lidar_gateway = bytes(int(b) for b in parsed["LiDAR Gateway"].split('.'))
    lidar_subnet = bytes([255, 255, 0, 0])  # example change

    # --- Example editable fields ---
    lidar_udp_port = int(parsed["LiDAR UDP Port"]).to_bytes(2, 'little')
    destination_ip = get_local_ip_bytes()  # your machine IP
    destination_udp_port = (2368).to_bytes(2, 'little')

    baudrate = (512000).to_bytes(4, 'little')
    uart_data_bits = bytes([8])
    uart_stop_bits = bytes([1])
    uart_parity_bits = bytes([0x04])
    close_connection = bytes([0x00])
    max_udp_packet_length = (1024).to_bytes(2, 'little')

    # --- Assemble payload ---
    payload = bytearray(PACKET_SIZE)
    payload[0:len(header)] = header
    payload[16] = cmd[0]
    payload[17:23] = lidar_mac
    payload[23:29] = local_mac_bytes
    payload[29] = 0xCC
    payload[0x23:0x2B] = b"CH9121 \x00"

    # --- Network config ---
    payload[0x3E:0x42] = lidar_ip
    payload[0x42:0x46] = lidar_gateway
    payload[0x46:0x4A] = lidar_subnet

    # --- UDP / UART config ---
    payload[0xAD:0xAF] = lidar_udp_port
    payload[0xAF:0xB3] = destination_ip
    payload[0xB3:0xB5] = destination_udp_port
    payload[0xB5:0xB9] = baudrate
    payload[0xB9] = uart_data_bits[0]
    payload[0xBA] = uart_stop_bits[0]
    payload[0xBB] = uart_parity_bits[0]
    payload[0xBC] = close_connection[0]
    payload[0xBD:0xBF] = max_udp_packet_length

    return bytes(payload)

def get_local_ip_bytes():
    """
    Returns the local machine's IP as 4 bytes suitable for the LiDAR payload.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # This doesn't actually send anything; it's just used to get the local IP
        s.connect(("8.8.8.8", 80))
        ip_str = s.getsockname()[0] 
    finally:
        s.close()
    # Convert to bytes
    return bytes(int(b) for b in ip_str.split('.'))

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

def send_request(lidar_mac_str, mode="read", local_mac_bytes=None, payload=None, return_data=False):
    """
    Send either a read or write request to the LiDAR using UDP broadcast.
    mode = "read" (0x02) or "write" (0x01)
    If 'payload' is provided, it overrides auto-generated packets.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", LOCAL_PORT))

    # --- Build or use provided payload ---
    if payload is not None:
        packet = payload
    elif mode == "read":
        packet = build_read_request_packet_with_mac(lidar_mac_str, local_mac_bytes)
    elif mode == "write":
        packet = build_write_request_packet(lidar_mac_str, local_mac_bytes)
    else:
        raise ValueError("mode must be 'read' or 'write'")

    sock.sendto(packet, (BROADCAST_IP, BROADCAST_PORT))
    print(f"Sent {len(packet)}-byte {mode.upper()} request to {BROADCAST_IP}:{BROADCAST_PORT}")

    sock.settimeout(3.0)
    recvd_data = None  # Initialize here
    try:
        while True:
            try:
                data, addr = sock.recvfrom(512)
            except socket.timeout:
                break

            if len(data) >= 23 and data.startswith(b"CH9121_CFG_FLAG\x00"):
                cmd_byte = data[16]
                print(f"Reply from {addr[0]} — CMD: 0x{cmd_byte:02X}")

                if cmd_byte == 0x82 and mode == "read":
                    # Parse configuration if this was a read request
                    try:
                        parsed = parse_lidar_config(data)
                        print("---- LiDAR Configuration ----")
                        for k, v in parsed.items():
                            print(f"{k:25}: {v}")
                        print("-----------------------------\n")
                        recvd_data = data  # Save raw data for return
                    except Exception as e:
                        print(f"Error parsing config: {e}")

                elif cmd_byte == 0x81 and mode == "write":
                    print("Write ACK received — configuration updated successfully.")

            else:
                print(f"Non-matching packet from {addr}: {data[:20]!r}")
    finally:
        sock.close()
    
    if return_data:
        return recvd_data

if __name__ == "__main__":
    print("=== Ping Discovery ===")
    mac_list = send_discovery_ping()

    if not mac_list:
        print("No LiDAR detected. Exiting.")
    else:
        lidar_mac_str = mac_list[0]
        print(f"\n=== Sending Read Request to {lidar_mac_str} ===")
        send_request(lidar_mac_str, mode="read")

        print(f"\n=== Sending Write Request (Change Subnet Mask) ===")
        send_request(lidar_mac_str, mode="write")

        print(f"\n=== Verifying Change (Re-read Configuration) ===")
        send_request(lidar_mac_str, mode="read")
