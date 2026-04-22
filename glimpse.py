import socket
import struct
from datetime import datetime, timezone

# --- Updated Configuration for 16.368 MHz / 35-byte Header ---
# Format: < (LE) B(type) I(fs) I(unix) I(tick) I(seq) 16s(tag) H(len)
HEADER_FORMAT = "<BIIII16sH"
HEADER_SIZE = 35
EXPECTED_SIZE = 1058  # 35 (Hdr) + 1023 (Payload)
CAPTURE_LIMIT = 80
FILENAME = "cap.raw"

RELAY_IP = "127.0.0.1"
RELAY_PORT = 12345

# 16.368 MHz math
# 1ms = 16,368 samples. Phase 0 happens every 16,368 ticks.
SAMPLES_PER_MS = 16368 
SAMPLES_PER_SEC = 16368000

payload_size = EXPECTED_SIZE - HEADER_SIZE
total_target_bytes = CAPTURE_LIMIT * payload_size

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.5)
sock.bind(('', 0))
sock.sendto(struct.pack("<I", 0x4A4F494E), (RELAY_IP, RELAY_PORT))

print(f"Targeting {CAPTURE_LIMIT} packets ({total_target_bytes} bytes) -> {FILENAME}")
print(f"Hunting for Phase 0 alignment (1ms = {SAMPLES_PER_MS} samples)...")
print("")

# Table Header
print(f"{'Sequence':>10} | {'ISO8601 Timestamp':>20} | {'Tick':>12} | {'Phase':>6}")
print("-" * 10 + "-+-" + "-" * 20 + "-+-" + "-" * 12 + "-+-" + "-" * 6)

captured_count = 0
aligned = False

try:
    with open(FILENAME, "wb") as f:
        while captured_count < CAPTURE_LIMIT:
            try:
                data, addr = sock.recvfrom(2048)
            except socket.timeout:
                continue
            
            if len(data) != EXPECTED_SIZE:
                # Useful for debugging if collector isn't sending new size yet
                if len(data) == 1052:
                    print(f"\r[!] Error: Receiving old packet size (1052). Recompile collector!", end='')
                continue
            
            # Unpack the new header
            pkt_type, fs, unix_t, tick, seq, dev_tag, p_len = struct.unpack(HEADER_FORMAT, data[:HEADER_SIZE])
            
            # New Phase calculation (modulo 16,368 samples)
            current_phase = tick % SAMPLES_PER_MS
            
            if not aligned:
                print(f"  [Waiting... Type: {pkt_type} Phase: {current_phase} Fs: {fs/1e6:.1f}M]", end='\r')
                
                # Check for Phase 0 alignment
                if (pkt_type == 1 or pkt_type == 49) and current_phase == 0:
                    aligned = True
                    dev = dev_tag.decode('ascii', errors='ignore').strip('\x00')
                    print(" " * 65, end='\r') 
                    print(f"[*] LOCKED: Phase 0 | Device: {dev} | Rate: {fs/1e6:.3f} MHz")
                else:
                    continue

            # Write raw payload
            f.write(data[HEADER_SIZE:])
            captured_count += 1
            
            # Timestamp calculation
            # ms = How many milliseconds into the current second
            ms = (tick % SAMPLES_PER_SEC) // SAMPLES_PER_MS
            ts = datetime.fromtimestamp(unix_t, tz=timezone.utc).strftime('%H:%M:%S')
            time_str = f"{ts}.{ms:03d}Z"
            
            print(f"{seq:10d} | {time_str:>20} | {tick:12d} | {current_phase:6d}")

except KeyboardInterrupt:
    print("\n\nCapture stopped.")

actual_bytes = captured_count * payload_size
print("-" * 57)
if captured_count > 0:
    print(f"Success: {captured_count} packets captured.")
    print(f"Written: {actual_bytes} bytes to {FILENAME}.")
else:
    print("\nAlignment not found. Check if collector is sending Phase 0.")