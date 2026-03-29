from dynamixel_sdk import PortHandler, PacketHandler

def write1(addr, val, packet, port, id, comm_success):
    r, e = packet.write1ByteTxRx(port, id, addr, val & 0xFF)
    if r != comm_success or e != 0:
        raise RuntimeError(f"write1 addr {addr} err: {packet.getTxRxResult(r)} / {packet.getRxPacketError(e)}")

def write2(addr, val, packet, port, id, comm_success):
    r, e = packet.write2ByteTxRx(port, id, addr, val & 0xFFFF)
    if r != comm_success or e != 0:
        raise RuntimeError(f"write2 addr {addr} err: {packet.getTxRxResult(r)} / {packet.getRxPacketError(e)}")

def write4(addr, val, packet, port, id, comm_success):
    r, e = packet.write4ByteTxRx(port, id, addr, val & 0xFFFFFFFF)
    if r != comm_success or e != 0:
        raise RuntimeError(f"write4 addr {addr} err: {packet.getTxRxResult(r)} / {packet.getRxPacketError(e)}")

def read1u(addr, packet, port, id, comm_success):
    val, r, e = packet.read1ByteTxRx(port, id, addr)
    if r != comm_success or e != 0:
        raise RuntimeError(f"read1 addr {addr} err: {packet.getTxRxResult(r)} / {packet.getRxPacketError(e)}")
    return val

def read2u(addr, packet, port, id, comm_success):
    val, r, e = packet.read2ByteTxRx(port, id, addr)
    if r != comm_success or e != 0:
        raise RuntimeError(f"read2u addr {addr} err: {packet.getTxRxResult(r)} / {packet.getRxPacketError(e)}")
    return val

def read2s(addr, packet, port, id, comm_success):
    val, r, e = packet.read2ByteTxRx(port, id, addr)
    if r != comm_success or e != 0:
        raise RuntimeError(f"read2s addr {addr} err: {packet.getTxRxResult(r)} / {packet.getRxPacketError(e)}")
    if val > 32767:
        val -= 65536
    return val

def read4s(addr, packet, port, id, comm_success):
    val, r, e = packet.read4ByteTxRx(port, id, addr)
    if r != comm_success or e != 0:
        raise RuntimeError(f"read4s addr {addr} err: {packet.getTxRxResult(r)} / {packet.getRxPacketError(e)}")
    if val > 0x7FFFFFFF:
        val -= 0x100000000
    return val