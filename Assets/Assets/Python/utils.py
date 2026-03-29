# 換算


def deg2pos(angle_deg):
    '''

    :param angle_deg: float, command
    :return: int, command for motor
    '''
    return int(angle_deg * (4096/360))


def pos2deg(position, extend = False):
    '''
    :param position: int, code for motor
    :return: float, anglr degree
    '''

    if extend:
        return position * 360 / 4096
    return (position % 4096) * 360 / 4096


def c_raw2mA(raw):
    '''
    讀馬達電流換算成 mA
    :param raw: 32-bit unsigned int 從 read4ByteTxRx 拿到
    :return: float, mA (signed)
    '''
    # 轉換成有號值（因為可能會是負數）
    if raw > 32767:
        raw -= 65536
    # 單位：1 = 1 mA
    current_mA = raw * 1
    mA = raw * 1  # 依你硬體換算，如果文件是 0.00269A/bit
    return mA

