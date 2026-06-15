from microbit import i2c, sleep, display

ADDR = 0x10

def _read_u8(reg):
    i2c.write(ADDR, bytes([reg]))
    return i2c.read(ADDR, 1)[0]

def _read_u16(reg):
    i2c.write(ADDR, bytes([reg]))
    d = i2c.read(ADDR, 2)
    return (d[0] << 8) | d[1]

def read_all():
    """Read direction and encoder in one burst per side, back to back."""
    dL = _read_u8(0x00)
    eL = _read_u16(0x04)
    dR = _read_u8(0x02)
    eR = _read_u16(0x06)
    return dL, eL, dR, eR

COUNTS_PER_CM = 5.91

dL, eL, dR, eR = read_all()
prevL, prevR = eL, eR
pos_counts = 0

while True:
    sleep(50)   # slightly longer window → more stable deltas

    dL, eL, dR, eR = read_all()

    deltaL = eL - prevL
    deltaR = eR - prevR

    # Only trust a delta if direction is unambiguous (not stopped)
    # and the delta is consistent with actually moving (not a glitch)
    if dL == 1:   pos_counts += deltaL
    elif dL == 2: pos_counts -= deltaL

    if dR == 1:   pos_counts += deltaR
    elif dR == 2: pos_counts -= deltaR

    # Average of both sides
    prevL, prevR = eL, eR

    cm = int((pos_counts / 2) / COUNTS_PER_CM)
    display.scroll(str(cm) + "cm", delay=60)
