# T&C Power Conversion AG 0613 600W RF Generator
# Dave Matthews, Swift Coat Inc, Oct 2020

# TODO: support Python context manager
import queue
import serial
import threading
import time

class TCPowerRFGenrator(object):
    BYTEORDER   = 'big'

    ACK         = 0x2A.to_bytes(1, BYTEORDER)
    NACK        = 0x3F.to_bytes(1, BYTEORDER)

    CMD_HEAD    = 0x43.to_bytes(1, BYTEORDER)
    CMD_ADDR    = 0x01.to_bytes(1, BYTEORDER)
    RSP_HEAD    = 0x52.to_bytes(1, BYTEORDER)

    FMT_FLOAT = '{0:.0f}'

    def __init__(self, min_power, max_power, port, baud=38400, timeout=0.2,
                 line_term='', readonly=False, debug=False):
        # Params
        self.debug = debug
        self.max_power = max_power
        self.min_power = min_power
        self.readonly = readonly

        # Serial communication
        self.baud = baud
        self.line_term = line_term
        self.port = port
        self.timeout = timeout

        # Com thread
        self.tx_queue = queue.SimpleQueue()
        self.rx_queue = queue.SimpleQueue()
        self.running = True
        self.com_thread = threading.Thread(target=self.__com_loop)
        self.com_thread.start()

        # Initialize
        if not readonly:
            # Request control
            success = self.requestControl()
            # Set power limit on hardware (also enforced in setForwardPower)
            if success:
                success = self.setForwardPowerLimit(self.max_power)
            if not success:
                print("WARNING -- RF: Generator initialization failed")
                self.stop()

    def __com_loop(self):
        ''' Serial communication loop
        - Sends commands from self.tx_queue or a heartbeat message if no
        command shows up in tx_queue for over one second.
        - Places responces in self.rx_queue. Response includes data, if
        expected, or just an acknowledgement message if no data expected.
        - tx_queue messages are tuples:
            (bytestring to send, bool whether data is expected)
        - rx_queue messages are tuples:
            (command id, response content)
        - Continues until self.running == False. '''

        # Connect
        try:
            ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        except serial.SerialException as e:
            print("RF: Failed to connect")
            raise e

        # Run send/receive loop until disconnect
        while self.running:
            # Wait one second for a real message, otherwise send heartbeat
            try:
                msg, read = self.tx_queue.get(timeout=1.0)
                heartbeat = False
                if self.debug:
                    print("> " + str(msg))
            except queue.Empty:
                msg, read = self.assemble_cmd(0x4250), False
                heartbeat = True
            ser.write(msg)
            cmd_id = msg[2:4]   # Save command id for response queue

            # Read until finding an acknowledgement
            msg = ser.read(1)
            while msg not in (self.ACK, self.NACK):
                if self.debug:
                    print("< " + str(msg) + " (discarded, waiting for ACK)")
                msg = ser.read(1)

            # Don't pollute the queue if this is just a heartbeat
            if heartbeat:
                continue

            # If no response expected, put ack in queue and continue
            if self.debug:
                print("< " + 'OK' if msg == self.ACK else 'NOT OK')
            if not read or msg != self.ACK:
                self.rx_queue.put((cmd_id, msg))
                continue

            # Read until finding a response byte
            msg = ser.read(1)
            while msg != self.RSP_HEAD:
                if self.debug:
                    print("< " + str(head) + " (discarded, waiting for resp)")
                msg = ser.read(1)
            # Address byte
            msg += ser.read(1)
            # Data length (2 bytes)
            len = ser.read(2)
            msg += len
            # Data (variable length)
            data = ser.read(self.from_bytes(len))
            msg += data
            # Checksum (2 bytes)
            chksum = ser.read(2)
            if sum(msg) != self.from_bytes(chksum):
                print("WARNING -- RF: Bad checksum received, check for errors")

            # Put data in queue
            self.rx_queue.put((cmd_id, data))
            if self.debug:
                print("< ", msg + chksum)

        # Cleanup
        ser.close()

    # Internal methods -----------------------------------------------------
    def from_bytes(self, v):
        ''' Helper function. Returns an int from bytes. '''
        return int.from_bytes(v, self.BYTEORDER)

    def to_bytes(self, v, l):
        ''' Helper function. Returns bytestring from int and length. '''
        return v.to_bytes(l, self.BYTEORDER)

    def assemble_cmd(self, cmd_id, parm1=0, parm2=0):
        ''' Internal function to format commands for sending. '''
        msg = self.CMD_HEAD + self.CMD_ADDR     # Header + address
        msg += self.to_bytes(cmd_id, 2)         # Command ID
        msg += self.to_bytes(parm1, 2)          # Param 1
        msg += self.to_bytes(parm2, 2)          # Param 2
        msg += self.to_bytes(sum(msg), 2)       # Checksum
        return msg

    def send_cmd(self, cmd_id, parm1=0x00, parm2=0x00, read=False):
        ''' Format and send a command, then wait for and return response.
        Returns data, if expected; otherwise returns bool representing
        whether command was acknowledged successfully.'''

        # Assemble and send command
        msg = self.assemble_cmd(cmd_id, parm1, parm2)
        self.tx_queue.put((msg, read))

        # Read receive queue until a response matches the correct id
        id = b'\x00'
        while self.from_bytes(id) != cmd_id:
            try:
                id, val = self.rx_queue.get(timeout=2)
            except queue.Empty:
                print("ERROR -- RF: Failed to get response, command below.")
                print("\t", msg)
                return False

        # Return data if requested, or whether ACK signal was okay otherwise
        if read:
            ret = val
        else:
            ret = val == self.ACK
        return ret

    # Misc control ---------------------------------------------------------
    def stop(self):
        ''' Release control, then set flag to close com loop. '''
        self.releaseControl()
        time.sleep(2)
        self.running = False

    def ping(self):
        ''' Returns true if unit will acknowledge commands. '''
        return self.send_cmd(0x4250)

    def requestControl(self):
        ''' Returns True if control was granted, False otherwise. '''
        status = self.send_cmd(0x4243, 0x5555, read=True)
        return self.from_bytes(status) == 1

    def releaseControl(self):
        ''' Returns True on success. Even if this fails, unit will
        automatically return control if heartbeat stops for 2 seconds. '''
        status = self.send_cmd(0x4243, read=True)
        return self.from_bytes(status) == 0

    def getStatus(self):
        ''' Returns [STATUS] [TEMP] [OPMODE] [TUNER], 2 bytes each '''
        return self.send_cmd(0x4753, read=True)

    def getTunerStatus(self):
        ''' Returns [STATUS] [LC POS] [TC POS] [VDC] [PRESET], 2 bytes each '''
        return self.send_cmd(0x4754, read=True)

    # On/Off ---------------------------------------------------------------
    def rf(self, on=False):
        ''' Turns rf power on/off, returns success either way. '''
        return self.send_cmd(0x4252, 0x5555 if on else 0x0000)

    def turnOn(self):
        ''' Alias for rf '''
        return self.rf(True)

    def turnOff(self):
        ''' Alias for rf '''
        return self.rf(False)

    # Power ----------------------------------------------------------------
    def readPower(self):
        ''' Returns tuple of current (forward, refected, load) power. '''
        ret = self.send_cmd(0x4750, read=True)
        fwd = self.from_bytes(ret[0:2]) / 10
        rev = self.from_bytes(ret[2:4]) / 10
        load = self.from_bytes(ret[4:6]) / 10
        return fwd, rev, load

    def readForwardPower(self):
        ''' Alias of readPower that only returns forward power as a string. '''
        return self.FMT_FLOAT.format(self.readPower()[0])

    def readForwardPowerSetpoint(self):
        ''' Returns current forward power setpoint, not actual power. '''
        setpt = self.from_bytes(self.send_cmd(0x474C, read=True)) / 10
        return self.FMT_FLOAT.format(setpt)

    def readReflectedPower(self):
        ''' Alias of readPower that only returns refl power as a string. '''
        return self.FMT_FLOAT.format(self.readPower()[1])

    def setForwardPower(self, setpoint):
        ''' Set forward power setpoint.
        Enforces min/max power limits from init. '''
        setpoint = min(max(setpoint, self.min_power), self.max_power)
        return self.send_cmd(0x5341, setpoint)

    def setForwardPowerLimit(self, limit):
        ''' Set forward power limit in hardware. '''
        limit = min(max(limit, self.min_power), self.max_power)
        return self.send_cmd(0x5355, 1, limit)

    def setReflectedPowerLimit(self, limit):
        ''' Set reflected power limit in hardware. '''
        return self.send_cmd(0x5355, 2, limit)


if __name__ == '__main__':
    gen = TCPowerRFGenrator(0, 600, 'COM5', debug=True)
    print("\tPing")
    print(gen.ping())
    print("\tSet forward power to 50")
    print(gen.setForwardPower(50))
    print("\tRead forward power")
    print(gen.readForwardPower())
    print("\tRead forward power setpoint")
    print(gen.readForwardPowerSetpoint())
    print("\tSleeping...")
    time.sleep(4)
    print("\tRF ON!")
    print(gen.turnOn())
    print("\tRead forward power")
    print(gen.readForwardPower())
    print("\tRead forward power setpoint")
    print(gen.readForwardPowerSetpoint())
    time.sleep(4)
    print("\tRead forward power")
    print(gen.readForwardPower())
    print("\tRF off")
    print(gen.turnOff())
    print("\tSet forward power to 10")
    print(gen.setForwardPower(10))
    print("\tRead forward power")
    print(gen.readForwardPower())
    print("\tRead forward power setpoint")
    print(gen.readForwardPowerSetpoint())
    time.sleep(4)
    gen.stop()
