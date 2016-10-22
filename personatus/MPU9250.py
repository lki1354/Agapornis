# MicroPython driver for the IMU sensor MPU9250 from InvenSense
# WHO AM I = 117
# value WHO AM I = 113

from array import array


class MPU9250(object):
    """
    InvenSense Sensor MPU9250
    """
    __address = None
    __address_magnetometer = 12  # mag AK8963
    __interface = None

    def __init__(self, interface, ad0=0):
        """
        initialize the sensor
        :param interface:
        :param ad0:
        """
        if ad0:
            self.__address = 0x69  # 105
        else:
            self.__address = 0x68  # 104
        self.__interface = interface
        self.wake()
        self.bypass_mode = True
        self.gyro_full_scale = 250  # degree/second
        self.accelerometer_full_scale = 2  # g
        # set gyro and accelerometer into full bandwidth and no filtering
        reg_val = self._read_byte(0x1B)
        self._write(0x1B, reg_val | 0x03)
        reg_val = self._read_byte(0x1D)
        self._write(0x1D, reg_val | 0x08)

    def _write(self, memory_address, buffer):
        """

        :param memory_address: memory address from the chip  where to write the data in the buffer
        :param buffer: data which has to be send to the chip
        :return: nothing
        """
        self.__interface.writeto_mem(self.__address, memaddr=memory_address, buf=buffer)

    def _write_mag(self, memory_address, buffer):
        """

        :param memory_address:
        :param buffer:
        :return:
        """
        self.__interface.writeto_mem(self.__address, memaddr=memory_address, buf=buffer)

    def _read(self, memory_address, buffer):
        """

        :param memory_address:
        :return:  number of read bytes
        """
        return self.__interface.readfrom_mem_into(self.__address, memaddr=memory_address, buf=buffer)

    def _read_byte(self, memory_address):
        """

        :param memory_address:
        :return:  the received byte
        """
        return self.__interface.readfrom_mem(self.__address, memaddr=memory_address, nbytes=1)[0]

    def wake(self):
        """
        wake up the chip and use best clock source
        :return: nothing
        """
        self._write(0x6B, 0x01)

    @property
    def bypass_mode(self):
        """

        :return: the actual mode of the chip I2C bypass interface
        """
        mode = self._read_byte(0x37)
        mode >>= 1
        return True if mode else False

    @bypass_mode.setter
    def bypass_mode(self, mode):
        """

        :param mode:
        :return: nothing
        """
        mode = 2 if mode else 0
        mode |= (self._read_byte(0x37) & 0xFD)
        self._write(0x37, mode)

    @property
    def gyro_full_scale(self):
        """

        :return: range value of the gyro sensor (degree/second)
        """
        max_range = self._read_byte(0x1B) & 0x18
        max_range *= 500  # *500 and /8 is *62.5
        max_range //= 8
        return int(max_range)

    @gyro_full_scale.setter
    def gyro_full_scale(self, max_range):
        """

        :param max_range: the max value of the gyro sensor (degree/second) allowed [250, 500, 1000, 2000]
        :return: nothing
        """
        # /500 and *8 is *0.016
        max_range //= 500    # for select right bit
        max_range *= 8      # for shift to right position
        max_range |= (self._read_byte(0x1B) & 0xE7)
        self._write(0x1B, int(max_range))

    @property
    def accelerometer_full_scale(self):
        """

        :return: range value of the gyro sensor (degree/second)
        """
        max_range = self._read_byte(0x1C) & 0x18
        max_range *= 4
        max_range //= 8
        return int(max_range)

    @accelerometer_full_scale.setter
    def accelerometer_full_scale(self, max_range):
        """

        :param max_range: the max value of the gyro sensor (degree/second) allowed [250, 500, 1000, 2000]
        :return: nothing
        """
        max_range //= 4    # for select right bit
        max_range *= 8    # for shift to right position
        max_range |= (self._read_byte(0x1C) & 0xE7)
        self._write(0x1C, int(max_range))

    def how_am_i(self):
        """

        :return: the byte identifier
        """
        return self._read_byte(0x75)

    def data_updated(self):
        """

        :return: True if new data in the register or False if not
        """
        register_value = self._read_byte(0x3A)
        return True if register_value & 0x01 else False

    @property
    def _raw_data(self):
        """

        :return: the bytearray for gyro, accelerometer and temperature sensor
        """
        data = bytearray(14)
        self._read(0x3B, data)  # start to read from acceleration data X register and than 13 more bytes
        return data

    @property
    def data(self):
        """

        :return: a array with calculated data
        """
        raw_data = self._raw_data
        data = array('l')

        data.append(1)
        data.append(1)
        data.append(1)

        data.append(((raw_data[6] << 8) | raw_data[7]) // 3 + 2100)  # 100 deg

        data.append(1)
        data.append(1)
        data.append(1)

        return data
