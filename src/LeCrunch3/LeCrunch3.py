# LeCrunch3
# Copyright (C) 2021 Nicola Minafra
#
# based on
#
# LeCrunch2
# Copyright (C) 2014 Benjamin Land
#
# based on
#
# LeCrunch
# Copyright (C) 2010 Anthony LaTorre
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import sys
import re
import array
import struct
import numpy as np
import socket
import os
import io

# data types in lecroy binary blocks, where:
# length  -- byte length of type
# string  -- string representation of type
# packfmt -- format string for struct.unpack()
class String:
    length = 16
    string = 'string'
class Byte:
    length = 1
    string = 'byte'
    packfmt = 'b'
class Word:
    length = 2
    string = 'word'
    packfmt = 'h'
class Long:
    length = 4
    string = 'long'
    packfmt = 'l'
class Enum:
    length = 2
    string = 'enum'
    packfmt = 'h'
class Float:
    length = 4
    string = 'float'
    packfmt = 'f'
class Double:
    length = 8
    string = 'double'
    packfmt = 'd'
class TimeStamp:
    length = 16
    string = 'time_stamp'
    packfmt = 'dbbbbhh'
class UnitDefinition:
    length = 48
    string = 'unit_definition'

# all commands to be querried as scope settings
setting_commands = ['TIME_DIV', 'COMM_FORMAT', 'COMM_HEADER', 'COMM_ORDER'] + \
    ['TRIG_DELAY', 'TRIG_SELECT', 'TRIG_MODE', 'TRIG_PATTERN', 'SEQUENCE'] + \
    ['C%i:COUPLING' % i for i in range(1,5)] + \
    ['C%i:VOLT_DIV' % i for i in range(1,5)] + \
    ['C%i:OFFSET' % i for i in range(1,5)] + \
    ['C%i:TRIG_COUPLING' % i for i in range(1,5)] + \
    ['C%i:TRIG_LEVEL' % i for i in range(1,5)] + \
    ['C%i:TRIG_SLOPE' % i for i in range(1,5)] + \
    ['C%i:TRACE' % i for i in range(1,5)]

# byte length of wavedesc block
wavedesclength = 346

# template of wavedesc block, where each entry in tuple is:
# (variable name, byte position from beginning of block, datatype)
wavedesc_template = ( ('descriptor_name'    , 0   , String),
                      ('template_name'      , 16  , String),
                      ('comm_type'          , 32  , Enum),
                      ('comm_order'         , 34  , Enum),
                      ('wave_descriptor'    , 36  , Long),
                      ('user_text'          , 40  , Long),
                      ('res_desc1'          , 44  , Long),
                      ('trigtime_array'     , 48  , Long),
                      ('ris_time_array'     , 52  , Long),
                      ('res_array1'         , 56  , Long),
                      ('wave_array_1'       , 60  , Long),
                      ('wave_array_2'       , 64  , Long),
                      ('res_array_2'        , 68  , Long),
                      ('res_array_3'        , 72  , Long),
                      ('instrument_name'    , 76  , String),
                      ('instrument_number'  , 92  , Long),
                      ('trace_label'        , 96  , String),
                      ('reserved1'          , 112 , Word),
                      ('reserved2'          , 114 , Word),
                      ('wave_array_count'   , 116 , Long),
                      ('pnts_per_screen'    , 120 , Long),
                      ('first_valid_pnt'    , 124 , Long),
                      ('last_valid_pnt'     , 128 , Long),
                      ('first_point'        , 132 , Long),
                      ('sparsing_factor'    , 136 , Long),
                      ('segment_index'      , 140 , Long),
                      ('subarray_count'     , 144 , Long),
                      ('sweeps_per_acq'     , 148 , Long),
                      ('points_per_pair'    , 152 , Word),
                      ('pair_offset'        , 154 , Word),
                      ('vertical_gain'      , 156 , Float),
                      ('vertical_offset'    , 160 , Float),
                      ('max_value'          , 164 , Float),
                      ('min_value'          , 168 , Float),
                      ('nominal_bits'       , 172 , Word),
                      ('nom_subarray_count' , 174 , Word),
                      ('horiz_interval'     , 176 , Float),
                      ('horiz_offset'       , 180 , Double),
                      ('pixel_offset'       , 188 , Double),
                      ('vertunit'           , 196 , UnitDefinition),
                      ('horunit'            , 244 , UnitDefinition),
                      ('horiz_uncertainty'  , 292 , Float),
                      ('trigger_time'       , 296 , TimeStamp),
                      ('acq_duration'       , 312 , Float),
                      ('record_type'        , 316 , Enum),
                      ('processing_done'    , 318 , Enum),
                      ('reserved5'          , 320 , Word),
                      ('ris_sweeps'         , 322 , Word),
                      ('timebase'           , 324 , Enum),
                      ('vert_coupling'      , 326 , Enum),
                      ('probe_att'          , 328 , Float),
                      ('fixed_vert_gain'    , 332 , Enum),
                      ('bandwidth_limit'    , 334 , Enum),
                      ('vertical_vernier'   , 336 , Float),
                      ('acq_vert_offset'    , 340 , Float),
                      ('wave_source'        , 344 , Enum) )

headerformat = '>BBBBL'

errors = { 1  : 'unrecognized command/query header',
           2  : 'illegal header path',
           3  : 'illegal number',
           4  : 'illegal number suffix',
           5  : 'unrecognized keyword',
           6  : 'string error',
           7  : 'GET embedded in another message',
           10 : 'arbitrary data block expected',
           11 : 'non-digit character in byte count field of arbitrary data block',
           12 : 'EOI detected during definite length data block transfer',
           13 : 'extra bytes detected during definite length data block transfer' }

class LeCrunch3(object):
    '''
    A class for triggering and fetching waveforms from a LeCroy oscilloscope.
    '''
    def __init__(self,  host, port=1861, timeout=5.0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))
        self.sock.settimeout(timeout)
        self.clear()
        self.send('comm_header short')
        self.check_last_command()
        self.send('comm_format DEF9,BYTE,BIN')
        self.check_last_command()

    def __del__(self):
        self.sock.close()

    def clear(self, timeout=0.5):
        '''
        Clear any bytes in the oscilloscope's output queue by receiving
        packets until the connection blocks for more than `timeout` seconds.
        '''
        t = self.sock.gettimeout()
        self.sock.settimeout(timeout)
        try:
            while True:
                self.sock.recv(4096)
        except socket.timeout:
            pass
        self.sock.settimeout(t)

    def send(self, msg):
        '''
        Format and send the string `msg`.
        '''
        if not msg.endswith('\n'):
            msg = str(msg) + '\n'
        header = struct.pack(headerformat, 129, 1, 1, 0, len(msg))
        self.sock.sendall(header + msg.encode('ascii'))

    def recv(self):
        '''
        Return a message from the scope.
        '''
        reply = b''
        while True:
            header = b''
            while len(header) < 8:
                try:
                    header += self.sock.recv(8 - len(header))
                except socket.timeout:
                    continue
            operation, headerver, seqnum, spare, totalbytes = \
                struct.unpack(headerformat, header)
            buffer = b''
            while len(buffer) < totalbytes:
                try:
                    buffer += self.sock.recv(totalbytes - len(buffer))
                except socket.timeout:
                    continue
            reply += buffer
            if operation % 2:
                break
        return reply

    def check_last_command(self):
        """
        Check that the last command sent was received okay; if not, raise
        an exception with details about the error.
        """
        self.send('cmr?')
        err = int(self.recv().split(b' ')[-1].rstrip(b'\n'))

        if err in errors:
            self.sock.close()
            raise Exception(errors[err])

    def get_settings(self):
        '''
        Captures the current settings of the scope as a dict of Command->Setting.
        '''
        settings = {}
        for command in setting_commands:
            self.send(command + '?')
            settings[command] = self.recv().strip()
            self.check_last_command()
        return settings

    def set_settings(self, settings):
        '''
        Sends a `settings` dict of Command->Setting to the scope.
        '''
        for command, setting in settings.items():
            self.send(setting)
            self.check_last_command()

    def get_channels(self):
        '''
        Returns a list of the active channels on the scope.
        '''
        channels = []
        for i in range(1, 5):
            self.send('c%i:trace?' %i)
            if b'ON' in self.recv():
                channels.append(i)
        return channels

    def trigger(self):
        '''
        Arms the oscilliscope and instructs it to wait before processing
        further commands, i.e. nonblocking.
        '''
        self.send('arm;wait')

    def set_sequence_mode(self, nsequence):
        '''
        Sets the scope to use sequence mode for aquisition.
        '''
        if nsequence == 1:
            self.send('seq off')
        else:
            self.send('seq on,%i'%nsequence)

    def get_wavedesc_frombuffer(self, msg):
        '''
        Interpret wavedescription data
        '''
        data = io.BytesIO(msg)
        searchResult = re.search(b'WAVEDESC', data.read())
        if searchResult is not None:
            startpos = searchResult.start()
        else:
            startpos = 22

        wavedesc = {}

        # check endian
        data.seek(startpos + 34)
        if struct.unpack('<'+Enum.packfmt, data.read(Enum.length)) == 0:
            endian = '>'
            wavedesc['little_endian'] = True
            np.little_endian = True
        else:
            endian = '<'
            wavedesc['little_endian'] = False
            np.little_endian = False
        data.seek(startpos)

        # build dictionary of wave description
        for name, pos, datatype in wavedesc_template:
            raw = data.read(datatype.length)
            if datatype in (String, UnitDefinition):
                wavedesc[name] = raw.rstrip(b'\x00')
            elif datatype in (TimeStamp,):
                wavedesc[name] = struct.unpack(endian+datatype.packfmt, raw)
            else:
                wavedesc[name] = struct.unpack(endian+datatype.packfmt, raw)[0]

        # determine data type
        if wavedesc['comm_type'] == 0:
            wavedesc['dtype'] = np.int8()
        elif wavedesc['comm_type'] == 1:
            wavedesc['dtype'] = np.int16()
        else:
            raise Exception('unknown comm_type.')

        return wavedesc, data.tell()


    def get_wavedesc(self, channel):
        '''
        Requests the wave descriptor for `channel` from the scope. Returns it in
        dictionary format.
        '''
        if channel not in range(1, 5):
            raise Exception('channel must be in %s.' % str(range(1, 5)))

        self.send('c%s:wf? desc' % str(channel))
        msg = self.recv()
        descr, _ = self.get_wavedesc_frombuffer(msg)
        return descr

    def get_waveform_all(self, channel):
        if channel not in range(1, 5):
            raise Exception('channel must be in %s.' % str(range(1, 5)))

        self.send('c%s:wf? all' % str(channel))

        msg = self.recv()

        if not int(chr(msg[1])) == channel:
            raise RuntimeError('waveforms out of sync or comm_header is off.')

        wavedesc, offset = self.get_wavedesc_frombuffer(msg)
        data = io.BytesIO(msg)
        data.seek(offset)

        # Read trigger data
        trigger_bytes = wavedesc['trigtime_array']
        typeSize = np.dtype('f8').itemsize
        trigger_count = trigger_bytes//typeSize//2
        trigger_times = np.zeros(trigger_count, 'f8')
        trigger_offsets = np.zeros(trigger_count, 'f8')
        for i in range(trigger_count):
            trigger_times[i] = np.frombuffer(data.read(typeSize), 'f8', 1)
            trigger_offsets[i] = np.frombuffer(data.read(typeSize), 'f8', 1)

        # Read waveform
        # TODO: implement dat2 concat
        wave_array_count = wavedesc['wave_array_1']
        if wavedesc['wave_array_2'] != 0:
            print(f'WARNING: waveform too long, ignoring {wavedesc["wave_array_2"]} samples!')

        waveform = np.frombuffer(data.read(wave_array_count*(wavedesc['dtype'].itemsize)), wavedesc['dtype'], wave_array_count//wavedesc['dtype'].itemsize)

        return (wavedesc, trigger_times, trigger_offsets, waveform)


    def get_waveform(self, channel):
        '''
        Capture the raw data for `channel` from the scope and return a tuple
        containing the wave descriptor and a numpy array of the digitized
        scope readout.
        '''
        if channel not in range(1, 9):
            raise Exception('channel must be in %s.' % str(range(1, 9)))
        self.send('c%s:wf? dat1' % str(channel))
        msg = self.recv()
        if not int(chr(msg[1])) == channel:
            raise RuntimeError('waveforms out of sync or comm_header is off.')

        wavedesc = self.get_wavedesc(channel)

        return (wavedesc, np.frombuffer(msg[22:], wavedesc['dtype'], wavedesc['wave_array_1']))
