#!/usr/bin/env python3
# LeCrunch3
# Copyright (C) 2021 Nicola Minafra
#               2025 Laurent Forthomme
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

import struct
import socket
import numpy as np
from LeCrunch3 import LeCrunch3
from typing import Any

from constellation.core.base import EPILOG
from constellation.core.configuration import Configuration
from constellation.core.datasender import DataSender, DataSenderArgumentParser
from constellation.core.logging import setup_cli_logging


class LeCrunchSatellite(DataSender):
    _scope = None
    _settings = None
    _channels = None
    _num_sequences = 1
    _sequence_mode = False

    def do_initializing(self, configuration: Configuration) -> str:
        self.log.info("Received configuration with parameters: %s", ', '.join(configuration.get_keys()))

        ip_address = configuration["ip_address"]
        port = configuration.setdefault("port", 1861)
        timeout = configuration.setdefault("timeout", 5.0)
        self._num_sequences = configuration.setdefault("nsequence", 1)
        #metrics_poll_interval = configuration["metrics_poll_interval"]

        try:
            self._scope = LeCrunch3(str(ip_address), port=int(port), timeout=float(timeout))
            self._scope.clear()
        except ConnectionRefusedError as e:
            self.log.error(f'Connection refused to {ip_address}:{port} -> {str(e)}')
            return ''

        if self._num_sequences > 0:
            self._scope.set_sequence_mode(self._num_sequences)
            self._sequence_mode = True

        self._channels = self._scope.get_channels()
        self._settings = self._scope.get_settings()
        channel_offsets = {}
        channel_trigger_levels = {}
        for key, value in self._settings.items():
            if ':OFFSET' in key:
                channel_offsets[key.split(':')[0].replace('C', '')] = float(value.split(b' ')[1])
            elif ':TRIG_LEVEL' in key:
                channel_trigger_levels[key.split(':')[0].replace('C', '')] = float(value.split(b' ')[1])

        if b'ON' in self._settings['SEQUENCE']:  # waveforms sequencing enabled
            sequence_count = int(self._settings['SEQUENCE'].split(b',')[1])
            self.log.info(f"Configured scope with sequence count = {sequence_count}")
            if self._num_sequences != sequence_count:  # sanity check
                self.log.error(f'Could not configure sequence mode properly: num_sequences={self._num_sequences} != sequences_count={sequence_count}')
        if self._num_sequences != 1:
            self.log.info(f'Using sequence mode with {self._num_sequences} traces per aquisition')

        self.BOR['trigger_delay'] = float(self._settings['TRIG_DELAY'].split(b' ')[1])
        self.BOR['sampling_period'] = float(self._settings['TIME_DIV'].split(b' ')[1])
        self.BOR['channels'] = ','.join([str(c) for c in self._channels])
        self.BOR['num_sequences'] = self._num_sequences
        # configure metrics sending
        #self._configure_monitoring(metrics_poll_interval)

        print('settings:\n', self._settings)

        return f"Connected to scope at {ip_address}"

    def do_run(self, payload: Any) -> str:
        num_sequences_acquired = 0
        num_events_acquired = 0
        while not self._state_thread_evt.is_set():
            try:
                self._scope.trigger()
                #waveforms = []
                #for channel in self._channels:
                #    wave_desc, trg_times, trg_offsets, wave_array = self._scope.get_waveform_all(channel)
                #    wave_array = wave_desc['vertical_offset'] + wave_array * wave_desc['vertical_gain']  # already transform to V
                #    waveforms.append(wave_array)
                #waveforms = np.stack(waveforms, axis=0)
                #if self._num_sequences > 1:
                #    waveforms = np.transpose(waveforms.reshape((len(self._channels), waveforms.size//self._num_sequences//len(self._channels), self._num_sequences)), (2, 0, 1))
                #self.data_queue.put((waveforms.tobytes(), {"dtype": f"{waveforms.dtype}"}))
                first_channel = True
                for channel in self._channels:
                    wave_desc, trg_times, trg_offsets, wave_array = self._scope.get_waveform_all(channel)
                    if first_channel:
                        trigger_header = trg_times
                        self.data_queue.put((trigger_header.tobytes(), {'dtype': f'{trigger_header.dtype}'}))
                        first_channel = False
                    num_samples = wave_desc['wave_array_count']//self._num_sequences
                    wave_array = wave_desc['vertical_offset'] + wave_array * wave_desc['vertical_gain']  # already transform to V
                    channel_header = trg_offsets
                    channel_header = np.append(channel_header, num_samples)
                    self.data_queue.put((channel_header.tobytes(), {'dtype': f'{channel_header.dtype}'}))
                    self.data_queue.put((wave_array.tobytes(), {'dtype': f'{wave_array.dtype}'}))
            except (socket.error, struct.error) as e:
                self.log.error(str(e))
                self._scope.clear()
                continue
            num_events_acquired += self._num_sequences
            num_sequences_acquired += 1
            self.log.info(f'Fetched event {num_events_acquired}/sequence {num_sequences_acquired}')

        return "Finised acquisition"

def main(args: Any = None) -> None:
    parser = DataSenderArgumentParser(description=main.__doc__, epilog=EPILOG)
    args = vars(parser.parse_args(args))

    # set up logging
    setup_cli_logging(args.pop("log_level"))

    # start server with remaining args
    s = LeCrunchSatellite(**args)
    s.run_satellite()


if __name__ == "__main__":
    main()

