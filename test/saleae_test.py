import argparse
import csv
import os
import random
import tempfile
import time

import numpy as np
from saleae import automation
import serial

_TX_CH = 0
_RX_CH = 1


def analyze_capture(filename):
  tx_data = []
  rx_data = []
  tx_timestamps = []
  rx_timestamps = []

  with open(filename, 'r', newline='') as f:
    reader = csv.DictReader(f)
    for row in reader:
      if row['name'] == 'TX':
        tx_data.append(int(row['data'], base=16))
        tx_timestamps.append(float(row['start_time']))
      if row['name'] == 'RX':
        rx_data.append(int(row['data'], base=16))
        rx_timestamps.append(float(row['start_time']))

    tx_data = np.array(tx_data)
    rx_data = np.array(rx_data)
    tx_timestamps = np.array(tx_timestamps)
    rx_timestamps = np.array(rx_timestamps)

  duration = np.max(tx_timestamps) - np.min(tx_timestamps)
  tx_bytes = tx_data.size
  rx_bytes = rx_data.size
  kbps = tx_bytes * 10.0 / duration / 1000.0

  min_len = min(tx_data.size, rx_data.size)
  missing_bytes = tx_data.size - rx_data.size
  mismatch_errors = np.count_nonzero(tx_data[:min_len] != rx_data[:min_len])

  return {
      'tx_bytes': tx_bytes,
      'rx_bytes': rx_bytes,
      'kbps': kbps,
      'mismatch_errors': mismatch_errors,
      'missing_bytes': missing_bytes,
  }


def main():
  parser = argparse.ArgumentParser('saleae_test',
                                   description='Test latency and accuracy of RF link.')
  parser.add_argument('-p', '--port', required=True, help='Serial device.')
  parser.add_argument('-b', '--baud', type=int, default=921600, help='Baud rate.')
  parser.add_argument('-d', '--duration', type=float, default=3.0, help='Duration of test.')
  parser.add_argument('--max_interval', type=float, default=0.01,
                      help='Maximum interval between byte bursts.')
  parser.add_argument('--max_size', type=int, default=1000, help='Maximum bursts size in bytes.')
  parser.add_argument('--kbps', type=float,
                      help='Average kilobits per second. This overrides --max_interval.')
  parser.add_argument('--pause', action='store_true', help='Pause before closing capture.')

  args = parser.parse_args()
  if args.kbps is not None:
    avg_packet_bits = args.max_size * 10.0 / 2.0
    args.max_interval = 2.0 * avg_packet_bits / (1000.0 * args.kbps)

  with automation.Manager.connect() as manager:
    device_config = automation.LogicDeviceConfiguration(
        enabled_digital_channels=[_TX_CH, _RX_CH],
        digital_sample_rate=10_000_000,
    )

    with manager.start_capture(device_configuration=device_config) as capture, \
        serial.Serial(args.port, args.baud) as ser:

      print(f'Running test for {args.duration}s...')
      try:
        byte_count = 0
        alarm_time = time.time() + args.duration
        now = time.time()
        while now < alarm_time:
          length = random.randrange(1, args.max_size)
          output = bytes((x % 256 for x in range(byte_count, byte_count + length)))

          assert (ser.write(output) == length)
          byte_count += length

          interval = random.uniform(0, args.max_interval)
          delay = max(now + interval - time.time(), 0)
          time.sleep(delay)
          now = time.time()
      except KeyboardInterrupt:
        print('\nReceived CTRL-C. Stopping test...')

      # Allow serial output to flush.
      time.sleep(0.05)

      print('Stopping capture...')
      capture.stop()

      print('Adding UART analyzers...')
      tx_analyzer = capture.add_analyzer(
          'Async Serial', settings={
              'Input Channel': _TX_CH,
              'Bit Rate (Bits/s)': args.baud,
          }, label='TX')

      rx_analyzer = capture.add_analyzer(
          'Async Serial', settings={
              'Input Channel': _RX_CH,
              'Bit Rate (Bits/s)': args.baud,
          }, label='RX')

      analyzers = [
          automation.DataTableExportConfiguration(tx_analyzer, automation.RadixType.HEXADECIMAL),
          automation.DataTableExportConfiguration(rx_analyzer, automation.RadixType.HEXADECIMAL),
      ]

      with tempfile.TemporaryDirectory() as temp_dir:
        print('Saving data table to temporary file...')
        path = os.path.join(temp_dir, 'data.csv')
        capture.export_data_table(filepath=path, analyzers=analyzers, columns=['Start', 'data'])

        print('Analyzing data table...')
        analysis = analyze_capture(path)

      print('')
      print(f'Total TX Bytes: {analysis["tx_bytes"]}')
      print(f'Total RX Bytes: {analysis["rx_bytes"]}')
      print(f'Average kbps: {analysis["kbps"]:.1f}')
      print(f'Match Errors: {analysis["mismatch_errors"]}')
      print(f'Missing Bytes: {analysis["missing_bytes"]}')

      if args.pause:
        print('')
        input('Press enter to close capture.')


if __name__ == '__main__':
  main()