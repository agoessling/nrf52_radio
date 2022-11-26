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
_DEBUG_CH1 = 2
_DEBUG_CH2 = 3
_DEBUG_CH3 = 4
_DEBUG_CH4 = 5
_SYNC_BYTE = 0xAA
_SYNC_WORD = 0xAAAAAAAA


def round_up(size: int, alignment: int) -> bytearray:
  return (size + alignment - 1) // alignment * alignment


def round_down(size: int, alignment: int) -> bytearray:
  return size // alignment * alignment


def get_test_bytes(size: int, start_count: int):
  size = round_up(size, 8)

  buf = bytearray(size)
  count = start_count

  for pkt_count in range(size // 8):
    for i in range(4):
      buf[8 * pkt_count + i] = _SYNC_BYTE

    for i in range(4):
      index = 8 * pkt_count + 7 - i
      buf[index] = (count >> (8 * i)) & 0xFF

      if buf[index] == _SYNC_BYTE:
        buf[index] += 1
        count += 1 << (8 * i)

    count += 1

  return buf, count


def get_stats(tx_data: np.ndarray, rx_data: np.ndarray, tx_timestamps: np.ndarray,
              rx_timestamps: np.ndarray):
  assert tx_data.size == tx_timestamps.size and rx_data.size == rx_timestamps.size

  tx_words = tx_data.view('>u4')
  assert np.all(tx_words[0::2] == _SYNC_WORD)

  tx_pkt_vals = tx_words[1::2].copy()
  assert np.all(tx_pkt_vals.view('u1') != _SYNC_BYTE)
  assert np.min(np.diff(tx_pkt_vals.view('>i4'))) > 0

  # Timestamp of least significant byte.
  tx_pkt_timestamps = tx_timestamps[7::8]

  rx_pkt_vals = np.empty(rx_data.size // 8, dtype='>u4')
  rx_pkt_timestamps = np.empty(rx_data.size // 8)

  rx_pkt_index = 0
  rx_index = 0
  while rx_index < rx_data.size - 7:
    if rx_data[rx_index:rx_index + 4].view('>u4') != _SYNC_WORD:
      rx_index += 1
      continue

    if np.any(rx_data[rx_index + 4:rx_index + 8] == _SYNC_BYTE):
      rx_index += 1
      continue

    rx_pkt_vals[rx_pkt_index] = rx_data[rx_index + 4:rx_index + 8].view('>u4')[0]
    rx_pkt_timestamps[rx_pkt_index] = rx_timestamps[rx_index + 7]

    rx_pkt_index += 1
    rx_index += 8

  rx_pkt_vals = rx_pkt_vals[:rx_pkt_index]
  rx_pkt_timestamps = rx_pkt_timestamps[:rx_pkt_index]

  wrong_order_inds = np.empty(rx_pkt_vals.size, dtype='?')
  wrong_order_inds[0] = False
  wrong_order_inds[1:] = np.diff(rx_pkt_vals.view('>i4')) <= 0

  map_inds = np.minimum(np.searchsorted(tx_pkt_vals, rx_pkt_vals), tx_pkt_vals.size - 1)
  no_match_inds = tx_pkt_vals[map_inds] != rx_pkt_vals

  latencies = rx_pkt_timestamps - tx_pkt_timestamps[map_inds]
  neg_latency_inds = ~no_match_inds & (latencies <= 0)

  valid_inds = ~(wrong_order_inds | no_match_inds | neg_latency_inds)
  valid_pkts = np.count_nonzero(valid_inds)

  return {
      'tx_pkt_timestamps': tx_pkt_timestamps[map_inds][valid_inds],
      'rx_pkt_timestamps': rx_pkt_timestamps[valid_inds],
      'latencies': latencies[valid_inds],
      'total_pkts': tx_pkt_vals.size,
      'dropped_pkts': tx_pkt_vals.size - valid_pkts,
      'wrong_order_pkts': np.count_nonzero(wrong_order_inds),
      'no_match_pkts': np.count_nonzero(no_match_inds),
      'neg_latency_pkts': np.count_nonzero(neg_latency_inds),
      '_tx_pkt_timestamps_raw': tx_pkt_timestamps[map_inds],
      '_rx_pkt_timestamps_raw': rx_pkt_timestamps,
      '_wrong_order_inds': wrong_order_inds,
      '_no_match_inds': no_match_inds,
      '_neg_latency_inds': neg_latency_inds,
  }


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

  tx_data = np.array(tx_data, dtype=np.uint8)
  rx_data = np.array(rx_data, dtype=np.uint8)
  tx_timestamps = np.array(tx_timestamps)
  rx_timestamps = np.array(rx_timestamps)

  duration = np.max(tx_timestamps) - np.min(tx_timestamps)
  tx_bytes = tx_data.size
  rx_bytes = rx_data.size
  kbps = tx_bytes * 10.0 / duration / 1000.0
  byte_loss = (tx_bytes - rx_bytes) / max(tx_bytes, 1)

  stats = get_stats(tx_data, rx_data, tx_timestamps, rx_timestamps)
  pkt_loss = stats['dropped_pkts'] / max(stats['total_pkts'], 1)

  if stats['latencies'].size == 0:
    max_latency = None
    avg_latency = None
    max_latency_timestamp = None
  else:
    max_latency_arg = np.argmax(stats['latencies'])
    max_latency = stats['latencies'][max_latency_arg]
    avg_latency = np.mean(stats['latencies'])
    max_latency_timestamp = stats['tx_pkt_timestamps'][max_latency_arg]

  return {
      'kbps': kbps,
      'tx_bytes': tx_bytes,
      'rx_bytes': rx_bytes,
      'byte_loss': byte_loss,
      'pkt_loss': pkt_loss,
      'avg_latency': avg_latency,
      'max_latency': max_latency,
      'max_latency_timestamp': max_latency_timestamp,
      'stats': stats,
  }


def array_str(arr, format_str='{}', start=0, stop=10):
  s = '['

  if start != 0:
    s += '..., '

  num_items = min(len(arr), stop)
  for i in range(start, num_items):
    s += format_str.format(arr[i])
    if i < num_items - 1:
      s += ', '

  if num_items < len(arr):
    s += ', ...'

  s += ']'

  return s


def main():
  parser = argparse.ArgumentParser('saleae_test',
                                   description='Test latency and accuracy of RF link.')
  parser.add_argument('-p', '--port', required=True, help='Serial device.')
  parser.add_argument('-b', '--baud', type=int, default=921600, help='Baud rate.')
  parser.add_argument('-d', '--duration', type=float, default=3.0, help='Duration of test.')
  parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output.')
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
        enabled_digital_channels=[_TX_CH, _RX_CH, _DEBUG_CH1, _DEBUG_CH2, _DEBUG_CH3, _DEBUG_CH4],
        digital_sample_rate=10_000_000,
    )

    with manager.start_capture(device_configuration=device_config) as capture, \
        serial.Serial(args.port, args.baud) as ser:

      print(f'Running test for {args.duration}s...')
      try:
        index = 0
        alarm_time = time.time() + args.duration
        now = time.time()
        while now < alarm_time:
          length = random.randrange(1, args.max_size)
          output, index = get_test_bytes(length, index)

          assert (ser.write(output) == len(output))

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

      stats = analysis["stats"]

      print('')
      print(f'Average kbps: {analysis["kbps"]:.1f}')
      print('')
      print(f'Total TX Bytes: {analysis["tx_bytes"]}')
      print(f'Total RX Bytes: {analysis["rx_bytes"]}')
      print('')
      print(f'Total Packets: {stats["total_pkts"]}')
      print(f'Dropped Packets: {stats["dropped_pkts"]}')
      print('')
      print(f'Byte Loss: {100 * analysis["byte_loss"]:.1f}%')
      print(f'Packet Loss: {100 * analysis["pkt_loss"]:.1f}%')
      print('')
      print(f'Average Latency: {1e3 * analysis["avg_latency"]:.2f} ms')
      print(f'Max Latency: {1e3 * analysis["max_latency"]:.2f} ms')
      print(f'Max latency TX timestamp: {analysis["max_latency_timestamp"]:.4f} s')

      if args.verbose:
        print('')
        print(f'Wrong Order Packets: {stats["wrong_order_pkts"]} ', end='')
        print(
            array_str(stats["_rx_pkt_timestamps_raw"][stats["_wrong_order_inds"]], '{:.4f}',
                      stop=5))
        print(f'No Match Packets: {stats["no_match_pkts"]} ', end='')
        print(array_str(stats["_rx_pkt_timestamps_raw"][stats["_no_match_inds"]], '{:.4f}', stop=5))
        print(f'Negative Latency Packets: {stats["neg_latency_pkts"]} ', end='')
        print(
            array_str(stats["_rx_pkt_timestamps_raw"][stats["_neg_latency_inds"]], '{:.4f}',
                      stop=5))

      if args.pause:
        print('')
        input('Press enter to close capture.')


if __name__ == '__main__':
  main()