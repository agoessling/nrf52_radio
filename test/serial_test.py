import argparse
import random
import time

import serial


def main():
  parser = argparse.ArgumentParser('serial_test', description='Send data for radio testing.')
  parser.add_argument('-p', '--port', required=True, help='Serial device.')
  parser.add_argument('-b', '--baud', required=True, type=int, help='Baud rate.')
  parser.add_argument('--total_bytes', type=int, default=10000, help='Total bytes for test.')

  args = parser.parse_args()

  with serial.Serial(args.port, args.baud) as ser:
    byte_count = 0
    while byte_count < args.total_bytes:
      len = min(args.total_bytes - byte_count, random.randrange(1, 1000))
      output = random.randbytes(len)
      assert (ser.write(output) == len)
      byte_count += len

      delay = random.uniform(0, 0.1)
      time.sleep(delay)


if __name__ == '__main__':
  main()