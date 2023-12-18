#!/usr/bin/env python3
import argparse

# import proto
import pandas
import matplotlib.pylab as plt
import matplotlib.font_manager as fm


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument('--data_files', nargs='+')
  parser.add_argument('--labels', nargs='+')
  args = parser.parse_args()

  # plt.rcParams.update({
  #   'text.usetex': True,
  #   'font.family': 'serif',
  #   'font.serif': ['Computer Modern Roman'],
  # })

  print(args.data_files)
  print(args.labels)

  series_data = {}
  for label, data_path in zip(args.labels, args.data_files):
    series_data[label] = pandas.read_csv(data_path)

  fig, ax = plt.subplots(1, 1)
  for label, data in series_data.items():
    ax.plot(data["rx"], data["ry"], label=label)

  ax.legend(loc=0)
  ax.set_xlabel("x [m]")
  ax.set_ylabel("y [m]")
  ax.set_aspect('equal', 'box')
  plt.show()
