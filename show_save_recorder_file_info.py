#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import argparse


def main():

    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-f', '--recorder_filename',
        metavar='F',
        default="est1.log",
        help='recorder filename (est1.log)')
    argparser.add_argument(
        '-a', '--show_all',
        action='store_true',
        help='show detailed info about all frames content')
    argparser.add_argument(
        '-s', '--save_to_file',       # 命令行中指定-s C:\Log_output\test5.txt来指定存储路径和文本文件的名称
        metavar='S',
        help='save result to file (specify name and extension)')

    #0416新添
    argparser.add_argument(
        '-i', '--hero_id',
        type=int,
        default=None,
        help='ID of the hero vehicle to filter logs for'
    )

    args = argparser.parse_args()

    try:

        client = carla.Client(args.host, args.port)
        client.set_timeout(60.0)

        #0416新添
        if args.hero_id is not None:
            all_data = client.show_recorder_file_info(args.recorder_filename, args.show_all)
            hero_data = [line for line in all_data.split('\n')if 'Version' in line or 'Map' in line or 'Date' in line or
                         f'Id: {args.hero_id}' in line or 'Frame' in line or f'between {args.hero_id}' in line]  # 命令行中通过指定-i来确定hero的id
            output_data = '\n'.join(hero_data)
        else:
            output_data = client.show_recorder_file_info(args.recorder_filename,args.show_all)


        if args.save_to_file:
                doc = open(args.save_to_file, "w+")
                doc.write(output_data)
                doc.close()
        else:
            # print(client.show_recorder_file_info(args.recorder_filename, args.show_all))    #0416注释掉了
            print(output_data)  #0416新添


    finally:
        pass


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
