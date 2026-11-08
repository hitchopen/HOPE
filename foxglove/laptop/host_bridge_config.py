#!/usr/bin/env python3
"""Render the Host endpoint from the audited control bridge allowlists."""
import argparse
from pathlib import Path
import yaml


def host_config(control):
    config = yaml.safe_load(control)
    params = config['/**']['ros__parameters']
    params['address'] = '127.0.0.1'
    params['port'] = 8767
    topics = params['topic_whitelist']
    if '^/hope/live/.*' not in topics:
        topics.insert(0, '^/hope/live/.*')
    # Stock Host bridge does not include the HDU display-thread patch. Do not
    # pretend these parameters take effect or lower heartbeat/service priority.
    params.pop('display_subscription_nice', None)
    params.pop('display_ball_marker_hz', None)
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('control', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    config = host_config(args.control.read_text())
    args.output.write_text(yaml.safe_dump(config, sort_keys=False))


if __name__ == '__main__':
    main()
