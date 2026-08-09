#!/usr/bin/env python3
"""Serve converted Foxglove assets with permissive CORS headers."""

from __future__ import annotations

import argparse
import functools
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class CorsHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, HEAD, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "*")
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(204)
        self.end_headers()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Serve a converted A3 Foxglove output directory for browser Foxglove mesh loading."
    )
    parser.add_argument("directory", help="Converted foxglove output directory")
    parser.add_argument("--host", default="127.0.0.1", help="Bind address. Default: 127.0.0.1")
    parser.add_argument("--port", type=int, default=8765, help="Port. Default: 8765")
    args = parser.parse_args()

    directory = Path(args.directory).resolve()
    if not directory.is_dir():
        raise SystemExit(f"not a directory: {directory}")

    handler = functools.partial(CorsHandler, directory=str(directory))
    server = ThreadingHTTPServer((args.host, args.port), handler)
    url = f"http://{args.host}:{args.port}"
    print(f"[foxglove-assets] serving {directory}")
    print(f"[foxglove-assets] URL base for converter: {url}/assets/a3")
    print("[foxglove-assets] press Ctrl-C to stop")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[foxglove-assets] stopped")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
