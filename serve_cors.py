#!/usr/bin/env python3
"""CORS-enabled file server + JSON API for the Tempest dashboard."""
import http.server, json, re, subprocess, os, sys

ROOT = os.path.dirname(os.path.abspath(__file__))
SHOOTING_MAPS = os.path.join(
    ROOT, 'src/main/java/frc/robot/subsystems/shooter/interpolatingTreeMaps/ShootingMaps.java'
)

def parse_shooting_maps():
    with open(SHOOTING_MAPS) as f:
        src = f.read()
    # strip commented-out lines
    lines = [l for l in src.splitlines() if not l.strip().startswith('//')]
    code = '\n'.join(lines)

    def extract(map_name, val_pat):
        pts = []
        for m in re.finditer(rf'{map_name}\.put\(([\d.]+),\s*{val_pat}\)', code):
            pts.append([float(m.group(1)), float(m.group(2))])
        return sorted(pts)

    return {
        'hood': extract(r'shotHoodAngleMap',    r'Rotation2d\.fromDegrees\(([\d.]+)\)'),
        'rpm':  extract(r'shotFlywheelSpeedMap', r'([\d.]+)'),
        'tof':  extract(r'timeOfFlightMap',      r'([\d.]+)'),
    }

def git_info():
    try:
        raw = subprocess.check_output(
            ['git', 'log', '-1', '--format=%H|%h|%s|%cd', '--date=format:%Y-%m-%d %H:%M'],
            cwd=ROOT, stderr=subprocess.DEVNULL
        ).decode().strip()
        h, short, msg, date = raw.split('|', 3)
        return {'hash': h, 'short': short, 'message': msg, 'date': date}
    except Exception:
        return {'hash': '', 'short': 'unknown', 'message': '', 'date': ''}

class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=ROOT, **kw)

    def do_GET(self):
        if self.path == '/api/shooting-maps':
            self._json(parse_shooting_maps())
        elif self.path == '/api/git-info':
            self._json(git_info())
        else:
            super().do_GET()

    def _json(self, data):
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        super().end_headers()

    def log_message(self, *_):
        pass

port = int(sys.argv[1]) if len(sys.argv) > 1 else 8081
print(f'Tempest API server → http://localhost:{port}')
http.server.HTTPServer(('', port), Handler).serve_forever()
