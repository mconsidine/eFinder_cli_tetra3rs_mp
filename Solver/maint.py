"""
Maintenance socket protocol for eFinder tetra3rs_mp.

Wire format: one JSON object per line, both directions.

Request:  {"cmd": "status", "args": {}}
Response (ok):   {"ok": true,  "result": {...}}
Response (fail): {"ok": false, "error": "..."}

Socket: /run/efinder/maint.sock
"""

import dataclasses
import json
import os
import socket
from typing import Any, Optional

SOCKET_PATH = os.environ.get("EFINDER_MAINT_SOCKET", "/run/efinder/maint.sock")


@dataclasses.dataclass
class MaintRequest:
    cmd: str
    args: dict

    def encode(self) -> bytes:
        return (json.dumps({"cmd": self.cmd, "args": self.args}) + "\n").encode("utf-8")

    @classmethod
    def decode(cls, line: bytes) -> "MaintRequest":
        obj = json.loads(line.decode("utf-8"))
        return cls(cmd=obj.get("cmd", ""), args=obj.get("args", {}))


@dataclasses.dataclass
class MaintResponse:
    ok: bool
    result: Optional[Any] = None
    error: Optional[str] = None

    def encode(self) -> bytes:
        d: dict = {"ok": self.ok}
        if self.ok:
            d["result"] = self.result or {}
        else:
            d["error"] = self.error or "unknown error"
        return (json.dumps(d) + "\n").encode("utf-8")

    @classmethod
    def decode(cls, line: bytes) -> "MaintResponse":
        obj = json.loads(line.decode("utf-8"))
        return cls(
            ok=bool(obj.get("ok", False)),
            result=obj.get("result"),
            error=obj.get("error"),
        )


def call(cmd: str, args: Optional[dict] = None, timeout: float = 5.0) -> MaintResponse:
    """Send a single request to the maintenance socket and return the response."""
    if args is None:
        args = {}
    req = MaintRequest(cmd=cmd, args=args)
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(timeout)
            sock.connect(SOCKET_PATH)
            sock.sendall(req.encode())
            data = b""
            while b"\n" not in data:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                data += chunk
        return MaintResponse.decode(data.split(b"\n")[0])
    except FileNotFoundError:
        return MaintResponse(ok=False, error="efinder not running (socket not found)")
    except Exception as exc:
        return MaintResponse(ok=False, error=str(exc))
