import json
import socket
import struct
import threading
import time
from typing import Any, Dict, Optional


SYNC = b"\xeb\x91\xeb\x90"
HEADER_LEN = 16


class M20ProtocolError(RuntimeError):
    pass


class M20TcpClient:
    """Small TCP client for the M20 Pro body-monitoring JSON protocol."""

    def __init__(self, ip: str = "10.21.31.103", port: int = 30001, timeout: float = 2.0):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._msg_id = 0
        self._lock = threading.Lock()

    def connect(self) -> None:
        self.close()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.timeout)
        sock.connect((self.ip, self.port))
        self._sock = sock

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def is_connected(self) -> bool:
        return self._sock is not None

    def request(self, msg_type: int, command: int, items: Optional[Dict[str, Any]] = None,
                wait_response: bool = True, response_timeout: Optional[float] = None) -> Optional[Dict[str, Any]]:
        payload = {
            "PatrolDevice": {
                "Type": msg_type,
                "Command": command,
                "Time": time.strftime("%Y-%m-%d %H:%M:%S"),
                "Items": items or {},
            }
        }
        return self.send(payload, wait_response=wait_response, response_timeout=response_timeout)

    def send(self, payload: Dict[str, Any], wait_response: bool = True,
             response_timeout: Optional[float] = None) -> Optional[Dict[str, Any]]:
        with self._lock:
            if self._sock is None:
                self.connect()
            assert self._sock is not None
            body = json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
            msg_id = self._msg_id
            self._msg_id = (self._msg_id + 1) & 0xFFFF
            header = SYNC + struct.pack("<H", len(body)) + struct.pack("<H", msg_id) + b"\x01" + b"\x00" * 7
            try:
                self._sock.sendall(header + body)
                if not wait_response:
                    return None
                return self._recv(response_timeout)
            except (OSError, M20ProtocolError):
                self.close()
                raise

    def _recv(self, response_timeout: Optional[float]) -> Dict[str, Any]:
        assert self._sock is not None
        old_timeout = self._sock.gettimeout()
        if response_timeout is not None:
            self._sock.settimeout(response_timeout)
        try:
            header = self._read_exact(HEADER_LEN)
            if header[:4] != SYNC:
                raise M20ProtocolError("bad protocol sync header")
            length = struct.unpack("<H", header[4:6])[0]
            body = self._read_exact(length)
            return json.loads(body.decode("utf-8"))
        finally:
            if response_timeout is not None:
                self._sock.settimeout(old_timeout)

    def _read_exact(self, size: int) -> bytes:
        assert self._sock is not None
        chunks = bytearray()
        while len(chunks) < size:
            chunk = self._sock.recv(size - len(chunks))
            if not chunk:
                raise M20ProtocolError("socket closed while reading response")
            chunks.extend(chunk)
        return bytes(chunks)


def patrol_items(response: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    if not response:
        return {}
    return response.get("PatrolDevice", {}).get("Items", {})
