#!/usr/bin/env python3
import json
from pathlib import Path
import socket
import struct
import sys
import threading


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "m20pro_navigation"))

from m20pro_navigation.tcp_protocol import (  # noqa: E402
    HEADER_LEN,
    M20TcpClient,
    SYNC,
    patrol_items,
)


def read_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise RuntimeError("socket closed")
        chunks.extend(chunk)
    return bytes(chunks)


def read_request(sock: socket.socket):
    header = read_exact(sock, HEADER_LEN)
    assert header[:4] == SYNC
    length = struct.unpack("<H", header[4:6])[0]
    msg_id = struct.unpack("<H", header[6:8])[0]
    payload = json.loads(read_exact(sock, length).decode("utf-8"))
    return msg_id, payload


def response_frame(msg_id: int, msg_type: int, command: int, items: dict) -> bytes:
    payload = {
        "PatrolDevice": {
            "Type": msg_type,
            "Command": command,
            "Items": items,
        }
    }
    body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    return SYNC + struct.pack("<H", len(body)) + struct.pack("<H", msg_id) + b"\x01" + b"\x00" * 7 + body


def test_async_response_is_not_used_as_status() -> None:
    client_sock, server_sock = socket.socketpair()
    client_sock.settimeout(0.5)
    server_sock.settimeout(0.5)
    client = M20TcpClient(timeout=0.5)
    client._sock = client_sock
    client._msg_id = 100

    def server() -> None:
        axis_id, axis = read_request(server_sock)
        assert axis["PatrolDevice"]["Type"] == 2
        assert axis["PatrolDevice"]["Command"] == 21
        server_sock.sendall(response_frame(axis_id, 2, 21, {"ErrorCode": 0}))

        status_id, status = read_request(server_sock)
        assert status["PatrolDevice"]["Type"] == 2002
        assert status["PatrolDevice"]["Command"] == 1
        server_sock.sendall(
            response_frame(status_id, 2002, 1, {"Location": 0, "ObsState": 0})
        )

    worker = threading.Thread(target=server)
    worker.start()
    try:
        assert client.request(2, 21, {"X": 0.25}, wait_response=False) is None
        response = client.request(2002, 1, {}, response_timeout=0.5)
        assert patrol_items(response) == {"Location": 0, "ObsState": 0}
        assert client.discarded_response_count == 1
    finally:
        worker.join(timeout=1.0)
        client.close()
        server_sock.close()
    assert not worker.is_alive()


def test_matching_response_does_not_increment_discard_count() -> None:
    client_sock, server_sock = socket.socketpair()
    client_sock.settimeout(0.5)
    server_sock.settimeout(0.5)
    client = M20TcpClient(timeout=0.5)
    client._sock = client_sock
    client._msg_id = 500

    def server() -> None:
        msg_id, _payload = read_request(server_sock)
        server_sock.sendall(
            response_frame(
                msg_id,
                1007,
                2,
                {"Location": 0, "PosX": 1.0, "PosY": 2.0},
            )
        )

    worker = threading.Thread(target=server)
    worker.start()
    try:
        response = client.request(1007, 2, {}, response_timeout=0.5)
        assert patrol_items(response)["Location"] == 0
        assert client.discarded_response_count == 0
    finally:
        worker.join(timeout=1.0)
        client.close()
        server_sock.close()
    assert not worker.is_alive()


def main() -> None:
    test_async_response_is_not_used_as_status()
    test_matching_response_does_not_increment_discard_count()
    print("TCP protocol response correlation tests passed")


if __name__ == "__main__":
    main()
