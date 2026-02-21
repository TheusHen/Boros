#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import re
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import TextIO

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
except Exception:
    serial = None
    list_ports = None


BAUD_DEFAULT = 115200
START_TIMEOUT_DEFAULT = 8.0
IDLE_TIMEOUT_DEFAULT = 10.0
TOTAL_TIMEOUT_DEFAULT = 7200.0
TRIGGER_PERIOD_DEFAULT = 1.2
RECORD_RAW_SIZE_BYTES = 44


FLAG_BITS: dict[int, str] = {
    0: "BMP_OK",
    1: "IMU_OK",
    2: "IMU_CAL_DONE",
    3: "BARO_BASE_OK",
    4: "USB_VBUS",
    5: "CHG_STAT",
    6: "IMU_INT1",
    7: "IMU_INT2",
    8: "FLASH_OK",
    9: "FLASH_FALLBACK",
    10: "BMP_STALE",
    11: "IMU_STALE",
    12: "LOG_DROPPED",
    13: "EVT_LAUNCH",
    14: "EVT_APOGEE",
    15: "EVT_LANDED",
    16: "ARMED",
    17: "FLIGHT_FAULT",
    18: "RST_WDT",
    19: "RST_BOR",
    20: "RST_PIN",
    21: "RST_SOFT",
    22: "RST_HFAULT",
}


KEYWORDS = (
    "stm32",
    "stmicro",
    "st-link",
    "virtual com",
    "cdc",
    "usb serial",
    "serial device",
    "cp210",
    "ch340",
    "ftdi",
    "boros",
)


@dataclass
class PortCandidate:
    device: str
    description: str
    hwid: str
    manufacturer: str | None
    vid: int | None
    pid: int | None
    score: int


def require_pyserial() -> None:
    if serial is not None and list_ports is not None:
        return
    print(
        "PySerial not found. Install with:\n"
        "  python -m pip install pyserial",
        file=sys.stderr,
    )
    raise SystemExit(2)


def safe_name(text: str) -> str:
    text = text.strip().replace(" ", "_")
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", text)
    return text or "unknown"


def score_port(p: object) -> int:
    desc = (getattr(p, "description", "") or "").lower()
    hwid = (getattr(p, "hwid", "") or "").lower()
    manufacturer = (getattr(p, "manufacturer", "") or "").lower()
    device = (getattr(p, "device", "") or "").lower()
    text = f"{device} {desc} {hwid} {manufacturer}"
    score = 0

    vid = getattr(p, "vid", None)
    pid = getattr(p, "pid", None)
    if (vid, pid) in {(0x0483, 0x5740), (0x0483, 0x5741)}:
        score += 40

    for k in KEYWORDS:
        if k in text:
            score += 6

    if "bluetooth" in text:
        score -= 20
    if "com1" in device:
        score -= 8
    return score


def list_candidate_ports() -> list[PortCandidate]:
    require_pyserial()
    out: list[PortCandidate] = []
    for p in list_ports.comports():
        out.append(
            PortCandidate(
                device=p.device,
                description=p.description or "",
                hwid=p.hwid or "",
                manufacturer=getattr(p, "manufacturer", None),
                vid=getattr(p, "vid", None),
                pid=getattr(p, "pid", None),
                score=score_port(p),
            ),
        )
    out.sort(key=lambda x: (x.score, x.device), reverse=True)
    return out


class DumpCollector:
    def __init__(self, out_dir: Path, port: str, baud: int):
        self.out_dir = out_dir
        self.port = port
        self.baud = baud
        self.started = False
        self.ended = False
        self.header: list[str] | None = None
        self.meta: dict[str, str] = {}
        self.rows = 0
        self.crc_bad = 0
        self.seq_first: int | None = None
        self.seq_last: int | None = None
        self.ms_first: int | None = None
        self.ms_last: int | None = None
        self.alt_min_cm: int | None = None
        self.alt_max_cm: int | None = None
        self.state_counts: dict[str, int] = {}
        self.flag_counts: dict[str, int] = {name: 0 for name in FLAG_BITS.values()}
        self.start_utc: str | None = None
        self.end_utc: str | None = None
        self.last_status_print = 0.0

        self._raw_fh: TextIO | None = None
        self._csv_fh: TextIO | None = None
        self._event_fh: TextIO | None = None
        self._fault_fh: TextIO | None = None
        self._csv_writer: csv.writer | None = None
        self._event_writer: csv.writer | None = None
        self._fault_writer: csv.writer | None = None

    def _open_outputs(self) -> None:
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self._raw_fh = (self.out_dir / "raw_uart.txt").open("w", encoding="utf-8", newline="")
        self._csv_fh = (self.out_dir / "records.csv").open("w", encoding="utf-8", newline="")
        self._event_fh = (self.out_dir / "events.csv").open("w", encoding="utf-8", newline="")
        self._fault_fh = (self.out_dir / "faults.csv").open("w", encoding="utf-8", newline="")
        self._csv_writer = csv.writer(self._csv_fh)
        self._event_writer = csv.writer(self._event_fh)
        self._fault_writer = csv.writer(self._fault_fh)

    def _close_outputs(self) -> None:
        for fh in (self._raw_fh, self._csv_fh, self._event_fh, self._fault_fh):
            if fh is not None:
                fh.close()

    def close(self) -> None:
        self._close_outputs()

    def _parse_int(self, value: str, default: int = 0) -> int:
        try:
            return int(value, 0)
        except Exception:
            return default

    def _decode_flags(self, flags: int) -> list[str]:
        names: list[str] = []
        for bit, name in FLAG_BITS.items():
            if (flags >> bit) & 1:
                names.append(name)
        return names

    def _update_minmax(self, attr_min: str, attr_max: str, value: int) -> None:
        mn = getattr(self, attr_min)
        mx = getattr(self, attr_max)
        if mn is None or value < mn:
            setattr(self, attr_min, value)
        if mx is None or value > mx:
            setattr(self, attr_max, value)

    def _handle_row(self, row: list[str]) -> None:
        if self.header is None:
            return
        if len(row) != len(self.header):
            return

        rec = dict(zip(self.header, row))
        self.rows += 1

        seq = self._parse_int(rec.get("seq", "0"))
        ms = self._parse_int(rec.get("ms", "0"))
        alt_cm = self._parse_int(rec.get("altitude_cm", "0"))
        state = self._parse_int(rec.get("flight_state", "0"))
        flags = self._parse_int(rec.get("flags", "0"))
        crc_ok = self._parse_int(rec.get("crc_ok", "0"))

        if self.seq_first is None:
            self.seq_first = seq
        self.seq_last = seq

        if self.ms_first is None:
            self.ms_first = ms
        self.ms_last = ms

        self._update_minmax("alt_min_cm", "alt_max_cm", alt_cm)
        self.state_counts[str(state)] = self.state_counts.get(str(state), 0) + 1

        decoded = self._decode_flags(flags)
        for name in decoded:
            self.flag_counts[name] += 1
        if crc_ok != 1:
            self.crc_bad += 1

        is_event = any(name.startswith("EVT_") for name in decoded)
        is_fault = ("FLIGHT_FAULT" in decoded) or ("RST_HFAULT" in decoded)

        if self._event_writer is not None and is_event:
            self._event_writer.writerow(row + ["|".join(decoded)])
        if self._fault_writer is not None and is_fault:
            self._fault_writer.writerow(row + ["|".join(decoded)])

        now = time.time()
        if now - self.last_status_print >= 1.0:
            self.last_status_print = now
            print(f"[dump] records={self.rows}", end="\r", flush=True)

    def feed_line(self, line: str) -> None:
        line = line.rstrip("\r")

        if not self.started:
            if line.strip() != "DUMP_BEGIN":
                return
            self.started = True
            self.start_utc = datetime.now(timezone.utc).isoformat()
            self._open_outputs()

        if self._raw_fh is not None:
            self._raw_fh.write(line + "\n")

        if line.strip() == "DUMP_END":
            self.ended = True
            self.end_utc = datetime.now(timezone.utc).isoformat()
            return

        if line.startswith("#"):
            body = line[1:]
            if "," in body:
                k, v = body.split(",", 1)
                self.meta[k.strip()] = v.strip()
            return

        if self.header is None:
            if line.startswith("ms,seq,"):
                try:
                    self.header = next(csv.reader([line]))
                except Exception:
                    return
                if self._csv_writer is not None:
                    self._csv_writer.writerow(self.header)
                if self._event_writer is not None:
                    self._event_writer.writerow(self.header + ["flags_text"])
                if self._fault_writer is not None:
                    self._fault_writer.writerow(self.header + ["flags_text"])
            return

        if not line:
            return

        try:
            row = next(csv.reader([line]))
        except Exception:
            return
        if self._csv_writer is not None:
            self._csv_writer.writerow(row)
        self._handle_row(row)

    def write_summaries(self) -> None:
        bytes_used = self._parse_int(self.meta.get("bytes_used", "0"))
        records_meta = self._parse_int(self.meta.get("records", "0"))
        est_raw = records_meta * RECORD_RAW_SIZE_BYTES
        ratio = (bytes_used / est_raw) if est_raw > 0 else None

        summary = {
            "port": self.port,
            "baud": self.baud,
            "start_utc": self.start_utc,
            "end_utc": self.end_utc,
            "rows_csv": self.rows,
            "rows_crc_bad": self.crc_bad,
            "seq_first": self.seq_first,
            "seq_last": self.seq_last,
            "ms_first": self.ms_first,
            "ms_last": self.ms_last,
            "altitude_min_cm": self.alt_min_cm,
            "altitude_max_cm": self.alt_max_cm,
            "state_counts": self.state_counts,
            "flag_counts": self.flag_counts,
            "metadata_board": self.meta,
            "compression": {
                "bytes_used_flash": bytes_used,
                "estimated_raw_bytes": est_raw,
                "ratio_used_over_raw": ratio,
                "saving_percent": (1.0 - ratio) * 100.0 if ratio is not None else None,
            },
        }

        metadata = {
            "port": self.port,
            "baud": self.baud,
            "captured_utc": datetime.now(timezone.utc).isoformat(),
            "board_meta": self.meta,
            "files": {
                "raw_uart": "raw_uart.txt",
                "records_csv": "records.csv",
                "events_csv": "events.csv",
                "faults_csv": "faults.csv",
                "summary_json": "summary.json",
            },
            "note": "SPI packet decompression is performed by the firmware during DUMP.",
        }

        (self.out_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        (self.out_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )


def open_serial(device: str, baud: int):
    return serial.Serial(  # type: ignore[union-attr]
        port=device,
        baudrate=baud,
        timeout=0.2,
        write_timeout=1.0,
    )


def capture_dump_on_port(
    device: str,
    baud: int,
    out_root: Path,
    start_timeout: float,
    idle_timeout: float,
    total_timeout: float,
    trigger_period: float,
) -> Path | None:
    session_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = out_root / f"dump_{session_stamp}_{safe_name(device)}"
    collector = DumpCollector(session_dir, port=device, baud=baud)
    buf = ""
    last_rx = time.time()
    t0 = last_rx
    last_trigger = 0.0

    try:
        with open_serial(device, baud) as ser:
            try:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            except Exception:
                pass

            while True:
                now = time.time()
                if (not collector.started) and ((now - last_trigger) >= trigger_period):
                    ser.write(b"d")
                    ser.flush()
                    last_trigger = now

                chunk = ser.read(2048)
                if chunk:
                    last_rx = now
                    buf += chunk.decode("utf-8", errors="replace")

                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        collector.feed_line(line)
                        if collector.ended:
                            break

                if collector.ended:
                    print()
                    collector.write_summaries()
                    collector.close()
                    return session_dir

                if not collector.started and (now - t0) > start_timeout:
                    collector.close()
                    return None

                if collector.started and (now - last_rx) > idle_timeout:
                    collector.close()
                    raise TimeoutError("Idle timeout while dumping")

                if (now - t0) > total_timeout:
                    collector.close()
                    raise TimeoutError("Total timeout while dumping")
    finally:
        collector.close()


def try_ports(
    ports: list[str],
    baud: int,
    out_root: Path,
    start_timeout: float,
    idle_timeout: float,
    total_timeout: float,
    trigger_period: float,
) -> Path:
    last_err: Exception | None = None
    for p in ports:
        print(f"[dump] trying port {p} ...")
        try:
            out = capture_dump_on_port(
                device=p,
                baud=baud,
                out_root=out_root,
                start_timeout=start_timeout,
                idle_timeout=idle_timeout,
                total_timeout=total_timeout,
                trigger_period=trigger_period,
            )
            if out is not None:
                return out
        except Exception as e:
            last_err = e
            print(f"[dump] failed on {p}: {e}")
    if last_err:
        raise last_err
    raise RuntimeError("No port responded with DUMP_BEGIN")


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Automatic Boros telemetry dump over UART (with COM autodetection)",
    )
    ap.add_argument("--port", type=str, default="", help="Manual serial port (e.g. COM7, /dev/ttyACM0)")
    ap.add_argument("--baud", type=int, default=BAUD_DEFAULT)
    ap.add_argument("--out", type=str, default="firmware/dumps", help="Output directory")
    ap.add_argument("--list-ports", action="store_true", help="List detected ports and exit")
    ap.add_argument("--start-timeout", type=float, default=START_TIMEOUT_DEFAULT)
    ap.add_argument("--idle-timeout", type=float, default=IDLE_TIMEOUT_DEFAULT)
    ap.add_argument("--total-timeout", type=float, default=TOTAL_TIMEOUT_DEFAULT)
    ap.add_argument("--trigger-period", type=float, default=TRIGGER_PERIOD_DEFAULT)
    args = ap.parse_args()

    require_pyserial()
    out_root = Path(args.out).resolve()
    out_root.mkdir(parents=True, exist_ok=True)

    cands = list_candidate_ports()
    if args.list_ports:
        if not cands:
            print("No serial ports detected.")
            return
        for c in cands:
            print(
                f"{c.device:12s} score={c.score:3d} | {c.description} | "
                f"VID:PID={c.vid if c.vid is not None else '-'}:{c.pid if c.pid is not None else '-'}",
            )
        return

    if args.port:
        ports = [args.port]
    else:
        if not cands:
            raise SystemExit("No serial ports found.")
        ports = [c.device for c in cands]
        print("[dump] port attempt order:", ", ".join(ports))

    session_dir = try_ports(
        ports=ports,
        baud=args.baud,
        out_root=out_root,
        start_timeout=args.start_timeout,
        idle_timeout=args.idle_timeout,
        total_timeout=args.total_timeout,
        trigger_period=args.trigger_period,
    )

    summary = json.loads((session_dir / "summary.json").read_text(encoding="utf-8"))
    print(f"[dump] completed at: {session_dir}")
    print(
        "[dump] records={rows} spi_bytes={bytes_used} ratio={ratio}".format(
            rows=summary.get("rows_csv"),
            bytes_used=summary.get("compression", {}).get("bytes_used_flash"),
            ratio=summary.get("compression", {}).get("ratio_used_over_raw"),
        ),
    )


if __name__ == "__main__":
    main()
