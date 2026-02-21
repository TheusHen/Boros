#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import struct
from dataclasses import dataclass
from pathlib import Path


LOG_FLAG_BMP_OK = 1 << 0
LOG_FLAG_IMU_OK = 1 << 1
LOG_FLAG_IMU_CAL_DONE = 1 << 2
LOG_FLAG_BARO_BASE_OK = 1 << 3
LOG_FLAG_USB_VBUS = 1 << 4
LOG_FLAG_FLASH_OK = 1 << 8
LOG_FLAG_FLASH_FALLBK = 1 << 9
LOG_FLAG_LOG_DROPPED = 1 << 12
LOG_FLAG_EVT_LAUNCH = 1 << 13
LOG_FLAG_EVT_APOGEE = 1 << 14
LOG_FLAG_EVT_LANDED = 1 << 15
LOG_FLAG_ARMED = 1 << 16
LOG_FLAG_FLIGHT_FAULT = 1 << 17

FLIGHT_STATE_PAD = 0
FLIGHT_STATE_ARMED = 1
FLIGHT_STATE_BOOST = 2
FLIGHT_STATE_COAST = 3
FLIGHT_STATE_APOGEE = 4
FLIGHT_STATE_DESCENT = 5
FLIGHT_STATE_LANDED = 6
FLIGHT_STATE_FAULT = 7

LOG_PKT_KEY = 0xA1
LOG_PKT_DELTA = 0xA2
LOG_KEY_INTERVAL = 32
LOG_PAGE_MAX = 256
LOG_RAM_RECS = 256
LOG_SIZE = 16 * 1024 * 1024
LOG_MAX_PAYLOAD = 220
RECORD_STRUCT = struct.Struct("<IIiihhhhhhhhhHHBBI")


@dataclass
class FlightEvents:
    state_changed: int = 0
    launch: int = 0
    apogee: int = 0
    landed: int = 0


@dataclass
class FlightFsm:
    state: int = FLIGHT_STATE_PAD
    state_since_ms: int = 0
    launch_ms: int = 0
    max_alt_cm: int = 0
    launch_votes: int = 0
    coast_votes: int = 0
    apogee_votes: int = 0
    landed_votes: int = 0
    fault_votes: int = 0


@dataclass
class LogRec:
    ms: int
    seq: int
    pressure_pa: int
    altitude_cm: int
    temp_centi: int
    ax_mg: int
    ay_mg: int
    az_mg: int
    gx_dps_x10: int
    gy_dps_x10: int
    gz_dps_x10: int
    vz_cms: int
    vbat_mv: int
    bmp_stale: int
    imu_stale: int
    flight_state: int
    reset_cause: int
    flags: int


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def put_uvar(v: int) -> bytes:
    out = bytearray()
    while True:
        b = v & 0x7F
        v >>= 7
        if v:
            b |= 0x80
        out.append(b)
        if not v:
            break
    return bytes(out)


def put_svar(v: int) -> bytes:
    zz = (v << 1) ^ (v >> 31)
    return put_uvar(zz & 0xFFFFFFFF)


def sat_i16(v: int) -> int:
    if v > 32767:
        return 32767
    if v < -32768:
        return -32768
    return v


def pack_record(r: LogRec) -> bytes:
    return RECORD_STRUCT.pack(
        r.ms,
        r.seq,
        r.pressure_pa,
        r.altitude_cm,
        r.temp_centi,
        r.ax_mg,
        r.ay_mg,
        r.az_mg,
        r.gx_dps_x10,
        r.gy_dps_x10,
        r.gz_dps_x10,
        r.vz_cms,
        r.vbat_mv,
        r.bmp_stale,
        r.imu_stale,
        r.flight_state,
        r.reset_cause,
        r.flags,
    )


def transition(fsm: FlightFsm, next_state: int, now_ms: int, events: FlightEvents) -> None:
    if fsm.state == next_state:
        return
    fsm.state = next_state
    fsm.state_since_ms = now_ms
    fsm.launch_votes = 0
    fsm.coast_votes = 0
    fsm.apogee_votes = 0
    fsm.landed_votes = 0
    events.state_changed = 1


def flight_fsm_step(
    fsm: FlightFsm,
    now_ms: int,
    altitude_cm: int,
    az_mg: int,
    vz_cms: int,
    sensors_ok: bool,
    arm_ok: bool,
    events: FlightEvents,
) -> None:
    events.state_changed = 0
    events.launch = 0
    events.apogee = 0
    events.landed = 0

    if altitude_cm > fsm.max_alt_cm:
        fsm.max_alt_cm = altitude_cm

    if not sensors_ok:
        fsm.fault_votes = min(2000, fsm.fault_votes + 1)
    elif fsm.fault_votes > 0:
        fsm.fault_votes -= 1

    if (
        fsm.fault_votes > 500
        and fsm.state not in (FLIGHT_STATE_PAD, FLIGHT_STATE_ARMED, FLIGHT_STATE_LANDED)
    ):
        transition(fsm, FLIGHT_STATE_FAULT, now_ms, events)
        return

    if fsm.state == FLIGHT_STATE_PAD:
        if arm_ok:
            transition(fsm, FLIGHT_STATE_ARMED, now_ms, events)
        return

    if fsm.state == FLIGHT_STATE_ARMED:
        if not arm_ok:
            transition(fsm, FLIGHT_STATE_PAD, now_ms, events)
            return
        if az_mg > 1700 and vz_cms > 150:
            fsm.launch_votes = min(1000, fsm.launch_votes + 1)
        else:
            fsm.launch_votes = 0
        if fsm.launch_votes >= 3:
            fsm.launch_ms = now_ms
            transition(fsm, FLIGHT_STATE_BOOST, now_ms, events)
            events.launch = 1
        return

    if fsm.state == FLIGHT_STATE_BOOST:
        if az_mg < 300:
            fsm.coast_votes = min(1000, fsm.coast_votes + 1)
        else:
            fsm.coast_votes = 0
        if fsm.coast_votes >= 8 or (now_ms - fsm.launch_ms) > 6000:
            transition(fsm, FLIGHT_STATE_COAST, now_ms, events)
        return

    if fsm.state == FLIGHT_STATE_COAST:
        if vz_cms < -80 and fsm.max_alt_cm > 300:
            fsm.apogee_votes = min(1000, fsm.apogee_votes + 1)
        else:
            fsm.apogee_votes = 0
        if fsm.apogee_votes >= 5:
            transition(fsm, FLIGHT_STATE_APOGEE, now_ms, events)
            events.apogee = 1
        return

    if fsm.state == FLIGHT_STATE_APOGEE:
        if (now_ms - fsm.state_since_ms) > 300:
            transition(fsm, FLIGHT_STATE_DESCENT, now_ms, events)
        return

    if fsm.state == FLIGHT_STATE_DESCENT:
        if abs(vz_cms) < 30 and altitude_cm < 300:
            fsm.landed_votes = min(2000, fsm.landed_votes + 1)
        elif fsm.landed_votes > 0:
            fsm.landed_votes -= 1
        if fsm.landed_votes >= 250:
            transition(fsm, FLIGHT_STATE_LANDED, now_ms, events)
            events.landed = 1


class FirmwareLogEmulator:
    def __init__(self) -> None:
        self.page_size = LOG_PAGE_MAX
        self.capacity = LOG_SIZE
        self.ram_capacity = LOG_RAM_RECS

        self.used_bytes = 0
        self.seq = 0
        self.records_total = 0
        self.drop_count = 0
        self.flash_fail_count = 0
        self.max_ram_pending = 0

        self.flash_enabled = True
        self.force_key = True
        self.need_key = True
        self.have_prev = False
        self.since_key = 0
        self.prev: LogRec | None = None

        self.buf = bytearray()
        self.buf_records: list[LogRec] = []
        self.ram: list[LogRec] = []

    def ram_count(self) -> int:
        return len(self.ram)

    def _ram_push(self, rec: LogRec) -> None:
        if self.ram and self.ram[-1].seq == rec.seq:
            return
        if len(self.ram) >= self.ram_capacity:
            self.drop_count += 1
            self.ram.pop(0)
        self.ram.append(rec)
        if len(self.ram) > self.max_ram_pending:
            self.max_ram_pending = len(self.ram)

    def _salvage_buffer_to_ram(self) -> None:
        for rec in self.buf_records:
            self._ram_push(rec)
        self.buf.clear()
        self.buf_records.clear()
        self.have_prev = False
        self.prev = None
        self.force_key = True
        self.need_key = True
        self.since_key = 0

    def _write_flash(self, nbytes: int, flash_hw_ok: bool) -> bool:
        if not self.flash_enabled or not flash_hw_ok:
            return False
        self.used_bytes = min(self.capacity, self.used_bytes + nbytes)
        return True

    def _flush_buffer_to_flash(self, flash_hw_ok: bool) -> bool:
        if not self.buf:
            return True
        if not self._write_flash(len(self.buf), flash_hw_ok):
            return False
        self.buf.clear()
        self.buf_records.clear()
        self.need_key = True
        return True

    def _append_packet(self, pkt: bytes, rec: LogRec, flash_hw_ok: bool) -> bool:
        if len(pkt) > (2 + LOG_MAX_PAYLOAD + 2):
            return False
        if len(self.buf) + len(pkt) > self.page_size:
            if not self._flush_buffer_to_flash(flash_hw_ok):
                return False
        self.buf.extend(pkt)
        self.buf_records.append(rec)
        if len(self.buf) == self.page_size:
            if not self._flush_buffer_to_flash(flash_hw_ok):
                return False
        return True

    def _encode_delta_payload(self, prev: LogRec, cur: LogRec) -> bytes | None:
        if cur.seq < prev.seq or cur.ms < prev.ms:
            return None
        payload = bytearray()

        def add_u(v: int) -> bool:
            b = put_uvar(v)
            if len(payload) + len(b) > LOG_MAX_PAYLOAD:
                return False
            payload.extend(b)
            return True

        def add_s(v: int) -> bool:
            b = put_svar(v)
            if len(payload) + len(b) > LOG_MAX_PAYLOAD:
                return False
            payload.extend(b)
            return True

        ok = (
            add_u(cur.seq - prev.seq)
            and add_u(cur.ms - prev.ms)
            and add_s(cur.pressure_pa - prev.pressure_pa)
            and add_s(cur.altitude_cm - prev.altitude_cm)
            and add_s(cur.temp_centi - prev.temp_centi)
            and add_s(cur.ax_mg - prev.ax_mg)
            and add_s(cur.ay_mg - prev.ay_mg)
            and add_s(cur.az_mg - prev.az_mg)
            and add_s(cur.gx_dps_x10 - prev.gx_dps_x10)
            and add_s(cur.gy_dps_x10 - prev.gy_dps_x10)
            and add_s(cur.gz_dps_x10 - prev.gz_dps_x10)
            and add_s(cur.vz_cms - prev.vz_cms)
            and add_s(cur.vbat_mv - prev.vbat_mv)
            and add_s(cur.bmp_stale - prev.bmp_stale)
            and add_s(cur.imu_stale - prev.imu_stale)
            and add_u(cur.flags ^ prev.flags)
            and add_u(cur.flight_state)
            and add_u(cur.reset_cause)
        )
        return bytes(payload) if ok else None

    def _encode_record_packet(self, rec: LogRec) -> bytes | None:
        use_key = (not self.have_prev) or self.force_key or self.need_key or (self.since_key >= LOG_KEY_INTERVAL)
        payload = b""
        tag = LOG_PKT_KEY

        if not use_key and self.prev is not None:
            payload = self._encode_delta_payload(self.prev, rec) or b""
            if not payload or len(payload) >= RECORD_STRUCT.size:
                use_key = True

        if use_key:
            tag = LOG_PKT_KEY
            payload = pack_record(rec)
            self.since_key = 0
        else:
            tag = LOG_PKT_DELTA
            self.since_key += 1

        if len(payload) > 255:
            return None
        header = bytes((tag, len(payload)))
        crc = crc16_ccitt(header + payload)
        pkt = header + payload + bytes((crc & 0xFF, (crc >> 8) & 0xFF))

        self.prev = rec
        self.have_prev = True
        self.force_key = False
        self.need_key = False
        return pkt

    def _enter_fallback(self, current: LogRec | None) -> None:
        self.flash_fail_count += 1
        self.flash_enabled = False
        self._salvage_buffer_to_ram()
        if current is not None:
            self._ram_push(current)

    def _flush_ram_to_flash(self, flash_hw_ok: bool) -> bool:
        while self.ram:
            rec = self.ram[0]
            pkt = self._encode_record_packet(rec)
            if pkt is None:
                return False
            if not self._write_flash(len(pkt), flash_hw_ok):
                return False
            self.ram.pop(0)
        return True

    def append(self, rec: LogRec, flash_hw_ok: bool) -> None:
        rec.seq = self.seq
        self.seq += 1
        self.records_total += 1

        if (not self.flash_enabled) and flash_hw_ok:
            self.flash_enabled = True
            self.force_key = True
            self.need_key = True
            self.have_prev = False
            self.since_key = 0

        if self.flash_enabled and (not flash_hw_ok):
            self._enter_fallback(rec)
            return

        if (not self.flash_enabled) or self.ram:
            self._ram_push(rec)
            return

        if self.buf and ((self.page_size - len(self.buf)) < (2 + LOG_MAX_PAYLOAD + 2)):
            if not self._flush_buffer_to_flash(flash_hw_ok):
                self._enter_fallback(rec)
                return

        pkt = self._encode_record_packet(rec)
        if pkt is None:
            self._enter_fallback(rec)
            return

        if not self._append_packet(pkt, rec, flash_hw_ok):
            self._enter_fallback(rec)

    def flush(self, flash_hw_ok: bool) -> None:
        if not self.flash_enabled and flash_hw_ok:
            self.flash_enabled = True
            self.force_key = True
            self.need_key = True
            self.have_prev = False
            self.since_key = 0

        if not self.flash_enabled:
            self._salvage_buffer_to_ram()
            return

        if self.ram:
            if not self._flush_ram_to_flash(flash_hw_ok):
                self.flash_fail_count += 1
                self.flash_enabled = False
                self.force_key = True
                self.need_key = True
                self.have_prev = False
                self.since_key = 0
                return

        if not self._flush_buffer_to_flash(flash_hw_ok):
            self.flash_fail_count += 1
            self.flash_enabled = False
            self._salvage_buffer_to_ram()


def load_firmware_like_rows(path: Path) -> list[dict[str, int]]:
    rows: list[dict[str, int]] = []
    with path.open("r", encoding="utf-8", newline="") as fh:
        rd = csv.DictReader(fh)
        for row in rd:
            rows.append(
                {
                    "ms": int(row["ms"]),
                    "pressure_pa": int(row["pressure_pa"]),
                    "altitude_cm": int(row["altitude_cm"]),
                    "temp_centi": int(row["temp_centi"]),
                    "ax_mg": int(row["ax_mg"]),
                    "ay_mg": int(row["ay_mg"]),
                    "az_mg": int(row["az_mg"]),
                    "gx_dps_x10": int(row["gx_dps_x10"]),
                    "gy_dps_x10": int(row["gy_dps_x10"]),
                    "gz_dps_x10": int(row["gz_dps_x10"]),
                    "flags": int(row["flags"], 0),
                }
            )
    return rows


def file_meta(path: Path) -> dict[str, object]:
    data = path.read_bytes()
    return {
        "path": str(path),
        "size_bytes": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
    }


def run_playback_scenario(
    rows: list[dict[str, int]],
    fail_ranges: list[tuple[int, int]],
    flush_period_ms: int = 250,
) -> dict[str, object]:
    fsm = FlightFsm()
    emu = FirmwareLogEmulator()
    event_times: dict[str, int] = {}
    state_counts: dict[str, int] = {}

    alt_last_ms = 0
    alt_last_cm = 0
    vz_lp_cms = 0
    last_flush = 0

    for i, row in enumerate(rows):
        flash_hw_ok = True
        for a, b in fail_ranges:
            if a <= i < b:
                flash_hw_ok = False
                break

        ms = row["ms"]
        altitude_cm = row["altitude_cm"]

        flags_in = row["flags"]
        bmp_ok = bool(flags_in & LOG_FLAG_BMP_OK)
        imu_ok = bool(flags_in & LOG_FLAG_IMU_OK)
        imu_cal_done = bool(flags_in & LOG_FLAG_IMU_CAL_DONE)
        baro_base_ok = bool(flags_in & LOG_FLAG_BARO_BASE_OK)
        usb_vbus = bool(flags_in & LOG_FLAG_USB_VBUS)

        if baro_base_ok:
            if alt_last_ms != 0 and ms > alt_last_ms:
                dt_ms = ms - alt_last_ms
                vz_inst = ((altitude_cm - alt_last_cm) * 1000) // dt_ms
                vz_lp_cms = ((vz_lp_cms * 7) + vz_inst) // 8
            alt_last_ms = ms
            alt_last_cm = altitude_cm
        else:
            vz_lp_cms = (vz_lp_cms * 7) // 8
        vz_cms = sat_i16(vz_lp_cms)

        arm_ok = baro_base_ok and imu_cal_done and (not usb_vbus)
        sensors_ok = bmp_ok and imu_ok

        ev = FlightEvents()
        flight_fsm_step(
            fsm=fsm,
            now_ms=ms,
            altitude_cm=altitude_cm,
            az_mg=row["az_mg"],
            vz_cms=vz_cms,
            sensors_ok=sensors_ok,
            arm_ok=arm_ok,
            events=ev,
        )

        out_flags = flags_in
        if emu.flash_enabled:
            out_flags |= LOG_FLAG_FLASH_OK
        else:
            out_flags |= LOG_FLAG_FLASH_FALLBK
        if emu.drop_count:
            out_flags |= LOG_FLAG_LOG_DROPPED
        if ev.launch:
            out_flags |= LOG_FLAG_EVT_LAUNCH
            event_times.setdefault("launch_ms", ms)
        if ev.apogee:
            out_flags |= LOG_FLAG_EVT_APOGEE
            event_times.setdefault("apogee_ms", ms)
        if ev.landed:
            out_flags |= LOG_FLAG_EVT_LANDED
            event_times.setdefault("landed_ms", ms)
        if fsm.state == FLIGHT_STATE_ARMED:
            out_flags |= LOG_FLAG_ARMED
        if fsm.state == FLIGHT_STATE_FAULT:
            out_flags |= LOG_FLAG_FLIGHT_FAULT

        rec = LogRec(
            ms=ms,
            seq=0,
            pressure_pa=row["pressure_pa"],
            altitude_cm=altitude_cm,
            temp_centi=row["temp_centi"],
            ax_mg=row["ax_mg"],
            ay_mg=row["ay_mg"],
            az_mg=row["az_mg"],
            gx_dps_x10=row["gx_dps_x10"],
            gy_dps_x10=row["gy_dps_x10"],
            gz_dps_x10=row["gz_dps_x10"],
            vz_cms=vz_cms,
            vbat_mv=-1,
            bmp_stale=0 if bmp_ok else 1,
            imu_stale=0 if imu_ok else 1,
            flight_state=fsm.state,
            reset_cause=0,
            flags=out_flags,
        )
        emu.append(rec, flash_hw_ok=flash_hw_ok)
        state_key = str(fsm.state)
        state_counts[state_key] = state_counts.get(state_key, 0) + 1

        if (ms - last_flush) >= flush_period_ms:
            last_flush = ms
            emu.flush(flash_hw_ok=flash_hw_ok)

    emu.flush(flash_hw_ok=True)

    mode = "flash" if emu.flash_enabled and emu.ram_count() == 0 else "ram_fallback"
    raw_bytes = emu.records_total * RECORD_STRUCT.size
    ratio = (emu.used_bytes / raw_bytes) if raw_bytes > 0 else None

    return {
        "records_total": emu.records_total,
        "bytes_used_flash": emu.used_bytes,
        "raw_bytes_equivalent": raw_bytes,
        "compression_ratio": ratio,
        "compression_saving_percent": ((1.0 - ratio) * 100.0) if ratio is not None else None,
        "flash_fail_count": emu.flash_fail_count,
        "dropped_records": emu.drop_count,
        "ram_pending_end": emu.ram_count(),
        "ram_peak_records": emu.max_ram_pending,
        "mode_end": mode,
        "final_flight_state": fsm.state,
        "state_counts": state_counts,
        "event_times_ms": event_times,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="Replay firmware-like telemetry through a firmware storage/state emulator")
    ap.add_argument("--input", type=str, default="sim/flight/out/firmware_like_log.csv")
    ap.add_argument("--firmware-bin", type=str, default="firmware/firmware.bin")
    ap.add_argument("--firmware-elf", type=str, default="firmware/firmware.elf")
    ap.add_argument("--out", type=str, default="sim/flight/out/firmware_playback_summary.json")
    ap.add_argument("--fallback-start-frac", type=float, default=0.25)
    ap.add_argument("--fallback-duration-frac", type=float, default=0.015)
    args = ap.parse_args()

    in_path = Path(args.input)
    if not in_path.exists():
        raise SystemExit(f"Input log not found: {in_path}")

    fw_bin = Path(args.firmware_bin)
    fw_elf = Path(args.firmware_elf)
    if not fw_bin.exists():
        raise SystemExit(f"Firmware binary not found: {fw_bin}")
    if not fw_elf.exists():
        raise SystemExit(f"Firmware ELF not found: {fw_elf}")

    rows = load_firmware_like_rows(in_path)
    n = len(rows)
    if n == 0:
        raise SystemExit("Input log is empty.")

    start = int(max(0.0, min(0.95, args.fallback_start_frac)) * n)
    dur = max(1, int(max(0.001, min(0.95, args.fallback_duration_frac)) * n))
    end = min(n, start + dur)

    nominal = run_playback_scenario(rows=rows, fail_ranges=[])
    injected = run_playback_scenario(rows=rows, fail_ranges=[(start, end)])

    payload = {
        "input": {
            "path": str(in_path),
            "records": n,
            "raw_record_size_bytes": RECORD_STRUCT.size,
        },
        "firmware": {
            "bin": file_meta(fw_bin),
            "elf": file_meta(fw_elf),
        },
        "scenarios": {
            "nominal": nominal,
            "fallback_injected": injected,
        },
        "fallback_window": {
            "start_index": start,
            "end_index": end,
        },
    }

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    print("=== Firmware Playback ===")
    print(f"Input records: {n}")
    print(f"Firmware BIN: {fw_bin} ({payload['firmware']['bin']['size_bytes']} bytes)")
    print(f"Firmware ELF: {fw_elf} ({payload['firmware']['elf']['size_bytes']} bytes)")
    print(
        "Nominal mode={mode} drop={drop} ratio={ratio:.3f}".format(
            mode=nominal["mode_end"],
            drop=nominal["dropped_records"],
            ratio=nominal["compression_ratio"] or 0.0,
        ),
    )
    print(
        "Injected fallback mode={mode} drop={drop} flash_fail={ff} ram_peak={rp}".format(
            mode=injected["mode_end"],
            drop=injected["dropped_records"],
            ff=injected["flash_fail_count"],
            rp=injected["ram_peak_records"],
        ),
    )
    print(f"Summary: {out_path}")


if __name__ == "__main__":
    main()
