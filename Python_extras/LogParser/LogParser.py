"""Parser for DATA_LOG.

The log is a tagged CSV stream where the first column identifies the record type:

- ``G`` -> GPS
- ``I`` -> BNO / IMU
- ``B`` -> BMP

The file in this folder is not valid Python yet, so this module provides a real
parser implementation that reads ``DATA_LOG`` and sorts each row into typed
records.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, List

import numpy as np
import matplotlib.pyplot as plt


@dataclass(slots=True)
class GPS:
    time: np.ndarray = field(default_factory=lambda: np.array([], dtype=int))
    fixtype: np.ndarray = field(default_factory=lambda: np.array([], dtype=int))
    lat: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    lon: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    alt: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    timestamp: np.ndarray = field(default_factory=lambda: np.array([], dtype=int))


@dataclass(slots=True)
class BNO:
    ax: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    ay: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    az: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    gx: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    gy: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    gz: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    timestamp: np.ndarray = field(default_factory=lambda: np.array([], dtype=int))


@dataclass(slots=True)
class BMP:
    temp: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    press: np.ndarray = field(default_factory=lambda: np.array([], dtype=float))
    timestamp: np.ndarray = field(default_factory=lambda: np.array([], dtype=int))


@dataclass(slots=True)
class ParsedLog:
    gps: GPS = field(default_factory=GPS)
    bno: BNO = field(default_factory=BNO)
    bmp: BMP = field(default_factory=BMP)
    unknown: List[list[str]] = field(default_factory=list)


class LogParser:
    def __init__(self, file_path: str | Path) -> None:
        self.file_path = Path(file_path)

    def parse(self) -> ParsedLog:
        gps_time: list[int] = []
        gps_fixtype: list[int] = []
        gps_lat: list[float] = []
        gps_lon: list[float] = []
        gps_alt: list[float] = []
        gps_timestamp: list[int] = []

        bno_ax: list[float] = []
        bno_ay: list[float] = []
        bno_az: list[float] = []
        bno_gx: list[float] = []
        bno_gy: list[float] = []
        bno_gz: list[float] = []
        bno_timestamp: list[int] = []

        bmp_temp: list[float] = []
        bmp_press: list[float] = []
        bmp_timestamp: list[int] = []

        unknown: List[list[str]] = []

        with self.file_path.open(newline="", encoding="utf-8") as handle:
            for row in csv.reader(self._iter_data_lines(handle)):
                if not row:
                    continue

                tag = row[0].strip().upper()
                fields = [value.strip() for value in row[1:]]

                if tag == "G":
                    record = self._parse_gps(fields)
                    if record is not None:
                        gps_time.append(record.time)
                        gps_fixtype.append(record.fixtype)
                        gps_lat.append(record.lat)
                        gps_lon.append(record.lon)
                        gps_alt.append(record.alt)
                        gps_timestamp.append(record.timestamp)
                    else:
                        unknown.append(row)
                elif tag == "I":
                    record = self._parse_bno(fields)
                    if record is not None:
                        bno_ax.append(record.ax)
                        bno_ay.append(record.ay)
                        bno_az.append(record.az)
                        bno_gx.append(record.gx)
                        bno_gy.append(record.gy)
                        bno_gz.append(record.gz)
                        bno_timestamp.append(record.timestamp)
                    else:
                        unknown.append(row)
                elif tag == "B":
                    record = self._parse_bmp(fields)
                    if record is not None:
                        bmp_temp.append(record.temp)
                        bmp_press.append(record.press)
                        bmp_timestamp.append(record.timestamp)
                    else:
                        unknown.append(row)
                else:
                    unknown.append(row)

        return ParsedLog(
            gps=GPS(
                time=np.asarray(gps_time, dtype=int),
                fixtype=np.asarray(gps_fixtype, dtype=int),
                lat=np.asarray(gps_lat, dtype=float),
                lon=np.asarray(gps_lon, dtype=float),
                alt=np.asarray(gps_alt, dtype=float),
                timestamp=np.asarray(gps_timestamp, dtype=int),
            ),
            bno=BNO(
                ax=np.asarray(bno_ax, dtype=float),
                ay=np.asarray(bno_ay, dtype=float),
                az=np.asarray(bno_az, dtype=float),
                gx=np.asarray(bno_gx, dtype=float),
                gy=np.asarray(bno_gy, dtype=float),
                gz=np.asarray(bno_gz, dtype=float),
                timestamp=np.asarray(bno_timestamp, dtype=int),
            ),
            bmp=BMP(
                temp=np.asarray(bmp_temp, dtype=float),
                press=np.asarray(bmp_press, dtype=float),
                timestamp=np.asarray(bmp_timestamp, dtype=int),
            ),
            unknown=unknown,
        )

    @staticmethod
    def _iter_data_lines(handle: Iterable[str]) -> Iterable[str]:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue
            if line.startswith("//"):
                continue
            if line in {"GPS:", "BNO", "BMP"}:
                continue
            yield line

    @staticmethod
    def _parse_gps(fields: list[str]) -> GPS | None:
        try:
            if len(fields) == 6:
                time, fixtype, lat, lon, alt, timestamp = fields
            elif len(fields) == 7:
                time, timestamp, fixtype, _num_sv, lat, lon, alt = fields
            else:
                return None

            return GPS(
                time=int(float(time)),
                fixtype=int(float(fixtype)),
                lat=float(lat),
                lon=float(lon),
                alt=float(alt),
                timestamp=int(float(timestamp)),
            )
        except ValueError:
            return None

    @staticmethod
    def _parse_bno(fields: list[str]) -> BNO | None:
        if len(fields) != 7:
            return None

        try:
            ax, ay, az, gx, gy, gz, timestamp = fields
            return BNO(
                ax=float(ax),
                ay=float(ay),
                az=float(az),
                gx=float(gx),
                gy=float(gy),
                gz=float(gz),
                timestamp=int(float(timestamp)),
            )
        except ValueError:
            return None

    @staticmethod
    def _parse_bmp(fields: list[str]) -> BMP | None:
        if len(fields) != 3:
            return None

        try:
            temp, press, timestamp = fields
            return BMP(
                temp=float(temp),
                press=float(press),
                timestamp=int(float(timestamp)),
            )
        except ValueError:
            return None


def parse_log(file_path: str | Path) -> ParsedLog:
    return LogParser(file_path).parse()


if __name__ == "__main__":
    default_path = Path(__file__).with_name("DATA_LOG")
    result = parse_log(default_path)

    print(f"GPS records: {len(result.gps.timestamp)}")
    print(f"BNO records: {len(result.bno.timestamp)}")
    print(f"BMP records: {len(result.bmp.timestamp)}")
    print(f"Unknown rows: {len(result.unknown)}")

    plt.figure()
    plt.plot(result.bno.timestamp, result.bno.ax)
    plt.plot(result.bno.timestamp, result.bno.ay)
    plt.plot(result.bno.timestamp, result.bno.az)
    plt.title("BNO ax vs timestamp")
    plt.xlabel("timestamp")
    plt.ylabel("ax")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    

