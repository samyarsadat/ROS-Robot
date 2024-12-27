#  The ROS robot project (Robot Driver Package)
#  Diagnostics report data structure
#  Copyright 2024 Samyar Sadat Akhavi
#  Written by Samyar Sadat Akhavi, 2024.
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https: www.gnu.org/licenses/>.

from enum import Enum
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from typing import Union


class DiagnosticsReport:
    level: "DiagnosticsReport.Level"
    name: str
    message: str
    hardware_id: str
    key_values: list[tuple]

    def from_ros_message(self, report: DiagnosticStatus) -> None:
        self.level = DiagnosticsReport.Level(int.from_bytes(report.level, "big"))
        self.name = report.name
        self.message = report.message
        self.hardware_id = report.hardware_id
        self.key_values = []

        for pair in report.values:
            self.key_values.append((pair.key, pair.value))

    def to_ros_message(self) -> DiagnosticStatus:
        ros_report = DiagnosticStatus()
        ros_report.level = self.level.value.to_bytes(1, "big")
        ros_report.name = self.name
        ros_report.message = self.message
        ros_report.hardware_id = self.hardware_id

        for pair in self.key_values:
            ros_report.values.append(KeyValue(key=pair[0], value=pair[1]))

        return ros_report

    class Level(Enum):
        OK = 0
        WARN = 1
        ERROR = 2
        STALE = 3

        @staticmethod
        def from_string(name: str) -> Union["DiagnosticsReport.Level", None]:
            match name.upper():
                case DiagnosticsReport.Level.OK.name: return DiagnosticsReport.Level.OK
                case DiagnosticsReport.Level.WARN.name: return DiagnosticsReport.Level.WARN
                case DiagnosticsReport.Level.ERROR.name: return DiagnosticsReport.Level.ERROR
                case DiagnosticsReport.Level.STALE.name: return DiagnosticsReport.Level.STALE
                case _: return None
