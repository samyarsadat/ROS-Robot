#  The ROS robot project (Robot Driver Package)
#  Self-test result data structure
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

from ros_robot_driver.interface_data_structs.diagnostics_report import DiagnosticsReport
from diagnostic_msgs.srv import SelfTest


class SelftestResult:
    id: str
    passed: bool
    status: list[DiagnosticsReport]

    def from_ros_result(self, result: SelfTest.Response) -> None:
        self.id = result.id
        self.passed = bool(result.passed)
        self.status = []

        for i, report in enumerate(result.status):
            self.status.append(DiagnosticsReport())
            self.status[i].from_ros_message(report)

    def to_ros_result(self) -> SelfTest.Response:
        report = SelfTest.Response()
        report.id = self.id
        report.passed = bytes(self.passed)

        for i, status in enumerate(self.status):
            report.status.append(status.to_ros_message())

        return report