/*
    The ROS robot project - Diagnostics Report Messages - Pico B
    Copyright 2022-2024 Samyar Sadat Akhavi
    Written by Samyar Sadat Akhavi, 2022-2024.
 
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
  
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
 
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https: www.gnu.org/licenses/>.
*/

#pragma once


// ---- STATUS OK ----
#define DIAG_OK_MSG_MICROSW_LED_TEST_PASS  "Camera LED and microswitch test passed."

// ---- STATUS WARN ----
#define DIAG_WARN_MSG_DHT11_MEASUREMENT_TIMEOUT  "DHT11 measurement timed out."

// ---- STATUS ERROR ----
#define DIAG_ERR_MSG_MICROSW_LED_TEST_FAIL  "Camera LED and microswitch test failed."
#define DIAG_ERR_MSG_DHT11_BAD_CHECKSUM     "DHT11 measurement bad checksum."
#define DIAG_ERR_MSG_DHT11_UNSPEC_FAIL      "DHT11 unspecified failure."

// ---- STATUS STALE ----