## Warning:
*Due to a small design error, plugging in a normal USB cable into the Raspberry Pi Picos will back feed `5v` into the `3.3v` line of the board potentially damaging some components.*

**Solutions:**<br>
 - **1. Option:**&nbsp; *Modify a normal USB cable by cutting the cable's red (`VCC`) wire, turning it into a data-only cable.*
 - **2. Option:**&nbsp; *Purchase and use a data-only USB cable.*

<br>
<br>

## Improvements for Rev. 2:
 - **1.**&nbsp; *Add diode (or `P-FET`) to Raspberry Pi Pico `VSYS` for power ORing. (https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf)*
 - **2.**&nbsp; *Combine power pins of the IR edge sensors to reduce wire count.*
 - **3.**&nbsp; *Add mounting supports for the `MPU6050` module.*
 - **4.**&nbsp; *Re-position decoupling capacitors to more optimal locations.*
 - **5.**&nbsp; *Add expansion connector that exposes unused ADC pins.*
 - **6.**&nbsp; *Connect fourth, 4-pin ultrasonic connector directly to mux so that it can be used as general analog I/O.*
