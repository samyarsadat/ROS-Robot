## Known Issue #1:
*Due to a small design error, plugging in a normal USB cable into the Raspberry Pi Picos will back feed `5v` into the `3.3v` line of the board, potentially damaging some components.*

**Solutions:**<br>
 - **1. Option:**&nbsp; *Modify a normal USB cable by cutting the cable's red (`VCC`) wire, turning it into a data-only cable.*
 - **2. Option:**&nbsp; *Purchase and use a data-only USB cable.*

<br>

## Known Issue #2:
*For unknown reasons, early Raspberry Pi Pico boards (The ones which have an `RP2-B1` marking on their chip) do not work (Windows doesn't recognize the board) when powered only through their `VSYS` pin. So make sure that the Picos that you are going to be using have an `RP2-B2` (or higher) marking on their chips. Changes were made to the USB hardware of the `RP2-B2` (3rd revision) chips however, I am not sure these changes affect this particular issue. I was not able to find any information to suggest that there has been more than one revision of the Raspberry Pi Pico PCB and I also was not able to spot any visual differences between the two boards, so the USB hardware changes to the RP2040 chip itself are probably what fixed this issue but, I am not sure.*

**Solutions:**<br>
 - **1. Option:**&nbsp; *Use Raspberry Pi Pico boards with chips that have `RP2-B2` (or higher) markings on them.*

<br>
<br>

## Improvements for Rev. 2 (Stage 2):
 - **1.**&nbsp; *Add diode (or `P-FET`) to Raspberry Pi Pico `VSYS` for power ORing. (https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf)*
 - **2.**&nbsp; *Combine power pins of the IR edge sensors to reduce wire count.*
 - **3.**&nbsp; *Add mounting supports for the `MPU6050` module.*
 - **4.**&nbsp; *Re-position decoupling capacitors to more optimal locations.*
 - **5.**&nbsp; *Add expansion connector that exposes unused ADC pins.*
 - **6.**&nbsp; *Connect fourth, 4-pin ultrasonic connector directly to mux so that it can be used as general analog I/O.*
 - **7.**&nbsp; *Consistent connector pinouts.*
 - **8.**&nbsp; *Replace power input terminal footprint with a 3-pin Molex KK-254 connector.*
