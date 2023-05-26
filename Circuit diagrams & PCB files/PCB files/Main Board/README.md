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

## Important Note #1:
*Because this revision of the PCB does not include a voltage divider for measuring the battery voltage, I have attached a wire to pin `ADC2` of Raspberry Pi Pico (B) and created a small proto board with two resistors on it so that I can measure the voltage of the battery safely. This functionality will be incorporated onto the board itself for rev. 2.*

## Important Note #2:
*On this revision of the PCB, there are two five-pin connectors for the motor encoders. I had initially planned to have one common current limiting resistor for all the encoders, however, this did not work. As a result, I have added a small proto board that splits the two five-pin connectors into four four-pin connectors (one connector per encoder) and adds individual current limiting resistors for each encoder. These extra resistors and the four-pin connectors will be incorporated onto the board itself for rev. 2.*

<br>
<br>

## Improvements for Rev. 2 (Stage 4):
 - **1.**&nbsp; *Add diode (or `P-FET`) to Raspberry Pi Pico `VSYS` for power ORing. (https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf)*
 - **2.**&nbsp; *Combine power pins of the IR edge sensors to reduce wire count.*
 - **3.**&nbsp; *Add mounting supports for the `MPU6050` module.*
 - **4.**&nbsp; *Re-position decoupling capacitors to more optimal locations.*
 - **5.**&nbsp; *Add expansion connector that exposes unused ADC pins.*
 - **6.**&nbsp; *Connect fourth, 4-pin ultrasonic connector directly to mux so that it can be used as general analog I/O.*
 - **7.**&nbsp; *Consistent connector pinouts.*
 - **8.**&nbsp; *Replace power input terminal footprint with a 3-pin Molex KK-254 connector.*
 - **9.**&nbsp; *Add battery voltage monitoring.*
 - **10.**&nbsp; *Add mosfet for fan control.*
 - **11.**&nbsp; *Add 3-pin addressable LED connectors.*
 - **12.**&nbsp; *Change encoder connectors from two five-pins to four four-pins (individual connector for each encoder).*
 - **13.**&nbsp; *Add individual current limiting resistors and connectors for motor encoders.*
