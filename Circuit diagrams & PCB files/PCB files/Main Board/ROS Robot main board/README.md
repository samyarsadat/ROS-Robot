## File Info:

### .PcbDoc & .SchDoc & .PrjPcb:
These files are for people that have access to Altium Designer.<br>
These files are ideal for editing.<br>

<br>

### Gerber Files:
These are manufacturing files.<br>
They were prepared according to JLCPCB's requirements.<br>
If you are using another manufacturer, you might need to re-export them from Altium according to your manufacturer's requirements.<br>

<br>
<br>

## PCB Connector Info:

#### 5 Volt, 4 pin (Echo, Trigger) ultrasonic sensors:
    Pinout: 
    1: +5v 
    2: GND 
    3: Trigger 
    4: Echo
    
    - H12 (Ultrasonic 1)
    - H13 (Ultrasonic 2)
    - H15 (Ultrasonic 3)
    - H16 (Ultrasonic 4)

#### 3.3 Volt, 3 pin (Echo, Trigger combined) ultrasonic sensor:
    Pinout: 
    1: +3v3 
    2: GND 
    3: Signal Pin (Echo + Trigger)

    - H14 (Ultrasonic 5)

#### Microswitches:
    Pinout (Bi-directional connector): 
    1: Signal 
    2: GND
    
    Note: Microswitches will use RPI Pico's internal pull-up resistors.

    - H11 (Back Left)
    - H8 (Back Right)
    - H4 (Front Left)
    - H1 (Front Right)

#### IR edge sensors:
    Pinout: 
    1: GND 
    2: IR Enable 
    3: +3v3 
    4: Analog Input
    
    - H25 (Edge Sensor 4)
    - H22 (Edge Sensor 3)
    - H20 (Edge Sensor 2)
    - H17 (Edge Sensor 1)
    - H26 (Edge Sensor 8)
    - H23 (Edge Sensor 7)
    - H21 (Edge Sensor 6)
    - H18 (Edge Sensor 5)

#### Motor encoders:
    Pinout (H24): 
    1: +3v3 (Rewired to +5v)
    2: GND 
    3: Back Right Encoder Input (Channel A) 
    4: Back Right Encoder Input (Channel B)
    5: Front Right Encoder Input (Channel A)

    Pinout (H19): 
    1: Front Right Encoder Input (Channel B)
    2: Back Left Encoder Input (Channel A) 
    3: Back Left Encoder Input (Channel B) 
    4: Front Left Encoder Input (Channel A)
    5: Front Left Encoder Input (Channel B)
    
    Note: All encoder inputs are pulled-up to 3.3v by 4.7k resistors.

    - H24 (Encoder Connector A)
    - H19 (Encoder Connector B)

#### Camera LEDs:
    Pinout:
    1: GND
    2: Camera LED Output 1
    2: Camera LED Output 2
    2: Camera LED Output 3
    2: Camera LED Output 4
    
    Note: All camera LED outputs are directly connected to RPI Pico's GPIOs without current limiting resistors.

    - H9

#### Motor driver:
    Pinout:
    1: GND
    2: Left Motor Output (Channel B)
    3: Left Motor Output (Channel A)
    4: Right Motor Output (Channel B)
    5: Right Motor Output (Channel A)

    - H2

#### Raspberry Pi 4 power relay:
    Pinout:
    1: Signal Output
    2: VCC (Can be set to +3v3 or +5v via a jumper (J1))
    3: GND

    - H6

#### Front panel:
    Pinout:
    1: Speed Select Switch Input (Contact 1)
    2: Mode Select Switch Input
    3: Speed Select Switch Input (Contact 2)
    4: GND
    5: Indicator LED Output (Through a 220ohm resistor)

    - H10
   
<br>
<br>

## Component values:

#### Resistors (All footprints are 1206 SMD):
    R5, R7, R8, R10, R11, R12, R13, R16 = 4.7k
    R1, R3, R4, R6 = 220ohm
    R2 = 430ohm
    R9 = 10k
    R15 = 0ohm (Shorted)
    R14 = Unpopulated
    
#### Capacitors (All footprints are 1206 SMD, Ceramic):
    C1 = 100nf
    C2, C3, C4 = 100nf (Not required)
    
#### Diodes & LEDs (All footprints are 1206 SMD):
    D1 = 1N5822 (or any similar Schottky diode)
    D2 = Red LED (+5v indicator LED)
    D3 = Green LED (+3v3 indicator LED)
    D4 = Blue LED (Edge IR sensor mosfet active indicator LED)
    
   
    
