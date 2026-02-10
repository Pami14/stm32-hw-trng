Project Documentation: Hardware Random Number Generator (TRNG)

1. Project Overview

This project is a True Random Number Generator (TRNG) based on an STM32 microcontroller. The system captures physical noise, processes it into high-entropy random numbers, and provides the data via USB or saves it directly to an SD card. The device is controlled via a 4x4 keypad and an LCD interface.

1.1 The Entropy Source (Noise Generation)
The core randomness is generated using 32 independent noise sources. Each source utilizes an NPN transistor in reverse-bias mode, operating in the avalanche breakdown region to create analog white noise. This noise is then amplified and passed through a Schmitt trigger to convert the analog signal into a clean digital bitstream.

1.2 Cascaded XOR Entropy Combiner
To ensure maximum entropy and eliminate bias, the system implements a Cascaded XOR Tree (Entropy Whitening). Instead of using a single bit directly, one "final" bit is derived from four independent noise sources:
- Logic: Final Bit = [(Source 1) ⊕ (Source 2)] ⊕ [(Source 3) ⊕ (Source 4)]
- Purpose: This cascading structure performs hardware whitening. By XORing multiple sources, the entropy of the output is significantly improved, ensuring the data remains unpredictable even if a single generator becomes unstable.

2. Core Features
USB Mode: Streams high-speed random numbers to a host computer via USB.
SD Mode: Logs random data directly to an SD card for long-term storage or analysis.
Calibration: Allows for the individual calibration of each of the 32 noise generators or the selection of a specific source for testing.
Error Detection: Real-time monitoring to detect if a specific generator becomes "static" (stuck at 0 or 1) or fails due to hardware drift.

3. System Components & File Paths
Component           Path / Details
Main Logic          /core/Src/main.c
Includes            /core/Inc/
Hardware Concept    /assets/images/HW_concept.png
Images              /assets/images/
Videos              /assets/videos/
