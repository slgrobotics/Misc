## PCA9685 emulator

### Why use emulator?

Some of us use "[Raspberry Pi Servo Hat](https://www.amazon.com/MakerFocus-Resolution-Interface-Compatible-Raspberry/dp/B07H9ZTWNC)"
or "[PCA9685 servo driver](https://www.amazon.com/HUAREW-PCA9685-Interface-Compatible-Raspberry/dp/B0CRV3MK14/)"
and its variations for its intended purpose - generating 50 Hz 800..2200 us "servo PWM". There's a PCA9685 driver in any software stack and is very easy to use.
It is also dirt cheap. Yes - its accuracy isn't very good, which results in servo "jitter", but "nobody is perfect".

Here is a bigger problem though - it doesn't know when the host machine (or the controlling process) dies. It leaves the servos (ESCs, wheels) in the "running" state and keeps sending the same PWM signal to them.

When your small robot tries to run away - you just pick it up and reboot. When your 22 hp lawnmower, blades spinning, runs away - you might have an expensive disaster waiting to happen. Don't ask me how I know.

### Original code

Good people at Donkey Car published code to [emulate PCA9685](https://github.com/jwatte/donkey_racing/tree/master/teensy_hat_firmware).

There's a lot of Donkey Car related stuff there, and just for my purposes I copied and pasted some of it and made it work for me.

### Implementation and features

This emulator puts all servos in "safe" positions if it doesn't receive an I2C transmission within 1 second interval.
In my case, wheel control levers, gas engine throttle and cutter deck clutch are set to neutral/idle/off. As a bonus, pulse accuracy/stability is very good too.

I tried to use a much cheaper RPi Pico, but it turns out that in "slave" mode its Wire library waits to fill the receive buffer (all 256 bytes)
before calling Receive service interrupt routine.

Teensy 4.0 has a very flexible I2C library that works well (passes the 65-byte messages from PX4 right when they arrive).

I am pretty sure there's a lot to improve in the code, and RPi Pico could be made to work in this role. Let me know if you figured out how to use it.

### Hardware

Here is how the hardware looks like in my case.
I am using [Teensy 4.0](https://www.amazon.com/Microcontroller-Development-Standard-Non-Lockable-Version/dp/B0CV19K9PY) with minimal added components.

Note that you may need to cut a trace on Teensy 4.0 PCB to isolate USB power from your robot's power source:

https://www.pjrc.com/teensy/external_power.html

https://forum.pjrc.com/index.php?threads/teensy-4-0-using-external-power.66111/post-268771

![Screenshot from 2025-06-27 18-35-18](https://github.com/user-attachments/assets/73eedf14-32f1-43cf-9551-5a25e288963a)

This is art, not engineering. That's my excuse - what's yours? ;-)

![Screenshot from 2025-06-27 18-36-02](https://github.com/user-attachments/assets/566b74eb-9e11-4167-9a04-71954c8c5571)
