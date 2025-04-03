# README

Minimal too to upload PX4 firmware. Stripped from lab's PX4 repo. 

Steps:

1. Copy firmware into this folder. That is `crazyflie_default.px4` into this folder, from `(LabCode)/PX4-firmware/build/nuttx_crazyflie_default`
2. Run `bash upload.sh`. This will in turn call the python script. You should see a message `Attempting to reboot ...`
3. Connect CF via USB. Should see a message that the board has been found, and progress (Erase, Program, Verify, Rebooting.)
4. You're done!
