# SD card flashing troubleshooting {#setup-troubleshooting-flashing status=draft}

What to do if you cannot complete the flashing procedure. 

Assigned: Aleks

## "DTShell object has no attribute sprint" when using `dts tok set`

You have to completely reinstall `dts` and its commands. Do that by:

1. Delete the `~/.dt-shell folder`
2. Uninstall `dts` by running `pip uninstall duckietown-shell`
3. Reinstall `dts` by following the procedure in [Laptop Setup](#laptop-setup)

## "Bad archive" when using `dts init_sd_card`

Symptom: The `dts init_sd_card` command exits with error "Bad archive"
 
Resolution: It usually means that you don't have enough disk space on your laptop. Free up some space and try again.
