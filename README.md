# tudelft_hackathon
Repository for the TU Delft hackathon

# Set SYSID_MYGCS

Ardupilot only allows override msgs from the GCS. It is necessary to change
the [SYSID_MYGCS](https://ardupilot.org/copter/docs/parameters.html#sysid-mygcs-my-ground-station-number) to allow msgs coming from mavros.

The following command can be used to inspect the current sysid.

```Bash
$ ros2 topic echo /uas1/mavlink_source
```

Or

```Bash
$ ros2 param get mavros system_id
```
https://ardupilot.org/dev/docs/mavlink-routing-in-ardupilot.html


set mavros sysid to 255,240
https://discuss.ardupilot.org/t/sending-commands-via-mavros-setpoint-velocity-cmd-vel-in-guided-mode/51610/4


### useful links
https://github.com/mavlink/mavros/issues/1718
