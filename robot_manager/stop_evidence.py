"""Read-only cold-start stop evidence; independent of ROS imports."""
import math


def cold_stopped(controllers, hardware, runtime, age):
    # The owned launch spawns controllers inactive. A partially loaded list
    # need not delay stopping hardware that has never been configured.
    return (
        controllers is not None and hardware is not None
        and set(hardware) == {'RobotCanopenSystem', 'CanDriverSystem'}
        and all(state == 1 for state in hardware.values())
        and all(state in ('inactive', 'unconfigured') for state in controllers)
        and math.isfinite(age) and 0 <= age < .5
        and runtime.get('state') == 'Stopped'
        and runtime.get('diagnostics', {}).get('readable') is True
        and set(runtime.get('groups', {})) == {'arm', 'base', 'flippers', 'gripper'}
        and all(state == 'Stopped' for state in runtime['groups'].values())
    )
