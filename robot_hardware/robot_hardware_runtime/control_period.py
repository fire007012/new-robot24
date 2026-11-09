"""Derive the manager rate from the installed bus, refusing mixed timing assets."""
import configparser
import atexit
from pathlib import Path
import tempfile

import yaml


def controller_parameter_file(template, rate):
    """Use node-scoped parameters: a wildcard ROS override may lose to a named YAML node."""
    parameters = yaml.safe_load(Path(template).read_text())
    parameters['controller_manager']['ros__parameters']['update_rate'] = rate
    with tempfile.NamedTemporaryFile(mode='w', prefix='robot-controllers-', suffix='.yaml',
                                     delete=False) as stream:
        yaml.safe_dump(parameters, stream)
        path = Path(stream.name)
    atexit.register(lambda: path.unlink(missing_ok=True))
    return str(path)


def control_rate(config_directory):
    directory = Path(config_directory)
    bus = yaml.safe_load((directory / 'bus.yml').read_text())
    sync = bus['master']['sync_period']
    if type(sync) is not int or sync not in (5000, 20000):
        raise ValueError('SYNC period must be 5000 or 20000 microseconds')
    period_ms = sync // 1000
    nodes = set(bus) - {'options', 'master'}
    if not nodes:
        raise ValueError('bus contains no axes')
    for name in nodes:
        node = bus[name]
        interpolation = [entry['value'] for entry in node['sdo']
                         if entry['index'] == 0x60C2 and entry['sub_index'] == 1]
        if (type(node['period']) is not int or node['period'] != period_ms or
                interpolation != [period_ms] or type(interpolation[0]) is not int):
            raise ValueError(f'{name}: driver/SYNC/interpolation periods disagree')
    dcf = configparser.ConfigParser(interpolation=None)
    dcf.read(directory / 'master.dcf')
    section = dcf['1006']
    if int(section.get('ParameterValue', section['DefaultValue']), 0) != sync:
        raise ValueError('master.dcf and bus.yml SYNC periods disagree')
    return 1000000 // sync
