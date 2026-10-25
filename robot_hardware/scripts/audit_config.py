#!/usr/bin/env python3
"""Audit generated startup writes, relative references and singleton/full-bus equivalence."""
import configparser,struct,sys
from pathlib import Path
import yaml
root=Path(sys.argv[1])/'robot_canopen_config/share/robot_canopen_config/config'
for directory in [root,*sorted(root.glob('axis_*'))]:
 bus=yaml.safe_load((directory/'bus.yml').read_text());assert bus['options']['dcf_path']=='.'
 nodes=set(bus)-{'options','master'};assert len(nodes)==(6 if directory==root else 1)
 dcf=configparser.ConfigParser(interpolation=None);dcf.read(directory/'master.dcf')
 for joint in nodes:
  cfg=bus[joint];node=cfg['node_id'];assert dcf['1F81Value'][str(node)]=='0x00000005'
  ref=dcf[f'1F22sub{node:X}']['UploadFile'];assert ref=='./'+joint+'.bin'
  data=(directory/ref).read_bytes();number=struct.unpack_from('<I',data)[0];cursor=4;objects=[]
  for _ in range(number):
   index,sub,length=struct.unpack_from('<HBI',data,cursor);cursor+=7
   value=int.from_bytes(data[cursor:cursor+length],'little');cursor+=length
   assert index not in {0x607C,0x607D,0x2003,0x1010,0x1011},f'{joint}: persistent startup write {index:04X}'
   objects.append((index,sub,value))
  assert cursor==len(data)
  assert (0x60C2,1,5) in objects
  assert (0x6060,0,7 if node in (5,6) else 9) in objects
  if directory!=root:assert data==(root/(joint+'.bin')).read_bytes()
 print(directory.name+': startup writes and references verified')
