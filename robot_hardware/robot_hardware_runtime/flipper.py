"""ROS-independent flipper jog reference; joint signs remain the URDF/driver convention."""
import math
NAMES = ('left_front_arm_joint', 'right_front_arm_joint', 'left_rear_arm_joint', 'right_rear_arm_joint')
GROUPS = {'independent': ((0,), (1,), (2,), (3,)), 'left_right_mirror': ((0,1),(2,3)),
          'front_rear_sync': ((0,1,2,3),), 'side_pair': ((0,2),(1,3)), 'diagonal_pair': ((0,3),(1,2))}
class Reference:
    def __init__(self, limits):
        self.limits = limits; self.linkage='independent'; self.reset()
    def reset(self):
        self.reference=None;self.velocity=[0.0]*4;self.filtered=[0.0]*4;self.stamp=None
    def command(self, names, values, now):
        if len(names)!=len(values) or not names or len(set(names))!=len(names):
            self.reset();raise ValueError('invalid jog names/size')
        expanded={}
        for name,value in zip(names,values):
            if name not in NAMES or not math.isfinite(value): self.reset();raise ValueError('invalid jog')
            i=NAMES.index(name)
            affected=next(group for group in GROUPS[self.linkage] if i in group)
            for index in affected:
                if index in expanded and abs(expanded[index]-value)>1e-9:
                    self.reset();raise ValueError('conflicting linkage commands')
                expanded[index]=value
        self.velocity=[max(-.3,min(.3,expanded.get(i,0.0))) for i in range(4)];self.stamp=now
    def step(self, measured, now, dt):
        if len(measured)!=4 or any(not math.isfinite(q) for q in measured): self.reset();raise ValueError('invalid feedback')
        if self.reference is None: self.reference=list(measured)
        timed_out=self.stamp is None or now<self.stamp or now-self.stamp>.25
        for i,name in enumerate(NAMES):
            lo,hi=self.limits[name]['lower'],self.limits[name]['upper']
            if abs(measured[i]-self.reference[i])>.35: self.reference[i]=measured[i]
            target=0.0 if timed_out else self.velocity[i]
            self.filtered[i]=0.0 if timed_out else .35*target+.65*self.filtered[i]
            if (measured[i]<=lo and self.filtered[i]<0) or (measured[i]>=hi and self.filtered[i]>0):self.filtered[i]=0.0
            self.reference[i]=max(lo,min(hi,self.reference[i]+self.filtered[i]*max(0,min(dt,.02))))
        return list(self.reference),list(self.filtered),timed_out
