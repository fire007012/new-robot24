#!/usr/bin/env python3
"""Attended bare-motor CANopen PP bounded continuous keyboard control on can0.
Uses project flipper scale 19857408 counts/output revolution. No zero/limit/save writes.
"""
import threading
import fcntl
import json
import math
from pathlib import Path
import signal
import sys
import tempfile
import time
from PyQt5 import QtCore, QtWidgets
from run_canopen_velocity import SdoBus

SCALE = 19857408 / (2 * math.pi)
STEP = round(SCALE * math.pi / 180)
FOLDER = Path(tempfile.mkdtemp(prefix='flipper-keyboard-'))
LOG = (FOLDER/'events.jsonl').open('w', buffering=1)
def record(**event):
    event['time'] = time.time()
    LOG.write(json.dumps(event, ensure_ascii=False)+'\n')
    print(event, flush=True)

class Axis:
    def __init__(self, ident):
        self.ident=ident;self.bus=SdoBus('can0',ident,.35);self.original={};self.origin=None
    def read(self, index, signed=False):
        for attempt in range(3):
            try:return self.bus.transfer(index,signed=signed)
            except RuntimeError as error:
                if '超时' not in str(error) or attempt==2:raise
    def write(self,index,value,size=4,signed=False):
        self.bus.transfer(index,value=value,size=size,signed=signed)
    def position(self):return self.read(0x6064,True)
    def check(self):
        status=self.read(0x6041)
        if status & 8:raise RuntimeError(f'节点{self.ident}故障 {self.read(0x603f):04x}')
        return status
    def prepare(self):
        if self.check() & 0x6f==0x27:raise RuntimeError(f'节点{self.ident}初始已使能')
        if self.read(0x6061,True)!=1:raise RuntimeError('需要原位置模式1')
        self.origin=self.position()
        for idx in (0x6081,0x6083,0x6084,0x6085,0x605d): self.original[idx]=self.read(idx)
        # Five degrees/s, bounded position target. Settings are RAM-only and restored.
        for idx,value in ((0x6081,STEP*5),(0x6083,STEP*4),(0x6084,STEP*4),(0x6085,STEP*8),(0x605d,1)):
            self.write(idx,value,2 if idx==0x605d else 4)
            if self.read(idx)!=value:raise RuntimeError('位置曲线参数读回不符')
        self.write(0x607a,self.origin,signed=True)
        self.bus.send(0,bytes([1,self.ident]))
        for word,state in ((6,0x21),(7,0x23),(15,0x27)):
            self.bus.control(word);self.bus.wait_state(state)
        record(node=self.ident,action='prepared',origin=self.origin)
    def move(self,target,cancel=lambda:False):
        start=self.position()
        if abs(target-start)>STEP*1.2:raise RuntimeError('单次目标超过1度')
        if abs(target-self.origin)>STEP*30:raise RuntimeError('超过本次空载调试±30度范围')
        self.bus.control(15);self.write(0x607a,target,signed=True);self.bus.control(63)
        deadline=time.monotonic()+5
        reached=False
        try:
            while time.monotonic()<deadline:
                if cancel():return
                if self.check() & 0x6f!=0x27:raise RuntimeError('运动中未处于使能态')
                actual=self.position()
                if abs(actual-start)>STEP*1.5:raise RuntimeError('位移超出本次目标范围')
                if abs(actual-target)<STEP*.03:
                    reached=True;record(node=self.ident,action='move',start=start,target=target,actual=actual);return
                time.sleep(.025)
            raise RuntimeError(f'节点{self.ident}未到达目标，start={start}, target={target}, actual={actual}')
        finally:
            if reached:self.bus.control(15)
            else:self.bus.control(2)
    def stop(self):self.bus.control(2)
    def close(self):
        errors=[]
        for word in (2,0):
            try:self.bus.control(word)
            except Exception as error:errors.append(str(error))
        try:
            self.bus.wait_state(0x40)
            for idx,value in self.original.items():self.write(idx,value,2 if idx==0x605d else 4)
            record(node=self.ident,action='disabled',position=self.position(),status=self.read(0x6041),errors=errors)
        except Exception as error:record(node=self.ident,action='cleanup_error',error=str(error))
        self.bus.bus.close()

class Controller:
    def __init__(self,axes):
        self.axes=axes;self.intent=None;self.heartbeat=time.monotonic()
        self.quit=False;self.error=None;self.message='等待按键';self.samples=[]
        self.thread=threading.Thread(target=self.run);self.thread.start()
    def run(self):
        active=None;direction=0;stopping=False;stopped_since=None;deadline=0
        try:
            while not self.quit or active is not None:
                now=time.monotonic()
                wanted=self.intent if not self.quit and now-self.heartbeat<.25 else None
                if active is None and wanted is not None:
                    idx,direction=wanted;active=self.axes[idx]
                    position=active.position()
                    target=active.origin+direction*STEP*30
                    if direction*(target-position)<STEP*.2:
                        self.message='已到行程边界，请松键或反向';active=None;time.sleep(.02);continue
                    active.write(0x607a,position,signed=True)
                    for word,state in ((6,0x21),(7,0x23),(15,0x27)):
                        active.bus.control(word);active.bus.wait_state(state)
                    active.bus.control(15);active.write(0x607a,target,signed=True);active.bus.control(63)
                    stopping=False;stopped_since=None
                    self.message=f'节点{active.ident} 连续运动'
                    record(node=active.ident,action='hold_start',position=position,direction=direction)
                if active is not None:
                    if active.check() & 0x6f!=0x27:raise RuntimeError('运动状态异常')
                    position=active.position();velocity=active.read(0x606c,True)
                    self.samples.append((now,active.ident,position,velocity,stopping))
                    if abs(position-active.origin)>STEP*30.2:raise RuntimeError('超出调试边界')
                    if abs(velocity)>STEP*6.5:raise RuntimeError('实际速度超限')
                    current=(active.ident-1,direction)
                    if not stopping and (wanted!=current or direction*(position-active.origin)>=STEP*29.8):
                        active.bus.control(0x10f)  # Halt: decelerate using 6084 / 605D=1.
                        stopping=True;deadline=now+4.5;stop_history=[]
                        record(node=active.ident,action='halt_ramp',position=position,velocity=velocity)
                    if stopping:
                        stop_history.append((now,position))
                        stop_history=[sample for sample in stop_history if now-sample[0]<.25]
                        # 606C is noisy at rest on these drives. Confirm a stable
                        # measured position window as well as bounded velocity.
                        stable=(len(stop_history)>5 and now-stop_history[0][0]>.18
                                and max(p for _,p in stop_history)-min(p for _,p in stop_history)<STEP*.01
                                and abs(velocity)<STEP*.5)
                        if stable:
                            active.bus.control(0);active.bus.wait_state(0x40)
                            record(node=active.ident,action='smooth_stopped',position=position,velocity=velocity)
                            active=None;self.message='已平滑停车，等待按键'
                        if active is not None and now>deadline:raise RuntimeError('减速停车超时')
                time.sleep(.02)
        except Exception as error:
            self.error=str(error);record(action='controller_error',error=self.error)
            for axis in self.axes:
                try:axis.stop();axis.bus.control(0)
                except Exception:pass
        finally:
            (FOLDER/'samples.json').write_text(json.dumps(self.samples))
    def close(self):
        self.intent=None;self.quit=True;self.thread.join()

class Window(QtWidgets.QWidget):
    def __init__(self,controller):
        super().__init__();self.controller=controller;self.keys=set()
        self.mapping={QtCore.Qt.Key_Y:(0,1),QtCore.Qt.Key_H:(0,-1),QtCore.Qt.Key_U:(1,1),QtCore.Qt.Key_J:(1,-1),QtCore.Qt.Key_I:(2,1),QtCore.Qt.Key_K:(2,-1),QtCore.Qt.Key_O:(3,1),QtCore.Qt.Key_L:(3,-1)}
        self.setWindowTitle('四摆臂空载连续控制 — can0');self.resize(740,300)
        layout=QtWidgets.QVBoxLayout(self)
        layout.addWidget(QtWidgets.QLabel('按住连续运动，松开平滑停车；每次仅操作一个轴。\n节点1 左前 Y/H；节点2 右前 U/J；节点3 左后 I/K；节点4 右后 O/L（正/反）\n速度约5°/秒；加减速度约4°/秒²；空载范围为启动位置±30°。\n空格、窗口失焦：减速停车；Esc：停车并退出失能。'))
        self.label=QtWidgets.QLabel();layout.addWidget(self.label)
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(20)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
    def keyPressEvent(self,e):
        if e.isAutoRepeat():return
        if e.key()==QtCore.Qt.Key_Escape:self.close();return
        if e.key()==QtCore.Qt.Key_Space:self.clear();return
        if e.key() in self.mapping:self.keys.add(e.key());self.update_intent()
    def keyReleaseEvent(self,e):
        if not e.isAutoRepeat():self.keys.discard(e.key());self.update_intent()
    def update_intent(self):
        self.controller.intent=self.mapping[next(iter(self.keys))] if len(self.keys)==1 else None
    def clear(self):self.keys.clear();self.controller.intent=None
    def event(self,e):
        if e.type() in (QtCore.QEvent.WindowDeactivate,QtCore.QEvent.FocusOut) and hasattr(self,'keys'):self.clear()
        return super().event(e)
    def tick(self):
        self.controller.heartbeat=time.monotonic()
        if not self.isActiveWindow():self.clear()
        self.label.setText('故障：'+self.controller.error if self.controller.error else self.controller.message)
    def closeEvent(self,e):
        self.clear();self.timer.stop();self.controller.close();e.accept()

def main():
    lock=open('/tmp/robot24-base-keyboard.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    axes=[];controller=None
    try:
        record(action='start',folder=str(FOLDER))
        for ident in range(1,5):
            axis=Axis(ident);axes.append(axis);axis.prepare()
            axis.bus.control(0);axis.bus.wait_state(0x40)
        controller=Controller(axes)
        if '--boundary-test' in sys.argv:
            def hold(intent,seconds):
                controller.intent=intent
                end=time.monotonic()+seconds
                while time.monotonic()<end and not controller.error:
                    controller.heartbeat=time.monotonic();time.sleep(.02)
                if controller.error:raise RuntimeError(controller.error)
            for idx,axis in enumerate(axes):
                start=axis.position()
                # Place the software boundary 1 degree away for a small real
                # reproduction of reaching a boundary and then reversing.
                axis.origin=start-29*STEP
                try:
                    hold((idx,1),3)
                    hold(None,.8)
                    boundary=axis.position()
                    if boundary-start<STEP*.7:raise RuntimeError('未到达边界测试位置')
                    hold((idx,-1),.8)
                    hold(None,1.8)
                    reversed_position=axis.position()
                    if boundary-reversed_position<STEP*.5:raise RuntimeError('边界反向测试失败')
                    record(node=axis.ident,action='boundary_reverse_passed',start=start,
                           boundary=boundary,reversed_position=reversed_position)
                finally:
                    controller.intent=None
                    axis.origin=axis.position()
            record(action='four_axis_boundary_test_passed')
        if '--self-test' in sys.argv:
            for idx in range(4):
                for direction in (1,-1):
                    controller.intent=(idx,direction)
                    end=time.monotonic()+1
                    while time.monotonic()<end and not controller.error:
                        controller.heartbeat=time.monotonic();time.sleep(.02)
                    controller.intent=None
                    end=time.monotonic()+.8
                    while time.monotonic()<end and not controller.error:
                        controller.heartbeat=time.monotonic();time.sleep(.02)
                    if controller.error:raise RuntimeError(controller.error)
            record(action='continuous_test_passed')
        app=QtWidgets.QApplication([]);window=Window(controller);window.show();window.activateWindow();window.setFocus()
        signal.signal(signal.SIGINT,lambda *_:window.close());signal.signal(signal.SIGTERM,lambda *_:window.close())
        record(action='keyboard_ready');app.exec_()
    except Exception as error:record(action='failed',error=str(error));return 1
    finally:
        if controller:controller.close()
        for axis in axes:axis.close()
        LOG.close()
    return 0
if __name__=='__main__':sys.exit(main())
