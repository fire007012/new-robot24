import fcntl, json, math, os, signal, subprocess, tempfile, time
from pathlib import Path
import rclpy
from ament_index_python.packages import get_package_prefix
from can_driver.msg import MotorState
from can_driver.srv import Init, Shutdown
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from PyQt5 import QtCore, QtWidgets

lock = open('/tmp/robot24-base-keyboard.lock', 'w')
fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
folder = Path(tempfile.mkdtemp(prefix='robot24-base-keyboard-'))
ns = '/base_keyboard'
names = ['left_front_wheel_joint', 'right_front_wheel_joint']
config = {'can_driver_node': {'control_frequency': 50., 'motor_query_hz': 50.,
    'motor_state_period_sec': .02, 'direct_cmd_timeout_sec': .2,
    'debug_bypass_ros_control': False, 'pp_fast_write_enabled': False,
    'joints': [dict(name=name, motor_id=ident, master_id=master, protocol='DM',
                    can_device='can0', control_mode='velocity', direction_sign=direction,
                    position_scale=.0001, velocity_scale=.0001)
               for name, ident, master, direction in zip(names, [7,5], [8,6], [1.,-1.])]}}
(folder/'config.yaml').write_text(json.dumps(config))
rclpy.init()
node = rclpy.create_node('base_keyboard_client')
latest = {}
clients = {}
def received(msg):
    if msg.name in names: latest[msg.name] = (msg, time.monotonic())
sub = node.create_subscription(MotorState, ns+'/motor_states', received, 10)
pubs = [node.create_publisher(Float64, ns+'/motor/'+name+'/cmd_velocity', 1) for name in names]
def spin(seconds):
    end = time.monotonic()+seconds
    while time.monotonic()<end: rclpy.spin_once(node, timeout_sec=.005)
def send(left=0., right=0.):
    for pub, value in zip(pubs, [left,right]): pub.publish(Float64(data=value))
def call(name, kind=Trigger, request=None):
    client = clients.setdefault(name, node.create_client(kind, ns+'/'+name))
    if not client.wait_for_service(timeout_sec=5): raise RuntimeError('服务未就绪 '+name)
    future = client.call_async(request or kind.Request())
    end = time.monotonic()+6
    while not future.done() and time.monotonic()<end: spin(.01)
    if not future.done(): raise RuntimeError('服务超时 '+name)
    result = future.result()
    if not result.success: raise RuntimeError(name+': '+result.message)
def healthy(enabled=False):
    for name in names:
        if name not in latest: raise RuntimeError(name+' 无反馈')
        m, stamp = latest[name]
        if time.monotonic()-stamp>.5 or not all([m.feedback_fresh,m.position_valid,m.velocity_valid,m.status_valid,m.mode_valid]):
            raise RuntimeError(name+' 反馈无效或过期')
        if m.fault or m.mode != MotorState.MODE_VELOCITY or not math.isfinite(m.joint_velocity) or abs(m.joint_velocity)>2.:
            raise RuntimeError(name+' 故障、模式异常或速度超限')
        if enabled and not m.enabled: raise RuntimeError(name+' 意外失能')
def zero_wait(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end: send();spin(.02)
class Window(QtWidgets.QWidget):
    def __init__(self):
        super().__init__();self.keys=set();self.failed=False
        self.setWindowTitle('底盘键盘控制 — can0');self.resize(680,280)
        layout=QtWidgets.QVBoxLayout(self)
        layout.addWidget(QtWidgets.QLabel('按住 W 前进 / S 后退 / A 左转 / D 右转\n松键、空格或窗口失焦停车；Esc 退出并失能\n轮轴速度 0.5 rad/s'))
        self.status=QtWidgets.QLabel();layout.addWidget(self.status)
        stop=QtWidgets.QPushButton('停止并退出');stop.clicked.connect(self.close);layout.addWidget(stop)
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(20)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
    def keyPressEvent(self,e):
        if e.isAutoRepeat(): return
        if e.key()==QtCore.Qt.Key_Escape: self.close();return
        if e.key()==QtCore.Qt.Key_Space: self.keys.clear();send();return
        self.keys.add(e.key())
    def keyReleaseEvent(self,e):
        if not e.isAutoRepeat(): self.keys.discard(e.key());send()
    def event(self,e):
        if e.type() in (QtCore.QEvent.WindowDeactivate,QtCore.QEvent.FocusOut) and hasattr(self,'keys'): self.keys.clear();send()
        return super().event(e)
    def tick(self):
        # Each driver cycle publishes one state per wheel; drain both.
        for _ in range(12): rclpy.spin_once(node,timeout_sec=0)
        try:
            if driver.poll() is not None: raise RuntimeError('驱动进程退出')
            healthy(True)
            if self.failed: send();return
            if not self.isActiveWindow(): self.keys.clear()
            f=int(QtCore.Qt.Key_W in self.keys)-int(QtCore.Qt.Key_S in self.keys)
            t=int(QtCore.Qt.Key_A in self.keys)-int(QtCore.Qt.Key_D in self.keys)
            left,right=f-t,f+t
            scale=.5/max(1,abs(left),abs(right));send(left*scale,right*scale)
            self.status.setText('就绪   左轮 %.3f / 右轮 %.3f rad/s'%tuple(latest[n][0].joint_velocity for n in names))
        except Exception as error:
            print('控制错误: '+str(error), flush=True)
            print('反馈详情: '+repr({n: (str(m), time.monotonic()-stamp) for n,(m,stamp) in latest.items()}), flush=True)
            self.failed=True;self.keys.clear();send();self.status.setText('已停止：'+str(error))
            self.timer.stop()
            for op in ('halt','disable'):
                try: call(op)
                except Exception as exc: print(exc,flush=True)
    def closeEvent(self,e): self.timer.stop();self.keys.clear();send();e.accept()
driver=None
try:
    log=(folder/'driver.log').open('w')
    driver=subprocess.Popen([str(Path(get_package_prefix('can_driver'))/'lib/can_driver/can_driver_node'), '--ros-args','-r','__ns:='+ns,'-p','config_file:='+str(folder/'config.yaml')],stdout=log,stderr=subprocess.STDOUT)
    print('日志目录:',folder,flush=True)
    call('init',Init,Init.Request(device='can0',loopback=False))
    spin(.6);healthy()
    if any(latest[n][0].enabled or abs(latest[n][0].joint_velocity)>.05 for n in names): raise RuntimeError('初始电机未失能或未停稳')
    call('resume');zero_wait(.2);call('enable');call('resume');zero_wait(.3);healthy(True)
    app=QtWidgets.QApplication([]);window=Window();window.show();window.activateWindow();window.setFocus()
    signal.signal(signal.SIGINT,lambda *_:window.close())
    signal.signal(signal.SIGTERM,lambda *_:window.close())
    print('READY: 双电机已使能，等待用户键盘输入。',flush=True)
    app.exec_()
except Exception as error: print('启动失败:',error,flush=True)
finally:
    if driver is not None:
        zero_wait(.5)
        for op in ('halt','disable'):
            try:call(op)
            except Exception as error:print('停车:',error,flush=True)
        spin(.3)
        print('停止反馈:',[(n,latest[n][0].enabled,latest[n][0].joint_velocity) for n in names if n in latest],flush=True)
        try:call('shutdown',Shutdown,Shutdown.Request(force=False))
        except Exception as error:print('关闭:',error,flush=True)
        if driver.poll() is None:
            driver.send_signal(signal.SIGINT)
            try:driver.wait(timeout=5)
            except subprocess.TimeoutExpired:driver.kill();driver.wait()
    node.destroy_node();rclpy.shutdown()
