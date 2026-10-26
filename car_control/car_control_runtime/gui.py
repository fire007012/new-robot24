"""Native ROS2 Qt interfaces. Qt owns widgets; ROS callbacks never block the UI."""
import json
import math
import sys
import time
import rclpy
from rclpy.node import Node
from PyQt5 import QtCore, QtWidgets
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TwistStamped
from control_msgs.msg import JointJog
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robot_hardware.srv import GroupOperation, SetProfile, Maintain
from controller_manager_msgs.srv import ListControllers
from .teleop import KeyboardTeleopNode, FLIPPERS

class ControlWindow(QtWidgets.QMainWindow):
    def __init__(self,role='keyboard'):
        super().__init__();self.role=role;self.setWindowTitle('Robot24 ROS2 — '+role);self.resize(1080,740)
        self.node=Node('robot24_'+role+'_ui');self.translator=KeyboardTeleopNode() if role=='keyboard' else None
        self.state={};self.state_at=0.;self.pending=[];self.keys=set();self.pulses=set();self.jog=[0.]*4;self.jog_epoch=None
        self.positions={};self.velocities={};self.samples={};self.clients={}
        self.node.create_subscription(String,'/robot_runtime/state',self.received_state,1)
        self.node.create_subscription(JointState,'/joint_states',self.received_joints,1)
        self.node.create_subscription(String,'/car_control/keyboard_teleop/status',self.keyboard_status,1)
        for kind,topic,name in [(Twist,'/cmd_vel','base'),(TwistStamped,'/servo_server/delta_twist_cmds','servo'),(JointJog,'/flipper_control/jog_cmd','flippers')]:
            self.node.create_subscription(kind,topic,lambda m,n=name:self.samples.update({n:str(m)}),1)
        self.raw=self.node.create_publisher(String,'/car_control/keyboard_teleop/raw_state',1)
        self.jog_pub=self.node.create_publisher(JointJog,'/flipper_control/jog_cmd',1)
        self.csp=self.node.create_publisher(JointTrajectory,'/flipper_control/command',1)
        root=QtWidgets.QWidget();self.setCentralWidget(root);layout=QtWidgets.QVBoxLayout(root)
        self.banner=QtWidgets.QLabel('等待 ROS2 运行态');layout.addWidget(self.banner)
        if role!='monitor':
            bar=QtWidgets.QHBoxLayout();layout.addLayout(bar)
            self.group=QtWidgets.QComboBox();self.group.addItems(['all','arm','base','flippers','gripper']);bar.addWidget(self.group)
            for op in ('init','enable','resume','halt','recover','shutdown'):
                b=QtWidgets.QPushButton(op);b.clicked.connect(lambda _,operation=op:self.operate(operation));bar.addWidget(b)
            self.group.setToolTip('操作作用组；不会自动使能或恢复运动')
        tabs=QtWidgets.QTabWidget();layout.addWidget(tabs)
        telemetry=QtWidgets.QWidget();tl=QtWidgets.QVBoxLayout(telemetry)
        self.table=QtWidgets.QTreeWidget();self.table.setHeaderLabels(['Joint','position (rad / m)','velocity (rad/s / m/s)']);self.table.header().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents);tl.addWidget(self.table)
        self.details=QtWidgets.QPlainTextEdit();self.details.setReadOnly(True);tl.addWidget(self.details)
        tabs.addTab(telemetry,'运行态与关节')
        if role!='monitor':
            commands=QtWidgets.QWidget();cl=QtWidgets.QVBoxLayout(commands)
            cl.addWidget(QtWidgets.QLabel('键盘：Tab 底盘/机械臂，1–5 速度，Enter 停止；底盘 WASD；摆臂 Y/H U/J I/K O/L；机械臂 WASD U/O Q/E I/K J/L，夹爪 F/H。\n机械臂点动需显式启用 Servo 模式；失焦和松开按键立即清空输入。'))
            modebar=QtWidgets.QHBoxLayout();cl.addLayout(modebar)
            if self.translator:
                b=QtWidgets.QPushButton('切换底盘／机械臂');b.clicked.connect(self.toggle_mode);modebar.addWidget(b)
            from std_srvs.srv import SetBool
            for mode,enabled in [('Servo 模式',True),('轨迹模式',False)]:
                b=QtWidgets.QPushButton(mode);b.clicked.connect(lambda _,value=enabled:self.rpc(SetBool,'/arm_control/set_servo_mode',SetBool.Request(data=value)));modebar.addWidget(b)
            for profile in ('csv','csp'):
                b=QtWidgets.QPushButton('摆臂 '+profile);b.clicked.connect(lambda _,p=profile:self.rpc(SetProfile,'/robot_runtime/set_flipper_profile',SetProfile.Request(profile=p)));modebar.addWidget(b)
            self.linkage=QtWidgets.QComboBox();self.linkage.addItems(['independent','left_right_mirror','front_rear_sync','side_pair','diagonal_pair']);modebar.addWidget(self.linkage)
            b=QtWidgets.QPushButton('应用联动');b.clicked.connect(lambda:self.rpc(SetProfile,'/flipper_control/set_linkage_mode',SetProfile.Request(profile=self.linkage.currentText())));modebar.addWidget(b)
            grid=QtWidgets.QGridLayout();cl.addLayout(grid);self.targets=[]
            for i,name in enumerate(FLIPPERS):
                grid.addWidget(QtWidgets.QLabel(name),i,0)
                for col,sign in ((1,-1),(2,1)):
                    b=QtWidgets.QPushButton('按住 −' if sign<0 else '按住 +');b.setAutoRepeat(False)
                    b.pressed.connect(lambda idx=i,v=sign*.1:self.set_jog(idx,v));b.released.connect(self.stop_input);grid.addWidget(b,i,col)
                spin=QtWidgets.QDoubleSpinBox();spin.setDecimals(3);spin.setRange(-3.665 if i<2 else -.524,.524 if i<2 else 3.665);self.targets.append(spin);grid.addWidget(spin,i,3)
            b=QtWidgets.QPushButton('发送四轴 CSP 目标（2 秒）');b.clicked.connect(self.send_csp);cl.addWidget(b)
            tabs.addTab(commands,'操作与点动')
            maintenance=QtWidgets.QWidget();ml=QtWidgets.QFormLayout(maintenance)
            self.axis=QtWidgets.QLineEdit('left_front_arm_joint');ml.addRow('Joint',self.axis)
            self.operation=QtWidgets.QComboBox();self.operation.addItems(['set_zero','set_offset','limits','limits_from_urdf','disable_limits','restore']);ml.addRow('Operation',self.operation)
            self.lower=QtWidgets.QDoubleSpinBox();self.upper=QtWidgets.QDoubleSpinBox()
            for w in (self.lower,self.upper):w.setRange(-100,100);w.setDecimals(6)
            ml.addRow('Lower / offset (SI)',self.lower);ml.addRow('Upper (SI)',self.upper)
            self.save=QtWidgets.QCheckBox('写入设备持久化参数');ml.addRow(self.save)
            b=QtWidgets.QPushButton('执行维护操作');b.clicked.connect(lambda:self.rpc(Maintain,'/robot_runtime/maintain',Maintain.Request(joint=self.axis.text(),operation=self.operation.currentText(),lower=self.lower.value(),upper=self.upper.value(),save=self.save.isChecked())));ml.addRow(b)
            b=QtWidgets.QPushButton('读取控制器状态');b.clicked.connect(lambda:self.rpc(ListControllers,'/controller_manager/list_controllers',ListControllers.Request()));ml.addRow(b)
            tabs.addTab(maintenance,'维护')
        self.result=QtWidgets.QPlainTextEdit();self.result.setReadOnly(True);self.result.setMaximumHeight(105);layout.addWidget(self.result)
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(30)
        QtWidgets.QApplication.instance().focusChanged.connect(lambda old,new:self.stop_input())
        QtWidgets.QApplication.instance().applicationStateChanged.connect(lambda state:self.stop_input() if state!=QtCore.Qt.ApplicationActive else None)

    def keyboard_status(self,msg):
        try:self.samples.update(keyboard=json.loads(msg.data))
        except (ValueError,TypeError):self.samples.pop("keyboard",None)

    def toggle_mode(self):
        self.stop_input();self.pulses.add('tab')

    def received_state(self,msg):
        try:
            state=json.loads(msg.data)
            epoch=json.dumps([state.get('groups',{}),state.get('operations',{})],sort_keys=True)
            if epoch!=self.jog_epoch or time.monotonic()-self.state_at>.3:self.stop_input()
            self.jog_epoch=epoch;self.state=state;self.state_at=time.monotonic()
        except (ValueError,TypeError):self.state={};self.stop_input()

    def received_joints(self,msg):
        self.positions=dict(zip(msg.name,msg.position));self.velocities=dict(zip(msg.name,msg.velocity))

    def rpc(self,kind,path,request):
        if path in {p[0] for p in self.pending}:self.result.appendPlainText('已有同类操作进行中');return
        client=self.clients.get(path)
        if client is None:client=self.node.create_client(kind,path);self.clients[path]=client
        if not client.service_is_ready():self.result.appendPlainText(path+' 不可用；未排队重放');return
        future=client.call_async(request);self.pending.append((path,future,time.monotonic()+30))

    def operate(self,op):
        self.stop_input();group=self.group.currentText()
        self.rpc(GroupOperation,'/robot_runtime/group_operation',GroupOperation.Request(operation=op,groups=['arm','base','flippers','gripper'] if group=='all' else [group]))

    def set_jog(self,index,value):
        self.jog=[0.]*4;self.jog[index]=value

    def stop_input(self):self.keys=set();self.pulses=set();self.jog=[0.]*4

    def send_csp(self):
        if self.state.get('profile')!='csp':self.result.appendPlainText('请先切换摆臂 CSP 模式');return
        q=[w.value() for w in self.targets]
        if not all(math.isfinite(v) for v in q):return
        point=JointTrajectoryPoint(positions=q);point.time_from_start.sec=2
        self.csp.publish(JointTrajectory(joint_names=FLIPPERS,points=[point]))

    @staticmethod
    def key(event):
        mapping={QtCore.Qt.Key_Tab:'tab',QtCore.Qt.Key_Return:'enter',QtCore.Qt.Key_Enter:'enter',QtCore.Qt.Key_Escape:'enter'}
        return mapping.get(event.key(),event.text().lower())

    def keyPressEvent(self,event):
        if event.isAutoRepeat():return
        key=self.key(event)
        if key in ('tab','enter','1','2','3','4','5'):self.pulses.add(key)
        elif len(key)==1:self.keys.add(key)

    def keyReleaseEvent(self,event):
        if not event.isAutoRepeat():self.keys.discard(self.key(event))

    def event(self,event):
        if event.type()==QtCore.QEvent.WindowDeactivate and hasattr(self,'keys'):self.stop_input()
        return super().event(event)

    def tick(self):
        rclpy.spin_once(self.node,timeout_sec=0)
        if self.translator:rclpy.spin_once(self.translator,timeout_sec=0)
        editing=isinstance(QtWidgets.QApplication.focusWidget(),(QtWidgets.QLineEdit,QtWidgets.QAbstractSpinBox,QtWidgets.QPlainTextEdit))
        active=self.isActiveWindow() and not editing;fresh=time.monotonic()-self.state_at<.3
        if not active or not fresh:self.stop_input()
        if self.translator:
            self.raw.publish(String(data=json.dumps({'pressed':sorted(self.keys),'pulse':sorted(self.pulses),'focused':active})));self.pulses.clear()
        if self.role=='flippers':
            running=self.state.get('groups',{}).get('flippers')=='Running'
            self.jog_pub.publish(JointJog(joint_names=FLIPPERS,velocities=self.jog if fresh and active and running else [0.]*4,duration=.15))
        remaining=[]
        for path,future,deadline in self.pending:
            if future.done():
                try:self.result.appendPlainText(path+' → '+str(future.result()))
                except Exception as error:self.result.appendPlainText(str(error))
            elif time.monotonic()>deadline:self.result.appendPlainText(path+' 等待超时；结果不确定，请核对运行态')
            else:remaining.append((path,future,deadline))
        self.pending=remaining
        self.banner.setText(('ROS2 '+self.state.get('state','未知')) if fresh else '运行态过期／未连接，运动输入已清空')
        self.table.clear()
        for name,q in sorted(self.positions.items()):QtWidgets.QTreeWidgetItem(self.table,[name,f'{q:.5f}',f'{self.velocities.get(name,float("nan")):.5f}'])
        self.details.setPlainText(json.dumps({'runtime':self.state,'signals':self.samples},ensure_ascii=False,indent=2))

    def closeEvent(self,event):
        self.timer.stop();self.stop_input()
        if self.translator:self.translator.clear();self.translator.step();self.translator.destroy_node()
        if self.role=='flippers':self.jog_pub.publish(JointJog(joint_names=FLIPPERS,velocities=[0.]*4,duration=.15))
        self.node.destroy_node();event.accept()


def main(role='keyboard'):
    rclpy.init();app=QtWidgets.QApplication(sys.argv);window=ControlWindow(role);window.show()
    smoke='--smoke-test' in sys.argv
    if smoke:QtCore.QTimer.singleShot(600,window.close)
    code=app.exec_()
    if rclpy.ok():rclpy.shutdown()
    return code
