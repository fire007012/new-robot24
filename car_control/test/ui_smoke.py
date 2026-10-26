#!/usr/bin/env python3
"""Open every native Qt UI and exercise actual ROS2 service requests."""
import os,time,threading,json
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
os.environ.setdefault('ROS_DOMAIN_ID','81')
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from PyQt5 import QtWidgets,QtCore
from robot_hardware.srv import GroupOperation,SetProfile,Maintain
from car_control_runtime.gui import ControlWindow
from std_msgs.msg import String
rclpy.init();app=QtWidgets.QApplication([]);server=Node('ui_test_services');calls=[]
def response(req,res):
    calls.append(req);res.success=True;res.message='ui-test-ok';return res
server.create_service(GroupOperation,'/robot_runtime/group_operation',response)
server.create_service(SetProfile,'/robot_runtime/set_flipper_profile',response)
server.create_service(Maintain,'/robot_runtime/maintain',response)
executor=SingleThreadedExecutor();executor.add_node(server);thread=threading.Thread(target=executor.spin);thread.start()
def pump(predicate=lambda:False,seconds=2):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        app.processEvents();time.sleep(.01)
        if predicate():return True
    return predicate()
try:
    for role in ('keyboard','flippers','monitor'):
        window=ControlWindow(role);window.show();window.activateWindow();pump(seconds=.5)
        if role!='monitor':
            window.operate('init')
            # Service discovery can take a short time; retry only because the first
            # unavailable request was explicitly not enqueued by the UI.
            pump(seconds=.5)
            if not window.pending and 'ui-test-ok' not in window.result.toPlainText():window.operate('init')
            assert pump(lambda:'ui-test-ok' in window.result.toPlainText(),5),window.result.toPlainText()
            assert calls[-1].operation=='init' and len(calls[-1].groups)==4
            window.rpc(SetProfile,'/robot_runtime/set_flipper_profile',SetProfile.Request(profile='csv'))
            pump(seconds=.4)
            if not any(isinstance(req,SetProfile.Request) for req in calls):
                window.rpc(SetProfile,'/robot_runtime/set_flipper_profile',SetProfile.Request(profile='csv'));pump(seconds=.4)
            assert any(isinstance(req,SetProfile.Request) for req in calls)
        window.keys={'w'};window.jog=[.1]*4
        state={'state':'Running','groups':{'arm':'Running','flippers':'Running'},'operations':{'arm':{'epoch':2}}}
        window.received_state(String(data=json.dumps(state)))
        assert not window.keys and window.jog==[0.]*4,'runtime epoch must clear all held input'
        window.keys={'w'};window.event(QtCore.QEvent(QtCore.QEvent.WindowDeactivate));assert not window.keys
        window.table.header().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        window.grab().save('/tmp/robot24-ui-'+role+'.png');window.close();pump(seconds=.1)
        print(role+' native ROS2 UI startup, RPC, epoch/focus clear and close passed',flush=True)
finally:
    executor.shutdown();thread.join();server.destroy_node();rclpy.shutdown()
