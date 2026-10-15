#!/usr/bin/env python3
"""Observe the original ROS image capture without running or commanding auto aim."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from simulator_baseline import hashes

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT/'bevy_robomaster_simulator'
OUT = ROOT/'sp_vision_ros/artifacts'

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    before=hashes()
    rclpy.init()
    node = rclpy.create_node('original_capture_observer')
    images = []; infos = []; last = [None]
    def on_image(msg):
        images.append({'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                       'received':time.time(),'width':msg.width,'height':msg.height,
                       'encoding':msg.encoding,'step':msg.step})
        last[0]=msg
    def on_info(msg):
        infos.append({'width':msg.width,'height':msg.height,'k':list(msg.k),'d':list(msg.d),
                      'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9})
    node.create_subscription(Image,'/image_raw',on_image,qos_profile_sensor_data)
    node.create_subscription(CameraInfo,'/camera_info',on_info,qos_profile_sensor_data)
    env = os.environ.copy()
    for key in ['DAEDALUS_SP_SCENARIO','DAEDALUS_AUTO_AIM','DAEDALUS_METRICS_PATH']:
        env.pop(key,None)
    rustlib=subprocess.check_output(['rustc','--print','target-libdir'],text=True).strip()
    env['LD_LIBRARY_PATH']=str(SIM/'target/debug/deps')+':'+rustlib+':'+env.get('LD_LIBRARY_PATH','')
    env['BEVY_ASSET_ROOT']=str(SIM)
    env['RUST_LOG']='daedalus=info,bevy_diagnostic=info,bevy_render=info,wgpu=warn'
    report={'variant':'original-ros-with-import-and-center-guards','frames':images,'camera_info':infos}
    with (OUT/'baseline_ros_simulator.log').open('w') as log:
        child=subprocess.Popen([str(SIM/'target/debug/daedalus'),'--duration-secs','40'],
                               cwd=SIM,env=env,start_new_session=True,stdout=log,stderr=subprocess.STDOUT)
        start=time.monotonic()
        try:
            while child.poll() is None and time.monotonic()-start<180:
                rclpy.spin_once(node,timeout_sec=.02)
        finally:
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
            node.destroy_node();rclpy.shutdown()
    if last[0] is not None:
        msg=last[0]
        image=np.frombuffer(bytes(msg.data),np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3]
        image=image.reshape(msg.height,msg.width,3)
        if msg.encoding=='rgb8':image=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
        cv2.imwrite(str(OUT/'baseline_ros_image.png'),image)
    report['exit_code']=child.returncode
    report['image_count']=len(images)
    report['info_count']=len(infos)
    ages=[m['received']-m['stamp'] for m in images]
    report['header_to_receipt_ms_median']=float(np.median(ages)*1000) if ages else None
    report['header_to_receipt_ms_p95']=float(np.percentile(ages,95)*1000) if ages else None
    report['source_unchanged']=hashes()==before
    report['status']='PASS' if child.returncode==0 and len(images)>20 and len(infos)>20 else 'FAIL'
    if not report['source_unchanged']:report['status']='FAIL'
    (OUT/'baseline_ros_result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ['frames','camera_info']},indent=2))
    return 0 if report['status']=='PASS' else 1

if __name__=='__main__':
    raise SystemExit(main())
