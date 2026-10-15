#!/usr/bin/env python3
"""Exercise exact frame pairing, ROS message types, blank detection and stale-stop."""
import json
import math
import signal
import subprocess
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from rm_interfaces.msg import SpGimbalPlan

ROOT=Path(__file__).resolve().parents[1]

def main():
    rclpy.init(); node=rclpy.create_node('sp_sim_smoke')
    reports=[]; plans=[]
    subs=[node.create_subscription(String,'/sp_vision/sim/diagnostics',lambda m:reports.append(json.loads(m.data)),qos_profile_sensor_data),
          node.create_subscription(SpGimbalPlan,'/sp_vision/sim/plan',lambda m:plans.append(m),qos_profile_sensor_data)]
    pubs=[node.create_publisher(Image,'/image_raw',10),node.create_publisher(CameraInfo,'/camera_info',10),
          node.create_publisher(TFMessage,'/sp_vision/sim/frame_tf',10)]
    log=(ROOT/'artifacts/ros_smoke_node.log').open('w')
    child=subprocess.Popen(['bash',str(ROOT/'scripts/run_aim.sh')],stdout=log,stderr=subprocess.STDOUT)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if child.poll() is not None: raise RuntimeError(f'node exited {child.returncode}; see ros_smoke_node.log')
            rclpy.spin_once(node,timeout_sec=.03)
    def frame(bad_k=False,unpaired=False):
        g=json.loads((ROOT/'configs/geometry.json').read_text());stamp=node.get_clock().now().to_msg()
        image=Image();image.header.stamp=stamp;image.header.frame_id='camera_optical_frame'
        image.width=g['width'];image.height=g['height'];image.encoding='rgb8';image.step=image.width*3
        image.data=bytes(image.step*image.height)
        info=CameraInfo();info.header=image.header;info.width=image.width;info.height=image.height
        info.k=[v for row in g['camera_matrix'] for v in row];info.d=[0.]*5
        if bad_k: info.k[0]+=10
        gun=TransformStamped();gun.header.stamp=stamp;gun.header.frame_id='map';gun.child_frame_id='sp_gimbal';gun.transform.rotation.w=1.
        cam=TransformStamped();cam.header.stamp=stamp;cam.header.frame_id='sp_gimbal';cam.child_frame_id='sp_camera_optical'
        cam.transform.translation.x,cam.transform.translation.y,cam.transform.translation.z=g['t_camera2gimbal']
        # Quaternion of [[0,0,1],[-1,0,0],[0,-1,0]].
        cam.transform.rotation.x=-.5;cam.transform.rotation.y=.5;cam.transform.rotation.z=-.5;cam.transform.rotation.w=.5
        tf=TFMessage(transforms=[gun,cam])
        # Deliberately publish in reverse order, before the large image.
        pubs[2].publish(tf);pubs[1].publish(info)
        if not unpaired: pubs[0].publish(image)
    try:
        deadline=time.monotonic()+25
        while not plans and time.monotonic()<deadline: spin(.2)
        assert plans,'no planner messages'
        assert all(not p.control and not p.fire for p in plans),'idle plan must not control/fire'
        for _ in range(5): frame();spin(.4)
        spin(2)
        valid=[r for r in reports if r.get('event')=='vision']
        assert valid,'no synchronized frame processed'
        assert all(r['armors']==0 and not r['has_target'] for r in valid)
        before=len(valid);frame(bad_k=True);spin(1)
        assert len([r for r in reports if r.get('event')=='vision'])==before,'mismatched calibration accepted'
        frame(unpaired=True);spin(1)
        assert len([r for r in reports if r.get('event')=='vision'])==before,'unpaired frame accepted'
        assert all(not p.control and not p.fire for p in plans[-20:]),'stale input must stop'
        result={'status':'PASS','valid_frames':len(valid),'plan_messages':len(plans),
                'checks':['ROS interfaces','exact timestamp pairing in reverse arrival order','RGB blank detection',
                          'reject changed intrinsics','reject unpaired frame','no-target and stale stop']}
        (ROOT/'artifacts/ros_smoke_result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    finally:
        child.send_signal(signal.SIGINT)
        try: child.wait(timeout=5)
        except subprocess.TimeoutExpired: child.terminate();child.wait(timeout=5)
        log.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__': main()
