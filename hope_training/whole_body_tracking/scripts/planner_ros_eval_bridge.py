"""Runs ONLY the real C++ ROS planner and simulated-input adapter in hope.

Private Unix socket connects host MuJoCo to this ROS-only process. No robot
runner, command bridge, mocap relay, or hardware launch file is started.
"""
import argparse,json,os,socket,subprocess,time,threading,signal
from pathlib import Path


def main():
    p=argparse.ArgumentParser();p.add_argument('--socket',required=True);p.add_argument('--config',required=True)
    p.add_argument('--planner',required=True);p.add_argument('--namespace',required=True);p.add_argument('--output',required=True)
    a=p.parse_args();out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Pose,PoseArray,PoseStamped
    from std_msgs.msg import Float64MultiArray
    from diagnostic_msgs.msg import DiagnosticArray
    rclpy.init();node=Node('eval_bridge',namespace=a.namespace)
    ns='/'+a.namespace.strip('/');allowed={(node.get_name(),ns),('hope_planner',ns)}
    commands=[];diagnostics=[];lock=threading.Lock();error=[]
    def cmd(m):
        with lock:commands.append({'received_unix_ns':time.time_ns(),'values':list(m.data)})
    def diag(m):
        with lock:diagnostics.append({'received_unix_ns':time.time_ns(),'status':[{'name':s.name,'message':s.message,'values':{v.key:v.value for v in s.values}} for s in m.status]})
    bp=node.create_publisher(PoseArray,ns+'/poses',qos_profile_sensor_data)
    rp=node.create_publisher(PoseStamped,ns+'/base',qos_profile_sensor_data)
    node.create_subscription(Float64MultiArray,ns+'/command',cmd,10)
    node.create_subscription(DiagnosticArray,ns+'/diagnostics',diag,10)
    log=(out/'planner.log').open('x');planner=None;server=None;conn=None;owns_socket=False
    stop=threading.Event()
    def stop_owned(signum,frame):
        stop.set()
        if conn is not None:
            try:conn.shutdown(socket.SHUT_RDWR)
            except OSError:pass
        elif server is not None:server.close()
    signal.signal(signal.SIGTERM,stop_owned)
    def spin():
        try:
            while not stop.is_set():rclpy.spin_once(node,timeout_sec=.005)
        except Exception as e:error.append(repr(e))
    thread=threading.Thread(target=spin);thread.start()
    def graph_check():
        names=set(node.get_node_names_and_namespaces())
        extra=names-allowed
        if extra:raise RuntimeError('Unrelated ROS nodes in isolated domain: '+str(extra))
    try:
        time.sleep(2);graph_check()
        argv=[a.planner,'--ros-args','--params-file',a.config,'-r','__ns:='+ns,
              '-r','/poses:='+ns+'/poses','-r','/racket/command_flat:='+ns+'/command',
              '-r','/planner/diagnostics:='+ns+'/diagnostics','-r','/racket/command:='+ns+'/typed']
        planner=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT)
        (out/'ownership.json').write_text(json.dumps({'bridge_pid':os.getpid(),'planner_pid':planner.pid,'output':str(out),'argv':argv})+'\n')
        end=time.monotonic()+15
        while bp.get_subscription_count()<1 or rp.get_subscription_count()<1:
            if stop.is_set() or planner.poll() is not None or time.monotonic()>end:raise RuntimeError('Planner exited or ROS discovery timed out; inspect planner.log')
            time.sleep(.05)
        graph_check()
        path=Path(a.socket)
        if path.exists():raise RuntimeError('Refusing existing bridge socket')
        server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);server.bind(a.socket);owns_socket=True;os.chmod(a.socket,0o600);server.listen(1);server.settimeout(30)
        conn,_=server.accept();conn.settimeout(5);stream=conn.makefile('rwb');last_stamp=0;last_graph=0
        with (out/'sensor-and-planner.jsonl').open('x') as trace:
            for line in stream:
                request=json.loads(line)
                # Host sends a one-way close notification then closes its socket.
                # Writing an unused acknowledgement races that close (EPIPE).
                if request.get('close'):break
                if planner.poll() is not None or error:raise RuntimeError('Planner/ROS spin process failed')
                now=time.monotonic()
                if now-last_graph>1:graph_check();last_graph=now
                stamp=int(request['stamp_ns'])
                if stamp<=last_stamp:raise RuntimeError('Nonmonotonic exposure timestamps')
                last_stamp=stamp
                if abs(time.time_ns()-stamp)>50_000_000:raise RuntimeError('Exposure/wall clock skew exceeds 50ms')
                sec,nsec=divmod(stamp,1_000_000_000)
                base=PoseStamped();base.header.frame_id='planner_table';base.header.stamp.sec=sec;base.header.stamp.nanosec=nsec
                base.pose.position.x,base.pose.position.y,base.pose.position.z=request['base_table']
                base.pose.orientation.w,base.pose.orientation.x,base.pose.orientation.y,base.pose.orientation.z=request['base_quat_wxyz']
                rp.publish(base)
                if request['ball_active']:
                    ball=PoseArray();ball.header=base.header;pose=Pose()
                    pose.position.x,pose.position.y,pose.position.z=request['ball_table']
                    pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z=request['ball_quat_wxyz']
                    ball.poses.append(pose);bp.publish(ball)
                with lock:
                    response={'commands':commands[:],'diagnostics':diagnostics[:],'reply_unix_ns':time.time_ns()}
                    commands.clear();diagnostics.clear()
                trace.write(json.dumps({'input':request,'output':response})+'\n')
                stream.write((json.dumps(response)+'\n').encode());stream.flush()
    finally:
        if conn:conn.close()
        if server:server.close()
        if owns_socket and Path(a.socket).exists():Path(a.socket).unlink()
        if planner and planner.poll() is None:
            planner.terminate()
            try:planner.wait(timeout=5)
            except subprocess.TimeoutExpired:planner.kill();planner.wait()
        stop.set();thread.join(timeout=2);log.close();node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
