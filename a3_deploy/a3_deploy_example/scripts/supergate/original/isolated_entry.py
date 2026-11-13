from pathlib import Path
import json,os,sys,subprocess
out=Path(sys.argv[1]);out.parent.mkdir(parents=True,exist_ok=True)
receipt={'pid_namespace':os.readlink('/proc/self/ns/pid'),'ipc_namespace':os.readlink('/proc/self/ns/ipc'),'net_namespace':os.readlink('/proc/self/ns/net'),'mount_namespace':os.readlink('/proc/self/ns/mnt'),'cpu_affinity':sorted(os.sched_getaffinity(0)),'proc_visible':sum(x.name.isdigit() for x in Path('/proc').iterdir()),'shm_before':list(str(x) for x in Path('/dev/shm').iterdir())}
(out.parent/(out.name+'-isolation.json')).write_text(json.dumps(receipt,indent=2))
raise SystemExit(subprocess.call([sys.executable,*sys.argv[2:]]))
