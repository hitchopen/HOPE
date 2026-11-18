import json,subprocess,shutil
from pathlib import Path
w=Path(__file__).resolve().parent
src=w.parent/'supergate-opt400-v5/collision-backend'
s=(src/'native_batch.cpp').read_text()
s=s.replace(' int foot_body[2];',' std::ofstream stance;\n int foot_body[2];')
s=s.replace('  observations.open(','''  stance.open(ss.out+"/env-"+std::to_string(id)+"-stance.csv");
  stance<<"time,shot,left_x,left_y,left_yaw,right_x,right_y,right_yaw\\n"<<std::setprecision(12);
  observations.open(''')
s=s.replace('  feet<<d->time', '''  stance<<d->time<<','<<current+1;
  for(int body:foot_body){const auto* pos=diagnostic->xpos+3*body;stance<<','<<pos[0]<<','<<pos[1]<<','<<yaw(diagnostic->xmat+9*body);}
  stance<<'\\n';
  feet<<d->time''')
(w/'native_batch.cpp').write_text(s)
shutil.copy(src/'physics.hpp',w/'physics.hpp')
cmd=[t.replace(str(src),str(w)) for t in json.loads((src/'build-command.json').read_text())]
subprocess.run(cmd,check=True)
script=(src.parent/'collision_run.py').read_text().replace(str(src/'native_batch.so'),str(w/'native_batch.so'))
(w/'run.py').write_text(script)
