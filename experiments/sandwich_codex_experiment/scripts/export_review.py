"""Create a labeled-speed review copy; preserve the raw screen recording."""
import json
from pathlib import Path
import subprocess
import sys

out=Path(sys.argv[1]);raw=out/'gazebo_recording.mp4'
probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries',
    'format=duration,size:stream=width,height','-of','json',str(raw)],text=True))
native=(out/'capture_window.json').exists()
crop=[] if native else ['crop=852:566:80:42']
subprocess.run(['ffmpeg','-nostdin','-v','error','-sseof','-1','-i',str(raw),
    '-vf',','.join(crop) if crop else 'null','-frames:v','1',str(out/'final_scene.png')],check=True)
subprocess.run(['ffmpeg','-nostdin','-v','error','-i',str(raw),'-vf',
    ','.join(crop+['setpts=PTS/4','fps=16']),'-c:v','libx264','-preset','fast','-crf','25',
    '-an','-movflags','+faststart','-metadata','comment=4x playback of actual Gazebo screen recording',
    str(out/'gazebo_review_4x.mp4')],check=True)
(out/'video_metadata.json').write_text(json.dumps(dict(raw=probe,review_playback_speed=4,
    raw_recording_preserved=True,native_window_capture=native),indent=2)+'\n')
