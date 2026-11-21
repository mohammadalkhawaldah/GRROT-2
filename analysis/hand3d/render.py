#!/usr/bin/env python3
"""Video of image landmarks beside wrist-centered raw 3D model estimates."""
import csv
from pathlib import Path
import subprocess
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from geometry import EDGES,palm_pose

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'outputs/hand3d'


def main():
    data=np.load(OUT/'video/landmarks.npz')
    select={int(f):i for i,(f,p) in enumerate(zip(data['frame'],data['primary'])) if p}
    with (OUT/'primary_pose_quality.csv').open() as f:quality={int(r['frame']):r for r in csv.DictReader(f)}
    cap=cv2.VideoCapture(str(ROOT/'data/raw/2026-11-15 20-59-05.mkv'))
    # Display every second original frame at 15 fps; no time interpolation or
    # hole filling. Every original frame remains present in the CSVs.
    encoder=subprocess.Popen(['ffmpeg','-hide_banner','-loglevel','error','-y',
        '-f','rawvideo','-pix_fmt','bgr24','-s','1280x560','-r','15','-i','-',
        '-an','-c:v','libx264','-preset','veryfast','-crf','23','-threads','3',
        '-pix_fmt','yuv420p','-movflags','+faststart',str(OUT/'hand3d_review.mp4')],stdin=subprocess.PIPE)
    fig=plt.figure(figsize=(5.6,5.6),dpi=100)
    ax=fig.add_subplot(111,projection='3d')
    frame=0;keyframes=[]
    representative={116,180,246,282,306,310}
    try:
        while True:
            ok,image=cap.read()
            if not ok:break
            if frame%2:
                frame+=1;continue
            row=quality[frame];present=frame in select
            ax.clear()
            ax.set(xlim=(-130,130),ylim=(-80,180),zlim=(-130,130),
                xlabel='MP x (model mm)',ylabel='MP y (model mm)',zlabel='MP z (model mm)')
            ax.set_box_aspect((1,1,1));ax.view_init(elev=22,azim=-65)
            ax.set_title('Raw 3D model; wrist centered\nNOT calibrated camera/world coordinates',fontsize=10)
            if present:
                i=select[frame];world=data['world'][i];pixels=data['image'][i,:,:2]*[image.shape[1],image.shape[0]]
                centered=(world-world[0])*1000
                for a,b in EDGES:
                    cv2.line(image,tuple(pixels[a].astype(int)),tuple(pixels[b].astype(int)),(60,220,60),3)
                    ax.plot(*centered[[a,b]].T,color='#2563eb',lw=2)
                for x,y in pixels:cv2.circle(image,(int(x),int(y)),4,(20,50,240),-1)
                ax.scatter(*centered.T,s=12,color='#b45309')
                center,r,_,_=palm_pose(world);c=(center-world[0])*1000
                for k,color in enumerate(['r','g','b']):ax.quiver(*c,*(r[:,k]*35),color=color)
            else:ax.text2D(.12,.5,'NO HAND DETECTION',transform=ax.transAxes,color='red',fontsize=13)
            fig.canvas.draw()
            panel=cv2.cvtColor(np.asarray(fig.canvas.buffer_rgba()),cv2.COLOR_RGBA2BGR)
            canvas=np.full((560,1280,3),245,np.uint8)
            left=cv2.resize(image,(720,405));canvas[70:475,:720]=left;canvas[:,720:]=panel
            cv2.putText(canvas,f'Source {frame/30:.2f}s  frame {frame}  {row["phase_approx"]}',(12,30),cv2.FONT_HERSHEY_SIMPLEX,.6,(30,30,30),1,cv2.LINE_AA)
            cv2.putText(canvas,'IMAGE motion retained on left; no global 3D translation on right',(12,55),cv2.FONT_HERSHEY_SIMPLEX,.47,(40,40,40),1,cv2.LINE_AA)
            text='No detection'
            if present:
                text=f'{row.get("handedness_raw", "")} label score {float(row["handedness_score"]):.2f} (not tracking confidence)'
                if row.get('orientation_jump')=='True':text+='  RAW ROTATION JUMP'
            cv2.putText(canvas,text,(12,505),cv2.FONT_HERSHEY_SIMPLEX,.48,(20,20,200) if 'JUMP' in text else (20,20,20),1,cv2.LINE_AA)
            cv2.putText(canvas,'Raw estimates shown. Finger occlusion and depth ambiguity remain.',(12,537),cv2.FONT_HERSHEY_SIMPLEX,.47,(60,60,60),1,cv2.LINE_AA)
            encoder.stdin.write(canvas.tobytes())
            if frame in representative:keyframes.append(cv2.resize(canvas,(640,280)))
            frame+=1
            if frame%60==0:print(f'Rendered through frame {frame}',flush=True)
    finally:
        cap.release();plt.close(fig);encoder.stdin.close()
    if encoder.wait()!=0:raise RuntimeError('Video encoding failed')
    if len(keyframes)==6:cv2.imwrite(str(OUT/'review_keyframes.jpg'),np.vstack([np.hstack(keyframes[i:i+2]) for i in range(0,6,2)]))


if __name__=='__main__':main()
