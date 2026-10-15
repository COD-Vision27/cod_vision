"""Measured camera and eye-in-hand calibration, metres and explicit pose frames."""
import hashlib,json,math
from pathlib import Path
import cv2 as cv
import numpy as np


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def dump(path,data):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')

def rotation(value):
    r=np.asarray(value,dtype=float).reshape(3,3)
    if not np.isfinite(r).all() or np.linalg.norm(r.T@r-np.eye(3))>1e-5 or abs(np.linalg.det(r)-1)>1e-5:
        raise ValueError('rotation must be a finite proper orthonormal matrix')
    return r

def quaternion(wxyz):
    a=np.asarray(wxyz,dtype=float)
    if a.shape!=(4,) or not np.isfinite(a).all() or abs(np.linalg.norm(a)-1)>.01:raise ValueError('invalid wxyz quaternion')
    w,x,y,z=a/np.linalg.norm(a)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def transform(r,t):
    a=np.eye(4);a[:3,:3]=rotation(r);a[:3,3]=np.asarray(t).reshape(3)
    if not np.isfinite(a).all():raise ValueError('nonfinite translation')
    return a

def angle(r):return math.degrees(math.acos(float(np.clip((np.trace(r)-1)/2,-1,1))))
def mean_pose(poses):
    u,_,v=np.linalg.svd(sum(a[:3,:3] for a in poses));r=u@np.diag([1,1,np.linalg.det(u@v)])@v
    return transform(r,np.mean([a[:3,3] for a in poses],axis=0))

def board_points(board):
    cols,rows=board['cols'],board['rows'];spacing=float(board['spacing_m'])
    if not isinstance(cols,int) or not isinstance(rows,int) or cols<3 or rows<3 or not 0<spacing<1:raise ValueError('invalid board count or spacing in metres')
    if board['type'] not in ['chessboard','circles']:raise ValueError('board type must be chessboard or symmetric circles')
    points=np.zeros((cols*rows,3),np.float32);points[:,:2]=np.mgrid[0:cols,0:rows].T.reshape(-1,2)*spacing
    return points

def dataset(folder,board):
    folder=Path(folder).resolve();manifest_path=folder/'dataset.json';manifest=json.loads(manifest_path.read_text())
    if manifest.get('schema')!='sp_capture_v1' or manifest.get('kind') not in ['hardware','synthetic']:raise ValueError('unsupported capture manifest')
    obj=board_points(board);size=tuple(manifest['image_size']);accepted=[];rejected=[];hashes={'dataset.json':digest(manifest_path)}
    if len(size)!=2 or any(not isinstance(x,int) or x<100 for x in size):raise ValueError('invalid image size')
    for sample in manifest['samples']:
        if manifest['kind']=='synthetic' and 'corners_px' in sample:
            points=np.asarray(sample['corners_px'],np.float32).reshape(-1,1,2)
        else:
            name=sample['image'];path=(folder/name).resolve()
            if not path.is_relative_to(folder) or not path.is_file():raise ValueError('image path missing or outside capture directory')
            hashes[name]=digest(path);image=cv.imread(str(path),cv.IMREAD_GRAYSCALE)
            if image is None or image.shape[::-1]!=size:raise ValueError('mixed resolution or unreadable image: '+name)
            pattern=(board['cols'],board['rows'])
            if board['type']=='chessboard':
                success,points=cv.findChessboardCornersSB(image,pattern,cv.CALIB_CB_NORMALIZE_IMAGE)
            else:success,points=cv.findCirclesGrid(image,pattern,flags=cv.CALIB_CB_SYMMETRIC_GRID)
            if not success:rejected.append({'image':name,'reason':'board not detected'});continue
        if len(points)!=len(obj) or not np.isfinite(points).all():raise ValueError('invalid corners')
        if np.any(points[...,0]<0) or np.any(points[...,0]>=size[0]) or np.any(points[...,1]<0) or np.any(points[...,1]>=size[1]):raise ValueError('corners outside image')
        accepted.append((sample,points))
    return manifest,size,obj,accepted,rejected,hashes

def project_error(obj,points,k,d,r,t):
    predicted=cv.projectPoints(obj,r,t,k,d)[0]
    return float(np.sqrt(np.mean(np.sum((predicted-points)**2,axis=2))))

def solve_pose(obj,points,k,d):
    ok,r,t=cv.solvePnP(obj,points,k,d,flags=cv.SOLVEPNP_ITERATIVE)
    if not ok or not np.isfinite(r).all() or not np.isfinite(t).all():raise ValueError('PnP failed')
    R=cv.Rodrigues(r)[0]
    if np.min((R@obj.T+t)[2])<=0:raise ValueError('PnP places board behind camera')
    return transform(R,t),project_error(obj,points,k,d,r,t)

def calibrate_camera(folder,board):
    manifest,size,obj,views,rejected,hashes=dataset(folder,board)
    if len(views)<20:raise ValueError('intrinsics require at least 20 detected views')
    centers=np.array([points.mean(axis=0).ravel()/size for _,points in views]);span=np.ptp(centers,axis=0)
    if span[0]<.25 or span[1]<.20:raise ValueError('intrinsic views do not cover enough image area (25% width / 20% height)')
    hold=[i for i in range(len(views)) if i%5==0];train=[i for i in range(len(views)) if i not in hold]
    rms,k,d,rs,ts=cv.calibrateCamera([obj for _ in train],[views[i][1] for i in train],size,None,None,
        flags=cv.CALIB_FIX_K3,criteria=(cv.TERM_CRITERIA_COUNT|cv.TERM_CRITERIA_EPS,100,1e-9))
    if not np.isfinite(k).all() or not np.isfinite(d).all() or k[0,0]<=0 or k[1,1]<=0:raise ValueError('invalid intrinsic solution')
    normals=np.array([cv.Rodrigues(r)[0][:,2] for r in rs]);tilt=max(angle(cv.Rodrigues(rs[i])[0].T@cv.Rodrigues(rs[j])[0]) for i in range(len(rs)) for j in range(i))
    errors=[solve_pose(obj,views[i][1],k,d)[1] for i in hold]
    normal_span=float(np.max(np.linalg.norm(normals-normals[0],axis=1)))
    passed=rms<=.5 and float(np.percentile(errors,95))<=.8 and normal_span>=.15
    return {'schema':'sp_intrinsics_v1','kind':manifest['kind'],'status':'PASS' if passed else 'FAIL','image_size':list(size),
        'config':{'camera_matrix':k.flatten().tolist(),'distort_coeffs':d.flatten().tolist()},
        'rms_px':float(rms),'holdout_rms_px':errors,'holdout_p95_px':float(np.percentile(errors,95)),
        'view_count':len(views),'training_count':len(train),'holdout_count':len(hold),'center_span_fraction':span.tolist(),
        'board_normal_span':normal_span,'max_pose_difference_deg':tilt,'rejected':rejected,'board':board,'input_hashes':hashes}

def handeye_from_poses(gimbal,board_camera,holdout):
    if len(gimbal)<12 or len(gimbal)!=len(board_camera):raise ValueError('handeye requires at least 12 matched poses')
    train=[i for i in range(len(gimbal)) if i not in holdout]
    motions=[cv.Rodrigues(gimbal[0][:3,:3].T@gimbal[i][:3,:3])[0].ravel() for i in train[1:]]
    singular=np.linalg.svd(np.array(motions),compute_uv=False)
    if len(singular)<2 or singular[0]<math.radians(15) or singular[1]/singular[0]<.15:
        raise ValueError('handeye motion is degenerate: include nonparallel yaw and pitch rotations')
    R,t=cv.calibrateHandEye([gimbal[i][:3,:3] for i in train],[gimbal[i][:3,3] for i in train],
        [board_camera[i][:3,:3] for i in train],[board_camera[i][:3,3] for i in train],method=cv.CALIB_HAND_EYE_PARK)
    X=transform(R,t);world=[a@X@b for a,b in zip(gimbal,board_camera)];mean=mean_pose([world[i] for i in train])
    residual=[{'translation_m':float(np.linalg.norm(a[:3,3]-mean[:3,3])),
               'rotation_deg':angle(mean[:3,:3].T@a[:3,:3])} for a in world]
    return X,residual,singular.tolist()

def calibrate_handeye(folder,board,intrinsics,fixed_center=False):
    if intrinsics.get('schema')!='sp_intrinsics_v1' or intrinsics['status']!='PASS':raise ValueError('validated intrinsics required')
    manifest,size,obj,views,rejected,hashes=dataset(folder,board)
    if list(size)!=intrinsics['image_size']:raise ValueError('handeye images do not match calibrated resolution')
    k=np.array(intrinsics['config']['camera_matrix']).reshape(3,3);d=np.array(intrinsics['config']['distort_coeffs'])
    S=rotation(manifest['R_gimbal2imubody']);gimbal=[];board_camera=[];pnp=[]
    for sample,points in views:
        R=S.T@quaternion(sample['quaternion_wxyz'])@S
        if 'gimbal_translation_m' in sample:t=sample['gimbal_translation_m']
        elif fixed_center:t=[0,0,0]
        else:raise ValueError('gimbal translation is missing: provide measured poses or explicitly declare --fixed-center')
        gimbal.append(transform(R,t));pose,error=solve_pose(obj,points,k,d);board_camera.append(pose);pnp.append(error)
    hold=[i for i in range(len(views)) if i%5==0];X,residual,singular=handeye_from_poses(gimbal,board_camera,hold)
    trans=float(np.percentile([r['translation_m'] for r in residual],95));rot=float(np.percentile([r['rotation_deg'] for r in residual],95))
    passed=trans<=.01 and rot<=1.0 and max(pnp)<=1.0 and np.linalg.norm(X[:3,3])<1.0
    return {'schema':'sp_handeye_v1','kind':manifest['kind'],'status':'PASS' if passed else 'FAIL',
        'config':{'R_gimbal2imubody':S.flatten().tolist(),'R_camera2gimbal':X[:3,:3].flatten().tolist(),'t_camera2gimbal':X[:3,3].tolist()},
        'translation_model':'fixed_center' if fixed_center else 'measured','view_count':len(views),'holdout_indices':hold,
        'board_world_residuals':residual,'translation_p95_m':trans,'rotation_p95_deg':rot,'pnp_rms_px':pnp,
        'motion_singular_values_rad':singular,'board':board,'intrinsics_sha256':hashlib.sha256(json.dumps(intrinsics,sort_keys=True).encode()).hexdigest(),
        'rejected':rejected,'input_hashes':hashes}
