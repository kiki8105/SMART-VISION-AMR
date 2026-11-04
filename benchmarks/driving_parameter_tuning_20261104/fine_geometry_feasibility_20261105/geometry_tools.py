"""Offline original-square geometry; no ROS imports or processes."""
from pathlib import Path
import math
import numpy as np
from PIL import Image
import yaml


def load_saved_map(path):
    path=Path(path); document=yaml.safe_load(path.read_text())
    if document.get('mode','trinary')!='trinary' or document['origin'][2]!=0:
        raise ValueError('This recorded prototype supports only the saved unrotated trinary map')
    image=Path(document['image'])
    if not image.is_absolute(): image=path.parent/image
    with Image.open(image) as value:
        pixels=np.asarray(value.convert('RGB'),dtype=float).mean(axis=2)
    probability=(pixels if document.get('negate',0) else 255-pixels)/255
    grid=np.full(probability.shape,-1,dtype=np.int16)
    grid[probability<document.get('free_thresh',.25)]=0
    grid[probability>document.get('occupied_thresh',.65)]=100
    grid=np.flipud(grid).copy()
    return {'width':grid.shape[1],'height':grid.shape[0], 'resolution':float(document['resolution']),
            'origin':dict(zip(('x','y','yaw'),document['origin']))},grid


def polyline_gap(points,lower,upper,radius,world_bounds):
    """Exact segment-to-full-rectangle distance, including original map edge."""
    p=np.asarray(points,dtype=float)
    if len(p)<2:raise ValueError('Need a polyline, including continuous connectors')
    x0,y0,x1,y1=world_bounds
    boundary=float(np.minimum.reduce([p[:,0]-x0,p[:,1]-y0,x1-p[:,0],y1-p[:,1]]).min())
    # Rectangles outside this bounded region cannot violate the requested body
    # gap; retain a larger .50m region so the reported minimum is exact here.
    keep=np.all(upper>=p.min(axis=0)-.5,axis=1)&np.all(lower<=p.max(axis=0)+.5,axis=1)
    lower,upper=lower[keep],upper[keep]
    if not len(lower):return boundary-radius
    first=p[:-1,None,:];delta=(p[1:]-p[:-1])[:,None,:]
    enter=np.zeros((len(p)-1,len(lower)));leave=np.ones_like(enter);possible=np.ones_like(enter,dtype=bool)
    for axis in (0,1):
        moving=np.abs(delta[...,axis])>1e-14
        a=np.divide(lower[None,:,axis]-first[...,axis],delta[...,axis],out=np.full_like(enter,-np.inf),where=moving)
        b=np.divide(upper[None,:,axis]-first[...,axis],delta[...,axis],out=np.full_like(enter,np.inf),where=moving)
        enter=np.maximum(enter,np.minimum(a,b));leave=np.minimum(leave,np.maximum(a,b))
        possible&=moving|((lower[None,:,axis]<=first[...,axis])&(first[...,axis]<=upper[None,:,axis]))
    if np.any(possible&(enter<=leave)):return -radius
    minimum=boundary
    for endpoint in (first,first+delta):
        separation=np.maximum(np.maximum(lower[None,:,:]-endpoint,endpoint-upper[None,:,:]),0.)
        minimum=min(minimum,float(np.hypot(separation[...,0],separation[...,1]).min()))
    squared=np.square(delta).sum(axis=2)
    for ix,iy in ((0,0),(0,1),(1,0),(1,1)):
        corner=np.column_stack(((upper if ix else lower)[:,0],(upper if iy else lower)[:,1]))
        along=np.divide(((corner[None,:,:]-first)*delta).sum(axis=2),squared,
                        out=np.zeros_like(enter),where=squared>0)
        projected=first+np.clip(along,0.,1.)[...,None]*delta-corner[None,:,:]
        minimum=min(minimum,float(np.hypot(projected[...,0],projected[...,1]).min()))
    return minimum-radius


def path_progress(points,position):
    best=(math.inf,0.);traveled=0.
    for a,b in zip(points,points[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
        t=max(0.,min(1.,((position[0]-a[0])*dx+(position[1]-a[1])*dy)/(length*length))) if length else 0.
        distance=math.hypot(position[0]-a[0]-t*dx,position[1]-a[1]-t*dy)
        if distance<best[0]:best=(distance,traveled+t*length)
        traveled+=length
    return best[1]
