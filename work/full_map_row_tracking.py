"""Explicit partial-bank fusion using a recent MEASURED gap and estimated pose."""
from collections import deque
import math,numpy as np

def wrap(x):return (x+math.pi)%(2*math.pi)-math.pi

class RowObservationTracker:
 def __init__(self):self.phase=None;self.widths=deque(maxlen=15);self.stamp=None;self.last_pair=None
 def resolve(self,raw,combined,phase,stamp,pose,row_y,heading):
  if phase!=self.phase:self.phase=phase;self.widths.clear();self.last_pair=None;self.stamp=None
  if raw.get('valid'):
   if stamp!=self.stamp:
    self.widths.append(raw['observed_bank_gap_m']);self.last_pair=(stamp,np.array(pose[:2],float));self.stamp=stamp
   return dict(raw,sensing_mode='CURRENT_TWO_BANKS')
  if not self.widths or not self.last_pair or not 0<=stamp-self.last_pair[0]<=10. or np.linalg.norm(np.array(pose[:2])-self.last_pair[1])>.5:return raw
  width=float(np.median(self.widths));gap_sigma=max(.001,float(1.4826*np.median(abs(np.array(self.widths)-width))))
  if gap_sigma>.004 or max(self.widths)-min(self.widths)>.015:return raw
  expected_heading=wrap(heading-pose[2]);c=math.cos(pose[2])
  if abs(c)<.7:return raw
  expected_center=(row_y-pose[1])/c;candidates=[]
  for side,sign in [('left',1),('right',-1)]:
   # The current scan must still support the side used for correction.
   current=raw.get(side)
   if not current:continue
   for source,obs in [('CURRENT',raw),('RECENT_RAW',combined)]:
    bank=obs.get(side)
    if not bank or bank['rms_m']>.003 or bank['span_m']<.8 or bank['center_sigma_m']>.003 or bank['heading_sigma_deg']>.15:continue
    angle=math.atan(bank['slope']);center=bank['intercept']-sign*.5*width*math.sqrt(1+bank['slope']**2)
    if abs(wrap(angle-expected_heading))>.08 or abs(center-expected_center)>.03 or abs(center)>.10:continue
    if abs(wrap(angle-math.atan(current['slope'])))>.02:continue
    sigma=math.hypot(bank['center_sigma_m'],gap_sigma/2.)
    score=sigma+.1*abs(wrap(angle-expected_heading))
    candidates.append((score,side,bank,center,angle,sigma,source))
  if not candidates:return raw
  _,side,bank,center,angle,sigma,source=min(candidates,key=lambda q:q[0])
  return dict(raw,valid=True,reason='VISIBLE_BANK_RECENT_MEASURED_GAP',sensing_mode='SINGLE_BANK_WITH_RECENT_MEASURED_GAP',visible_bank=side,bank_source=source,center_error_body_y_m=center,heading_error_rad=angle,observed_bank_gap_m=None,remembered_bank_gap_m=width,gap_sigma_m=gap_sigma,gap_age_s=stamp-self.last_pair[0],partial_center_sigma_m=sigma,partial_heading_sigma_deg=bank['heading_sigma_deg'])
