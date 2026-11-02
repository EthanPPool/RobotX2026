"""Observed-map mission execution with active viewpoints and local replanning.

Inputs are measured Marker records only. No scene buoy count, color oracle,
unseen endpoints, or complete-survey predicate is used by this mission engine.
Static beacon assignments are retained for an attempt; conflicts invalidate them.
"""
from dataclasses import replace
import math

from robotx_coordination.geometry import segment_distance
from .models import Marker,angle_difference
from .routing import Course,Orbit
from .trajectory import Primitive,Sample,Trajectory,duration,polyline,orbit

INCREMENTAL_PLANNING = True


class LocalCourse(Course):
    def __init__(self,markers,boundary,config):
        super().__init__(markers,boundary,config,allow_partial=True)

    def passages(self,a,b,heading=None,local_only=False):
        legal,bits=super().passages(a,b,heading,local_only)
        if not legal:return False,bits
        length=math.dist(a,b)
        if length<1e-9:return True,bits
        dx,dy=((b[0]-a[0])/length,(b[1]-a[1])/length) if heading is None else (math.cos(heading),math.sin(heading))
        for m in self.markers:
            if m.color!='unknown':continue
            aa=(m.x-a[0])*dx+(m.y-a[1])*dy
            bb=(m.x-b[0])*dx+(m.y-b[1])*dy
            if not (aa>=0>=bb or heading is not None and aa<=0<=bb):continue
            t=aa/(aa-bb) if aa!=bb else 0.
            p=(a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]))
            side=dx*(m.y-p[1])-dy*(m.x-p[0])
            if local_only and abs(side)>self.config.passage_range:continue
            # Passing a closest unknown could violate either color assignment.
            shielded=any(o.track_id!=m.track_id and
                (dx*(o.y-p[1])-dy*(o.x-p[0]))*side>0 and
                math.dist(o.point,p)<abs(side)-1e-8 for o in self.markers)
            if not shielded:return False,bits
        return True,bits

    def orbit_sides_clear(self,arc):
        if not super().orbit_sides_clear(arc):return False
        for m in self.markers:
            if m.color!='unknown':continue
            bearing=math.atan2(m.y-arc.center[1],m.x-arc.center[0])
            for angle in (bearing,bearing+math.pi):
                points=[(arc.center[0]+arc.radius*math.cos(angle+arc.direction*d),
                         arc.center[1]+arc.radius*math.sin(angle+arc.direction*d)) for d in (-1e-5,1e-5)]
                if not self.passages(*points,heading=angle+arc.direction*math.pi/2,local_only=True)[0]:return False
        return True


class IncrementalPassage:
    PHASES=('SEARCH_ENTRY','OBSERVE','APPROACH_ENTRY','CIRCLE_ENTRY','TRANSIT','CIRCLE_EXIT','HOLD')

    def __init__(self,config,boundary,tier='core',allow_scans=True,observation_distance=8.,simulate_reports=False,physical_clearance=None):
        config.validate()
        if tier not in ('core','advanced'):raise ValueError('Unsupported incremental tier')
        self.config,self.boundary,self.tier=config,tuple(map(tuple,boundary)),tier
        self.allow_scans=allow_scans
        self.observation_distance=observation_distance
        self.simulate_reports=simulate_reports
        self.physical_clearance=config.clearance if physical_clearance is None else physical_clearance
        self.state,self.reason='IDLE','NOT_STARTED'
        self.belief={};self.next_id=1;self.revision=0
        self.course=LocalCourse((),self.boundary,config)
        self.plan=self.trajectory=None;self.elapsed=0.
        self.run_id='';self.last_pose=self.last_now=None
        self.entry_done=self.exit_done=False
        self.passed_ids=set();self.circle_progress=0.;self.last_angle=None
        self.arc=None;self.task=None;self.route_points=()
        self.visited=set();self.view_visits=set();self.scan_count=0
        self.last_plan=-math.inf;self.planned_revision=-1;self.replans=0
        self.history=[];self.entry_reported=False;self.event_seq=0;self.map_error=''

    @property
    def passed_mask(self):
        return sum(bit for ident,bit in self.course.bits.items() if ident in self.passed_ids)

    @property
    def phase_index(self):return 3 if self.state=='CIRCLE_EXIT' else 2 if self.entry_done else 1 if self.state=='CIRCLE_ENTRY' else 0

    @property
    def local_sides(self):return not self.entry_done or self.state.startswith('CIRCLE')

    def endpoint(self,role):
        found=[m for m in self.belief.values() if m.role==role and m.confidence>=self.config.min_confidence]
        return found[0] if len(found)==1 else None

    def observe(self,markers,conflicts=()):
        before=self.fingerprint()
        self.map_error=''
        for observed in markers:
            try:observed.validate()
            except ValueError:
                self.map_error='MAP_GEOMETRY_REJECTED';continue
            nearby=[m for m in self.belief.values() if math.dist(m.point,observed.point)<=.7]
            if len(nearby)>1:continue  # Never guess identities in an ambiguous map.
            old=nearby[0] if nearby else None
            if old is None and len(self.belief)>=self.config.max_markers:
                self.map_error='MAP_CAPACITY_LIMIT';continue
            key=old.track_id if old else self.next_id
            if old is None:self.next_id+=1
            good=(observed.confidence>=self.config.min_confidence and
                (observed.role!='obstacle' or observed.color=='black' and observed.light_state=='inactive') and
                (self.tier=='core' or observed.color_source=='uav'))
            m=replace(observed,track_id=key)
            if observed.track_id in conflicts:
                m=replace(m,color='unknown',light_state='unknown',confidence=0.,color_source='')
                self.passed_ids.discard(key)
            elif not good:
                if old is not None and old.confidence>=self.config.min_confidence:
                    m=replace(m,color=old.color,light_state=old.light_state,confidence=old.confidence,
                        semantic_stamp=old.semantic_stamp,color_source=old.color_source)
                else:m=replace(m,color='unknown',light_state='unknown',confidence=0.,color_source='')
            elif old is not None and old.color!='unknown' and m.role!=old.role:
                self.passed_ids.discard(key)
            self.belief[key]=m
        if self.fingerprint()!=before:
            self.revision+=1
            self.course=LocalCourse(tuple(self.belief.values()),self.boundary,self.config)

    def fingerprint(self):
        return tuple((k,round(m.x,2),round(m.y,2),round(m.radius+m.uncertainty,2),m.color,m.light_state)
                     for k,m in sorted(self.belief.items()))

    def observe_aerial(self,markers):
        """Preserve explicit aerial provenance when same-time fusion ties favor USV.

        Enrich semantics on the spatially matched object, retaining its precise
        local geometry. Inputs must be actual projected aerial observations.
        """
        before=self.fingerprint();previous_error=self.map_error
        for m in markers:
            m.validate()
            if m.color_source!='uav' or m.confidence<self.config.min_confidence:continue
            nearby=[old for old in self.belief.values() if math.dist(m.point,old.point)<=.7]
            if len(nearby)==1:
                old=nearby[0]
                self.belief[old.track_id]=replace(old,color=m.color,light_state=m.light_state,
                    confidence=m.confidence,semantic_stamp=m.semantic_stamp,color_source='uav')
            elif not nearby:self.observe((m,))
        if self.fingerprint()!=before:
            self.revision+=1;self.course=LocalCourse(tuple(self.belief.values()),self.boundary,self.config)
        self.map_error=previous_error or self.map_error

    def start(self,pose,now,run_id='incremental'):
        pose.validate()
        if self.state!='IDLE':raise ValueError('Reset before another attempt')
        self.run_id,self.last_pose,self.last_now=run_id,pose,now
        self.state='SEARCH_ENTRY';self.reason='Learning the course from current observations'
        self._record(now,'START',known=len(self.belief))
        self.prepare(pose,now)

    def _record(self,now,event,**extra):
        self.event_seq+=1
        self.history.append(dict(seq=self.event_seq,t=now,event=event,revision=self.revision,**extra))
        self.history=self.history[-200:]

    def stop(self,reason='OPERATOR_STOP'):
        self.state,self.reason='STOPPED',reason
        self.trajectory=None

    def acknowledge_report(self):
        """Called by real reporting integration only after its successful report."""
        self.entry_reported=True

    def preview(self):
        return self.trajectory.preview(self.elapsed,self.config.preview_dt) if self.trajectory else ()

    def motion_legal(self,a,b):
        if math.dist(a.point,b.point)<1e-9:return True  # Same circumscribed hull footprint during a scan.
        clear=self.course.segment_clear(a.point,b.point)
        if not clear:
            physical=LocalCourse(self.course.markers,self.boundary,
                replace(self.config,clearance=self.physical_clearance))
            # Starting inside an extra planning margin permits retreat only.
            clear=physical.segment_clear(a.point,b.point) and all(
                math.dist(a.point,m.point)>=self.course.inflated(m) or (
                    math.dist(b.point,m.point)>=math.dist(a.point,m.point)-1e-9 and
                    segment_distance(m.point,a.point,b.point)>=math.dist(a.point,m.point)-1e-9)
                for m in self.course.markers)
        return clear and self.course.passages(a.point,b.point,b.yaw,local_only=self.local_sides)[0]

    def _route(self,pose,goal,required=0):
        return self.course.route(pose.point,goal,True,required_mask=required,local_sides=not self.entry_done)

    def _navigate(self,pose,now,path,state,task):
        # Keep the reference horizon local. Revisit the goal from the new pose.
        points=[path[0]];remaining=8.
        for a,b in zip(path,path[1:]):
            distance=math.dist(a,b)
            if distance>remaining:
                t=remaining/distance;points.append((a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])));break
            points.append(b);remaining-=distance
        self.state,self.task=state,task
        self.route_points=tuple(points)
        self.trajectory=polyline(points,pose.yaw,self.config);self.elapsed=0.
        self.arc=None;self.planned_revision=self.revision;self.replans+=1
        self.reason='Following local route; map remains open to new observations'
        self._record(now,'LOCAL_ROUTE',state=state,known=len(self.belief),goal=list(points[-1]))

    def _scan(self,pose,now,target=None):
        amount=2*math.pi if self.allow_scans else 0.
        seconds=duration(amount,self.config.yaw_rate,self.config.yaw_acceleration)
        primitive=Primitive('turn',pose.point,pose.point,pose.yaw,amount,seconds)
        self.trajectory=Trajectory((primitive,),Sample(*pose.point,pose.yaw))
        self.elapsed=0.;self.arc=None;self.route_points=()
        self.state='OBSERVE' if target is not None else 'SEARCH_ENTRY' if not self.entry_done else 'TRANSIT'
        self.task=('scan',target,self.endpoint('entry') is None and not self.entry_done)
        self.scan_count+=1;self.planned_revision=self.revision
        self.reason='Scanning for a clearer beacon observation'
        self._record(now,'SCAN',target=target)

    def _orbit(self,pose,now,marker,which):
        direction=-1 if which=='entry' else 1
        arc=self.course.orbit(marker,pose.point,direction)
        if math.dist(pose.point,arc.start)>self.config.arrival_tolerance:
            path=self._route(pose,arc.start)
            self._navigate(pose,now,path,'APPROACH_ENTRY' if which=='entry' else 'TRANSIT',('approach',which,marker.track_id))
            return
        self.arc=arc;self.trajectory=orbit(arc,pose.yaw,self.config);self.elapsed=0.
        self.state='CIRCLE_ENTRY' if which=='entry' else 'CIRCLE_EXIT'
        self.task=('orbit',which,marker.track_id);self.route_points=()
        self.circle_progress=0.;self.last_angle=None;self.planned_revision=self.revision
        self.reason='Measured '+which+' orbit; continuing to learn surrounding buoys'
        self._record(now,'ORBIT_START',which=which,known=len(self.belief))

    def _remaining_safe(self):
        if self.trajectory is None:return False
        if self.arc:
            owner=self.belief.get(self.task[2])
            return bool(owner and self.course.orbit_clear(owner,self.arc.radius) and self.course.orbit_sides_clear(self.arc))
        remaining=self.elapsed
        for primitive in self.trajectory.primitives:
            if remaining>=primitive.duration:
                remaining-=primitive.duration;continue
            if primitive.kind=='line':
                sample=primitive.sample(remaining)
                a,b=(sample.x,sample.y),primitive.end
                if not self.course.segment_clear(a,b) or not self.course.passages(a,b,
                        local_only=self.local_sides)[0]:return False
            remaining=0.
        return True

    def _viewpoint(self,pose,unknown):
        # Rank viewpoints using only observed geometry, not synthetic visibility.
        choices=[]
        for target in unknown:
            base=math.atan2(pose.y-target.y,pose.x-target.x)
            for da in (0.,math.pi/4,-math.pi/4,math.pi/2,-math.pi/2,math.pi):
                base_radius=max(self.course.inflated(target)+.4,self.observation_distance)
                for radius in (base_radius,base_radius*1.35):
                    a=base+da;point=(target.x+radius*math.cos(a),target.y+radius*math.sin(a))
                    key=(target.track_id,round(point[0]),round(point[1]))
                    if key in self.view_visits or not self.course.point_clear(point):continue
                    if math.dist(pose.point,point)<.6:continue
                    blocked=sum(segment_distance(m.point,point,target.point)<m.radius+m.uncertainty+.1
                        for m in self.belief.values() if m.track_id!=target.track_id)
                    if blocked:continue
                    choices.append((math.dist(pose.point,point),target,point,key))
        for _,target,point,key in sorted(choices,key=lambda v:v[0])[:8]:
            try:return self._route(pose,point),target,key
            except ValueError:continue
        return None

    def prepare(self,pose,now):
        if self.state not in self.PHASES:return
        if self.map_error:
            self.state,self.reason='HOLD',self.map_error
            self.trajectory=None;self.task=None;self.arc=None;self.route_points=();return
        if self.tier=='advanced' and self.entry_done and not self.entry_reported:
            self.state,self.reason='HOLD','WAITING_FOR_UAV_REPORT_ACKNOWLEDGEMENT'
            self.trajectory=None;self.task=None;self.arc=None;self.route_points=();return
        entry=self.endpoint('entry');exit_=self.endpoint('exit')
        if self.state=='CIRCLE_EXIT' and any(m.track_id not in self.passed_ids for m in self.course.semantic):
            self.trajectory=None;self.arc=None;self.task=None;self.state='TRANSIT'
            self._record(now,'REPLAN_NEW_PASSAGE_BEFORE_EXIT')
        if self.trajectory is not None:
            if self.task and self.task[0]=='scan':
                target=self.belief.get(self.task[1]) if self.task[1] is not None else None
                done=(target is not None and target.color!='unknown') or (self.task[2] and entry is not None)
                if done:self.trajectory=None
            if self.trajectory is not None and self.planned_revision!=self.revision:
                if not self._remaining_safe():
                    self._record(now,'REPLAN_MAP_CHANGED');self.trajectory=None
                self.planned_revision=self.revision
            if self.trajectory is not None:return
        if now-self.last_plan<.8:return
        self.last_plan=now
        if not self.course.point_clear(pose.point):
            recovery=[]
            for i in range(24):
                a=2*math.pi*i/24
                point=(pose.x+1.2*math.cos(a),pose.y+1.2*math.sin(a))
                proposed=replace(pose,x=point[0],y=point[1],yaw=a)
                if self.course.point_clear(point) and self.motion_legal(pose,proposed):
                    margin=min((math.dist(point,m.point)-self.course.inflated(m) for m in self.course.markers),default=10.)
                    recovery.append((margin,point))
            if recovery:
                _,point=max(recovery,key=lambda v:v[0])
                self._navigate(pose,now,(pose.point,point),'OBSERVE',('recover_margin',));return
            self.state,self.reason='HOLD','Current pose lacks a safe translation; holding for new observations'
            return
        if not self.entry_done and entry is not None:
            try:
                if self.tier=='advanced' and not self.entry_reported:
                    if exit_ is None:
                        self.state,self.reason='HOLD','Waiting for aerial entry and exit identification; UAV survey continues'
                        return
                    if not self.simulate_reports:
                        self.state,self.reason='HOLD','WAITING_FOR_UAV_REPORT_ACKNOWLEDGEMENT'
                        return
                    self.entry_reported=True;self._record(now,'SIMULATED_UAV_ENTRY_REPORT')
                self._orbit(pose,now,entry,'entry');return
            except ValueError:
                radius=max(self.config.orbit_radius,self.course.inflated(entry)+.2)
                possible=False
                while radius<=self.config.max_orbit_radius+1e-9:
                    possible=possible or self.course.orbit_clear(entry,radius);radius+=.5
                neighbors=[m for m in self.belief.values() if math.dist(m.point,entry.point)<
                    self.config.max_orbit_radius+self.course.inflated(m)]
                if not possible and all(m.uncertainty<=.15 for m in neighbors):
                    self.state,self.reason='HOLD','Observed geometry blocks every configured entry orbit; awaiting map change'
                    self.route_points=();return
        if self.entry_done:
            pending=[m for m in self.course.semantic if m.track_id not in self.passed_ids]
            # Try a path covering the currently known remaining passages to exit.
            if exit_ is not None:
                try:
                    arc=self.course.orbit(exit_,entry.point if entry else pose.point,1)
                    if not pending and math.dist(pose.point,arc.start)<self.config.arrival_tolerance:
                        self._orbit(pose,now,exit_,'exit');return
                    mask=sum(self.course.bits[m.track_id] for m in pending)
                    path=self._route(pose,arc.start,mask)
                    self._navigate(pose,now,path,'TRANSIT',('exit_route',exit_.track_id));return
                except ValueError:pass
            # The exit need not be known to begin legal progress past known buoys.
            if pending:
                anchor=entry.point if entry else pose.point
                center=exit_.point if exit_ else (sum(m.x for m in self.belief.values())/max(1,len(self.belief)),
                    sum(m.y for m in self.belief.values())/max(1,len(self.belief)))
                dx,dy=center[0]-anchor[0],center[1]-anchor[1];norm=max(math.hypot(dx,dy),1e-9);dx/=norm;dy/=norm
                for target in sorted(pending,key=lambda m:math.dist(m.point,pose.point))[:4]:
                    sign=-1 if target.role=='green' else 1
                    goal=(target.x-sign*dy*3+dx*2,target.y+sign*dx*3+dy*2)
                    try:
                        path=self._route(pose,goal,self.course.bits[target.track_id])
                        self._navigate(pose,now,path,'TRANSIT',('pass',target.track_id));return
                    except ValueError:continue
        if self.tier=='advanced':
            self.state,self.reason='HOLD','Awaiting UAV classifications for the next local route; aerial survey continues'
            self.route_points=();return
        # Move to a different viewpoint instead of waiting for a full survey.
        unknown=[m for m in self.belief.values() if m.color=='unknown']
        if self.scan_count==0:
            self._scan(pose,now);return
        view=self._viewpoint(pose,unknown)
        if view:
            path,target,key=view
            self._navigate(pose,now,path,'OBSERVE',('view',target.track_id,key));return
        # If geometry is not yet in range, explore bounded course grid frontiers.
        a,b,d=self.boundary[0],self.boundary[1],self.boundary[-1]
        candidates=[]
        for u in (.2,.4,.6,.8):
            for v in (.2,.4,.6,.8):
                p=(a[0]+u*(b[0]-a[0])+v*(d[0]-a[0]),a[1]+u*(b[1]-a[1])+v*(d[1]-a[1]))
                key=(round(p[0]),round(p[1]))
                if key not in self.visited and self.course.point_clear(p):candidates.append((math.dist(p,pose.point),p,key))
        for _,point,key in sorted(candidates)[:6]:
            try:
                path=self._route(pose,point)
                self._navigate(pose,now,path,'SEARCH_ENTRY' if not self.entry_done else 'TRANSIT',('frontier',key));return
            except ValueError:continue
        self.state,self.reason='HOLD','No currently safe observation route; waiting for new evidence'
        self.route_points=()

    def update(self,pose,now,permitted=True):
        if self.state not in self.PHASES:return False
        previous=self.last_pose;dt=now-self.last_now
        self.last_pose,self.last_now=pose,now
        if not 0<=dt<=1.:self.reason='CLOCK_OR_UPDATE_GAP';return False
        legal,bits=self.course.passages(previous.point,pose.point,pose.yaw,local_only=self.local_sides)
        if not legal:
            self.stop('WRONG_OR_UNCONFIRMED_SIDE_PASSAGE');return False
        if self.entry_done and not self.state.startswith('CIRCLE'):
            self.passed_ids.update(k for k,bit in self.course.bits.items() if bits&bit)
        if not permitted:self.reason='WAITING_FOR_FRESH_LOCAL_CLEARANCE';return False
        if self.trajectory is None:return True
        reference=self.trajectory.sample(self.elapsed)
        error=math.dist(pose.point,(reference.x,reference.y))
        if self.arc:
            angle=math.atan2(pose.y-self.arc.center[1],pose.x-self.arc.center[0])
            if abs(math.dist(pose.point,self.arc.center)-self.arc.radius)<=self.config.tracking_soft_limit:
                if self.last_angle is not None:
                    delta=angle_difference(angle,self.last_angle)
                    if abs(delta)<=self.config.yaw_rate*max(dt,.01)*3+.05:
                        self.circle_progress=max(0.,self.circle_progress+self.arc.direction*delta)
                self.last_angle=angle
        if error<=self.config.tracking_soft_limit and abs(angle_difference(pose.yaw,reference.yaw))<=.6:
            self.elapsed=min(self.trajectory.duration,self.elapsed+dt)
        else:self.reason='REFERENCE_PAUSED_FOR_TRACKING'
        end=self.trajectory.end
        reached=math.dist(pose.point,(end.x,end.y))<=self.config.arrival_tolerance and abs(angle_difference(pose.yaw,end.yaw))<=self.config.heading_tolerance
        if self.elapsed>=self.trajectory.duration and reached:
            if self.arc and self.circle_progress<2*math.pi-.05:return True
            task=self.task
            if task[0]=='orbit':
                if task[1]=='entry':
                    self.entry_done=True;self.state='TRANSIT'
                    self._record(now,'ENTRY_COMPLETE',known=len(self.belief),classified=sum(m.color!='unknown' for m in self.belief.values()))
                else:self.exit_done=True;self.state='COMPLETE';self.reason='Observed course complete';self._record(now,'COMPLETE')
            elif task[0]=='view':
                self.view_visits.add(task[2]);self._scan(pose,now,task[1]);return True
            elif task[0]=='frontier':
                self.visited.add(task[1]);self._scan(pose,now);return True
            self.trajectory=None;self.arc=None;self.task=None
        return self.state!='COMPLETE'

    def route_lines(self):
        if self.arc:
            a=self.arc
            points=[(a.center[0]+a.radius*math.cos(a.start_angle+a.direction*2*math.pi*i/120),
                     a.center[1]+a.radius*math.sin(a.start_angle+a.direction*2*math.pi*i/120)) for i in range(121)]
        else:points=self.route_points
        return [dict(name='Current local plan',points=points,color='#54d6b0')] if points else []
