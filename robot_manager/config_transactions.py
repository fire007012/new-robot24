"""Management-v1 additive, immutable profile bundles with atomic activation.

Only confirmed management policy fields are editable; no ROS/device NVM writes.
"""
import copy
import hashlib
import json
import os
from pathlib import Path
import threading
import time
import uuid

METHODS = ['config.schema','config.status','config.source','config.candidate',
           'config.preview','config.apply','config.rollback','config.operation','config.audit']


def encode(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False).encode()


def atomic(path, value):
    tmp=path.with_suffix('.tmp')
    with tmp.open('wb') as stream:
        stream.write(encode(value))
        stream.flush();os.fsync(stream.fileno())
    tmp.replace(path)
    fd=os.open(path.parent,os.O_DIRECTORY)
    try:os.fsync(fd)
    finally:os.close(fd)


class ConfigTransactions:
    def __init__(self, manager, fault):
        self.m,self.Fault=manager,fault
        self.root=manager.state_dir/'configuration'
        self.root.mkdir(mode=0o700,exist_ok=True)
        self.bundles=self.root/'bundles';self.bundles.mkdir(exist_ok=True)
        self.pointer=self.root/'active.json'
        self.journal=self.root/'journal.json'
        self.busy=False
        self.worker=None
        self.hook=lambda phase:None  # Deterministic no-device fault injection seam.
        self.data=json.loads(self.journal.read_text()) if self.journal.exists() else dict(candidates={},operations={},requests={},audit=[])
        if not self.pointer.exists():
            profiles={}
            for path in sorted(manager.root.glob('*.json')):
                # Initial import is a trust boundary; never follow a profile
                # symlink outside the deployed configuration root.
                resolved = path.resolve()
                if path.is_symlink() or resolved.parent != manager.root or not resolved.is_file():
                    raise self.Fault('CONFIG', 'profile must be a regular file inside config_root')
                try:
                    value = json.loads(resolved.read_text())
                except (OSError, ValueError, TypeError):
                    raise self.Fault('CONFIG', 'invalid deployed profile JSON/schema')
                if not isinstance(value, dict) or value.get('profile_id') != path.stem:
                    raise self.Fault('CONFIG', 'profile_id does not match deployed filename')
                profiles[path.stem] = value
            if not profiles:
                raise self.Fault('CONFIG', 'config_root contains no JSON profiles')
            revision=self.bundle(profiles, rewrite=False)
            atomic(self.pointer,dict(revision=revision))
        for op in self.data['operations'].values():
            if op['state'] in ('accepted','running'):
                op.update(state='result_unknown',error='MANAGER_RESTART',pending_restart=True)
        self.save()

    def save(self):atomic(self.journal,self.data)

    def current(self):return json.loads(self.pointer.read_text())['revision']

    def summary(self, revision=None, profiles=None):
        revision = revision or self.current()
        profiles = self._read(revision) if profiles is None else profiles
        return dict(revision=revision, profile_count=len(profiles),
                    profile_ids=sorted(profiles))

    def profiles_root(self):return self.bundles/self.current()/'profiles'

    def _read(self, revision):
        # All revision strings are looked up against an immutable server-owned manifest.
        if not isinstance(revision,str) or len(revision)!=64 or any(c not in '0123456789abcdef' for c in revision):
            raise self.Fault('REVISION','invalid configuration revision')
        path=self.bundles/revision/'manifest.json'
        if not path.is_file() or path.is_symlink() or path.resolve().parent != (self.bundles/revision).resolve():
            raise self.Fault('REVISION','unknown configuration revision')
        try:
            manifest=json.loads(path.read_text())
        except (OSError, ValueError, TypeError):
            raise self.Fault('CONFIG_DIGEST','invalid immutable bundle manifest')
        if (not isinstance(manifest,dict) or manifest.get('revision') != revision
                or not isinstance(manifest.get('files'),dict)):
            raise self.Fault('CONFIG_DIGEST','invalid immutable bundle manifest')
        profiles={}
        for name,digest in manifest['files'].items():
            if (not isinstance(name,str) or not name.endswith('.json')
                    or '/' in name or '\\' in name or name in ('.json','..json')):
                raise self.Fault('CONFIG_DIGEST','invalid immutable bundle path')
            p=self.bundles/revision/'profiles'/name
            if p.is_symlink() or p.resolve().parent != (self.bundles/revision/'profiles').resolve():
                raise self.Fault('CONFIG_DIGEST','immutable bundle path escapes profile directory')
            try:
                raw=p.read_bytes()
            except OSError:
                raise self.Fault('CONFIG_DIGEST','immutable bundle file missing')
            if not isinstance(digest,str) or hashlib.sha256(raw).hexdigest()!=digest:
                raise self.Fault('CONFIG_DIGEST','immutable bundle digest mismatch')
            try:
                profile=json.loads(raw)
            except (ValueError, TypeError):
                raise self.Fault('CONFIG_DIGEST','immutable bundle JSON invalid')
            if not isinstance(profile,dict) or profile.get('profile_id') != name[:-5]:
                raise self.Fault('CONFIG_DIGEST','profile_id does not match immutable bundle path')
            profiles[name[:-5]]=profile
        return profiles

    def bundle(self, profiles, rewrite=True):
        semantic=copy.deepcopy(profiles)
        for p in semantic.values():p.pop('config_revision',None)
        revision=hashlib.sha256(encode(semantic)).hexdigest()
        final=self.bundles/revision
        if final.exists():
            self._read(revision)
            return revision
        stage=self.bundles/('stage-'+uuid.uuid4().hex)
        stage.mkdir();(stage/'profiles').mkdir()
        files={}
        for name,profile in sorted(profiles.items()):
            import re
            if not re.fullmatch(r'[a-zA-Z0-9_-]{1,64}',name):raise self.Fault('CONFIG','invalid profile name')
            value=copy.deepcopy(profile)
            if rewrite:value['config_revision']=revision
            target=stage/'profiles'/(name+'.json')
            atomic(target,value)
            files[target.name]=hashlib.sha256(target.read_bytes()).hexdigest()
            self.hook('generated_file')
        atomic(stage/'manifest.json',dict(revision=revision,files=files))
        stage.rename(final)
        fd=os.open(self.bundles,os.O_DIRECTORY)
        try:os.fsync(fd)
        finally:os.close(fd)
        self._read(revision)
        return revision

    def stopped(self):
        if (self.m.active() or self.m.data['software_state']!='stopped' or
                self.m.data['resources'] or self.m.recovery):
            raise self.Fault('MAINTENANCE_REQUIRED','stop managed stack and reconcile first')

    def _status(self):
        active=self.current()
        running=self.m.data.get('profile')
        observed=self.m.data.get('health',{}).get('observed_at')
        healthy=(self.m.data['software_state'] in ('ready','degraded') and observed is not None and
                 self.m.last_health_monotonic is not None and time.monotonic()-self.m.last_health_monotonic<5)
        current_profiles=self._read(active)
        runtime_revision=(active if running and healthy and running==current_profiles.get(running.get('profile_id')) else None)
        candidates = [dict(candidate_revision=revision,
                           diff_count=len(candidate.get('diff', [])),
                           affected_groups=list(candidate.get('affected_groups', [])),
                           pending_restart=bool(candidate.get('pending_restart', True)))
                      for revision, candidate in self.data['candidates'].items()]
        return dict(source_revision=active, source_summary=self.summary(active, current_profiles),
            candidate_revisions=list(self.data['candidates']), candidate_summaries=candidates,
            effective_management_revision=active,actual_runtime_revision=runtime_revision,
            runtime_observed_at=observed if healthy else None,pending_restart=not bool(runtime_revision==active),
            applying=self.busy,scope='manager_profiles',device_nvm_written=False)

    def dispatch(self, method, rid, p):
        with self.m.lock:
            if method=='config.schema':
                return dict(schema_version=1,capability='config_transactions_v1',scope='manager_profiles',
                    fields={
                        'startup_timeout':dict(type='number',unit='s',minimum=1,maximum=180,editable=True,
                            maintenance=True,stop_groups=['all'],restart=True,source='Manager.work monotonic startup health budget'),
                        'video_required':dict(type='boolean',unit=None,editable=True,depends_on='video == true when value == true',
                            maintenance=True,stop_groups=['all'],restart=True,source='Backend.required / optional video health policy')},
                    readonly_fields=['mode','devices','baseline','video_config','build_version','profile_id','config_revision'],
                    max_patches=64,max_request_bytes=49152,max_page_size=50,device_nvm=False)
            if method=='config.status':return self._status()
            if method=='config.source':
                revision=p.get('revision',self.current())
                profiles=self._read(revision)
                name=p.get('profile_id')
                if name not in profiles:raise self.Fault('PROFILE','profile_id required')
                result=dict(revision=revision,summary=self.summary(revision, profiles),profile=profiles[name])
                if len(encode(result))>49152:raise self.Fault('RESPONSE_TOO_LARGE','profile too large; offline maintenance required')
                return result
            if method=='config.operation':
                oid=p.get('operation_id')
                if not isinstance(oid,str) or oid not in self.data['operations']:raise self.Fault('NOT_FOUND','unknown config operation')
                return copy.deepcopy(self.data['operations'][oid])
            if method=='config.audit':
                cursor,limit=p.get('cursor',0),p.get('limit',50)
                if type(cursor)is not int or cursor<0 or type(limit)is not int or not 1<=limit<=50:raise self.Fault('BAD_REQUEST','invalid cursor/limit')
                if cursor>len(self.data['audit']):raise self.Fault('CURSOR','cursor beyond end')
                items=self.data['audit'][cursor:cursor+limit]
                return dict(items=items,next_cursor=cursor+len(items))
            if method=='config.preview':
                candidate=self.data['candidates'].get(p.get('candidate_revision'))
                if not candidate:raise self.Fault('REVISION','unknown candidate')
                return copy.deepcopy(candidate)
            if method not in ('config.candidate','config.apply','config.rollback'):
                raise self.Fault('BAD_REQUEST','unknown config method')
            try:
                fingerprint=hashlib.sha256(encode([method,p])).hexdigest()
            except (TypeError, ValueError, OverflowError):
                raise self.Fault('BAD_REQUEST','params must be finite JSON values')
            previous=self.data['requests'].get(rid)
            if previous:
                if previous['fingerprint']!=fingerprint:raise self.Fault('REQUEST_CONFLICT','request_id content mismatch')
                if 'operation_id' in previous:return copy.deepcopy(self.data['operations'][previous['operation_id']])
                return copy.deepcopy(previous['result'])
            if self.busy:raise self.Fault('BUSY','configuration transaction running',True)
            if len(self.data['requests'])>=512:raise self.Fault('CONFIG_CACHE_FULL','archive configuration state offline before new transactions')
            if p.get('base_revision')!=self.current():raise self.Fault('REVISION_CONFLICT','source revision changed')
            if method=='config.candidate':
                if len(encode(p))>49152:raise self.Fault('REQUEST_TOO_LARGE','candidate exceeds budget; split into separate candidates')
                patches=p.get('patches')
                if not isinstance(patches,list) or not 1<=len(patches)<=64:raise self.Fault('BAD_REQUEST','1..64 patches required')
                profiles=self._read(self.current())
                diff=[];seen=set()
                for patch in patches:
                    if not isinstance(patch,dict) or set(patch)!={'profile_id','field','value'}:raise self.Fault('BAD_REQUEST','invalid patch fields')
                    name,field,value=patch['profile_id'],patch['field'],patch['value']
                    if not isinstance(name,str) or name not in profiles or field not in ('startup_timeout','video_required'):
                        raise self.Fault('CONFIG_FIELD','unknown or readonly field')
                    if (name,field) in seen:raise self.Fault('CONFIG_FIELD','duplicate patch')
                    seen.add((name,field))
                    profile=profiles[name]
                    if field=='startup_timeout' and (type(value) not in (int,float) or not 1<=value<=180):raise self.Fault('CONFIG_VALIDATION','startup_timeout outside 1..180s')
                    if field=='video_required' and (type(value)is not bool or value and not profile.get('video')):raise self.Fault('CONFIG_VALIDATION','video_required depends on video')
                    old=profile.get(field,90 if field=='startup_timeout' else False)
                    profile[field]=value
                    if field=='video_required':
                        profile['required_modules']=['core','bridge']+(['video'] if value else [])
                        profile['optional_modules']=['video'] if profile.get('video') and not value else []
                    diff.append(dict(profile_id=name,field=field,before=old,after=value))
                for profile in profiles.values():self.m.backend.validate(profile)
                revision=self.bundle(profiles)
                candidate=dict(candidate_revision=revision,base_revision=self.current(),diff=diff,
                    summary=self.summary(revision, profiles), affected_groups=['all'],
                    maintenance_required=True,pending_restart=True,device_nvm_written=False)
                self.data['candidates'][revision]=candidate
                self.data['requests'][rid]=dict(fingerprint=fingerprint,result=candidate)
                self.data['audit'].append(dict(event='candidate_saved',revision=revision,request_id=rid))
                self.save()
                return copy.deepcopy(candidate)
            self.stopped()
            target=p.get('candidate_revision') if method=='config.apply' else p.get('target_revision')
            profiles=self._read(target)
            if method=='config.apply':
                candidate=self.data['candidates'].get(target)
                if not candidate or candidate['base_revision']!=self.current():raise self.Fault('REVISION_CONFLICT','candidate base changed')
            oid=str(uuid.uuid4())
            op=dict(operation_id=oid,method=method,state='accepted',phase='validate',base_revision=self.current(),
                    target_revision=target,summary=self.summary(target, profiles), affected_groups=['all'],
                    maintenance_required=True,error=None,pending_restart=True,verification=None,
                    created_at=time.time(),updated_at=time.time())
            self.data['operations'][oid]=op
            self.data['requests'][rid]=dict(fingerprint=fingerprint,operation_id=oid)
            self.busy=True
            self.save()
            self.worker=threading.Thread(target=self.work,args=(oid,profiles),daemon=True)
            self.worker.start()
            return copy.deepcopy(op)

    def work(self, oid, profiles):
        try:
            with self.m.lock:
                op=self.data['operations'][oid]
                op.update(state='running',phase='validate',updated_at=time.time());self.save()
                self.stopped()
            # Only server-owned bundle data. Never apply paths supplied by a client.
            for profile in profiles.values():self.m.backend.validate(profile)
            # Independently verify no unmanaged runtime/device ownership before activation.
            # Backend preflight includes existing runtime detection; no enable/resume calls.
            check=self.m.backend.preflight(next(iter(profiles.values())))
            if not check['passed']:raise self.Fault('CONFIG_PREFLIGHT','deployment/ownership preflight failed')
            self.hook('before_switch')
            with self.m.lock:
                self.stopped()
                if self.current()!=op['base_revision']:raise self.Fault('REVISION_CONFLICT','source changed before activation')
                op['phase']='activate';op['updated_at']=time.time();self.save()
                atomic(self.pointer,dict(revision=op['target_revision']))
                self.hook('after_switch')
                # Actual consumer reads immutable active source; compare every profile.
                expected=self._read(op['target_revision'])
                actual={path.stem:json.loads(path.read_text()) for path in self.profiles_root().glob('*.json')}
                if actual!=expected:raise self.Fault('CONFIG_EFFECTIVE_MISMATCH','active management source differs')
                self.hook('verify')
                op.update(state='succeeded',phase='complete',verification='manager_source_readback',
                    updated_at=time.time(),
                    effective_management_revision=self.current(),actual_runtime_revision=None)
                self.data['audit'].append(dict(event='applied',operation_id=oid,revision=self.current(),method=op['method']))
                self.save()
        except Exception as exc:
            with self.m.lock:
                op=self.data['operations'][oid]
                try:
                    # Last usable bundle is retained; only one atomic pointer is rolled back.
                    self._read(op['base_revision'])
                    atomic(self.pointer,dict(revision=op['base_revision']))
                    if self.current()!=op['base_revision']:raise RuntimeError('rollback readback')
                    op.update(state='failed',phase='rolled_back',updated_at=time.time(),error=getattr(exc,'data',dict(code=type(exc).__name__,message='configuration apply failed')),
                        rollback_revision=op['base_revision'],verification='manager_source_readback',actual_runtime_revision=None)
                except Exception:
                    op.update(state='result_unknown',phase='reconcile',updated_at=time.time(),error=dict(code='ROLLBACK_UNKNOWN'))
                self.data['audit'].append(dict(event=op['phase'],operation_id=oid,revision=self.current()))
                self.save()
        finally:
            with self.m.lock:self.busy=False
