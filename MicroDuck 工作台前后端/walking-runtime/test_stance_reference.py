import copy
import math
from types import SimpleNamespace
import unittest

from policy_coordinates import HOME, IDS
from stance_reference import StanceReference, PolicyReferenceHardware, ready_for_handoff
from stance_state import StanceState
from teleop_state import TeleopState


class ReferenceTests(unittest.TestCase):
    def setUp(self):
        self.pose=[q+math.radians((i%3)-1) for i,q in enumerate(HOME)]
        self.value={'kind':'operator_standing_reference_v1','profile_sha256':'profile',
                    'contract_sha256':'contract','joint_radians':self.pose}
        self.reference=StanceReference(self.value,'profile','contract')

    def test_saved_stance_is_home_in_policy_coordinates_and_restores_exactly(self):
        for a,b in zip(self.reference.to_policy(self.pose),HOME):self.assertAlmostEqual(a,b)
        for a,b in zip(self.reference.to_physical(HOME),self.pose):self.assertAlmostEqual(a,b)
        arbitrary=[q+.1 for q in self.pose]
        for a,b in zip(self.reference.to_physical(self.reference.to_policy(arbitrary)),arbitrary):
            self.assertAlmostEqual(a,b)

    def test_reference_refuses_drift_nonfinite_large_bias_and_incomplete_vectors(self):
        for changes in ({'profile_sha256':'other'}, {'contract_sha256':'other'},
                        {'joint_radians':list(HOME[:-1])},
                        {'joint_radians':[float('nan')]*15},
                        {'joint_radians':[True]*15},
                        {'joint_radians':[q+.2 for q in HOME]}):
            with self.assertRaises(ValueError):StanceReference({**self.value,**changes},'profile','contract')

    def test_model_home_reference_preserves_calibrated_angles_and_native_goals(self):
        reference=StanceReference({**self.value,'kind':'model_home_reference_v1',
                                   'joint_radians':list(HOME)},'profile','contract')
        measured=[q+math.radians(3) for q in HOME]
        seen=[]
        class Hardware:
            def dispatch(self,request):
                seen.append(request)
                return {'positions':measured[:],'imu':{'gravity':[.1,0.,-.995]}}
        wrapper=PolicyReferenceHardware(Hardware(),reference)
        self.assertEqual(wrapper.dispatch({'op':'read'})['positions'],measured)
        target=list(HOME);target[IDS.index(12)]=math.pi
        wrapper.dispatch({'op':'write','targets':target})
        self.assertEqual(seen[-1]['targets'],target)
        self.assertEqual(reference.bias,[0.]*15)

    def test_model_home_kind_cannot_disguise_a_saved_pose_as_unbiased_model_coordinates(self):
        value={**self.value,'kind':'model_home_reference_v1','joint_radians':list(HOME)}
        value['joint_radians'][IDS.index(13)]+=math.radians(1)
        with self.assertRaisesRegex(ValueError,'cannot contain standing-pose offsets'):
            StanceReference(value,'profile','contract')

    def test_physical_sensor_frame_and_imu_are_never_rewritten(self):
        real={'positions':self.pose[:],'imu':{'gravity':[.1,0.,-.995]},'velocities':[.2]*15}
        original=copy.deepcopy(real)
        class Hardware:
            def dispatch(self,request):return real
        frame=PolicyReferenceHardware(Hardware(),self.reference).dispatch({'op':'read'})
        self.assertEqual(real,original)
        self.assertEqual(frame['imu'],original['imu'])
        self.assertEqual(frame['velocities'],original['velocities'])
        self.assertNotEqual(frame['positions'],real['positions'])

    def test_physical_goals_still_pass_through_the_original_hardware_guards(self):
        seen=[]
        class Hardware:
            def dispatch(self,request):
                seen.append(request)
                if request['op']=='write' and request['targets'][0]>.5:raise RuntimeError('physical limit')
                return {}
        wrapped=PolicyReferenceHardware(Hardware(),self.reference)
        wrapped.dispatch({'op':'write','targets':list(HOME)})
        for a,b in zip(seen[-1]['targets'],self.pose):self.assertAlmostEqual(a,b)
        with self.assertRaisesRegex(RuntimeError,'physical limit'):
            wrapped.dispatch({'op':'write','targets':[1.]*15})

    def test_reference_offset_after_native_saturation_preserves_the_pd_target(self):
        seen=[];events=[]
        class Hardware:
            def dispatch(self,request):seen.append(request);return {}
            def record(self,event,**values):events.append((event,values))
        for sign in (-1,1):
            pose=list(HOME);pose[IDS.index(12)]+=sign*math.radians(2.68027818596)
            ref=StanceReference({**self.value,'joint_radians':pose},'profile','contract')
            wrapped=PolicyReferenceHardware(Hardware(),ref,record_targets_only=True)
            targets=list(HOME);targets[IDS.index(12)]=sign*math.pi
            wrapped.dispatch({'op':'write','targets':targets})
            self.assertAlmostEqual(seen[-1]['targets'][IDS.index(12)],sign*(math.pi+math.radians(2.68027818596)))
            self.assertGreater(abs(events[-1][1]['physical_targets'][IDS.index(12)]),math.pi)
            self.assertEqual(events[-1][1]['physical_actuator_clamped_ids'],[])
            # Corrupt native vectors are still refused before any write.
            before=len(seen);targets[IDS.index(12)]=sign*(math.pi+.1)
            with self.assertRaises(ValueError):wrapped.dispatch({'op':'write','targets':targets})
            self.assertEqual(len(seen),before)

    def test_manual_home_and_reset_both_use_saved_startup_reference(self):
        state=StanceState();state.initialize(HOME,reset_target=self.pose)
        for action in ('home','reset'):
            plan=state.plan({'action':action},HOME,[0.,0.,-1.])
            self.assertEqual(plan['target'],self.pose)

    def test_policy_cannot_take_over_until_actual_pose_and_gravity_have_settled(self):
        self.assertTrue(ready_for_handoff(self.pose,self.pose,[0,0,-1],50,'held'))
        bent=self.pose[:];bent[IDS.index(13)]+=math.radians(14)
        self.assertFalse(ready_for_handoff(bent,self.pose,[0,0,-1],50,'held'))
        self.assertFalse(ready_for_handoff(self.pose,self.pose,[.3,0,-.95],50,'held'))
        self.assertFalse(ready_for_handoff(self.pose,self.pose,[0,0,-1],30,'held'))
        self.assertFalse(ready_for_handoff(self.pose,self.pose,[0,0,-1],50,'homing'))
        loaded=self.pose[:];loaded[IDS.index(13)]-=math.radians(3.34)
        tilt=math.radians(11.7)
        self.assertTrue(ready_for_handoff(loaded,self.pose,[math.sin(tilt),0,-math.cos(tilt)],50,'held'))
        loaded[IDS.index(13)]=self.pose[IDS.index(13)]+math.radians(6.1)
        self.assertFalse(ready_for_handoff(loaded,self.pose,[0,0,-1],50,'held'))

    def test_floor_recovery_uses_actual_pose_and_keeps_health_and_gravity_validation(self):
        gravity=[1.,0.,0.]
        self.assertFalse(ready_for_handoff(self.pose,self.pose,gravity,50,'held'))
        self.assertTrue(ready_for_handoff(self.pose,self.pose,gravity,50,'held',floor_recovery=True))
        bent=self.pose[:];bent[IDS.index(12)]+=.2
        self.assertTrue(ready_for_handoff(bent,self.pose,gravity,50,'held',floor_recovery=True))
        for actual,g,hz,policy in [(self.pose,gravity,20,'held'),
                                   (self.pose,gravity,50,'walk'),(self.pose,[0,0,0],50,'held')]:
            self.assertFalse(ready_for_handoff(actual,self.pose,g,hz,policy,floor_recovery=True))

    def test_first_policy_target_jump_is_rejected_before_hardware_dispatch(self):
        seen=[]
        class Hardware:
            frame={'positions':self.pose[:]}
            def dispatch(self,request):seen.append(request);return {}
        wrapped=PolicyReferenceHardware(Hardware(),self.reference)
        now=[0.];wrapped.begin_handoff(clock=lambda:now[0])
        wrapped.dispatch({'op':'write','targets':list(HOME)})
        jump=list(HOME);jump[IDS.index(13)]+=math.radians(46)
        wrapped.dispatch({'op':'write','targets':jump})
        self.assertEqual(wrapped.handoff_rejection['joint_id'],13)
        self.assertAlmostEqual(wrapped.handoff_rejection['error_degrees'],46)
        self.assertEqual(seen[-1]['targets'],self.pose)
        now[0]=1.01
        wrapped.dispatch({'op':'write','targets':jump})
        self.assertEqual(seen[-1]['targets'],self.pose)

    def test_supported_recording_forwards_the_model_target_and_original_hardware_faults(self):
        seen=[]
        class Hardware:
            frame={'positions':self.pose[:]}
            def dispatch(self,request):
                seen.append(request)
                if request.get('targets',[0])[0]>.5:raise RuntimeError('original hardware limit')
                return {}
        wrapped=PolicyReferenceHardware(Hardware(),self.reference,record_targets_only=True)
        now=[0.];wrapped.begin_handoff(clock=lambda:now[0])
        proposal=list(HOME);proposal[IDS.index(12)]-=math.radians(19)
        wrapped.dispatch({'op':'write','targets':proposal})
        self.assertIsNone(wrapped.handoff_rejection)
        self.assertEqual(seen[-1]['targets'],self.reference.to_physical(proposal))
        self.assertAlmostEqual(wrapped.handoff_diagnostic['max_error_degrees'],19)
        self.assertFalse(wrapped.handoff_diagnostic['target_gap_blocks_policy'])
        with self.assertRaisesRegex(RuntimeError,'original hardware limit'):
            wrapped.dispatch({'op':'write','targets':[1.]*15})
        now[0]=1.01
        wrapped.dispatch({'op':'write','targets':proposal})
        self.assertEqual(seen[-1]['targets'],self.reference.to_physical(proposal))


class ClickStepTests(unittest.TestCase):
    def setUp(self):
        self.now=0.
        self.state=TeleopState(clock=lambda:self.now)
        self.state.ready=True;self.state.receive({'action':'confirm_balance'})

    def step(self,vx=.05,seconds=.5):
        self.state.receive({'action':'step','vx':vx,'seconds':seconds})

    def test_pulse_stops_at_deadline_even_if_heartbeats_continue(self):
        self.step()
        for value in (.1,.3,.49):
            self.now=value;self.state.receive({'action':'heartbeat'})
            self.assertEqual(self.state.command(),(.05,0.,0.))
        self.now=.5;self.state.receive({'action':'heartbeat'})
        self.assertEqual(self.state.command(),(0.,0.,0.))
        self.now=10.;self.assertEqual(self.state.command(),(0.,0.,0.))

    def test_reverse_stop_and_queued_duplicate_behave_independently(self):
        self.step(-.05)
        with self.assertRaises(ValueError):self.step()
        self.assertEqual(self.state.command(),(-.05,0.,0.))
        self.state.receive({'action':'drive','sequence':1,'vx':0,'vy':0,'vyaw':0})
        self.assertEqual(self.state.command(),(0.,0.,0.))
        self.step();self.state.receive({'action':'off'})
        self.assertEqual(self.state.command(),(0.,0.,0.))
        with self.assertRaises(ValueError):self.step()

    def test_step_cannot_bypass_confirmation_speed_duration_or_ready_gates(self):
        for vx,seconds in ((True,.5),(.15,.5),(.05,1.),(.05,float('nan'))):
            with self.assertRaises(ValueError):self.step(vx,seconds)
        self.state.ready=False
        with self.assertRaises(ValueError):self.step()
        self.state.ready=True;self.state.unlocked=False
        with self.assertRaises(ValueError):self.step()


if __name__=='__main__':unittest.main()
