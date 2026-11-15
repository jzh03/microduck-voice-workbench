import unittest
from teleop_state import TeleopState


class TeleopTests(unittest.TestCase):
    def setUp(self):
        self.now=0.
        self.state=TeleopState(clock=lambda:self.now)

    def drive(self,sequence=1,vx=.2):
        return self.state.receive({'action':'drive','sequence':sequence,'vx':vx,'vy':0,'vyaw':0})

    def unlock(self):
        self.state.ready=True
        self.state.receive({'action':'confirm_balance'})

    def test_motion_requires_observed_independent_standing(self):
        with self.assertRaisesRegex(ValueError,'Confirm'):self.drive()
        self.assertEqual(self.state.command(),(0.,0.,0.))
        with self.assertRaisesRegex(ValueError,'not ready'):
            self.state.receive({'action':'confirm_balance'})

    def test_configured_trial_speed_keeps_ready_confirmation_and_board_timer(self):
        trial=TeleopState(clock=lambda:self.now,step_speed=.3)
        with self.assertRaises(ValueError):trial.receive({'action':'step','vx':.3,'seconds':.5})
        trial.ready=True;trial.receive({'action':'confirm_balance'})
        with self.assertRaises(ValueError):trial.receive({'action':'step','vx':.05,'seconds':.5})
        trial.receive({'action':'step','vx':-.3,'seconds':.5})
        self.assertEqual(trial.command(),(-.3,0.,0.))
        self.now=.5;self.assertEqual(trial.command(),(0.,0.,0.))
        for value in (True,.4,float('nan')):
            with self.assertRaises(ValueError):TeleopState(step_speed=value)

    def test_stale_driver_stops_walking_without_dropping_balance(self):
        self.unlock();self.drive();self.now=.31
        self.assertEqual(self.state.command(),(0.,0.,0.))
        self.now=10
        self.assertIsNone(self.state.abort_reason())

    def test_heartbeat_does_not_refresh_a_stuck_motion_command(self):
        self.unlock();self.drive();self.now=.4
        self.state.receive({'action':'heartbeat'})
        self.assertEqual(self.state.command(),(0.,0.,0.))

    def test_supported_startup_link_loss_aborts(self):
        self.state.receive({'action':'heartbeat'});self.now=.81
        self.assertIn('connection lost',self.state.abort_reason())

    def test_emergency_stop_cannot_be_cleared_by_queued_commands(self):
        self.unlock();self.drive();self.state.receive({'action':'off'})
        with self.assertRaisesRegex(ValueError,'stopped'):self.drive(2)
        self.assertEqual(self.state.command(),(0.,0.,0.))
        self.assertIsNotNone(self.state.abort_reason())

    def test_stale_out_of_range_and_nonfinite_commands_are_rejected(self):
        self.unlock();self.drive()
        for sequence,vx in [(1,0),(2,1.01),(2,float('nan')),(2,float('inf'))]:
            with self.assertRaises(ValueError):self.drive(sequence,vx)
        self.assertEqual(self.state.command(),(.2,0,0))

    def test_one_meter_per_second_boundary_keeps_unlock_and_deadman(self):
        with self.assertRaises(ValueError):self.drive(vx=1.)
        self.unlock()
        self.drive(sequence=1,vx=1.)
        self.assertEqual(self.state.command(),(1.,0,0))
        self.drive(sequence=2,vx=-1.)
        self.assertEqual(self.state.command(),(-1.,0,0))
        for vx in (1.01,-1.01):
            with self.assertRaises(ValueError):self.drive(sequence=3,vx=vx)
        self.now=.31
        self.assertEqual(self.state.command(),(0.,0.,0.))

    def test_pitch_needs_supported_ready_session_and_small_steps(self):
        with self.assertRaisesRegex(ValueError,'ready'):
            self.state.receive({'action':'stand_pitch','degrees':2})
        self.state.ready=True
        for pitch in [True,float('nan'),float('inf'),-8,8,4,-4]:
            with self.assertRaises(ValueError):
                self.state.receive({'action':'stand_pitch','degrees':pitch})
        self.state.receive({'action':'stand_pitch','degrees':2})
        self.assertFalse(self.state.ready)
        self.assertFalse(self.state.public()['can_confirm_balance'])
        self.assertAlmostEqual(self.state.pose()['pitch'],.034906585,places=7)
        self.assertEqual(self.state.command(),(0.,0.,0.))

    def test_negative_pitch_uses_same_step_bounds_and_walking_gate(self):
        for pitch in [-2,-4,-6]:
            self.state.ready=True
            self.state.receive({'action':'stand_pitch','degrees':pitch})
            self.assertAlmostEqual(self.state.pose()['pitch'],pitch*3.141592653589793/180)
            self.assertFalse(self.state.public()['can_confirm_balance'])
        self.state.ready=True
        with self.assertRaises(ValueError):self.state.receive({'action':'stand_pitch','degrees':-8})
        with self.assertRaisesRegex(ValueError,'zero'):self.state.receive({'action':'confirm_balance'})

    def test_positive_pitch_reaches_sixteen_only_in_two_degree_steps(self):
        for pitch in range(2,17,2):
            self.state.ready=True
            self.state.receive({'action':'stand_pitch','degrees':pitch})
            self.assertAlmostEqual(self.state.public()['stand_pitch_deg'],pitch)
            self.assertFalse(self.state.public()['can_confirm_balance'])
            self.assertFalse(self.state.ready)
        self.state.ready=True
        with self.assertRaises(ValueError):self.state.receive({'action':'stand_pitch','degrees':18})
        with self.assertRaises(ValueError):self.state.receive({'action':'stand_pitch','degrees':12})
        self.state.receive({'action':'stand_pitch','degrees':14})

    def test_pitch_cannot_unlock_walking_and_does_not_bypass_deadman(self):
        self.state.ready=True
        self.state.receive({'action':'stand_pitch','degrees':2})
        self.state.ready=True
        with self.assertRaisesRegex(ValueError,'zero'):
            self.state.receive({'action':'confirm_balance'})
        with self.assertRaisesRegex(ValueError,'Confirm'):self.drive()
        self.now=.81
        self.assertIn('connection lost',self.state.abort_reason())

    def test_pitch_can_return_to_zero_but_is_locked_after_walking_unlock(self):
        self.state.ready=True
        self.state.receive({'action':'stand_pitch','degrees':2})
        self.state.ready=True
        self.state.receive({'action':'stand_pitch','degrees':0})
        self.assertTrue(self.state.pose()['active'])
        self.unlock()
        self.assertFalse(self.state.pose()['active'])
        with self.assertRaisesRegex(ValueError,'supported'):
            self.state.receive({'action':'stand_pitch','degrees':2})
        self.state.receive({'action':'off'})
        self.assertFalse(self.state.public()['can_confirm_balance'])
        with self.assertRaisesRegex(ValueError,'stopped'):
            self.state.receive({'action':'stand_pitch','degrees':2})

    def test_height_bounds_units_and_shared_settling_gate(self):
        with self.assertRaisesRegex(ValueError,'ready'):
            self.state.receive({'action':'stand_height','millimeters':-1})
        self.state.ready=True
        for value in [True,float('nan'),float('inf'),1,-12,-4]:
            with self.assertRaises(ValueError):
                self.state.receive({'action':'stand_height','millimeters':value})
        for value in range(-1,-11,-1):
            self.state.ready=True
            self.state.receive({'action':'stand_height','millimeters':value})
            self.assertAlmostEqual(self.state.pose()['z'],value/1000)
            self.assertFalse(self.state.public()['can_adjust_pitch'])
            self.assertFalse(self.state.public()['can_adjust_height'])
            with self.assertRaisesRegex(ValueError,'ready'):
                self.state.receive({'action':'stand_pitch','degrees':2})
        self.state.ready=True
        with self.assertRaises(ValueError):
            self.state.receive({'action':'stand_height','millimeters':-12})

    def test_nonzero_height_blocks_walking_and_preserves_deadman(self):
        self.state.ready=True
        self.state.receive({'action':'stand_height','millimeters':-1})
        self.state.ready=True
        self.assertFalse(self.state.public()['can_confirm_balance'])
        with self.assertRaisesRegex(ValueError,'height to zero'):
            self.state.receive({'action':'confirm_balance'})
        with self.assertRaisesRegex(ValueError,'Confirm'):self.drive()
        self.now=.81
        self.assertIn('connection lost',self.state.abort_reason())
        self.state.receive({'action':'stand_height','millimeters':0})
        self.unlock()
        with self.assertRaisesRegex(ValueError,'supported'):
            self.state.receive({'action':'stand_height','millimeters':-1})
        self.state.receive({'action':'off'})
        self.assertFalse(self.state.public()['can_adjust_height'])
        with self.assertRaisesRegex(ValueError,'stopped'):
            self.state.receive({'action':'stand_height','millimeters':-1})

    def test_old_pose_health_cannot_reopen_new_pose_settling_gate(self):
        old_pose=self.state.pose()
        self.state.set_ready_for_pose(old_pose,True)
        self.state.receive({'action':'stand_height','millimeters':-1})
        self.state.set_ready_for_pose(old_pose,True)
        self.assertFalse(self.state.ready)
        with self.assertRaisesRegex(ValueError,'ready'):
            self.state.receive({'action':'stand_height','millimeters':-2})
        self.state.set_ready_for_pose(self.state.pose(),True)
        self.assertTrue(self.state.ready)
        self.state.receive({'action':'off'})
        self.state.set_ready_for_pose(self.state.pose(),True)
        self.assertFalse(self.state.ready)


if __name__=='__main__':unittest.main()
