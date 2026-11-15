"""A bounded drag/hold target follower for hand-supported pose teaching.

Position alone cannot distinguish a hand from gravity. This mode therefore
requires a supported robot; it is not a balance or load-bearing controller.
Targets follow measured motion with a small lag, never an invented stand pose.
"""
from collections import deque
import math


class SoftFollower:
    def __init__(self, angle, now):
        self.target = float(angle)
        self.dragging = False
        self.direction = 0
        self.samples = deque([(now, float(angle))])
        self.last_motion = now
        self.motion_anchor = float(angle)

    def update(self, angle, now):
        if not math.isfinite(angle) or not math.isfinite(now):
            raise ValueError('Invalid pose sample')
        if now < self.samples[-1][0]:
            raise ValueError('Pose sample time moved backwards')
        self.samples.append((now, float(angle)))
        # Retain one sample before the window for a meaningful duration.
        while len(self.samples) > 2 and self.samples[1][0] < now-.25:
            self.samples.popleft()
        displacement = angle-self.samples[0][1]
        if not self.dragging:
            if abs(angle-self.target) < .9 or abs(displacement) < .3:
                return None
            self.dragging = True
            self.direction = 1 if displacement > 0 else -1
            self.last_motion = now
            self.motion_anchor = angle
            self.target = angle-self.direction*.6
            return self.target
        else:
            delta = angle-self.motion_anchor
            if delta*self.direction <= -.18:
                # Reversal may be the motor relaxing its own spring, not a hand.
                # Capture here, then require a new external deflection to follow.
                self.dragging = False
                self.target = angle
                self.samples.clear()
                self.samples.append((now, angle))
                return self.target
            if delta*self.direction >= .18 and (angle-self.target)*self.direction > .6:
                self.motion_anchor = angle
                self.last_motion = now
                self.target = angle-self.direction*.6
                return self.target
            if now-self.last_motion >= .25:
                self.dragging = False
                self.target = angle
                self.samples.clear()
                self.samples.append((now, angle))
                return self.target
        # A stationary hand must never cause a new motion target.
        return None
