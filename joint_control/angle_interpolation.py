'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''

from pid import PIDAgent
from keyframes import hello


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = None

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE

        if self.start_time is None:
            self.start_time = perception.time

        (names, times, keys) = keyframes
        start_time = perception.time - self.start_time

        for i in range(len(names)):
            joint = names[i]
            time = times[i]
            key = keys[i]

            if joint in self.joint_names:
                for j in range(len(time) - 1):
                    result = self.compute_interpolation(start_time, time, j, perception, joint, key)
                    if result is not None:
                        target_joints[joint] = result

        return target_joints

    def compute_interpolation(self, start_time, time, j, perception, joint, key):
        if start_time < time[0] and j == 0:
            t0 = 0.0
            t3 = time[0]
            p0 = perception.joint[joint]
            p3 = key[0][0]
        elif time[j] < start_time < time[j + 1]:
            t0 = time[j]
            t3 = time[j + 1]
            p0 = key[j][0]
            p3 = key[j + 1][0]
        else:
            return None

        p1 = key[j][1][1] + p0
        p2 = key[j][2][1] + p3

        t = (start_time - t0) / (t3 - t0)

        return self.bezier_interpolation(t, p0, p1, p2, p3)

    def bezier_interpolation(self, t, p0, p1, p2, p3):
        return (1 - t) ** 3 * p0 + 3 * t * (1 - t) ** 2 * p1 + 3 * t ** 2 * (1 - t) * p2 + t ** 3 * p3


if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
