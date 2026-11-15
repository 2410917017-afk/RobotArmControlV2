import json
from utils.serial_worker import SerialWorker
from utils.teach_store import load_points, save_points
from utils.kinematics import forward, inverse, circular_path
from dataclasses import dataclass

from utils.robot_config import (
    DEFAULT_CARTESIAN_JOG_MM,
    DEFAULT_FEED_STEPS_S,
    DEFAULT_JOINT_JOG_DEG,
    DEFAULT_ANGULAR_JOG_DEG,
    JOINT_LIMITS_DEG,
    JOINT_NAMES,
    MAX_FEED_STEPS_S,
)

class tooldata:



class jtarget:
    def __init__(self, joints_deg: list[float]) -> None:
        self.valid = True

        for i, angulo in enumerate(joints_deg):
            lowlim, maxlim = JOINT_LIMITS_DEG[i]

            if angulo <  lowlim or angulo > maxlim:
                self.valid = False 
                print("Error. Angulos fuera de los límites articulares")
                return
            


class ctarget:
    def __init__(self, coord: list[float], orient: list[float]) -> None:
        self.valid = True

        result = inverse(
            coord,
            orient[0],
            orient[1],
            self.current_joints,
        )
        if not result.success:
            self.valid = False
            return


class robot:
    def __init__(self) -> None:
        self.worker: SerialWorker | None = None


    def connect(self, port: str) -> None:
        self.worker = SerialWorker(port)
        self.worker.start()


    def jMove(self, target: jtarget):
        if target.valid == False:
            print("Error. jtarget no valido.")


    def lMove(self, target: ctarget):


    def jAbsMove(self, target: jtarget):


    def lAbsMove(self, target: ctarget):


    def cMove(self, radius: float, plane: tuple, endp: ctarget):


    def toolAction(self, state: bool):


    def send_json(self, command_dict):


    def stop(self):


    def updateParams(self, )
    






