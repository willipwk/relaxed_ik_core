#! /usr/bin/env python3

import ctypes
import os

class Opt(ctypes.Structure):
    _fields_ = [("data", ctypes.POINTER(ctypes.c_double)), ("length", ctypes.c_int)]

class RelaxedIKS(ctypes.Structure):
    pass


class StringArray(ctypes.Structure):
    _fields_ = [("data", ctypes.POINTER(ctypes.c_char_p)),
                ("len", ctypes.c_size_t)]
    
    
dir_path = os.path.dirname(os.path.realpath(__file__))
lib = ctypes.cdll.LoadLibrary(dir_path + '/../target/debug/librelaxed_ik_lib.so')

lib.relaxed_ik_new.restype = ctypes.POINTER(RelaxedIKS)
lib.solve.argtypes = [ctypes.POINTER(RelaxedIKS), ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_double)]
lib.solve.restype = Opt
lib.solve_position.argtypes = [ctypes.POINTER(RelaxedIKS), ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int]
lib.solve_position.restype = Opt
lib.solve_velocity.argtypes = [ctypes.POINTER(RelaxedIKS), ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int]
lib.solve_velocity.restype = Opt
lib.reset.argtypes = [ctypes.POINTER(RelaxedIKS)]
lib.get_jointstate_loss.argtypes = [ctypes.POINTER(RelaxedIKS)]
lib.get_jointstate_loss.restype = ctypes.c_double
lib.get_objective_weight_names.argtypes = [ctypes.POINTER(RelaxedIKS)]
lib.get_objective_weight_names.restype = ctypes.POINTER(StringArray)

lib.get_objective_weight_priors.argtypes = [ctypes.POINTER(RelaxedIKS)]
lib.get_objective_weight_priors.restype  = Opt
lib.set_objective_weight_priors.argtypes = [ctypes.POINTER(RelaxedIKS), ctypes.POINTER(ctypes.c_double), ctypes.c_int]
lib.set_objective_weight_priors.restype  = None


class RelaxedIKRust:
    def __init__(self, setting_file_path = None):
        '''
        setting_file_path (string): path to the setting file
                                    if no path is given, the default setting file will be used
                                    /configs/settings.yaml
        '''
        if setting_file_path is None:
            self.obj = lib.relaxed_ik_new(ctypes.c_char_p())
        else:
            self.obj = lib.relaxed_ik_new(ctypes.c_char_p(setting_file_path.encode('utf-8')))
    
    def __exit__(self, exc_type, exc_value, traceback):
        lib.relaxed_ik_free(self.obj)
    
    def solve_position(self, positions, orientations, tolerances):
        '''
        Assuming the robot has N end-effectors
        positions (1D array with length as 3*N): list of end-effector positions
        orientations (1D array with length as 4*N): list of end-effector orientations (in quaternion xyzw format)
        tolerances (1D array with length as 6*N): list of tolerances for each end-effector (x, y, z, rx, ry, rz)
        '''
        pos_arr = (ctypes.c_double * len(positions))()
        quat_arr = (ctypes.c_double * len(orientations))()
        tole_arr = (ctypes.c_double * len(tolerances))()
        for i in range(len(positions)):
            pos_arr[i] = positions[i]
        for i in range(len(orientations)):
            quat_arr[i] = orientations[i]
        for i in range(len(tolerances)):
            tole_arr[i] = tolerances[i]
        xopt = lib.solve_position(self.obj, pos_arr, len(pos_arr), quat_arr, len(quat_arr), tole_arr, len(tole_arr))
        return xopt.data[:xopt.length]
    
    def solve_velocity(self, linear_velocities, angular_velocities, tolerances):
        '''
        Assuming the robot has N end-effectors
        linear_velocities (1D array with length as 3*N): list of end-effector linear velocities
        angular_velocities (1D array with length as 4*N): list of end-effector angular velocities
        tolerances (1D array with length as 6*N): list of tolerances for each end-effector (x, y, z, rx, ry, rz)
        '''
        linear_arr = (ctypes.c_double * len(linear_velocities))()
        angular_arr = (ctypes.c_double * len(angular_velocities))()
        tole_arr = (ctypes.c_double * len(tolerances))()
        for i in range(len(linear_velocities)):
            linear_arr[i] = linear_velocities[i]
        for i in range(len(angular_velocities)):
            angular_arr[i] = angular_velocities[i]
        for i in range(len(tolerances)):
            tole_arr[i] = tolerances[i]
        xopt = lib.solve_velocity(self.obj, linear_arr, len(linear_arr), angular_arr, len(angular_arr), tole_arr, len(tole_arr))
        return xopt.data[:xopt.length]
    
    def reset(self, joint_state):
        js_arr = (ctypes.c_double * len(joint_state))()
        for i in range(len(joint_state)):
            js_arr[i] = joint_state[i]
        lib.reset(self.obj, js_arr, len(js_arr))
    
    def get_jointstate_loss(self, joint_state) -> float:
        js_arr = (ctypes.c_double * len(joint_state))()
        for i in range(len(joint_state)):
            js_arr[i] = joint_state[i]
        return lib.get_jointstate_loss(self.obj, js_arr, len(js_arr))
        
    def dynamic_obstacle_cb(self, marker_name: str, pose):
        pos_arr = (ctypes.c_double * 3)()
        quat_arr = (ctypes.c_double * 4)()
        pos_arr[0]  = pose.position.x
        pos_arr[1]  = pose.position.y
        pos_arr[2]  = pose.position.z
        quat_arr[0] = pose.orientation.x
        quat_arr[1] = pose.orientation.y
        quat_arr[2] = pose.orientation.z
        quat_arr[3] = pose.orientation.w 
        
        name_bytes = marker_name.encode('utf-8') # Convert to bytes
        c_name = ctypes.c_char_p(name_bytes)  # Create ctypes char pointer
        
        lib.dynamic_obstacle_cb(self.obj, c_name, pos_arr, quat_arr)
        
    def get_objective_weight_names(self):
        string_array_ptr = lib.get_objective_weight_names(self.obj)
        strings = [string_array_ptr.contents.data[i].decode('utf-8') for i in range(string_array_ptr.contents.len)]
        return strings
    
    def get_objective_weight_priors(self):
        w = lib.get_objective_weight_priors(self.obj)
        return w.data[:w.length]
    
    def set_objective_weight_priors(self, w):
        w_arr = (ctypes.c_double * len(w))()
        for i in range(len(w)):
            w_arr[i] = w[i]
        lib.set_objective_weight_priors(self.obj, w_arr, len(w_arr))
    
    
if __name__ == '__main__':
    pass
