use crate::groove::vars::RelaxedIKVars;
use crate::groove::groove::{OptimizationEngineOpen};
use crate::groove::objective_master::ObjectiveMaster;
use crate::utils_rust::file_utils::{*};
use crate::utils_rust::transformations::{*};
use nalgebra::{Vector3, UnitQuaternion, Quaternion};
use std::os::raw::{c_double, c_int};

#[repr(C)]
pub struct Opt {
    pub data: *const c_double,
    pub length: c_int,
}

pub struct RelaxedIK {
    pub vars: RelaxedIKVars,
    pub om: ObjectiveMaster,
    pub groove: OptimizationEngineOpen
}

impl RelaxedIK {
    pub fn load_settings( path_to_setting: &str) -> Self {
        println!("RelaxedIK is using below setting file {}", path_to_setting);

        let vars = RelaxedIKVars::from_local_settings(path_to_setting);
        let num_dofs = vars.robot.num_dofs;
        let om = ObjectiveMaster::relaxed_ik(&vars.robot.chain_lengths, &vars.chains_def, num_dofs, &vars.is_active_chain, &vars.arm_group, &vars.collision_starting_indices, vars.num_links_ee_to_tip);

        let groove = OptimizationEngineOpen::new(vars.robot.num_dofs.clone());

        Self{vars, om, groove}
    }

    pub fn reset(&mut self, x: Vec<f64>) {
        self.vars.reset( x.clone());
    }

    pub fn solve(&mut self) -> Vec<f64> {
        let mut out_x = self.vars.xopt.clone();

        let in_collision = self.vars.update_collision_world();

        if !false {
            self.groove.optimize(&mut out_x, &self.vars, &self.om, 100);

            // let frames = self.vars.robot.get_frames_immutable(&out_x);

            for i in 0..out_x.len() {
                if out_x[i].is_nan() {
                    println!("No valid solution found! Returning previous solution: {:?}. End effector position goals: {:?}", self.vars.xopt, self.vars.goal_positions);
                    // return self.vars.xopt.clone();
                    return self.vars.xopt.clone();
                }
            }
            self.vars.update(out_x.clone());  
        }
        if in_collision {
            println!("Robot in collision with obstacles!");
        }
        out_x
    }

    pub fn get_jointstate_loss(&mut self, x: Vec<f64>) -> f64 {
        self.om.call(&x[..], &self.vars)
    }
}
