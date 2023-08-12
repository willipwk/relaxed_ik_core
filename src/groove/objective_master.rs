use crate::groove::objective::*;
use crate::groove::vars::RelaxedIKVars;
use std::collections::BTreeSet;
pub struct ObjectiveMaster {
    pub objectives: Vec<Box<dyn ObjectiveTrait + Send>>,
    pub num_chains: usize,
    pub weight_priors: Vec<f64>,
    pub weight_names: Vec<String>,
    pub lite: bool,
    pub finite_diff_grad: bool,
}

impl ObjectiveMaster {
    // pub fn standard_ik(num_chains: usize) -> Self {
    //     let mut objectives: Vec<Box<dyn ObjectiveTrait + Send>> = Vec::new();
    //     let mut weight_priors: Vec<f64> = Vec::new();
    //     let mut weight_names: Vec<String> = Vec::new();
    //     for i in 0..num_chains {
    //         objectives.push(Box::new(MatchEEPosGoals::new(i)));
    //         weight_priors.push(1.0);
    //         weight_names.push(String::from("eepos"));
    //         objectives.push(Box::new(MatchEEQuatGoals::new(i)));
    //         weight_priors.push(1.0);
    //         weight_names.push(String::from("eequat"));
    //     }
    //     Self{objectives, num_chains, weight_priors, weight_names, lite: true, finite_diff_grad: true}
    // }


    pub fn relaxed_ik(arm_link_names: &Vec<Vec<String>>, chain_lengths: &[usize], chains_def: &Vec<Vec<i64>>, num_dofs: usize, is_active_chain: &[bool], arm_group: &[usize], collision_starting_indices: &[usize], disabled_collisions: &BTreeSet<(String, String)>, num_links_ee_to_tip: i64) -> Self {
        let mut objectives: Vec<Box<dyn ObjectiveTrait + Send>> = Vec::new();
        let mut weight_priors: Vec<f64> = Vec::new();
        let mut weight_names: Vec<String> = Vec::new();
        let num_chains = chain_lengths.len();
        // let mut num_dofs = 0;
        println!("collision_starting_indices: {:?}",collision_starting_indices);
        println!("chain_lengths: {:?}", chain_lengths);
        println!("arm_group: {:?}", arm_group);
        println!("disabled_collisions: {:?}", disabled_collisions);
        for i in 0..num_chains {
            if is_active_chain[i] {
                // objectives.push(Box::new(MatchEEPosiDoF::new(i, 0)));
                // weight_priors.push(50.0);
                // weight_names.push(String::from("eepos"));
                // objectives.push(Box::new(MatchEEPosiDoF::new(i, 1)));
                // weight_priors.push(50.0);
                // weight_names.push(String::from("eepos"));
                // objectives.push(Box::new(MatchEEPosiDoF::new(i, 2)));
                // weight_priors.push(50.0);
                // weight_names.push(String::from("eepos"));
                // objectives.push(Box::new(MatchEERotaDoF::new(i, 0)));
                // weight_priors.push(10.0);
                // weight_names.push(String::from("eequat"));
                // objectives.push(Box::new(MatchEERotaDoF::new(i, 1)));
                // weight_priors.push(10.0);
                // weight_names.push(String::from("eequat"));
                // objectives.push(Box::new(MatchEERotaDoF::new(i, 2)));
                // weight_priors.push(10.0);
                // weight_names.push(String::from("eequat"));

                objectives.push(Box::new(MatchEEPosGoals::new(i, num_links_ee_to_tip as usize)));
                weight_priors.push(50.0);
                weight_names.push(String::from("eepos"));
                objectives.push(Box::new(MatchEEQuatGoals::new(i, num_links_ee_to_tip as usize)));
                weight_priors.push(30.0);
                weight_names.push(String::from("eequat"));
            }
            objectives.push(Box::new(EnvCollision::new(i, collision_starting_indices[i])));
            weight_priors.push(5.0);
            weight_names.push(format!("envcollision_{}",i));
            // num_dofs += chain_lengths[i];
        }

        for j in 0..num_dofs {
            // println!("JointLimitIdx:{:?}",j);
            objectives.push(Box::new(EachJointLimits::new(j))); weight_priors.push(1.0);
            weight_names.push(String::from("jointlimit"));
        }

        objectives.push(Box::new(MinimizeVelocity));        weight_priors.push(0.7);  weight_names.push(String::from("minvel"));
        objectives.push(Box::new(MinimizeAcceleration));    weight_priors.push(0.5);  weight_names.push(String::from("minacc"));
        objectives.push(Box::new(MinimizeJerk));            weight_priors.push(0.3);  weight_names.push(String::from("minjerk"));
        objectives.push(Box::new(MaximizeManipulability));  weight_priors.push(1.0);  weight_names.push(String::from("maxmanip"));

        let mut collisions : BTreeSet<(String, String)> = BTreeSet::new();
        for i in 0..num_chains {
            if !is_active_chain[i] {
                continue;
            }
            for j in collision_starting_indices[i]..chain_lengths[i]-2 {
                for k in j+2..chain_lengths[i] {
                    let link_name_1 = arm_link_names[i][j].clone();
                    let link_name_2 = arm_link_names[i][k].clone();
                    let link_pair = if link_name_1 < link_name_2 {(link_name_1, link_name_2)} else {(link_name_2, link_name_1)};
                    if collisions.contains(&link_pair) {
                        println!("Dup1! {:?}",link_pair);
                    }
                    if disabled_collisions.contains(&link_pair) || collisions.contains(&link_pair){
                        continue;
                    }
                    
                    collisions.insert(link_pair);
                    objectives.push(Box::new(SelfCollision::new(i, i,  j, k, false, false))); 
                    weight_priors.push(0.1);
                    weight_names.push(String::from("selfcollision"));
                }
            }
        }
        
        for a1 in 0..num_chains {
            for a2 in a1..num_chains {
                if arm_group[a1] == arm_group[a2] {
                    continue;
                }
                for i in collision_starting_indices[a1]..chain_lengths[a1] {
                    for j in collision_starting_indices[a2]..chain_lengths[a2] {
                        let link_name_1 = arm_link_names[a1][i].clone();
                        let link_name_2 = arm_link_names[a2][j].clone();
                        let link_pair = if link_name_1 < link_name_2 {(link_name_1, link_name_2)} else {(link_name_2, link_name_1)};
                        if collisions.contains(&link_pair) {
                            println!("Dup2! {:?}",link_pair);
                        }
                        if disabled_collisions.contains(&link_pair) || collisions.contains(&link_pair){
                            continue;
                        }
                        
                        let is_ee_link_0 = i >= chain_lengths[a1] - 3;
                        let is_ee_link_1 = j >= chain_lengths[a2] - 3;

                        collisions.insert(link_pair);
                        objectives.push(Box::new(SelfCollision::new(a1, a2,  i, j, is_ee_link_0, is_ee_link_1))); 
                        if is_ee_link_0 && is_ee_link_1 {
                            weight_priors.push(0.05);
                            weight_names.push(String::from("selfcollision_ee"));
                        } else {
                            weight_priors.push(0.1);
                            weight_names.push(String::from("selfcollision"));
                        }
                        
                        
                    }
                }
            }
        }

        
        Self{objectives, num_chains, weight_priors, weight_names, lite: false, finite_diff_grad: false}
    }

    pub fn call(&self, x: &[f64], vars: &RelaxedIKVars) -> f64 {
        if self.lite {
            self.__call_lite(x, vars)
        } else {
            self.__call(x, vars)
        }
    }

    pub fn gradient(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
        if self.lite {
            if self.finite_diff_grad {
                self.__gradient_finite_diff_lite(x, vars)
            } else {
                self.__gradient_lite(x, vars)
            }
        } else {
            if self.finite_diff_grad {
                self.__gradient_finite_diff(x, vars)
            } else {
                self.__gradient(x, vars)
            }
        }
    }

    pub fn gradient_finite_diff(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
        if self.lite {
            self.__gradient_finite_diff_lite(x, vars)
        } else {
            self.__gradient_finite_diff(x, vars)
        }
    }

    fn __call(&self, x: &[f64], vars: &RelaxedIKVars) -> f64 {
        let mut out = 0.0;
        let frames = vars.robot.get_frames_immutable(x);
        let mut temp: Vec<f64> = Vec::new();
        for i in 0..self.objectives.len() {
            let l = self.weight_priors[i] * self.objectives[i].call(x, vars, &frames);
            temp.push(l);
            out += l;
        }
        // println!("temp: {:?}", temp);
        out
    }

    fn __call_lite(&self, x: &[f64], vars: &RelaxedIKVars) -> f64 {
        let mut out = 0.0;
        let poses = vars.robot.get_ee_pos_and_quat_immutable(x);
        for i in 0..self.objectives.len() {
            out += self.weight_priors[i] * self.objectives[i].call_lite(x, vars, &poses);
        }
        out
    }

    fn __gradient(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = vec![0. ; x.len()];
        let mut obj = 0.0;

        let mut finite_diff_list: Vec<usize> = Vec::new();
        let mut f_0s: Vec<f64> = Vec::new();
        let frames_0 = vars.robot.get_frames_immutable(x);
        for i in 0..self.objectives.len() {
            if self.objectives[i].gradient_type() == 0 {
                let (local_obj, local_grad) = self.objectives[i].gradient(x, vars, &frames_0);
                f_0s.push(local_obj);
                obj += self.weight_priors[i] * local_obj;
                for j in 0..local_grad.len() {
                    grad[j] += self.weight_priors[i] * local_grad[j];
                }
            } else if self.objectives[i].gradient_type() == 1 {
                finite_diff_list.push(i);
                let local_obj = self.objectives[i].call(x, vars, &frames_0);
                obj += self.weight_priors[i] * local_obj;
                f_0s.push(local_obj);
            }
        }

        if finite_diff_list.len() > 0 {
            for i in 0..x.len() {
                let mut x_h = x.to_vec();
                x_h[i] += 0.0000001;
                let frames_h = vars.robot.get_frames_immutable(x_h.as_slice());
                for j in &finite_diff_list {
                    let f_h = self.objectives[*j].call(&x_h, vars, &frames_h);
                    grad[i] += self.weight_priors[*j] * ((-f_0s[*j] + f_h) /  0.0000001);
                }
            }
        }

        (obj, grad)
    }

    fn __gradient_lite(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = vec![0. ; x.len()];
        let mut obj = 0.0;

        let mut finite_diff_list: Vec<usize> = Vec::new();
        let mut f_0s: Vec<f64> = Vec::new();
        let poses_0 = vars.robot.get_ee_pos_and_quat_immutable(x);
        for i in 0..self.objectives.len() {
            if self.objectives[i].gradient_type() == 1 {
                let (local_obj, local_grad) = self.objectives[i].gradient_lite(x, vars, &poses_0);
                f_0s.push(local_obj);
                obj += self.weight_priors[i] * local_obj;
                for j in 0..local_grad.len() {
                    grad[j] += self.weight_priors[i] * local_grad[j];
                }
            } else if self.objectives[i].gradient_type() == 0 {
                finite_diff_list.push(i);
                let local_obj = self.objectives[i].call_lite(x, vars, &poses_0);
                obj += self.weight_priors[i] * local_obj;
                f_0s.push(local_obj);
            }
        }

        if finite_diff_list.len() > 0 {
            for i in 0..x.len() {
                let mut x_h = x.to_vec();
                x_h[i] += 0.0000001;
                let poses_h = vars.robot.get_ee_pos_and_quat_immutable(x_h.as_slice());
                for j in &finite_diff_list {
                    let f_h = self.objectives[*j].call_lite(x, vars, &poses_h);
                    grad[i] += self.weight_priors[*j] * ((-f_0s[*j] + f_h) /  0.0000001);
                }
            }
        }

        (obj, grad)
    }

    fn __gradient_finite_diff(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>)  {
        let mut grad: Vec<f64> = vec![0. ; x.len()];
        let mut f_0 = self.call(x, vars);

        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.000001;
            let f_h = self.call(x_h.as_slice(), vars);
            grad[i] = (-f_0 + f_h) / 0.000001;
        }

        (f_0, grad)
    }

    fn __gradient_finite_diff_lite(&self, x: &[f64], vars: &RelaxedIKVars) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = vec![0. ; x.len()];
        let mut f_0 = self.call(x, vars);

        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.000001;
            let f_h = self.__call_lite(x_h.as_slice(), vars);
            grad[i] = (-f_0 + f_h) / 0.000001;
        }

        (f_0, grad)
    }
}