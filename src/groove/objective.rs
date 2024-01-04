use crate::groove::{vars};
use crate::utils_rust::transformations::{*};
use nalgebra::geometry::{Translation3, UnitQuaternion, Quaternion};
use std::cmp;
use std::f64::consts::PI;
use crate::groove::vars::RelaxedIKVars;
use nalgebra::{Vector3, Isometry3, Point3, Rotation, Unit};
use std::ops::Deref;
use time::PreciseTime;
use parry3d_f64::{shape, query};
use ncollide3d::{shape as shape_nc, query as query_nc};

pub fn groove_loss(x_val: f64, t: f64, d: i32, c: f64, f: f64, g: i32) -> f64 {
    -( (-(x_val - t).powi(d)) / (2.0 * c.powi(2) ) ).exp() + f * (x_val - t).powi(g)
}

pub fn groove_loss_derivative(x_val: f64, t: f64, d: i32, c: f64, f: f64, g: i32) -> f64 {
    -( (-(x_val - t).powi(d)) / (2.0 * c.powi(2) ) ).exp() *  ((-d as f64 * (x_val - t)) /  (2.0 * c.powi(2))) + g as f64 * f * (x_val - t)
}

pub fn swamp_groove_loss(x_val: f64, g:f64, l_bound: f64, u_bound: f64, c : f64, f1: f64, f2: f64, f3:f64, p1:i32) -> f64 {
    let x = (2.0 * x_val - l_bound - u_bound) / (u_bound - l_bound);
    let b = (-1.0 / (0.05 as f64).ln()).powf(1.0 / p1 as f64);
    - f1 * ( (-(x_val-g).powi(2)) / (2.0 * c.powi(2)) ).exp() 
    + f2 * (x_val-g).powi(2) 
    + f3 * (1.0 - (-(x/b).powi(p1)).exp()) 
}

pub fn swamp_loss(x_val: f64, l_bound: f64, u_bound: f64, f1: f64, f2: f64, p1:i32) -> f64 {
    let x = (2.0 * x_val - l_bound - u_bound) / (u_bound - l_bound);
    let b = (-1.0 / (0.05 as f64).ln()).powf(1.0 / p1 as f64);
    (f1 + f2 * x.powi(2)) *  (1.0 - (- (x/b).powi(p1)).exp()) - 1.0
}

pub fn swamp_groove_loss_derivative(x_val: f64, g:f64, l_bound: f64, u_bound: f64, c : f64, f1: f64, f2: f64, f3:f64, p1:i32) -> f64 {
    if (2.0 * x_val - l_bound - u_bound).abs() < 1e-8 {
        return 0.0;
    }
    let x = (2.0 * x_val - l_bound - u_bound) / (u_bound - l_bound);
    let b = (-1.0 / (0.05 as f64).ln()).powf(1.0 / p1 as f64);

    - f1 * ( (-x_val.powi(2)) / (2.0 * c.powi(2) ) ).exp() *  ((-2.0 * x_val) /  (2.0 * c.powi(2))) 
    + 2.0 * f2 * x_val 
    + f3 / (2.0 * x_val - l_bound - u_bound) * ( 2.0 * (x/b).powi(p1) * p1 as f64 * (- (x/b).powi(p1)).exp()) 
}

pub trait ObjectiveTrait {
    fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64;
    fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64;
    fn gradient(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = Vec::new();
        let f_0 = self.call(x, v, frames);
        
        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.000000001;
            let frames_h = v.robot.get_frames_immutable(x_h.as_slice());
            let f_h = self.call(x_h.as_slice(), v, &frames_h);
            grad.push( (-f_0 + f_h) / 0.000000001);
        }

        (f_0, grad)
    }
    fn gradient_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> (f64, Vec<f64>) {
        let mut grad: Vec<f64> = Vec::new();
        let f_0 = self.call_lite(x, v, ee_poses);

        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.0000001;
            let ee_poses_h = v.robot.get_ee_pos_and_quat_immutable(x_h.as_slice());
            let f_h = self.call_lite(x_h.as_slice(), v, &ee_poses_h);
            grad.push( (-f_0 + f_h) / 0.0000001);
        }

        (f_0, grad)
    }
    fn gradient_type(&self) -> usize {return 1}  // manual diff = 0, finite diff = 1
}


pub struct MatchEEPosiDoF {
    pub arm_idx: usize,
    pub axis: usize
}
impl MatchEEPosiDoF {
    pub fn new(arm_idx: usize, axis: usize) -> Self {Self{arm_idx, axis}}
}
impl ObjectiveTrait for MatchEEPosiDoF {
    fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        let last_elem = frames[self.arm_idx].0.len() - 1;
        let goal_quat = v.goal_quats[self.arm_idx];
        // E_{gc} = R_{gw} * T_{gw} * T_{wc} * R_{wc}, R_{wc} won't matter since we are only interested in the translation
        // so  we get: T_{gc} = R_{gw} * T_{gw} * T_{wc}
        let T_gw_T_wc =  nalgebra::Vector3::new(    frames[self.arm_idx].0[last_elem].x - v.goal_positions[self.arm_idx].x, 
                                                    frames[self.arm_idx].0[last_elem].y - v.goal_positions[self.arm_idx].y, 
                                                    frames[self.arm_idx].0[last_elem].z - v.goal_positions[self.arm_idx].z );

        let T_gc = goal_quat.inverse() * T_gw_T_wc;
 
        let dist: f64 = T_gc[self.axis];

        let bound =  v.tolerances[self.arm_idx][self.axis];

        if (bound <= 1e-2) {
            groove_loss(dist, 0., 2, 0.1, 10.0, 2)
        } else {
            swamp_groove_loss(dist, 0.0, -bound, bound, bound*2.0, 1.0, 0.01, 100.0, 20) 
        }
    }
    fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        let x_val = ( ee_poses[self.arm_idx].0 - v.goal_positions[self.arm_idx] ).norm();
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}

pub struct MatchEERotaDoF {
    pub arm_idx: usize,
    pub axis: usize
}
impl MatchEERotaDoF {
    pub fn new(arm_idx: usize, axis: usize) -> Self {Self{arm_idx, axis}}
}
impl ObjectiveTrait for MatchEERotaDoF {
    fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        let last_elem = frames[self.arm_idx].1.len() - 1;
        let ee_quat = frames[self.arm_idx].1[last_elem];
        let goal_quat = v.goal_quats[self.arm_idx];
        let rotation = goal_quat.inverse()*ee_quat;


        let euler = rotation.euler_angles(); 
        let scaled_axis = rotation.scaled_axis();

        // println!("axisAngle: {:?} {:?}", euler, axisAngle);
        
        let mut angle: f64 = 0.0;
        angle += scaled_axis[self.axis].abs();

        let bound =  v.tolerances[self.arm_idx][self.axis + 3];

        if (bound <= 1e-2) {
            groove_loss(angle, 0., 2, 0.1, 10.0, 2)
        } else {
            if bound >= 3.14159260 {
                swamp_loss(angle, -bound, bound, 100.0, 0.1, 20) 
            } else {
                swamp_groove_loss(angle, 0.0, -bound, bound, bound*2.0, 1.0, 0.01, 100.0, 20) 
                // swamp_groove_loss(angle, 0.0, -bound, bound, 10.0, 1.0, 0.01, 100.0, 20)
            }
        }
    }

    fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        let x_val = ( ee_poses[self.arm_idx].0 - v.goal_positions[self.arm_idx] ).norm();
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}

pub struct SelfCollision {
    pub arm_idx_0: usize,
    pub arm_idx_1: usize,
    pub first_link: usize,
    pub second_link: usize,
    pub is_ee_link_0: bool,
    pub is_ee_link_1: bool,
}
impl SelfCollision {
    pub fn new(arm_idx_0: usize, arm_idx_1: usize, first_link: usize, second_link: usize, is_ee_link_0: bool, is_ee_link_1: bool) 
        -> Self {Self{arm_idx_0, arm_idx_1, first_link, second_link, is_ee_link_0, is_ee_link_1}}
}
impl ObjectiveTrait for SelfCollision {
    fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        for i in 0..x.len() {
            if x[i].is_nan() {
                return 10.0
            }
        }
       
        let mut x_val: f64 = 0.0;
        let link_radius = 0.05;

        let start_pt_1 = Point3::from(frames[self.arm_idx_0].0[self.first_link]);
        let end_pt_1 = Point3::from(frames[self.arm_idx_0].0[self.first_link+1]);
        let segment_1 = shape::Segment::new(start_pt_1, end_pt_1);

        let mut start_pt_2 = Point3::from(frames[self.arm_idx_1].0[self.second_link]);
        let mut end_pt_2 = Point3::from(frames[self.arm_idx_1].0[self.second_link+1]);

        let segment_2 = shape::Segment::new(start_pt_2, end_pt_2);

        let segment_pos = nalgebra::one(); 
        // pos 1 and pos 2 are transformations, since we use global positions, just pass identity
        // println!("start_pt_1:{} end_pt_1:{}  start_pt_2:{} end_pt_2:{} x: {:?}", start_pt_1, end_pt_1, start_pt_2, end_pt_2, x);
        
        
        // if self.is_ee_link_0 && self.is_ee_link_1{
        //     let mut shape_0 = shape::Capsule::new(start_pt_1, start_pt_2, 0.1);
        // } 
        // else {
        //     let mut shape_0 = segment_1;
        // }

        let capsule_1 = shape::Capsule::new(start_pt_1 + (end_pt_1 - start_pt_1) * 0.2, end_pt_1 + (end_pt_1 - start_pt_1) * 0.2, 0.01);
        let capsule_2 = shape::Capsule::new(start_pt_2 + (end_pt_1 - start_pt_1) * 0.2, end_pt_2 + (end_pt_2 - start_pt_2) * 0.2, 0.01);


        let mut dis: f64 = 0.0;
        
        if !self.is_ee_link_0 && !self.is_ee_link_1 {
            dis = query::distance(&segment_pos, &segment_1, &segment_pos, &segment_2).unwrap() - 0.05;
        } 

        if self.is_ee_link_0 && !self.is_ee_link_1 {
            dis = query::distance(&segment_pos, &capsule_1, &segment_pos, &segment_2).unwrap() - 0.025;
        }
        if !self.is_ee_link_0 && self.is_ee_link_1 {
            dis = query::distance(&segment_pos, &segment_1, &segment_pos, &capsule_2).unwrap() - 0.025;
        }
        if self.is_ee_link_0 && self.is_ee_link_1 {
            dis = query::distance(&segment_pos, &capsule_1, &segment_pos, &capsule_2).unwrap() - 0.0;
        }
        // if self.is_ee_link_0 {
        //     println!("{:?} {:?}",self.arm_idx_0, self.arm_idx_1);
        //     println!("{:?} -- {:?}",start_pt_1, end_pt_1);
        // }
        if dis < 0.0 {
            dis = 0.0
        }
        
        // println!("dis:{:?}",dis);
        swamp_loss(dis, 0.02, 1.5, 60.0, 0.0001, 30)
    }

    fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        let x_val = 1.0; // placeholder
        groove_loss(x_val, 0., 2, 2.1, 0.0002, 4)
    }
}


pub struct EnvCollision {
    pub arm_idx: usize,
    pub collision_start_idx: usize
}
impl EnvCollision {
    pub fn new(arm_idx: usize, collision_start_idx: usize) -> Self {Self{arm_idx, collision_start_idx}}
}
impl ObjectiveTrait for EnvCollision {
    fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        // let start = PreciseTime::now();\
        
        for i in 0..x.len() {
            if x[i].is_nan() {
                return 10.0
            }
        }
        
        let mut x_val: f64 = 0.0;
        let link_radius = v.env_collision.link_radius;
        let link_radius_finger = 0.01;
        let penalty_cutoff: f64 = link_radius * 2.0;
        let a = penalty_cutoff.powi(2);
        // println!("active obs: {:?}",v.env_collision.active_obstacles);
        for (option, score) in &v.env_collision.active_obstacles[self.arm_idx] {
            if let Some(handle) = option {
                let mut sum: f64 = 0.0;
                let obstacle = v.env_collision.world.objects.get(*handle).unwrap();
                let last_elem = frames[self.arm_idx].0.len() - 1;
                for i in self.collision_start_idx..last_elem {
                    let start_pt = Point3::from(frames[self.arm_idx].0[i]);
                    let start_quat = frames[self.arm_idx].1[i];
                     // hard coded for ur5
                    // if i == last_elem - 1 {
                    //     start_pt = Point3::from(frames[self.arm_idx].0[i] + 0.2 * (frames[self.arm_idx].0[i] - frames[self.arm_idx].0[i + 1]));
                    // }
                    let end_pt = Point3::from(frames[self.arm_idx].0[i + 1]);
                    
                    //
                    let end_quat = frames[self.arm_idx].1[i+1];
                    

                    
                    let segment = shape_nc::Segment::new(start_pt, end_pt);
                    let segment_pos = nalgebra::one();
                    let dis = 
                        // hard coded for movo
                        if i == last_elem - 1 {
                            // is last link (e.g. fingers)

                            // let ee_direction = end_quat;
                            // let ee_dir_vec = ee_direction * Point3::from(nalgebra::Vector3::new(1.0, 0.0, 0.0));
                            // let capsule_translation = Translation3::from(end_pt - ee_dir_vec * 0.1);

                            // let initial_direction = Vector3::new(0.0, 1.0, 0.0); // capsule primary axis is y axis
                            // let capsule_rotation = match nalgebra::Rotation3::rotation_between(&initial_direction, &(ee_direction * nalgebra::Vector3::new(1.0, 0.0, 0.0))) {
                            //     Some(r) => r,
                            //     None => nalgebra::Rotation3::from_euler_angles(0.0, 0.0, 0.0)
                            // };

                            // let capsule_pos = nalgebra::Isometry3::from_parts(capsule_translation, UnitQuaternion::from_rotation_matrix(&capsule_rotation));
                            // let capsule = shape_nc::Capsule::new(0.10, 0.05);
                            // query_nc::distance(obstacle.position(), obstacle.shape().deref(), &capsule_pos, &capsule)
                            query_nc::distance(obstacle.position(), obstacle.shape().deref(), &segment_pos, &segment) - link_radius_finger
                        } else {
                            // regular links
                            query_nc::distance(obstacle.position(), obstacle.shape().deref(), &segment_pos, &segment) - link_radius
                        };

                        // if i == last_elem - 1 && self.arm_idx == 0 {
                        //     println!("dis_right_fingers: {:?}",dis);
                        // }

                    // println!("Obstacle: {}, Link: {}, Distance: {:?}", obstacle.data().name, i, dis);
                    sum += a / (dis + link_radius).powi(2);
                }
                // println!("OBJECTIVE -> {:?}, Sum: {:?}", obstacle.data().name, sum);
                x_val += sum;
            }
        }
        
        // let end = PreciseTime::now();
        // println!("Obstacles calculating takes {}", start.to(end));
        // println!("x_collision = {:?}",x_val);
        groove_loss(x_val, 0., 2, 3.5, 0.00005, 4)

    }

    fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        let x_val = 1.0; // placeholder
        groove_loss(x_val, 0., 2, 2.1, 0.0002, 4)
    }
}

pub struct MaximizeManipulability;
impl ObjectiveTrait for MaximizeManipulability {
    fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        let mut x_val = v.robot.get_manipulability_immutable(&x);
        if x_val.is_nan() {
            // eprintln!("Manipulability is nan!");
            x_val = 10.0;
        }
        groove_loss(x_val, 1.0, 2, 0.5, 0.1, 2)
    }

    fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        0.0
    }
}
pub struct EachJointLimits{
    pub joint_idx: usize
}
impl EachJointLimits {
    pub fn new(joint_idx: usize) -> Self {Self{joint_idx}}
}
impl ObjectiveTrait for EachJointLimits {
    fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
    
        let l = v.robot.lower_joint_limits[self.joint_idx];
        let u = v.robot.upper_joint_limits[self.joint_idx];
        swamp_loss(x[self.joint_idx], l, u, 10.0, 10.0, 20)
    }

    fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        0.0
    }
}

pub struct MinimizeVelocity;
impl ObjectiveTrait for MinimizeVelocity {
    fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
           x_val += (x[i] - v.xopt[i]).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }

    fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
           x_val += (x[i] - v.xopt[i]).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

pub struct MinimizeAcceleration;
impl ObjectiveTrait for MinimizeAcceleration {
    fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            x_val += (v1 - v2).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }

    fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            x_val += (v1 - v2).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

pub struct MinimizeJerk;
impl ObjectiveTrait for MinimizeJerk {
    fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            let v3 = v.prev_state[i] - v.prev_state2[i];
            let a1 = v1 - v2;
            let a2 = v2 - v3;
            x_val += (a1 - a2).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1 , 10.0, 2)
    }

    fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            let v3 = v.prev_state[i] - v.prev_state2[i];
            let a1 = v1 - v2;
            let a2 = v2 - v3;
            x_val += (a1 - a2).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}


pub struct MatchEEPosGoals {
    pub arm_idx: usize,
    pub chain_idx_offset: usize
}
impl MatchEEPosGoals {
    pub fn new(arm_idx: usize, chain_idx_offset: usize) -> Self {Self{arm_idx, chain_idx_offset}}
}
impl ObjectiveTrait for MatchEEPosGoals {
    fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        let last_elem = frames[self.arm_idx].0.len() - 1 - self.chain_idx_offset;
        let x_val = ( frames[self.arm_idx].0[last_elem] - v.goal_positions[self.arm_idx] ).norm();
        // println!("{:?},{:?}",frames[self.arm_idx].0[last_elem], v.goal_positions[self.arm_idx]);
        // println!("dist x: {x_val}");
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }

    fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        let x_val = ( ee_poses[self.arm_idx].0 - v.goal_positions[self.arm_idx] ).norm();
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}


pub struct MatchEEQuatGoals {
    pub arm_idx: usize,
    pub chain_idx_offset: usize
}
impl MatchEEQuatGoals {
    pub fn new(arm_idx: usize, chain_idx_offset: usize) -> Self {Self{arm_idx, chain_idx_offset}}
}
impl ObjectiveTrait for MatchEEQuatGoals {
    fn call(&self, x: &[f64], v: &vars::RelaxedIKVars, frames: &Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)>) -> f64 {
        let last_elem = frames[self.arm_idx].1.len() - 1 - self.chain_idx_offset;
        let tmp = Quaternion::new(-frames[self.arm_idx].1[last_elem].w, -frames[self.arm_idx].1[last_elem].i, -frames[self.arm_idx].1[last_elem].j, -frames[self.arm_idx].1[last_elem].k);
        let ee_quat2 = UnitQuaternion::from_quaternion(tmp);

        let disp = angle_between_quaternion(v.goal_quats[self.arm_idx], frames[self.arm_idx].1[last_elem]);
        let disp2 = angle_between_quaternion(v.goal_quats[self.arm_idx], ee_quat2);
        let x_val = disp.min(disp2);

        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }

    fn call_lite(&self, x: &[f64], v: &vars::RelaxedIKVars, ee_poses: &Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)>) -> f64 {
        let tmp = Quaternion::new(-ee_poses[self.arm_idx].1.w, -ee_poses[self.arm_idx].1.i, -ee_poses[self.arm_idx].1.j, -ee_poses[self.arm_idx].1.k);
        let ee_quat2 = UnitQuaternion::from_quaternion(tmp);

        let disp = angle_between_quaternion(v.goal_quats[self.arm_idx], ee_poses[self.arm_idx].1);
        let disp2 = angle_between_quaternion(v.goal_quats[self.arm_idx], ee_quat2);
        let x_val = disp.min(disp2);
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
}
