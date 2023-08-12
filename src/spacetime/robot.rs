use crate::spacetime::arm;
use nalgebra;
use urdf_rs;
use std::collections::HashSet;
#[derive(Clone, Debug)]
pub struct Robot {
    pub arms: Vec<arm::Arm>,
    pub num_chains: usize,
    pub num_dofs: usize,
    pub chain_lengths: Vec<usize>,
    pub chains_def : Vec<Vec<i64>>,
    pub lower_joint_limits: Vec<f64>,
    pub upper_joint_limits: Vec<f64>,
}

impl Robot {
    pub fn from_urdf(urdf: &str, base_links: &[String], ee_links: &[String], chains_def: &Vec<Vec<i64>>) -> Self {
        
        // let chain = k::Chain::<f64>::from_urdf_file(urdf_fp).unwrap();
        let description : urdf_rs::Robot = urdf_rs::read_from_string(urdf).unwrap();
        let chain: k::Chain<f64> = k::Chain::from(description.clone());

        let mut arms: Vec<arm::Arm> = Vec::new();
        let num_chains = base_links.len();
        let mut chain_lengths = Vec::new();
        let mut num_dofs = 0;
        let mut articulated_joint_names:HashSet<String> = HashSet::new();

        let mut lower_joint_limits = Vec::new();
        let mut upper_joint_limits = Vec::new();
        println!("chains_def: {:?}",chains_def);

        for i in 0..num_chains {
            let base_link = chain.find_link(base_links[i].as_str()).unwrap();
            let ee_link = chain.find_link(ee_links[i].as_str()).unwrap();
            let serial_chain = k::SerialChain::from_end_to_root(&ee_link, &base_link);
            let base_link_name = base_link.link().as_ref().unwrap().name.clone();
            let ee_link_name  = ee_link.link().as_ref().unwrap().name.clone();

            let mut axis_types: Vec<String> = Vec::new();
            let mut joint_types: Vec<String> = Vec::new();
            let disp_offset = nalgebra::Vector3::new(0.0, 0.0, 0.0);
            let mut displacements = Vec::new();
            let mut rot_offsets = Vec::new();
            let mut num_links_inbetween: usize = 0;
            let mut link_names: Vec<String> = Vec::new();
            serial_chain.iter().for_each(|node| {
                let link = node.link();
                let link_name = link.as_ref().unwrap().clone().name;
                // println!("link: {:?}",link_name);
                if link_name != base_link_name && link_name != ee_link_name {
                 
                    num_links_inbetween += 1;
                }
                link_names.push(link_name);
            });
            

            serial_chain.iter().for_each(|node| {
                let joint = node.joint();
                
                match joint.joint_type {
                    k::JointType::Fixed => {
                        joint_types.push("fixed".to_string());
                    },
                    k::JointType::Rotational { axis } => {
                        if axis[0] == 1.0 {
                            axis_types.push("x".to_string());
                        } else if axis[1] == 1.0 {
                            axis_types.push("y".to_string());
                        } else if axis[2] == 1.0 {
                            axis_types.push("z".to_string());
                        } else if axis[0] == -1.0 {
                            axis_types.push("-x".to_string());
                        } else if axis[1] == -1.0 {
                            axis_types.push("-y".to_string());
                        } else if axis[2] == -1.0 {
                            axis_types.push("-z".to_string());
                        }
                        joint_types.push("revolute".to_string());
                        lower_joint_limits.push(joint.limits.unwrap().min);
                        upper_joint_limits.push(joint.limits.unwrap().max);
                        articulated_joint_names.insert(joint.name.clone());
                    }
                    k::JointType::Linear { axis } => {
                        if axis[0] == 1.0 {
                            axis_types.push("x".to_string());
                        } else if axis[1] == 1.0 {
                            axis_types.push("y".to_string());
                        } else if axis[2] == 1.0 {
                            axis_types.push("z".to_string());
                        } else if axis[0] == -1.0 {
                            axis_types.push("-x".to_string());
                        } else if axis[1] == -1.0 {
                            axis_types.push("-y".to_string());
                        } else if axis[2] == -1.0 {
                            axis_types.push("-z".to_string());
                        }
                        joint_types.push("prismatic".to_string());
                        lower_joint_limits.push(joint.limits.unwrap().min);
                        upper_joint_limits.push(joint.limits.unwrap().max);
                        articulated_joint_names.insert(joint.name.clone());
                    }
                }

                displacements.push(joint.origin().translation.vector);
                rot_offsets.push(joint.origin().rotation);
            });
            let arm: arm::Arm = arm::Arm::init(axis_types.clone(), displacements.clone(),
            rot_offsets.clone(), joint_types.clone());
            arms.push(arm);
            // chain_lengths.push(axis_types.len() as usize);
            chain_lengths.push(num_links_inbetween);
            // num_dofs += axis_types.len();
            println!("Link names of arm {}: {:?}", i, link_names)
        }
        num_dofs = articulated_joint_names.len();
        println!("axis types: {:?}", arms[0].axis_types);
        println!("num_dofs: {:?}",num_dofs);
        println!("articulated_joint_names: {:?}",articulated_joint_names);
        println!("lower_joint_limits: {:?}",lower_joint_limits);
        println!("upper_joint_limits: {:?}",upper_joint_limits);
        println!("chain_lengths: {:?}", chain_lengths);
        Robot{arms, num_chains, num_dofs, chain_lengths, chains_def: chains_def.clone(), lower_joint_limits, upper_joint_limits}

    }
    
    pub fn split_joint_angles(&self, x: &[f64], idx: usize) -> Vec<f64> {
        self.chains_def[idx].iter().map(|&i| x[i as usize].clone()).collect()
    }

    pub fn get_frames(&mut self, x: &[f64]) { // need translation to handle multiple arms having duplicate joints
        // let mut l = 0;
        // let mut r = 0;
        for i in 0..self.num_chains {
            // r += self.chain_lengths[i];
            let ja_vec = self.split_joint_angles(x, i);
            // self.arms[i].get_frames(&x[l..r]);
            self.arms[i].get_frames(&ja_vec[0..ja_vec.len()]);
            // l = r;
        }
    }

    pub fn get_frames_immutable(&self, x: &[f64]) -> Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)> {
        let mut out: Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)> = Vec::new();
        // let mut l = 0;
        // let mut r = 0;
        for i in 0..self.num_chains {
            // r += self.chain_lengths[i];
            let ja_vec = self.split_joint_angles(x, i);
            // out.push( self.arms[i].get_frames_immutable( &x[l..r] ) );
            out.push( self.arms[i].get_frames_immutable(&ja_vec[0..ja_vec.len()]));
            // l = r;
        }
        out
    }
    
    pub fn get_manipulability_immutable(&self, x: &[f64]) -> f64 {
        let mut out = 0.0;
        // let mut l = 0;
        // let mut r = 0;
        for i in 0..self.num_chains {
            // r += self.chain_lengths[i];
            let ja_vec = self.split_joint_angles(x, i);
            // out += self.arms[i].get_manipulability_immutable( &x[l..r] );
            out += self.arms[i].get_manipulability_immutable(&ja_vec[0..ja_vec.len()]);
            // l = r;
        }
        out
    }

    pub fn get_ee_pos_and_quat_immutable(&self, x: &[f64]) -> Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)> {
        let mut out: Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)> = Vec::new();
        // let mut l = 0;
        // let mut r = 0;
        for i in 0..self.num_chains {
            // r += self.chain_lengths[i];
            let ja_vec = self.split_joint_angles(x, i);
            // out.push( self.arms[i].get_ee_pos_and_quat_immutable( &x[l..r] ));
            out.push( self.arms[i].get_ee_pos_and_quat_immutable(&ja_vec[0..ja_vec.len()]));
            // l = r;
        }
        out
    }
}

