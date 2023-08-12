use nalgebra::{UnitQuaternion, Vector3, Vector6, Quaternion, Point3};
use std::f64::consts::PI;
use std::collections::{BTreeSet, HashSet};
use crate::spacetime::robot::Robot;
use crate::utils_rust::file_utils::{*};
use crate::utils_rust::yaml_utils::{*};
use crate::groove::env_collision::{*};
use time::PreciseTime;
use std::ops::Deref;
use yaml_rust::{YamlLoader, Yaml};
use std::fs;
use std::fs::File;
use std::io::prelude::*;
use ncollide3d::pipeline::{*};
use ncollide3d::query::{*};
use ncollide3d::shape::{*};

use wasm_bindgen::prelude::*;
use serde::{Serialize, Deserialize};
use serde_xml_rs;

#[derive(Serialize, Deserialize)]
pub struct VarsConstructorData {
    // pub urdf: String,
    pub link_radius:f64,
    pub base_links: Vec<String>,
    pub ee_links: Vec<String>,
    pub starting_config: Vec<f64>,
    // below are here just for compatibility, may not be implemented properly
    pub chains_def: Vec<Vec<i64>>,
    pub is_active_chain: Vec<bool>,
    pub arm_group: Vec<usize>,
    pub num_links_ee_to_tip: i64,
    pub collision_starting_indices: Vec<usize>,
    ////
}

pub struct RelaxedIKVars {
    pub robot: Robot,
    pub srdf_robot: SRDFRobot,
    pub init_state: Vec<f64>,
    pub xopt: Vec<f64>,
    pub prev_state: Vec<f64>,
    pub prev_state2: Vec<f64>,
    pub prev_state3: Vec<f64>,
    pub goal_positions: Vec<Vector3<f64>>,
    pub goal_quats: Vec<UnitQuaternion<f64>>,
    pub tolerances: Vec<Vector6<f64>>,
    pub init_ee_positions: Vec<Vector3<f64>>,
    pub init_ee_quats: Vec<UnitQuaternion<f64>>,
    pub env_collision: RelaxedIKEnvCollision,
    pub chains_def: Vec<Vec<i64>>,
    pub is_active_chain: Vec<bool>,
    pub arm_group: Vec<usize>,
    pub collision_starting_indices: Vec<usize>,
    pub num_links_ee_to_tip: i64,
}

#[derive(Debug, Deserialize)]
pub struct SRDFRobot {
    #[serde(rename = "disable_collisions")]
    disable_collisions: Vec<DisableCollisions>,
}

#[derive(Debug, Deserialize)]
pub struct DisableCollisions {
    #[serde(rename = "link1")]
    link1: String,
    #[serde(rename = "link2")]
    link2: String,
    reason: String,
}

impl RelaxedIKVars {
    pub fn from_local_settings(path_to_setting: &str) -> Self {
        let path_to_src = get_path_to_src();
        let mut file = File::open(path_to_setting).unwrap();
        let mut contents = String::new();
        let res = file.read_to_string(&mut contents).unwrap();
        let docs = YamlLoader::load_from_str(contents.as_str()).unwrap();
        let settings = &docs[0];

        let path_to_urdf = path_to_src.clone() + "configs/urdfs/" + settings["urdf"].as_str().unwrap();
        let path_to_srdf = path_to_src.clone() + "configs/urdfs/" + settings["srdf"].as_str().unwrap();

        println!("RelaxedIK is using below URDF file: {}", path_to_urdf);
        println!("RelaxedIK is using below SRDF file: {}", path_to_srdf);

        let srdf_file =  fs::read_to_string(path_to_srdf).unwrap();
        let srdf_robot: SRDFRobot = serde_xml_rs::from_str(&srdf_file).unwrap();    
        for collision in &srdf_robot.disable_collisions {
            println!("{:?}", *collision);
        }

        let chain = k::Chain::<f64>::from_urdf_file(path_to_urdf.clone()).unwrap();

        let base_links_arr = settings["base_links"].as_vec().unwrap();
        let ee_links_arr = settings["ee_links"].as_vec().unwrap();
        let chains_def_arr = settings["chains_def"].as_vec().unwrap();
        let is_active_chain_arr = settings["is_active_chain"].as_vec().unwrap();
        let num_links_ee_to_tip = settings["num_links_ee_to_tip"].as_i64().unwrap();
        let enforce_ja_arr = settings["enforce_joint_angles"].as_vec().unwrap();
        let arm_group_arr = settings["arm_group"].as_vec().unwrap();

        let num_chains = base_links_arr.len();
        
        let mut base_links = Vec::new();
        let mut ee_links = Vec::new();
        let mut tolerances: Vec<Vector6<f64>> = Vec::new();
        
        for i in 0..num_chains {
            base_links.push(base_links_arr[i].as_str().unwrap().to_string());
            ee_links.push(ee_links_arr[i].as_str().unwrap().to_string());
            tolerances.push(Vector6::new(0., 0., 0., 0., 0., 0.));
        }

        let chains_def: Vec<Vec<i64>> = chains_def_arr.iter().map(|row| {
            row.as_vec().unwrap().iter().map(|element| {
                element.as_i64().unwrap()
            }).collect()
        }).collect();

        let is_active_chain: Vec<bool> = is_active_chain_arr.iter().map(|item| {item.as_bool().unwrap()}).collect();
        let arm_group: Vec<usize> = arm_group_arr.iter().map(|item| {item.as_i64().unwrap() as usize}).collect();
        let enforce_ja: Vec<f64> = enforce_ja_arr.iter().map(|item| {item.as_f64().unwrap()}).collect();
        // calculate the index of link where collision needs to start of each chain
        let mut collision_starting_indices: Vec<usize> = Vec::new();
        let mut link_considered_in_collision: BTreeSet<i64> = BTreeSet::new();
        for i in 0..num_chains {
            let mut c_idx: i64 = -1;
            for (j, k) in chains_def[i].iter().enumerate() {
                if !link_considered_in_collision.contains(k) {
                    link_considered_in_collision.insert(*k);
                    if c_idx == -1 {
                        c_idx = j as i64; 
                    };
                }
            }
            if c_idx != -1 {
                collision_starting_indices.push(c_idx as usize);  
            } 
        }


        let urdf = &std::fs::read_to_string(path_to_urdf).unwrap();
        let mut robot = Robot::from_urdf(urdf, &base_links, &ee_links, &chains_def);

        // enforce joint angles by modifying joint limits
        let eps: f64 = 0.1;
        for i in 0..robot.num_dofs {
            if enforce_ja[i] > -PI && enforce_ja[i] < PI {
                robot.lower_joint_limits[i] = enforce_ja[i] - eps;
                robot.upper_joint_limits[i] = enforce_ja[i] + eps;
            } 
        }

        println!("num_dofs:{:?}",robot.num_dofs);
        let mut starting_config = Vec::new();
        if settings["starting_config"].is_badvalue() {
            println!("No starting config provided, using all zeros");
            for i in 0..robot.num_dofs {
                starting_config.push(0.0);
            }
        } else {
            let starting_config_arr = settings["starting_config"].as_vec().unwrap();
            for i in 0..starting_config_arr.len() {
                starting_config.push(starting_config_arr[i].as_f64().unwrap());
            }
        }

        let mut init_ee_positions: Vec<Vector3<f64>> = Vec::new();
        let mut init_ee_quats: Vec<UnitQuaternion<f64>> = Vec::new();
        // configurations need to be without duplicated joints
        let pose = robot.get_ee_pos_and_quat_immutable(&starting_config);
        assert!(pose.len() == num_chains);

        for i in 0..pose.len() {
            init_ee_positions.push(pose[i].0);
            init_ee_quats.push(pose[i].1);
        }

        let env_collision_file = EnvCollisionFileParser::from_yaml_path(path_to_setting.to_string());
        let frames = robot.get_frames_immutable(&starting_config.clone());
        let env_collision = RelaxedIKEnvCollision::init_collision_world(env_collision_file, &frames);

        RelaxedIKVars{robot, srdf_robot, init_state: starting_config.clone(), xopt: starting_config.clone(),
            prev_state: starting_config.clone(), prev_state2: starting_config.clone(), prev_state3: starting_config.clone(),
            goal_positions: init_ee_positions.clone(), goal_quats: init_ee_quats.clone(), tolerances, init_ee_positions, init_ee_quats, env_collision:env_collision,
            chains_def, is_active_chain, arm_group, collision_starting_indices, num_links_ee_to_tip}
    }
    
    // for webassembly
    pub fn from_jsvalue( configs: VarsConstructorData, urdf: &str, srdf: &str) -> Self  {

        let num_chains = configs.base_links.len();

        let mut tolerances: Vec<Vector6<f64>> = Vec::new();
        for i in 0..num_chains {
            tolerances.push(Vector6::new(0., 0., 0., 0., 0., 0.));
        }

        let robot = Robot::from_urdf(urdf, &configs.base_links, &configs.ee_links, &configs.chains_def);
        let srdf_robot: SRDFRobot = serde_xml_rs::from_str(srdf).unwrap();    

        let mut init_ee_positions: Vec<Vector3<f64>> = Vec::new();
        let mut init_ee_quats: Vec<UnitQuaternion<f64>> = Vec::new();
        let pose = robot.get_ee_pos_and_quat_immutable(&configs.starting_config);
        assert!(pose.len() == num_chains);

        for i in 0..pose.len() {
            init_ee_positions.push(pose[i].0);
            init_ee_quats.push(pose[i].1);
        }
        let path_to_src = get_path_to_src();
        let env_collision_file = EnvCollisionFileParser::from_yaml_path(path_to_src);
        let frames = robot.get_frames_immutable(&configs.starting_config.clone());
        let env_collision = RelaxedIKEnvCollision::init_collision_world(env_collision_file, &frames);



        RelaxedIKVars{robot, srdf_robot, init_state: configs.starting_config.clone(), xopt: configs.starting_config.clone(),
            prev_state: configs.starting_config.clone(), prev_state2: configs.starting_config.clone(), prev_state3: configs.starting_config.clone(),
            goal_positions: init_ee_positions.clone(), goal_quats: init_ee_quats.clone(), tolerances, init_ee_positions, init_ee_quats, env_collision:env_collision,
            chains_def:configs.chains_def.clone(), is_active_chain:configs.is_active_chain.clone(), arm_group: configs.arm_group.clone(), collision_starting_indices:configs.collision_starting_indices.clone(), num_links_ee_to_tip: configs.num_links_ee_to_tip.clone()}

    }

    pub fn update(&mut self, xopt: Vec<f64>) {
        self.prev_state3 = self.prev_state2.clone();
        self.prev_state2 = self.prev_state.clone();
        self.prev_state = self.xopt.clone();
        self.xopt = xopt.clone();
    }

    pub fn reset(&mut self, init_state: Vec<f64>) {
        self.prev_state3 = init_state.clone();
        self.prev_state2 = init_state.clone();
        self.prev_state = init_state.clone();
        self.xopt = init_state.clone();
        self.init_state = init_state.clone();

        let mut init_ee_positions: Vec<Vector3<f64>> = Vec::new();
        let mut init_ee_quats: Vec<UnitQuaternion<f64>> = Vec::new();
        let pose = self.robot.get_ee_pos_and_quat_immutable(&init_state);

        for i in 0..pose.len() {
            init_ee_positions.push(pose[i].0);
            init_ee_quats.push(pose[i].1);
        }

        self.init_ee_positions = init_ee_positions.clone();
        self.init_ee_quats = init_ee_quats.clone();
    }



    pub fn update_collision_world(&mut self) -> bool {
        let frames = self.robot.get_frames_immutable(&self.xopt);
        self.env_collision.update_links(&frames);
        for event in self.env_collision.world.proximity_events() {
            let c1 = self.env_collision.world.objects.get(event.collider1).unwrap();
            let c2 = self.env_collision.world.objects.get(event.collider2).unwrap();
            if event.new_status == ncollide3d::query::Proximity::Intersecting {
                println!("===== {:?} Intersecting of {:?} =====", c1.data().name, c2.data().name);
            } else if event.new_status == ncollide3d::query::Proximity::WithinMargin {
                println!("===== {:?} WithinMargin of {:?} =====", c1.data().name, c2.data().name);
                if c1.data().link_data.is_link {
                    let arm_idx = c1.data().link_data.arm_idx as usize;
                    if self.env_collision.active_pairs[arm_idx].contains_key(&event.collider2) {
                        let links = self.env_collision.active_pairs[arm_idx].get_mut(&event.collider2).unwrap();
                        if !links.contains(&event.collider1) {
                            links.push(event.collider1);
                        }
                    } else {
                        let links: Vec<ncollide3d::pipeline::object::CollisionObjectSlabHandle> = vec![event.collider1];
                        self.env_collision.active_pairs[arm_idx].insert(event.collider2, links);
                    }
                } else if c2.data().link_data.is_link {
                    let arm_idx = c2.data().link_data.arm_idx as usize;
                    if self.env_collision.active_pairs[arm_idx].contains_key(&event.collider1) {
                        let links = self.env_collision.active_pairs[arm_idx].get_mut(&event.collider1).unwrap();
                        if !links.contains(&event.collider2) {
                            links.push(event.collider2);
                        }
                    } else {
                        let links: Vec<ncollide3d::pipeline::object::CollisionObjectSlabHandle> = vec![event.collider2];
                        self.env_collision.active_pairs[arm_idx].insert(event.collider1, links);
                    }
                }
            } else {
                println!("===== {:?} Disjoint of {:?} =====", c1.data().name, c2.data().name);
                if c1.data().link_data.is_link {
                    let arm_idx = c1.data().link_data.arm_idx as usize;
                    if self.env_collision.active_pairs[arm_idx].contains_key(&event.collider2) {
                        let links = self.env_collision.active_pairs[arm_idx].get_mut(&event.collider2).unwrap();
                        if links.contains(&event.collider1) {
                            let index = links.iter().position(|x| *x == event.collider1).unwrap();
                            links.remove(index);
                        }
                        if links.len() == 0 {
                            self.env_collision.active_pairs[arm_idx].remove(&event.collider2);
                        }
                    }
                } else if c2.data().link_data.is_link {
                    let arm_idx = c2.data().link_data.arm_idx as usize;
                    if self.env_collision.active_pairs[arm_idx].contains_key(&event.collider1) {
                        let links = self.env_collision.active_pairs[arm_idx].get_mut(&event.collider1).unwrap();
                        if links.contains(&event.collider2) {
                            let index = links.iter().position(|x| *x == event.collider2).unwrap();
                            links.remove(index);
                        }
                        if links.len() == 0 {
                            self.env_collision.active_pairs[arm_idx].remove(&event.collider1);
                        }
                    }
                } 
            }
            // self.print_active_pairs();
        }

        self.env_collision.world.update();

        let link_radius = self.env_collision.link_radius;
        let link_radius_finger = 0.01;
        let penalty_cutoff: f64 = link_radius * 2.0;
        let a = penalty_cutoff.powi(2);
        let filter_cutoff = 3;
        for arm_idx in 0..frames.len() {
            // let mut sum_max: f64 = 0.0;
            let mut active_candidates: Vec<(Option<CollisionObjectSlabHandle>, f64)> = Vec::new();
            for key in self.env_collision.active_pairs[arm_idx].keys() {
                let obstacle = self.env_collision.world.objects.get(*key).unwrap();
                // println!("Obstacle: {:?}", obstacle.data());
                let mut sum: f64 = 0.0;
                let last_elem = frames[arm_idx].0.len() - 1;
                for j in 0..last_elem {
                    let start_pt = Point3::from(frames[arm_idx].0[j]);
                    let end_pt = Point3::from(frames[arm_idx].0[j + 1]);
                    let segment = Segment::new(start_pt, end_pt);
                    let segment_pos = nalgebra::one();
                    let dis = if j == last_elem - 1 {
                         // hard coded for movo
                        // let ee_direction = frames[arm_idx].1[j+1];
                        // let ee_dir_vec = ee_direction * Point3::from(nalgebra::Vector3::new(1.0, 0.0, 0.0));
                        // let capsule_translation = nalgebra::Translation3::from(end_pt - ee_dir_vec * 0.1);
                        // let initial_direction = Vector3::new(0.0, 1.0, 0.0); // capsule primary axis is y axis
                        // let capsule_rotation = match nalgebra::Rotation3::rotation_between(&initial_direction, &(ee_direction * nalgebra::Vector3::new(1.0, 0.0, 0.0))) {
                        //     Some(r) => r,
                        //     None => nalgebra::Rotation3::from_euler_angles(0.0, 0.0, 0.0)
                        // };
                        // let capsule_pos = nalgebra::Isometry3::from_parts(capsule_translation, UnitQuaternion::from_rotation_matrix(&capsule_rotation));
                        // let capsule = Capsule::new(0.10, 0.05);

                        // distance(obstacle.position(), obstacle.shape().deref(), &capsule_pos, &capsule)
                        distance(obstacle.position(), obstacle.shape().deref(), &segment_pos, &segment) - link_radius_finger
                    }
                    else { 
                        distance(obstacle.position(), obstacle.shape().deref(), &segment_pos, &segment) - link_radius 
                    }; 

                    // println!("VARS -> {:?}, Link{}, Distance: {:?}", obstacle.data(), j, dis);
                    if dis > 0.0 {
                        sum += a / (dis + link_radius).powi(2);
                    } else if /*self.objective_mode != "noECA"*/ true {
                        return true;
                    } else {
                        break;
                    }
                }
                active_candidates.push((Some(*key), sum));
            }

            // println!("Number of active obstacles candidates: {}", active_candidates.len());
            {
                if active_candidates.len() > filter_cutoff {
                    active_candidates.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
                    let active_obstacles = active_candidates[0..filter_cutoff].iter().cloned().collect();
                    self.env_collision.active_obstacles[arm_idx] = active_obstacles;
                } else {
                    self.env_collision.active_obstacles[arm_idx] = active_candidates;
                }
            }
        }
        
        return false;
    }

    pub fn print_active_pairs(&self) {
        let frames = self.robot.get_frames_immutable(&self.xopt);
        for i in 0..frames.len() {
            for (key, values) in self.env_collision.active_pairs[i].iter() {
                let collider = self.env_collision.world.objects.get(*key).unwrap();
                for v in values {
                    let link = self.env_collision.world.objects.get(*v).unwrap();
                    println!("Arm {}, Active pair {:?} and {:?}", i, collider.data().name, link.data().name);
                }
            }
        }
    }

    pub fn get_collision_disabled_set(&self) -> BTreeSet<(String, String)> {
        let mut is_disabled: BTreeSet<(String, String)> = BTreeSet::new();
        for collision in &self.srdf_robot.disable_collisions {
            let l1: String = collision.link1.clone();
            let l2: String = collision.link2.clone();
            if l1 < l2 {
                is_disabled.insert((l1, l2));
            }
            else {
                is_disabled.insert((l2, l1));
            }
        }
        is_disabled
    }
}

