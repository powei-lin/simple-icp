use std::collections::HashMap;
use std::sync::Arc;

use tiny_solver::{self, na, Optimizer};
use tiny_solver::manifold::se3::{SE3Manifold, SE3};

use super::factors::{OdometryFactor, PriorFactor};

#[derive(Clone, Debug)]
pub struct Keyframe {
    pub id: usize,
    pub pose: na::Isometry3<f64>,
    pub fixed: bool,
}

struct Constraint {
    from_id: usize,
    to_id: usize,
    relative_pose: na::Isometry3<f64>,
}

pub struct PoseGraphOptimizer {
    keyframes: HashMap<usize, Keyframe>,
    constraints: Vec<Constraint>,
}

impl PoseGraphOptimizer {
    pub fn new() -> Self {
        Self {
            keyframes: HashMap::new(),
            constraints: Vec::new(),
        }
    }

    pub fn add_keyframe(&mut self, id: usize, pose: na::Isometry3<f64>, fixed: bool) {
        self.keyframes.insert(id, Keyframe { id, pose, fixed });
    }

    pub fn add_odometry_constraint(&mut self, from_id: usize, to_id: usize, relative_pose: na::Isometry3<f64>) {
        self.constraints.push(Constraint {
            from_id,
            to_id,
            relative_pose,
        });
    }

    pub fn optimize(&mut self) {
        if self.keyframes.is_empty() {
            return;
        }

        let mut problem = tiny_solver::Problem::new();
        let se3_manifold = Arc::new(SE3Manifold);

        // Add variables and set manifolds
        let mut initial_values = HashMap::new();
        for (id, kf) in &self.keyframes {
            let key = format!("pose_{}", id);
            
            // Convert Isometry3 to SE3 vector (7D: [qx, qy, qz, qw, tx, ty, tz])
            let rot = kf.pose.rotation;
            let trans = kf.pose.translation;
            let q = rot.quaternion();
            
            // tiny-solver SE3 expects [qx, qy, qz, qw, tx, ty, tz]
            let se3_vec = na::dvector![
                q.i, q.j, q.k, q.w,
                trans.x, trans.y, trans.z
            ];
            
            initial_values.insert(key.clone(), se3_vec);
            problem.set_variable_manifold(&key, se3_manifold.clone());
            
            // If fixed, add a strong prior factor
            if kf.fixed {
                 problem.add_residual_block(
                    6,
                    &[&key],
                    Box::new(PriorFactor::new(kf.pose)),
                    None,
                );
            }
        }

        // Add constraints
        for (_i, constraint) in self.constraints.iter().enumerate() {
            let key_from = format!("pose_{}", constraint.from_id);
            let key_to = format!("pose_{}", constraint.to_id);
            
            // Check if keys exist (might have been pruned if we implement windowing later)
            if initial_values.contains_key(&key_from) && initial_values.contains_key(&key_to) {
                problem.add_residual_block(
                    6,
                    &[&key_from, &key_to],
                    Box::new(OdometryFactor::new(constraint.relative_pose)),
                    None, // No loss function for now, can add Huber/Cauchy later
                );
            }
        }

        // Optimize
        let optimizer = tiny_solver::LevenbergMarquardtOptimizer::default();
        if let Some(result) = optimizer.optimize(&problem, &initial_values, None) {
            // Update keyframe poses
            for (key, value) in result {
                if let Some(id_str) = key.strip_prefix("pose_") {
                    if let Ok(id) = id_str.parse::<usize>() {
                        if let Some(kf) = self.keyframes.get_mut(&id) {
                            // Convert back from SE3 vector to Isometry3
                            // value is [qx, qy, qz, qw, tx, ty, tz]
                            let se3 = SE3::<f64>::from_vec(value.as_view());
                            let q_vec = se3.rot.to_vec();
                            // na::Quaternion::new(w, i, j, k)
                            let q = na::Quaternion::new(q_vec[3], q_vec[0], q_vec[1], q_vec[2]);
                            
                            let rotation = na::UnitQuaternion::from_quaternion(q);
                            let translation = na::Translation3::from(se3.xyz);
                            
                            kf.pose = na::Isometry3::from_parts(translation, rotation);
                        }
                    }
                }
            }
        } else {
            println!("Optimization failed!");
        }
    }

    pub fn get_optimized_pose(&self, id: usize) -> Option<na::Isometry3<f64>> {
        self.keyframes.get(&id).map(|kf| kf.pose)
    }
}
