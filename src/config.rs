use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Config {
    // map params
    pub voxel_size: f32,
    pub max_range: f32,
    pub min_range: f32,
    pub max_points_per_voxel: u16,

    // th parms
    pub min_motion_th: f64,
    pub initial_threshold: f64,

    // registration params
    pub max_num_iterations: u16,
    pub convergence_criterion: f64,
    pub max_num_threads: u8,

    // Motion compensation
    pub deskew: bool,
}
impl Config {
    pub fn default_values() -> Config {
        Config {
            voxel_size: 1.0,
            max_range: 100.0,
            min_range: 1.0,
            max_points_per_voxel: 20,

            // th parms
            min_motion_th: 0.1,
            initial_threshold: 2.0,

            // registration params
            max_num_iterations: 500,
            convergence_criterion: 0.0001,
            max_num_threads: 0,

            // Motion compensation
            deskew: false,
        }
    }
}
