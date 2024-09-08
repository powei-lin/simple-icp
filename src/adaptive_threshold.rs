use nalgebra as na;

pub struct AdaptiveThreshold {
    // configurable parameters
    min_motion_threshold: f64,
    max_range: f64,

    // Local cache for ccomputation
    model_sse: f64,
    num_samples: f64,
}
impl AdaptiveThreshold {
    pub fn new(init_thres: f64, min_motion_threshold: f64, max_range: f64) -> AdaptiveThreshold {
        AdaptiveThreshold {
            min_motion_threshold,
            max_range,
            model_sse: init_thres * init_thres,
            num_samples: 1.0,
        }
    }
    pub fn compute_threshold(&self) -> f64 {
        (self.model_sse / self.num_samples).sqrt()
    }
    pub fn update_model_deviation(&mut self, current_deviation: &na::Isometry3<f64>) {
        let theta = current_deviation.rotation.angle();
        let delta_rot = 2.0 * self.max_range * (theta / 2.0).sin();
        let delta_trans = current_deviation.translation.vector.norm();
        let model_error = delta_rot + delta_trans;
        if model_error > self.min_motion_threshold {
            self.model_sse += model_error * model_error;
            self.num_samples += 1.0;
        }
    }
}
