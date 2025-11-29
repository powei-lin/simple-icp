use tiny_solver::{self, na};
use tiny_solver::manifold::se3::SE3;

pub struct OdometryFactor {
    pub relative_pose_measurement: na::Isometry3<f64>,
}

impl OdometryFactor {
    pub fn new(relative_pose: na::Isometry3<f64>) -> Self {
        Self {
            relative_pose_measurement: relative_pose,
        }
    }
}

impl<T: na::RealField> tiny_solver::factors::Factor<T> for OdometryFactor {
    fn residual_func(&self, params: &[na::DVector<T>]) -> na::DVector<T> {
        let pose_i_vec = &params[0];
        let pose_j_vec = &params[1];

        let pose_i = SE3::<T>::from_vec(pose_i_vec.as_view());
        let pose_j = SE3::<T>::from_vec(pose_j_vec.as_view());

        // Measurement: T_ij_meas
        // Estimate: T_ij_est = T_i^-1 * T_j
        // Residual = log(T_ij_meas^-1 * T_ij_est)

        // Convert measurement to SE3<T>
        let meas_rot = self.relative_pose_measurement.rotation;
        let meas_trans = self.relative_pose_measurement.translation;
        
        let meas_q = meas_rot.quaternion();
        let meas_q_vec = na::dvector![
            T::from_f64(meas_q.i).unwrap(),
            T::from_f64(meas_q.j).unwrap(),
            T::from_f64(meas_q.k).unwrap(),
            T::from_f64(meas_q.w).unwrap()
        ];
        let meas_t_vec = na::dvector![
            T::from_f64(meas_trans.x).unwrap(),
            T::from_f64(meas_trans.y).unwrap(),
            T::from_f64(meas_trans.z).unwrap()
        ];
        
        let pose_meas = SE3::<T>::from_qvec_tvec(meas_q_vec.as_view(), meas_t_vec.as_view());

        // T_ij_est = T_i.inverse() * T_j
        let pose_i_inv = pose_i.inverse();
        let pose_est = pose_i_inv.compose(&pose_j);

        // Error = T_meas.inverse() * T_est
        let pose_meas_inv = pose_meas.inverse();
        let error_se3 = pose_meas_inv.compose(&pose_est);

        error_se3.log()
    }
}

pub struct PriorFactor {
    pub prior_pose: na::Isometry3<f64>,
}

impl PriorFactor {
    pub fn new(prior_pose: na::Isometry3<f64>) -> Self {
        Self {
            prior_pose,
        }
    }
}

impl<T: na::RealField> tiny_solver::factors::Factor<T> for PriorFactor {
    fn residual_func(&self, params: &[na::DVector<T>]) -> na::DVector<T> {
        let pose_vec = &params[0];
        let pose = SE3::<T>::from_vec(pose_vec.as_view());

        // Convert prior to SE3<T>
        let prior_rot = self.prior_pose.rotation;
        let prior_trans = self.prior_pose.translation;
        
        let prior_q = prior_rot.quaternion();
        let prior_q_vec = na::dvector![
            T::from_f64(prior_q.i).unwrap(),
            T::from_f64(prior_q.j).unwrap(),
            T::from_f64(prior_q.k).unwrap(),
            T::from_f64(prior_q.w).unwrap()
        ];
        let prior_t_vec = na::dvector![
            T::from_f64(prior_trans.x).unwrap(),
            T::from_f64(prior_trans.y).unwrap(),
            T::from_f64(prior_trans.z).unwrap()
        ];
        
        let pose_prior = SE3::<T>::from_qvec_tvec(prior_q_vec.as_view(), prior_t_vec.as_view());

        // Error = T_prior.inverse() * T_current
        let pose_prior_inv = pose_prior.inverse();
        let error_se3 = pose_prior_inv.compose(&pose);

        error_se3.log()
    }
}
