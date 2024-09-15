use crate::{
    adaptive_threshold::AdaptiveThreshold,
    config,
    lie_group::{Exp, Hat},
    point3d, voxel_hash_map, voxel_util,
};
use nalgebra as na;
use rayon::iter::{IntoParallelRefIterator, IntoParallelRefMutIterator, ParallelIterator};

pub struct IcpPipeline {
    config: config::Config,
    pub t_origin_current: na::Isometry3<f64>,
    t_prev_current: na::Isometry3<f64>,
    voxel_map: voxel_hash_map::VoxelHashMap,
    adaptive_threshold: AdaptiveThreshold,
}

impl IcpPipeline {
    pub fn default_values() -> IcpPipeline {
        let config = config::Config::default_values();
        IcpPipeline {
            config: config.clone(),
            t_origin_current: na::Isometry::identity(),
            t_prev_current: na::Isometry::identity(),
            voxel_map: voxel_hash_map::VoxelHashMap::default_values(),
            adaptive_threshold: AdaptiveThreshold::new(
                config.initial_threshold,
                config.min_motion_th,
                config.max_range as f64,
            ),
        }
    }
    pub fn get_last_batch_points(&self) -> &Vec<point3d::Point3d> {
        &self.voxel_map.last_batch_points
    }
    pub fn process_frame(&mut self, point_cloud: &[point3d::Point3d]) {
        // clip distance
        let cropped_frame = point3d::clip_point_cloud_by_distance(
            point_cloud,
            self.config.min_range,
            self.config.max_range,
        );

        // voxelize
        let (source, frame_downsample) = voxelize(&cropped_frame, self.config.voxel_size);

        // get adaptive threshold
        let sigma = self.adaptive_threshold.compute_threshold();
        // let sigma = 2.0;

        // initial guess
        let t_origin_next_init = self.t_origin_current * self.t_prev_current;

        // Run ICP
        let t_origin_next = align_points_to_map(
            &source,
            &self.voxel_map,
            &t_origin_next_init,
            3.0 * sigma,
            sigma / 3.0,
            self.config.max_num_iterations,
            self.config.convergence_criterion,
        );

        // Compute the difference between the prediction and the actual estimate
        let model_deviation = t_origin_next_init.inverse() * t_origin_next;

        // Update step: threshold, local map, delta, and the last pose
        self.adaptive_threshold
            .update_model_deviation(&model_deviation);
        self.voxel_map
            .update_with_pose(&frame_downsample, &t_origin_next);
        self.t_prev_current = self.t_origin_current.inverse() * t_origin_next;

        // bug in nalgebra
        self.t_origin_current = na::Isometry3::from_parts(
            t_origin_next.translation,
            na::UnitQuaternion::from_quaternion(t_origin_next.rotation.normalize()),
        );

        // Return the (deskew) input raw scan (frame) and the points used for registration (source)
        // (point_cloud, source)
    }
}

fn voxelize(
    point_cloud: &[point3d::Point3d],
    voxel_size: f32,
) -> (Vec<point3d::Point3d>, Vec<point3d::Point3d>) {
    let frame_downsample = voxel_util::voxel_downsample(point_cloud, voxel_size * 0.5);
    let source = voxel_util::voxel_downsample(&frame_downsample, voxel_size * 1.5);
    (source, frame_downsample)
}

fn transform_points(transform: &na::Isometry3<f64>, point_cloud: &mut [point3d::Point3d]) {
    point_cloud.par_iter_mut().for_each(|pt| {
        let transformed_pt =
            transform.transform_point(&na::Point3::new(pt.x as f64, pt.y as f64, pt.z as f64));
        pt.x = transformed_pt.x as f32;
        pt.y = transformed_pt.y as f32;
        pt.z = transformed_pt.z as f32;
    });
}

fn point_association(
    points: &[point3d::Point3d],
    voxel_map: &voxel_hash_map::VoxelHashMap,
    max_correspondance_distance: f64,
) -> Vec<(point3d::Point3d, point3d::Point3d)> {
    points
        .par_iter()
        .filter_map(|pt| {
            if let Some((closest_neighbor, distance)) = voxel_map.get_closest_neighbor(pt) {
                if distance < max_correspondance_distance {
                    Some((pt.to_owned(), closest_neighbor))
                } else {
                    None
                }
            } else {
                None
            }
        })
        .collect()
}

fn build_linear_system(
    correspondences: &[(point3d::Point3d, point3d::Point3d)],
    kernel_scale: f64,
) -> (na::Matrix6<f64>, na::Vector6<f64>) {
    let compute_jacobian_and_residual =
        |(source, target): &(na::Vector3<f64>, na::Vector3<f64>)| {
            let redisual = source - target;

            // [trans 3, rotation 3]
            let mut j_r = na::Matrix3x6::identity();
            j_r.fixed_columns_mut::<3>(3)
                .copy_from(&(-1.0 * source.hat()));
            (j_r, redisual)
        };

    correspondences
        .par_iter()
        .map(|corr_p3d| {
            let square = |x| x * x;
            let weight = |residual2| square(kernel_scale) / square(kernel_scale + residual2);
            let corr = (corr_p3d.0.to_na_vec_f64(), corr_p3d.1.to_na_vec_f64());
            let (j_r, residual) = compute_jacobian_and_residual(&corr);
            let w = weight(residual.norm_squared());

            let j_t = j_r.transpose();
            let j_tw = j_t * w;
            let j_tj = j_tw * j_r;
            let j_tr = j_tw * residual;
            (j_tj, j_tr)
        })
        // 2nd Lambda: Parallel reduction of the private Jacboians
        .reduce(Default::default, |(j_tj_a, j_tr_a), (j_tj_b, j_tr_b)| {
            (j_tj_a + j_tj_b, j_tr_a + j_tr_b)
        })
}

fn align_points_to_map(
    point_cloud: &[point3d::Point3d],
    voxel_map: &voxel_hash_map::VoxelHashMap,
    initial_guess: &na::Isometry3<f64>,
    max_distance: f64,
    kernel_scale: f64,
    max_num_iterations: u16,
    convergence_criterion: f64,
) -> na::Isometry3<f64> {
    if voxel_map.is_empty() {
        return initial_guess.to_owned();
    }

    let mut source = point_cloud.to_owned();
    transform_points(initial_guess, &mut source);

    let mut t_icp = na::Isometry3::<f64>::identity();
    let mut converge_flag = false;
    for i in 0..max_num_iterations {
        let correspondences = point_association(&source, voxel_map, max_distance);
        let (jtj, jtr) = build_linear_system(&correspondences, kernel_scale);
        let dx = match jtj.qr().solve(&(-jtr)) {
            Some(dx) => dx,
            None => {
                println!("i {} cor {} k {}", i, correspondences.len(), kernel_scale);
                println!("jtj {}", jtj);
                println!("jtr {}", jtr);
                panic!()
            }
        };
        let estimation = dx.exp();
        transform_points(&estimation, &mut source);
        t_icp = estimation * t_icp;
        if dx.norm() < convergence_criterion {
            converge_flag = true;
            break;
        }
    }
    if !converge_flag {
        println!("not converge!!!!!!!!!!!");
    }
    t_icp * initial_guess
}
